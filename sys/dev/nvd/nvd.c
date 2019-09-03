/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (C) 2012-2016 Intel Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/bio.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/sysctl.h>
#include <sys/systm.h>
#include <sys/taskqueue.h>

#include <geom/geom.h>
#include <geom/geom_disk.h>

#include <dev/nvme/nvme.h>

#define NVD_STR		"nvd"

struct nvd_disk;
struct nvd_path;

static disk_ioctl_t nvd_ioctl;
static disk_strategy_t nvd_strategy;
static dumper_t nvd_dump;

static void nvd_done(void *arg, const struct nvme_completion *cpl);

static void *nvd_new_path(struct nvme_namespace *ns, void *ctrlr);
static void destroy_geom_disk(struct nvd_disk *ndisk);

static void *nvd_new_controller(struct nvme_controller *ctrlr);
static void nvd_controller_fail(void *ctrlr);

static int nvd_load(void);
static void nvd_unload(void);

MALLOC_DEFINE(M_NVD, "nvd", "nvd(4) allocations");

struct nvme_consumer *consumer_handle;

struct nvd_path {

	struct nvme_namespace	*ns;

	struct nvd_controller	*path_ctrlr;
	TAILQ_ENTRY(nvd_path)	ctrlr_tailq;

	struct nvd_disk		*path_nvd;
	TAILQ_ENTRY(nvd_path)	nvd_tailq;
};

struct nvd_disk {

	struct bio_queue_head		bioq;
	struct task			bioqtask;
	struct mtx			bioqlock;

	volatile struct nvd_path	*currpath;     /* Chosen working path */
	struct disk			*disk;
	struct taskqueue		*tq;

	uint32_t			cur_depth;
	uint32_t			ordered_in_flight;

	TAILQ_ENTRY(nvd_disk)		global_tailq;

	TAILQ_HEAD(, nvd_path)		nvd_livepaths; /* Working paths */
	TAILQ_HEAD(, nvd_path)		nvd_deadpaths; /* Disconnecting paths */

	struct mtx			pathslck;
	struct nvme_ns_gid		dns_gids;
};

struct nvd_controller {

	TAILQ_ENTRY(nvd_controller)	tailq;
	TAILQ_HEAD(, nvd_path)		ctrlr_paths;
};

static TAILQ_HEAD(, nvd_controller)	ctrlr_head;
static TAILQ_HEAD(ndisk_list, nvd_disk)	ndisk_head;
struct sx nvd_glblstslck;
SX_SYSINIT(nvd_glblstslck, &nvd_glblstslck, "NVD Lists Lock");


static SYSCTL_NODE(_hw, OID_AUTO, nvd, CTLFLAG_RD, 0, "nvd driver parameters");
/*
 * The NVMe specification does not define a maximum or optimal delete size, so
 *  technically max delete size is min(full size of the namespace, 2^32 - 1
 *  LBAs).  A single delete for a multi-TB NVMe namespace though may take much
 *  longer to complete than the nvme(4) I/O timeout period.  So choose a sensible
 *  default here that is still suitably large to minimize the number of overall
 *  delete operations.
 */
static uint64_t nvd_delete_max = (1024 * 1024 * 1024);  /* 1GB */
SYSCTL_UQUAD(_hw_nvd, OID_AUTO, delete_max, CTLFLAG_RDTUN, &nvd_delete_max, 0,
	     "nvd maximum BIO_DELETE size in bytes");

static int nvd_modevent(module_t mod, int type, void *arg)
{
	int error = 0;

	switch (type) {
	case MOD_LOAD:
		error = nvd_load();
		break;
	case MOD_UNLOAD:
		nvd_unload();
		break;
	default:
		break;
	}

	return (error);
}

moduledata_t nvd_mod = {
	NVD_STR,
	(modeventhand_t)nvd_modevent,
	0
};

DECLARE_MODULE(nvd, nvd_mod, SI_SUB_DRIVERS, SI_ORDER_ANY);
MODULE_VERSION(nvd, 1);
MODULE_DEPEND(nvd, nvme, 1, 1, 1);

static int
nvd_load()
{
	if (!nvme_use_nvd)
		return 0;

	TAILQ_INIT(&ctrlr_head);
	TAILQ_INIT(&ndisk_head);

	consumer_handle = nvme_register_consumer(nvd_new_path,
	    nvd_new_controller, NULL, nvd_controller_fail);

	return (consumer_handle != NULL ? 0 : -1);
}

static void
delete_path_from_ndisk(struct nvd_disk *ndisk, struct nvd_path *path)
{
	struct nvd_path *tmpath;

	DBGSPEW("Removing path:%p from ndisk:%p\n", path, ndisk);
	mtx_lock_spin(&ndisk->pathslck);

	/* path should be on one of .nvd_livepaths or .nvd_deadpaths */
	TAILQ_FOREACH(tmpath, &ndisk->nvd_livepaths, nvd_tailq) {
		if (tmpath == path) {
			break;
		}
	}
	if (tmpath == path) {
		DBGSPEW("Found path:%p among livepaths\n", path);
		TAILQ_REMOVE(&ndisk->nvd_livepaths, path, nvd_tailq);
	} else {
		TAILQ_FOREACH(tmpath, &ndisk->nvd_deadpaths,
		    nvd_tailq) {
			if (tmpath == path) {
				break;
			}
		}
		if (tmpath == path) {
			DBGSPEW("Found path:%p among deadpaths\n", path);
			TAILQ_REMOVE(&ndisk->nvd_deadpaths, path, nvd_tailq);
		} else {
			panic("path:%p not found in ndisk:%p\n",
			    path, ndisk);
		}
	}

	if (ndisk->currpath == path) {
		tmpath = TAILQ_FIRST(&ndisk->nvd_livepaths);
		DBGSPEW("Replacing currpath:%p with %p\n", path, tmpath);
		atomic_store_rel_ptr((void *)&ndisk->currpath,
		    (uintptr_t)tmpath); /* tmpath can be NULL */
	}

	mtx_unlock_spin(&ndisk->pathslck);
}

static void
nvd_unload()
{
	struct nvd_controller	*ctrlr;
	struct nvd_disk		*ndisk;
	struct nvd_path		*path;

	if (!nvme_use_nvd)
		return;

	sx_xlock(&nvd_glblstslck);

	while (!TAILQ_EMPTY(&ctrlr_head)) {
		ctrlr = TAILQ_FIRST(&ctrlr_head);

		TAILQ_REMOVE(&ctrlr_head, ctrlr, tailq);

		while (!TAILQ_EMPTY(&ctrlr->ctrlr_paths)) {
			path = TAILQ_FIRST(&ctrlr->ctrlr_paths);
			TAILQ_REMOVE(&ctrlr->ctrlr_paths, path, ctrlr_tailq);
			KASSERT((path->path_ctrlr == ctrlr), ("ctrlr:%p "
			    "path:%p path_ctrlr:%p", ctrlr, path,
			    path->path_ctrlr));

			ndisk = path->path_nvd;
			KASSERT((path->path_nvd == ndisk), ("ndisk:%p path:%p "
			    "path_nvd:%p", ndisk, path, path->path_nvd));
			free(path, M_NVD);
		}
		free(ctrlr, M_NVD);
	}

	while (!TAILQ_EMPTY(&ndisk_head)) {
		ndisk = TAILQ_FIRST(&ndisk_head);

		TAILQ_REMOVE(&ndisk_head, ndisk, global_tailq);

		destroy_geom_disk(ndisk);
		free(ndisk, M_NVD);
	}

	sx_xunlock(&nvd_glblstslck);

	nvme_unregister_consumer(consumer_handle);
}

static int
nvd_bio_submit(struct nvd_disk *ndisk, struct bio *bp)
{
	int err;
	struct nvd_path *path, *prevpath, *tmpath;

	bp->bio_driver1 = NULL;

	/* veliath: The increment below won't scale...*/
	atomic_add_int(&ndisk->cur_depth, 1);

	prevpath = NULL;
	err = 0;
	do {
		path = (void *)atomic_load_acq_ptr(
		    (volatile void *)&ndisk->currpath);
		if ((prevpath != path) && (path != NULL)) {
			err = nvme_ns_bio_process(path->ns, bp, nvd_done);
			prevpath = path;
		} else if (path == NULL) {
			DBGSPEW("No paths found, ndisk:%p!\n", ndisk);
			err = ENODEV;
		} else {
			KASSERT((prevpath == path) && (err == ESHUTDOWN),
			    ("prev:%p p:%p e:%d\n", prevpath, path, err));

			DBGSPEW("Seeking new path, n:%p dp:%p!\n", ndisk, path);

			mtx_lock_spin(&ndisk->pathslck);

			/* Confirm path chosen hasn't changed after locking */
			path = (void *)atomic_load_acq_ptr(
			    (volatile void *)&ndisk->currpath);
			if (prevpath == path) {
				/* Remove the path from .nvd_livepaths... */
				tmpath = TAILQ_FIRST(&ndisk->nvd_livepaths);
				/* (it should be the first) */
				KASSERT(tmpath == path, ("path:%p not first "
				    "among living:%p!\n", path, ndisk));
				TAILQ_REMOVE(&ndisk->nvd_livepaths, path,
				    nvd_tailq);
				/* ...and add it to .nvd_deadpaths */
				TAILQ_INSERT_TAIL(&ndisk->nvd_deadpaths, path,
				    nvd_tailq);

				/* Find a new live path */
				path = TAILQ_FIRST(&ndisk->nvd_livepaths);
				atomic_store_rel_ptr((void *)&ndisk->currpath,
				    (uintptr_t)path); /* Store even if NULL */
			}

			mtx_unlock_spin(&ndisk->pathslck);
			DBGSPEW("New path for n:%p is p:%p\n", ndisk, path);
			prevpath = NULL; /* Any path is valid */
		}
		if (err == ESHUTDOWN) {
			DBGSPEW("err == ESHUTDOWN, n:%p b:%p\n", ndisk, bp);
		}
	} while (err == ESHUTDOWN);

	if (err) {
		atomic_add_int(&ndisk->cur_depth, -1);
		if (__predict_false(bp->bio_flags & BIO_ORDERED))
			atomic_add_int(&ndisk->ordered_in_flight, -1);
		bp->bio_error = err;
		bp->bio_flags |= BIO_ERROR;
		bp->bio_resid = bp->bio_bcount;
		biodone(bp);
		return (-1);
	}

	return (0);
}

static void
nvd_strategy(struct bio *bp)
{
	struct nvd_disk *ndisk;

	ndisk = (struct nvd_disk *)bp->bio_disk->d_drv1;

	if (__predict_false(bp->bio_flags & BIO_ORDERED))
		atomic_add_int(&ndisk->ordered_in_flight, 1);

	if (__predict_true(ndisk->ordered_in_flight == 0)) {
		nvd_bio_submit(ndisk, bp);
		return;
	}

	/*
	 * There are ordered bios in flight, so we need to submit
	 *  bios through the task queue to enforce ordering.
	 */
	mtx_lock(&ndisk->bioqlock);
	bioq_insert_tail(&ndisk->bioq, bp);
	mtx_unlock(&ndisk->bioqlock);
	taskqueue_enqueue(ndisk->tq, &ndisk->bioqtask);
}

static int
nvd_ioctl(struct disk *disk, u_long cmd, void *data, int fflag,
    struct thread *td)
{
	int ret = 0;

	switch (cmd) {
	default:
		ret = EIO;
	}

	return (ret);
}

static int
nvd_dump(void *arg, void *virt, vm_offset_t phys, off_t offset, size_t len)
{
	struct nvd_disk *ndisk;
	struct nvd_path *path;
	struct disk *dp;
	int err;

	dp = arg;
	ndisk = dp->d_drv1;

	path = (void *)atomic_load_acq_ptr((volatile void *)&ndisk->currpath);
	if (path != NULL) {
		err = nvme_ns_dump(path->ns, virt, offset, len);
	} else {
		err = ENODEV;
	}
	return (err);
}

static void
nvd_done(void *arg, const struct nvme_completion *cpl)
{
	struct bio *bp;
	struct nvd_disk *ndisk;

	bp = (struct bio *)arg;

	ndisk = bp->bio_disk->d_drv1;

	atomic_add_int(&ndisk->cur_depth, -1);
	if (__predict_false(bp->bio_flags & BIO_ORDERED))
		atomic_add_int(&ndisk->ordered_in_flight, -1);

	biodone(bp);
}

static void
nvd_bioq_process(void *arg, int pending)
{
	struct nvd_disk *ndisk = arg;
	struct bio *bp;

	for (;;) {
		mtx_lock(&ndisk->bioqlock);
		bp = bioq_takefirst(&ndisk->bioq);
		mtx_unlock(&ndisk->bioqlock);
		if (bp == NULL)
			break;

		if (nvd_bio_submit(ndisk, bp) != 0) {
			continue;
		}

#ifdef BIO_ORDERED
		/*
		 * BIO_ORDERED flag dictates that the bio with BIO_ORDERED
		 *  flag set must be completed before proceeding with
		 *  additional bios.
		 */
		if (bp->bio_flags & BIO_ORDERED) {
			while (ndisk->cur_depth > 0) {
				pause("nvd flush", 1);
			}
		}
#endif
	}
}

static void *
nvd_new_controller(struct nvme_controller *ctrlr)
{
	struct nvd_controller	*nvd_ctrlr;

	nvd_ctrlr = malloc(sizeof(struct nvd_controller), M_NVD,
	    M_ZERO | M_WAITOK);

	TAILQ_INIT(&nvd_ctrlr->ctrlr_paths);

	sx_xlock(&nvd_glblstslck);
	TAILQ_INSERT_TAIL(&ctrlr_head, nvd_ctrlr, tailq);
	sx_xunlock(&nvd_glblstslck);

	return (nvd_ctrlr);
}

static struct nvd_disk *
find_ndisk_by_gid(struct nvme_ns_gid *gids)
{
	struct nvd_disk *ndisk, *retndisk;

	retndisk = NULL;

	sx_assert(&nvd_glblstslck, SA_XLOCKED);

	TAILQ_FOREACH(ndisk, &ndisk_head, global_tailq) {
		if (memcmp(&ndisk->dns_gids, gids, sizeof(*gids)) == 0) {
			KASSERT(retndisk == NULL,
			    ("Another ret:%p already found! n:%p\n", retndisk,
			        ndisk));
			DBGSPEW("Matched "NVD_STR"%u!\n", ndisk->disk->d_unit);
			retndisk = ndisk;
		}
	}

	return retndisk;
}


static void
nvd_new_disk(struct nvd_path *path)
{
	uint8_t			descr[NVME_MODEL_NUMBER_LENGTH+1];
	struct nvd_disk		*ndisk;
	struct disk		*disk;

	sx_assert(&nvd_glblstslck, SA_XLOCKED);

	ndisk = malloc(sizeof(struct nvd_disk), M_NVD, M_ZERO | M_WAITOK);
	mtx_init(&ndisk->pathslck, "NVD path lock", NULL, MTX_SPIN);

	disk = disk_alloc();
	disk->d_strategy = nvd_strategy;
	disk->d_ioctl = nvd_ioctl;
	disk->d_dump = nvd_dump;
	disk->d_name = NVD_STR;
	disk->d_drv1 = ndisk;

	disk->d_maxsize = nvme_ns_get_max_io_xfer_size(path->ns);
	disk->d_sectorsize = nvme_ns_get_sector_size(path->ns);
	disk->d_mediasize = (off_t)nvme_ns_get_size(path->ns);
	disk->d_delmaxsize = (off_t)nvme_ns_get_size(path->ns);
	if (disk->d_delmaxsize > nvd_delete_max)
		disk->d_delmaxsize = nvd_delete_max;
	disk->d_stripesize = nvme_ns_get_stripesize(path->ns);

	if (TAILQ_EMPTY(&ndisk_head))
		disk->d_unit = 0;
	else
		disk->d_unit =
		    TAILQ_LAST(&ndisk_head, ndisk_list)->disk->d_unit + 1;

	disk->d_flags = DISKFLAG_DIRECT_COMPLETION;

	if (nvme_ns_get_flags(path->ns) & NVME_NS_DEALLOCATE_SUPPORTED)
		disk->d_flags |= DISKFLAG_CANDELETE;

	if (nvme_ns_get_flags(path->ns) & NVME_NS_FLUSH_SUPPORTED)
		disk->d_flags |= DISKFLAG_CANFLUSHCACHE;

/* ifdef used here to ease porting to stable branches at a later point. */
#ifdef DISKFLAG_UNMAPPED_BIO
	disk->d_flags |= DISKFLAG_UNMAPPED_BIO;
#endif

	/*
	 * d_ident and d_descr are both far bigger than the length of either
	 *  the serial or model number strings.
	 */
	nvme_strvis(disk->d_ident, nvme_ns_get_serial_number(path->ns),
	    sizeof(disk->d_ident), NVME_SERIAL_NUMBER_LENGTH);
	nvme_strvis(descr, nvme_ns_get_model_number(path->ns), sizeof(descr),
	    NVME_MODEL_NUMBER_LENGTH);
	strlcpy(disk->d_descr, descr, sizeof(descr));

	disk->d_rotation_rate = DISK_RR_NON_ROTATING;

	ndisk->disk = disk;
	ndisk->cur_depth = 0;
	ndisk->ordered_in_flight = 0;

	mtx_init(&ndisk->bioqlock, "NVD bioq lock", NULL, MTX_DEF);
	bioq_init(&ndisk->bioq);

	TASK_INIT(&ndisk->bioqtask, 0, nvd_bioq_process, ndisk);
	ndisk->tq = taskqueue_create("nvd_taskq", M_WAITOK,
	    taskqueue_thread_enqueue, &ndisk->tq);
	taskqueue_start_threads(&ndisk->tq, 1, PI_DISK, "nvd taskq");

	TAILQ_INIT(&ndisk->nvd_livepaths);
	TAILQ_INIT(&ndisk->nvd_deadpaths);

	path->path_nvd = ndisk;

	/* No need to grab .pathslck here: ndisk not globally visible yet */
	TAILQ_INSERT_TAIL(&ndisk->nvd_livepaths, path, nvd_tailq);

	ndisk->dns_gids = path->ns->nns_gids;
	ndisk->currpath = path;

	TAILQ_INSERT_TAIL(&path->path_ctrlr->ctrlr_paths, path, ctrlr_tailq);
	TAILQ_INSERT_TAIL(&ndisk_head, ndisk, global_tailq);


	disk_create(disk, DISK_VERSION);

	printf(NVD_STR"%u: <%s> NVMe namespace\n", disk->d_unit, descr);
	printf(NVD_STR"%u: %juMB (%ju %u byte sectors)\n", disk->d_unit,
		(uintmax_t)disk->d_mediasize / (1024*1024),
		(uintmax_t)disk->d_mediasize / disk->d_sectorsize,
		disk->d_sectorsize);
	DBGSPEW("ndisk:%p created with path:%p\n", ndisk, path);
}


static void *
nvd_new_path(struct nvme_namespace *ns, void *ctrlr_arg)
{
	struct nvd_controller	*ctrlr;
	struct nvd_disk		*ndisk;
	struct nvd_path		*path;

	ctrlr = ctrlr_arg;

	/* Allocate a path */
	path = malloc(sizeof(struct nvd_path), M_NVD, M_ZERO | M_WAITOK);
	path->ns = ns;
	path->path_ctrlr = ctrlr;

	ndisk = NULL;

	sx_xlock(&nvd_glblstslck);
	if (ns->nvmes_nsd.nmic & NVME_NS_DATA_NMIC_MAY_BE_SHARED_MASK) {
		ndisk = find_ndisk_by_gid(&ns->nns_gids);
	}

	if (ndisk == NULL) {
		/* Not a shared NS or no nvd associated with it */
		nvd_new_disk(path);
	} else {
		path->path_nvd = ndisk;

		mtx_lock_spin(&ndisk->pathslck);
		TAILQ_INSERT_TAIL(&ndisk->nvd_livepaths, path, nvd_tailq);
		/*
		 * An ndisk can remain locatable while it has (only) dead
		 * paths hanging off it.  In such a scenario .currpath will
		 * be NULL and the ndisk can be resuscitated by this new path.
		 */
		if ((void *)atomic_load_acq_ptr((volatile void *)&ndisk->
		    currpath) == NULL) {
			atomic_store_rel_ptr((void *)&ndisk->currpath,
			    (uintptr_t)path);
		}
		mtx_unlock_spin(&ndisk->pathslck);
		DBGSPEW("Added path:%p to ndisk:%p\n", path, ndisk);

		sx_assert(&nvd_glblstslck, SA_XLOCKED);
		TAILQ_INSERT_TAIL(&ctrlr->ctrlr_paths, path, ctrlr_tailq);
	}
	sx_xunlock(&nvd_glblstslck);

	return NULL;
}


static void
destroy_geom_disk(struct nvd_disk *ndisk)
{
	struct bio	*bp;
	struct disk	*disk;
	uint32_t	unit;
	int		cnt = 0;

	disk = ndisk->disk;
	unit = disk->d_unit;
	taskqueue_free(ndisk->tq);

	disk_destroy(ndisk->disk);

	mtx_lock(&ndisk->bioqlock);
	for (;;) {
		bp = bioq_takefirst(&ndisk->bioq);
		if (bp == NULL)
			break;
		bp->bio_error = EIO;
		bp->bio_flags |= BIO_ERROR;
		bp->bio_resid = bp->bio_bcount;
		cnt++;
		biodone(bp);
	}

	printf(NVD_STR"%u: lost device - %d outstanding\n", unit, cnt);
	printf(NVD_STR"%u: removing device entry\n", unit);

	mtx_unlock(&ndisk->bioqlock);

	mtx_destroy(&ndisk->bioqlock);
}

static void
nvd_controller_fail(void *ctrlr_arg)
{
	struct nvd_controller	*ctrlr = ctrlr_arg;
	struct nvd_disk		*ndisk;
	struct nvd_path		*path;
	boolean_t		delete;

	sx_xlock(&nvd_glblstslck);

	while (!TAILQ_EMPTY(&ctrlr->ctrlr_paths)) {
		path = TAILQ_FIRST(&ctrlr->ctrlr_paths);
		TAILQ_REMOVE(&ctrlr->ctrlr_paths, path, ctrlr_tailq);
		KASSERT((path->path_ctrlr == ctrlr), ("ctrlr:%p path:%p "
		    "path_ctrlr:%p", ctrlr, path, path->path_ctrlr));

		ndisk = path->path_nvd;
		KASSERT((path->path_nvd == ndisk), ("ndisk:%p path:%p "
		    "path_nvd:%p", ndisk, path, path->path_nvd));

		delete_path_from_ndisk(ndisk, path);

		delete = false;
		mtx_lock_spin(&ndisk->pathslck);
		/* Grab the spinlock so the two paths Qs aren't in flux */
		if (TAILQ_EMPTY(&ndisk->nvd_livepaths) &&
		    TAILQ_EMPTY(&ndisk->nvd_deadpaths)) {
			delete = true;
		}
		mtx_unlock_spin(&ndisk->pathslck);

		if (delete) {
			TAILQ_REMOVE(&ndisk_head, ndisk, global_tailq);
			DBGSPEW("No paths to ndisk:%p, delete\n", ndisk);
			destroy_geom_disk(ndisk);
			free(ndisk, M_NVD);
		}

		free(path, M_NVD);
	}

	TAILQ_REMOVE(&ctrlr_head, ctrlr, tailq);

	sx_xunlock(&nvd_glblstslck);

	free(ctrlr, M_NVD);
}

