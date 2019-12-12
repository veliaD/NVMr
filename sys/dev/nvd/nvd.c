/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (C) 2012-2016 Intel Corporation
 * All rights reserved.
 * Copyright (C) 2018 Alexander Motin <mav@FreeBSD.org>
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
#include <sys/queue.h>
#include <sys/sysctl.h>
#include <sys/systm.h>
#include <sys/taskqueue.h>
#include <machine/atomic.h>

#include <geom/geom.h>
#include <geom/geom_disk.h>

#include <sys/counter.h>

#include <sys/epoch.h>

#include <dev/nvme/nvme.h>

#define NVD_STR		"nvd"

struct nvd_disk;
struct nvd_path;

static disk_ioctl_t nvd_ioctl;
static disk_strategy_t nvd_strategy;
static dumper_t nvd_dump;
static disk_getattr_t nvd_getattr;

static void nvd_done(void *arg1, void *arg2, const struct nvme_completion *cpl);
static void nvd_gone(struct nvd_disk *ndisk);

static void *nvd_new_path(struct nvme_namespace *ns, void *ctrlr);

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

	/*
	 * GEOM KPIs allow us to delay deallocating the ndisk until all IOs
	 * issued to it have drained.  With native multi-pathing support in
	 * nvd this is no longer sufficient.
	 * IO counts through individual paths have to be tracked so that
	 * path deletions can be delayed until IO through them have been
	 * drained
	 */
	counter_u64_t		nvdp_usecount;
	boolean_t		nvdp_failing;
};

struct nvd_disk {
	volatile struct nvd_path	*currpath;     /* Chosen working path */

	struct bio_queue_head		bioq;
	struct task			bioqtask;
	struct mtx			bioqlock;

	struct disk			*disk;
	struct taskqueue		*tq;

	uint32_t			cur_depth;
#define	NVD_ODEPTH	(1 << 30)
	uint32_t			ordered_in_flight;

	u_int				unit;
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

static struct mtx			nvd_lock;
static TAILQ_HEAD(, nvd_controller)	ctrlr_head;
static TAILQ_HEAD(ndisk_list, nvd_disk)	ndisk_head;
static uint32_t				ndisk_count;


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

	mtx_init(&nvd_lock, "nvd_lock", NULL, MTX_DEF);
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
	uint64_t iocount;

	mtx_assert(&nvd_lock, MA_NOTOWNED);

	DBGSPEW("Removing path:%p from ndisk:%p\n", path, ndisk);

	mtx_lock_spin(&ndisk->pathslck);

	path->nvdp_failing = true;

	/* path should be on .nvd_livepaths or .nvd_deadpaths */
	TAILQ_FOREACH(tmpath, &ndisk->nvd_livepaths, nvd_tailq) {
		if (tmpath == path) {
			break;
		}
	}
	if (tmpath == path) {
		DBGSPEW("Found path:%p among livepaths\n", path);
		TAILQ_REMOVE(&ndisk->nvd_livepaths, path, nvd_tailq);
	} else {
		TAILQ_FOREACH(tmpath, &ndisk->nvd_deadpaths, nvd_tailq) {
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

	/*
	 * Wait for any racing IO initiation in nvd_bio_submit() to see the
	 * change to .nvdp_failing above or have bumped up .nvdp_usecount
	 */
	epoch_wait(global_epoch);

	/*
	 * Now that the path cannot be reached via .currpath wait for
	 * any references to it to drop off.  When delete_path_from_ndisk()
	 * returns the expectation is that it will be safe to free() the path
	 */
	while ((iocount = counter_u64_fetch(path->nvdp_usecount)) != 0) {
		DBGSPEW("Waiting for %lu IOs from path:%p to drain\n", iocount,
		    path);
		tsleep(path, 0, "nvdpath", hz);
	}
}


void nvd_controller_delete(struct nvd_controller *ctrlr);
/*
 * Deletion of a controller is what triggers the deletion of a path from an
 * nvd disk
 */
void
nvd_controller_delete(struct nvd_controller *ctrlr)
{
	struct nvd_disk		*ndisk;
	struct nvd_path		*path;
	boolean_t		delete;

	mtx_assert(&nvd_lock, MA_NOTOWNED);

	mtx_lock(&nvd_lock);
	while ((path = TAILQ_FIRST(&ctrlr->ctrlr_paths)) != NULL) {
		TAILQ_REMOVE(&ctrlr->ctrlr_paths, path, ctrlr_tailq);
		KASSERT((path->path_ctrlr == ctrlr), ("ctrlr:%p "
		    "path:%p path_ctrlr:%p", ctrlr, path,
		    path->path_ctrlr));

		ndisk = path->path_nvd;
		KASSERT((path->path_nvd == ndisk), ("ndisk:%p path:%p "
		    "path_nvd:%p", ndisk, path, path->path_nvd));
		mtx_unlock(&nvd_lock);

		delete_path_from_ndisk(ndisk, path);

		mtx_lock(&nvd_lock);
		/**********
		 Now figure out if the ndisk has to be removed
		 **********/
		delete = false;
		mtx_lock_spin(&ndisk->pathslck);
		/* Grab the spinlock so the two paths Qs aren't in flux */
		if (TAILQ_EMPTY(&ndisk->nvd_livepaths) &&
		    TAILQ_EMPTY(&ndisk->nvd_deadpaths)) {
			delete = true;
		}
		mtx_unlock_spin(&ndisk->pathslck);

		if (delete) {
			DBGSPEW("No paths to ndisk:%p, delete\n", ndisk);
			nvd_gone(ndisk);
			/* No need to wait for the callback from nvd_gone() */
		}

		free(path, M_NVD);
	}

	free(ctrlr, M_NVD);
	mtx_unlock(&nvd_lock);
}


static void
nvd_unload()
{
	struct nvd_controller	*ctrlr;

	if (!nvme_use_nvd)
		return;

	mtx_lock(&nvd_lock);
	while ((ctrlr = TAILQ_FIRST(&ctrlr_head)) != NULL) {
		TAILQ_REMOVE(&ctrlr_head, ctrlr, tailq);
		mtx_unlock(&nvd_lock);

		nvd_controller_delete(ctrlr);

		mtx_lock(&nvd_lock);
	}

	/**********
	 Wait for the nvd_gonecb(),s to empty the list
	 **********/
	while (ndisk_count != 0) {
		msleep(&ndisk_count, &nvd_lock, 0, "nvd_unload",0);
	}

	mtx_unlock(&nvd_lock);
	/**********
	 veliaD: Should nvme_unregister_consumer() be moved to the beginning
	 of this routine?  We don't want registrations to come in between
	 the time we unlock above and unregister below.
	 **********/
	nvme_unregister_consumer(consumer_handle);

	mtx_destroy(&nvd_lock);
}


void nvd_putpath(struct nvd_path *path);
void
nvd_putpath(struct nvd_path *path)
{
	boolean_t failing;

	failing = path->nvdp_failing;

	/* The decrement below must happen after the dereference above */
	atomic_thread_fence_acq();
	counter_u64_add(path->nvdp_usecount, -1);

	if (failing) {
		wakeup(path);
	}
}


struct nvd_path *nvd_getcurrpath(struct nvd_disk *ndisk);
struct nvd_path *
nvd_getcurrpath(struct nvd_disk *ndisk)
{
	struct nvd_path *path;

	path = NULL;

	epoch_enter(global_epoch);/* Delay delete_path_from_ndisk()...*/

	path = (void *)atomic_load_acq_ptr((volatile void *)&ndisk->currpath);
	if ((path != NULL) && !path->nvdp_failing) {
		counter_u64_add(path->nvdp_usecount, 1);
	} else {
		path = NULL;
	}

	epoch_exit(global_epoch); /*...until this point*/

	return path;
}

static void
nvd_bio_submit(struct nvd_disk *ndisk, struct bio *bp, boolean_t bump_cur_depth)
{
	int err;
	struct nvd_path *path, *prevpath, *tmpath;
	struct nvme_namespace *ns;

	bp->bio_driver1 = NULL;

	/* bump_cur_depth is false if the bio is being retried */
	if (bump_cur_depth) {
		/* veliath: The increments below won't scale...*/
		if (__predict_false(bp->bio_flags & BIO_ORDERED))
			atomic_add_int(&ndisk->cur_depth, NVD_ODEPTH);
		else
			atomic_add_int(&ndisk->cur_depth, 1);
	}

	prevpath = NULL;
	err = 0;
	/*
	 * The chosen path is read atomically from the .currpath field
	 * If .currpath is NULL it means no more paths exist for this drive
	 *     and the IO can be permanently failed with ENOLINK
	 * If .currpath is not NULL and this is our first time seeing this path
	 *     for this specific bio/IO issue the IO
	 * If we've seen this path for this bio/IO then we were returned an
	 *     ESHUTDOWN in the previous iteration through the loop and we
	 *     have to replace the path with the next one known to be good.
	 */
	do {
		path = nvd_getcurrpath(ndisk);
		if ((path != prevpath) && (path != NULL)) {
			ns = path->ns;
			err = nvme_ns_bio_process(ns, bp, nvd_done);
			if (err != 0) {
				nvd_putpath(path);
			}
			prevpath = path;
		} else if (path == NULL) {
			DBGSPEW("No paths found, ndisk:%p!\n", ndisk);
			err = ENOLINK;
		} else {
			nvd_putpath(path); /* path unused, drop .nvdp_usecount*/
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

	/* On error decrement counts, irrespective of bump_cur_depth argument */
	if (err) {
		if (__predict_false(bp->bio_flags & BIO_ORDERED)) {
			atomic_add_int(&ndisk->cur_depth, -NVD_ODEPTH);
			atomic_add_int(&ndisk->ordered_in_flight, -1);
			wakeup(&ndisk->cur_depth);
		} else {
			if (atomic_fetchadd_int(&ndisk->cur_depth, -1) == 1 &&
			    __predict_false(ndisk->ordered_in_flight != 0))
				wakeup(&ndisk->cur_depth);
		}
		bp->bio_error = err;
		bp->bio_flags |= BIO_ERROR;
		bp->bio_resid = bp->bio_bcount;
		biodone(bp);
	}
}

static void
nvd_strategy(struct bio *bp)
{
	struct nvd_disk *ndisk = (struct nvd_disk *)bp->bio_disk->d_drv1;

	/*
	 * bio with BIO_ORDERED flag must be executed after all previous
	 * bios in the queue, and before any successive bios.
	 */
	if (__predict_false(bp->bio_flags & BIO_ORDERED)) {
		if (atomic_fetchadd_int(&ndisk->ordered_in_flight, 1) == 0 &&
		    ndisk->cur_depth == 0 && bioq_first(&ndisk->bioq) == NULL) {
			nvd_bio_submit(ndisk, bp, true);
			return;
		}
	} else if (__predict_true(ndisk->ordered_in_flight == 0)) {
		nvd_bio_submit(ndisk, bp, true);
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

static void
nvd_gone(struct nvd_disk *ndisk)
{
	struct bio	*bp;

	mtx_assert(&nvd_lock, MA_OWNED);

	printf(NVD_STR"%u: detached\n", ndisk->unit);
	/*
	 * Make sure the nvd drive can no longer be found by new paths before
	 * we initiate removal with GEOM
	 */
	TAILQ_REMOVE(&ndisk_head, ndisk, global_tailq);

	mtx_lock(&ndisk->bioqlock);
	disk_gone(ndisk->disk);
	while ((bp = bioq_takefirst(&ndisk->bioq)) != NULL) {
		if (__predict_false(bp->bio_flags & BIO_ORDERED))
			atomic_add_int(&ndisk->ordered_in_flight, -1);
		bp->bio_error = ENXIO;
		bp->bio_flags |= BIO_ERROR;
		bp->bio_resid = bp->bio_bcount;
		biodone(bp);
	}
	mtx_unlock(&ndisk->bioqlock);
}

static void
nvd_gonecb(struct disk *dp)
{
	struct nvd_disk *ndisk = (struct nvd_disk *)dp->d_drv1;
	u_int unit;

	unit = ndisk->unit;
	disk_destroy(ndisk->disk);

	taskqueue_free(ndisk->tq);
	mtx_destroy(&ndisk->bioqlock);
	free(ndisk, M_NVD);
	printf(NVD_STR"%u: destroyed\n", unit);

	mtx_lock(&nvd_lock);
	ndisk_count--;
	if (ndisk_count == 0) {
		wakeup(&ndisk_count);
	}
	mtx_unlock(&nvd_lock);
}

static int
nvd_ioctl(struct disk *dp, u_long cmd, void *data, int fflag,
    struct thread *td)
{
	struct nvd_disk		*ndisk = dp->d_drv1;
	struct nvme_namespace	*ns;
	struct nvd_path		*path;
	int			err;

	path = nvd_getcurrpath(ndisk);
	if (path != NULL) {
		ns = path->ns;
		err = nvme_ns_ioctl_process(ns, cmd, data, fflag, td);
		nvd_putpath(path);
	} else {
		err = ENOLINK;
	}

	return err;
}

static int
nvd_dump(void *arg, void *virt, vm_offset_t phys, off_t offset, size_t len)
{
	struct nvme_namespace *ns;
	struct nvd_disk *ndisk;
	struct nvd_path *path;
	struct disk *dp;
	int err;

	dp = arg;
	ndisk = dp->d_drv1;

	path = nvd_getcurrpath(ndisk);
	if (path != NULL) {
		ns = path->ns;
		err = nvme_ns_dump(ns, virt, offset, len);
		nvd_putpath(path);
	} else {
		err = ENOLINK;
	}

	return (err);
}

static int
nvd_getattr(struct bio *bp)
{
	struct nvd_disk *ndisk = (struct nvd_disk *)bp->bio_disk->d_drv1;
	const struct nvme_namespace_data *nsdata;
	u_int i;

	struct nvme_namespace	*ns;
	struct nvd_path		*path;

	if (!strcmp("GEOM::lunid", bp->bio_attribute)) {
		path = nvd_getcurrpath(ndisk);
		if (path == NULL) {
			return (-1);
		}
		ns = path->ns;
		nsdata = nvme_ns_get_data(ns);
		nvd_putpath(path);

		/* Try to return NGUID as lunid. */
		for (i = 0; i < sizeof(nsdata->nguid); i++) {
			if (nsdata->nguid[i] != 0)
				break;
		}
		if (i < sizeof(nsdata->nguid)) {
			if (bp->bio_length < sizeof(nsdata->nguid) * 2 + 1)
				return (EFAULT);
			for (i = 0; i < sizeof(nsdata->nguid); i++) {
				sprintf(&bp->bio_data[i * 2], "%02x",
				    nsdata->nguid[i]);
			}
			bp->bio_completed = bp->bio_length;
			return (0);
		}

		/* Try to return EUI64 as lunid. */
		for (i = 0; i < sizeof(nsdata->eui64); i++) {
			if (nsdata->eui64[i] != 0)
				break;
		}
		if (i < sizeof(nsdata->eui64)) {
			if (bp->bio_length < sizeof(nsdata->eui64) * 2 + 1)
				return (EFAULT);
			for (i = 0; i < sizeof(nsdata->eui64); i++) {
				sprintf(&bp->bio_data[i * 2], "%02x",
				    nsdata->eui64[i]);
			}
			bp->bio_completed = bp->bio_length;
			return (0);
		}
	}
	return (-1);
}

static void nvd_done(void *arg1, void *arg2, const struct nvme_completion *cpl);
static void
nvd_done(void *arg1, void *arg2, const struct nvme_completion *cpl)
{
	struct bio *bp = (struct bio *)arg1;
	struct nvme_namespace *ns = (struct nvme_namespace *)arg2;
	struct nvd_disk *ndisk = bp->bio_disk->d_drv1;
	struct nvd_path *path;

	path = (struct nvd_path *)(ns->nvmens_conscookie);
	nvd_putpath(path);

	if ((bp->bio_flags & BIO_ERROR) && (bp->bio_error == ESHUTDOWN)) {
		/* Retry IOs that failed because a path went away */
		bp->bio_flags &= ~(BIO_DONE | BIO_ERROR);
		bp->bio_error = 0;
		bp->bio_completed = 0;
		bp->bio_children = 0;
		bp->bio_inbed = 0;
		bp->bio_error = 0;

		DBGSPEW("Retrying n:%p b:%p\n", ndisk, bp);
		nvd_bio_submit(ndisk, bp, false);
		DBGSPEW("Retry queued n:%p b:%p\n", ndisk, bp);
		goto out;
	}

	if (__predict_false(bp->bio_flags & BIO_ORDERED)) {
		atomic_add_int(&ndisk->cur_depth, -NVD_ODEPTH);
		atomic_add_int(&ndisk->ordered_in_flight, -1);
		wakeup(&ndisk->cur_depth);
	} else {
		if (atomic_fetchadd_int(&ndisk->cur_depth, -1) == 1 &&
		    __predict_false(ndisk->ordered_in_flight != 0))
			wakeup(&ndisk->cur_depth);
	}

	biodone(bp);

out:
	return;
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

		if (__predict_false(bp->bio_flags & BIO_ORDERED)) {
			/*
			 * bio with BIO_ORDERED flag set must be executed
			 * after all previous bios.
			 */
			while (ndisk->cur_depth > 0)
				tsleep(&ndisk->cur_depth, 0, "nvdorb", 1);
		} else {
			/*
			 * bio with BIO_ORDERED flag set must be completed
			 * before proceeding with additional bios.
			 */
			while (ndisk->cur_depth >= NVD_ODEPTH)
				tsleep(&ndisk->cur_depth, 0, "nvdora", 1);
		}

		nvd_bio_submit(ndisk, bp, true);
	}
}

static void *
nvd_new_controller(struct nvme_controller *ctrlr)
{
	struct nvd_controller	*nvd_ctrlr;

	nvd_ctrlr = malloc(sizeof(struct nvd_controller), M_NVD,
	    M_ZERO | M_WAITOK);

	TAILQ_INIT(&nvd_ctrlr->ctrlr_paths);

	mtx_lock(&nvd_lock);
	TAILQ_INSERT_TAIL(&ctrlr_head, nvd_ctrlr, tailq);
	mtx_unlock(&nvd_lock);

	return (nvd_ctrlr);
}

static struct nvd_disk *
find_ndisk_by_gid(struct nvme_ns_gid *gids)
{
	struct nvd_disk *ndisk, *retndisk;

	retndisk = NULL;

	mtx_assert(&nvd_lock, MA_OWNED);

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


void nvd_new_disk(struct nvd_disk *ndisk, struct nvd_path *path);
void
nvd_new_disk(struct nvd_disk *ndisk, struct nvd_path *path)
{
	uint8_t			descr[NVME_MODEL_NUMBER_LENGTH+1];
	struct nvd_disk		*tnd;
	struct disk		*disk;
	int unit;

	mtx_assert(&nvd_lock, MA_OWNED);

	ndisk->cur_depth = 0;
	ndisk->ordered_in_flight = 0;
	mtx_init(&ndisk->bioqlock, "nvd bioq lock", NULL, MTX_DEF);
	bioq_init(&ndisk->bioq);
	TASK_INIT(&ndisk->bioqtask, 0, nvd_bioq_process, ndisk);

	unit = 0;
	TAILQ_FOREACH(tnd, &ndisk_head, global_tailq) {
		if (tnd->unit > unit)
			break;
		unit = tnd->unit + 1;
	}
	ndisk->unit = unit;
	ndisk_count++;
	if (tnd != NULL)
		TAILQ_INSERT_BEFORE(tnd, ndisk, global_tailq);
	else
		TAILQ_INSERT_TAIL(&ndisk_head, ndisk, global_tailq);

	disk = ndisk->disk;
	disk->d_strategy = nvd_strategy;
	disk->d_ioctl = nvd_ioctl;
	disk->d_dump = nvd_dump;
	disk->d_getattr = nvd_getattr;
	disk->d_gone = nvd_gonecb;
	disk->d_name = NVD_STR;
	disk->d_unit = ndisk->unit;
	disk->d_drv1 = ndisk;

	disk->d_sectorsize = nvme_ns_get_sector_size(path->ns);
	disk->d_mediasize = (off_t)nvme_ns_get_size(path->ns);
	disk->d_maxsize = nvme_ns_get_max_io_xfer_size(path->ns);
	disk->d_delmaxsize = (off_t)nvme_ns_get_size(path->ns);
	if (disk->d_delmaxsize > nvd_delete_max)
		disk->d_delmaxsize = nvd_delete_max;
	disk->d_stripesize = nvme_ns_get_stripesize(path->ns);
	disk->d_flags = DISKFLAG_UNMAPPED_BIO | DISKFLAG_DIRECT_COMPLETION;
	if (nvme_ns_get_flags(path->ns) & NVME_NS_DEALLOCATE_SUPPORTED)
		disk->d_flags |= DISKFLAG_CANDELETE;
	if (nvme_ns_get_flags(path->ns) & NVME_NS_FLUSH_SUPPORTED)
		disk->d_flags |= DISKFLAG_CANFLUSHCACHE;

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

	mtx_init(&ndisk->pathslck, "NVD path lock", NULL, MTX_SPIN);

	TAILQ_INIT(&ndisk->nvd_livepaths);
	TAILQ_INIT(&ndisk->nvd_deadpaths);

	path->path_nvd = ndisk;

	/* No need to grab .pathslck here: ndisk not globally visible yet */
	TAILQ_INSERT_TAIL(&ndisk->nvd_livepaths, path, nvd_tailq);

	ndisk->dns_gids = path->ns->nns_gids;
	ndisk->currpath = path;

	TAILQ_INSERT_TAIL(&path->path_ctrlr->ctrlr_paths, path, ctrlr_tailq);
}


struct nvd_disk * ndisk_lookup(struct nvme_namespace *ns);
struct nvd_disk *
ndisk_lookup(struct nvme_namespace *ns)
{
	struct nvd_disk		*ndisk;
	
	mtx_assert(&nvd_lock, MA_OWNED);

	if (ns->data.nmic & NVME_NS_DATA_NMIC_MAY_BE_SHARED_MASK) {
		ndisk = find_ndisk_by_gid(&ns->nns_gids);
	} else {
		ndisk = NULL;
	}

	return ndisk;
}

void ndisk_add_path(struct nvd_disk *ndisk, struct nvd_path *path);
void
ndisk_add_path(struct nvd_disk *ndisk, struct nvd_path *path)
{
	struct nvd_controller	*ctrlr;

	mtx_assert(&nvd_lock, MA_OWNED);

	ctrlr = path->path_ctrlr;
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

	TAILQ_INSERT_TAIL(&ctrlr->ctrlr_paths, path, ctrlr_tailq);
}


static void *
nvd_new_path(struct nvme_namespace *ns, void *ctrlr_arg)
{
	struct nvd_controller	*ctrlr;
	struct nvd_disk		*ndisk, *ndisktmp;
	struct nvd_path		*path;
	struct disk		*disk;

	ctrlr = ctrlr_arg;

	/* Allocate a path */
	path = malloc(sizeof(struct nvd_path), M_NVD, M_ZERO | M_WAITOK);
	path->nvdp_usecount = counter_u64_alloc(M_WAITOK);
	path->ns = ns;
	ns->nvmens_conscookie = (void *)path;
	path->path_ctrlr = ctrlr;
	path->nvdp_failing = false;
	counter_u64_zero(path->nvdp_usecount);

	ndisk = NULL;

	mtx_lock(&nvd_lock);
	ndisk = ndisk_lookup(ns);
	if (ndisk != NULL) {
		ndisk_add_path(ndisk, path);
		mtx_unlock(&nvd_lock);

		goto out;
	} else {
		mtx_unlock(&nvd_lock);
	}

	/* Perform allocations before re-acquiring the nvd_lock */
	ndisk = malloc(sizeof(struct nvd_disk), M_NVD, M_ZERO|M_WAITOK);
	ndisk->disk = disk = disk_alloc();

	mtx_lock(&nvd_lock);
	ndisktmp = ndisk_lookup(ns);
	if (ndisktmp) {
		/*
		 * Between the time we dropped the lock to allocate ndisk
		 * another path to the same NS allocated an ndisk
		 */
		ndisk_add_path(ndisktmp, path);
		mtx_unlock(&nvd_lock);

		free(ndisk, M_NVD);
		disk_destroy(disk);
	} else {
		nvd_new_disk(ndisk, path);
		mtx_unlock(&nvd_lock);

		ndisk->tq = taskqueue_create("nvd_taskq", M_WAITOK,
		    taskqueue_thread_enqueue, &ndisk->tq);
		taskqueue_start_threads(&ndisk->tq, 1, PI_DISK, "nvd taskq");

		disk_create(disk, DISK_VERSION);

		printf(NVD_STR"%u: <%s> NVMe namespace\n", disk->d_unit,
		    disk->d_descr);
		printf(NVD_STR"%u: %juMB (%ju %u byte sectors)\n", disk->d_unit,
			(uintmax_t)disk->d_mediasize / (1024*1024),
			(uintmax_t)disk->d_mediasize / disk->d_sectorsize,
			disk->d_sectorsize);
		DBGSPEW("ndisk:%p created with path:%p\n", ndisk, path);
	}

out:
	return NULL;
}


static void
nvd_controller_fail(void *ctrlr_arg)
{
	struct nvd_controller	*ctrlr = ctrlr_arg;

	mtx_lock(&nvd_lock);
	TAILQ_REMOVE(&ctrlr_head, ctrlr, tailq);
	mtx_unlock(&nvd_lock);

	nvd_controller_delete(ctrlr);
}

