/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (C) 2012-2014 Intel Corporation
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
 *
 * $FreeBSD$
 */

#ifndef __NVME_PRIVATE_H__
#define __NVME_PRIVATE_H__

#include <sys/param.h>
#include <sys/bio.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/systm.h>
#include <sys/taskqueue.h>

#include <vm/uma.h>

#include <machine/bus.h>

#include "nvme.h"

#define DEVICE2SOFTC(dev) ((struct nvme_pci_controller *) device_get_softc(dev))

MALLOC_DECLARE(M_NVME);

#define IDT32_PCI_ID		0x80d0111d /* 32 channel board */
#define IDT8_PCI_ID		0x80d2111d /* 8 channel board */

/*
 * For commands requiring more than 2 PRP entries, one PRP will be
 *  embedded in the command (prp1), and the rest of the PRP entries
 *  will be in a list pointed to by the command (prp2).  This means
 *  that real max number of PRP entries we support is 32+1, which
 *  results in a max xfer size of 32*PAGE_SIZE.
 */
#define NVME_MAX_PRP_LIST_ENTRIES	(NVME_MAX_XFER_SIZE / PAGE_SIZE)

#define NVME_ADMIN_TRACKERS	(16)
#define NVME_ADMIN_ENTRIES	(128)
/* min and max are defined in admin queue attributes section of spec */
#define NVME_MIN_ADMIN_ENTRIES	(2)
#define NVME_MAX_ADMIN_ENTRIES	(4096)

/*
 * NVME_IO_ENTRIES defines the size of an I/O qpair's submission and completion
 *  queues, while NVME_IO_TRACKERS defines the maximum number of I/O that we
 *  will allow outstanding on an I/O qpair at any time.  The only advantage in
 *  having IO_ENTRIES > IO_TRACKERS is for debugging purposes - when dumping
 *  the contents of the submission and completion queues, it will show a longer
 *  history of data.
 */
#define NVME_IO_ENTRIES		(256)
#define NVME_IO_TRACKERS	(128)
#define NVME_MIN_IO_TRACKERS	(4)
#define NVME_MAX_IO_TRACKERS	(1024)

/*
 * NVME_MAX_IO_ENTRIES is not defined, since it is specified in CC.MQES
 *  for each controller.
 */

#define NVME_INT_COAL_TIME	(0)	/* disabled */
#define NVME_INT_COAL_THRESHOLD (0)	/* 0-based */

#define NVME_MAX_NAMESPACES	(16)
#define NVME_MAX_CONSUMERS	(2)
#define NVME_MAX_ASYNC_EVENTS	(8)

#define NVME_DEFAULT_TIMEOUT_PERIOD	(30)    /* in seconds */
#define NVME_MIN_TIMEOUT_PERIOD		(5)
#define NVME_MAX_TIMEOUT_PERIOD		(120)

#define NVME_DEFAULT_RETRY_COUNT	(4)

/* Maximum log page size to fetch for AERs. */
#define NVME_MAX_AER_LOG_SIZE		(4096)

/*
 * Define CACHE_LINE_SIZE here for older FreeBSD versions that do not define
 *  it.
 */
#ifndef CACHE_LINE_SIZE
#define CACHE_LINE_SIZE		(64)
#endif

extern uma_zone_t	nvme_request_zone;
extern int32_t		nvme_retry_count;
extern bool		nvme_verbose_cmd_dump;

struct nvme_completion_poll_status {

	struct nvme_completion	cpl;
	int			done;
};

extern devclass_t nvme_devclass;

#define NVME_REQUEST_VADDR	1
#define NVME_REQUEST_NULL	2 /* For requests with no payload. */
#define NVME_REQUEST_UIO	3
#define NVME_REQUEST_BIO	4
#define NVME_REQUEST_CCB        5

struct nvme_request {

	struct nvme_command		cmd;
	struct nvme_qpair		*rqpair;
	union {
		void			*payload;
		struct bio		*bio;
	} u;
	uint32_t			type;
	uint32_t			payload_size;
	bool				timeout;
	nvme_cb_fn_t			cb_fn;
	void				*cb_arg;
	int32_t				retries;
	STAILQ_ENTRY(nvme_request)	stailq;
};

struct nvme_async_event_request {

	struct nvme_controller		*nvmea_ctrlrp;
	struct nvme_request		*req;
	struct nvme_completion		cpl;
	uint32_t			log_page_id;
	uint32_t			log_page_size;
	uint8_t				log_page_buffer[NVME_MAX_AER_LOG_SIZE];
};

struct nvme_tracker {

	TAILQ_ENTRY(nvme_tracker)	tailq;
	struct nvme_request		*req;
	struct nvme_pci_qpair		*qpair;
	struct callout			timer;
	bus_dmamap_t			payload_dma_map;
	uint16_t			cid;

	uint64_t			*prp;
	bus_addr_t			prp_bus_addr;
};

enum nvme_transport {
	NVMET_PCIE = 0xBF83E31D,
};

struct nvme_qpair {
	uint32_t		qid;
	uint32_t		num_qentries;
	volatile boolean_t	qis_enabled;
	struct mtx		qlock __aligned(CACHE_LINE_SIZE);
	enum nvme_transport	qttype;
	struct nvme_controller  *gqctrlr;
};

struct nvme_pci_qpair {

	struct nvme_pci_controller	*qpctrlr;
	int			domain;
	int			cpu;

	uint16_t		vector;
	int			rid;
	struct resource		*res;
	void 			*tag;

	uint32_t		num_trackers;
	uint32_t		sq_tdbl_off;
	uint32_t		cq_hdbl_off;

	uint32_t		phase;
	uint32_t		sq_head;
	uint32_t		sq_tail;
	uint32_t		cq_head;

	int64_t			num_cmds;
	int64_t			num_intr_handler_calls;
	int64_t			num_retries;
	int64_t			num_failures;

	struct nvme_command	*cmd;
	struct nvme_completion	*cpl;

	bus_dma_tag_t		dma_tag;
	bus_dma_tag_t		dma_tag_payload;

	bus_dmamap_t		queuemem_map;
	uint64_t		cmd_bus_addr;
	uint64_t		cpl_bus_addr;

	TAILQ_HEAD(, nvme_tracker)	free_tr;
	TAILQ_HEAD(, nvme_tracker)	outstanding_tr;
	STAILQ_HEAD(, nvme_request)	queued_req;

	struct nvme_tracker	**act_tr;

	struct nvme_qpair	gqpair;
} __aligned(CACHE_LINE_SIZE);

struct nvme_namespace {

	struct nvme_controller		*ctrlr;
	struct nvme_namespace_data	data;
	uint32_t			id;
	uint32_t			flags;
	struct cdev			*cdev;
	void				*cons_cookie[NVME_MAX_CONSUMERS];
	uint32_t			boundary;
	struct mtx			lock;
};

struct nvme_controller {
	struct mtx		lockc;

	uint32_t		ready_timeout_in_ms;
	uint32_t		cquirks;
#define QUIRK_DELAY_B4_CHK_RDY 1		/* Can't touch MMIO on disable */
	uint32_t		num_io_queues;
	uint32_t		max_hw_pend_io;

	struct task		reset_task;
	struct task		fail_req_task;
	struct taskqueue	*taskqueue;

	/** maximum i/o size in bytes */
	uint32_t		max_xfer_size;

	struct nvme_controller_data	cdata;

	/*
	 * Present in PCIe transport controllers from version 1.0 and in
	 * struct nvme_controller_data and above from version 1.2
	 */
	uint32_t		nvme_version;

	/** minimum page size supported by this controller in bytes */
	uint32_t		min_page_size;
	struct nvme_namespace		cns[NVME_MAX_NAMESPACES];

	struct cdev			*ccdev;

	/** bit mask of event types currently enabled for async events */
	uint32_t			async_event_config;

	uint32_t			num_aers;
	struct nvme_async_event_request	aer[NVME_MAX_ASYNC_EVENTS];

	void				*ccons_cookie[NVME_MAX_CONSUMERS];

	uint32_t			is_resetting;
	uint32_t			is_initialized;
	uint32_t			notification_sent;

	volatile boolean_t		is_failed;
	STAILQ_HEAD(, nvme_request)	fail_req;

	STAILQ_ENTRY(nvme_controller)	nvmec_lst;

	/** timeout period in seconds */
	uint32_t		timeout_period;

	enum nvme_transport		nvmec_ttype;
	int				nvmec_unit;
	char				nvmec_nameunit[SPECNAMELEN + 1];
	void				*nvmec_tsp;
	void 				(*nvmec_delist)(struct nvme_controller *);
	void 				(*nvmec_subadmreq)(struct nvme_controller *, struct nvme_request *);
	void 				(*nvmec_subioreq)(struct nvme_controller *, struct nvme_request *);
};
#define NVME_VFFSTRSZ 32

#define NVMP_STRING "NVMe over PCIe"
#define CONFIRMPCIECONTROLLER KASSERT(strncmp(pctrlr->very_first_field, \
    NVMP_STRING, sizeof(pctrlr->very_first_field)) == 0, \
    ("%s@%d NOT a PCIe controller!\n", __func__, __LINE__))
#define KASSERT_NVMP_CNTRLR(c) KASSERT((c)->nvmec_ttype == NVMET_PCIE, \
    ("%s@%d c:%p t:%d\n", __func__, __LINE__, (c), (c)->nvmec_ttype))
#define IS_NVMP_CNTRLR(c) ((c)->nvmec_ttype == NVMET_PCIE)

/*
 * One of these per allocated PCI device.
 */
struct nvme_pci_controller {
	char			very_first_field[NVME_VFFSTRSZ+1];
	device_t		dev;
	int			domain;

	uint32_t		quirks;
	bus_space_tag_t		bus_tag;
	bus_space_handle_t	bus_handle;
	int			resource_id;
	struct resource		*resource;

	/*
	 * The NVMe spec allows for the MSI-X table to be placed in BAR 4/5,
	 *  separate from the control registers which are in BAR 0/1.  These
	 *  members track the mapping of BAR 4/5 for that reason.
	 */
	int			bar4_resource_id;
	struct resource		*bar4_resource;

	uint32_t		msix_enabled;
	uint32_t		enable_aborts;

	/* Fields for tracking progress during controller initialization. */
	struct intr_config_hook	config_hook;
	uint32_t		ns_identified;
	uint32_t		queues_created;

	/* For shared legacy interrupt. */
	int			rid;
	struct resource		*res;
	void			*tag;

	bus_dma_tag_t		hw_desc_tag;
	bus_dmamap_t		hw_desc_map;

	/** interrupt coalescing time period (in microseconds) */
	uint32_t		int_coal_time;

	/** interrupt coalescing threshold */
	uint32_t		int_coal_threshold;

	/** doorbell stride */
	uint32_t		dstrd;

	struct nvme_pci_qpair	adminq;
	struct nvme_pci_qpair	*ioq;

	struct nvme_registers	*regs;

	struct nvme_controller	ctrlr;
};

#define GCNTRLR2PCI(c)  __containerof((c), struct nvme_pci_controller, ctrlr)

#define nvme_mmio_offsetof(reg)						       \
	offsetof(struct nvme_registers, reg)

#define nvme_mmio_read_4(sc, reg)					       \
	bus_space_read_4((sc)->bus_tag, (sc)->bus_handle,		       \
	    nvme_mmio_offsetof(reg))

#define nvme_mmio_write_4(sc, reg, val)					       \
	bus_space_write_4((sc)->bus_tag, (sc)->bus_handle,		       \
	    nvme_mmio_offsetof(reg), val)

#define nvme_mmio_write_8(sc, reg, val)					       \
	do {								       \
		bus_space_write_4((sc)->bus_tag, (sc)->bus_handle,	       \
		    nvme_mmio_offsetof(reg), val & 0xFFFFFFFF); 	       \
		bus_space_write_4((sc)->bus_tag, (sc)->bus_handle,	       \
		    nvme_mmio_offsetof(reg)+4,				       \
		    (val & 0xFFFFFFFF00000000ULL) >> 32);		       \
	} while (0);

#define NVMESPEW(c, format, ...) printf("nvme%d:<%s@%d> " format, \
    (c)->nvmec_unit, __func__, __LINE__, ## __VA_ARGS__)

#define nvme_printf(ctrlr, fmt, args...)	\
    NVMESPEW(ctrlr, fmt, ##args)

void	nvme_ns_test(struct nvme_namespace *ns, u_long cmd, caddr_t arg);

void	nvme_ctrlr_cmd_identify_controller(struct nvme_pci_controller *ctrlr,
					   void *payload,
					   nvme_cb_fn_t cb_fn, void *cb_arg);
void	nvme_ctrlr_cmd_identify_namespace(struct nvme_controller *ctrlr,
					  uint32_t nsid, void *payload,
					  nvme_cb_fn_t cb_fn, void *cb_arg);
void	nvme_ctrlr_cmd_set_interrupt_coalescing(struct nvme_pci_controller *ctrlr,
						uint32_t microseconds,
						uint32_t threshold,
						nvme_cb_fn_t cb_fn,
						void *cb_arg);
void	nvme_ctrlr_cmd_get_error_page(struct nvme_pci_controller *ctrlr,
				      struct nvme_error_information_entry *payload,
				      uint32_t num_entries, /* 0 = max */
				      nvme_cb_fn_t cb_fn,
				      void *cb_arg);
void	nvme_ctrlr_cmd_get_health_information_page(struct nvme_pci_controller *ctrlr,
						   uint32_t nsid,
						   struct nvme_health_information_page *payload,
						   nvme_cb_fn_t cb_fn,
						   void *cb_arg);
void	nvme_ctrlr_cmd_get_firmware_page(struct nvme_pci_controller *ctrlr,
					 struct nvme_firmware_page *payload,
					 nvme_cb_fn_t cb_fn,
					 void *cb_arg);
void	nvme_ctrlr_cmd_create_io_cq(struct nvme_pci_controller *ctrlr,
				    struct nvme_pci_qpair *io_que,
				    nvme_cb_fn_t cb_fn, void *cb_arg);
void	nvme_ctrlr_cmd_create_io_sq(struct nvme_pci_controller *ctrlr,
				    struct nvme_pci_qpair *io_que,
				    nvme_cb_fn_t cb_fn, void *cb_arg);
void	nvme_ctrlr_cmd_delete_io_cq(struct nvme_pci_controller *ctrlr,
				    struct nvme_pci_qpair *io_que,
				    nvme_cb_fn_t cb_fn, void *cb_arg);
void	nvme_ctrlr_cmd_delete_io_sq(struct nvme_pci_controller *ctrlr,
				    struct nvme_pci_qpair *io_que,
				    nvme_cb_fn_t cb_fn, void *cb_arg);
void	nvme_ctrlr_cmd_set_num_queues(struct nvme_pci_controller *ptrlr,
				      uint32_t num_queues, nvme_cb_fn_t cb_fn,
				      void *cb_arg);
void	nvme_ctrlr_cmd_set_async_event_config(struct nvme_pci_controller *ctrlr,
					      uint32_t state,
					      nvme_cb_fn_t cb_fn, void *cb_arg);
void	nvme_ctrlr_cmd_abort(struct nvme_pci_controller *ctrlr, uint16_t cid,
			     uint16_t sqid, nvme_cb_fn_t cb_fn, void *cb_arg);

void	nvme_completion_poll_cb(void *arg, const struct nvme_completion *cpl);

int	nvme_ctrlr_construct(struct nvme_pci_controller *ctrlr, device_t dev);
void	nvme_ctrlr_destruct(struct nvme_pci_controller *ctrlr, device_t dev);
void	nvme_ctrlr_shutdown(struct nvme_pci_controller *ctrlr);
int	nvme_ctrlr_hw_reset(struct nvme_pci_controller *ctrlr);
void	nvme_ctrlr_reset(struct nvme_pci_controller *ctrlr);
/* ctrlr defined as void * to allow use with config_intrhook. */
void	nvme_ctrlr_start_config_hook(void *ctrlr_arg);
void	nvmp_submit_adm_request(struct nvme_controller *ctrlr,
					struct nvme_request *req);
void	nvmp_submit_io_request(struct nvme_controller *ctrlr,
				     struct nvme_request *req);
void	nvme_ctrlr_post_failed_request(struct nvme_pci_controller *ctrlr,
				       struct nvme_request *req);

int	nvme_qpair_construct(struct nvme_pci_qpair *qpair,
			     uint32_t num_entries, uint32_t num_trackers,
			     struct nvme_pci_controller *ctrlr);
void	nvme_qpair_submit_tracker(struct nvme_pci_qpair *qpair,
				  struct nvme_tracker *tr);
bool	nvmp_qpair_process_completions(struct nvme_qpair *qpair);
void	nvme_qpair_submit_request(struct nvme_pci_qpair *qpair,
				  struct nvme_request *req);
void	nvme_qpair_reset(struct nvme_pci_qpair *qpair);
void	nvmp_qpair_fail(struct nvme_pci_qpair *qpair);
void	nvme_qpair_manual_complete_request(struct nvme_qpair *qpair,
					   struct nvme_request *req,
                                           uint32_t sct, uint32_t sc);

void	nvme_admin_qpair_enable(struct nvme_pci_qpair *qpair);
void	nvme_admin_qpair_disable(struct nvme_pci_qpair *qpair);
void	nvme_admin_qpair_destroy(struct nvme_pci_qpair *qpair);

void	nvme_io_qpair_enable(struct nvme_pci_qpair *qpair);
void	nvme_io_qpair_disable(struct nvme_pci_qpair *qpair);
void	nvme_io_qpair_destroy(struct nvme_pci_qpair *qpair);

int	nvme_ns_construct(struct nvme_namespace *ns, uint32_t id,
			  struct nvme_controller *ctrlr);
void	nvme_ns_destruct(struct nvme_namespace *ns);

void	nvme_sysctl_initialize_ctrlr(struct nvme_pci_controller *ctrlr);

void	nvme_dump_command(struct nvme_command *cmd);
void	nvme_dump_completion(struct nvme_completion *cpl);

int	nvme_attach(device_t dev);
int	nvme_shutdown(device_t dev);
int	nvme_detach(device_t dev);

/*
 * Wait for a command to complete using the nvme_completion_poll_cb.
 * Used in limited contexts where the caller knows it's OK to block
 * briefly while the command runs. The ISR will run the callback which
 * will set status->done to true.usually within microseconds. A 1s
 * pause means something is seriously AFU and we should panic to
 * provide the proper context to diagnose.
 */
static __inline
void
nvme_completion_poll(struct nvme_completion_poll_status *status)
{
	int sanity = hz * 1;

	while (!atomic_load_acq_int(&status->done) && --sanity > 0)
		pause("nvme", 1);
	if (sanity <= 0)
		panic("NVME polled command failed to complete within 1s.");
}

static __inline void
nvme_single_map(void *arg, bus_dma_segment_t *seg, int nseg, int error)
{
	uint64_t *bus_addr = (uint64_t *)arg;

	if (error != 0)
		printf("nvme_single_map err %d\n", error);
	*bus_addr = seg[0].ds_addr;
}

static __inline struct nvme_request *
_nvme_allocate_request(nvme_cb_fn_t cb_fn, void *cb_arg)
{
	struct nvme_request *req;

	req = uma_zalloc(nvme_request_zone, M_NOWAIT | M_ZERO);
	if (req != NULL) {
		req->cb_fn = cb_fn;
		req->cb_arg = cb_arg;
		req->timeout = true;
	}
	return (req);
}

static __inline struct nvme_request *
nvme_allocate_request_vaddr(void *payload, uint32_t payload_size,
    nvme_cb_fn_t cb_fn, void *cb_arg)
{
	struct nvme_request *req;

	req = _nvme_allocate_request(cb_fn, cb_arg);
	if (req != NULL) {
		req->type = NVME_REQUEST_VADDR;
		req->u.payload = payload;
		req->payload_size = payload_size;
	}
	return (req);
}

static __inline struct nvme_request *
nvme_allocate_request_null(nvme_cb_fn_t cb_fn, void *cb_arg)
{
	struct nvme_request *req;

	req = _nvme_allocate_request(cb_fn, cb_arg);
	if (req != NULL)
		req->type = NVME_REQUEST_NULL;
	return (req);
}

static __inline struct nvme_request *
nvme_allocate_request_bio(struct bio *bio, nvme_cb_fn_t cb_fn, void *cb_arg)
{
	struct nvme_request *req;

	req = _nvme_allocate_request(cb_fn, cb_arg);
	if (req != NULL) {
		req->type = NVME_REQUEST_BIO;
		req->u.bio = bio;
	}
	return (req);
}

static __inline struct nvme_request *
nvme_allocate_request_ccb(union ccb *ccb, nvme_cb_fn_t cb_fn, void *cb_arg)
{
	struct nvme_request *req;

	req = _nvme_allocate_request(cb_fn, cb_arg);
	if (req != NULL) {
		req->type = NVME_REQUEST_CCB;
		req->u.payload = ccb;
	}

	return (req);
}

#define nvme_free_request(req)	uma_zfree(nvme_request_zone, req)

void	nvme_notify_async_consumers(struct nvme_pci_controller *ctrlr,
				    const struct nvme_completion *async_cpl,
				    uint32_t log_page_id, void *log_page_buffer,
				    uint32_t log_page_size);
void	nvme_notify_fail_consumers(struct nvme_controller *ctrlr);
void	nvme_notify_new_controller(struct nvme_controller *ctrlr);
void	nvme_notify_ns(struct nvme_pci_controller *ctrlr, int nsid);

void	nvme_ctrlr_intx_handler(void *arg);
void	nvmp_ctrlr_poll(struct nvme_controller *ctrlr);

int	nvme_ctrlr_suspend(struct nvme_pci_controller *ctrlr);
int	nvme_ctrlr_resume(struct nvme_pci_controller *ctrlr);

#endif /* __NVME_PRIVATE_H__ */
