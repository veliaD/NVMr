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

#include "opt_cam.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/buf.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/ioccom.h>
#include <sys/proc.h>
#include <sys/smp.h>
#include <sys/uio.h>
#include <sys/endian.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include "nvme_private.h"

#define B4_CHK_RDY_DELAY_MS	2300		/* work around controller bug */

static void nvme_ctrlr_construct_and_submit_aer(struct nvme_pci_controller *pctrlr,
						struct nvme_async_event_request *aer);
static void nvme_ctrlr_setup_interrupts(struct nvme_pci_controller *pctrlr);

static int
nvme_ctrlr_allocate_bar(struct nvme_pci_controller *pctrlr)
{

	CONFIRMPCIECONTROLLER;
	pctrlr->resource_id = PCIR_BAR(0);

	pctrlr->resource = bus_alloc_resource_any(pctrlr->dev, SYS_RES_MEMORY,
	    &pctrlr->resource_id, RF_ACTIVE);

	if(pctrlr->resource == NULL) {
		nvme_printf(&(pctrlr->ctrlr), "unable to allocate pci resource\n");
		return (ENOMEM);
	}

	pctrlr->bus_tag = rman_get_bustag(pctrlr->resource);
	pctrlr->bus_handle = rman_get_bushandle(pctrlr->resource);
	pctrlr->regs = (struct nvme_registers *)pctrlr->bus_handle;

	/*
	 * The NVMe spec allows for the MSI-X table to be placed behind
	 *  BAR 4/5, separate from the control/doorbell registers.  Always
	 *  try to map this bar, because it must be mapped prior to calling
	 *  pci_alloc_msix().  If the table isn't behind BAR 4/5,
	 *  bus_alloc_resource() will just return NULL which is OK.
	 */
	pctrlr->bar4_resource_id = PCIR_BAR(4);
	pctrlr->bar4_resource = bus_alloc_resource_any(pctrlr->dev, SYS_RES_MEMORY,
	    &pctrlr->bar4_resource_id, RF_ACTIVE);

	return (0);
}

static int
nvme_ctrlr_construct_admin_qpair(struct nvme_pci_controller *pctrlr)
{
	struct nvme_pci_qpair	*qpair;
	uint32_t		num_entries;
	int			error;

	CONFIRMPCIECONTROLLER;
	qpair = &pctrlr->adminq;

	num_entries = NVME_ADMIN_ENTRIES;
	TUNABLE_INT_FETCH("hw.nvme.admin_entries", &num_entries);
	/*
	 * If admin_entries was overridden to an invalid value, revert it
	 *  back to our default value.
	 */
	if (num_entries < NVME_MIN_ADMIN_ENTRIES ||
	    num_entries > NVME_MAX_ADMIN_ENTRIES) {
		nvme_printf(&(pctrlr->ctrlr), "invalid hw.nvme.admin_entries=%d "
		    "specified\n", num_entries);
		num_entries = NVME_ADMIN_ENTRIES;
	}

	/*
	 * The admin queue's max xfer size is treated differently than the
	 *  max I/O xfer size.  16KB is sufficient here - maybe even less?
	 */
	error = nvme_qpair_construct(qpair, 
				     0, /* qpair ID */
				     0, /* vector */
				     num_entries,
				     NVME_ADMIN_TRACKERS,
				     pctrlr);
	return (error);
}

static int
nvme_ctrlr_construct_io_qpairs(struct nvme_pci_controller *pctrlr)
{
	struct nvme_pci_qpair	*qpair;
	uint32_t		cap_lo;
	uint16_t		mqes;
	int			i, error, num_entries, num_trackers;

	CONFIRMPCIECONTROLLER;
	num_entries = NVME_IO_ENTRIES;
	TUNABLE_INT_FETCH("hw.nvme.io_entries", &num_entries);

	/*
	 * NVMe spec sets a hard limit of 64K max entries, but
	 *  devices may specify a smaller limit, so we need to check
	 *  the MQES field in the capabilities register.
	 */
	cap_lo = nvme_mmio_read_4(pctrlr, cap_lo);
	mqes = (cap_lo >> NVME_CAP_LO_REG_MQES_SHIFT) & NVME_CAP_LO_REG_MQES_MASK;
	num_entries = min(num_entries, mqes + 1);

	num_trackers = NVME_IO_TRACKERS;
	TUNABLE_INT_FETCH("hw.nvme.io_trackers", &num_trackers);

	num_trackers = max(num_trackers, NVME_MIN_IO_TRACKERS);
	num_trackers = min(num_trackers, NVME_MAX_IO_TRACKERS);
	/*
	 * No need to have more trackers than entries in the submit queue.
	 *  Note also that for a queue size of N, we can only have (N-1)
	 *  commands outstanding, hence the "-1" here.
	 */
	num_trackers = min(num_trackers, (num_entries-1));

	/*
	 * Our best estimate for the maximum number of I/Os that we should
	 * noramlly have in flight at one time. This should be viewed as a hint,
	 * not a hard limit and will need to be revisitted when the upper layers
	 * of the storage system grows multi-queue support.
	 */
	pctrlr->ctrlr.max_hw_pend_io = num_trackers * pctrlr->ctrlr.num_io_queues * 3 / 4;

	/*
	 * This was calculated previously when setting up interrupts, but
	 *  a controller could theoretically support fewer I/O queues than
	 *  MSI-X vectors.  So calculate again here just to be safe.
	 */
	pctrlr->ctrlr.num_cpus_per_ioq = howmany(mp_ncpus, pctrlr->ctrlr.num_io_queues);

	pctrlr->ioq = malloc(pctrlr->ctrlr.num_io_queues * sizeof(struct nvme_pci_qpair),
	    M_NVME, M_ZERO | M_WAITOK);

	for (i = 0; i < pctrlr->ctrlr.num_io_queues; i++) {
		qpair = &pctrlr->ioq[i];

		/*
		 * Admin queue has ID=0. IO queues start at ID=1 -
		 *  hence the 'i+1' here.
		 *
		 * For I/O queues, use the controller-wide max_xfer_size
		 *  calculated in nvme_attach().
		 */
		error = nvme_qpair_construct(qpair,
				     i+1, /* qpair ID */
				     pctrlr->msix_enabled ? i+1 : 0, /* vector */
				     num_entries,
				     num_trackers,
				     pctrlr);
		if (error)
			return (error);

		/*
		 * Do not bother binding interrupts if we only have one I/O
		 *  interrupt thread for this controller.
		 */
		if (pctrlr->ctrlr.num_io_queues > 1)
			bus_bind_intr(pctrlr->dev, qpair->res,
			    i * pctrlr->ctrlr.num_cpus_per_ioq);
	}

	return (0);
}

static void
nvme_ctrlr_fail(struct nvme_pci_controller *pctrlr)
{
	int i;

	CONFIRMPCIECONTROLLER;
	pctrlr->ctrlr.is_failed = TRUE;
	nvmp_qpair_fail(&pctrlr->adminq);
	if (pctrlr->ioq != NULL) {
		for (i = 0; i < pctrlr->ctrlr.num_io_queues; i++)
			nvmp_qpair_fail(&pctrlr->ioq[i]);
	}
	nvme_notify_fail_consumers(pctrlr);
}

void
nvme_ctrlr_post_failed_request(struct nvme_pci_controller *pctrlr,
    struct nvme_request *req)
{

	CONFIRMPCIECONTROLLER;
	mtx_lock(&pctrlr->ctrlr.lockc);
	STAILQ_INSERT_TAIL(&pctrlr->ctrlr.fail_req, req, stailq);
	mtx_unlock(&pctrlr->ctrlr.lockc);
	taskqueue_enqueue(pctrlr->ctrlr.taskqueue, &pctrlr->ctrlr.fail_req_task);
}

void
nvme_ctrlr_fail_req_task(void *arg, int pending)
{
	struct nvme_controller	*ctrlr = arg;
	struct nvme_request	*req;

	KASSERT((ctrlr->nvmec_ttype == NVMET_RDMA) ||
	    (ctrlr->nvmec_ttype == NVMET_RDMA),
	    ("Unknown NVMe transport c:%p t:%d", ctrlr, ctrlr->nvmec_ttype));

	mtx_lock(&ctrlr->lockc);
	while ((req = STAILQ_FIRST(&ctrlr->fail_req)) != NULL) {
		STAILQ_REMOVE_HEAD(&ctrlr->fail_req, stailq);
		mtx_unlock(&ctrlr->lockc);
		nvme_qpair_manual_complete_request(req->rqpair, req,
		    NVME_SCT_GENERIC, NVME_SC_ABORTED_BY_REQUEST, TRUE);
		mtx_lock(&ctrlr->lockc);
	}
	mtx_unlock(&ctrlr->lockc);
}

static int
nvme_ctrlr_wait_for_ready(struct nvme_pci_controller *pctrlr, int desired_val)
{
	int ms_waited;
	uint32_t csts;

	CONFIRMPCIECONTROLLER;
	csts = nvme_mmio_read_4(pctrlr, csts);

	ms_waited = 0;
	while (((csts >> NVME_CSTS_REG_RDY_SHIFT) & NVME_CSTS_REG_RDY_MASK) != desired_val) {
		if (ms_waited++ > pctrlr->ctrlr.ready_timeout_in_ms) {
			nvme_printf(&(pctrlr->ctrlr), "controller ready did not become %d "
			    "within %d ms\n", desired_val, pctrlr->ctrlr.ready_timeout_in_ms);
			return (ENXIO);
		}
		DELAY(1000);
		csts = nvme_mmio_read_4(pctrlr, csts);
	}

	return (0);
}

static int
nvme_ctrlr_disable(struct nvme_pci_controller *pctrlr)
{
	uint32_t cc;
	uint32_t csts;
	uint8_t  en, rdy;
	int err;

	CONFIRMPCIECONTROLLER;
	cc = nvme_mmio_read_4(pctrlr, cc);
	csts = nvme_mmio_read_4(pctrlr, csts);

	en = (cc >> NVME_CC_REG_EN_SHIFT) & NVME_CC_REG_EN_MASK;
	rdy = (csts >> NVME_CSTS_REG_RDY_SHIFT) & NVME_CSTS_REG_RDY_MASK;

	/*
	 * Per 3.1.5 in NVME 1.3 spec, transitioning CC.EN from 0 to 1
	 * when CSTS.RDY is 1 or transitioning CC.EN from 1 to 0 when
	 * CSTS.RDY is 0 "has undefined results" So make sure that CSTS.RDY
	 * isn't the desired value. Short circuit if we're already disabled.
	 */
	if (en == 1) {
		if (rdy == 0) {
			/* EN == 1, wait for  RDY == 1 or fail */
			err = nvme_ctrlr_wait_for_ready(pctrlr, 1);
			if (err != 0)
				return (err);
		}
	} else {
		/* EN == 0 already wait for RDY == 0 */
		if (rdy == 0)
			return (0);
		else
			return (nvme_ctrlr_wait_for_ready(pctrlr, 0));
	}

	cc &= ~NVME_CC_REG_EN_MASK;
	nvme_mmio_write_4(pctrlr, cc, cc);
	/*
	 * Some drives have issues with accessing the mmio after we
	 * disable, so delay for a bit after we write the bit to
	 * cope with these issues.
	 */
	if (pctrlr->ctrlr.cquirks & QUIRK_DELAY_B4_CHK_RDY)
		pause("nvmeR", B4_CHK_RDY_DELAY_MS * hz / 1000);
	return (nvme_ctrlr_wait_for_ready(pctrlr, 0));
}

static int
nvme_ctrlr_enable(struct nvme_pci_controller *pctrlr)
{
	uint32_t	cc;
	uint32_t	csts;
	uint32_t	aqa;
	uint32_t	qsize;
	uint8_t		en, rdy;
	int		err;

	CONFIRMPCIECONTROLLER;
	cc = nvme_mmio_read_4(pctrlr, cc);
	csts = nvme_mmio_read_4(pctrlr, csts);

	en = (cc >> NVME_CC_REG_EN_SHIFT) & NVME_CC_REG_EN_MASK;
	rdy = (csts >> NVME_CSTS_REG_RDY_SHIFT) & NVME_CSTS_REG_RDY_MASK;

	/*
	 * See note in nvme_ctrlr_disable. Short circuit if we're already enabled.
	 */
	if (en == 1) {
		if (rdy == 1)
			return (0);
		else
			return (nvme_ctrlr_wait_for_ready(pctrlr, 1));
	} else {
		/* EN == 0 already wait for RDY == 0 or fail */
		err = nvme_ctrlr_wait_for_ready(pctrlr, 0);
		if (err != 0)
			return (err);
	}

	nvme_mmio_write_8(pctrlr, asq, pctrlr->adminq.cmd_bus_addr);
	DELAY(5000);
	nvme_mmio_write_8(pctrlr, acq, pctrlr->adminq.cpl_bus_addr);
	DELAY(5000);

	/* acqs and asqs are 0-based. */
	qsize = pctrlr->adminq.gqpair.num_qentries - 1;

	aqa = 0;
	aqa = (qsize & NVME_AQA_REG_ACQS_MASK) << NVME_AQA_REG_ACQS_SHIFT;
	aqa |= (qsize & NVME_AQA_REG_ASQS_MASK) << NVME_AQA_REG_ASQS_SHIFT;
	nvme_mmio_write_4(pctrlr, aqa, aqa);
	DELAY(5000);

	/* Initialization values for CC */
	cc = 0;
	cc |= 1 << NVME_CC_REG_EN_SHIFT;
	cc |= 0 << NVME_CC_REG_CSS_SHIFT;
	cc |= 0 << NVME_CC_REG_AMS_SHIFT;
	cc |= 0 << NVME_CC_REG_SHN_SHIFT;
	cc |= 6 << NVME_CC_REG_IOSQES_SHIFT; /* SQ entry size == 64 == 2^6 */
	cc |= 4 << NVME_CC_REG_IOCQES_SHIFT; /* CQ entry size == 16 == 2^4 */

	/* This evaluates to 0, which is according to spec. */
	cc |= (PAGE_SIZE >> 13) << NVME_CC_REG_MPS_SHIFT;

	nvme_mmio_write_4(pctrlr, cc, cc);

	return (nvme_ctrlr_wait_for_ready(pctrlr, 1));
}

int
nvme_ctrlr_hw_reset(struct nvme_pci_controller *pctrlr)
{
	int i, err;

	CONFIRMPCIECONTROLLER;
	nvme_admin_qpair_disable(&pctrlr->adminq);
	/*
	 * I/O queues are not allocated before the initial HW
	 *  reset, so do not try to disable them.  Use is_initialized
	 *  to determine if this is the initial HW reset.
	 */
	if (pctrlr->ctrlr.is_initialized) {
		for (i = 0; i < pctrlr->ctrlr.num_io_queues; i++)
			nvme_io_qpair_disable(&pctrlr->ioq[i]);
	}

	DELAY(100*1000);

	err = nvme_ctrlr_disable(pctrlr);
	if (err != 0)
		return err;
	return (nvme_ctrlr_enable(pctrlr));
}

void
nvme_ctrlr_reset(struct nvme_pci_controller *pctrlr)
{
	int cmpset;

	CONFIRMPCIECONTROLLER;
	cmpset = atomic_cmpset_32(&pctrlr->ctrlr.is_resetting, 0, 1);

	if (cmpset == 0 || pctrlr->ctrlr.is_failed)
		/*
		 * Controller is already resetting or has failed.  Return
		 *  immediately since there is no need to kick off another
		 *  reset in these cases.
		 */
		return;

	taskqueue_enqueue(pctrlr->ctrlr.taskqueue, &pctrlr->ctrlr.reset_task);
}

static int
nvme_ctrlr_identify(struct nvme_pci_controller *pctrlr)
{
	struct nvme_completion_poll_status	status;

	CONFIRMPCIECONTROLLER;
	status.done = 0;
	nvme_ctrlr_cmd_identify_controller(pctrlr, &pctrlr->ctrlr.cdata,
	    nvme_completion_poll_cb, &status);
	while (!atomic_load_acq_int(&status.done))
		pause("nvme", 1);
	if (nvme_completion_is_error(&status.cpl)) {
		nvme_printf(&(pctrlr->ctrlr), "nvme_identify_controller failed!\n");
		return (ENXIO);
	}

	/* Convert data to host endian */
	nvme_controller_data_swapbytes(&pctrlr->ctrlr.cdata);

	/*
	 * Use MDTS to ensure our default max_xfer_size doesn't exceed what the
	 *  controller supports.
	 */
	if (pctrlr->ctrlr.cdata.mdts > 0)
		pctrlr->ctrlr.max_xfer_size = min(pctrlr->ctrlr.max_xfer_size,
		    pctrlr->ctrlr.min_page_size * (1 << (pctrlr->ctrlr.cdata.mdts)));

	return (0);
}

static int
nvme_ctrlr_set_num_qpairs(struct nvme_pci_controller *pctrlr)
{
	struct nvme_completion_poll_status	status;
	int					cq_allocated, sq_allocated;

	CONFIRMPCIECONTROLLER;
	status.done = 0;
	nvme_ctrlr_cmd_set_num_queues(pctrlr, pctrlr->ctrlr.num_io_queues,
	    nvme_completion_poll_cb, &status);
	while (!atomic_load_acq_int(&status.done))
		pause("nvme", 1);
	if (nvme_completion_is_error(&status.cpl)) {
		nvme_printf(&(pctrlr->ctrlr), "nvme_ctrlr_set_num_qpairs failed!\n");
		return (ENXIO);
	}

	/*
	 * Data in cdw0 is 0-based.
	 * Lower 16-bits indicate number of submission queues allocated.
	 * Upper 16-bits indicate number of completion queues allocated.
	 */
	sq_allocated = (status.cpl.cdw0 & 0xFFFF) + 1;
	cq_allocated = (status.cpl.cdw0 >> 16) + 1;

	/*
	 * Controller may allocate more queues than we requested,
	 *  so use the minimum of the number requested and what was
	 *  actually allocated.
	 */
	pctrlr->ctrlr.num_io_queues = min(pctrlr->ctrlr.num_io_queues, sq_allocated);
	pctrlr->ctrlr.num_io_queues = min(pctrlr->ctrlr.num_io_queues, cq_allocated);

	return (0);
}

static int
nvme_ctrlr_create_qpairs(struct nvme_pci_controller *pctrlr)
{
	struct nvme_completion_poll_status	status;
	struct nvme_pci_qpair			*qpair;
	int					i;

	CONFIRMPCIECONTROLLER;
	for (i = 0; i < pctrlr->ctrlr.num_io_queues; i++) {
		qpair = &pctrlr->ioq[i];

		status.done = 0;
		nvme_ctrlr_cmd_create_io_cq(pctrlr, qpair, qpair->vector,
		    nvme_completion_poll_cb, &status);
		while (!atomic_load_acq_int(&status.done))
			pause("nvme", 1);
		if (nvme_completion_is_error(&status.cpl)) {
			nvme_printf(&(pctrlr->ctrlr), "nvme_create_io_cq failed!\n");
			return (ENXIO);
		}

		status.done = 0;
		nvme_ctrlr_cmd_create_io_sq(qpair->qpctrlr, qpair,
		    nvme_completion_poll_cb, &status);
		while (!atomic_load_acq_int(&status.done))
			pause("nvme", 1);
		if (nvme_completion_is_error(&status.cpl)) {
			nvme_printf(&(pctrlr->ctrlr), "nvme_create_io_sq failed!\n");
			return (ENXIO);
		}
	}

	return (0);
}

static int
nvme_ctrlr_destroy_qpair(struct nvme_pci_controller *pctrlr,
    struct nvme_pci_qpair *pqpair)
{
	struct nvme_completion_poll_status	status;

	CONFIRMPCIECONTROLLER;
	status.done = 0;
	nvme_ctrlr_cmd_delete_io_sq(pctrlr, pqpair,
	    nvme_completion_poll_cb, &status);
	while (!atomic_load_acq_int(&status.done))
		pause("nvme", 1);
	if (nvme_completion_is_error(&status.cpl)) {
		nvme_printf(&(pctrlr->ctrlr), "nvme_destroy_io_sq failed!\n");
		return (ENXIO);
	}

	status.done = 0;
	nvme_ctrlr_cmd_delete_io_cq(pctrlr, pqpair,
	    nvme_completion_poll_cb, &status);
	while (!atomic_load_acq_int(&status.done))
		pause("nvme", 1);
	if (nvme_completion_is_error(&status.cpl)) {
		nvme_printf(&(pctrlr->ctrlr), "nvme_destroy_io_cq failed!\n");
		return (ENXIO);
	}

	return (0);
}

int
nvme_ctrlr_construct_namespaces(struct nvme_controller *ctrlr)
{
	struct nvme_namespace	*ns;
	uint32_t 		i;
	int			retval;

	retval = 0;
	for (i = 0; i < min(ctrlr->cdata.nn, NVME_MAX_NAMESPACES); i++) {
		ns = &ctrlr->cns[i];
		retval = nvme_ns_construct(ns, i+1, ctrlr);
		if (retval != 0) {
			ERRSPEW("nvme_ns_construct(i:%u c:%p):%d\n", i, ctrlr,
			    retval);
		}
	}

	return (retval);
}

static boolean_t
is_log_page_id_valid(uint8_t page_id)
{

	switch (page_id) {
	case NVME_LOG_ERROR:
	case NVME_LOG_HEALTH_INFORMATION:
	case NVME_LOG_FIRMWARE_SLOT:
	case NVME_LOG_CHANGED_NAMESPACE:
		return (TRUE);
	}

	return (FALSE);
}

static uint32_t
nvme_ctrlr_get_log_page_size(struct nvme_pci_controller *pctrlr, uint8_t page_id)
{
	uint32_t	log_page_size;

	CONFIRMPCIECONTROLLER;
	switch (page_id) {
	case NVME_LOG_ERROR:
		log_page_size = min(
		    sizeof(struct nvme_error_information_entry) *
		    (pctrlr->ctrlr.cdata.elpe + 1), NVME_MAX_AER_LOG_SIZE);
		break;
	case NVME_LOG_HEALTH_INFORMATION:
		log_page_size = sizeof(struct nvme_health_information_page);
		break;
	case NVME_LOG_FIRMWARE_SLOT:
		log_page_size = sizeof(struct nvme_firmware_page);
		break;
	case NVME_LOG_CHANGED_NAMESPACE:
		log_page_size = sizeof(struct nvme_ns_list);
		break;
	default:
		log_page_size = 0;
		break;
	}

	return (log_page_size);
}

static void
nvme_ctrlr_log_critical_warnings(struct nvme_pci_controller *pctrlr,
    uint8_t state)
{

	CONFIRMPCIECONTROLLER;
	if (state & NVME_CRIT_WARN_ST_AVAILABLE_SPARE)
		nvme_printf(&(pctrlr->ctrlr), "available spare space below threshold\n");

	if (state & NVME_CRIT_WARN_ST_TEMPERATURE)
		nvme_printf(&(pctrlr->ctrlr), "temperature above threshold\n");

	if (state & NVME_CRIT_WARN_ST_DEVICE_RELIABILITY)
		nvme_printf(&(pctrlr->ctrlr), "device reliability degraded\n");

	if (state & NVME_CRIT_WARN_ST_READ_ONLY)
		nvme_printf(&(pctrlr->ctrlr), "media placed in read only mode\n");

	if (state & NVME_CRIT_WARN_ST_VOLATILE_MEMORY_BACKUP)
		nvme_printf(&(pctrlr->ctrlr), "volatile memory backup device failed\n");

	if (state & NVME_CRIT_WARN_ST_RESERVED_MASK)
		nvme_printf(&(pctrlr->ctrlr),
		    "unknown critical warning(s): state = 0x%02x\n", state);
}

static void
nvme_ctrlr_async_event_log_page_cb(void *arg, const struct nvme_completion *cpl)
{
	struct nvme_async_event_request		*aer = arg;
	struct nvme_health_information_page	*health_info;
	struct nvme_ns_list			*nsl;
	struct nvme_error_information_entry	*err;
	int i;
	struct nvme_pci_controller *pctrlr;

	KASSERT_NVMP_CNTRLR(aer->nvmea_ctrlrp);
	pctrlr = aer->nvmea_ctrlrp->nvmec_tsp;
	CONFIRMPCIECONTROLLER;

	/*
	 * If the log page fetch for some reason completed with an error,
	 *  don't pass log page data to the consumers.  In practice, this case
	 *  should never happen.
	 */
	if (nvme_completion_is_error(cpl))
		nvme_notify_async_consumers(pctrlr, &aer->cpl,
		    aer->log_page_id, NULL, 0);
	else {
		/* Convert data to host endian */
		switch (aer->log_page_id) {
		case NVME_LOG_ERROR:
			err = (struct nvme_error_information_entry *)aer->log_page_buffer;
			for (i = 0; i < (pctrlr->ctrlr.cdata.elpe + 1); i++)
				nvme_error_information_entry_swapbytes(err++);
			break;
		case NVME_LOG_HEALTH_INFORMATION:
			nvme_health_information_page_swapbytes(
			    (struct nvme_health_information_page *)aer->log_page_buffer);
			break;
		case NVME_LOG_FIRMWARE_SLOT:
			nvme_firmware_page_swapbytes(
			    (struct nvme_firmware_page *)aer->log_page_buffer);
			break;
		case NVME_LOG_CHANGED_NAMESPACE:
			nvme_ns_list_swapbytes(
			    (struct nvme_ns_list *)aer->log_page_buffer);
			break;
		case INTEL_LOG_TEMP_STATS:
			intel_log_temp_stats_swapbytes(
			    (struct intel_log_temp_stats *)aer->log_page_buffer);
			break;
		default:
			break;
		}

		if (aer->log_page_id == NVME_LOG_HEALTH_INFORMATION) {
			health_info = (struct nvme_health_information_page *)
			    aer->log_page_buffer;
			nvme_ctrlr_log_critical_warnings(pctrlr,
			    health_info->critical_warning);
			/*
			 * Critical warnings reported through the
			 *  SMART/health log page are persistent, so
			 *  clear the associated bits in the async event
			 *  config so that we do not receive repeated
			 *  notifications for the same event.
			 */
			aer->nvmea_ctrlrp->async_event_config &=
			    ~health_info->critical_warning;
			nvme_ctrlr_cmd_set_async_event_config(pctrlr,
			    aer->nvmea_ctrlrp->async_event_config, NULL, NULL);
		} else if (aer->log_page_id == NVME_LOG_CHANGED_NAMESPACE &&
		    !nvme_use_nvd) {
			nsl = (struct nvme_ns_list *)aer->log_page_buffer;
			for (i = 0; i < nitems(nsl->ns) && nsl->ns[i] != 0; i++) {
				if (nsl->ns[i] > NVME_MAX_NAMESPACES)
					break;
				nvme_notify_ns(pctrlr, nsl->ns[i]);
			}
		}


		/*
		 * Pass the cpl data from the original async event completion,
		 *  not the log page fetch.
		 */
		nvme_notify_async_consumers(pctrlr, &aer->cpl,
		    aer->log_page_id, aer->log_page_buffer, aer->log_page_size);
	}

	/*
	 * Repost another asynchronous event request to replace the one
	 *  that just completed.
	 */
	nvme_ctrlr_construct_and_submit_aer(pctrlr, aer);
}

static void
nvme_ctrlr_async_event_cb(void *arg, const struct nvme_completion *cpl)
{
	struct nvme_async_event_request	*aer = arg;
	struct nvme_pci_controller *pctrlr;

	KASSERT_NVMP_CNTRLR(aer->nvmea_ctrlrp);
	pctrlr = aer->nvmea_ctrlrp->nvmec_tsp;
	CONFIRMPCIECONTROLLER;


	if (nvme_completion_is_error(cpl)) {
		/*
		 *  Do not retry failed async event requests.  This avoids
		 *  infinite loops where a new async event request is submitted
		 *  to replace the one just failed, only to fail again and
		 *  perpetuate the loop.
		 */
		return;
	}

	/* Associated log page is in bits 23:16 of completion entry dw0. */
	aer->log_page_id = (cpl->cdw0 & 0xFF0000) >> 16;

	nvme_printf(&(pctrlr->ctrlr), "async event occurred (type 0x%x, info 0x%02x,"
	    " page 0x%02x)\n", (cpl->cdw0 & 0x03), (cpl->cdw0 & 0xFF00) >> 8,
	    aer->log_page_id);

	if (is_log_page_id_valid(aer->log_page_id)) {
		aer->log_page_size = nvme_ctrlr_get_log_page_size(pctrlr,
		    aer->log_page_id);
		memcpy(&aer->cpl, cpl, sizeof(*cpl));
		nvme_ctrlr_cmd_get_log_page(pctrlr, aer->log_page_id,
		    NVME_GLOBAL_NAMESPACE_TAG, aer->log_page_buffer,
		    aer->log_page_size, nvme_ctrlr_async_event_log_page_cb,
		    aer);
		/* Wait to notify consumers until after log page is fetched. */
	} else {
		nvme_notify_async_consumers(pctrlr, cpl, aer->log_page_id,
		    NULL, 0);

		/*
		 * Repost another asynchronous event request to replace the one
		 *  that just completed.
		 */
		nvme_ctrlr_construct_and_submit_aer(pctrlr, aer);
	}
}

static void
nvme_ctrlr_construct_and_submit_aer(struct nvme_pci_controller *pctrlr,
    struct nvme_async_event_request *aer)
{
	struct nvme_request *req;

	CONFIRMPCIECONTROLLER;
	aer->nvmea_ctrlrp = &pctrlr->ctrlr;
	req = nvme_allocate_request_null(nvme_ctrlr_async_event_cb, aer);
	aer->req = req;

	/*
	 * Disable timeout here, since asynchronous event requests should by
	 *  nature never be timed out.
	 */
	req->timeout = FALSE;
	req->cmd.opc = NVME_OPC_ASYNC_EVENT_REQUEST;
	nvme_ctrlr_submit_admin_request(aer->nvmea_ctrlrp, req);
}

static void
nvme_ctrlr_configure_aer(struct nvme_pci_controller *pctrlr)
{
	struct nvme_completion_poll_status	status;
	struct nvme_async_event_request		*aer;
	uint32_t				i;

	CONFIRMPCIECONTROLLER;
	pctrlr->ctrlr.async_event_config = NVME_CRIT_WARN_ST_AVAILABLE_SPARE |
	    NVME_CRIT_WARN_ST_DEVICE_RELIABILITY |
	    NVME_CRIT_WARN_ST_READ_ONLY |
	    NVME_CRIT_WARN_ST_VOLATILE_MEMORY_BACKUP;
	if (pctrlr->ctrlr.cdata.ver >= NVME_REV(1, 2))
		pctrlr->ctrlr.async_event_config |= 0x300;

	status.done = 0;
	nvme_ctrlr_cmd_get_feature(pctrlr, NVME_FEAT_TEMPERATURE_THRESHOLD,
	    0, NULL, 0, nvme_completion_poll_cb, &status);
	while (!atomic_load_acq_int(&status.done))
		pause("nvme", 1);
	if (nvme_completion_is_error(&status.cpl) ||
	    (status.cpl.cdw0 & 0xFFFF) == 0xFFFF ||
	    (status.cpl.cdw0 & 0xFFFF) == 0x0000) {
		nvme_printf(&(pctrlr->ctrlr), "temperature threshold not supported\n");
	} else
		pctrlr->ctrlr.async_event_config |= NVME_CRIT_WARN_ST_TEMPERATURE;

	nvme_ctrlr_cmd_set_async_event_config(pctrlr,
	    pctrlr->ctrlr.async_event_config, NULL, NULL);

	/* aerl is a zero-based value, so we need to add 1 here. */
	pctrlr->ctrlr.num_aers = min(NVME_MAX_ASYNC_EVENTS, (pctrlr->ctrlr.cdata.aerl+1));

	for (i = 0; i < pctrlr->ctrlr.num_aers; i++) {
		aer = &pctrlr->ctrlr.aer[i];
		nvme_ctrlr_construct_and_submit_aer(pctrlr, aer);
	}
}

static void
nvme_ctrlr_configure_int_coalescing(struct nvme_pci_controller *pctrlr)
{

	CONFIRMPCIECONTROLLER;
	pctrlr->int_coal_time = 0;
	TUNABLE_INT_FETCH("hw.nvme.int_coal_time",
	    &pctrlr->int_coal_time);

	pctrlr->int_coal_threshold = 0;
	TUNABLE_INT_FETCH("hw.nvme.int_coal_threshold",
	    &pctrlr->int_coal_threshold);

	nvme_ctrlr_cmd_set_interrupt_coalescing(pctrlr, pctrlr->int_coal_time,
	    pctrlr->int_coal_threshold, NULL, NULL);
}

static void
nvme_ctrlr_start(void *ctrlr_arg)
{
	struct nvme_pci_controller *pctrlr = ctrlr_arg;
	uint32_t old_num_io_queues;
	int i;

	CONFIRMPCIECONTROLLER;
	/*
	 * Only reset adminq here when we are restarting the
	 *  controller after a reset.  During initialization,
	 *  we have already submitted admin commands to get
	 *  the number of I/O queues supported, so cannot reset
	 *  the adminq again here.
	 */
	if (pctrlr->ctrlr.is_resetting) {
		nvme_qpair_reset(&pctrlr->adminq);
	}

	for (i = 0; i < pctrlr->ctrlr.num_io_queues; i++)
		nvme_qpair_reset(&pctrlr->ioq[i]);

	nvme_admin_qpair_enable(&pctrlr->adminq);

	if (nvme_ctrlr_identify(pctrlr) != 0) {
		nvme_ctrlr_fail(pctrlr);
		return;
	}

	/*
	 * The number of qpairs are determined during controller initialization,
	 *  including using NVMe SET_FEATURES/NUMBER_OF_QUEUES to determine the
	 *  HW limit.  We call SET_FEATURES again here so that it gets called
	 *  after any reset for controllers that depend on the driver to
	 *  explicit specify how many queues it will use.  This value should
	 *  never change between resets, so panic if somehow that does happen.
	 */
	if (pctrlr->ctrlr.is_resetting) {
		old_num_io_queues = pctrlr->ctrlr.num_io_queues;
		if (nvme_ctrlr_set_num_qpairs(pctrlr) != 0) {
			nvme_ctrlr_fail(pctrlr);
			return;
		}

		if (old_num_io_queues != pctrlr->ctrlr.num_io_queues) {
			panic("num_io_queues changed from %u to %u",
			      old_num_io_queues, pctrlr->ctrlr.num_io_queues);
		}
	}

	if (nvme_ctrlr_create_qpairs(pctrlr) != 0) {
		nvme_ctrlr_fail(pctrlr);
		return;
	}

	if (nvme_ctrlr_construct_namespaces(&pctrlr->ctrlr) != 0) {
		nvme_ctrlr_fail(pctrlr);
		return;
	}

	nvme_ctrlr_configure_aer(pctrlr);
	nvme_ctrlr_configure_int_coalescing(pctrlr);

	for (i = 0; i < pctrlr->ctrlr.num_io_queues; i++)
		nvme_io_qpair_enable(&pctrlr->ioq[i]);
}

void
nvme_ctrlr_start_config_hook(void *arg)
{
	struct nvme_pci_controller *pctrlr = arg;

	CONFIRMPCIECONTROLLER;
	nvme_qpair_reset(&pctrlr->adminq);
	nvme_admin_qpair_enable(&pctrlr->adminq);

	if (nvme_ctrlr_set_num_qpairs(pctrlr) == 0 &&
	    nvme_ctrlr_construct_io_qpairs(pctrlr) == 0)
		nvme_ctrlr_start(pctrlr);
	else
		nvme_ctrlr_fail(pctrlr);

	nvme_sysctl_initialize_ctrlr(pctrlr);
	config_intrhook_disestablish(&pctrlr->config_hook);

	pctrlr->ctrlr.is_initialized = 1;
	nvme_notify_new_controller(pctrlr);
}

static void
nvme_ctrlr_reset_task(void *arg, int pending)
{
	struct nvme_pci_controller	*pctrlr = arg;
	int			status;

	CONFIRMPCIECONTROLLER;
	nvme_printf(&(pctrlr->ctrlr), "resetting controller\n");
	status = nvme_ctrlr_hw_reset(pctrlr);
	/*
	 * Use pause instead of DELAY, so that we yield to any nvme interrupt
	 *  handlers on this CPU that were blocked on a qpair lock. We want
	 *  all nvme interrupts completed before proceeding with restarting the
	 *  controller.
	 *
	 * XXX - any way to guarantee the interrupt handlers have quiesced?
	 */
	pause("nvmereset", hz / 10);
	if (status == 0)
		nvme_ctrlr_start(pctrlr);
	else
		nvme_ctrlr_fail(pctrlr);

	atomic_cmpset_32(&pctrlr->ctrlr.is_resetting, 1, 0);
}

/*
 * Poll all the queues enabled on the device for completion.
 */
void
nvme_ctrlr_poll(struct nvme_pci_controller *pctrlr)
{
	int i;

	CONFIRMPCIECONTROLLER;
	nvmp_qpair_process_completions(&pctrlr->adminq.gqpair);

	for (i = 0; i < pctrlr->ctrlr.num_io_queues; i++)
		if (pctrlr->ioq && pctrlr->ioq[i].cpl)
			nvmp_qpair_process_completions(&pctrlr->ioq[i].gqpair);
}

/*
 * Poll the single-vector intertrupt case: num_io_queues will be 1 and
 * there's only a single vector. While we're polling, we mask further
 * interrupts in the controller.
 */
void
nvme_ctrlr_intx_handler(void *arg)
{
	struct nvme_pci_controller *pctrlr = arg;

	CONFIRMPCIECONTROLLER;
	nvme_mmio_write_4(pctrlr, intms, 1);
	nvme_ctrlr_poll(pctrlr);
	nvme_mmio_write_4(pctrlr, intmc, 1);
}

static int
nvme_ctrlr_configure_intx(struct nvme_pci_controller *pctrlr)
{

	CONFIRMPCIECONTROLLER;
	pctrlr->msix_enabled = 0;
	pctrlr->ctrlr.num_io_queues = 1;
	pctrlr->ctrlr.num_cpus_per_ioq = mp_ncpus;
	pctrlr->rid = 0;
	pctrlr->res = bus_alloc_resource_any(pctrlr->dev, SYS_RES_IRQ,
	    &pctrlr->rid, RF_SHAREABLE | RF_ACTIVE);

	if (pctrlr->res == NULL) {
		nvme_printf(&(pctrlr->ctrlr), "unable to allocate shared IRQ\n");
		return (ENOMEM);
	}

	bus_setup_intr(pctrlr->dev, pctrlr->res,
	    INTR_TYPE_MISC | INTR_MPSAFE, NULL, nvme_ctrlr_intx_handler,
	    pctrlr, &pctrlr->tag);

	if (pctrlr->tag == NULL) {
		nvme_printf(&(pctrlr->ctrlr), "unable to setup intx handler\n");
		return (ENOMEM);
	}

	return (0);
}

static void
nvme_pt_done(void *arg, const struct nvme_completion *cpl)
{
	struct nvme_pt_command *pt = arg;
	struct mtx *mtx = pt->driver_lock;
	uint16_t status;

	bzero(&pt->cpl, sizeof(pt->cpl));
	pt->cpl.cdw0 = cpl->cdw0;

	status = cpl->status;
	status &= ~NVME_STATUS_P_MASK;
	pt->cpl.status = status;

	mtx_lock(mtx);
	pt->driver_lock = NULL;
	wakeup(pt);
	mtx_unlock(mtx);
}

int
nvme_ctrlr_passthrough_cmd(struct nvme_controller *ctrlr,
    struct nvme_pt_command *pt, uint32_t nsid, int is_user_buffer,
    int is_admin_cmd)
{
	struct nvme_request	*req;
	struct mtx		*mtx;
	struct buf		*buf = NULL;
	int			ret = 0;
	vm_offset_t		addr, end;

	struct nvme_pci_controller *pctrlr;

	KASSERT_NVMP_CNTRLR(ctrlr);
	pctrlr = ctrlr->nvmec_tsp;

	CONFIRMPCIECONTROLLER;
	if (pt->len > 0) {
		/*
		 * vmapbuf calls vm_fault_quick_hold_pages which only maps full
		 * pages. Ensure this request has fewer than MAXPHYS bytes when
		 * extended to full pages.
		 */
		addr = (vm_offset_t)pt->buf;
		end = round_page(addr + pt->len);
		addr = trunc_page(addr);
		if (end - addr > MAXPHYS)
			return EIO;

		if (pt->len > ctrlr->max_xfer_size) {
			nvme_printf(&(pctrlr->ctrlr), "pt->len (%d) "
			    "exceeds max_xfer_size (%d)\n", pt->len,
			    ctrlr->max_xfer_size);
			return EIO;
		}
		if (is_user_buffer) {
			/*
			 * Ensure the user buffer is wired for the duration of
			 *  this passthrough command.
			 */
			PHOLD(curproc);
			buf = getpbuf(NULL);
			buf->b_data = pt->buf;
			buf->b_bufsize = pt->len;
			buf->b_iocmd = pt->is_read ? BIO_READ : BIO_WRITE;
#ifdef NVME_UNMAPPED_BIO_SUPPORT
			if (vmapbuf(buf, 1) < 0) {
#else
			if (vmapbuf(buf) < 0) {
#endif
				ret = EFAULT;
				goto err;
			}
			req = nvme_allocate_request_vaddr(buf->b_data, pt->len, 
			    nvme_pt_done, pt);
		} else
			req = nvme_allocate_request_vaddr(pt->buf, pt->len,
			    nvme_pt_done, pt);
	} else
		req = nvme_allocate_request_null(nvme_pt_done, pt);

	/* Assume userspace already converted to little-endian */
	req->cmd.opc = pt->cmd.opc;
	req->cmd.fuse = pt->cmd.fuse;
	req->cmd.cdw10 = pt->cmd.cdw10;
	req->cmd.cdw11 = pt->cmd.cdw11;
	req->cmd.cdw12 = pt->cmd.cdw12;
	req->cmd.cdw13 = pt->cmd.cdw13;
	req->cmd.cdw14 = pt->cmd.cdw14;
	req->cmd.cdw15 = pt->cmd.cdw15;

	req->cmd.nsid = htole32(nsid);

	mtx = mtx_pool_find(mtxpool_sleep, pt);
	pt->driver_lock = mtx;

	if (is_admin_cmd)
		nvme_ctrlr_submit_admin_request(ctrlr, req);
	else
		nvme_ctrlr_submit_io_request(ctrlr, req);

	mtx_lock(mtx);
	while (pt->driver_lock != NULL)
		mtx_sleep(pt, mtx, PRIBIO, "nvme_pt", 0);
	mtx_unlock(mtx);

err:
	if (buf != NULL) {
		relpbuf(buf, NULL);
		PRELE(curproc);
	}

	return (ret);
}

static int
nvme_ctrlr_ioctl(struct cdev *cdev, u_long cmd, caddr_t arg, int flag,
    struct thread *td)
{
	struct nvme_pci_controller			*pctrlr;
	struct nvme_pt_command			*pt;

	pctrlr = cdev->si_drv1;
	CONFIRMPCIECONTROLLER;

	switch (cmd) {
	case NVME_RESET_CONTROLLER:
		nvme_ctrlr_reset(pctrlr);
		break;
	case NVME_PASSTHROUGH_CMD:
		pt = (struct nvme_pt_command *)arg;
		return (nvme_ctrlr_passthrough_cmd(&pctrlr->ctrlr, pt, le32toh(pt->cmd.nsid),
		    1 /* is_user_buffer */, 1 /* is_admin_cmd */));
	default:
		return (ENOTTY);
	}

	return (0);
}

static struct cdevsw nvme_ctrlr_cdevsw = {
	.d_version =	D_VERSION,
	.d_flags =	0,
	.d_ioctl =	nvme_ctrlr_ioctl
};

static void
nvme_ctrlr_setup_interrupts(struct nvme_pci_controller *pctrlr)
{
	device_t	dev;
	int		per_cpu_io_queues;
	int		min_cpus_per_ioq;
	int		num_vectors_requested, num_vectors_allocated;
	int		num_vectors_available;

	CONFIRMPCIECONTROLLER;
	dev = pctrlr->dev;
	min_cpus_per_ioq = 1;
	TUNABLE_INT_FETCH("hw.nvme.min_cpus_per_ioq", &min_cpus_per_ioq);

	if (min_cpus_per_ioq < 1) {
		min_cpus_per_ioq = 1;
	} else if (min_cpus_per_ioq > mp_ncpus) {
		min_cpus_per_ioq = mp_ncpus;
	}

	per_cpu_io_queues = 1;
	TUNABLE_INT_FETCH("hw.nvme.per_cpu_io_queues", &per_cpu_io_queues);

	if (per_cpu_io_queues == 0) {
		min_cpus_per_ioq = mp_ncpus;
	}

	pctrlr->force_intx = 0;
	TUNABLE_INT_FETCH("hw.nvme.force_intx", &pctrlr->force_intx);

	/*
	 * FreeBSD currently cannot allocate more than about 190 vectors at
	 *  boot, meaning that systems with high core count and many devices
	 *  requesting per-CPU interrupt vectors will not get their full
	 *  allotment.  So first, try to allocate as many as we may need to
	 *  understand what is available, then immediately release them.
	 *  Then figure out how many of those we will actually use, based on
	 *  assigning an equal number of cores to each I/O queue.
	 */

	/* One vector for per core I/O queue, plus one vector for admin queue. */
	num_vectors_available = min(pci_msix_count(dev), mp_ncpus + 1);
	if (pci_alloc_msix(dev, &num_vectors_available) != 0) {
		num_vectors_available = 0;
	}
	pci_release_msi(dev);

	if (pctrlr->force_intx || num_vectors_available < 2) {
		nvme_ctrlr_configure_intx(pctrlr);
		return;
	}

	/*
	 * Do not use all vectors for I/O queues - one must be saved for the
	 *  admin queue.
	 */
	pctrlr->ctrlr.num_cpus_per_ioq = max(min_cpus_per_ioq,
	    howmany(mp_ncpus, num_vectors_available - 1));

	pctrlr->ctrlr.num_io_queues = howmany(mp_ncpus, pctrlr->ctrlr.num_cpus_per_ioq);
	num_vectors_requested = pctrlr->ctrlr.num_io_queues + 1;
	num_vectors_allocated = num_vectors_requested;

	/*
	 * Now just allocate the number of vectors we need.  This should
	 *  succeed, since we previously called pci_alloc_msix()
	 *  successfully returning at least this many vectors, but just to
	 *  be safe, if something goes wrong just revert to INTx.
	 */
	if (pci_alloc_msix(dev, &num_vectors_allocated) != 0) {
		nvme_ctrlr_configure_intx(pctrlr);
		return;
	}

	if (num_vectors_allocated < num_vectors_requested) {
		pci_release_msi(dev);
		nvme_ctrlr_configure_intx(pctrlr);
		return;
	}

	pctrlr->msix_enabled = 1;
}

int
nvme_ctrlr_construct(struct nvme_pci_controller *pctrlr, device_t dev)
{
	struct make_dev_args	md_args;
	uint32_t	cap_lo;
	uint32_t	cap_hi;
	uint8_t		to;
	uint8_t		dstrd;
	uint8_t		mpsmin;
	int		status, timeout_period;

	CONFIRMPCIECONTROLLER;
	pctrlr->dev = dev;

	mtx_init(&pctrlr->ctrlr.lockc, "nvme ctrlr lock", NULL, MTX_DEF);

	status = nvme_ctrlr_allocate_bar(pctrlr);

	if (status != 0)
		return (status);

	/*
	 * Software emulators may set the doorbell stride to something
	 *  other than zero, but this driver is not set up to handle that.
	 */
	cap_hi = nvme_mmio_read_4(pctrlr, cap_hi);
	dstrd = (cap_hi >> NVME_CAP_HI_REG_DSTRD_SHIFT) & NVME_CAP_HI_REG_DSTRD_MASK;
	if (dstrd != 0)
		return (ENXIO);

	mpsmin = (cap_hi >> NVME_CAP_HI_REG_MPSMIN_SHIFT) & NVME_CAP_HI_REG_MPSMIN_MASK;
	pctrlr->ctrlr.min_page_size = 1 << (12 + mpsmin);

	/* Get ready timeout value from controller, in units of 500ms. */
	cap_lo = nvme_mmio_read_4(pctrlr, cap_lo);
	to = (cap_lo >> NVME_CAP_LO_REG_TO_SHIFT) & NVME_CAP_LO_REG_TO_MASK;
	pctrlr->ctrlr.ready_timeout_in_ms = to * 500;

	timeout_period = NVME_DEFAULT_TIMEOUT_PERIOD;
	TUNABLE_INT_FETCH("hw.nvme.timeout_period", &timeout_period);
	timeout_period = min(timeout_period, NVME_MAX_TIMEOUT_PERIOD);
	timeout_period = max(timeout_period, NVME_MIN_TIMEOUT_PERIOD);
	pctrlr->ctrlr.timeout_period = timeout_period;

	nvme_retry_count = NVME_DEFAULT_RETRY_COUNT;
	TUNABLE_INT_FETCH("hw.nvme.retry_count", &nvme_retry_count);

	pctrlr->enable_aborts = 0;
	TUNABLE_INT_FETCH("hw.nvme.enable_aborts", &pctrlr->enable_aborts);

	nvme_ctrlr_setup_interrupts(pctrlr);

	pctrlr->ctrlr.max_xfer_size = NVME_MAX_XFER_SIZE;
	if (nvme_ctrlr_construct_admin_qpair(pctrlr) != 0)
		return (ENXIO);

	pctrlr->ctrlr.taskqueue = taskqueue_create("nvme_taskq", M_WAITOK,
	    taskqueue_thread_enqueue, &pctrlr->ctrlr.taskqueue);
	taskqueue_start_threads(&pctrlr->ctrlr.taskqueue, 1, PI_DISK, "nvme taskq");

	pctrlr->ctrlr.is_resetting = 0;
	pctrlr->ctrlr.is_initialized = 0;
	pctrlr->ctrlr.notification_sent = 0;
	TASK_INIT(&pctrlr->ctrlr.reset_task, 0, nvme_ctrlr_reset_task, pctrlr);
	TASK_INIT(&pctrlr->ctrlr.fail_req_task, 0, nvme_ctrlr_fail_req_task,
	    &pctrlr->ctrlr);
	STAILQ_INIT(&pctrlr->ctrlr.fail_req);
	pctrlr->ctrlr.is_failed = FALSE;
	pctrlr->ctrlr.nvmec_unit = device_get_unit(dev);

	make_dev_args_init(&md_args);
	md_args.mda_devsw = &nvme_ctrlr_cdevsw;
	md_args.mda_uid = UID_ROOT;
	md_args.mda_gid = GID_WHEEL;
	md_args.mda_mode = 0600;
	md_args.mda_unit = pctrlr->ctrlr.nvmec_unit;
	md_args.mda_si_drv1 = (void *)pctrlr;
	status = make_dev_s(&md_args, &pctrlr->ctrlr.ccdev, "nvme%d",
	     pctrlr->ctrlr.nvmec_unit);
	if (status != 0)
		return (ENXIO);

	return (0);
}

void
nvme_ctrlr_destruct(struct nvme_pci_controller *pctrlr, device_t dev)
{
	int				i;

	CONFIRMPCIECONTROLLER;
	if (pctrlr->resource == NULL)
		goto nores;

	nvme_notify_fail_consumers(pctrlr);

	for (i = 0; i < NVME_MAX_NAMESPACES; i++)
		nvme_ns_destruct(&pctrlr->ctrlr.cns[i]);

	if (pctrlr->ctrlr.ccdev)
		destroy_dev(pctrlr->ctrlr.ccdev);

	for (i = 0; i < pctrlr->ctrlr.num_io_queues; i++) {
		nvme_ctrlr_destroy_qpair(pctrlr, &pctrlr->ioq[i]);
		nvme_io_qpair_destroy(&pctrlr->ioq[i]);
	}
	free(pctrlr->ioq, M_NVME);

	nvme_admin_qpair_destroy(&pctrlr->adminq);

	/*
	 *  Notify the controller of a shutdown, even though this is due to
	 *   a driver unload, not a system shutdown (this path is not invoked
	 *   during shutdown).  This ensures the controller receives a
	 *   shutdown notification in case the system is shutdown before
	 *   reloading the driver.
	 */
	nvme_ctrlr_shutdown(pctrlr);

	nvme_ctrlr_disable(pctrlr);

	if (pctrlr->ctrlr.taskqueue)
		taskqueue_free(pctrlr->ctrlr.taskqueue);

	if (pctrlr->tag)
		bus_teardown_intr(pctrlr->dev, pctrlr->res, pctrlr->tag);

	if (pctrlr->res)
		bus_release_resource(pctrlr->dev, SYS_RES_IRQ,
		    rman_get_rid(pctrlr->res), pctrlr->res);

	if (pctrlr->msix_enabled)
		pci_release_msi(dev);

	if (pctrlr->bar4_resource != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY,
		    pctrlr->bar4_resource_id, pctrlr->bar4_resource);
	}

	bus_release_resource(dev, SYS_RES_MEMORY,
	    pctrlr->resource_id, pctrlr->resource);

nores:
	mtx_destroy(&pctrlr->ctrlr.lockc);
}

void
nvme_ctrlr_shutdown(struct nvme_pci_controller *pctrlr)
{
	uint32_t	cc;
	uint32_t	csts;
	int		ticks = 0;

	CONFIRMPCIECONTROLLER;
	cc = nvme_mmio_read_4(pctrlr, cc);
	cc &= ~(NVME_CC_REG_SHN_MASK << NVME_CC_REG_SHN_SHIFT);
	cc |= NVME_SHN_NORMAL << NVME_CC_REG_SHN_SHIFT;
	nvme_mmio_write_4(pctrlr, cc, cc);

	csts = nvme_mmio_read_4(pctrlr, csts);
	while ((NVME_CSTS_GET_SHST(csts) != NVME_SHST_COMPLETE) && (ticks++ < 5*hz)) {
		pause("nvme shn", 1);
		csts = nvme_mmio_read_4(pctrlr, csts);
	}
	if (NVME_CSTS_GET_SHST(csts) != NVME_SHST_COMPLETE)
		nvme_printf(&(pctrlr->ctrlr), "did not complete shutdown within 5 seconds "
		    "of notification\n");
}

void
nvmp_submit_adm_request(struct nvme_controller *ctrlr, struct nvme_request *req)
{
	struct nvme_pci_controller *pctrlr;

	KASSERT_NVMP_CNTRLR(ctrlr);
	pctrlr = ctrlr->nvmec_tsp;
	CONFIRMPCIECONTROLLER;
	nvme_qpair_submit_request(&pctrlr->adminq, req);
}

void
nvmp_submit_io_request(struct nvme_controller *ctrlr, struct nvme_request *req)
{
	struct nvme_pci_qpair       *qpair;
	struct nvme_pci_controller *pctrlr;

	KASSERT_NVMP_CNTRLR(ctrlr);
	pctrlr = ctrlr->nvmec_tsp;
	CONFIRMPCIECONTROLLER;
	qpair = &pctrlr->ioq[curcpu / pctrlr->ctrlr.num_cpus_per_ioq];
	nvme_qpair_submit_request(qpair, req);
}

device_t
nvme_ctrlr_get_device(struct nvme_pci_controller *pctrlr)
{

	CONFIRMPCIECONTROLLER;
	return (pctrlr->dev);
}

const struct nvme_controller_data *
nvme_ctrlr_get_data(struct nvme_pci_controller *pctrlr)
{

	CONFIRMPCIECONTROLLER;
	return (&pctrlr->ctrlr.cdata);
}
