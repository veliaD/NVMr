/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2019 Dell Inc. or its subsidiaries. All Rights Reserved.
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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/module.h>
#include <sys/lock.h>
#include <sys/sx.h>

#include <vm/uma.h>

#include "nvme_private.h"

struct nvme_consumer {
	uint32_t		id;
	nvme_cons_ns_fn_t	ns_fn;
	nvme_cons_ctrlr_fn_t	ctrlr_fn;
	nvme_cons_async_fn_t	async_fn;
	nvme_cons_fail_fn_t	fail_fn;
};

struct nvme_consumer nvme_consumer[NVME_MAX_CONSUMERS];
#define	INVALID_CONSUMER_ID	0xFFFF

uma_zone_t	nvme_request_zone;
int32_t		nvme_retry_count;

MALLOC_DEFINE(M_NVME, "nvme", "nvme(4) memory allocations");

devclass_t nvme_devclass;

static void
nvme_init(void)
{
	uint32_t	i;

	nvme_request_zone = uma_zcreate("nvme_request",
	    sizeof(struct nvme_request), NULL, NULL, NULL, NULL, 0, 0);

	for (i = 0; i < NVME_MAX_CONSUMERS; i++)
		nvme_consumer[i].id = INVALID_CONSUMER_ID;
}

SYSINIT(nvme_register, SI_SUB_DRIVERS, SI_ORDER_SECOND, nvme_init, NULL);

static void
nvme_uninit(void)
{
	uma_zdestroy(nvme_request_zone);
}

SYSUNINIT(nvme_unregister, SI_SUB_DRIVERS, SI_ORDER_SECOND, nvme_uninit, NULL);

/*
 * The list of NVMe controllers registered with the NVMe sub-system
 */
static STAILQ_HEAD(, nvme_controller) nclst_hd =
    STAILQ_HEAD_INITIALIZER(nclst_hd);
static struct sx nclst_lock;

int
nvme_shutdown(device_t dev)
{
	struct nvme_pci_controller	*pctrlr;

	pctrlr = DEVICE2SOFTC(dev);
	CONFIRMPCIECONTROLLER;
	nvme_ctrlr_shutdown(pctrlr);

	return (0);
}

void
nvme_dump_command(struct nvme_command *cmd)
{

	printf(
"opc:%x f:%x cid:%x nsid:%x r2:%x r3:%x mptr:%jx prp1:%jx prp2:%jx cdw:%x %x %x %x %x %x\n",
	    cmd->opc, cmd->fuse, cmd->cid, le32toh(cmd->nsid),
	    cmd->rsvd2, cmd->rsvd3,
	    (uintmax_t)le64toh(cmd->mptr), (uintmax_t)le64toh(cmd->prp1), (uintmax_t)le64toh(cmd->prp2),
	    le32toh(cmd->cdw10), le32toh(cmd->cdw11), le32toh(cmd->cdw12),
	    le32toh(cmd->cdw13), le32toh(cmd->cdw14), le32toh(cmd->cdw15));
}

void
nvme_dump_completion(struct nvme_completion *cpl)
{
	uint8_t p, sc, sct, m, dnr;
	uint16_t status;

	status = le16toh(cpl->status);

	p = NVME_STATUS_GET_P(status);
	sc = NVME_STATUS_GET_SC(status);
	sct = NVME_STATUS_GET_SCT(status);
	m = NVME_STATUS_GET_M(status);
	dnr = NVME_STATUS_GET_DNR(status);

	printf("cdw0:%08x sqhd:%04x sqid:%04x "
	    "cid:%04x p:%x sc:%02x sct:%x m:%x dnr:%x\n",
	    le32toh(cpl->cdw0), le16toh(cpl->sqhd), le16toh(cpl->sqid),
	    cpl->cid, p, sc, sct, m, dnr);
}

void
nvme_register_controller(struct nvme_controller *ctrlr)
{
	nvme_printf(ctrlr,"c:%p\n", ctrlr);
	sx_xlock(&nclst_lock);
	STAILQ_INSERT_TAIL(&nclst_hd, ctrlr, nvmec_lst);
	sx_xunlock(&nclst_lock);
}

void
nvme_unregister_controller(struct nvme_controller *ctrlr)
{
	nvme_printf(ctrlr,"c:%p\n", ctrlr);
	sx_xlock(&nclst_lock);
	STAILQ_REMOVE(&nclst_hd, ctrlr, nvme_controller, nvmec_lst);
	sx_xunlock(&nclst_lock);
}

static void
nvmp_delist_cb(struct nvme_controller *ctrlr)
{
	nvme_printf(ctrlr, "nvmp_delist_cb() invoked for ctrlr:%p\n", ctrlr);
	return;
}

int
nvme_attach(device_t dev)
{
	struct nvme_pci_controller	*pctrlr = DEVICE2SOFTC(dev);
	int			status;

	pctrlr->ctrlr.nvmec_tsp = pctrlr;
	pctrlr->ctrlr.nvmec_ttype = NVMET_PCIE;
	pctrlr->ctrlr.nvmec_delist = &nvmp_delist_cb;
	pctrlr->ctrlr.nvmec_subadmreq = &nvmp_submit_adm_request;
	pctrlr->ctrlr.nvmec_subioreq = &nvmp_submit_io_request;
	strncpy(pctrlr->very_first_field, NVMP_STRING, NVME_VFFSTRSZ);

	status = nvme_ctrlr_construct(pctrlr, dev);
	if (status != 0) {
		nvme_ctrlr_destruct(pctrlr, dev);
		return (status);
	}

	pctrlr->config_hook.ich_func = nvme_ctrlr_start_config_hook;
	pctrlr->config_hook.ich_arg = pctrlr;

	config_intrhook_establish(&pctrlr->config_hook);

	nvme_register_controller(&pctrlr->ctrlr);

	return (0);
}

int
nvme_detach (device_t dev)
{
	struct nvme_pci_controller	*pctrlr = DEVICE2SOFTC(dev);

	CONFIRMPCIECONTROLLER;
	nvme_unregister_controller(&pctrlr->ctrlr);

	nvme_ctrlr_destruct(pctrlr, dev);
	return (0);
}

static void
nvme_notify(struct nvme_consumer *cons,
	    struct nvme_controller *ctrlr)
{
	struct nvme_namespace	*ns;
	void			*ctrlr_cookie;
	int			cmpset, ns_idx;

	nvme_printf(ctrlr, "Notifying consumer for ctrlr:%p\n", ctrlr);
	/*
	 * The consumer may register itself after the nvme devices
	 *  have registered with the kernel, but before the
	 *  driver has completed initialization.  In that case,
	 *  return here, and when initialization completes, the
	 *  controller will make sure the consumer gets notified.
	 */
	if (!ctrlr->is_initialized) {
		nvme_printf(ctrlr, "Uninited:not registering controller:%p\n",
		    ctrlr);
		return;
	}

	cmpset = atomic_cmpset_32(&ctrlr->notification_sent, 0, 1);

	if (cmpset == 0) {
		nvme_printf(ctrlr, "Returning w/o registering controller:%u\n",
		    ctrlr->notification_sent);
		return;
	}

	if (cons->ctrlr_fn != NULL)
		ctrlr_cookie = (*cons->ctrlr_fn)(ctrlr);
	else
		ctrlr_cookie = (void *)(uintptr_t)0xdeadc0dedeadc0de;
	ctrlr->ccons_cookie[cons->id] = ctrlr_cookie;

	/* ctrlr_fn has failed.  Nothing to notify here any more. */
	if (ctrlr_cookie == NULL)
		return;

	if (NVME_IS_CTRLR_FAILED(ctrlr)) {
		ctrlr->ccons_cookie[cons->id] = NULL;
		if (cons->fail_fn != NULL)
			(*cons->fail_fn)(ctrlr_cookie);
		/*
		 * Do not notify consumers about the namespaces of a
		 *  failed controller.
		 */
		nvme_printf(ctrlr, "Returning w/o examing NSes controller\n");
		return;
	}
	for (ns_idx = 0; ns_idx < min(ctrlr->cdata.nn, NVME_MAX_NAMESPACES); ns_idx++) {
		nvme_printf(ctrlr, "Examing NS idx:%d\n", ns_idx);
		ns = &ctrlr->cns[ns_idx];
		nvme_printf(ctrlr, "\tsize:%lu\n", ns->data.nsze);
		if (ns->data.nsze == 0) {
			continue;
		}
		if (cons->ns_fn != NULL)
			ns->cons_cookie[cons->id] =
			    (*cons->ns_fn)(ns, ctrlr_cookie);
	}
}

void
nvme_notify_new_controller(struct nvme_controller *ctrlr)
{
	int i;

	for (i = 0; i < NVME_MAX_CONSUMERS; i++) {
		if (nvme_consumer[i].id != INVALID_CONSUMER_ID) {
			nvme_notify(&nvme_consumer[i], ctrlr);
		}
	}
}

static void
nvme_notify_new_consumer(struct nvme_consumer *cons)
{
	struct nvme_controller     *ctrlr;

	printf("NVMe: Notifying consumer\n");
	sx_slock(&nclst_lock);
	STAILQ_FOREACH(ctrlr, &nclst_hd, nvmec_lst) {
		nvme_notify(cons, ctrlr);
	}
	sx_sunlock(&nclst_lock);
}

void
nvme_notify_async_consumers(struct nvme_pci_controller *pctrlr,
			    const struct nvme_completion *async_cpl,
			    uint32_t log_page_id, void *log_page_buffer,
			    uint32_t log_page_size)
{
	struct nvme_consumer	*cons;
	void			*ctrlr_cookie;
	uint32_t		i;

	CONFIRMPCIECONTROLLER;
	for (i = 0; i < NVME_MAX_CONSUMERS; i++) {
		cons = &nvme_consumer[i];
		if (cons->id != INVALID_CONSUMER_ID && cons->async_fn != NULL &&
		    (ctrlr_cookie = pctrlr->ctrlr.ccons_cookie[i]) != NULL) {
			(*cons->async_fn)(pctrlr->ctrlr.ccons_cookie[i], async_cpl,
			    log_page_id, log_page_buffer, log_page_size);
		}
	}
}

void
nvme_notify_fail_consumers(struct nvme_controller *ctrlr)
{
	struct nvme_consumer	*cons;
	void			*ctrlr_cookie;
	uint32_t		i;

	/*
	 * This controller failed during initialization (i.e. IDENTIFY
	 *  command failed or timed out).  Do not notify any nvme
	 *  consumers of the failure here, since the consumer does not
	 *  even know about the controller yet.
	 */
	if (!ctrlr->is_initialized)
		return;

	for (i = 0; i < NVME_MAX_CONSUMERS; i++) {
		cons = &nvme_consumer[i];
		if (cons->id != INVALID_CONSUMER_ID &&
		    (ctrlr_cookie = ctrlr->ccons_cookie[i]) != NULL) {
			ctrlr->ccons_cookie[i] = NULL;
			if (cons->fail_fn != NULL)
				cons->fail_fn(ctrlr_cookie);
		}
	}
}

void
nvme_notify_ns(struct nvme_pci_controller *pctrlr, int nsid)
{
	struct nvme_consumer	*cons;
	struct nvme_namespace	*ns = &pctrlr->ctrlr.cns[nsid - 1];
	void			*ctrlr_cookie;
	uint32_t		i;

	if (!pctrlr->ctrlr.is_initialized)
		return;

	CONFIRMPCIECONTROLLER;
	for (i = 0; i < NVME_MAX_CONSUMERS; i++) {
		cons = &nvme_consumer[i];
		if (cons->id != INVALID_CONSUMER_ID && cons->ns_fn != NULL &&
		    (ctrlr_cookie = pctrlr->ctrlr.ccons_cookie[cons->id]) != NULL)
			ns->cons_cookie[i] = (*cons->ns_fn)(ns, ctrlr_cookie);
	}
}

struct nvme_consumer *
nvme_register_consumer(nvme_cons_ns_fn_t ns_fn, nvme_cons_ctrlr_fn_t ctrlr_fn,
		       nvme_cons_async_fn_t async_fn,
		       nvme_cons_fail_fn_t fail_fn)
{
	int i;

	/*
	 * TODO: add locking around consumer registration.
	 */
	for (i = 0; i < NVME_MAX_CONSUMERS; i++)
		if (nvme_consumer[i].id == INVALID_CONSUMER_ID) {
			nvme_consumer[i].id = i;
			nvme_consumer[i].ns_fn = ns_fn;
			nvme_consumer[i].ctrlr_fn = ctrlr_fn;
			nvme_consumer[i].async_fn = async_fn;
			nvme_consumer[i].fail_fn = fail_fn;

			nvme_notify_new_consumer(&nvme_consumer[i]);
			return (&nvme_consumer[i]);
		}

	printf("nvme(4): consumer not registered - no slots available\n");
	return (NULL);
}

void
nvme_unregister_consumer(struct nvme_consumer *consumer)
{

	consumer->id = INVALID_CONSUMER_ID;
}

void
nvme_completion_poll_cb(void *arg1, void *arg2, const struct nvme_completion *cpl)
{
	struct nvme_completion_poll_status	*status = arg1;

	/*
	 * Copy status into the argument passed by the caller, so that
	 *  the caller can check the status to determine if the
	 *  the request passed or failed.
	 */
	memcpy(&status->cpl, cpl, sizeof(*cpl));
	atomic_store_rel_int(&status->done, 1);
}

static int
nvme_modevent(module_t mod __unused, int type __unused, void *argp __unused)
{
	switch (type) {
	case MOD_LOAD:
		sx_init(&nclst_lock, "NVMe controllers list lock");
		break;
	default:
		break;
	}

	return (0);
}

static moduledata_t nvme_mod = {
       "nvme",
       nvme_modevent,
       0
};

DECLARE_MODULE(nvme, nvme_mod, SI_SUB_DRIVERS, SI_ORDER_FIRST);
MODULE_VERSION(nvme, 1);
MODULE_DEPEND(nvme, cam, 1, 1, 1);
