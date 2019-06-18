/*
 * Copyright (c) 2019 Dell Inc. or its subsidiaries. All Rights Reserved.
 */

#include <dev/nvmrdma/nvmr_spec.h>
#include <dev/nvmrdma/nvmrdma.h>

static void
nvmr_qphndlr(struct ib_event *ev, void *ctx)
{
	ERRSPEW("Event \"%s\" on QP:%p\n", ib_event_msg(ev->event), ctx);
}


char nvrdma_host_uuid_string[80];
struct uuid nvrdma_host_uuid;

static void
nvmr_rg_done(struct ib_cq *cq, struct ib_wc *wc)
{
	struct ib_cqe *rcve;

	rcve = wc->wr_cqe;

	/*
	DBGSPEW("rcve:%p, wc_status:\"%s\"\n", rcve,
	    ib_wc_status_msg(wc->status));
	 */
}

static void
nvmr_snd_done(struct ib_cq *cq, struct ib_wc *wc)
{
	struct ib_cqe *rcve;

	rcve = wc->wr_cqe;

	/*
	DBGSPEW("rcve:%p, wc_status:\"%s\"\n", rcve,
	    ib_wc_status_msg(wc->status));
	 */
}

static char *nvmr_qndxnames[NVMR_NUM_QTYPES] = {
	[NVMR_QTYPE_ADMIN] = "Admin Q",
	[NVMR_QTYPE_IO] = "I/O Q",
};

static inline char *
nvmr_qndx2name(nvmr_qndx_t qndx)
{
	KASSERT(qndx < NVMR_NUM_QTYPES, ("qndx:%d", qndx));
	return nvmr_qndxnames[qndx];
}

static MALLOC_DEFINE(M_NVMR, "nvmr", "nvmr");

int nvmr_command_async(nvmr_qpair_t q, struct nvme_request *req);
void
nvmr_ctrlr_post_failed_request(nvmr_cntrlr_t cntrlr, struct nvme_request *req);

void nvmr_cleanup_q_next(nvmr_qpair_t q, struct nvmr_ncommcont *commp);
void
nvmr_cleanup_q_next(nvmr_qpair_t q, struct nvmr_ncommcont *commp)
{
	int retval;
	struct nvme_request *req;

	mtx_lock(&q->nvmrq_gqp.qlock);

	atomic_store_rel_int(&commp->nvmrsnd_state, NVMRSND_FREE);
	STAILQ_INSERT_HEAD(&q->nvmrq_comms, commp, nvmrsnd_nextfree);
	q->nvmrq_numFsndqe++;

	retval = EDOOFUS;
	while (retval != 0) {
		req = STAILQ_FIRST(&q->nvmrq_defreqs);
		if (req == NULL) {
			break;
		}

		STAILQ_REMOVE(&q->nvmrq_defreqs, req, nvme_request, stailq);
		retval = nvmr_command_async(q, req);
		if (retval != 0) {
			mtx_unlock(&q->nvmrq_gqp.qlock);
			ERRSPEW("Failing req: %d\n", retval);
			nvmr_ctrlr_post_failed_request(q->nvmrq_cntrlr, req);
			mtx_lock(&q->nvmrq_gqp.qlock);
		}
	}

	mtx_unlock(&q->nvmrq_gqp.qlock);
}


static void
nvmr_localinv_done(struct ib_cq *cq, struct ib_wc *wc)
{
	struct nvmr_ncommcont *commp;
	nvmr_qpair_t q;

	q = cq->cq_context;
	commp = container_of(wc->wr_cqe, struct nvmr_ncommcont, nvmrsnd_regcqe);

	if (wc->status != IB_WC_SUCCESS) {
		ERRSPEW("Local rKey invalidation failed q:%p commp:%p r:%x\n",
		    q, commp, commp->nvmrsnd_mr->rkey);
		/* Bail for now, wake waiters */
	}

	nvmr_cleanup_q_next(q, commp);
}


#define NVMR_INCR_Q_USECNT(q)                                            \
    /* DBGSPEW("q:%p o:%u\n", q, q->nvmrq_usecnt), */                    \
    atomic_fetchadd_int(&(q)->nvmrq_usecnt, 1)

#define NVMR_DECR_Q_USECNT(q)                                            \
    /* DBGSPEW("q:%p o:%u\n", q, q->nvmrq_usecnt), */                    \
    atomic_fetchadd_int(&(q)->nvmrq_usecnt, -1)

#define NVMR_IS_CNTRLR_FAILED(c)                                         \
    NVME_IS_CTRLR_FAILED(&(c)->nvmrctr_nvmec)

static inline void
nvmr_drop_q_usecount(nvmr_qpair_t q)
{
	uint32_t		ousecnt;

	ousecnt = NVMR_DECR_Q_USECNT(q);
	if (NVMR_IS_CNTRLR_FAILED(q->nvmrq_cntrlr) && (ousecnt == 1)) {
		wakeup(__DEVOLATILE(void *, &q->nvmrq_usecnt));
	}
}

void nvmr_nreq_compl(nvmr_qpair_t q, struct nvmr_ncommcont *commp, struct nvme_completion *c);
/**********
 TODO:
 1) Retry logic
 2) Track in-use comm
 **********/
void
nvmr_nreq_compl(nvmr_qpair_t q, struct nvmr_ncommcont *commp, struct nvme_completion *c)
{
	struct nvme_request *req;
	boolean_t retry, error;

	req = commp->nvmrsnd_req;
	error = nvme_completion_is_error(c);
	retry = error && nvme_completion_is_retry(c); /* Need to check count */

	if (error) {
		nvme_qpair_print_command(&q->nvmrq_gqp, &req->cmd);
		nvme_qpair_print_completion(&q->nvmrq_gqp, c);
	}

	if (retry) {
		ERRSPEW("Not retrying: unimplemented yet! c:%p\n", commp);
	}
	
	if (req->cb_fn) {
		req->cb_fn(req->cb_arg, c);
	}

	commp->nvmrsnd_req = NULL;
	nvme_free_request(req);

	/*
	 * The assumption is that at this point the consumer layer has no more
	 * state associated with this IO
	 */
	nvmr_drop_q_usecount(q);
}

void nvmr_qfail_freecmpl(nvmr_qpair_t q, struct nvmr_ncmplcont *cmplp);
void
nvmr_qfail_freecmpl(nvmr_qpair_t q, struct nvmr_ncmplcont *cmplp)
{
	mtx_lock(&q->nvmrq_gqp.qlock);
	STAILQ_INSERT_HEAD(&q->nvmrq_cmpls, cmplp, nvmrsp_next);
	q->nvmrq_numFrcvqe++;
	if (q->nvmrq_numFrcvqe == q->nvmrq_numrcvqe) {
		wakeup(&q->nvmrq_numFrcvqe);
	}
	mtx_unlock(&q->nvmrq_gqp.qlock);
}


static void
nvmr_recv_cmplhndlr(struct ib_cq *cq, struct ib_wc *wc);

static int
nvmr_post_cmpl(nvmr_qpair_t q, struct nvmr_ncmplcont *cmplp)
{
	struct nvme_completion *ncmp;
	struct ib_recv_wr rcvwr, *badrcvwrp;
	struct ib_sge sgl;
	struct ib_device *ibd;
	int retval;
	struct ib_pd *ibpd;

	if (Q_IS_FAILED(q)) {
		retval = ESHUTDOWN;
		DBGSPEW("Q:%p has failed, freeing %p\n", q, cmplp);
		nvmr_qfail_freecmpl(q, cmplp);
		goto out;
	}

	ibd = q->nvmrq_cmid->device;
	ibpd = q->nvmrq_ibpd;

	ncmp = &cmplp->nvmrsp_nvmecmpl;
	memset(&rcvwr, 0, sizeof(rcvwr));
	memset(&sgl, 0, sizeof(sgl));

	sgl.addr   = cmplp->nvmrsp_dmaddr;
	sgl.length = sizeof(*ncmp);
	sgl.lkey   = ibpd->local_dma_lkey;

	cmplp->nvmrsp_cqe.done = nvmr_recv_cmplhndlr;

	rcvwr.sg_list = &sgl;
	rcvwr.num_sge = 1;
	rcvwr.wr_cqe  = &cmplp->nvmrsp_cqe;
	rcvwr.next    = NULL;

	ib_dma_sync_single_for_device(ibd, cmplp->nvmrsp_dmaddr,
	    sizeof(cmplp->nvmrsp_nvmecmpl), DMA_FROM_DEVICE);
	retval = ib_post_recv(q->nvmrq_ibqp, &rcvwr, &badrcvwrp);

out:
	return retval;
}


static void
nvmr_recv_cmplhndlr(struct ib_cq *cq, struct ib_wc *wc)
{
	struct nvmr_ncmplcont *cmplp;
	struct nvmr_ncommcont *commp;
	struct nvme_completion *c;
	struct ib_send_wr invwr, *badinvwrp;
	nvmr_qpair_t q;
	int retval;

	q = cq->cq_context;
	cmplp = container_of(wc->wr_cqe, struct nvmr_ncmplcont, nvmrsp_cqe);

	switch(wc->status) {
	case IB_WC_SUCCESS:
	case IB_WC_WR_FLUSH_ERR:
		break;
	default:
		ERRSPEW("cmplp:%p, wc_status:\"%s\"\n", cmplp,
		    ib_wc_status_msg(wc->status));
		break;
	}

	ib_dma_sync_single_for_cpu(q->nvmrq_cmid->device, cmplp->nvmrsp_dmaddr,
	    sizeof cmplp->nvmrsp_nvmecmpl, DMA_FROM_DEVICE);

	if (wc->status == IB_WC_WR_FLUSH_ERR) {
		nvmr_qfail_freecmpl(q, cmplp);
		goto out_no_repost;
	}

	if (wc->status != IB_WC_SUCCESS) {
		goto out;
	}

	c = &cmplp->nvmrsp_nvmecmpl;

	if ((c->cid == 0) || (c->cid > q->nvmrq_numsndqe)) {
		ERRSPEW("Returned CID in NVMe completion is not valid "
		    "cid:%hu max:%hu\n", c->cid, q->nvmrq_numsndqe);
		goto out;
	}

	commp = q->nvmrq_commcid[c->cid];
	atomic_store_rel_int(&commp->nvmrsnd_state, NVMRSND_CMPL);
	callout_stop(&commp->nvmrsnd_to);
	nvme_completion_swapbytes(c);

	KASSERT(commp->nvmrsnd_req != NULL, ("%s@%d c:%p r:%p", __func__,
	    __LINE__, commp, commp->nvmrsnd_req));
	nvmr_nreq_compl(q, commp, c);

	/*
	 * The NVMeoF spec allows the host to ask the target to send over an
	 * rkey(remote-key)-invalidate-request after it has RDMAed into/from
	 * the host memory.  This improves latency I suppose but is optional
	 */
	if (wc->wc_flags & IB_WC_WITH_INVALIDATE) {
		/*
		 * The target sent over an NVMe completion along with a request
		 * to our RDMA stack to invalidate the rkey we setup for the
		 * corresponding NVMe command.  Confirm the keys match.
		 */
		if (commp->nvmrsnd_mr->rkey != wc->ex.invalidate_rkey) {
			ERRSPEW("Key's don't match! q:%p commp:%p cid:%d "
			    "wkey:%x okey:%x\n", q, commp, c->cid,
			    wc->ex.invalidate_rkey, commp->nvmrsnd_mr->rkey);

			/* Bail for now, wake waiters */
		}
	} else if (commp->nvmrsnd_rkeyvalid) {
		/*
		 * We didn't request invalidation of our RDMA rkey by the
		 * target or it was ignored.  Invalidate the rkey ourselves.
		 */
		memset(&invwr, 0, sizeof(invwr));
		invwr.num_sge = 0;
		invwr.next = NULL;
		invwr.opcode = IB_WR_LOCAL_INV;
		invwr.ex.invalidate_rkey = commp->nvmrsnd_mr->rkey;
		invwr.send_flags = IB_SEND_SIGNALED;
		invwr.wr_cqe = &commp->nvmrsnd_regcqe;
		commp->nvmrsnd_regcqe.done = nvmr_localinv_done;

		retval = ib_post_send(q->nvmrq_ibqp, &invwr, &badinvwrp);
		if (retval != 0) {
			ERRSPEW("Local Invalidate failed! commp:%p cid:%d\n",
			    commp, c->cid);
			/* Bail for now, wake waiters */
		} else {
			/* Continue in nvmr_localinv_done() */
			goto out;
		}
	}

	nvmr_cleanup_q_next(q, commp);

out:
	nvmr_post_cmpl(q, cmplp);

out_no_repost:
	return;
}

void nvmr_qfail_drain(nvmr_qpair_t q, uint16_t *counter, uint16_t count);
void
nvmr_qfail_drain(nvmr_qpair_t q, uint16_t *counter, uint16_t count)
{
	KASSERT(Q_IS_FAILED(q), ("q:%p is still enabled!\n", q));

	KASSERT((counter == &q->nvmrq_numFsndqe) ||
	    (counter == &q->nvmrq_numFrcvqe), ("%s@%d %p is neither of q:%p\n",
	    __func__, __LINE__, counter, q));

	mtx_lock(&q->nvmrq_gqp.qlock);
	while(*counter != count) {
		DBGSPEW("Sleeping for q:%p field:%p to drain\n", q, counter);
		mtx_sleep(counter, &q->nvmrq_gqp.qlock, 0, "qdrain", HZ);
	}
	mtx_unlock(&q->nvmrq_gqp.qlock);
}


static void
nvmr_qpair_destroy(nvmr_qpair_t q)
{
	nvmr_ncmplcon_t *cmplp, *tcmplp;
	nvmr_ncommcon_t *commp, *tcommp;
	struct ib_device *ibd;
	int count;

	if (q == NULL) {
		goto out;
	}

	atomic_store_rel_int(&q->nvmrq_gqp.qis_enabled, FALSE);

	ibd = q->nvmrq_cmid->device;

	if (q->nvmrq_state >= NVMRQ_CONNECT_SUCCEEDED) {
		DBGSPEW("Invoking rdma_disconnect(%p)...\n", q->nvmrq_cmid);
		rdma_disconnect(q->nvmrq_cmid);
	}
	if (q->nvmrq_ibqp != NULL) {
		DBGSPEW("Invoking ib_drain_qp(%p)...\n", q->nvmrq_ibqp);
		ib_drain_qp(q->nvmrq_ibqp);

		DBGSPEW("Invoking rdma_destroy_qp(%p)..\n", q->nvmrq_cmid);
		rdma_destroy_qp(q->nvmrq_cmid);
		q->nvmrq_ibqp = NULL;
	}

	if (q->nvmrq_ibcq != NULL) {
		DBGSPEW("Invoking ib_free_cq(%p)...\n", q->nvmrq_ibcq);
		ib_free_cq(q->nvmrq_ibcq);
		q->nvmrq_ibcq = NULL;
	}

	while (q->nvmrq_usecnt != 0) {
		ERRSPEW("Waiting for q:%p usecount to drop to 0, %d\n", q,
		    q->nvmrq_usecnt);
		tsleep(__DEVOLATILE(void *, &q->nvmrq_usecnt), 0, "qpzero",
		    2 * HZ);
	}

	nvmr_qfail_drain(q, &q->nvmrq_numFrcvqe, q->nvmrq_numrcvqe);
	KASSERT(q->nvmrq_numFrcvqe == q->nvmrq_numrcvqe, ("%s@%d q:%p\n",
	    __func__, __LINE__, q));

	/*
	 * Loop through the list of completion container structures unmapping
	 * the NVMe completion structure and then free'ing the containers
	 * themselves
	 */
	count = 0;
	STAILQ_FOREACH_SAFE(cmplp, &q->nvmrq_cmpls, nvmrsp_next, tcmplp) {
		if (cmplp->nvmrsp_dmaddr != 0) {
			ib_dma_sync_single_for_cpu(ibd, cmplp->nvmrsp_dmaddr,
			    sizeof(cmplp->nvmrsp_nvmecmpl), DMA_FROM_DEVICE);
			ib_dma_unmap_single(ibd, cmplp->nvmrsp_dmaddr,
			    sizeof(cmplp->nvmrsp_nvmecmpl), DMA_FROM_DEVICE);
		}
		free(cmplp, M_NVMR);
		count++;
	}
	KASSERT(count == q->nvmrq_numrcvqe, ("%s@%d count:%d numrcvqe:%d",
	    __func__, __LINE__, count, q->nvmrq_numrcvqe));
	DBGSPEW("Freed %d completion containers\n", q->nvmrq_numrcvqe);

	free(q->nvmrq_commcid, M_NVMR);

	nvmr_qfail_drain(q, &q->nvmrq_numFsndqe, q->nvmrq_numsndqe);
	KASSERT(q->nvmrq_numFsndqe == q->nvmrq_numsndqe, ("%s@%d q:%p\n",
	    __func__, __LINE__, q));

	count = 0;
	STAILQ_FOREACH_SAFE(commp, &q->nvmrq_comms, nvmrsnd_nextfree, tcommp) {
		STAILQ_REMOVE(&q->nvmrq_comms, commp, nvmr_ncommcont,
		    nvmrsnd_nextfree);
		q->nvmrq_numFsndqe--;
		if (commp->nvmrsnd_mr != NULL) {
			ib_dereg_mr(commp->nvmrsnd_mr);
		}
		if (commp->nvmrsnd_dmaddr != 0) {
			ib_dma_sync_single_for_cpu(ibd, commp->nvmrsnd_dmaddr,
			    sizeof(*commp->nvmrsnd_nvmecomm), DMA_FROM_DEVICE);
			ib_dma_unmap_single(ibd, commp->nvmrsnd_dmaddr,
			    sizeof(*commp->nvmrsnd_nvmecomm), DMA_FROM_DEVICE);
		}
		free(commp, M_NVMR);
		count++;
	}
	KASSERT(count == q->nvmrq_numsndqe, ("%s@%d count:%d numsndqe:%d",
	    __func__, __LINE__, count, q->nvmrq_numsndqe));
	DBGSPEW("Freed %d command containers\n", q->nvmrq_numsndqe);

	if (q->nvmrq_ibpd != NULL) {
		DBGSPEW("ib_dealloc_pd(%p)...\n", q->nvmrq_ibpd);
		ib_dealloc_pd(q->nvmrq_ibpd);
		q->nvmrq_ibpd = NULL;
	}

	if (q->nvmrq_cmid != NULL) {
		DBGSPEW("rdma_destroy_id(%p)...\n", q->nvmrq_cmid);
		rdma_destroy_id(q->nvmrq_cmid);
		q->nvmrq_cmid = NULL;
	}

	DBGSPEW("free(%p)ing Q\n", q);
	free(q, M_NVMR);

out:
	return;
}


/* Tracks the number of NVMr controllers */
int nvmr_cntrlrs_count;

TASKQUEUE_DEFINE_THREAD(nvmr_cntrlrs_reaper);

void nvmr_destroy_cntrlrs(void *arg, int pending);
struct task nvmr_destroy_cntrlrs_task =
    TASK_INITIALIZER(0, nvmr_destroy_cntrlrs, NULL);


TAILQ_HEAD(, nvmr_cntrlr_tag) nvmr_cntrlrs_active =
    TAILQ_HEAD_INITIALIZER(nvmr_cntrlrs_active);

TAILQ_HEAD(, nvmr_cntrlr_tag) nvmr_condemned_cntrlrs =
    TAILQ_HEAD_INITIALIZER(nvmr_condemned_cntrlrs);

/* Protects the two lists above, as well as nvmr_cntrlrs_count */
struct mtx nvmr_cntrlrs_lstslock;
MTX_SYSINIT(nvmr_cntrlrs_lstslock, &nvmr_cntrlrs_lstslock,
    "NVMr Lists of Controllers Lock", MTX_DEF);


/*
 * Atomically, looks at the state of the controller and switches it to the
 * condemned state while deciding to enqueue right away or later
 */
nvmr_condemned_disposition_t nvmr_cntrlr_mark_condemned(nvmr_cntrlr_t cntrlr);
nvmr_condemned_disposition_t
nvmr_cntrlr_mark_condemned(nvmr_cntrlr_t cntrlr)
{
	nvmr_condemned_disposition_t retval;

	mtx_assert(&cntrlr->nvmrctr_nvmec.lockc, MA_OWNED);

	/*
	 * Mark early so IOs can be failed out early.  The necessary location
	 * for this invocation is just before queueing onto the condemned
	 * queue
	 */
	NVME_SET_CTRLR_FAILED(&cntrlr->nvmrctr_nvmec);

	switch(cntrlr->nvmrctr_state) {
	case NVMRC_PRE_INIT:
	case NVMRC_INITED:
		atomic_store_rel_int(&cntrlr->nvmrctr_state, NVMRC_CONDEMNED);
		switch (cntrlr->nvmrctr_state) {
		case NVMRC_PRE_INIT:
			retval = NVMRCD_SKIP_ENQUEUE;
			break;
		default:
			retval = NVMRCD_ENQUEUE;
			break;
		}
		break;

	case NVMRC_CONDEMNED:
		retval = NVMRCD_SKIP_ENQUEUE;
		break;
	}

	return retval;
}


/*
 * Routine that blindly adds a controller to the condemned queue for destruction
 */
void nvmr_cntrlr_condemn_enqueue(nvmr_cntrlr_t cntrlr);
void
nvmr_cntrlr_condemn_enqueue(nvmr_cntrlr_t cntrlr)
{
	boolean_t queue_task;
	DBGSPEW("Queueing controller:%p for destruction\n", cntrlr);

	queue_task = FALSE;

	mtx_lock(&nvmr_cntrlrs_lstslock);
	if (cntrlr->nvmrctr_glblst != NVMR_CNTRLRLST_CONDEMNED) {
		KASSERT(cntrlr->nvmrctr_glblst == NVMR_CNTRLRLST_ACTIVE,
		    ("%s@%d c:%p type:%d\n", __func__, __LINE__, cntrlr,
		    cntrlr->nvmrctr_glblst));
		TAILQ_REMOVE(&nvmr_cntrlrs_active, cntrlr, nvmrctr_nxt);

		NVME_SET_CTRLR_FAILED(&cntrlr->nvmrctr_nvmec);
		/*
		 * Wait for IOs to have either 1) Seen that the controller
		 * is failed OR 2) Bumped up a reference count on a queue-pair
		 */
		epoch_wait(global_epoch);

		TAILQ_INSERT_TAIL(&nvmr_condemned_cntrlrs, cntrlr, nvmrctr_nxt);
		cntrlr->nvmrctr_glblst = NVMR_CNTRLRLST_CONDEMNED;
		queue_task = TRUE;

		DBGSPEW("ctrlr:%p queued for destruction\n", cntrlr);
	}
	mtx_unlock(&nvmr_cntrlrs_lstslock);

	if (queue_task == TRUE) {
		taskqueue_enqueue(taskqueue_nvmr_cntrlrs_reaper,
		    &nvmr_destroy_cntrlrs_task);
	}
}


/*
 * High-level routine invoked when something bad has been detected on the
 * controller and it needs to be destroyed.  It might enqueue for immediate
 * destruction or not based on the current state.
 */
void nvmr_cntrlr_condemn_init(nvmr_cntrlr_t cntrlr);
void
nvmr_cntrlr_condemn_init(nvmr_cntrlr_t cntrlr)
{
	nvmr_condemned_disposition_t retval;

	DBGSPEW("Condemning controller:%p\n", cntrlr);
	mtx_lock(&cntrlr->nvmrctr_nvmec.lockc);
	retval = nvmr_cntrlr_mark_condemned(cntrlr);
	mtx_unlock(&cntrlr->nvmrctr_nvmec.lockc);

	if (retval == NVMRCD_ENQUEUE) {
		nvmr_cntrlr_condemn_enqueue(cntrlr);
	}
}


/*
 * Moves a controller from the pre-init state to the initialized state unless
 * it has been condemned
 */
void nvmr_cntrlr_inited(nvmr_cntrlr_t cntrlr);
void
nvmr_cntrlr_inited(nvmr_cntrlr_t cntrlr)
{
	mtx_lock(&cntrlr->nvmrctr_nvmec.lockc);
	switch(cntrlr->nvmrctr_state) {
	case NVMRC_PRE_INIT:
		atomic_store_rel_int(&cntrlr->nvmrctr_state, NVMRC_INITED);
		break;
	case NVMRC_INITED:
		panic("%s@%d Controller:%p already marked initialized!\n",
		    __func__, __LINE__, cntrlr);
		break;
	case NVMRC_CONDEMNED:
		break;
	}
	mtx_unlock(&cntrlr->nvmrctr_nvmec.lockc);
}

/*
 * Run through the list of active controllers searching for the controller
 * with the unitnum passed in.  When/if found the controller is marked
 * condemned and optionally queued for destruction to the reaper taskqueue
 *
 * The routine can be invoked with NVMR_ALLUNITNUMS to do the above for all
 * active controllers.
 */
void nvmr_cntrlrs_condemn_init(int unitnum);
void
nvmr_cntrlrs_condemn_init(int unitnum)
{
	boolean_t queue_task;
	nvmr_cntrlr_t cntrlr, tcntrlr;
	nvmr_condemned_disposition_t retval;

	queue_task = FALSE;

	mtx_lock(&nvmr_cntrlrs_lstslock);
	TAILQ_FOREACH_SAFE(cntrlr, &nvmr_cntrlrs_active, nvmrctr_nxt, tcntrlr) {
		if ((unitnum != NVMR_ALLUNITNUMS) && (unitnum !=
		    cntrlr->nvmrctr_nvmec.nvmec_unit)) {
			continue;
		}
		mtx_lock(&cntrlr->nvmrctr_nvmec.lockc);
		retval = nvmr_cntrlr_mark_condemned(cntrlr);
		mtx_unlock(&cntrlr->nvmrctr_nvmec.lockc);

		if ((retval == NVMRCD_SKIP_ENQUEUE) ||
		    (cntrlr->nvmrctr_glblst == NVMR_CNTRLRLST_CONDEMNED)) {
			continue;
		}

		KASSERT(cntrlr->nvmrctr_glblst == NVMR_CNTRLRLST_ACTIVE,
		    ("%s@%d c:%p type:%d\n", __func__, __LINE__, cntrlr,
		    cntrlr->nvmrctr_glblst));
		TAILQ_REMOVE(&nvmr_cntrlrs_active, cntrlr, nvmrctr_nxt);

		NVME_SET_CTRLR_FAILED(&cntrlr->nvmrctr_nvmec);
		/*
		 * Wait for IOs to have either 1) Seen that the controller
		 * is failed OR 2) Bumped up a reference count on a queue-pair
		 */
		epoch_wait(global_epoch);

		TAILQ_INSERT_TAIL(&nvmr_condemned_cntrlrs, cntrlr, nvmrctr_nxt);
		cntrlr->nvmrctr_glblst = NVMR_CNTRLRLST_CONDEMNED;
		DBGSPEW("Queued ctrlr:%p for destruction\n", cntrlr);
		queue_task = TRUE;
	}
	mtx_unlock(&nvmr_cntrlrs_lstslock);

	if (queue_task == TRUE) {
		taskqueue_enqueue(taskqueue_nvmr_cntrlrs_reaper,
		    &nvmr_destroy_cntrlrs_task);
	}
}

void nvmr_all_cntrlrs_condemn_init(void);
void
nvmr_all_cntrlrs_condemn_init(void)
{
	nvmr_cntrlrs_condemn_init(NVMR_ALLUNITNUMS);
}


static void
nvmr_cntrlr_destroy(nvmr_cntrlr_t cntrlr)
{
	int count;

	DBGSPEW("Controller:%p being destroyed\n", cntrlr);


	if (cntrlr->nvmrctr_nvmereg) {
		nvme_unregister_controller(&cntrlr->nvmrctr_nvmec);
	}

	for (count = 0; count < NVME_MAX_NAMESPACES; count++) {
		nvme_ns_destruct(&cntrlr->nvmrctr_nvmec.cns[count]);
	}

	if (cntrlr->nvmrctr_nvmec.ccdev) {
		destroy_dev(cntrlr->nvmrctr_nvmec.ccdev);
	}

	if (cntrlr->nvmrctr_ioqarr != NULL) {
		for (count = 0; count < cntrlr->nvmrctr_numioqs; count++) {
			nvmr_qpair_destroy(cntrlr->nvmrctr_ioqarr[count]);
			cntrlr->nvmrctr_ioqarr[count] = NULL;
		}
		DBGSPEW("free(%p)ing Q array\n", cntrlr->nvmrctr_ioqarr);
		free(cntrlr->nvmrctr_ioqarr, M_NVMR);
	}
	nvmr_qpair_destroy(cntrlr->nvmrctr_adminqp);

	if (cntrlr->nvmrctr_nvmec.taskqueue) {
		taskqueue_drain_all(cntrlr->nvmrctr_nvmec.taskqueue);
		taskqueue_free(cntrlr->nvmrctr_nvmec.taskqueue);
	}

	nvme_notify_fail_consumers(&cntrlr->nvmrctr_nvmec);

	DBGSPEW("NVMr controller:%p wiped\n", cntrlr);

	return;
}


void
nvmr_destroy_cntrlrs(void *arg, int pending)
{
	nvmr_cntrlr_t cntrlr;
	int count;

	DBGSPEW("Controller destruction task woken up\n");
	count = 0;
	while (cntrlr = TAILQ_FIRST(&nvmr_condemned_cntrlrs), cntrlr != NULL) {
		count++;
		nvmr_cntrlr_destroy(cntrlr);

		mtx_lock(&nvmr_cntrlrs_lstslock);
		TAILQ_REMOVE(&nvmr_condemned_cntrlrs, cntrlr, nvmrctr_nxt);
		nvmr_cntrlrs_count--;
		if (nvmr_cntrlrs_count == 0) {
			DBGSPEW("nvmr_cntrlrs_count is 0\n");
			wakeup(&nvmr_cntrlrs_count);
		}
		mtx_unlock(&nvmr_cntrlrs_lstslock);

		DBGSPEW("free(%p)ing NVMr controller\n", cntrlr);
		free(cntrlr, M_NVMR);
	}
	DBGSPEW("%d controllers destroyed\n", count);
}


static int
nvmr_connmgmt_handler(struct rdma_cm_id *cmid, struct rdma_cm_event *event)
{
	nvmr_qpair_t q;
	nvmr_qpair_state_t qstate;
	const nvmr_rdma_cm_reject_t *ps;

	DBGSPEW("Event \"%s\" returned status \"%d\" for cmid:%p\n",
	    rdma_event_msg(event->event), event->status, cmid);

	/* Every cmid is associated with a controller or a queue? */
	q = cmid->context;
	q->nvmrq_last_cm_status = event->status;

	switch(event->event) {
	case RDMA_CM_EVENT_ADDR_RESOLVED:
	case RDMA_CM_EVENT_ADDR_ERROR:
	case RDMA_CM_EVENT_ROUTE_RESOLVED:
	case RDMA_CM_EVENT_ROUTE_ERROR:
	case RDMA_CM_EVENT_ESTABLISHED:
	case RDMA_CM_EVENT_UNREACHABLE:
	case RDMA_CM_EVENT_REJECTED:
		switch(event->event) {
		case RDMA_CM_EVENT_ADDR_RESOLVED:
			qstate = NVMRQ_ADDR_RESOLV_SUCCEEDED;
			break;

		case RDMA_CM_EVENT_ADDR_ERROR:
			qstate = NVMRQ_ADDR_RESOLV_FAILED;
			break;

		case RDMA_CM_EVENT_ROUTE_RESOLVED:
			qstate = NVMRQ_ROUTE_RESOLV_SUCCEEDED;
			break;

		case RDMA_CM_EVENT_ROUTE_ERROR:
			qstate = NVMRQ_ROUTE_RESOLV_FAILED;
			break;

		case RDMA_CM_EVENT_ESTABLISHED:
			qstate = NVMRQ_CONNECT_SUCCEEDED;
			break;

		case RDMA_CM_EVENT_UNREACHABLE:
			qstate = NVMRQ_CONNECT_FAILED;
			break;

		case RDMA_CM_EVENT_REJECTED:
			ps = (const nvmr_rdma_cm_reject_t *)
			    event->param.conn.private_data;
			DBGSPEW("Reject reason recfmt:%hu sts:%hu\n",
			    ps->nvmrcrj_recfmt, ps->nvmrcrj_sts);
			qstate = NVMRQ_CONNECT_FAILED;
			break;
		default:
			panic("%s@%d: Unhandled Conn Manager event Q:%p e:%d\n",
			    __func__, __LINE__, q, event->event);
		}
		mtx_lock(&q->nvmrq_cntrlr->nvmrctr_nvmec.lockc);
		q->nvmrq_state = qstate;
		mtx_unlock(&q->nvmrq_cntrlr->nvmrctr_nvmec.lockc);
		wakeup(&q->nvmrq_cmid);
		/* No touching q beyond this point */
		break;
	case RDMA_CM_EVENT_DISCONNECTED:
	case RDMA_CM_EVENT_DEVICE_REMOVAL:
		nvmr_cntrlr_condemn_init(q->nvmrq_cntrlr);
		break;

	default:
		dump_stack();
		break;
	}

	/* No touching q beyond this point */
	if (event->status != 0) {
		dump_stack();
	}

	return 0;
}

/*
 * TODO: Unify into nvme_ctrlr_post_failed_request(): It's the same
 */
void
nvmr_ctrlr_post_failed_request(nvmr_cntrlr_t cntrlr, struct nvme_request *req)
{
	struct nvme_controller *ctrlr;

	CONFIRMRDMACONTROLLER;
	ctrlr = &cntrlr->nvmrctr_nvmec;

	mtx_lock(&ctrlr->lockc);
	STAILQ_INSERT_TAIL(&ctrlr->fail_req, req, stailq);
	mtx_unlock(&ctrlr->lockc);
	taskqueue_enqueue(ctrlr->taskqueue, &ctrlr->fail_req_task);
}


/*
 *    Based on iser_bio_to_sg()
 */
int
nvmr_map_bio(nvmr_qpair_t q, nvmr_ncommcon_t *commp, int *np);
int
nvmr_map_bio(nvmr_qpair_t q, nvmr_ncommcon_t *commp, int *np)
{
	uintptr_t offset;
	int n, error;
	size_t len, translen;
	struct scatterlist *s, *scl;
	struct bio *bio;

	mtx_assert(&q->nvmrq_gqp.qlock, MA_OWNED);

	*np = 0;
	bio = commp->nvmrsnd_req->u.bio;
	translen = bio->bio_bcount;
	if (translen == 0) {
		error = ENOTSUPP;
		ERRSPEW("0 length NVMe request data not supported\n");
		goto out;
	}
	offset = bio->bio_ma_offset;
	scl = q->nvmrq_scl;

	for (n = 0; (0 < translen) && (n < MAX_NVME_RDMA_SEGMENTS); n++,
	    translen -= len) {
		memset(scl + n, 0, sizeof(*scl));
		s = scl + n;
		len = min(PAGE_SIZE - offset, translen);
		sg_set_page(s, bio->bio_ma[n], len, offset);
		offset = 0;
	}

	if (translen != 0) {
		ERRSPEW("Could not complete translation of Data. n:%d "
		    "translen:%zu\n", n, translen);
		error = E2BIG;
		goto out;
	} else {
		/*
		DBGSPEW("data is 0x%p\n", commp->nvmrsnd_req->u.payload);
		for (count = 0; count < n; count++) {
			DBGSPEW("scl[%d](hex) p:%16lX o:%8X l:%8X a:%16lX\n",
			count,
			scl[count].page_link,
			scl[count].offset,
			scl[count].length,
			scl[count].address);
		}
		 */
	}
	sg_mark_end(s);

	*np = n;
	error = 0;

out:
	return error;
}

/*
 *    Based on iser_buf_to_sg()
 */
int
nvmr_map_vaddr(nvmr_qpair_t q, nvmr_ncommcon_t *commp, int *np);
int
nvmr_map_vaddr(nvmr_qpair_t q, nvmr_ncommcon_t *commp, int *np)
{
	uintptr_t offset;
	int n, error;
	size_t len, translen;
	void *cbuf;
	struct scatterlist *s, *scl;
	uint32_t type;

	mtx_assert(&q->nvmrq_gqp.qlock, MA_OWNED);

	*np = 0;
	scl = q->nvmrq_scl;
	type = commp->nvmrsnd_req->type;

	switch (type) {
	case NVME_REQUEST_BIO:
		translen = commp->nvmrsnd_req->u.bio->bio_bcount;
		cbuf = commp->nvmrsnd_req->u.bio->bio_data;
		break;
	case NVME_REQUEST_VADDR:
		translen = commp->nvmrsnd_req->payload_size;
		cbuf = commp->nvmrsnd_req->u.payload;
		break;
	default:
		panic("%s@%d Unsupported NVMe request type:%d c:%p\n", __func__,
		    __LINE__, type, commp);
		break;
	}

	if (cbuf == NULL) {
		error = EINVAL;
		ERRSPEW("NULL NVMe request data not supported, t:%u\n", type);
		goto out;
	}
	if (translen == 0) {
		error = ENOTSUPP;
		ERRSPEW("0 length NVMe request data not supported\n");
		goto out;
	}

	for (n = 0; (0 < translen) && (n < MAX_NVME_RDMA_SEGMENTS); n++,
	    translen -= len) {
		memset(scl + n, 0, sizeof(*scl));
		s = scl + n;
		offset = ((uintptr_t)cbuf) & ~PAGE_MASK;
		len = min(PAGE_SIZE - offset, translen);
		sg_set_buf(s, cbuf, len);
		cbuf = (void *)(((u64)cbuf) + (u64)len);
	}

	if (translen != 0) {
		ERRSPEW("Could not complete translation of Data. n:%d "
		    "translen:%zu\n", n, translen);
		error = E2BIG;
		goto out;
	} else {
		/*
		DBGSPEW("data is 0x%p\n", commp->nvmrsnd_req->u.payload);
		for (count = 0; count < n; count++) {
			DBGSPEW("scl[%d](hex) p:%16lX o:%8X l:%8X a:%16lX\n",
			count,
			scl[count].page_link,
			scl[count].offset,
			scl[count].length,
			scl[count].address);
		}
		 */
	}
	sg_mark_end(s);

	*np = n;
	error = 0;

out:
	return error;
}

int
nvmr_map_data(nvmr_qpair_t q, nvmr_ncommcon_t *commp, nvmr_ksgl_t *k,
    struct ib_reg_wr *regwrp);
int
nvmr_map_data(nvmr_qpair_t q, nvmr_ncommcon_t *commp, nvmr_ksgl_t *k,
    struct ib_reg_wr *regwrp)
{
	int n, nn, nnn, error, retval;
	struct ib_mr *mr;
	struct scatterlist *scl;
	enum dma_data_direction dir;
	struct ib_device *ibd;
	uint8_t dirbits;
	nvmr_communion_t *cmd;
	uint16_t bf;

	mtx_assert(&q->nvmrq_gqp.qlock, MA_OWNED);

	KASSERT(commp->nvmrsnd_req->type != NVME_REQUEST_NULL, ("%s@%d r:%p "
	    "t:%d\n", __func__, __LINE__, commp->nvmrsnd_req,
	    commp->nvmrsnd_req->type));

	ibd = q->nvmrq_cmid->device;
	/*
	 * 1) Translate the command data into a scatterlist array
	 *    for ib_dma_map_sg() to use a la iser_buf_to_sg()
	 */
	switch (commp->nvmrsnd_req->type) {
	case NVME_REQUEST_VADDR:
		retval = nvmr_map_vaddr(q, commp, &n);
		break;
	case NVME_REQUEST_BIO:
		bf = commp->nvmrsnd_req->u.bio->bio_flags;
		switch(bf & (BIO_VLIST|BIO_UNMAPPED)) {
		case 0:
			retval = nvmr_map_vaddr(q, commp, &n);
			break;
		case BIO_UNMAPPED:
			retval = nvmr_map_bio(q, commp, &n);
			break;
		case BIO_VLIST:
		default:
			retval = ENOTSUPP;
			break;

		}
		break;
	default:
		panic("%s@%d Unsupported NVMe req memory type r:%p t:0x%x",
		    __func__, __LINE__, commp->nvmrsnd_req,
		    commp->nvmrsnd_req->type);
	}
	if (retval != 0) {
		error = retval;
		ERRSPEW("nvmr_map_(q:%p commp:%p) failed:%d\n", q, commp,
		    retval);
		goto out;
	}

	scl = q->nvmrq_scl; /* Now contains RDMA segments */

	/* 2) Map the scatterlist array per the direction to/from IB device */
	cmd = (nvmr_communion_t *)&commp->nvmrsnd_req->cmd;
	if (cmd->nvmrcu_stub.nvmrsb_nvmf.nvmf_opc == NVME_OPC_FABRIC_COMMAND) {
		dirbits = cmd->nvmrcu_stub.nvmrsb_nvmf.nvmf_fctype;
	} else {
		dirbits = cmd->nvmrcu_stub.nvmrsb_nvmf.nvmf_opc;
	}
	dirbits &= 0x3;
	switch(dirbits) {
	case 0x1:
		dir = DMA_TO_DEVICE;
		break;
	case 0x2:
		dir = DMA_FROM_DEVICE;
		break;
	default:
		panic("%s@%d Bad direction in NVMe command r:%p d:0x%x",
		    __func__, __LINE__, commp->nvmrsnd_req, dirbits);
	}

	nn = ib_dma_map_sg(ibd, scl, n, dir);
	if (nn < 1) {
		ERRSPEW("ib_dma_map_sg() failed with count:%d\n", nn);
		error = E2BIG;
		goto out;
	}
	/* DBGSPEW("ib_dma_map_sg() returned a count of %d\n", nn); */

	/* 3) Map the scatterlist elements to the MR */
	mr = commp->nvmrsnd_mr;
	nnn = ib_map_mr_sg(mr, scl, nn, NULL, NVMR_FOURK);
	if (nnn < nn) {
		ERRSPEW("ib_map_mr_sg() failed. nnn:%d < nn:%d\n", nnn, nn);
		error = E2BIG;
		goto out;
	}
	/* DBGSPEW("ib_map_mr_sg() returned a count of %d\n", nnn); */
	ib_update_fast_reg_key(mr, ib_inc_rkey(mr->rkey));

	/* 4) Craft the memory registration work-request but don't post it */
	commp->nvmrsnd_regcqe.done = nvmr_rg_done;
	memset(regwrp, 0, sizeof(*regwrp));
	/* NB the Registration work-request contains a Send work-request */
	regwrp->wr.num_sge = 0; /* No send/recv buffers are being posted */
	regwrp->wr.send_flags = IB_SEND_SIGNALED; /* Invoke .done when done */
	regwrp->wr.opcode = IB_WR_REG_MR;
	regwrp->wr.wr_cqe = &commp->nvmrsnd_regcqe;
	regwrp->wr.next = NULL;
	regwrp->access = (dir == DMA_TO_DEVICE) ?
	    IB_ACCESS_REMOTE_READ : IB_ACCESS_REMOTE_WRITE;
	regwrp->access |= IB_ACCESS_LOCAL_WRITE;
	regwrp->key = mr->rkey;
	regwrp->mr = mr;

	k->nvmrk_address = htole64(mr->iova);
	k->nvmrk_length[0] = htole32(mr->length) & 0xFF;
	k->nvmrk_length[1] = (htole32(mr->length)>>8) & 0xFF;
	k->nvmrk_length[2] = (htole32(mr->length)>>16) & 0xFF;
	k->nvmrk_key = htole32(mr->rkey);
	k->nvmrk_sgl_identifier = NVMF_KEYED_SGL_INVALIDATE;

	commp->nvmrsnd_rkeyvalid = true;
	
	error = 0;

out:
	return error;
}


void nvmr_timeout(void *arg);
void
nvmr_timeout(void *arg)
{
	nvmr_qpair_t q;
	struct nvme_qpair *gq;
	nvmr_ncommcon_t *commp;
	nvmr_cntrlr_t cntrlr;
	struct nvme_request *req;

	commp = (nvmr_ncommcon_t *)arg;
	req = commp->nvmrsnd_req;
	ERRSPEW("c:%p r:%p timed out!\n", commp, req);

	q = commp->nvmrsnd_q;
	cntrlr = q->nvmrq_cntrlr;

	nvmr_ctrlr_post_failed_request(cntrlr, req);

	gq = &q->nvmrq_gqp;
	mtx_lock(&gq->qlock);
	STAILQ_INSERT_HEAD(&q->nvmrq_comms, commp, nvmrsnd_nextfree);
	q->nvmrq_numFsndqe++;
	mtx_unlock(&gq->qlock);

	ERRSPEW("Failing controller c:%p of q:%p!\n", cntrlr, q);
	nvmr_cntrlr_condemn_init(cntrlr);
}


int
nvmr_command_async(nvmr_qpair_t q, struct nvme_request *req)
{
	nvmr_stub_t *cp;
	nvmr_ksgl_t *k;
	nvmr_ncommcon_t *commp;
	struct nvme_qpair *gq;
	nvmr_cntrlr_t cntrlr;
	int error, retval;
	struct ib_reg_wr regwr;
	u64 dmaddr;
	struct ib_sge sgl;
	struct ib_send_wr sndwr, *sndwrp, *badsndwrp;
	struct ib_device *ibd;

	gq = &q->nvmrq_gqp;
	cntrlr = q->nvmrq_cntrlr;

	mtx_assert(&gq->qlock, MA_OWNED);

	switch(req->type) {
	case NVME_REQUEST_BIO:
	case NVME_REQUEST_VADDR:
	case NVME_REQUEST_NULL:
		break;
	default:
		commp = NULL;
		error = ENOTSUP;
		goto out;
	}

	commp = STAILQ_FIRST(&q->nvmrq_comms);
	if (commp == NULL) {
		if (NVMR_IS_CNTRLR_FAILED(cntrlr)) {
			ERRSPEW("Controller:%p:%p has failed\n", cntrlr,
			    &cntrlr->nvmrctr_nvmec);
			error = ESHUTDOWN;
		} else {
			STAILQ_INSERT_TAIL(&q->nvmrq_defreqs, req, stailq);
			error = 0;
		}
		goto out;
	}
	STAILQ_REMOVE(&q->nvmrq_comms, commp, nvmr_ncommcont, nvmrsnd_nextfree);
	q->nvmrq_numFsndqe--;

	cp = (nvmr_stub_t *)&req->cmd;
	ibd = q->nvmrq_cmid->device;
	k = &cp->nvmrsb_nvmf.nvmf_ksgl;

	cp->nvmrsb_nvmf.nvmf_sgl_fuse = NVMF_SINGLE_BUF_SGL;
	memset(&sndwr, 0, sizeof(sndwr));
	sndwr.next  = NULL;

	commp->nvmrsnd_rkeyvalid = false;
	commp->nvmrsnd_nvmecomm = cp;
	commp->nvmrsnd_req = req;

	memset(k, 0, sizeof(*k));
	switch(req->type) {
	case NVME_REQUEST_BIO:
	case NVME_REQUEST_VADDR:
		retval = nvmr_map_data(q, commp, k, &regwr);
		if (retval != 0) {
			error = retval;
			ERRSPEW("nvmr_map_data(c:%p q:%p):%d\n", commp,
			    q, retval);
			goto out;
		}
		break;
	case NVME_REQUEST_NULL:
		k->nvmrk_sgl_identifier = NVMF_KEYED_SGL_NO_INVALIDATE;
		break;
	default:
		panic("%s@%d Unimplemented t:%d req:%p\n", __func__, __LINE__,
		    req->type, req);
	}


	cp->nvmrsb_nvmf.nvmf_cid = commp->nvmrsnd_cid;
	dmaddr = ib_dma_map_single(ibd, cp, sizeof(*cp), DMA_TO_DEVICE);
	if (ib_dma_mapping_error(ibd, dmaddr) != 0) {
		ERRSPEW("ib_dma_map_single() failed for %p\n", cp);
		error = ENOENT;
		goto out;
	}
	commp->nvmrsnd_dmaddr = dmaddr;
	commp->nvmrsnd_rspndd = false;
	/* Transfer ownership of command structure to device */
	ib_dma_sync_single_for_device(ibd, dmaddr, sizeof(*cp), DMA_TO_DEVICE);

	memset(&sgl, 0, sizeof(sgl));

	sgl.addr   = dmaddr;
	sgl.length = sizeof(*(commp->nvmrsnd_nvmecomm));
	sgl.lkey   = q->nvmrq_ibpd->local_dma_lkey;

	commp->nvmrsnd_cqe.done = nvmr_snd_done;

	sndwr.wr_cqe = &commp->nvmrsnd_cqe;
	sndwr.sg_list = &sgl;
	sndwr.num_sge = 1;
	sndwr.opcode = IB_WR_SEND;
	sndwr.send_flags = IB_SEND_SIGNALED;
	sndwr.next = NULL;

	if (req->type == NVME_REQUEST_NULL) {
		sndwrp = &sndwr;
	} else {
		/* If there's data to be RDMAed chain in the registration */
		regwr.wr.next = &sndwr;
		sndwrp = &regwr.wr;
	}

	badsndwrp = NULL;
	retval = ib_post_send(q->nvmrq_ibqp, sndwrp, &badsndwrp);
	if (retval != 0) {
		ERRSPEW("ib_post_send(%p) failed with %d, badsndwrp:%p\n",
		    sndwrp, retval, badsndwrp);
		error = retval;
		goto out;
	}

	error = 0;
out:
	if (commp != NULL) {
		if (error != 0) {
			STAILQ_INSERT_HEAD(&q->nvmrq_comms, commp,
			    nvmrsnd_nextfree);
			q->nvmrq_numFsndqe++;
		} else {
			callout_reset_curcpu(&commp->nvmrsnd_to,
			    cntrlr->nvmrctr_nvmec.timeout_period * hz,
			    nvmr_timeout, commp);
			atomic_store_rel_int(&commp->nvmrsnd_state,
			NVMRSND_IOQD);
		}
	}

	return error;
}


void
nvmr_submit_req(nvmr_qpair_t q, struct nvme_request *req);
void
nvmr_submit_req(nvmr_qpair_t q, struct nvme_request *req)
{
	nvmr_cntrlr_t cntrlr;
	struct nvme_qpair *gqp;
	int retval;

	cntrlr = q->nvmrq_cntrlr;
	gqp = &q->nvmrq_gqp;
	req->rqpair = gqp;

	if (NVMR_IS_CNTRLR_FAILED(cntrlr)) {
		retval = ESHUTDOWN;
	} else {
		mtx_lock(&gqp->qlock);
		retval = nvmr_command_async(q, req);
		mtx_unlock(&gqp->qlock);
	}

	if (retval != 0) {
		ERRSPEW("Failing req:%p %d\n", req, retval);
		nvmr_ctrlr_post_failed_request(cntrlr, req);
	}

	return;
}

#define NVMR_SUBMIT_BEGIN                                                \
	nvmr_cntrlr_t cntrlr;                                            \
	nvmr_qpair_t q;                                                  \
                                                                         \
	KASSERT_NVMR_CNTRLR(ctrlr);                                      \
	cntrlr = ctrlr->nvmec_tsp;                                       \
	CONFIRMRDMACONTROLLER;                                           \
                                                                         \


#define NVMR_SUBMIT_END                                                  \
	NVMR_INCR_Q_USECNT(q);                                           \
	epoch_exit(global_epoch);                                        \
	nvmr_submit_req(q, req)

void
nvmr_submit_adm_req(struct nvme_controller *ctrlr, struct nvme_request *req);
void
nvmr_submit_adm_req(struct nvme_controller *ctrlr, struct nvme_request *req)
{
	NVMR_SUBMIT_BEGIN;
	q = cntrlr->nvmrctr_adminqp;
	NVMR_SUBMIT_END;
}


void
nvmr_submit_io_req(struct nvme_controller *ctrlr, struct nvme_request *req);
void
nvmr_submit_io_req(struct nvme_controller *ctrlr, struct nvme_request *req)
{
	NVMR_SUBMIT_BEGIN;
	q = cntrlr->nvmrctr_ioqarr[0]; /* 1 Q until IO qpairs are up */
	NVMR_SUBMIT_END;
}


#define PRE_ASYNC_CM_INVOCATION(pre_state) \
	q->nvmrq_state = (pre_state);

#define POST_ASYNC_CM_INVOCATION(routine, pre_state, success_state)        \
	if (retval != 0) {                                                 \
		ERRSPEW("Failed, %s()> %d\n", routine, retval);            \
		error = retval;                                            \
		goto out;                                                  \
	}                                                                  \
	/* DBGSPEW("Successfully invoked %s()\n", routine); */             \
	mtx_lock(&cntrlr->nvmrctr_nvmec.lockc);                            \
	if (q->nvmrq_state == (pre_state)) {                               \
		DBGSPEW("Sleeping with message \"%s\"\n",                  \
		    __stringify(__LINE__));                                \
		retval = mtx_sleep(&q->nvmrq_cmid,                         \
		    &cntrlr->nvmrctr_nvmec.lockc,                          \
		    0, __stringify(__LINE__), NVMRTO+1000);                \
		mtx_unlock(&cntrlr->nvmrctr_nvmec.lockc);                  \
		switch (retval) {                                          \
		case 0:                                                    \
			break;                                             \
		case EWOULDBLOCK:                                          \
			ERRSPEW("No response after %d ms\n",  NVMRTO+1000);\
		default:                                                   \
			error = retval;                                    \
			goto out;                                          \
		}                                                          \
	} else {                                                           \
		mtx_unlock(&cntrlr->nvmrctr_nvmec.lockc);                  \
	}                                                                  \
	if (q->nvmrq_state < (success_state)) {                            \
		error = q->nvmrq_last_cm_status;                           \
		goto out;                                                  \
	}                                                                  \

#define ISSUE_WAIT_CHECK_REQ                                                  \
	status.done = 0;                                                      \
	epoch_enter(global_epoch);                                            \
	if (!NVMR_IS_CNTRLR_FAILED(cntrlr)) {                                 \
		nvme_ctrlr_submit_admin_request(&cntrlr->nvmrctr_nvmec, req); \
		while (!atomic_load_acq_int(&status.done)) {                  \
			pause("nvmr", HZ > 100 ? (HZ/100) : 1);               \
		}                                                             \
	} else {                                                              \
		status.cpl.status =                                           \
		    (NVME_SCT_GENERIC << NVME_STATUS_SCT_SHIFT) |             \
		    (NVME_SC_ABORTED_BY_REQUEST << NVME_STATUS_SC_SHIFT);     \
		epoch_exit(global_epoch);                                     \
	}                                                                     \
	if (nvme_completion_is_error(&status.cpl))


static int
nvmr_qpair_create(nvmr_cntrlr_t cntrlr, nvmr_qpair_t *qp, uint16_t qid,
    uint16_t cntlid, uint32_t numsndqe, uint32_t numrcvqe, uint32_t kato)
{
	u64 dmaddr;
	struct ib_mr *mr;
	struct ib_device *ibd;
	struct sockaddr_storage saddr;
	struct sockaddr_in *sin4;
	struct rdma_cm_id *cmid;
	int error, retval, count;
	nvmr_ncmplcon_t *cmplp;
	nvmr_ncommcon_t *commp, **commparrp;
	struct ib_pd *ibpd;
	nvmr_qpair_t q;
	struct ib_qp_init_attr init_attr;
	struct rdma_conn_param conn_param;
	nvmr_rdma_cm_request_t privdata;
	struct ib_cq *ibcq;
	struct nvmrdma_connect_data ncdata;
	size_t msz;

	nvmr_communion_t *cmd;
	struct nvme_request *req;
	struct nvme_completion_poll_status status;


	sin4 = (struct sockaddr_in *)&saddr;

	msz = sizeof *q;
	q = malloc(msz, M_NVMR, M_WAITOK|M_ZERO);
	if (q == NULL) {
		ERRSPEW("NVMr Q allocation sized \"%zu\" failed\n", msz);
		error = ENOMEM;
		goto out;
	}
	DBGSPEW("NVMr Q allocation of size \"%zu\"\n", msz);
	q->nvmrq_cntrlr = cntrlr;
	q->nvmrq_state = NVMRQ_PRE_INIT;
	q->nvmrq_gqp.qttype = NVMET_RDMA;
	q->nvmrq_gqp.gqctrlr = &cntrlr->nvmrctr_nvmec;
	atomic_store_rel_int(&q->nvmrq_gqp.qis_enabled, FALSE);
	mtx_init(&q->nvmrq_gqp.qlock, "nvme qpair lock", NULL, MTX_DEF);
	STAILQ_INIT(&q->nvmrq_defreqs);
	STAILQ_INIT(&q->nvmrq_comms);
	STAILQ_INIT(&q->nvmrq_cmpls);

	cmid = rdma_create_id(TD_TO_VNET(curthread), nvmr_connmgmt_handler,
	    q, RDMA_PS_TCP, IB_QPT_RC);
	if (IS_ERR(cmid)) {
		ERRSPEW("rdma_create_id() failed:%ld\n", PTR_ERR(cmid));
		error = EINVAL;
		goto out;
	}
	q->nvmrq_cmid = cmid;

	/*
	 * Allocate containers for the Send Q elements which are always
	 * struct nvme_command (or fabric equivalent)
	 */
	for (count = 0; count < numsndqe; count++) {
		msz = sizeof(*commp);
		commp = malloc(msz,  M_NVMR, M_WAITOK|M_ZERO);
		if (commp == NULL) {
			ERRSPEW("Command Q container allocation failed after"
			    " %d iterations\n", count);
			break;
		}

		STAILQ_INSERT_HEAD(&q->nvmrq_comms, commp, nvmrsnd_nextfree);
		q->nvmrq_numFsndqe++;
		q->nvmrq_numsndqe++;
	}
	DBGSPEW("Alloced %d command Q containers\n", q->nvmrq_numsndqe);
	q->nvmrq_gqp.num_qentries = q->nvmrq_numsndqe;

	/*
	 * Allocate a mapping array for looking up an NVMe command when we get
	 * an NVMe completion
	 */

	msz = sizeof(*commparrp) * (q->nvmrq_numsndqe + 1);
	commparrp = malloc(msz, M_NVMR, M_WAITOK|M_ZERO);
	if (commparrp == NULL) {
		ERRSPEW("NVMr allocation of %zu for mapping array failed \n",
		    msz);
		error = ENOMEM;
		goto out;
	}
	DBGSPEW("NVMr allocation of %zu bytes for mapping array\n", msz);
	commp = STAILQ_FIRST(&q->nvmrq_comms);
	commparrp[0] = (nvmr_ncommcon_t *)(0xDEADD00D8BADBEEFull);
	for (count = 1; count < (q->nvmrq_numsndqe + 1); count++) {
		commparrp[count] = commp;
		commp->nvmrsnd_cid = count;
		commp->nvmrsnd_q = q;
		callout_init(&commp->nvmrsnd_to, 1);
		atomic_store_rel_int(&commp->nvmrsnd_state, NVMRSND_FREE);
		commp = STAILQ_NEXT(commp, nvmrsnd_nextfree);
	}
	KASSERT(commp == NULL, ("%s@%d commp:%p, q:%p\n", __func__, __LINE__,
	    commp, q));
	q->nvmrq_commcid = commparrp;

	/*
	 * Allocate containers for the Recv Q elements which are always
	 * struct nvme_completion
	 */
	for (count = 0; count < numrcvqe; count++) {
		msz = sizeof(*cmplp);
		cmplp = malloc(msz,  M_NVMR, M_WAITOK|M_ZERO);
		if (cmplp == NULL) {
			ERRSPEW("Completion Q container allocation failed after"
			    " %d iterations\n", count);
			break;
		}

		STAILQ_INSERT_HEAD(&q->nvmrq_cmpls, cmplp, nvmrsp_next);
		q->nvmrq_numFrcvqe++;
		q->nvmrq_numrcvqe++;
	}
	if (q->nvmrq_numrcvqe < q->nvmrq_gqp.num_qentries) {
		ERRSPEW("Only allocated %hu of %u required, failing\n",
		    q->nvmrq_numrcvqe, q->nvmrq_gqp.num_qentries);
		error = ENOMEM;
		goto out;
	}


	memset(&saddr, 0, sizeof(saddr));
	sin4->sin_len = sizeof(*sin4);
	sin4->sin_family = AF_INET;
	memcpy((void *)&sin4->sin_addr.s_addr, &cntrlr->nvmrctr_ipv4,
	    sizeof sin4->sin_addr.s_addr);
	sin4->sin_port = cntrlr->nvmrctr_port;

	/*
	 * NB Once rdma_resolve_addr() is called nvmr_connmgmt_handler() can be
	 * invoked.  Keep cntrlr consistent as it can be reached via cmid
	 * asynchronously.  Bump up the reference count for the activity in
	 * the nvmr_connmgmt_handler() contexts.  Assign the allocated q
	 * structure into the qarr so that cleaning up the controller structure
	 * will clean up the q as well.  nvmr_qpair_destroy() can no longer be
	 * called except by nvmr_cntrlr_destroy()
	 */
	*qp = q;
	PRE_ASYNC_CM_INVOCATION(NVMRQ_PRE_ADDR_RESOLV);
	retval = rdma_resolve_addr(cmid, NULL, (struct sockaddr *)sin4, NVMRTO);
	POST_ASYNC_CM_INVOCATION(__stringify(rdma_resolve_addr),
	    NVMRQ_PRE_ADDR_RESOLV, NVMRQ_ADDR_RESOLV_SUCCEEDED);

	/*
	 * Once address resoltion is complete the cmid will have the IB device
	 * that our RDMA connection will be using
	 */
	ibd = q->nvmrq_cmid->device;
	if (ibd == NULL) {
		error = EDOOFUS;
		goto out;
	}

	if (!(ibd->attrs.device_cap_flags & IB_DEVICE_MEM_MGT_EXTENSIONS)) {
		ERRSPEW("Memory management extensions not supported. 0x%lX\n",
		    cmid->device->attrs.device_cap_flags);
		error = ENXIO;
		goto out;
	}

	ibpd  = ib_alloc_pd(ibd, 0);
	if (IS_ERR(ibpd)) {
		ERRSPEW("ib_alloc_pd() failed: 0x%lx\n", PTR_ERR(ibpd));
		error = ENOENT;
		goto out;
	}
	q->nvmrq_ibpd = ibpd;

	/*
	 * Allocate MR structures for use by any data that the NVMe commands
	 * posted to the IB SND Q need to describe
	 */
	count = 0;
	STAILQ_FOREACH(commp, &q->nvmrq_comms, nvmrsnd_nextfree) {
		mr = ib_alloc_mr(ibpd, IB_MR_TYPE_MEM_REG,
		    MAX_NVME_RDMA_SEGMENTS);
		if (IS_ERR(mr)) {
			ERRSPEW("ib_alloc_mr() failed with \"%ld\" for "
			    "count #%d\n", PTR_ERR(mr), count);
			error = ENOENT;
			goto out;
		}
		commp->nvmrsnd_mr = mr;
		count++;
	}
	KASSERT(count == q->nvmrq_numsndqe, ("%s@%d count:%d numsndqe:%d",
	    __func__, __LINE__, count, q->nvmrq_numsndqe));

	/*
	 * Loop through the list of completion container structures mapping
	 * the corresponding NVMe completion structure
	 */
	count = 0;
	STAILQ_FOREACH(cmplp, &q->nvmrq_cmpls, nvmrsp_next) {
		dmaddr =  ib_dma_map_single(ibd, &cmplp->nvmrsp_nvmecmpl,
		    sizeof(cmplp->nvmrsp_nvmecmpl), DMA_FROM_DEVICE);
		if (ib_dma_mapping_error(ibd, dmaddr) != 0) {
			ERRSPEW("ib_dma_map_single() failed for #%d\n", count);
			error = ENOMEM;
			goto out;
		}
		cmplp->nvmrsp_dmaddr = dmaddr;
		ib_dma_sync_single_for_cpu(ibd, dmaddr,
		    sizeof(cmplp->nvmrsp_nvmecmpl), DMA_FROM_DEVICE);
		count++;
	}
	KASSERT(count == q->nvmrq_numrcvqe, ("%s@%d count:%d numrcvqe:%d",
	    __func__, __LINE__, count, q->nvmrq_numrcvqe));


	/*
	 * NB Once rdma_resolve_route() is called nvmr_connmgmt_handler() can be
	 * once again invoked.	Keep cntrlr consistent as it can be reached
	 * via cmid asynchronously.  Bump up the reference count for the
	 * activity in the nvmr_connmgmt_handler() contexts.
	 */
	PRE_ASYNC_CM_INVOCATION(NVMRQ_PRE_ROUTE_RESOLV);
	retval = rdma_resolve_route(cmid, NVMRTO);
	POST_ASYNC_CM_INVOCATION(__stringify(rdma_resolve_route),
	    NVMRQ_PRE_ROUTE_RESOLV, NVMRQ_ROUTE_RESOLV_SUCCEEDED);


	/*
	 * Now create the RDMA queue pair that we can post the NVMe command
	 * (Send Q) and NVMe completion (Recv Q) buffers to.
	 */
	memset(&init_attr, 0, sizeof(init_attr));
	/*
	 * Every Send Queue Element can potentially use a Work Request for:
	 * 1) The Send 2) An optional MR Registration 3) An invalidate for
	 * the MR Registration.  So a total of 3 per SendQE.  Add one more
	 * for when ib_drain_qp() is invoked.
	 */
	init_attr.cap.max_send_wr = (q->nvmrq_numsndqe * 3) + 1;

	/* One more for ib_drain_qp() */
	init_attr.cap.max_recv_wr = q->nvmrq_numrcvqe + 1;

	/*
	 * Allocate an RDMA completion Q for receiving the status of Work
	 * Requests (Send/Recv) on the Q pair.  It should be deep enough to
	 * handle completion Q elements from WRs (not QEs) in both Qs in
	 * the pair.
	 */
	ibcq = ib_alloc_cq(ibd, q, init_attr.cap.max_send_wr +
	    init_attr.cap.max_recv_wr, 0 /* hard-coded! completion vector */,
	    IB_POLL_WORKQUEUE);
	if (IS_ERR(ibcq)) {
		ERRSPEW("ib_alloc_cq() failed with 0x%lX\n", PTR_ERR(ibcq));
		error = ESPIPE;
		goto out;
	}
	q->nvmrq_ibcq = ibcq;

	init_attr.cap.max_send_sge = NVMR_NUMSNDSGE;
	init_attr.cap.max_recv_sge = NVMR_NUMRCVSGE;
	init_attr.qp_type = IB_QPT_RC;
	init_attr.send_cq = ibcq;
	init_attr.recv_cq = ibcq;
	init_attr.sq_sig_type = IB_SIGNAL_REQ_WR;
	init_attr.event_handler = nvmr_qphndlr;
	retval = rdma_create_qp(cmid, ibpd, &init_attr);
	if (retval != 0) {
		ERRSPEW("rdma_create_qp() failed with %d\n", retval);
		error = retval;
		goto out;
	}
	q->nvmrq_ibqp = cmid->qp;

	/* The QP is now open for business */
	atomic_store_rel_int(&q->nvmrq_gqp.qis_enabled, TRUE);

	/*
	 * Post NVMe completion buffers to the RDMA Recv Q, registering their
	 * associated Completion Q elements as well.  Use locks because the QP
	 * is active and the posting here can race with any flushing that
	 * can be triggered asynchronously.
	 */
	count = 0;
	STAILQ_FOREACH(cmplp, &q->nvmrq_cmpls, nvmrsp_next) {
		mtx_lock(&q->nvmrq_gqp.qlock);
		STAILQ_REMOVE(&q->nvmrq_cmpls, cmplp, nvmr_ncmplcont,
		    nvmrsp_next);
		q->nvmrq_numFrcvqe--;
		mtx_unlock(&q->nvmrq_gqp.qlock);

		retval = nvmr_post_cmpl(q, cmplp);
		if (retval != 0) {
			ERRSPEW("ib_post_recv() failed for #%d with %d\n",
			    count, retval);
			error = ENOMSG;
			goto out;
		}
		count++;
	}
	KASSERT(count == q->nvmrq_numrcvqe, ("%s@%d count:%d numrcvqe:%d",
	    __func__, __LINE__, count, q->nvmrq_numrcvqe));

	/*
	 * NB: The conn_param has to pass in an NVMeoF RDMA Private Data
	 * structure for the NVMeoF RDMA target to setup its Q sizes et al.
	 */
	memset(&conn_param, 0, sizeof(conn_param));
	memset(&privdata, 0, sizeof(privdata));
	privdata.nvmrcr_recfmt = 0;
	privdata.nvmrcr_qid = qid;
	privdata.nvmrcr_hrqsize = htole16(q->nvmrq_numrcvqe);
	privdata.nvmrcr_hsqsize = htole16(q->nvmrq_numsndqe);
	conn_param.responder_resources = ibd->attrs.max_qp_rd_atom;
	conn_param.qp_num = q->nvmrq_ibqp->qp_num;
	conn_param.flow_control = 1;
	conn_param.retry_count = 3;
	conn_param.rnr_retry_count = 3;
	conn_param.private_data = &privdata;
	conn_param.private_data_len = sizeof(privdata);

	PRE_ASYNC_CM_INVOCATION(NVMRQ_PRE_CONNECT);
	retval = rdma_connect(cmid, &conn_param);
	POST_ASYNC_CM_INVOCATION(__stringify(rdma_connect),
	    NVMRQ_PRE_CONNECT, NVMRQ_CONNECT_SUCCEEDED);

	/*
	 * Now that a connection has been established send out a CONNECT
	 * NVMeoF command identifying our system and the NVMe subsystem
	 * we're trying to reach via the nvmrcd_subnqn field
	 */
	memset(&ncdata, 0, sizeof(ncdata));
	ncdata.nvmrcd_hostid = nvrdma_host_uuid;
	ncdata.nvmrcd_cntlid = htole16(cntlid);
	snprintf(ncdata.nvmrcd_subnqn, sizeof(ncdata.nvmrcd_subnqn),
	    "%s", cntrlr->nvmrctr_subnqn);
	snprintf(ncdata.nvmrcd_hostnqn, sizeof(ncdata.nvmrcd_hostnqn),
	    HOSTNQN_TEMPLATE, nvrdma_host_uuid_string);

	req = nvme_allocate_request_vaddr(&ncdata, sizeof(ncdata),
	    nvme_completion_poll_cb, &status);
	cmd = (nvmr_communion_t *)&req->cmd;
	cmd->nvmrcu_conn.nvmrcn_nvmf.nvmf_opc = NVME_OPC_FABRIC_COMMAND;
	cmd->nvmrcu_conn.nvmrcn_nvmf.nvmf_fctype = NVMF_FCTYPE_CONNECT;
	cmd->nvmrcu_conn.nvmrcn_recfmt = 0;
	cmd->nvmrcu_conn.nvmrcn_qid = qid;
	cmd->nvmrcu_conn.nvmrcn_sqsize = htole16(q->nvmrq_numsndqe - 1);
	cmd->nvmrcu_conn.nvmrcn_cattr = 0;
	cmd->nvmrcu_conn.nvmrcn_kato = htole32(kato);

	status.done = 0;
	NVMR_INCR_Q_USECNT(q);
	nvmr_submit_req(q, req);
	while (!atomic_load_acq_int(&status.done)) {
		pause("nvmr", HZ > 100 ? (HZ/100) : 1);
	}                                                             \
	if (nvme_completion_is_error(&status.cpl)) {
		ERRSPEW("CONNECT NVMeoF command to subNQN \"%s\" failed!\n",
		    cntrlr->nvmrctr_subnqn);
		error = ENXIO;
		goto out;
	}

	error = 0;

out:
	if ((error != 0) && (q->nvmrq_state < NVMRQ_PRE_ADDR_RESOLV)) {
		/* Cleanup the Q because nvmr_cntrlr_destroy() won't see it */
		nvmr_qpair_destroy(q);
	}

	return error;
}

int
nvmr_admin_identify(nvmr_cntrlr_t cntrlr, uint16_t cntid, uint32_t nsid,
    uint8_t cns, void *datap, int datalen);
int
nvmr_admin_identify(nvmr_cntrlr_t cntrlr, uint16_t cntid, uint32_t nsid,
    uint8_t cns, void *datap, int datalen)
{
	int error;
	nvmr_communion_t *cmd;
	struct nvme_request *req;
	struct nvme_completion_poll_status status;

	if ((cntrlr == NULL) || (datap == NULL) || (datalen != IDENTIFYLEN)) {
		ERRSPEW("INVALID! cntrlr:%p datap:%p datalen:%d\n", cntrlr,
		    datap, datalen);
		error = EINVAL;
		goto out;
	}

	req = nvme_allocate_request_vaddr(&cntrlr->nvmrctr_nvmec.cdata,
	    sizeof(struct nvme_controller_data), nvme_completion_poll_cb,
	    &status);
	cmd = (nvmr_communion_t *)&req->cmd;
	cmd->nvmrcu_idnt.nvmrid_nvmf.nvmf_opc = NVME_OPC_IDENTIFY;
	cmd->nvmrcu_idnt.nvmrid_nvmf.nvmf_nsid = htole32(nsid);
	cmd->nvmrcu_idnt.nvmrid_cns = cns;
	cmd->nvmrcu_idnt.nvmrid_cntid = htole16(cntid);

	ISSUE_WAIT_CHECK_REQ {
		ERRSPEW("IDENTIFY NVMeoF command to subNQN \"%s\" failed!\n",
		    cntrlr->nvmrctr_subnqn);
		error = ENXIO;
		goto out;
	}

	error = 0;
out:
	return error;
}


int
nvmr_admin_propset(nvmr_cntrlr_t cntrlr, uint32_t offset, uint64_t value,
    nvmr_proplent_t len);
int
nvmr_admin_propset(nvmr_cntrlr_t cntrlr, uint32_t offset, uint64_t value,
    nvmr_proplent_t len)
{
	int error;
	nvmr_communion_t *cmd;
	struct nvme_request *req;
	struct nvme_completion_poll_status status;

	if ((cntrlr == NULL) || (offset > MAX_NVMR_PROP_GET) ||
	    (len >= NVMR_PROPLEN_MAX)) {
		ERRSPEW("INVALID! cntrlr:%p offset:0x%X len:%d\n",
		    cntrlr, offset, len);
		error = EINVAL;
		goto out;
	}

	req = nvme_allocate_request_null(nvme_completion_poll_cb, &status);
	cmd = (nvmr_communion_t *)&req->cmd;
	cmd->nvmrcu_prst.nvmrps_nvmf.nvmf_opc = NVME_OPC_FABRIC_COMMAND;
	cmd->nvmrcu_prst.nvmrps_nvmf.nvmf_fctype = NVMF_FCTYPE_PROPSET;
	cmd->nvmrcu_prst.nvmrps_attrib = len;
	cmd->nvmrcu_prst.nvmrps_ofst = offset;
	cmd->nvmrcu_prst.nvmrps_value = value;

	ISSUE_WAIT_CHECK_REQ {
		ERRSPEW("PROPSET NVMeoF command to subNQN \"%s\" failed!\n",
		    cntrlr->nvmrctr_subnqn);
		error = ENXIO;
		goto out;
	}

	error = 0;

out:
	return error;
}


static void
nvmr_ctrlr_reset_task(void *arg, int pending)
{
	ERRSPEW("Invoked for arg:%p pending:%d\n", arg, pending);
}


static struct cdevsw nvmr_ctrlr_cdevsw = {
	.d_version =	D_VERSION,
	.d_flags =	0,
};

static void
nvmr_delist_cb(struct nvme_controller *ctrlrp)
{
	panic("%s() invoked for ctrlr:%p\n", __func__, ctrlrp);
}

uint32_t
nvmr_req_ioq_count(nvmr_cntrlr_t cntrlr, uint16_t nioqs, uint16_t *nalloced);
uint32_t
nvmr_req_ioq_count(nvmr_cntrlr_t cntrlr, uint16_t nioqs, uint16_t *nalloced)
{
	int error;
	uint32_t qcount;
	nvmr_communion_t *cmd;
	struct nvme_request *req;
	struct nvme_completion_poll_status status;

	if ((cntrlr == NULL) || (nioqs < 1)) {
		ERRSPEW("INVALID! cntrlr:%p nioqs:%u\n", cntrlr, nioqs);
		error = EINVAL;
		goto out;
	}

	qcount = nioqs - 1; /* 0 based */

	req = nvme_allocate_request_null(nvme_completion_poll_cb, &status);
	cmd = (nvmr_communion_t *)&req->cmd;
	cmd->nvmrcu_stft.nvmrsf_nvmf.nvmf_opc = NVME_OPC_SET_FEATURES;
	cmd->nvmrcu_stft.nvmrsf_fid = htole32(NVME_FEAT_NUMBER_OF_QUEUES);
	cmd->nvmrcu_stft.nvmrsf_cmdw11  = htole32(qcount); /* NSQR */
	cmd->nvmrcu_stft.nvmrsf_cmdw11 |= htole32(qcount) << NCQ_SHIFT;

	ISSUE_WAIT_CHECK_REQ {
		ERRSPEW("Set IOQ count NVMe command failed!\n");
		error = ENXIO;
		goto out;
	}

	*nalloced = MIN(status.cpl.cdw0 & NSQR_MASK,
	    status.cpl.cdw0 >> NCQ_SHIFT);
	(*nalloced)++;                     /* Response is 0 based */
	*nalloced = MIN(*nalloced, nioqs); /* Don't use more than we can */
	error = 0;

out:
	return error;
}


int
nvmr_admin_propget(nvmr_cntrlr_t cntrlr, uint32_t offset, uint64_t *valuep,
    nvmr_proplent_t len);
int
nvmr_admin_propget(nvmr_cntrlr_t cntrlr, uint32_t offset, uint64_t *valuep,
    nvmr_proplent_t len)
{
	int error;
	nvmr_communion_t *cmd;
	struct nvme_request *req;
	struct nvme_completion_poll_status status;

	if ((cntrlr == NULL) || (offset > MAX_NVMR_PROP_GET) ||
	    (len >= NVMR_PROPLEN_MAX) || (valuep == NULL)) {
		ERRSPEW("INVALID! cntrlr:%p offset:0x%X len:%d valuep:%p\n",
		    cntrlr, offset, len, valuep);
		error = EINVAL;
		goto out;
	}

	req = nvme_allocate_request_null(nvme_completion_poll_cb, &status);
	cmd = (nvmr_communion_t *)&req->cmd;
	cmd->nvmrcu_prgt.nvmrpg_nvmf.nvmf_opc = NVME_OPC_FABRIC_COMMAND;
	cmd->nvmrcu_prgt.nvmrpg_nvmf.nvmf_fctype = NVMF_FCTYPE_PROPGET;
	cmd->nvmrcu_prgt.nvmrpg_attrib = len;
	cmd->nvmrcu_prgt.nvmrpg_ofst = offset;

	ISSUE_WAIT_CHECK_REQ {
		ERRSPEW("PROPGET NVMeoF command to subNQN \"%s\" failed!\n",
		    cntrlr->nvmrctr_subnqn);
		error = ENXIO;
		goto out;
	}

	*valuep = (((uint64_t)status.cpl.rsvd1) << 32) | status.cpl.cdw0;
	error = 0;

out:
	return error;
}


static void
nvmr_register_cntrlr(nvmr_cntrlr_t cntrlr)
{
	mtx_lock(&nvmr_cntrlrs_lstslock);
	TAILQ_INSERT_HEAD(&nvmr_cntrlrs_active, cntrlr, nvmrctr_nxt);
	cntrlr->nvmrctr_glblst = NVMR_CNTRLRLST_ACTIVE;
	nvmr_cntrlrs_count++;
	mtx_unlock(&nvmr_cntrlrs_lstslock);
}

void
nvmr_qpair_manual_complete_request(struct nvme_qpair *qpair,
    struct nvme_request *req, uint32_t sct, uint32_t sc,
    boolean_t print_on_error);
void
nvmr_qpair_manual_complete_request(struct nvme_qpair *qpair,
    struct nvme_request *req, uint32_t sct, uint32_t sc,
    boolean_t print_on_error)
{
	struct nvme_completion	cpl;
	boolean_t		error;
	nvmr_qpair_t		q;

	memset(&cpl, 0, sizeof(cpl));
	cpl.sqid = qpair->qid;
	cpl.status |= (sct & NVME_STATUS_SCT_MASK) << NVME_STATUS_SCT_SHIFT;
	cpl.status |= (sc & NVME_STATUS_SC_MASK) << NVME_STATUS_SC_SHIFT;

	error = nvme_completion_is_error(&cpl);

	if (error && print_on_error) {
		nvme_qpair_print_command(qpair, &req->cmd);
		nvme_qpair_print_completion(qpair, &cpl);
	}

	if (req->cb_fn)
		req->cb_fn(req->cb_arg, &cpl);

	nvme_free_request(req);
	
	/*
	 * The assumption is that at this point the consumer layer has no more
	 * state associated with this IO
	 */
	q = __containerof(qpair, struct nvmr_qpair_tag, nvmrq_gqp);
	nvmr_drop_q_usecount(q);
}

void
nvmr_ctrlr_fail_req_task(void *arg, int pending);
void
nvmr_ctrlr_fail_req_task(void *arg, int pending)
{
	struct nvme_controller	*ctrlr = arg;
	struct nvme_request	*req;

	KASSERT(ctrlr->nvmec_ttype == NVMET_RDMA,
	    ("Non NVMr transport c:%p t:%d", ctrlr, ctrlr->nvmec_ttype));

	mtx_lock(&ctrlr->lockc);
	while ((req = STAILQ_FIRST(&ctrlr->fail_req)) != NULL) {
		STAILQ_REMOVE_HEAD(&ctrlr->fail_req, stailq);
		mtx_unlock(&ctrlr->lockc);
		nvmr_qpair_manual_complete_request(req->rqpair, req,
		    NVME_SCT_GENERIC, NVME_SC_ABORTED_BY_REQUEST, TRUE);
		mtx_lock(&ctrlr->lockc);
	}
	mtx_unlock(&ctrlr->lockc);
}

#define BAIL_IF_CNTRLR_CONDEMNED(c)                                        \
	if (atomic_load_acq_int(&(c)->nvmrctr_state) == NVMRC_CONDEMNED) { \
		error = ESHUTDOWN;                                         \
		goto out;                                                  \
	}


int
nvmr_cntrlr_create(nvmr_addr_t *addr, nvmr_cntrlrprof_t *prof,
    nvmr_cntrlr_t *retcntrlrp);
int
nvmr_cntrlr_create(nvmr_addr_t *addr, nvmr_cntrlrprof_t *prof,
    nvmr_cntrlr_t *retcntrlrp)
{
	int error, retval, count, timeout_period;
	struct make_dev_args md_args;
	nvmr_cntrlr_t cntrlr;
	nvmr_qpair_t *qarr;
	unsigned long tmp;
	nvmr_cntrlcap_t cntrlrcap;
	/* uint64_t cntrlrconf; */
	uint32_t unitnum, cc;
	nvmripv4_t ipv4;
	nvmr_qprof_t *pf;
	uint16_t port, ioqcount, reqioqcount;
	uint16_t io_numsndqe, io_numrcvqe;
	/* uint8_t *identp; */
	char *retp;
	struct ib_device *ibd;
	struct nvme_controller_data *cd;

	retval = -1;
	if ((addr->nvmra_subnqn == NULL) ||
	    (retval = strlen(addr->nvmra_subnqn), retval > MAX_NQN_LEN)) {
		ERRSPEW("Invalid NVMe Subsystem NQN string passed in: \"%s\" "
		    "of length %d\n", addr->nvmra_subnqn, retval);
		error = EINVAL;
		goto outret;
	}

	if (inet_pton(AF_INET, addr->nvmra_ipaddr, &ipv4) != 1) {
		ERRSPEW("Parsing failed for IPV4 address \"%s\"\n",
		    addr->nvmra_ipaddr);
		error = EINVAL;
		goto outret;
	}

	tmp = strtoul(addr->nvmra_port, &retp, 0);
	if ((*retp != '\0') || (tmp > UINT16_MAX)) {
		ERRSPEW("Parsing failed with %lu for RDMA port \"%s\"\n", tmp,
		    addr->nvmra_port);
		error = EINVAL;
		goto outret;
	}
	port = htons((uint16_t)tmp);

	cntrlr = malloc(sizeof(*cntrlr), M_NVMR, M_WAITOK|M_ZERO);
	if (cntrlr == NULL) {
		ERRSPEW("Controller allocation sized \"%zu\" failed\n",
		    sizeof(*cntrlr));
		error = ENOMEM;
		goto outret;
	}
	DBGSPEW("Controller allocation of size \"%zu\"@%p\n", sizeof(*cntrlr),
	    cntrlr);
	memcpy(&cntrlr->nvmrctr_ipv4, ipv4, sizeof(cntrlr->nvmrctr_ipv4));
	cntrlr->nvmrctr_port = port;
	cntrlr->nvmrctr_prof = prof;
	strncpy(cntrlr->nvmrctr_ipv4str, addr->nvmra_ipaddr,
	    sizeof(cntrlr->nvmrctr_ipv4str));
	strncpy(cntrlr->nvmrctr_subnqn, addr->nvmra_subnqn,
	    sizeof(cntrlr->nvmrctr_subnqn));
	cntrlr->nvmrctr_state = NVMRC_PRE_INIT;
	cntrlr->nvmrctr_nvmereg = FALSE;
	strncpy(cntrlr->very_first_field, NVMR_STRING, NVME_VFFSTRSZ);
	mtx_init(&cntrlr->nvmrctr_nvmec.lockc, "nvme ctrlr lock", NULL,
	    MTX_DEF);

	/*
	 * After the following call the cntrlr can only be destroyed via
	 * nvmr_cntrlr_condemn_enqueue() by this routine
	 */
	nvmr_register_cntrlr(cntrlr);

	/**********
	 Set up the NVMe stack facing fields so we can issue NVMe commands to
	 the target using the stack
	 **********/
	cntrlr->nvmrctr_nvmec.is_resetting = 0;
	cntrlr->nvmrctr_nvmec.notification_sent = 0;
	TASK_INIT(&cntrlr->nvmrctr_nvmec.reset_task, 0,
	    nvmr_ctrlr_reset_task, cntrlr);
	TASK_INIT(&cntrlr->nvmrctr_nvmec.fail_req_task, 0,
	    nvmr_ctrlr_fail_req_task, &cntrlr->nvmrctr_nvmec);
	STAILQ_INIT(&cntrlr->nvmrctr_nvmec.fail_req);
	cntrlr->nvmrctr_nvmec.is_failed = FALSE;

	timeout_period = NVME_DEFAULT_TIMEOUT_PERIOD;
	TUNABLE_INT_FETCH("hw.nvme.timeout_period", &timeout_period);
	timeout_period = min(timeout_period, NVME_MAX_TIMEOUT_PERIOD);
	timeout_period = max(timeout_period, NVME_MIN_TIMEOUT_PERIOD);
	cntrlr->nvmrctr_nvmec.timeout_period = timeout_period;

	cntrlr->nvmrctr_nvmec.timeout_period = NVME_DEFAULT_TIMEOUT_PERIOD;
	cntrlr->nvmrctr_nvmec.max_xfer_size = NVME_MAX_XFER_SIZE;
	cntrlr->nvmrctr_nvmec.taskqueue = taskqueue_create("nvmr_taskq",
	    M_WAITOK, taskqueue_thread_enqueue,
	    &cntrlr->nvmrctr_nvmec.taskqueue);
	taskqueue_start_threads(&cntrlr->nvmrctr_nvmec.taskqueue, 1, PI_DISK,
	    "nvmr taskq");

	cntrlr->nvmrctr_nvmec.nvmec_tsp = cntrlr;
	cntrlr->nvmrctr_nvmec.nvmec_ttype = NVMET_RDMA;
	cntrlr->nvmrctr_nvmec.nvmec_delist = &nvmr_delist_cb;
	cntrlr->nvmrctr_nvmec.nvmec_subadmreq = &nvmr_submit_adm_req;
	cntrlr->nvmrctr_nvmec.nvmec_subioreq = &nvmr_submit_io_req;

	retval = nvmr_qpair_create(cntrlr, &cntrlr->nvmrctr_adminqp, 0,
	    NVMR_DYNANYCNTLID,
	    prof->nvmrp_qprofs[NVMR_QTYPE_ADMIN].nvmrqp_numqe - 1,
	    prof->nvmrp_qprofs[NVMR_QTYPE_ADMIN].nvmrqp_numqe,
	    prof->nvmrp_qprofs[NVMR_QTYPE_ADMIN].nvmrqp_kato);
	if (retval != 0) {
		ERRSPEW("%s creation failed with %d\n",
		    nvmr_qndx2name(NVMR_QTYPE_ADMIN), retval);
		error = retval;
		goto out;
	}

	/*
	retval = nvmr_admin_propget(cntrlr, 0, (uint64_t *)&cntrlrcap,
	    NVMR_PROPLEN_8BYTES);
	if (retval != 0) {
		error = retval;
		ERRSPEW("nvmr_admin_propget(o:0x%X l:%d) failed:%d\n", 0,
		    NVMR_PROPLEN_8BYTES, retval);
		goto out;
	}

	DBGSPEW("PROPGET CAP:\n\t"
	    "MQES:%lu CQR:%lu CMS:%lu TO:%lu DSTRD:%lu\n\t"
	    "NSSRS:%lu CSS:%lu BPS:%lu MPSMIN:%lu MPSMAX:%lu\n",
	    cntrlrcap.nvmrcc_mqes, cntrlrcap.nvmrcc_cqr, cntrlrcap.nvmrcc_ams,
	    cntrlrcap.nvmrcc_to, cntrlrcap.nvmrcc_dstrd, cntrlrcap.nvmrcc_nssrs,
	    cntrlrcap.nvmrcc_css, cntrlrcap.nvmrcc_bps, cntrlrcap.nvmrcc_mpsmin,
	    cntrlrcap.nvmrcc_mpsmax);

	cntrlrconf = 0;
	retval = nvmr_admin_propget(cntrlr, 20, &cntrlrconf,
	    NVMR_PROPLEN_4BYTES);
	if (retval != 0) {
		error = retval;
		ERRSPEW("nvmr_admin_propget(o:0x%X l:%d) failed:%d\n", 20,
		    NVMR_PROPLEN_4BYTES, retval);
		goto out;
	} else {
		DBGSPEW("cntrlrconf is 0x%08lX\n", cntrlrconf);
	}

	cntrlrconf = 0;
	retval = nvmr_admin_propget(cntrlr, 28, &cntrlrconf,
	    NVMR_PROPLEN_4BYTES);
	if (retval != 0) {
		error = retval;
		ERRSPEW("nvmr_admin_propget(o:0x%X l:%d) failed:%d\n", 28,
		    NVMR_PROPLEN_4BYTES, retval);
		goto out;
	} else {
		DBGSPEW("reg 28 is 0x%08lX\n", cntrlrconf);
	}

	cntrlrconf = 0;
	retval = nvmr_admin_propget(cntrlr, 8, &cntrlrconf,
	    NVMR_PROPLEN_4BYTES);
	if (retval != 0) {
		error = retval;
		ERRSPEW("nvmr_admin_propget(o:0x%X l:%d) failed:%d\n", 8,
		    NVMR_PROPLEN_4BYTES, retval);
		goto out;
	} else {
		DBGSPEW("Reg 8 is 0x%08lX\n", cntrlrconf);
	}
	 */

	/* Initialization values for Controller Configuration */
	cc = 0;
	cc |= 1 << NVME_CC_REG_EN_SHIFT;
	cc |= 0 << NVME_CC_REG_CSS_SHIFT;
	cc |= 0 << NVME_CC_REG_AMS_SHIFT;
	cc |= 0 << NVME_CC_REG_SHN_SHIFT;
	cc |= 6 << NVME_CC_REG_IOSQES_SHIFT; /* SQ entry size == 64 == 2^6 */
	cc |= 4 << NVME_CC_REG_IOCQES_SHIFT; /* CQ entry size == 16 == 2^4 */

	retval = nvmr_admin_propset(cntrlr, NVMFPD_CC_OFF, cc, NVMFPD_CC_SZ);
	if (retval != 0) {
		error = retval;
		ERRSPEW("nvmr_admin_propset(o:0x%X l:%d) failed:%d\n", 0x14,
		    NVMR_PROPLEN_4BYTES, retval);
		goto out;
	}

	cntrlr->nvmrctr_nvmec.guard0 = cntrlr->nvmrctr_nvmec.guard1 = 
	    0xDEADD00D8BADBEEF;
	retval = nvmr_admin_identify(cntrlr, 0, 0, 1,
	    &cntrlr->nvmrctr_nvmec.cdata, sizeof(cntrlr->nvmrctr_nvmec.cdata));
	if (retval != 0) {
		error = retval;
		ERRSPEW("nvmr_admin_identify() failed:%d\n", retval);
		goto out;
	} else {
		KASSERT((cntrlr->nvmrctr_nvmec.guard0 == 0xDEADD00D8BADBEEF) &&
		    (cntrlr->nvmrctr_nvmec.guard1 == 0xDEADD00D8BADBEEF),
		    ("%s@%d Guards failed! %lx %lx", __func__, __LINE__,
		    cntrlr->nvmrctr_nvmec.guard0,
		    cntrlr->nvmrctr_nvmec.guard1));
		/*
		identp = (uint8_t *)&cntrlr->nvmrctr_nvmec.cdata;
		printf("       ");
		for (count = 0; count < 16; count++) {
			printf(" %02x", count);
		}
		printf("\n");
		printf("       ");
		for (count = 0; count < 16; count++) {
			printf(" vv");
		}
		printf("\n");
		for (count = 0; count < IDENTIFYLEN; count++) {
			if ((count % 16) == 0) {
				printf("0x%04x:", count);
			}
			printf(" %02hhX", identp[count]);
			if ((count % 16) == 15) {
				printf("\n");
			}
		}
		 */
	}
	cd = &cntrlr->nvmrctr_nvmec.cdata;
	DBGSPEW("ioccsz:%u iorcsz:%u icdoff:%hu ctrattr:%hhu msdbd:%hhu\n",
	     cd->nidf_ioccsz, cd->nidf_iorcsz, cd->nidf_icdoff,
	     cd->nidf_ctrattr, cd->nidf_msdbd);
	if ((cd->nidf_ioccsz*NVMR_PAYLOAD_UNIT) > sizeof(nvmr_stub_t)) {
		ERRSPEW("\n\t!!! Unimplemented inline data @%lu possible !!!\n",
		    (cd->nidf_ioccsz*NVMR_PAYLOAD_UNIT) - sizeof(nvmr_stub_t));
	}

	/* Retrieve the Maximum number of Queue Elements Supported */
	bzero(&cntrlrcap, sizeof(cntrlrcap));
	retval = nvmr_admin_propget(cntrlr, 0, (uint64_t *)&cntrlrcap,
	    NVMR_PROPLEN_8BYTES);
	if (retval != 0) {
		error = retval;
		ERRSPEW("nvmr_admin_propget(o:0x%X l:%d) failed:%d\n", 0,
		    NVMR_PROPLEN_8BYTES, retval);
		goto out;
	}
	DBGSPEW("PROPGET CAP:\n\t"
	    "MQES:%lu CQR:%lu CMS:%lu TO:%lu DSTRD:%lu\n\t"
	    "NSSRS:%lu CSS:%lu BPS:%lu MPSMIN:%lu MPSMAX:%lu\n",
	    cntrlrcap.nvmrcc_mqes, cntrlrcap.nvmrcc_cqr, cntrlrcap.nvmrcc_ams,
	    cntrlrcap.nvmrcc_to, cntrlrcap.nvmrcc_dstrd, cntrlrcap.nvmrcc_nssrs,
	    cntrlrcap.nvmrcc_css, cntrlrcap.nvmrcc_bps, cntrlrcap.nvmrcc_mpsmin,
	    cntrlrcap.nvmrcc_mpsmax);

	if (cntrlrcap.nvmrcc_dstrd != 0) {
		ERRSPEW("Non-zero Doorbell stride (%lu) unsupported\n",
		    cntrlrcap.nvmrcc_dstrd);
		error = ENOSPC;
		goto out;
	}

	/* Setup the min_page_size and ready_timeout_in_ms fields per NVMp */
	cntrlr->nvmrctr_nvmec.min_page_size =
	    1 << (12 + cntrlrcap.nvmrcc_mpsmin);
	cntrlr->nvmrctr_nvmec.ready_timeout_in_ms = cntrlrcap.nvmrcc_to *
	    500;

	/**********
	 Come up with a unique unitnum that has enough room for
	 NVME_MAX_NAMESPACES sub-unitnums.
	 **********/
	unitnum = (uint32_t)((uint64_t)(&cntrlr->nvmrctr_nvmec)/
	    sizeof(cntrlr->nvmrctr_nvmec));
	unitnum &= INT_MAX;
	unitnum *= NVME_MAX_NAMESPACES;
	unitnum /= NVME_MAX_NAMESPACES;
	cntrlr->nvmrctr_nvmec.nvmec_unit = (int)(uint32_t)unitnum;

	make_dev_args_init(&md_args);
	md_args.mda_devsw = &nvmr_ctrlr_cdevsw;
	md_args.mda_uid = UID_ROOT;
	md_args.mda_gid = GID_WHEEL;
	md_args.mda_mode = 0600;
	md_args.mda_unit = cntrlr->nvmrctr_nvmec.nvmec_unit;/* Security hole? */
	md_args.mda_si_drv1 = (void *)cntrlr;
	retval = make_dev_s(&md_args, &cntrlr->nvmrctr_nvmec.ccdev, "nvme%d",
	    md_args.mda_unit);
	if (retval != 0) {
		ERRSPEW("make_dev_s() for cntrlr:%p returned %d\n", cntrlr,
		    retval);
		error = retval;
		goto out;
	}
	DBGSPEW("NVMe device with unitnum:%d\n",
	    cntrlr->nvmrctr_nvmec.nvmec_unit);

	retval = nvme_ctrlr_construct_namespaces(&cntrlr->nvmrctr_nvmec);
	if (retval != 0) {
		error = retval;
		ERRSPEW("nvme_ctrlr_construct_namespaces(c:%p):%d\n",
		    &cntrlr->nvmrctr_nvmec, error);
		goto out;
	}


	/**********
	 Now set up the IO Qs
	 **********/
	ibd = cntrlr->nvmrctr_adminqp->nvmrq_cmid->device;
	reqioqcount = MIN(mp_ncpus, ibd->num_comp_vectors);
	if (reqioqcount > 1) {
		reqioqcount--; /* Decrement one vector/CPU for admin Q */
	}
	retval = nvmr_req_ioq_count(cntrlr, reqioqcount, &ioqcount);
	if (retval != 0) {
		error = retval;
		ERRSPEW("nvmr_req_ioq_count(count:%hu):%d\n", reqioqcount,
		    retval);
		goto out;
	}
	DBGSPEW("Controller allocated IOQ count is %hu, requested is %hu\n",
	    ioqcount, reqioqcount);

	pf = &prof->nvmrp_qprofs[NVMR_QTYPE_IO];
	if (pf->nvmrqp_numqe != 0) {
		io_numsndqe = pf->nvmrqp_numqe;
	} else {
		io_numsndqe = cntrlrcap.nvmrcc_mqes;
	}
	io_numrcvqe = io_numsndqe + 1;


	if (pf->nvmrqp_numqueues != 0) {
		cntrlr->nvmrctr_numioqs = pf->nvmrqp_numqueues;
	} else {
		cntrlr->nvmrctr_numioqs = ioqcount;
	}

	BAIL_IF_CNTRLR_CONDEMNED(cntrlr);

	if (cntrlr->nvmrctr_numioqs != 0) {
		qarr = malloc(cntrlr->nvmrctr_numioqs * sizeof(nvmr_qpair_t),
		    M_NVMR, M_WAITOK|M_ZERO);
		if (qarr == NULL) {
			ERRSPEW("IO Q array allocation sized \"%zu\" failed\n",
			    cntrlr->nvmrctr_numioqs * sizeof(nvmr_qpair_t));
			error = ENOMEM;
			goto out;
		}
		DBGSPEW("IO Q array allocation of size \"%zu\"\n",
		    cntrlr->nvmrctr_numioqs * sizeof(nvmr_qpair_t));
		cntrlr->nvmrctr_ioqarr = qarr;

		/* Allocate IO queues and store pointers to them in the qarr */
		for (count = 0; count < cntrlr->nvmrctr_numioqs; count++) {
			retval = nvmr_qpair_create(cntrlr, &qarr[count],
			    NVMR_IOQID_ADDEND + count,
			    cntrlr->nvmrctr_nvmec.cdata.ctrlr_id,
			    io_numsndqe, io_numrcvqe,
			    prof->nvmrp_qprofs[NVMR_QTYPE_IO].nvmrqp_kato);
			if (retval != 0) {
				ERRSPEW("%s#%d creation failed with %d\n",
				    nvmr_qndx2name(NVMR_QTYPE_IO), count,
				    retval);
				error = retval;
				goto out;
			}

			BAIL_IF_CNTRLR_CONDEMNED(cntrlr);
		}
		KASSERT(count == cntrlr->nvmrctr_numioqs,
		    ("count:%d numqs:%d", count, cntrlr->nvmrctr_numioqs));
	} else {
		cntrlr->nvmrctr_ioqarr = NULL;
	}

	nvmr_cntrlr_inited(cntrlr);
	BAIL_IF_CNTRLR_CONDEMNED(cntrlr); /* It's important this be here */

	cntrlr->nvmrctr_nvmec.is_initialized = 1;

	nvme_register_controller(&cntrlr->nvmrctr_nvmec);
	cntrlr->nvmrctr_nvmereg = TRUE;
	nvme_notify_new_controller(&cntrlr->nvmrctr_nvmec);

	error = 0;
	*retcntrlrp = cntrlr;

out:
	if (error == 0) {
		goto outret;
	}

	DBGSPEW("Controller:%p init failed:%d, queuing for destruction\n",
	    cntrlr, error);

	nvmr_cntrlr_condemn_enqueue(cntrlr);
	*retcntrlrp = NULL;

outret:
	return error;
}

/*
 * Invoked whenever the routine is registered with ib_register_client()
 * below or when an IB interface is added to the system.  In the former case
 * the routine is invoked for every IB interface already known.
 */
static void
nvmr_add_ibif(struct ib_device *ib_device)
{
	DBGSPEW("rdma_node_get_transport(%p)> %d\n", ib_device,
	    rdma_node_get_transport(ib_device->node_type));
}


/*
 * Invoked whenever the routine is unregistered with ib_unregister_client()
 * below or when an IB interface is removed from the system.  In the former case
 * the routine is invoked for every IB interface already known.
 */
static void
nvmr_remove_ibif(struct ib_device *ib_device, void *client_data)
{
	DBGSPEW("%p removed\n", ib_device);
}


static struct ib_client nvmr_ib_client = {
	.name   = "nvmrdma",
	.add    = nvmr_add_ibif,
	.remove = nvmr_remove_ibif
};

#define MAX_IOQ_ELEMENTS 32

nvmr_cntrlrprof_t nvmr_regularprof = {
	.nvmrp_qprofs[NVMR_QTYPE_ADMIN] = {
		.nvmrqp_numqueues = 1,
		.nvmrqp_numqe = MAX_ADMINQ_ELEMENTS,
		.nvmrqp_kato = NVMR_DEFAULT_KATO,
	},
	.nvmrp_qprofs[NVMR_QTYPE_IO] = {
		.nvmrqp_numqueues = 1,
		.nvmrqp_numqe = MAX_IOQ_ELEMENTS,
		.nvmrqp_kato = NVMR_DEFAULT_KATO,
	},
};

#define NVMR_CMD_ATT "attach"
#define NVMR_CMD_DET "detach"
#define NVMR_CMD_LST "list"
#define NVMR_CNTRLRS_LIST_INIT_SZ 10

char nvmr_cmd0[] = "";

#define NVMR_SYSCTL_RETNULL \
     error = SYSCTL_OUT(req, nvmr_cmd0, sizeof(nvmr_cmd0))

int nvmr_sysctl_attach_cntrlr(struct sysctl_req *req, char *buf);
int
nvmr_sysctl_attach_cntrlr(struct sysctl_req *req, char *buf)
{
	char *ptr;
	int error;
	nvmr_addr_t addr;
	nvmr_cntrlr_t cntrlr;

	ptr = buf;

	strsep(&ptr, ":");
	addr.nvmra_ipaddr = strsep(&ptr, ",");
	addr.nvmra_port   = strsep(&ptr, ",");
	addr.nvmra_subnqn = ptr;

	DBGSPEW("addr is %s,%s,%s\n", addr.nvmra_ipaddr, addr.nvmra_port,
	    addr.nvmra_subnqn);
	error = nvmr_cntrlr_create(&addr, &nvmr_regularprof, &cntrlr);
	if (error != 0) {
		ERRSPEW("nvmr_cntrlr_create(\"%s\", \"%s\", \"%s\") failed "
		    "with %d\n", addr.nvmra_ipaddr, addr.nvmra_port,
		    addr.nvmra_subnqn, error);
	}

	NVMR_SYSCTL_RETNULL;
	return error;
}

int nvmr_sysctl_detach_cntrlr(struct sysctl_req *req, char *buf);
int
nvmr_sysctl_detach_cntrlr(struct sysctl_req *req, char *buf)
{
	char *ptr;
	int unitnum, error;

	ptr = buf;

	strsep(&ptr, ":");
	unitnum = (int)strtol(ptr, NULL, 0);

	if (unitnum != 0) {
		nvmr_cntrlrs_condemn_init(unitnum);
	}

	NVMR_SYSCTL_RETNULL;

	return error;
}

int nvmr_sysctl_list_cntrlrs(struct sysctl_req *req, char *buf);
int
nvmr_sysctl_list_cntrlrs(struct sysctl_req *req, char *buf)
{
	int error;
	struct sbuf *sb;
	nvmr_cntrlr_t cntrlr;

	error = sysctl_wire_old_buffer(req, 0);
	if (error != 0) {
		goto out;
	}

	sb = sbuf_new_for_sysctl(NULL, NULL, NVMR_CNTRLRS_LIST_INIT_SZ, req);
	if (sb == NULL) {
		error = ENOMEM;
		goto out;
	}
	sbuf_printf(sb, "\n");

	mtx_lock(&nvmr_cntrlrs_lstslock);
	TAILQ_FOREACH(cntrlr, &nvmr_cntrlrs_active, nvmrctr_nxt) {
		sbuf_printf(sb, "%d %s,%hu,%s\n",
		    cntrlr->nvmrctr_nvmec.nvmec_unit, cntrlr->nvmrctr_ipv4str,
		    ntohs(cntrlr->nvmrctr_port), cntrlr->nvmrctr_subnqn);
	}
	mtx_unlock(&nvmr_cntrlrs_lstslock);

	error = sbuf_finish(sb);
	sbuf_delete(sb);

out:
	return error;
}


static int
nvmr_sysctl_conrollers(SYSCTL_HANDLER_ARGS)
{
	int error;
	char buf[256];

	if (req->newptr == NULL) {
		error = nvmr_sysctl_list_cntrlrs(req, buf);
		goto out;
	}

	error = sysctl_handle_string(oidp, buf, sizeof(buf), req);
	if (error == 0) {
		if (!strncmp(buf, NVMR_CMD_LST, strlen(NVMR_CMD_LST))) {
			error = nvmr_sysctl_list_cntrlrs(req, buf);
			goto out;
		} else if (!strncmp(buf, NVMR_CMD_ATT, strlen(NVMR_CMD_ATT))) {
			error = nvmr_sysctl_attach_cntrlr(req, buf);
			goto out;
		} else if (!strncmp(buf, NVMR_CMD_DET, strlen(NVMR_CMD_DET))) {
			error = nvmr_sysctl_detach_cntrlr(req, buf);
			goto out;
		} else {
			NVMR_SYSCTL_RETNULL;
			goto out;
		}
	}

out:
	return error;
}

char nvmr_cmdusage[] = "Usage:\n"
	"\tlist (lists the NVMr controllers and their unit-numbers)\n"
	"\tattach:IP-address,port-number,sub-NQN (connects to a controller)\n"
	"\tdetach:unit-number (disconnects from a controller.  -1 means all)\n";

static SYSCTL_NODE(_hw, OID_AUTO, nvmrdma, CTLFLAG_RD, 0, "NVMeoRDMA");

SYSCTL_PROC(_hw_nvmrdma, OID_AUTO, controllers,
	    CTLTYPE_STRING | CTLFLAG_RW,
	    NULL, 0, nvmr_sysctl_conrollers, "A", nvmr_cmdusage);

SYSCTL_U32(_hw_nvmrdma, OID_AUTO, io_numqueues, CTLFLAG_RW,
    &nvmr_regularprof.nvmrp_qprofs[NVMR_QTYPE_IO].nvmrqp_numqueues, 1,
    "Number of IO queues to allocate a controller. 0 means query controller.");

SYSCTL_U32(_hw_nvmrdma, OID_AUTO, io_numqe, CTLFLAG_RW,
    &nvmr_regularprof.nvmrp_qprofs[NVMR_QTYPE_IO].nvmrqp_numqe,
    MAX_IOQ_ELEMENTS,
    "Number of IO queue elements to allocate. 0 means query controller.");

static void
nvmr_init(void)
{
	int retval;

	retval = ib_register_client(&nvmr_ib_client);
	if (retval != 0) {
		ERRSPEW("ib_register_client() for NVMeoF failed, ret:%d\n",
		    retval);
		goto out;
	}

	kern_uuidgen(&nvrdma_host_uuid, 1);
	snprintf_uuid(nvrdma_host_uuid_string, sizeof(nvrdma_host_uuid_string),
	    &nvrdma_host_uuid);
	DBGSPEW("Generated UUID is \"%s\"\n", nvrdma_host_uuid_string);

out:
	return;
}


static void
nvmr_uninit(void)
{
	DBGSPEW("Uninit invoked\n");

	nvmr_all_cntrlrs_condemn_init();

	mtx_lock(&nvmr_cntrlrs_lstslock);
	while (nvmr_cntrlrs_count != 0) {
		ERRSPEW("Waiting for all NVMr controllers to be deallocated\n");
		mtx_sleep(&nvmr_cntrlrs_count, &nvmr_cntrlrs_lstslock, 0,
		    "cdrain", HZ*2);
	}
	mtx_unlock(&nvmr_cntrlrs_lstslock);

	taskqueue_drain_all(taskqueue_nvmr_cntrlrs_reaper);

	ib_unregister_client(&nvmr_ib_client);
}


SYSINIT(nvmr, SI_SUB_DRIVERS, SI_ORDER_ANY, nvmr_init, NULL);
SYSUNINIT(nvmr, SI_SUB_DRIVERS, SI_ORDER_ANY, nvmr_uninit, NULL);
MODULE_DEPEND(nvmr, linuxkpi, 1, 1, 1);
MODULE_DEPEND(nvmr, ibcore, 1, 1, 1);
MODULE_DEPEND(nvmr, nvme, 1, 1, 1);
MODULE_VERSION(nvmr, 1);
