#undef VELOLD

#include <sys/cdefs.h>
#include <sys/types.h>
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/uuid.h>
#include <sys/conf.h>
#include <sys/systm.h>
#include <sys/module.h>

#include <netinet/in.h>

#include <linux/kernel.h>
#include <linux/netdevice.h>

#include <rdma/rdma_cm.h>
#include <rdma/ib_verbs.h>

#include <dev/nvme/nvme_shared.h>

#define SPEWCMN(prefix, format, ...) printf(prefix "%s@%d> " format, \
    __func__, __LINE__, ## __VA_ARGS__)
#define SPEW(format, ...) SPEWCMN("", format, ## __VA_ARGS__)
#define ERRSPEW(format, ...) SPEWCMN("ERR|", format, ## __VA_ARGS__)
#define DBGSPEW(format, ...) SPEWCMN("DBG|", format, ## __VA_ARGS__)

#define MAX_ADMIN_WORK_REQUESTS 32
#define MAX_NVME_RDMA_SEGMENTS 256

struct nvmr_ncmplcont {
	struct ib_cqe			nvmrsp_cqe;
	STAILQ_ENTRY(nvmr_ncmplcont)	nvmrsp_next;
	struct nvme_completion	       *nvmrsp_nvmecompl;
	struct nvme_completion	        nvmrsp_nvmecmpl;
	u64				nvmrsp_dmaddr;
};
typedef struct nvmr_ncmplcont nvmr_ncmplcon_t;

struct nvmr_ncommcont {
	struct ib_cqe		     nvmsnd_cqe;
	STAILQ_ENTRY(nvmr_ncommcont) nvmsnd_next;
	struct nvme_command         *nvmsnd_nvmecomm;
	u64			     nvmsnd_dmaddr;
};
typedef struct nvmr_ncommcont nvmr_ncommcon_t;

typedef struct {
	uint16_t nvmrcr_recfmt;
	uint16_t nvmrcr_qid;
	uint16_t nvmrcr_hrqsize;
	uint16_t nvmrcr_hsqsize;
	uint8_t  nvmrcr_resv0[24];
} __packed nvmr_rdma_cm_request_t;
CTASSERT(sizeof(nvmr_rdma_cm_request_t) == 32);

typedef struct {
	uint16_t nvmrcrj_recfmt;
	uint16_t nvmrcrj_sts;
} __packed nvmr_rdma_cm_reject_t;
CTASSERT(sizeof(nvmr_rdma_cm_reject_t) == 4);

static void
nvmr_qphndlr(struct ib_event *ev, void *ctx)
{
	ERRSPEW("Event \"%s\" on QP:%p\n", ib_event_msg(ev->event), ctx);
}


#ifdef VELOLD
struct nvmr_ncmplcont ncmplcntarr[MAX_ADMIN_WORK_REQUESTS];
struct nvme_completion ncmplsarr[MAX_ADMIN_WORK_REQUESTS];

struct nvmr_ncommcont ncommcntarr[MAX_ADMIN_WORK_REQUESTS];
struct nvme_command ncommsarr[MAX_ADMIN_WORK_REQUESTS];

struct ib_mr *nsndmrarr[MAX_ADMIN_WORK_REQUESTS];

static struct rdma_cm_id *glbl_cmid = NULL;
static struct ib_device  *glbl_ibdev = NULL;
static struct ib_pd      *glbl_ibpd  = NULL;
static struct ib_cq      *glbl_ibcq  = NULL;
static struct ib_qp      *glbl_ibqp  = NULL;
/*
 * Invoked on event RDMA_CM_EVENT_ADDR_RESOLVED.  Invokes rdma_resolve_route()
 */
static void
nvmr_addr_resolved(struct rdma_cm_id *cm_id)
{
	int retval;
	struct ib_pd *ibpd;

	KASSERT(glbl_cmid == cm_id, ("Global CM id not the passed in CM id"));

	if (!(cm_id->device->attrs.device_cap_flags &
	    IB_DEVICE_MEM_MGT_EXTENSIONS)) {
		ERRSPEW("Memory management extensions not supported. 0x%lX\n",
		    cm_id->device->attrs.device_cap_flags);
		goto out;
	}

	ibpd  = ib_alloc_pd(cm_id->device, 0);
	if (IS_ERR(ibpd)) {
		ERRSPEW("ib_alloc_pd() failed: 0x%lx\n", PTR_ERR(ibpd));
		goto out;
	}
	glbl_ibpd  = ibpd;
	glbl_ibdev = cm_id->device;

	retval = rdma_resolve_route(cm_id, 2000);
	if (retval != 0) {
		ERRSPEW("rdma_resolve_route() failed w/%d\n", retval);
	} else {
		DBGSPEW("Successfully invoked rdma_resolve_route()\n");
	}
out:
	return;
}

/*
 * Invoked on event RDMA_CM_EVENT_ROUTE_RESOLVED.
 * A Queue-Pair to the RDMA sub-system is created along with a Completion-Queue
 * for its use internally.  An RDMA connection is established which generates
 * the first packets on the wite.  The NVMeoF RDMA spec requires that the
 * initiator pass along a parameters block in the private data section.
 */
static void
nvmr_route_resolved(struct rdma_cm_id *cm_id)
{
	int retval;
	struct ib_cq *ib_cq;
	struct ib_qp_init_attr init_attr;
	struct rdma_conn_param conn_param;
	nvmr_rdma_cm_request_t privdata;

	KASSERT(glbl_cmid == cm_id, ("Global CM id not the passed in CM id"));

	ib_cq = ib_alloc_cq(glbl_ibdev, NULL /* priv */,
	    (MAX_ADMIN_WORK_REQUESTS* 4) + 1, 0 /* completion vector */,
	    IB_POLL_WORKQUEUE);

	if (IS_ERR(ib_cq)) {
		ERRSPEW("ib_alloc_cq() failed with 0x%lX\n", PTR_ERR(ib_cq));
		goto out;
	} else {
		DBGSPEW("ib_alloc_cq() returned %p\n", ib_cq);
		glbl_ibcq = ib_cq;
	}

	KASSERT((glbl_ibcq != NULL) && (glbl_ibpd != NULL) &&
	    (glbl_ibdev != NULL), ("Global(s) NULL"));

	memset(&init_attr, 0, sizeof(init_attr));
	init_attr.cap.max_send_wr = (MAX_ADMIN_WORK_REQUESTS * 3) + 1;
	init_attr.cap.max_recv_wr = MAX_ADMIN_WORK_REQUESTS + 1;
	init_attr.cap.max_recv_sge = 1;
	init_attr.cap.max_send_sge = 1 + 1;
	init_attr.qp_type = IB_QPT_RC;
	init_attr.send_cq = glbl_ibcq;
	init_attr.recv_cq = glbl_ibcq;
	init_attr.sq_sig_type = IB_SIGNAL_REQ_WR;

	init_attr.event_handler = nvmr_qphndlr;

	retval = rdma_create_qp(glbl_cmid, glbl_ibpd, &init_attr);
	if (retval != 0) {
		ERRSPEW("rdma_create_qp() failed with %d\n", retval);
	} else {
		DBGSPEW("Successfully created QP!\n");
		glbl_ibqp = glbl_cmid->qp;
	}

	/*
	 * NB: The conn_param has to pass in an NVMeoF RDMA Private Data
	 * structure for the NVMeoF RDMA target to setup its Q sizes et al.
	 */
	memset(&conn_param, 0, sizeof(conn_param));
	memset(&privdata, 0, sizeof(privdata));
	privdata.nvmrcr_recfmt = 0;
	privdata.nvmrcr_qid = 0;
	privdata.nvmrcr_hrqsize = htole16(MAX_ADMIN_WORK_REQUESTS);
	privdata.nvmrcr_hsqsize = htole16(MAX_ADMIN_WORK_REQUESTS - 1);
	conn_param.responder_resources = glbl_ibdev->attrs.max_qp_rd_atom;
	conn_param.qp_num = glbl_ibqp->qp_num;
	conn_param.flow_control = 1;
	conn_param.retry_count = 3;
	conn_param.rnr_retry_count = 3;
	conn_param.private_data = &privdata;
	conn_param.private_data_len = sizeof(privdata);

	retval = rdma_connect(glbl_cmid, &conn_param);
	if (retval != 0) {
		ERRSPEW("rdma_connect() failed with %d\n", retval);
	} else {
		DBGSPEW("Wait for \"established\" after rdma_connected()...\n");
	}

out:
	return;
}


#define MAX_SGS	2
#define CTASSERT_MAX_SGS_LARGE_ENOUGH_FOR(data_structure) \
    CTASSERT(MAX_SGS >= ((sizeof(data_structure)/PAGE_SIZE)+1))

struct nvmrdma_connect_data {
	struct uuid nvmrcd_hostid;
	uint16_t    nvmrcd_cntlid;
	uint8_t     nvmrcd_resv0[238];
	uint8_t     nvmrcd_subnqn[256];
	uint8_t     nvmrcd_hostnqn[256];
	uint8_t     nvmrcd_resv1[256];
} __packed;
CTASSERT(sizeof(struct nvmrdma_connect_data) == 1024);
CTASSERT_MAX_SGS_LARGE_ENOUGH_FOR(struct nvmrdma_connect_data);

struct nvmrdma_connect_data glbl_ncdata;

static void
nvmr_recv_cmplhndlr(struct ib_cq *cq, struct ib_wc *wc)
{
	struct nvmr_ncmplcont *rcve;
	struct nvme_completion *c;

	rcve = container_of(wc->wr_cqe, struct nvmr_ncmplcont, nvmrsp_cqe);

	DBGSPEW("rcve:%p, wc_status:\"%s\"\n", rcve,
	    ib_wc_status_msg(wc->status));

	ib_dma_sync_single_for_cpu(glbl_ibdev, rcve->nvmrsp_dmaddr,
	    sizeof(*(rcve->nvmrsp_nvmecompl)), DMA_FROM_DEVICE);

	if (wc->status == IB_WC_SUCCESS) {
		c = rcve->nvmrsp_nvmecompl;
		DBGSPEW("c:0x%08X r:0x%08X hd:0x%04hX id:0x%04hX cid:0x%04hX "
		    "status:0x%04hX\n", c->cdw0, c->rsvd1, c->sqhd, c->sqid,
		    c->cid, c->status);
		if (c->status == 0) {
			DBGSPEW("NVMe Command succeeded for Host \"%s\"\n",
			    glbl_ncdata.nvmrcd_hostnqn);
		}
	}
}


#define NVMR_FOURK (4096)
#define NVMR_DYNANYCNTLID 0xFFFF

#define HOSTNQN_TEMPLATE "nqn.2014-08.org.nvmexpress:uuid:%s"
#define DISCOVERY_SUBNQN "nqn.2014-08.org.nvmexpress.discovery"

char nvrdma_host_uuid_string[80];
struct uuid nvrdma_host_uuid;

struct ib_cqe regcqe, sndcqe;

static void
nvmr_rg_done(struct ib_cq *cq, struct ib_wc *wc)
{
	struct ib_cqe *rcve;

	rcve = wc->wr_cqe;

	DBGSPEW("regcqe:%p, rcve:%p, wc_status:\"%s\"\n", &regcqe, rcve,
	    ib_wc_status_msg(wc->status));
}


typedef struct {
	uint8_t  nvmrcon_opcode;
	uint8_t  nvmrcon_sgl_fuse;
	uint16_t nvmrcon_cid;
	uint8_t  nvmrcon_fctype;
	uint8_t  nvmrcon_resv1[19];
	struct {
		uint64_t nvmrconk_address;
		uint8_t  nvmrconk_length[3];
		uint32_t nvmrconk_key;
		uint8_t  nvmrconk_sgl_identifier;
	} __packed;
	uint16_t nvmrcon_recfmt;
	uint16_t nvmrcon_qid;
	uint16_t nvmrcon_sqsize;
	uint8_t  nvmrcon_cattr;
	uint8_t  nvmrcon_resv2;
	uint32_t nvmrcon_kato;
	uint8_t  nvmrcon_resv3[12];
} __packed nvmr_connect_t;
CTASSERT(sizeof(nvmr_connect_t) == sizeof(struct nvme_command));

nvmr_connect_t glbl_connect;
#define NVMR_DEFAULT_KATO 0x1D4C0
#define NVMR_DISCOVERY_KATO 0x0 /* DISCOVERY KATO has to be 0 */
#define NVMF_FCTYPE_CONNECT 0x1
#define NVMR_PSDT_SHIFT 6
#define NVMF_SINGLE_BUF_SGL (0x1 << NVMR_PSDT_SHIFT)
#define NVMF_MULT_SEG_SGL   (0x2 << NVMR_PSDT_SHIFT)
#define NVMF_KEYED_SGL_NO_INVALIDATE 0x40
#define NVMF_KEYED_SGL_INVALIDATE    0x4F


static void
nvmr_snd_done(struct ib_cq *cq, struct ib_wc *wc)
{
	struct ib_cqe *rcve;

	rcve = wc->wr_cqe;

	DBGSPEW("ncommcntarr:%p, rcve:%p, wc_status:\"%s\"\n", &ncommcntarr,
	    rcve, ib_wc_status_msg(wc->status));
}


/*
 * Invoked on event RDMA_CM_EVENT_ESTABLISHED after the rdma_connect() and
 * associated data are accepted by the NVMeoF target.
 * Now we setup the Queue-Pair for Admin use:
 * 1) The Receive Queue will only receive nvme_completion messages
 * 2) The Send Queue will take nvme_command messages, with optional data
 *    represented in an SGL.  The latter means that Memory Registrations will
 *    be needed to describe the location of the data which will be sent over
 *    to the target. The target will RDMA this data over to itself.
 * 3) Posting Recv buffers (nvme_completion) and Send buffers (nvme_command) is
 *    collectively called posting Work Requests (WRs).  When posting WRs it
 *    is necessary to post a Completion Queue Entry structure to the RDMA
 *    stack which will be used internally in its CQ, unless we don't care
 *    to be notified of the WR having completed and the completion status.
 * 4) Memory Region (MR) Registrations are made by posting MR registration WRs
 *    to the Send Queue
 */
static void
nvmr_event_established(struct rdma_cm_id *cm_id)
{
	int retval, count, offset, n, nn, nnn;
	struct scatterlist scl[MAX_SGS], *s;
	struct ib_recv_wr rcvwr, *badrcvwrp;
	struct ib_send_wr sndwr, *badsndwrp;
	struct nvme_completion *cmplp;
	struct nvmr_ncmplcont *rcve;
	struct nvmr_ncommcont *snde;
	nvmr_connect_t *connectp;
	struct ib_reg_wr regwr;
	size_t len, translen;
	struct ib_sge sgl;
	struct ib_mr *mr;
	void *cbuf;
	u64 dmaddr;

	KASSERT(glbl_cmid == cm_id, ("Global CM id not the passed in CM id"));
	KASSERT((glbl_ibcq != NULL) && (glbl_ibpd != NULL) &&
	    (glbl_ibdev != NULL), ("Global(s) NULL"));

	/*
	 * The first loop maps the NVMe completion buffers, the next posts
	 * them to the IB RCV Q
	 */
	for (count = 0; count < MAX_ADMIN_WORK_REQUESTS; count++) {
		dmaddr =  ib_dma_map_single(glbl_ibdev, ncmplsarr + count,
		    sizeof(*(ncmplsarr + count)), DMA_FROM_DEVICE);
		if (ib_dma_mapping_error(glbl_ibdev, dmaddr) != 0) {
			ERRSPEW("ib_dma_map_single() failed for #%d\n", count);
			goto out;
		} else {
			(ncmplcntarr+count)->nvmrsp_dmaddr = dmaddr;
		}
		ib_dma_sync_single_for_device(glbl_ibdev, dmaddr,
		    sizeof(*(ncmplsarr + count)), DMA_FROM_DEVICE);
	}
	for (count = 0; count < MAX_ADMIN_WORK_REQUESTS; count++) {
		rcve = ncmplcntarr + count;
		cmplp = ncmplsarr + count;

		memset(cmplp, 0, sizeof(*cmplp));
		memset(&rcvwr, 0, sizeof(rcvwr));
		memset(&sgl, 0, sizeof(sgl));
		rcve->nvmrsp_nvmecompl = cmplp;

		sgl.addr   = rcve->nvmrsp_dmaddr;
		sgl.length = sizeof(*(rcve->nvmrsp_nvmecompl));
		sgl.lkey   = glbl_ibpd->local_dma_lkey;

		rcve->nvmrsp_cqe.done = nvmr_recv_cmplhndlr;

		rcvwr.sg_list = &sgl;
		rcvwr.num_sge = 1;
		rcvwr.wr_cqe  = &rcve->nvmrsp_cqe;
		rcvwr.next    = NULL;

		retval = ib_post_recv(glbl_ibqp, &rcvwr, &badrcvwrp);
		if (retval != 0) {
			ERRSPEW("ib_post_recv() failed for #%d with %d\n",
			    count, retval);
			goto out;
		}
	}
	DBGSPEW("Successfully posted receive buffers for NVMe completions\n");


	/*
	 * Allocate MR structures for use by any data that the NVMe commands
	 * posted to the IB SND Q need to describe
	 */
	for (count = 0; count < MAX_ADMIN_WORK_REQUESTS; count++) {
		mr = ib_alloc_mr(glbl_ibpd, IB_MR_TYPE_MEM_REG,
		    MAX_NVME_RDMA_SEGMENTS);
		if (IS_ERR(mr)) {
			ERRSPEW("ib_alloc_mr() failed with \"%ld\" for "
			    "count #%d\n", PTR_ERR(mr), count);
			goto out;
		}
		nsndmrarr[count] = mr;
	}
	DBGSPEW("Successfully allocated MRs for Admin commands\n");

	/*
	 * The next loop maps the NVMe command buffers
	 */
	for (count = 0; count < MAX_ADMIN_WORK_REQUESTS; count++) {
		dmaddr =  ib_dma_map_single(glbl_ibdev, ncommsarr + count,
		    sizeof(*(ncommsarr + count)), DMA_TO_DEVICE);
		if (ib_dma_mapping_error(glbl_ibdev, dmaddr) != 0) {
			ERRSPEW("ib_dma_map_single() failed for #%d\n", count);
			goto out;
		} else {
			(ncommcntarr+count)->nvmsnd_dmaddr = dmaddr;
		}
		/* Retain ownership with the driver until we actually use it */
		ib_dma_sync_single_for_cpu(glbl_ibdev, dmaddr,
		    sizeof(*(ncommsarr + count)), DMA_TO_DEVICE);
	}


	/*
	 * Craft a CONNECT command to begin discovery: Steps 1 through 5
	 */

	/*
	 * 1) Generate UUID et al for CONNECT Data.  Translate to a scatterlist
	 *    array of pages a la iser_buf_to_sg()
	 */
	kern_uuidgen(&nvrdma_host_uuid, 1);
	snprintf_uuid(nvrdma_host_uuid_string, sizeof(nvrdma_host_uuid_string),
	    &nvrdma_host_uuid);
	DBGSPEW("Generated UUID is \"%s\"\n", nvrdma_host_uuid_string);
	glbl_ncdata.nvmrcd_hostid = nvrdma_host_uuid;
	glbl_ncdata.nvmrcd_cntlid = htole16(NVMR_DYNANYCNTLID);
	snprintf(glbl_ncdata.nvmrcd_subnqn, sizeof(glbl_ncdata.nvmrcd_subnqn),
	    DISCOVERY_SUBNQN);
	snprintf(glbl_ncdata.nvmrcd_hostnqn, sizeof(glbl_ncdata.nvmrcd_hostnqn),
	    HOSTNQN_TEMPLATE, nvrdma_host_uuid_string);

	translen = sizeof(glbl_ncdata);
	cbuf = &glbl_ncdata;
	for (n = 0; (0 < translen) && (n < MAX_SGS); n++, translen -= len) {
		s = scl + n;
		offset = ((uintptr_t)cbuf) & ~PAGE_MASK;
		len = min(PAGE_SIZE - offset, translen);
		sg_set_buf(s, cbuf, len);
		cbuf = (void *)(((u64)cbuf) + (u64)len);
	}
	if (translen != 0) {
		ERRSPEW("Could not complete translation of CONNECT Data. n:%d "
		    "translen:%zu\n", n, translen);
		goto out;
	} else {
		DBGSPEW("&glbl_ncdata is 0x%p\n", &glbl_ncdata);
		for (count = 0; count < n; count++) {
			DBGSPEW("scl[%d](hex) p:%16lX o:%8X l:%8X a:%16lX\n",
			count,
			scl[count].page_link,
			scl[count].offset,
			scl[count].length,
			scl[count].address);
		}
	}

	/* 2) Map the scatterlist array per the restrictions of the IB device */
	nn = ib_dma_map_sg(glbl_ibdev, scl, n, DMA_TO_DEVICE);
	if (nn < 1) {
		ERRSPEW("ib_dma_map_sg() failed with count:%d\n", nn);
		goto out;
	} else {
		DBGSPEW("ib_dma_map_sg() returned a count of %d\n", nn);
	}

	/* 3) Map the scatterlist elements to the MR  */
	mr = nsndmrarr[0];
	nnn = ib_map_mr_sg(mr, scl, nn, NULL, NVMR_FOURK);
	if (nnn < nn) {
		ERRSPEW("ib_map_mr_sg() failed. nnn:%d < nn:%d\n", nnn, nn);
		goto out;
	} else {
		DBGSPEW("ib_map_mr_sg() returned a count of %d\n", nnn);
	}
	ib_update_fast_reg_key(mr, ib_inc_rkey(mr->rkey));

	/* 4) Craft the memory registration work-request but don't post it */
	regcqe.done = nvmr_rg_done;
	memset(&regwr, 0, sizeof(regwr));
	/* NB the Registration work-request contains a Send work-request */
	regwr.wr.num_sge = 0; /* No send/recv buffers are being posted */
	regwr.wr.send_flags = IB_SEND_SIGNALED; /* Invoke .done when done */
	regwr.wr.opcode = IB_WR_REG_MR;
	regwr.wr.wr_cqe = &regcqe;
	regwr.wr.next = NULL;

	regwr.access = IB_ACCESS_LOCAL_WRITE | IB_ACCESS_REMOTE_READ |
	    IB_ACCESS_REMOTE_WRITE;
	regwr.key = mr->rkey;
	regwr.mr = mr;

	/* 5) Craft an NVMeoF Connect Command and post it */
	ncommcntarr[0].nvmsnd_nvmecomm = &ncommsarr[0];
	snde = &ncommcntarr[0];
	KASSERT(sizeof(*connectp) == sizeof(*snde->nvmsnd_nvmecomm), ("Hm..."));
	connectp = (nvmr_connect_t *)snde->nvmsnd_nvmecomm;
	memset(connectp, 0, sizeof(*connectp));

	connectp->nvmrcon_opcode = NVME_OPC_FABRIC_COMMAND;
	connectp->nvmrcon_sgl_fuse = NVMF_SINGLE_BUF_SGL;
	connectp->nvmrcon_fctype = NVMF_FCTYPE_CONNECT;

	connectp->nvmrconk_address = htole64(mr->iova);
	connectp->nvmrconk_length[0] = htole32(mr->length) & 0xFF;
	connectp->nvmrconk_length[1] = (htole32(mr->length)>>8) & 0xFF;
	connectp->nvmrconk_length[2] = (htole32(mr->length)>>16) & 0xFF;
	connectp->nvmrconk_key = htole32(mr->rkey);
	connectp->nvmrconk_sgl_identifier = NVMF_KEYED_SGL_NO_INVALIDATE;

	connectp->nvmrcon_recfmt = 0;
	connectp->nvmrcon_qid = 0;
	connectp->nvmrcon_sqsize = MAX_ADMIN_WORK_REQUESTS - 1; /* 0 based */
	connectp->nvmrcon_cattr = 0;
	connectp->nvmrcon_kato = NVMR_DISCOVERY_KATO;

	memset(&sndwr, 0, sizeof(sndwr));
	memset(&sgl, 0, sizeof(sgl));

	sgl.addr   = snde->nvmsnd_dmaddr;
	sgl.length = sizeof(*(snde->nvmsnd_nvmecomm));
	sgl.lkey   = glbl_ibpd->local_dma_lkey;

	snde->nvmsnd_cqe.done = nvmr_snd_done;

	sndwr.next  = &regwr.wr; /* Chain the MR registration WR into send WR */
	sndwr.wr_cqe = &snde->nvmsnd_cqe;
	sndwr.sg_list = &sgl;
	sndwr.num_sge = 1;
	sndwr.opcode = IB_WR_SEND;
	sndwr.send_flags = IB_SEND_SIGNALED;
	ib_dma_sync_single_for_device(glbl_ibdev, snde->nvmsnd_dmaddr,
	    sizeof(*(snde->nvmsnd_nvmecomm)), DMA_TO_DEVICE);

	badsndwrp = NULL;
	retval = ib_post_send(glbl_ibqp, &sndwr, &badsndwrp);
	if (retval != 0) {
		ERRSPEW("ib_post_send(%p) failed with %d, badsndwrp:%p\n",
		    &sndwr, retval, badsndwrp);
		goto out;
	} else {
		DBGSPEW("ib_post_send(&sndwr) returned without incident\n");
	}

out:
	return;
}

static int
nvmr_cm_handler(struct rdma_cm_id *cm_id, struct rdma_cm_event *event)
{
	DBGSPEW("Event \"%s\" returned status \"%d\" for cm_id:%p\n",
	    rdma_event_msg(event->event), event->status, cm_id);

	KASSERT(glbl_cmid == cm_id, ("Global CM id not the passed in CM id"));

	switch(event->event) {
	case RDMA_CM_EVENT_ADDR_RESOLVED:
		nvmr_addr_resolved(cm_id);
		break;

	case RDMA_CM_EVENT_ROUTE_RESOLVED:
		nvmr_route_resolved(cm_id);
		break;

	case RDMA_CM_EVENT_ESTABLISHED:
		nvmr_event_established(cm_id);
		break;

	case RDMA_CM_EVENT_DISCONNECTED:
		break;

	default:
		dump_stack();
		break;
	}

	return 0;
}

#else /* VELOLD */

#define NUMIPV4OCTETS 4
typedef uint8_t nvmripv4_t[NUMIPV4OCTETS];

struct nvmr_cntrlr_tag;
typedef void (*nvmr_crtcntrlrcb_t)(struct nvmr_cntrlr_tag *cntrlr);

typedef struct {
	int      nvmrqp_numqueues;
	int      nvmrqp_numsndqe;
	int      nvmrqp_numrcvqe;
	int      nvmrqp_numsndsge;
	int      nvmrqp_numrcvsge;
	uint16_t nvmrqp_pdnumsndqsz;
	uint16_t nvmrqp_pdnumrcvqsz;
} nvmr_qprof_t;

typedef enum {
	NVMR_QTYPE_ADMIN = 0,
	NVMR_QTYPE_IO,
	NVMR_NUM_QTYPES
} nvmr_qndx_t;

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

typedef struct {
	nvmr_qprof_t	   nvmrp_qprofs[NVMR_NUM_QTYPES];
	nvmr_crtcntrlrcb_t nvmrp_cbfunc;
} nvmr_cntrlrprof_t;

typedef enum {
	NVMRQ_PRE_INIT = 0,
	NVMRQ_PRE_ADDR_RESOLV,
	NVMRQ_ADDR_RESOLV_FAILED,
	NVMRQ_ADDR_RESOLV_SUCCEEDED,
	NVMRQ_PRE_ROUTE_RESOLV,
	NVMRQ_ROUTE_RESOLV_FAILED,
	NVMRQ_ROUTE_RESOLV_SUCCEEDED,
	NVMRQ_PRE_CONNECT,
	NVMRQ_CONNECT_FAILED,
	NVMRQ_CONNECT_SUCCEEDED,
} nvmr_queue_state_t;

typedef struct {
	struct rdma_cm_id             *nvmrq_cmid;
	struct nvmr_cntrlr_tag        *nvmrq_cntrlr; /* Owning Controller */
	struct ib_pd                  *nvmrq_ibpd;
	struct ib_cq                  *nvmrq_ibcq;
	struct ib_qp                  *nvmrq_ibqp;
	nvmr_qprof_t                  *nvmrq_prof;
	STAILQ_HEAD(, nvmr_ncommcont)  nvmrq_comms;
	STAILQ_HEAD(, nvmr_ncmplcont)  nvmrq_cmpls;
	volatile nvmr_queue_state_t    nvmrq_state;  /* nvmrctr_lock protects */
	int                            nvmrq_last_cm_status;
	uint16_t                       nvmrq_numsndqe;/* nvmrq_comms count */
	uint16_t                       nvmrq_numrcvqe;/* nvmrq_cmpls count */
} *nvmr_queue_t;

typedef struct nvmr_cntrlr_tag {
	struct mtx         nvmrctr_lock;
	nvmr_queue_t      *nvmrctr_qarr;  /* Array size determined by prof */
	nvmr_cntrlrprof_t *nvmrctr_prof;
	nvmripv4_t         nvmrctr_ipv4;
	int                nvmrctr_numqs; /* Q count not always fixed in prof */
	volatile int       nvmrctr_refcount;
	uint16_t           nvmrctr_port;
} *nvmr_cntrlr_t;

static void
nvmr_discov_cntrlr(nvmr_cntrlr_t cntrlr)
{
	return;
}

nvmr_cntrlrprof_t nvmr_discoveryprof = {
	.nvmrp_qprofs[NVMR_QTYPE_ADMIN] = {
		.nvmrqp_numqueues = 1,

		.nvmrqp_numsndqe = (3 * MAX_ADMIN_WORK_REQUESTS) + 1,
		.nvmrqp_numsndsge = 1 + 1, /* NVMe Command  + Register MR */
		.nvmrqp_pdnumsndqsz = MAX_ADMIN_WORK_REQUESTS - 1,

		.nvmrqp_numrcvqe = MAX_ADMIN_WORK_REQUESTS + 1,
		.nvmrqp_numrcvsge = 1, /* NVMe Completion */
		.nvmrqp_pdnumrcvqsz = MAX_ADMIN_WORK_REQUESTS,
	},
	.nvmrp_cbfunc   = nvmr_discov_cntrlr,
};

static MALLOC_DEFINE(M_NVMR, "nvmr", "nvmr");

static void
nvmr_recv_cmplhndlr(struct ib_cq *cq, struct ib_wc *wc)
{
	struct nvmr_ncmplcont *rcve;
	struct nvme_completion *c;
	nvmr_queue_t q;

	q = cq->cq_context;
	rcve = container_of(wc->wr_cqe, struct nvmr_ncmplcont, nvmrsp_cqe);

	DBGSPEW("rcve:%p, wc_status:\"%s\". Repost me!\n", rcve,
	    ib_wc_status_msg(wc->status));

	ib_dma_sync_single_for_cpu(q->nvmrq_cmid->device, rcve->nvmrsp_dmaddr,
	    sizeof(*(rcve->nvmrsp_nvmecompl)), DMA_FROM_DEVICE);

	if (wc->status == IB_WC_SUCCESS) {
		c = rcve->nvmrsp_nvmecompl;
		DBGSPEW("c:0x%08X r:0x%08X hd:0x%04hX id:0x%04hX cid:0x%04hX "
		    "status:0x%04hX\n", c->cdw0, c->rsvd1, c->sqhd, c->sqid,
		    c->cid, c->status);
		if (c->status == 0) {
			DBGSPEW("NVMe Command succeeded\n");
		}
	}
}

static void
nvmr_queue_destroy(nvmr_queue_t q)
{
	nvmr_ncmplcon_t *cmplp, *tcmplp;
	nvmr_ncommcon_t *commp, *tcommp;
	struct ib_device *ibd;
	int count;

	if (q == NULL) {
		goto out;
	}
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

	count = 0;
	STAILQ_FOREACH_SAFE(commp, &q->nvmrq_comms, nvmsnd_next, tcommp) {
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


static void
nvmr_cntrlr_destroy(nvmr_cntrlr_t cntrlr)
{
	int count;

	if (cntrlr == NULL) {
		goto out;
	}

	if (cntrlr->nvmrctr_qarr != NULL) {
		for (count = 0; count < cntrlr->nvmrctr_numqs; count++) {
			nvmr_queue_destroy(cntrlr->nvmrctr_qarr[count]);
			cntrlr->nvmrctr_qarr[count] = NULL;
		}
		DBGSPEW("free(%p)ing Q array\n", cntrlr->nvmrctr_qarr);
		free(cntrlr->nvmrctr_qarr, M_NVMR);
	}

	DBGSPEW("free(%p)ing NVMr controller\n", cntrlr);
	free(cntrlr, M_NVMR);
out:
	return;
}


static void
nvmr_cntrlr_rele(nvmr_cntrlr_t cntrlr)
{
	int retval;

	retval = atomic_fetchadd_int(&cntrlr->nvmrctr_refcount, -1);
	KASSERT(retval > 0, ("%s@%d refcount:%d cntrlr:%p\n", __func__,
	    __LINE__, retval, cntrlr));
	if (retval == 1) {
		nvmr_cntrlr_destroy(cntrlr);
	}
}


static void
nvmr_cntrlr_ref(nvmr_cntrlr_t cntrlr)
{
	int retval;

	retval = atomic_fetchadd_int(&cntrlr->nvmrctr_refcount, 1);
	KASSERT(retval > 0, ("%s@%d refcount:%d cntrlr:%p\n", __func__,
	    __LINE__, retval, cntrlr));
}


static void
nvmr_cm_addr_resolved(nvmr_queue_t q)
{
	mtx_lock(&q->nvmrq_cntrlr->nvmrctr_lock);
	q->nvmrq_state = NVMRQ_ADDR_RESOLV_SUCCEEDED;
	mtx_unlock(&q->nvmrq_cntrlr->nvmrctr_lock);
	wakeup(q->nvmrq_cntrlr);

	nvmr_cntrlr_rele(q->nvmrq_cntrlr);

	return;
}

static void
nvmr_cm_addr_error(nvmr_queue_t q)
{
	mtx_lock(&q->nvmrq_cntrlr->nvmrctr_lock);
	q->nvmrq_state = NVMRQ_ADDR_RESOLV_FAILED;
	mtx_unlock(&q->nvmrq_cntrlr->nvmrctr_lock);
	wakeup(q->nvmrq_cntrlr);

	nvmr_cntrlr_rele(q->nvmrq_cntrlr);

	return;
}

static void
nvmr_cm_route_resolved(nvmr_queue_t q)
{
	mtx_lock(&q->nvmrq_cntrlr->nvmrctr_lock);
	q->nvmrq_state = NVMRQ_ROUTE_RESOLV_SUCCEEDED;
	mtx_unlock(&q->nvmrq_cntrlr->nvmrctr_lock);
	wakeup(q->nvmrq_cntrlr);

	nvmr_cntrlr_rele(q->nvmrq_cntrlr);

	return;
}

static void
nvmr_cm_route_error(nvmr_queue_t q)
{
	mtx_lock(&q->nvmrq_cntrlr->nvmrctr_lock);
	q->nvmrq_state = NVMRQ_ROUTE_RESOLV_FAILED;
	mtx_unlock(&q->nvmrq_cntrlr->nvmrctr_lock);
	wakeup(q->nvmrq_cntrlr);

	nvmr_cntrlr_rele(q->nvmrq_cntrlr);

	return;
}

static void
nvmr_cm_established(nvmr_queue_t q)
{
	mtx_lock(&q->nvmrq_cntrlr->nvmrctr_lock);
	q->nvmrq_state = NVMRQ_CONNECT_SUCCEEDED;
	mtx_unlock(&q->nvmrq_cntrlr->nvmrctr_lock);
	wakeup(q->nvmrq_cntrlr);

	nvmr_cntrlr_rele(q->nvmrq_cntrlr);

	return;
}

static void
nvmr_cm_rejected(nvmr_queue_t q)
{
	mtx_lock(&q->nvmrq_cntrlr->nvmrctr_lock);
	q->nvmrq_state = NVMRQ_CONNECT_FAILED;
	mtx_unlock(&q->nvmrq_cntrlr->nvmrctr_lock);
	wakeup(q->nvmrq_cntrlr);

	nvmr_cntrlr_rele(q->nvmrq_cntrlr);

	return;
}

static void
nvmr_cm_unreachable(nvmr_queue_t q)
{
	mtx_lock(&q->nvmrq_cntrlr->nvmrctr_lock);
	q->nvmrq_state = NVMRQ_CONNECT_FAILED;
	mtx_unlock(&q->nvmrq_cntrlr->nvmrctr_lock);
	wakeup(q->nvmrq_cntrlr);

	nvmr_cntrlr_rele(q->nvmrq_cntrlr);

	return;
}


static int
nvmr_connmgmt_handler(struct rdma_cm_id *cmid, struct rdma_cm_event *event)
{
	nvmr_queue_t q;
	const nvmr_rdma_cm_reject_t *ps;

	DBGSPEW("Event \"%s\" returned status \"%d\" for cmid:%p\n",
	    rdma_event_msg(event->event), event->status, cmid);

	/* Every cmid is associated with a controller or a queue? */
	q = cmid->context;
	q->nvmrq_last_cm_status = event->status;

	switch(event->event) {
	case RDMA_CM_EVENT_ADDR_RESOLVED:
		nvmr_cm_addr_resolved(q);
		break;

	case RDMA_CM_EVENT_ADDR_ERROR:
		nvmr_cm_addr_error(q);
		break;

	case RDMA_CM_EVENT_ROUTE_RESOLVED:
		nvmr_cm_route_resolved(q);
		break;

	case RDMA_CM_EVENT_ROUTE_ERROR:
		nvmr_cm_route_error(q);
		break;

	case RDMA_CM_EVENT_ESTABLISHED:
		nvmr_cm_established(q);
		break;

	case RDMA_CM_EVENT_UNREACHABLE:
		nvmr_cm_unreachable(q);
		break;

	case RDMA_CM_EVENT_REJECTED:
		ps = (const nvmr_rdma_cm_reject_t *)event->param.conn.private_data;
		DBGSPEW("recfmt:%hu sts:%hu\n", ps->nvmrcrj_recfmt,
		    ps->nvmrcrj_sts);
		nvmr_cm_rejected(q);
		break;

	case RDMA_CM_EVENT_DISCONNECTED:
		break;

	default:
		dump_stack();
		break;
	}

	if (event->status != 0) {
		dump_stack();
	}

	return 0;
}

typedef struct {
	char *nvmra_ipaddr;
	char *nvmra_port;
} nvmr_addr_t;

#define NVMRTO 3000

#define PRE_ASYNC_CM_INVOCATION(pre_state) \
	nvmr_cntrlr_ref(cntrlr); \
	q->nvmrq_state = (pre_state);

#define POST_ASYNC_CM_INVOCATION(routine, pre_state, success_state)        \
	if (retval != 0) {                                                 \
		ERRSPEW("Failed, %s()> %d\n", routine, retval);            \
		error = retval;                                            \
		goto out;                                                  \
	}                                                                  \
	DBGSPEW("Successfully invoked %s()\n", routine);                   \
	mtx_lock(&cntrlr->nvmrctr_lock);                                   \
	if (q->nvmrq_state == (pre_state)) {                               \
		DBGSPEW("Sleeping with message \"%s\"\n",                  \
		    __stringify(__LINE__));                                \
		retval = mtx_sleep(cntrlr, &cntrlr->nvmrctr_lock, 0,       \
		    __stringify(__LINE__), NVMRTO+1000);                   \
		mtx_unlock(&cntrlr->nvmrctr_lock);                         \
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
		mtx_unlock(&cntrlr->nvmrctr_lock);                         \
	}                                                                  \
	if (q->nvmrq_state < (success_state)) {                            \
		error = q->nvmrq_last_cm_status;                           \
		goto out;                                                  \
	}                                                                  \

	
static int
nvmr_create_queue(nvmr_qprof_t *prof, nvmr_cntrlr_t cntrlr, nvmr_queue_t *qp)
{
	u64 dmaddr;
	struct ib_recv_wr rcvwr, *badrcvwrp;
	struct ib_sge sgl;
	struct nvme_completion *ncmp;
	struct ib_device *ibd;
	struct sockaddr_storage saddr;
	struct sockaddr_in *sin4;
	struct rdma_cm_id *cmid;
	int error, retval, count;
	nvmr_ncmplcon_t *cmplp;
	nvmr_ncommcon_t *commp;
	struct ib_pd *ibpd;
	nvmr_queue_t q;
	struct ib_qp_init_attr init_attr;
	struct rdma_conn_param conn_param;
	nvmr_rdma_cm_request_t privdata;
	struct ib_cq *ibcq;

	sin4 = (struct sockaddr_in *)&saddr;

	q = malloc(sizeof *q, M_NVMR, M_WAITOK|M_ZERO);
	if (q == NULL) {
		ERRSPEW("NVMr Q allocation sized \"%zu\" failed\n",
		    sizeof *q);
		error = ENOMEM;
		goto out;
	}
	DBGSPEW("NVMr Q allocation of size \"%zu\"\n", sizeof *q);
	q->nvmrq_cntrlr = cntrlr;
	q->nvmrq_state = NVMRQ_PRE_INIT;
	q->nvmrq_prof = prof;
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
	DBGSPEW("rdma_create_id() succeeded:%p\n", q->nvmrq_cmid);
	/*
	ibd = cmid->device;
	if (ibd == NULL) {
		DBGSPEW("ibd is NULL\n");
		goto out;
	}
	 */

	for (count = 0; count < prof->nvmrqp_numsndqe; count++) {
		commp = malloc(sizeof(*commp),  M_NVMR, M_WAITOK|M_ZERO);
		if (commp == NULL) {
			ERRSPEW("Command Q container allocation failed after"
			    " %d iterations\n", count);
			error = ENOMEM;
			/* For now bail.  This needs to be more robust */
			goto out;
		} else {
			STAILQ_INSERT_HEAD(&q->nvmrq_comms, commp, nvmsnd_next);
			q->nvmrq_numsndqe++;
		}
	}
	DBGSPEW("Alloced %d command Q containers\n", q->nvmrq_numsndqe);

	for (count = 0; count < prof->nvmrqp_numrcvqe; count++) {
		cmplp = malloc(sizeof(*cmplp),  M_NVMR, M_WAITOK|M_ZERO);
		if (cmplp == NULL) {
			ERRSPEW("Completion Q container allocation failed after"
			    " %d iterations\n", count);
			error = ENOMEM;
			/* For now bail.  This needs to be more robust */
			goto out;
		}

		STAILQ_INSERT_HEAD(&q->nvmrq_cmpls, cmplp, nvmrsp_next);
		q->nvmrq_numrcvqe++;
	}
	DBGSPEW("Allocated %d completion Q containers\n", q->nvmrq_numrcvqe);


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
	 * will clean up the q as well.  nvmr_queue_destroy() can no longer be
	 * called except by nvmr_cntrlr_destroy()
	 */
	*qp = q;
	PRE_ASYNC_CM_INVOCATION(NVMRQ_PRE_ADDR_RESOLV);
	retval = rdma_resolve_addr(cmid, NULL, (struct sockaddr *)sin4, NVMRTO);
	POST_ASYNC_CM_INVOCATION(__stringify(rdma_resolve_addr),
	    NVMRQ_PRE_ADDR_RESOLV, NVMRQ_ADDR_RESOLV_SUCCEEDED);

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
	DBGSPEW("Associated %d completion containers\n", q->nvmrq_numrcvqe);


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

	ibcq = ib_alloc_cq(ibd, q,
	    q->nvmrq_numrcvqe + q->nvmrq_numsndqe, 0 /* completion vector */,
	    IB_POLL_WORKQUEUE);
	if (IS_ERR(ibcq)) {
		ERRSPEW("ib_alloc_cq() failed with 0x%lX\n", PTR_ERR(ibcq));
		error = ESPIPE;
		goto out;
	}
	DBGSPEW("ib_alloc_cq() returned %p\n", ibcq);
	q->nvmrq_ibcq = ibcq;

	memset(&init_attr, 0, sizeof(init_attr));
	init_attr.cap.max_send_wr = q->nvmrq_numsndqe;
	init_attr.cap.max_recv_wr = q->nvmrq_numrcvqe;
	init_attr.cap.max_recv_sge = prof->nvmrqp_numrcvsge;
	init_attr.cap.max_send_sge = prof->nvmrqp_numsndsge;
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
	DBGSPEW("Successfully created QP!\n");
	q->nvmrq_ibqp = cmid->qp;

	/*
	 * NB: The conn_param has to pass in an NVMeoF RDMA Private Data
	 * structure for the NVMeoF RDMA target to setup its Q sizes et al.
	 */
	memset(&conn_param, 0, sizeof(conn_param));
	memset(&privdata, 0, sizeof(privdata));
	privdata.nvmrcr_recfmt = 0;
	privdata.nvmrcr_qid = 0;
	privdata.nvmrcr_hrqsize = htole16(prof->nvmrqp_pdnumrcvqsz);
	privdata.nvmrcr_hsqsize = htole16(prof->nvmrqp_pdnumsndqsz);
	privdata.nvmrcr_hsqsize = 2000;
	conn_param.responder_resources = ibd->attrs.max_qp_rd_atom;
	conn_param.qp_num = q->nvmrq_ibqp->qp_num;
	conn_param.flow_control = 1;
	conn_param.retry_count = 3;
	conn_param.rnr_retry_count = 3;
	conn_param.private_data = &privdata;
	conn_param.private_data_len = sizeof(privdata);

	STAILQ_FOREACH(cmplp, &q->nvmrq_cmpls, nvmrsp_next) {
		ncmp = &cmplp->nvmrsp_nvmecmpl;
		memset(ncmp, 0, sizeof(*ncmp));
		memset(&rcvwr, 0, sizeof(rcvwr));
		memset(&sgl, 0, sizeof(sgl));
		cmplp->nvmrsp_nvmecompl = ncmp;

		sgl.addr   = cmplp->nvmrsp_dmaddr;
		sgl.length = sizeof(*(cmplp->nvmrsp_nvmecompl));
		sgl.lkey   = ibpd->local_dma_lkey;

		cmplp->nvmrsp_cqe.done = nvmr_recv_cmplhndlr;

		rcvwr.sg_list = &sgl;
		rcvwr.num_sge = 1;
		rcvwr.wr_cqe  = &cmplp->nvmrsp_cqe;
		rcvwr.next    = NULL;

		ib_dma_sync_single_for_device(ibd, cmplp->nvmrsp_dmaddr,
		    sizeof(cmplp->nvmrsp_nvmecmpl), DMA_FROM_DEVICE);
		retval = ib_post_recv(q->nvmrq_ibqp, &rcvwr, &badrcvwrp);
		if (retval != 0) {
			ERRSPEW("ib_post_recv() failed for #%d with %d\n",
			    count, retval);
			error = ENOMSG;
			goto out;
		}
	}
	KASSERT(count == q->nvmrq_numrcvqe, ("%s@%d count:%d numrcvqe:%d",
	    __func__, __LINE__, count, q->nvmrq_numrcvqe));
	DBGSPEW("Associated %d completion containers\n", q->nvmrq_numrcvqe);

	PRE_ASYNC_CM_INVOCATION(NVMRQ_PRE_CONNECT);
	retval = rdma_connect(cmid, &conn_param);
	POST_ASYNC_CM_INVOCATION(__stringify(rdma_connect),
	    NVMRQ_PRE_CONNECT, NVMRQ_CONNECT_SUCCEEDED);

	error = 0;

out:
	if ((error != 0) && (q->nvmrq_state < NVMRQ_PRE_ADDR_RESOLV)) {
		/* Cleanup the Q because nvmr_cntrlr_destroy() won't see it */
		nvmr_queue_destroy(q);
	}

	return error;
}


static int
nvmr_cntrlr_create(nvmr_addr_t *addr, nvmr_cntrlrprof_t *prof,
    nvmr_cntrlr_t *retcntrlrp)
{
	int error, retval, count, qcount, qarrndx;
	nvmr_cntrlr_t cntrlr;
	nvmr_queue_t *qarr;
	nvmr_qndx_t qtndx;
	unsigned long tmp;
	nvmripv4_t ipv4;
	uint16_t port;
	char *retp;

	if (inet_pton(AF_INET, addr->nvmra_ipaddr, &ipv4) != 1) {
		ERRSPEW("Parsing failed for IPV4 address \"%s\"\n",
		    addr->nvmra_ipaddr);
		error = EINVAL;
		goto out;
	}

	tmp = strtoul(addr->nvmra_port, &retp, 0);
	if ((*retp != '\0') || (tmp > UINT16_MAX)) {
		ERRSPEW("Parsing failed with %lu for RDMA port \"%s\"\n", tmp,
		    addr->nvmra_port);
		error = EINVAL;
		goto out;
	}
	port = htons((uint16_t)tmp);
	
	cntrlr = malloc(sizeof(*cntrlr), M_NVMR, M_WAITOK|M_ZERO);
	if (cntrlr == NULL) {
		ERRSPEW("Controller allocation sized \"%zu\" failed\n",
		    sizeof(*cntrlr));
		error = ENOMEM;
		goto out;
	}
	DBGSPEW("Controller allocation of size \"%zu\"\n", sizeof(*cntrlr));
	memcpy(&cntrlr->nvmrctr_ipv4, ipv4, sizeof(cntrlr->nvmrctr_ipv4));
	cntrlr->nvmrctr_port = port;
	cntrlr->nvmrctr_prof = prof;
	mtx_init(&cntrlr->nvmrctr_lock, "NVMr Controller Lock", NULL, MTX_DEF);
	cntrlr->nvmrctr_refcount = 1;

	qcount = 0;
	for (qtndx = 0; qtndx < NVMR_NUM_QTYPES; qtndx++) {
		for (count = 0;
		    count < prof->nvmrp_qprofs[qtndx].nvmrqp_numqueues;
		    count++) {
			qcount++;
		}
	}
	DBGSPEW("Q count is %d\n", qcount);
	cntrlr->nvmrctr_numqs = qcount;

	qarr = malloc(qcount * sizeof(nvmr_queue_t), M_NVMR, M_WAITOK|M_ZERO);
	if (qarr == NULL) {
		ERRSPEW("Controller Q array allocation sized \"%zu\" failed\n",
		    qcount * sizeof(nvmr_queue_t));
		error = ENOMEM;
		goto out;
	}
	DBGSPEW("Controller Q array allocation of size \"%zu\"\n",
	    qcount * sizeof(nvmr_queue_t));
	cntrlr->nvmrctr_qarr = qarr;

	/* Allocate the queues and store pointers to them in the qarr */
	qarrndx = 0;
	for (qtndx = 0; qtndx < NVMR_NUM_QTYPES; qtndx++) {
		for (count = 0;
		    count < prof->nvmrp_qprofs[qtndx].nvmrqp_numqueues;
		    count++) {
			retval = nvmr_create_queue(&prof->nvmrp_qprofs[qtndx],
			    cntrlr, &qarr[qarrndx]);
			if (retval != 0) {
				ERRSPEW("%s#%d creation failed with %d\n",
				    nvmr_qndx2name(qtndx), count, retval);
				break;
			} else {
				qarrndx++;
			}
		}
		if (count != prof->nvmrp_qprofs[qtndx].nvmrqp_numqueues) {
			error = retval;
			goto out;
		}
	}
	KASSERT(qarrndx == cntrlr->nvmrctr_numqs,("qarrndx:%d numqs:%d",
	    qarrndx, cntrlr->nvmrctr_numqs));

	error = 0;
	*retcntrlrp = cntrlr;

out:
	if (error == 0) {
		goto outret;
	}

	ERRSPEW("Returning with error \"%d\"\n", error);

	nvmr_cntrlr_rele(cntrlr);
	*retcntrlrp = NULL;

outret:
	return error;
}

#endif /* VELOLD */

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

nvmr_addr_t r640gent07eno1 = {
	"10.1.87.194", "4420"
};

nvmr_addr_t r640gent07enp94s0f1 = {
	"11.10.10.200", "4420"
};

static nvmr_cntrlr_t glbl_discovctrlrp;

static void
nvmr_init(void)
{
	int retval;
#ifdef VELOLD
	struct rdma_cm_id *cmid;
	struct sockaddr_storage saddr;
	struct sockaddr_in *sin4;
	uint32_t ipaddr;
	uint16_t rdmaport;

	cmid = NULL;
#else /* VELOLD */
#endif /* VELOLD */

	retval = ib_register_client(&nvmr_ib_client);
	if (retval != 0) {
		ERRSPEW("ib_register_client() for NVMeoF failed, ret:%d\n",
		    retval);
		goto out;
	}

#ifdef VELOLD
	sin4 = (struct sockaddr_in *)&saddr;
	/* rdmaport = 0x4411; */
	/* rdmaport = 0x0E28; */
	rdmaport = 0x4411;
	/* ipaddr = 0x0A0A0A0AU; */
	/* ipaddr = 0xC80A0A0BU; 11.10.10.200 */
	/* ipaddr = 0xC257010AU; 10.1.87.194 */
	ipaddr = 0xC257010AU;

	cmid = rdma_create_id(TD_TO_VNET(curthread), nvmr_cm_handler,
	    nvmr_init, RDMA_PS_TCP, IB_QPT_RC);
	if (IS_ERR(cmid)) {
		ERRSPEW("rdma_create_id() failed:%ld\n", PTR_ERR(cmid));
		cmid = NULL;
		goto out;
	} else {
		DBGSPEW("rdma_create_id() succeeded:%p\n", cmid);
	}

	memset(&saddr, 0, sizeof(saddr));
	sin4->sin_len = sizeof(*sin4);
	sin4->sin_family = AF_INET;
	memcpy((void *)&sin4->sin_addr.s_addr, &ipaddr, sizeof(ipaddr));
	sin4->sin_port = rdmaport;
	retval = rdma_resolve_addr(cmid, NULL, (struct sockaddr *)sin4, 2000);
	if (retval != 0) {
		ERRSPEW("Failed, rdma_resolve_addr()> %d\n", retval);
		goto out;
	}
	DBGSPEW("Successfully invoked rdma_resolve_addr()\n");
	glbl_cmid = cmid;
	cmid = NULL;

#else /* VELOLD */
	
	retval = nvmr_cntrlr_create(&r640gent07enp94s0f1, &nvmr_discoveryprof,
	    &glbl_discovctrlrp);
	if (retval != 0) {
		ERRSPEW("nvmr_cntrlr_create(\"%s\", \"%s\") failed with %d\n",
		    r640gent07eno1.nvmra_ipaddr, r640gent07eno1.nvmra_port,
		    retval);
		goto out;
	}

#endif /* VELOLD */

out:

#ifdef VELOLD
	if (cmid != NULL) {
		rdma_destroy_id(cmid);
	}
#else /* VELOLD */
#endif /* VELOLD */

	return;
}


static void
nvmr_uninit(void)
{
	DBGSPEW("Uninit invoked\n");

#ifdef VELOLD
	int count;

	DBGSPEW("Invoking ib_dereg_mr() on non-NULL MR elements...\n");
	for (count = 0; count < MAX_ADMIN_WORK_REQUESTS; count++) {
		if (nsndmrarr[count] == NULL) {
			continue;
		}
		ib_dereg_mr(nsndmrarr[count]);
		nsndmrarr[count] = NULL;
	}

	if ((glbl_ibcq != NULL) && (glbl_ibpd != NULL) && (glbl_cmid != NULL)) {
		DBGSPEW("Invoking ib_drain_qp(%p)...\n", glbl_ibqp);
		ib_drain_qp(glbl_ibqp);

		DBGSPEW("Invoking ib_dma_unmap_single()s on NVMe "
		    "completions...\n");
		for (count = 0; count < MAX_ADMIN_WORK_REQUESTS; count++) {
			ib_dma_unmap_single(glbl_ibdev,
			    (ncmplcntarr+count)->nvmrsp_dmaddr,
			    sizeof(*(ncmplsarr + count)), DMA_FROM_DEVICE);
		}

		DBGSPEW("Invoking ib_dma_unmap_single()s on NVMe "
		    "commands...\n");
		for (count = 0; count < MAX_ADMIN_WORK_REQUESTS; count++) {
			ib_dma_unmap_single(glbl_ibdev,
			    (ncommcntarr+count)->nvmsnd_dmaddr,
			    sizeof(*(ncommsarr + count)), DMA_TO_DEVICE);
		}

		DBGSPEW("Invoking rdma_disconnect(%p)...\n", glbl_cmid);
		rdma_disconnect(glbl_cmid);
	}

	if (glbl_ibqp != NULL) {
		DBGSPEW("Invoking rdma_destroy_qp(%p)...\n", glbl_cmid);
		rdma_destroy_qp(glbl_cmid);
		glbl_ibqp = NULL;
	}

	if (glbl_ibcq != NULL) {
		DBGSPEW("Invoking ib_free_cq(%p)...\n", glbl_ibcq);
		ib_free_cq(glbl_ibcq);
		glbl_ibcq = NULL;
	}

	if (glbl_ibpd != NULL) {
		DBGSPEW("Invoking ib_dealloc_pd(%p)...\n", glbl_ibpd);
		ib_dealloc_pd(glbl_ibpd);
		glbl_ibpd = NULL;
	}

	if (glbl_cmid != NULL) {
		DBGSPEW("Invoking rdma_destroy_id(%p)...\n", glbl_cmid);
		rdma_destroy_id(glbl_cmid);
		glbl_cmid = NULL;
	}

#else /* VELOLD */
	if (glbl_discovctrlrp == NULL) {
		goto out;
	}
	
	nvmr_cntrlr_rele(glbl_discovctrlrp);
	glbl_discovctrlrp = NULL;
	
#endif /* VELOLD */
out:
	ib_unregister_client(&nvmr_ib_client);
}


SYSINIT(nvmr, SI_SUB_DRIVERS, SI_ORDER_ANY, nvmr_init, NULL);
SYSUNINIT(nvmr, SI_SUB_DRIVERS, SI_ORDER_ANY, nvmr_uninit, NULL);
MODULE_DEPEND(nvmr, linuxkpi, 1, 1, 1);
MODULE_DEPEND(nvmr, ibcore, 1, 1, 1);
MODULE_VERSION(nvmr, 1);
