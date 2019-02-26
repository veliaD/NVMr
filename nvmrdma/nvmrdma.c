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
	struct ib_cqe		nvmrsp_cqe;
	struct nvme_completion *nvmrsp_nvmecompl;
	u64			nvmrsp_dmaddr;
};

struct nvmr_ncommcont {
	struct ib_cqe		nvmsnd_cqe;
	struct nvme_command     *nvmsnd_nvmecoomm;
	u64			nvmsnd_dmaddr;
};

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

static void
nvmr_qphndlr(struct ib_event *ev, void *ctx)
{
	ERRSPEW("Event \"%s\" on QP:%p\n", ib_event_msg(ev->event), ctx);
}

static void
nvmr_route_resolved(struct rdma_cm_id *cm_id)
{
	int retval;
	struct ib_cq *ib_cq;
	struct ib_qp_init_attr init_attr;
	struct rdma_conn_param conn_param;

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

	memset(&conn_param, 0, sizeof conn_param);
	conn_param.responder_resources = 1;
	conn_param.initiator_depth = 1;
	conn_param.retry_count = 1;

	retval = rdma_connect(glbl_cmid, &conn_param);
	if (retval != 0) {
		ERRSPEW("rdma_connect() failed with %d\n", retval);
	} else {
		DBGSPEW("Wait for \"established\" after rdma_connected()...\n");
	}

out:
	return;
}


static void
nvmr_recv_cmplhndlr(struct ib_cq *cq, struct ib_wc *wc)
{
	struct nvmr_ncmplcont *rcve;

	rcve = container_of(wc->wr_cqe, struct nvmr_ncmplcont, nvmrsp_cqe);

	DBGSPEW("rcve:%p, wc_status:\"%s\"\n", rcve,
	    ib_wc_status_msg(wc->status));

	ib_dma_sync_single_for_cpu(glbl_ibdev, rcve->nvmrsp_dmaddr,
	    sizeof(*(rcve->nvmrsp_nvmecompl)), DMA_FROM_DEVICE);
}


struct nvmrdma_connect_data {
	struct uuid nvmrcd_hostid;
	uint16_t    nvmrcd_cntlid;
	uint8_t     nvmrcd_resv0[238];
	uint8_t     nvmrcd_subnqn[256];
	uint8_t     nvmrcd_hostnqn[256];
	uint8_t     nvmrcd_resv1[256];
} __packed;
CTASSERT(sizeof(struct nvmrdma_connect_data) == 1024);
struct nvmrdma_connect_data glbl_ncdata;

#define NVMR_DYNANYCNTLID 0xFFFF

#define HOSTNQN_TEMPLATE "nqn.2014-08.org.nvmexpress:uuid:%s"
#define DISCOVERY_SUBNQN "nqn.2014-08.org.nvmexpress.discovery"

char nvrdma_host_uuid_string[80];
struct uuid nvrdma_host_uuid;


static void
nvmr_event_established(struct rdma_cm_id *cm_id)
{
	struct ib_recv_wr rqwr, *bad_wr;
	struct nvme_completion *cmplp;
	struct nvmr_ncmplcont *rcve;
	struct ib_sge sgl;
	int retval, count;
	struct ib_mr *mr;
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
		rcve->nvmrsp_nvmecompl = cmplp;

		sgl.addr   = rcve->nvmrsp_dmaddr;
		sgl.length = sizeof(*(rcve->nvmrsp_nvmecompl));
		sgl.lkey   = glbl_ibpd->local_dma_lkey;

		rcve->nvmrsp_cqe.done = nvmr_recv_cmplhndlr;

		rqwr.sg_list = &sgl;
		rqwr.num_sge = 1;
		rqwr.wr_cqe  = &rcve->nvmrsp_cqe;
		rqwr.next    = NULL;

		retval = ib_post_recv(glbl_ibqp, &rqwr, &bad_wr);
		if (retval != 0) {
			ERRSPEW("ib_post_recv() failed for #%d with %d\n",
			    count, retval);
			goto out;
		}
	}
	DBGSPEW("Successfully posted receive buffers for NVMe completions\n");


	/*
	 * Allocate MR structures for use by any data that the NVMe commands
	 * posted to the IB SND Q
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
/**********
 We need to send out a CONNECT message and have that responded to.  We already
 have the NVMe Completion buffers posted at this point.  The NVMe Command
 buffer has been mapped but not been relinquished for use by the HCA.
 **********/

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


static void
nvmr_init(void)
{
	int retval;
	struct rdma_cm_id *cm_id;
	struct sockaddr_storage saddr;
	struct sockaddr_in *sin4;
	uint32_t ipaddr;
	uint16_t rdmaport;

	cm_id = NULL;
	sin4 = (struct sockaddr_in *)&saddr;
	/* rdmaport = 0x4411; */
	/* rdmaport = 0x0E28; */
	rdmaport = 0x4411;
	/* ipaddr = 0x0A0A0A0AU; */
	/* ipaddr = 0xC80A0A0BU; */
	ipaddr = 0xC80A0A0BU;

	retval = ib_register_client(&nvmr_ib_client);
	if (retval != 0) {
		ERRSPEW("ib_register_client() for NVMeoF failed, ret:%d\n",
		    retval);
		goto out;
	}

	cm_id = rdma_create_id(TD_TO_VNET(curthread), nvmr_cm_handler,
	    nvmr_init, RDMA_PS_TCP, IB_QPT_RC);
	if (IS_ERR(cm_id)) {
		ERRSPEW("rdma_create_id() failed:%ld\n", PTR_ERR(cm_id));
		cm_id = NULL;
		goto out;
	} else {
		DBGSPEW("rdma_create_id() succeeded:%p\n", cm_id);
	}

	memset(&saddr, 0, sizeof(saddr));
	sin4->sin_len = sizeof(*sin4);
	sin4->sin_family = AF_INET;
	memcpy((void *)&sin4->sin_addr.s_addr, &ipaddr, sizeof(ipaddr));
	sin4->sin_port = rdmaport;
	retval = rdma_resolve_addr(cm_id, NULL, (struct sockaddr *)sin4, 2000);
	if (retval != 0) {
		ERRSPEW("Failed, rdma_resolve_addr()> %d\n", retval);
		goto out;
	}
	DBGSPEW("Successfully invoked rdma_resolve_addr()\n");
	glbl_cmid = cm_id;
	cm_id = NULL;

out:
	if (cm_id != NULL) {
		rdma_destroy_id(cm_id);
	}
}


static void
nvmr_uninit(void)
{
	int count;

	DBGSPEW("Uninit invoked\n");

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

	ib_unregister_client(&nvmr_ib_client);
}


SYSINIT(nvmr, SI_SUB_DRIVERS, SI_ORDER_ANY, nvmr_init, NULL);
SYSUNINIT(nvmr, SI_SUB_DRIVERS, SI_ORDER_ANY, nvmr_uninit, NULL);
MODULE_DEPEND(nvmr, linuxkpi, 1, 1, 1);
MODULE_DEPEND(nvmr, ibcore, 1, 1, 1);
MODULE_VERSION(nvmr, 1);
