#include <sys/cdefs.h>
#include <sys/types.h>
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/module.h>
#include <sys/systm.h>

#include <netinet/in.h>

#include <linux/kernel.h>
#include <linux/netdevice.h>

#include <rdma/rdma_cm.h>
#include <rdma/ib_verbs.h>

#define SPEWCMN(prefix, format, ...) printf(prefix "%s@%d> " format, \
    __func__, __LINE__, ## __VA_ARGS__)
#define SPEW(format, ...) SPEWCMN("", format, ## __VA_ARGS__)
#define ERRSPEW(format, ...) SPEWCMN("ERR|", format, ## __VA_ARGS__)
#define DBGSPEW(format, ...) SPEWCMN("DBG|", format, ## __VA_ARGS__)

static struct rdma_cm_id *glbl_cmid = NULL;
static struct ib_device  *glbl_ibdev = NULL;
static struct ib_pd      *glbl_ibpd  = NULL;
static struct ib_cq      *glbl_ibcq  = NULL;
/*
 * Invoked whenever the routine is registered with ib_register_client()
 * below or when an IB interface is added to the system.  In the former case
 * the routine is invoked for every IB interface already known.
 */
static void
nrev2_add_ibif(struct ib_device *ib_device)
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
nrev2_remove_ibif(struct ib_device *ib_device, void *client_data)
{
	DBGSPEW("%p removed\n", ib_device);
}


static struct ib_client nvmeofrocev2 = {
	.name   = "nrev2_ib_client",
	.add    = nrev2_add_ibif,
	.remove = nrev2_remove_ibif
};

static void
nrev2_addr_resolved(struct rdma_cm_id *cm_id)
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
nrev2_qphndlr(struct ib_event *ev, void *ctx)
{
	DBGSPEW("Event \"%s\" on QP:%p\n", ib_event_msg(ev->event), ctx);
}

#define MAX_ADMIN_WORK_REQUESTS 32

static void
nrev2_route_resolved(struct rdma_cm_id *cm_id)
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

	KASSERT((glbl_ibcq != NULL) && (glbl_ibpd != NULL), ("Global(s) NULL"));

	memset(&init_attr, 0, sizeof(init_attr));
	init_attr.cap.max_send_wr = (MAX_ADMIN_WORK_REQUESTS * 3) + 1;
	init_attr.cap.max_recv_wr = MAX_ADMIN_WORK_REQUESTS + 1;
	init_attr.cap.max_recv_sge = 1;
	init_attr.cap.max_send_sge = 1 + 1;
	init_attr.qp_type = IB_QPT_RC;
	init_attr.send_cq = glbl_ibcq;
	init_attr.recv_cq = glbl_ibcq;
	init_attr.sq_sig_type = IB_SIGNAL_REQ_WR;

	init_attr.event_handler = nrev2_qphndlr;

	retval = rdma_create_qp(glbl_cmid, glbl_ibpd, &init_attr);
	if (retval != 0) {
		ERRSPEW("rdma_create_qp() failed with %d\n", retval);
	} else {
		DBGSPEW("Successfully created QP!\n");
	}

	memset(&conn_param, 0, sizeof conn_param);
	conn_param.responder_resources = 1;
	conn_param.initiator_depth = 1;
	conn_param.retry_count = 1;

	retval = rdma_connect(glbl_cmid, &conn_param);
	if (retval != 0) {
		ERRSPEW("rdma_connect() failed with %d\n", retval);
	} else {
		DBGSPEW("Successfully rdma_connected()!\n");
	}

out:
	return;
}


static int
nrev2_cm_handler(struct rdma_cm_id *cm_id, struct rdma_cm_event *event)
{
	DBGSPEW("Event \"%s\" returned status \"%d\" for cm_id:%p\n",
	    rdma_event_msg(event->event), event->status, cm_id);

	KASSERT(glbl_cmid == cm_id, ("Global CM id not the passed in CM id"));

	switch(event->event) {
	case RDMA_CM_EVENT_ADDR_RESOLVED:
		nrev2_addr_resolved(cm_id);
		break;
	case RDMA_CM_EVENT_ROUTE_RESOLVED:
		nrev2_route_resolved(cm_id);
		break;
	default:
		dump_stack();
		break;
	}

	return 0;
}


static void
nrev2_init(void)
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

	retval = ib_register_client(&nvmeofrocev2);
	if (retval != 0) {
		ERRSPEW("ib_register_client() for NVMeoF failed, ret:%d\n",
		    retval);
		goto out;
	}

	cm_id = rdma_create_id(TD_TO_VNET(curthread), nrev2_cm_handler,
	    nrev2_init, RDMA_PS_TCP, IB_QPT_RC);
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
nrev2_uninit(void)
{
	DBGSPEW("Uninit invoked\n");

	if ((glbl_ibcq != NULL) && (glbl_ibpd != NULL) && (glbl_cmid != NULL)) {
		DBGSPEW("Invoking rdma_disconnect(%p)...\n", glbl_cmid);
		rdma_disconnect(glbl_cmid);
		DBGSPEW("Invoking rdma_destroy_qp(%p)...\n", glbl_cmid);
		rdma_destroy_qp(glbl_cmid);
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

	ib_unregister_client(&nvmeofrocev2);
}


SYSINIT(nrev2, SI_SUB_DRIVERS, SI_ORDER_ANY, nrev2_init, NULL);
SYSUNINIT(nrev2, SI_SUB_DRIVERS, SI_ORDER_ANY, nrev2_uninit, NULL);
MODULE_DEPEND(nrev2, linuxkpi, 1, 1, 1);
MODULE_DEPEND(nrev2, ibcore, 1, 1, 1);
MODULE_VERSION(nrev2, 1);
