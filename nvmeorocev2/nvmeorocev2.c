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

static struct rdma_cm_id *glbl_cm_id = NULL;
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

	retval = rdma_resolve_route(cm_id, 2000);
	if (retval != 0) {
		ERRSPEW("rdma_resolve_route() failed w/%d\n", retval);
		/* rdma_destroy_id(cm_id); */
	} else {
		DBGSPEW("Successfully invoked rdma_resolve_route()\n");
	}
}


static void
nrev2_route_resolved(struct rdma_cm_id *cm_id)
{
	DBGSPEW("\nRoute Resolved!\n");
	/* rdma_destroy_id(cm_id); */
}


static int
nrev2_cm_handler(struct rdma_cm_id *cm_id, struct rdma_cm_event *event)
{
	DBGSPEW("Event \"%s\" returned status \"%d\" for cm_id:%p\n",
	    rdma_event_msg(event->event), event->status, cm_id);

	switch(event->event) {
	case RDMA_CM_EVENT_ADDR_RESOLVED:
		nrev2_addr_resolved(cm_id);
		break;
	case RDMA_CM_EVENT_ROUTE_RESOLVED:
		nrev2_route_resolved(cm_id);
		break;
	default:
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
	rdmaport = 0x0E28;
	/* ipaddr = 0x0A0A0A0AU; */
	ipaddr = 0xC90A0A0B;

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
	glbl_cm_id = cm_id;
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
	if (glbl_cm_id != NULL) {
		rdma_destroy_id(glbl_cm_id);
	}

	ib_unregister_client(&nvmeofrocev2);
}


SYSINIT(nrev2, SI_SUB_DRIVERS, SI_ORDER_ANY, nrev2_init, NULL);
SYSUNINIT(nrev2, SI_SUB_DRIVERS, SI_ORDER_ANY, nrev2_uninit, NULL);
MODULE_DEPEND(nrev2, linuxkpi, 1, 1, 1);
MODULE_DEPEND(nrev2, ibcore, 1, 1, 1);
MODULE_VERSION(nrev2, 1);
