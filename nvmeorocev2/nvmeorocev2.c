#include <sys/cdefs.h>
#include <sys/types.h>
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/module.h>
#include <sys/systm.h>

#include <linux/kernel.h>

#include <rdma/ib_verbs.h>

#define SPEWCMN(prefix, format, ...) printf(prefix "%s@%d> " format, \
    __func__, __LINE__, ## __VA_ARGS__)
#define SPEW(format, ...) SPEWCMN("", format, ## __VA_ARGS__)
#define ERRSPEW(format, ...) SPEWCMN("ERR|", format, ## __VA_ARGS__)

/*
 * Invoked whenever the routine is registered with ib_register_client()
 * below or when an IB interface is added to the system.  In the former case
 * the routine is invoked for every IB interface already known.
 */
static void
nrev2_add_ibif(struct ib_device *ib_device)
{
	ERRSPEW("rdma_node_get_transport(%p)> %d\n", ib_device,
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
	ERRSPEW("%p removed\n", ib_device);
}


static struct ib_client nvmeofrocev2 = {
	.name   = "nrev2_ib_client",
	.add    = nrev2_add_ibif,
	.remove = nrev2_remove_ibif
};


static void
nrev2_init(void)
{
	int retval;

	retval = ib_register_client(&nvmeofrocev2);
	if (retval != 0) {
		printf("ib_register_client() for NVMeoF failed, ret:%d\n",
		    retval);
	}
}


static void
nrev2_uninit(void)
{
	ib_unregister_client(&nvmeofrocev2);
}


SYSINIT(nrev2, SI_SUB_DRIVERS, SI_ORDER_ANY, nrev2_init, NULL);
SYSUNINIT(nrev2, SI_SUB_DRIVERS, SI_ORDER_ANY, nrev2_uninit, NULL);
MODULE_DEPEND(nrev2, linuxkpi, 1, 1, 1);
MODULE_DEPEND(nrev2, ibcore, 1, 1, 1);
MODULE_VERSION(nrev2, 1);
