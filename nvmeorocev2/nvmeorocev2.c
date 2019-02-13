#include <sys/cdefs.h>
#include <sys/types.h>
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/module.h>
#include <sys/systm.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <rdma/ib_verbs.h>

#define SPEWCMN(prefix, format, ...) printf(prefix "%s(%d) " format, \
    __func__, __LINE__, ## __VA_ARGS__)
#define SPEW(format, ...) SPEWCMN("", format, ## __VA_ARGS__)
#define ERRSPEW(format, ...) SPEWCMN("ERR|", format, ## __VA_ARGS__)

struct nvmeorocev2_controller {
	int field;
};

static device_method_t nvmeorocev2_pci_methods[] = {
	/* Device interface */
	{ 0, 0 }
};

static driver_t nvmeorocev2_pci_driver = {
	"nvmeorocev2",
	nvmeorocev2_pci_methods,
	sizeof(struct nvmeorocev2_controller),
};

static devclass_t nvmeorocev2_devclass;

/*
 * Invoked whenever the routine is registered with ib_register_client()
 * below or when an IB interface is added to the system.  In the former case
 * the routine is invoked for every IB interface already known.
 */
static void
nvmeorocev2_add_one(struct ib_device *ib_device)
{
	SPEW("rdma_node_get_transport(%p)> %d\n", ib_device,
	    rdma_node_get_transport(ib_device->node_type));
}

/*
 * Invoked whenever the routine is unregistered with ib_unregister_client()
 * below or when an IB interface is removed from the system.  In the former case
 * the routine is invoked for every IB interface already known.
 */
static void
nvmeorocev2_remove_one(struct ib_device *ib_device, void *client_data)
{
	SPEW("%p removed\n", ib_device);
}

static struct ib_client nvmeofrocev2 = {
	.name   = "nvmeofrocev2_ib_client",
	.add    = nvmeorocev2_add_one,
	.remove = nvmeorocev2_remove_one
};


static void
nvmeorocev2_load(void)
{
	int retval;

	retval = ib_register_client(&nvmeofrocev2);
	if (retval != 0) {
		printf("ib_register_client() for NVMeoF failed, ret:%d\n",
		    retval);
	}
}

static void
nvmeorocev2_unload(void)
{
	ib_unregister_client(&nvmeofrocev2);
	return;
}


static void
nvmeorocev2_unimplemented_modevent(void)
{
	return;
}


static int
nvmeorocev2_modevent(module_t mod, int type, void *arg)
{
	ERRSPEW("modevent %d invoked\n", type);
	switch (type) {
	case MOD_LOAD:
		nvmeorocev2_load();
		break;
	case MOD_UNLOAD:
		nvmeorocev2_unload();
		break;
	default:
		nvmeorocev2_unimplemented_modevent();
		break;
	}

	return (0);
}

DRIVER_MODULE(nvmeorocev2, pci, nvmeorocev2_pci_driver, nvmeorocev2_devclass,
    nvmeorocev2_modevent, 0);
MODULE_DEPEND(nvmeorocev2, ibcore, 1, 1, 1);
MODULE_VERSION(nvmeorocev2, 1);
