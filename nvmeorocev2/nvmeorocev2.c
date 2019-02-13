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

static void
nvmeorocev2_load(void)
{
	return;
}


static void
nvmeorocev2_unload(void)
{
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
MODULE_VERSION(nvmeorocev2, 1);
