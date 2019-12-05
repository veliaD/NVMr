# $FreeBSD$
.PATH:	${SRCTOP}/sys/dev/nvmrdma

KMOD=nvmrdma

SRCS= \
	nvmrdma.c \
	device_if.h bus_if.h vnode_if.h pci_if.h \
	opt_inet.h opt_inet6.h opt_rss.h opt_ratelimit.h \
	opt_cam.h opt_nvme.h

CFLAGS+= -I${SRCTOP}/sys/ofed/include
CFLAGS+= -I${SRCTOP}/sys/ofed/include/uapi
CFLAGS+= -I${SRCTOP}/sys/compat/linuxkpi/common/include
CFLAGS+= -DINET

.include <bsd.kmod.mk>
