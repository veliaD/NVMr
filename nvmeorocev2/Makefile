# $FreeBSD$
.PATH:	${.CURDIR}/../../dev/nvmeorocev2

KMOD=nvmeorocev2
SRCS= \
	nvmeorocev2.c \
	device_if.h bus_if.h vnode_if.h pci_if.h \
	opt_inet.h opt_inet6.h opt_rss.h opt_ratelimit.h \
	opt_cam.h opt_nvme.h

.include <bsd.kmod.mk>
