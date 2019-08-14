/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2019 Dell Inc. or its subsidiaries. All Rights Reserved.
 * Copyright (C) 2012-2013 Intel Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#ifndef __NVME_SHARED_H__
#define __NVME_SHARED_H__

#include <sys/param.h>
#include <sys/bio.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#ifdef _KERNEL
#include <sys/taskqueue.h>
#endif /* _KERNEL */

#include <vm/uma.h>

#include <sys/epoch.h>

#define SPEWCMN(prefix, format, ...) printf(prefix "%s@%d> " format, \
    __func__, __LINE__, ## __VA_ARGS__)
#define SPEW(format, ...) SPEWCMN("", format, ## __VA_ARGS__)
#define ERRSPEW(format, ...) SPEWCMN("ERR|", format, ## __VA_ARGS__)
#define DBGSPEW(format, ...) SPEWCMN("DBG|", format, ## __VA_ARGS__)
#define NVMESPEW(c, format, ...) printf("nvme%d:<%s@%d> " format, \
    (c)->nvmec_unit, __func__, __LINE__, ## __VA_ARGS__)

#define NVME_DEFAULT_TIMEOUT_PERIOD	(30)    /* in seconds */
#define NVME_MIN_TIMEOUT_PERIOD		(5)
#define NVME_MAX_TIMEOUT_PERIOD		(120)

#define NVME_VFFSTRSZ 32

/* Cap nvme to 1MB transfers driver explodes with larger sizes */
#define NVME_MAX_XFER_SIZE		(MAXPHYS < (1<<20) ? MAXPHYS : (1<<20))

struct nvme_completion {

	/* dword 0 */
	uint32_t		cdw0;	/* command-specific */

	/* dword 1 */
	uint32_t		rsvd1;

	/* dword 2 */
	uint16_t		sqhd;	/* submission queue head pointer */
	uint16_t		sqid;	/* submission queue identifier */

	/* dword 3 */
	uint16_t		cid;	/* command identifier */
	uint16_t		status;
} __packed;

_Static_assert(sizeof(struct nvme_completion) == 4 * 4, "bad size for nvme_completion");

#define NVME_SUBNQN_MAX_LENGTH 256

struct nvme_command
{
	/* dword 0 */
	uint8_t opc;		/* opcode */
	uint8_t fuse;		/* fused operation */
	uint16_t cid;		/* command identifier */

	/* dword 1 */
	uint32_t nsid;		/* namespace identifier */

	/* dword 2-3 */
	uint32_t rsvd2;
	uint32_t rsvd3;

	/* dword 4-5 */
	uint64_t mptr;		/* metadata pointer */

	/* dword 6-7 */
	uint64_t prp1;		/* prp entry 1 */

	/* dword 8-9 */
	uint64_t prp2;		/* prp entry 2 */

	/* dword 10-15 */
	uint32_t cdw10;		/* command-specific */
	uint32_t cdw11;		/* command-specific */
	uint32_t cdw12;		/* command-specific */
	uint32_t cdw13;		/* command-specific */
	uint32_t cdw14;		/* command-specific */
	uint32_t cdw15;		/* command-specific */
} __packed;

_Static_assert(sizeof(struct nvme_command) == 16 * 4, "bad size for nvme_command");

/* admin opcodes */
enum nvme_admin_opcode {
	NVME_OPC_DELETE_IO_SQ			= 0x00,
	NVME_OPC_CREATE_IO_SQ			= 0x01,
	NVME_OPC_GET_LOG_PAGE			= 0x02,
	/* 0x03 - reserved */
	NVME_OPC_DELETE_IO_CQ			= 0x04,
	NVME_OPC_CREATE_IO_CQ			= 0x05,
	NVME_OPC_IDENTIFY			= 0x06,
	/* 0x07 - reserved */
	NVME_OPC_ABORT				= 0x08,
	NVME_OPC_SET_FEATURES			= 0x09,
	NVME_OPC_GET_FEATURES			= 0x0a,
	/* 0x0b - reserved */
	NVME_OPC_ASYNC_EVENT_REQUEST		= 0x0c,
	NVME_OPC_NAMESPACE_MANAGEMENT		= 0x0d,
	/* 0x0e-0x0f - reserved */
	NVME_OPC_FIRMWARE_ACTIVATE		= 0x10,
	NVME_OPC_FIRMWARE_IMAGE_DOWNLOAD	= 0x11,
	NVME_OPC_DEVICE_SELF_TEST		= 0x14,
	NVME_OPC_NAMESPACE_ATTACHMENT		= 0x15,
	NVME_OPC_KEEP_ALIVE			= 0x18,
	NVME_OPC_DIRECTIVE_SEND			= 0x19,
	NVME_OPC_DIRECTIVE_RECEIVE		= 0x1a,
	NVME_OPC_VIRTUALIZATION_MANAGEMENT	= 0x1c,
	NVME_OPC_NVME_MI_SEND			= 0x1d,
	NVME_OPC_NVME_MI_RECEIVE		= 0x1e,
	NVME_OPC_DOORBELL_BUFFER_CONFIG		= 0x7c,

	NVME_OPC_FABRIC_COMMAND			= 0x7F,

	NVME_OPC_FORMAT_NVM			= 0x80,
	NVME_OPC_SECURITY_SEND			= 0x81,
	NVME_OPC_SECURITY_RECEIVE		= 0x82,
	NVME_OPC_SANITIZE			= 0x84,
};

enum nvme_feature {
	/* 0x00 - reserved */
	NVME_FEAT_ARBITRATION			= 0x01,
	NVME_FEAT_POWER_MANAGEMENT		= 0x02,
	NVME_FEAT_LBA_RANGE_TYPE		= 0x03,
	NVME_FEAT_TEMPERATURE_THRESHOLD		= 0x04,
	NVME_FEAT_ERROR_RECOVERY		= 0x05,
	NVME_FEAT_VOLATILE_WRITE_CACHE		= 0x06,
	NVME_FEAT_NUMBER_OF_QUEUES		= 0x07,
	NVME_FEAT_INTERRUPT_COALESCING		= 0x08,
	NVME_FEAT_INTERRUPT_VECTOR_CONFIGURATION = 0x09,
	NVME_FEAT_WRITE_ATOMICITY		= 0x0A,
	NVME_FEAT_ASYNC_EVENT_CONFIGURATION	= 0x0B,
	NVME_FEAT_AUTONOMOUS_POWER_STATE_TRANSITION = 0x0C,
	NVME_FEAT_HOST_MEMORY_BUFFER		= 0x0D,
	NVME_FEAT_TIMESTAMP			= 0x0E,
	NVME_FEAT_KEEP_ALIVE_TIMER		= 0x0F,
	NVME_FEAT_HOST_CONTROLLED_THERMAL_MGMT	= 0x10,
	NVME_FEAT_NON_OP_POWER_STATE_CONFIG	= 0x11,
	/* 0x12-0x77 - reserved */
	/* 0x78-0x7f - NVMe Management Interface */
	NVME_FEAT_SOFTWARE_PROGRESS_MARKER	= 0x80,
	/* 0x81-0xBF - command set specific (reserved) */
	/* 0xC0-0xFF - vendor specific */
};

#define NVME_SERIAL_NUMBER_LENGTH	20
#define NVME_MODEL_NUMBER_LENGTH	40
#define NVME_FIRMWARE_REVISION_LENGTH	8

#define NVME_MAX_NAMESPACES	(16)

#define NVME_MAX_CONSUMERS	(2)
#define NVME_MAX_ASYNC_EVENTS	(8)

/* Maximum log page size to fetch for AERs. */
#define NVME_MAX_AER_LOG_SIZE		(4096)

struct nvme_async_event_request {

	struct nvme_controller		*nvmea_ctrlrp;
	struct nvme_request		*req;
	struct nvme_completion		cpl;
	uint32_t			log_page_id;
	uint32_t			log_page_size;
	uint8_t				log_page_buffer[NVME_MAX_AER_LOG_SIZE];
};

struct nvme_power_state {
	/** Maximum Power */
	uint16_t	mp;			/* Maximum Power */
	uint8_t		ps_rsvd1;
	uint8_t		mps_nops;		/* Max Power Scale, Non-Operational State */

	uint32_t	enlat;			/* Entry Latency */
	uint32_t	exlat;			/* Exit Latency */

	uint8_t		rrt;			/* Relative Read Throughput */
	uint8_t		rrl;			/* Relative Read Latency */
	uint8_t		rwt;			/* Relative Write Throughput */
	uint8_t		rwl;			/* Relative Write Latency */

	uint16_t	idlp;			/* Idle Power */
	uint8_t		ips;			/* Idle Power Scale */
	uint8_t		ps_rsvd8;

	uint16_t	actp;			/* Active Power */
	uint8_t		apw_aps;		/* Active Power Workload, Active Power Scale */
	uint8_t		ps_rsvd10[9];
} __packed;

_Static_assert(sizeof(struct nvme_power_state) == 32, "bad size for nvme_power_state");


struct nvme_controller_data {

	/* bytes 0-255: controller capabilities and features */

	/** pci vendor id */
	uint16_t		vid;

	/** pci subsystem vendor id */
	uint16_t		ssvid;

	/** serial number */
	uint8_t			sn[NVME_SERIAL_NUMBER_LENGTH];

	/** model number */
	uint8_t			mn[NVME_MODEL_NUMBER_LENGTH];

	/** firmware revision */
	uint8_t			fr[NVME_FIRMWARE_REVISION_LENGTH];

	/** recommended arbitration burst */
	uint8_t			rab;

	/** ieee oui identifier */
	uint8_t			ieee[3];

	/** multi-interface capabilities */
	uint8_t			mic;

	/** maximum data transfer size */
	uint8_t			mdts;

	/** Controller ID */
	uint16_t		ctrlr_id;

	/** Version */
	uint32_t		ver;

	/** RTD3 Resume Latency */
	uint32_t		rtd3r;

	/** RTD3 Enter Latency */
	uint32_t		rtd3e;

	/** Optional Asynchronous Events Supported */
	uint32_t		oaes;	/* bitfield really */

	/** Controller Attributes */
	uint32_t		ctratt;	/* bitfield really */

	uint8_t			reserved1[12];

	/** FRU Globally Unique Identifier */
	uint8_t			fguid[16];

	uint8_t			reserved2[128];

	/* bytes 256-511: admin command set attributes */

	/** optional admin command support */
	uint16_t		oacs;

	/** abort command limit */
	uint8_t			acl;

	/** asynchronous event request limit */
	uint8_t			aerl;

	/** firmware updates */
	uint8_t			frmw;

	/** log page attributes */
	uint8_t			lpa;

	/** error log page entries */
	uint8_t			elpe;

	/** number of power states supported */
	uint8_t			npss;

	/** admin vendor specific command configuration */
	uint8_t			avscc;

	/** Autonomous Power State Transition Attributes */
	uint8_t			apsta;

	/** Warning Composite Temperature Threshold */
	uint16_t		wctemp;

	/** Critical Composite Temperature Threshold */
	uint16_t		cctemp;

	/** Maximum Time for Firmware Activation */
	uint16_t		mtfa;

	/** Host Memory Buffer Preferred Size */
	uint32_t		hmpre;

	/** Host Memory Buffer Minimum Size */
	uint32_t		hmmin;

	/** Name space capabilities  */
	struct {
		/* if nsmgmt, report tnvmcap and unvmcap */
		uint8_t    tnvmcap[16];
		uint8_t    unvmcap[16];
	} __packed untncap;

	/** Replay Protected Memory Block Support */
	uint32_t		rpmbs; /* Really a bitfield */

	/** Extended Device Self-test Time */
	uint16_t		edstt;

	/** Device Self-test Options */
	uint8_t			dsto; /* Really a bitfield */

	/** Firmware Update Granularity */
	uint8_t			fwug;

	/** Keep Alive Support */
	uint16_t		kas;

	/** Host Controlled Thermal Management Attributes */
	uint16_t		hctma; /* Really a bitfield */

	/** Minimum Thermal Management Temperature */
	uint16_t		mntmt;

	/** Maximum Thermal Management Temperature */
	uint16_t		mxtmt;

	/** Sanitize Capabilities */
	uint32_t		sanicap; /* Really a bitfield */

	uint8_t			reserved3[180];
	/* bytes 512-703: nvm command set attributes */

	/** submission queue entry size */
	uint8_t			sqes;

	/** completion queue entry size */
	uint8_t			cqes;

	/** Maximum Outstanding Commands */
	uint16_t		maxcmd;

	/** number of namespaces */
	uint32_t		nn;

	/** optional nvm command support */
	uint16_t		oncs;

	/** fused operation support */
	uint16_t		fuses;

	/** format nvm attributes */
	uint8_t			fna;

	/** volatile write cache */
	uint8_t			vwc;

	/** Atomic Write Unit Normal */
	uint16_t		awun;

	/** Atomic Write Unit Power Fail */
	uint16_t		awupf;

	/** NVM Vendor Specific Command Configuration */
	uint8_t			nvscc;
	uint8_t			reserved5;

	/** Atomic Compare & Write Unit */
	uint16_t		acwu;
	uint16_t		reserved6;

	/** SGL Support */
	uint32_t		sgls;

	/* bytes 540-767: Reserved */
	uint8_t			reserved7[228];

	/** NVM Subsystem NVMe Qualified Name */
	uint8_t			subnqn[NVME_SUBNQN_MAX_LENGTH];

	/* bytes 1024-1791: Reserved */
	uint8_t			reserved8[768];

	/* bytes 1792-2047: NVMe over Fabrics specification */
	uint32_t		nidf_ioccsz;
	uint32_t		nidf_iorcsz;
	uint16_t		nidf_icdoff;
	uint8_t			nidf_ctrattr;
	uint8_t			nidf_msdbd;
	uint8_t			reserved9[244];

	/* bytes 2048-3071: power state descriptors */
	struct nvme_power_state power_state[32];

	/* bytes 3072-4095: vendor specific */
	uint8_t			vs[1024];
} __packed __aligned(4);

_Static_assert(sizeof(struct nvme_controller_data) == 4096, "bad size for nvme_controller_data");

struct nvme_namespace_data {

	/** namespace size */
	uint64_t		nsze;

	/** namespace capacity */
	uint64_t		ncap;

	/** namespace utilization */
	uint64_t		nuse;

	/** namespace features */
	uint8_t			nsfeat;

	/** number of lba formats */
	uint8_t			nlbaf;

	/** formatted lba size */
	uint8_t			flbas;

	/** metadata capabilities */
	uint8_t			mc;

	/** end-to-end data protection capabilities */
	uint8_t			dpc;

	/** end-to-end data protection type settings */
	uint8_t			dps;

	/** Namespace Multi-path I/O and Namespace Sharing Capabilities */
	uint8_t			nmic;

	/** Reservation Capabilities */
	uint8_t			rescap;

	/** Format Progress Indicator */
	uint8_t			fpi;

	/** Deallocate Logical Block Features */
	uint8_t			dlfeat;

	/** Namespace Atomic Write Unit Normal  */
	uint16_t		nawun;

	/** Namespace Atomic Write Unit Power Fail */
	uint16_t		nawupf;

	/** Namespace Atomic Compare & Write Unit */
	uint16_t		nacwu;

	/** Namespace Atomic Boundary Size Normal */
	uint16_t		nabsn;

	/** Namespace Atomic Boundary Offset */
	uint16_t		nabo;

	/** Namespace Atomic Boundary Size Power Fail */
	uint16_t		nabspf;

	/** Namespace Optimal IO Boundary */
	uint16_t		noiob;

	/** NVM Capacity */
	uint8_t			nvmcap[16];

	/* bytes 64-103: Reserved */
	uint8_t			reserved5[40];

	/** Namespace Globally Unique Identifier */
	uint8_t			nguid[16];

	/** IEEE Extended Unique Identifier */
	uint8_t			eui64[8];

	/** lba format support */
	uint32_t		lbaf[16];

	uint8_t			reserved6[192];

	uint8_t			vendor_specific[3712];
} __packed __aligned(4);

_Static_assert(sizeof(struct nvme_namespace_data) == 4096, "bad size for nvme_namepsace_data");

struct nvme_namespace {
	struct nvme_controller		*nvmes_ctrlr;
	struct nvme_namespace_data	nvmes_nsd;
	uint32_t			id;
	uint32_t			flags;
	struct cdev			*cdev;
	void				*cons_cookie[NVME_MAX_CONSUMERS];
	uint32_t			stripesize;
	struct mtx			lock;
};

enum nvme_transport {
	NVMET_PCIE = 0xBF83E31D,
	NVMET_RDMA = 0x92A3317C,
};

#define NVME_IS_CTRLR_FAILED(c) atomic_load_acq_int(&(c)->is_failed)
#define NVME_SET_CTRLR_FAILED(c) atomic_store_rel_int(&(c)->is_failed, TRUE)

#ifdef _KERNEL
struct nvme_controller {
	struct mtx		lockc;

	uint32_t		ready_timeout_in_ms;
	uint32_t		cquirks;
#define QUIRK_DELAY_B4_CHK_RDY 1		/* Can't touch MMIO on disable */
	uint32_t		num_io_queues;
	uint32_t		num_cpus_per_ioq;
	uint32_t		max_hw_pend_io;

	uint32_t		ns_identified;
	uint32_t		queues_created;

	struct task		reset_task;
	struct task		fail_req_task;
	struct taskqueue	*taskqueue;

	/** maximum i/o size in bytes */
	uint32_t		max_xfer_size;

	uint64_t		guard0;
	struct nvme_controller_data	cdata;
	uint64_t		guard1;

	/** minimum page size supported by this controller in bytes */
	uint32_t		min_page_size;
	struct nvme_namespace		cns[NVME_MAX_NAMESPACES];

	struct cdev			*ccdev;

	/** bit mask of event types currently enabled for async events */
	uint32_t			async_event_config;

	uint32_t			num_aers;
	struct nvme_async_event_request	aer[NVME_MAX_ASYNC_EVENTS];

	void				*ccons_cookie[NVME_MAX_CONSUMERS];

	uint32_t			is_resetting;
	uint32_t			is_initialized;
	uint32_t			notification_sent;

	volatile boolean_t		is_failed;
	STAILQ_HEAD(, nvme_request)	fail_req;

	STAILQ_ENTRY(nvme_controller)	nvmec_lst;

	/** timeout period in seconds */
	uint32_t		timeout_period;

	enum nvme_transport		nvmec_ttype;
	int				nvmec_unit;
	void				*nvmec_tsp;
	void 				(*nvmec_delist)(struct nvme_controller *);
	void 				(*nvmec_subadmreq)(struct nvme_controller *, struct nvme_request *);
	void 				(*nvmec_subioreq)(struct nvme_controller *, struct nvme_request *);
};
#endif /* _KERNEL */


void nvme_register_controller(struct nvme_controller *);
void nvme_unregister_controller(struct nvme_controller *);

typedef void (*nvme_cb_fn_t)(void *, const struct nvme_completion *);

#ifdef _KERNEL
struct nvme_qpair {
	uint32_t		qid;
	uint32_t		num_qentries;
	volatile boolean_t	qis_enabled;
	struct mtx		qlock __aligned(CACHE_LINE_SIZE);
	enum nvme_transport	qttype;
	struct nvme_controller  *gqctrlr;
};

struct nvme_request {

	struct nvme_command		cmd;
	struct nvme_qpair		*rqpair;
	union {
		void			*payload;
		struct bio		*bio;
	} u;
	uint32_t			type;
	uint32_t			payload_size;
	boolean_t			timeout;
	nvme_cb_fn_t			cb_fn;
	void				*cb_arg;
	int32_t				retries;
	STAILQ_ENTRY(nvme_request)	stailq;
};
#endif /* _KERNEL */

int nvme_ctrlr_construct_namespaces(struct nvme_controller *ctrlr);

void nvme_ctrlr_fail_req_task(void *arg, int pending);

/*
 * Use presence of the BIO_UNMAPPED flag to determine whether unmapped I/O
 *  support and the bus_dmamap_load_bio API are available on the target
 *  kernel.  This will ease porting back to earlier stable branches at a
 *  later point.
 */
#ifdef BIO_UNMAPPED
#define NVME_UNMAPPED_BIO_SUPPORT
#endif

#define NVME_REQUEST_VADDR	1
#define NVME_REQUEST_NULL	2 /* For requests with no payload. */
#define NVME_REQUEST_UIO	3
#ifdef NVME_UNMAPPED_BIO_SUPPORT
#define NVME_REQUEST_BIO	4
#endif
#define NVME_REQUEST_CCB        5

/* Endianess conversion functions for NVMe structs */
static inline
void	nvme_completion_swapbytes(struct nvme_completion *s)
{

	s->cdw0 = le32toh(s->cdw0);
	/* omit rsvd1 */
	s->sqhd = le16toh(s->sqhd);
	s->sqid = le16toh(s->sqid);
	/* omit cid */
	s->status = le16toh(s->status);
}

extern uma_zone_t	nvme_request_zone;
#define nvme_free_request(req)	uma_zfree(nvme_request_zone, req)

#define NVME_CC_REG_EN_SHIFT				(0)
#define NVME_CC_REG_EN_MASK				(0x1)
#define NVME_CC_REG_CSS_SHIFT				(4)
#define NVME_CC_REG_CSS_MASK				(0x7)
#define NVME_CC_REG_MPS_SHIFT				(7)
#define NVME_CC_REG_MPS_MASK				(0xF)
#define NVME_CC_REG_AMS_SHIFT				(11)
#define NVME_CC_REG_AMS_MASK				(0x7)
#define NVME_CC_REG_SHN_SHIFT				(14)
#define NVME_CC_REG_SHN_MASK				(0x3)
#define NVME_CC_REG_IOSQES_SHIFT			(16)
#define NVME_CC_REG_IOSQES_MASK				(0xF)
#define NVME_CC_REG_IOCQES_SHIFT			(20)
#define NVME_CC_REG_IOCQES_MASK				(0xF)

#define NVME_STATUS_P_SHIFT				(0)
#define NVME_STATUS_P_MASK				(0x1)
#define NVME_STATUS_SC_SHIFT				(1)
#define NVME_STATUS_SC_MASK				(0xFF)
#define NVME_STATUS_SCT_SHIFT				(9)
#define NVME_STATUS_SCT_MASK				(0x7)
#define NVME_STATUS_M_SHIFT				(14)
#define NVME_STATUS_M_MASK				(0x1)
#define NVME_STATUS_DNR_SHIFT				(15)
#define NVME_STATUS_DNR_MASK				(0x1)

#define NVME_STATUS_GET_P(st)				(((st) >> NVME_STATUS_P_SHIFT) & NVME_STATUS_P_MASK)
#define NVME_STATUS_GET_SC(st)				(((st) >> NVME_STATUS_SC_SHIFT) & NVME_STATUS_SC_MASK)
#define NVME_STATUS_GET_SCT(st)				(((st) >> NVME_STATUS_SCT_SHIFT) & NVME_STATUS_SCT_MASK)
#define NVME_STATUS_GET_M(st)				(((st) >> NVME_STATUS_M_SHIFT) & NVME_STATUS_M_MASK)
#define NVME_STATUS_GET_DNR(st)				(((st) >> NVME_STATUS_DNR_SHIFT) & NVME_STATUS_DNR_MASK)

#define nvme_completion_is_error(cpl)					\
	(NVME_STATUS_GET_SC((cpl)->status) != 0 || NVME_STATUS_GET_SCT((cpl)->status) != 0)

#ifdef _KERNEL
boolean_t nvme_completion_is_retry(const struct nvme_completion *cpl);
void nvme_qpair_print_command(struct nvme_qpair *qpair, struct nvme_command
    *cmd);
void nvme_qpair_print_completion(struct nvme_qpair *qpair,
    struct nvme_completion *cpl);
void nvme_ns_destruct(struct nvme_namespace *ns);
#endif /* _KERNEL */

struct nvme_completion_poll_status {

	struct nvme_completion	cpl;
	int			done;
};

#ifdef _KERNEL
static __inline struct nvme_request *
_nvme_allocate_request(nvme_cb_fn_t cb_fn, void *cb_arg)
{
	struct nvme_request *req;

	req = uma_zalloc(nvme_request_zone, M_NOWAIT | M_ZERO);
	if (req != NULL) {
		req->cb_fn = cb_fn;
		req->cb_arg = cb_arg;
		req->timeout = TRUE;
	}
	return (req);
}

static __inline struct nvme_request *
nvme_allocate_request_null(nvme_cb_fn_t cb_fn, void *cb_arg)
{
	struct nvme_request *req;

	req = _nvme_allocate_request(cb_fn, cb_arg);
	if (req != NULL)
		req->type = NVME_REQUEST_NULL;
	return (req);
}

static __inline struct nvme_request *
nvme_allocate_request_vaddr(void *payload, uint32_t payload_size,
    nvme_cb_fn_t cb_fn, void *cb_arg)
{
	struct nvme_request *req;

	req = _nvme_allocate_request(cb_fn, cb_arg);
	if (req != NULL) {
		req->type = NVME_REQUEST_VADDR;
		req->u.payload = payload;
		req->payload_size = payload_size;
	}
	return (req);
}
#endif /* _KERNEL */

void	nvme_completion_poll_cb(void *arg, const struct nvme_completion *cpl);

#define nvme_ctrlr_submit_admin_request(c,r) (c)->nvmec_subadmreq((c), (r))
#define nvme_ctrlr_submit_io_request(c,r) (c)->nvmec_subioreq((c), (r))

void	nvme_notify_new_controller(struct nvme_controller *ctrlr);
void	nvme_notify_fail_consumers(struct nvme_controller *ctrlr);
void	nvme_notify_mark_failing_consumers(struct nvme_controller *ctrlr);
void	nvme_ctrlr_cmd_get_log_page(struct nvme_controller *ctrlr,
				    uint8_t log_page, uint32_t nsid,
				    void *payload, uint32_t payload_size,
				    nvme_cb_fn_t cb_fn, void *cb_arg);

/* status code types */
enum nvme_status_code_type {
	NVME_SCT_GENERIC		= 0x0,
	NVME_SCT_COMMAND_SPECIFIC	= 0x1,
	NVME_SCT_MEDIA_ERROR		= 0x2,
	/* 0x3-0x6 - reserved */
	NVME_SCT_VENDOR_SPECIFIC	= 0x7,
};

/* generic command status codes */
enum nvme_generic_command_status_code {
	NVME_SC_SUCCESS				= 0x00,
	NVME_SC_INVALID_OPCODE			= 0x01,
	NVME_SC_INVALID_FIELD			= 0x02,
	NVME_SC_COMMAND_ID_CONFLICT		= 0x03,
	NVME_SC_DATA_TRANSFER_ERROR		= 0x04,
	NVME_SC_ABORTED_POWER_LOSS		= 0x05,
	NVME_SC_INTERNAL_DEVICE_ERROR		= 0x06,
	NVME_SC_ABORTED_BY_REQUEST		= 0x07,
	NVME_SC_ABORTED_SQ_DELETION		= 0x08,
	NVME_SC_ABORTED_FAILED_FUSED		= 0x09,
	NVME_SC_ABORTED_MISSING_FUSED		= 0x0a,
	NVME_SC_INVALID_NAMESPACE_OR_FORMAT	= 0x0b,
	NVME_SC_COMMAND_SEQUENCE_ERROR		= 0x0c,
	NVME_SC_INVALID_SGL_SEGMENT_DESCR	= 0x0d,
	NVME_SC_INVALID_NUMBER_OF_SGL_DESCR	= 0x0e,
	NVME_SC_DATA_SGL_LENGTH_INVALID		= 0x0f,
	NVME_SC_METADATA_SGL_LENGTH_INVALID	= 0x10,
	NVME_SC_SGL_DESCRIPTOR_TYPE_INVALID	= 0x11,
	NVME_SC_INVALID_USE_OF_CMB		= 0x12,
	NVME_SC_PRP_OFFET_INVALID		= 0x13,
	NVME_SC_ATOMIC_WRITE_UNIT_EXCEEDED	= 0x14,
	NVME_SC_OPERATION_DENIED		= 0x15,
	NVME_SC_SGL_OFFSET_INVALID		= 0x16,
	/* 0x17 - reserved */
	NVME_SC_HOST_ID_INCONSISTENT_FORMAT	= 0x18,
	NVME_SC_KEEP_ALIVE_TIMEOUT_EXPIRED	= 0x19,
	NVME_SC_KEEP_ALIVE_TIMEOUT_INVALID	= 0x1a,
	NVME_SC_ABORTED_DUE_TO_PREEMPT		= 0x1b,
	NVME_SC_SANITIZE_FAILED			= 0x1c,
	NVME_SC_SANITIZE_IN_PROGRESS		= 0x1d,
	NVME_SC_SGL_DATA_BLOCK_GRAN_INVALID	= 0x1e,
	NVME_SC_NOT_SUPPORTED_IN_CMB		= 0x1f,

	NVME_SC_LBA_OUT_OF_RANGE		= 0x80,
	NVME_SC_CAPACITY_EXCEEDED		= 0x81,
	NVME_SC_NAMESPACE_NOT_READY		= 0x82,
	NVME_SC_RESERVATION_CONFLICT		= 0x83,
	NVME_SC_FORMAT_IN_PROGRESS		= 0x84,
};

enum nvme_log_page {

	/* 0x00 - reserved */
	NVME_LOG_ERROR			= 0x01,
	NVME_LOG_HEALTH_INFORMATION	= 0x02,
	NVME_LOG_FIRMWARE_SLOT		= 0x03,
	NVME_LOG_CHANGED_NAMESPACE	= 0x04,
	NVME_LOG_COMMAND_EFFECT		= 0x05,
	/* 0x06-0x6F - reserved */
	NVME_LOG_DISCOVERY		= 0x70,
	/* 0x71-0x7F - reserved */
	/* 0x80-0xBF - I/O command set specific */
	NVME_LOG_RES_NOTIFICATION	= 0x80,
	/* 0xC0-0xFF - vendor specific */

	/*
	 * The following are Intel Specific log pages, but they seem
	 * to be widely implemented.
	 */
	INTEL_LOG_READ_LAT_LOG		= 0xc1,
	INTEL_LOG_WRITE_LAT_LOG		= 0xc2,
	INTEL_LOG_TEMP_STATS		= 0xc5,
	INTEL_LOG_ADD_SMART		= 0xca,
	INTEL_LOG_DRIVE_MKT_NAME	= 0xdd,

	/*
	 * HGST log page, with lots ofs sub pages.
	 */
	HGST_INFO_LOG			= 0xc1,
};

#define	NVME_PASSTHROUGH_CMD		_IOWR('n', 0, struct nvme_pt_command)
#define	NVME_RESET_CONTROLLER		_IO('n', 1)
#define	NVMR_DISCOVERY			_IOWR('n', 2, nvmr_ioctl_t)

#define	NVME_IO_TEST			_IOWR('n', 100, struct nvme_io_test)
#define	NVME_BIO_TEST			_IOWR('n', 101, struct nvme_io_test)

typedef struct {
	char *nvmrpi_port;
	char *nvmrpi_ip;
} nvmr_portip_t;

typedef struct {
	nvmr_portip_t nvmri_pi;
	void         *nvmri_retbuf;
	uint16_t      nvmri_retlen;
	uint8_t       nvmri_portstrlen;
	uint8_t       nvmri_ipstrlen;
} nvmr_ioctl_t;

#define NVMR_DEV		"nvmr"

struct nvme_pt_command {

	/*
	 * cmd is used to specify a passthrough command to a controller or
	 *  namespace.
	 *
	 * The following fields from cmd may be specified by the caller:
	 *	* opc  (opcode)
	 *	* nsid (namespace id) - for admin commands only
	 *	* cdw10-cdw15
	 *
	 * Remaining fields must be set to 0 by the caller.
	 */
	struct nvme_command	cmd;

	/*
	 * cpl returns completion status for the passthrough command
	 *  specified by cmd.
	 *
	 * The following fields will be filled out by the driver, for
	 *  consumption by the caller:
	 *	* cdw0
	 *	* status (except for phase)
	 *
	 * Remaining fields will be set to 0 by the driver.
	 */
	struct nvme_completion	cpl;

	/* buf is the data buffer associated with this passthrough command. */
	void *			buf;

	/*
	 * len is the length of the data buffer associated with this
	 *  passthrough command.
	 */
	uint32_t		len;

	/*
	 * is_read = 1 if the passthrough command will read data into the
	 *  supplied buffer from the controller.
	 *
	 * is_read = 0 if the passthrough command will write data from the
	 *  supplied buffer to the controller.
	 */
	uint32_t		is_read;

	/*
	 * driver_lock is used by the driver only.  It must be set to 0
	 *  by the caller.
	 */
	struct mtx *		driver_lock;
};

int	nvme_ctrlr_passthrough_cmd(struct nvme_controller *ctrlr,
				   struct nvme_pt_command *pt,
				   uint32_t nsid, int is_user_buffer,
				   int is_admin_cmd);

#endif /* __NVME_SHARED_H__ */
