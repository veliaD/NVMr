/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
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
 * $FreeBSD: releng/12.0/sys/dev/nvme/nvme.h 338182 2018-08-22 04:29:24Z chuck $
 */

#ifndef __NVME_SHARED_H__
#define __NVME_SHARED_H__

#include <sys/param.h>
#include <sys/bio.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/taskqueue.h>

#define SPEWCMN(prefix, format, ...) printf(prefix "%s@%d> " format, \
    __func__, __LINE__, ## __VA_ARGS__)
#define SPEW(format, ...) SPEWCMN("", format, ## __VA_ARGS__)
#define ERRSPEW(format, ...) SPEWCMN("ERR|", format, ## __VA_ARGS__)
#define DBGSPEW(format, ...) SPEWCMN("DBG|", format, ## __VA_ARGS__)

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
	uint8_t			subnqn[256];

	/* bytes 1024-1791: Reserved */
	uint8_t			reserved8[768];

	/* bytes 1792-2047: NVMe over Fabrics specification */
	uint8_t			reserved9[256];

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
	NVMET_PCI  = 0xBF83E31D,
	NVMET_RDMA = 0x92A3317C,
};

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

	boolean_t			is_failed;
	STAILQ_HEAD(, nvme_request)	fail_req;

	STAILQ_ENTRY(nvme_controller)	nvmec_lst;

	/** timeout period in seconds */
	uint32_t		timeout_period;

	enum nvme_transport		nvmec_ttype;
	void				*nvmec_tsp;
	void 				(*nvmec_delist)(struct nvme_controller *);
	void 				(*nvmec_subadmreq)(struct nvme_controller *, struct nvme_request *);
	void 				(*nvmec_subioreq)(struct nvme_controller *, struct nvme_request *);
};


void nvme_register_controller(struct nvme_controller *);
void nvme_unregister_controller(struct nvme_controller *);

typedef void (*nvme_cb_fn_t)(void *, const struct nvme_completion *);

struct nvme_qpair {
	uint32_t		qid;
	uint32_t		num_qentries;
	boolean_t		qis_enabled;
	struct mtx		qlock __aligned(CACHE_LINE_SIZE);
	enum nvme_transport	qttype;
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

#endif /* __NVME_SHARED_H__ */
