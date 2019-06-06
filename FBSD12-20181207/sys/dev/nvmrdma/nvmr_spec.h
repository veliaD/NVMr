/*
 * Copyright (c) 2019 Dell Inc. or its subsidiaries. All Rights Reserved.
 */

#ifndef _NVMR_SPEC_H
#define _NVMR_SPEC_H

#include <sys/cdefs.h>
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/endian.h>
#include <sys/uuid.h>

#include <dev/nvme/nvme_shared.h>

#define MAX_NVME_RDMA_SEGMENTS 256

typedef struct {
	uint64_t nvmrk_address;
	uint8_t  nvmrk_length[3];
	uint32_t nvmrk_key;
	uint8_t  nvmrk_sgl_identifier;
} __packed nvmr_ksgl_t;


typedef struct {
	uint8_t     nvmf_opc;
	uint8_t     nvmf_sgl_fuse;
	uint16_t    nvmf_cid;
	union {
		struct {
			uint8_t     nvmf_fctype;
			uint8_t     nvmf_resvf1[19];
		};
		struct {
			uint32_t    nvmf_nsid;
			uint8_t     nvmf_resvn1[16];
		};
	};
	nvmr_ksgl_t nvmf_ksgl;
} __packed nvmf_prfx_t;

typedef struct {
	nvmf_prfx_t nvmrsb_nvmf;
	uint8_t     nvmrsb_resv1[24];
} __packed nvmr_stub_t;
CTASSERT(sizeof(nvmr_stub_t) == sizeof(struct nvme_command));

typedef struct {
	nvmf_prfx_t nvmrcn_nvmf;
	uint16_t    nvmrcn_recfmt;
	uint16_t    nvmrcn_qid;
	uint16_t    nvmrcn_sqsize;
	uint8_t     nvmrcn_cattr;
	uint8_t     nvmrcn_resv2;
	uint32_t    nvmrcn_kato;
	uint8_t     nvmrcn_resv3[12];
} __packed nvmr_connect_t;
CTASSERT(sizeof(nvmr_connect_t) == sizeof(struct nvme_command));

typedef struct {
	nvmf_prfx_t nvmrpg_nvmf;
	uint8_t     nvmrpg_attrib;
	uint8_t     nvmrpg_resv1[3];
	uint32_t    nvmrpg_ofst;
	uint8_t     nvmrpg_resv2[16];
} __packed nvmr_propget_t;
CTASSERT(sizeof(nvmr_propget_t) == sizeof(struct nvme_command));

typedef struct {
	nvmf_prfx_t nvmrps_nvmf;
	uint8_t     nvmrps_attrib;
	uint8_t     nvmrps_resv1[3];
	uint32_t    nvmrps_ofst;
	uint64_t    nvmrps_value;
	uint8_t     nvmrps_resv2[8];
} __packed nvmr_propset_t;
CTASSERT(sizeof(nvmr_propset_t) == sizeof(struct nvme_command));

typedef struct {
	nvmf_prfx_t nvmrid_nvmf;
	uint8_t     nvmrid_cns;
	uint8_t     nvmrid_resv1;
	uint16_t    nvmrid_cntid;
	uint8_t     nvmrid_resv2[20];
} __packed nvmr_identify_t;
CTASSERT(sizeof(nvmr_identify_t) == sizeof(struct nvme_command));

typedef union {
	struct nvme_command nvmrcu_nvme;
	nvmr_connect_t      nvmrcu_conn;
	nvmr_propget_t      nvmrcu_prgt;
	nvmr_propset_t      nvmrcu_prst;
	nvmr_identify_t     nvmrcu_idnt;
	nvmr_stub_t         nvmrcu_stub;
} nvmr_communion_t;
CTASSERT(sizeof(nvmr_communion_t) == sizeof(struct nvme_command));

typedef struct {
	uint16_t nvmrcr_recfmt;
	uint16_t nvmrcr_qid;
	uint16_t nvmrcr_hrqsize;
	uint16_t nvmrcr_hsqsize;
	uint8_t  nvmrcr_resv0[24];
} __packed nvmr_rdma_cm_request_t;
CTASSERT(sizeof(nvmr_rdma_cm_request_t) == 32);

typedef struct {
	uint16_t nvmrcrj_recfmt;
	uint16_t nvmrcrj_sts;
} __packed nvmr_rdma_cm_reject_t;
CTASSERT(sizeof(nvmr_rdma_cm_reject_t) == 4);

#define MAX_NQN_LEN 255
struct nvmrdma_connect_data {
	struct uuid nvmrcd_hostid;
	uint16_t    nvmrcd_cntlid;
	uint8_t     nvmrcd_resv0[238];
	uint8_t     nvmrcd_subnqn[MAX_NQN_LEN+1];
	uint8_t     nvmrcd_hostnqn[MAX_NQN_LEN+1];
	uint8_t     nvmrcd_resv1[256];
} __packed;
CTASSERT(sizeof(struct nvmrdma_connect_data) == 1024);

#define NVMR_FOURK (4096)
#define NVMR_DYNANYCNTLID 0xFFFF

#define HOSTNQN_TEMPLATE "nqn.2014-08.org.nvmexpress:uuid:%s"
#define DISCOVERY_SUBNQN "nqn.2014-08.org.nvmexpress.discovery"

#define NVMR_DEFAULT_KATO 0x1D4C0000
#define NVMR_DISCOVERY_KATO 0x0 /* DISCOVERY KATO has to be 0 */
#define NVMF_FCTYPE_PROPSET 0x0
#define NVMF_FCTYPE_CONNECT 0x1
#define NVMF_FCTYPE_PROPGET 0x4
#define NVMR_PSDT_SHIFT 6
#define NVMF_SINGLE_BUF_SGL (0x1 << NVMR_PSDT_SHIFT)
#define NVMF_MULT_SEG_SGL   (0x2 << NVMR_PSDT_SHIFT)
#define NVMF_KEYED_SGL_NO_INVALIDATE 0x40
#define NVMF_KEYED_SGL_INVALIDATE    0x4F

#define NUMIPV4OCTETS 4
typedef uint8_t nvmripv4_t[NUMIPV4OCTETS];

#define NUMSUBNQNBYTES 256
typedef char nvmrsubnqn_t[NUMSUBNQNBYTES+1];

#define NUMIPV4STRNGSZ 15
typedef char nvmripv4str_t[NUMIPV4STRNGSZ+1];

typedef enum {
	NVMR_PROPLEN_4BYTES = 0,
	NVMR_PROPLEN_8BYTES = 1,
	NVMR_PROPLEN_MAX
} nvmr_proplent_t;

#define MAX_NVMR_PROP_GET 0x12FFU
#define IDENTIFYLEN 4096

typedef struct {
	uint64_t nvmrcc_mqes  :16;
	uint64_t nvmrcc_cqr   : 1;
	uint64_t nvmrcc_ams   : 2;
	uint64_t nvmrcc_resv1 : 5;
	uint64_t nvmrcc_to    : 8;
	uint64_t nvmrcc_dstrd : 4;
	uint64_t nvmrcc_nssrs : 1;
	uint64_t nvmrcc_css   : 8;
	uint64_t nvmrcc_bps   : 1;
	uint64_t nvmrcc_resv2 : 2;
	uint64_t nvmrcc_mpsmin: 4;
	uint64_t nvmrcc_mpsmax: 4;
	uint64_t nvmrcc_resv3 : 8;
} nvmr_cntrlcap_t;
CTASSERT(sizeof(nvmr_cntrlcap_t) == sizeof(uint64_t));

#define NVMR_IOQID_ADDEND 1 /* Only a single AdminQ */
#define NVMR_PAYLOAD_UNIT 16U

#define MAX_ADMINQ_ELEMENTS 32

#endif /* _NVMR_SPEC_H */
