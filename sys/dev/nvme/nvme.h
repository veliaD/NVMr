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

#ifndef __NVME_H__
#define __NVME_H__

#ifdef _KERNEL
#include <sys/types.h>
#endif

#include <sys/param.h>
#include <sys/endian.h>

#include <dev/nvme/nvme_shared.h>

/*
 * Macros to deal with NVME revisions, as defined VS register
 */
#define NVME_REV(x, y)			(((x) << 16) | ((y) << 8))
#define NVME_MAJOR(r)			(((r) >> 16) & 0xffff)
#define NVME_MINOR(r)			(((r) >> 8) & 0xff)

/*
 * Use to mark a command to apply to all namespaces, or to retrieve global
 *  log pages.
 */
#define NVME_GLOBAL_NAMESPACE_TAG	((uint32_t)0xFFFFFFFF)

/* Register field definitions */
#define NVME_CAP_LO_REG_MQES_SHIFT			(0)
#define NVME_CAP_LO_REG_MQES_MASK			(0xFFFF)
#define NVME_CAP_LO_REG_CQR_SHIFT			(16)
#define NVME_CAP_LO_REG_CQR_MASK			(0x1)
#define NVME_CAP_LO_REG_AMS_SHIFT			(17)
#define NVME_CAP_LO_REG_AMS_MASK			(0x3)
#define NVME_CAP_LO_REG_TO_SHIFT			(24)
#define NVME_CAP_LO_REG_TO_MASK				(0xFF)

#define NVME_CAP_HI_REG_DSTRD_SHIFT			(0)
#define NVME_CAP_HI_REG_DSTRD_MASK			(0xF)
#define NVME_CAP_HI_REG_CSS_NVM_SHIFT			(5)
#define NVME_CAP_HI_REG_CSS_NVM_MASK			(0x1)
#define NVME_CAP_HI_REG_MPSMIN_SHIFT			(16)
#define NVME_CAP_HI_REG_MPSMIN_MASK			(0xF)
#define NVME_CAP_HI_REG_MPSMAX_SHIFT			(20)
#define NVME_CAP_HI_REG_MPSMAX_MASK			(0xF)

#define NVME_CSTS_REG_RDY_SHIFT				(0)
#define NVME_CSTS_REG_RDY_MASK				(0x1)
#define NVME_CSTS_REG_CFS_SHIFT				(1)
#define NVME_CSTS_REG_CFS_MASK				(0x1)
#define NVME_CSTS_REG_SHST_SHIFT			(2)
#define NVME_CSTS_REG_SHST_MASK				(0x3)

#define NVME_CSTS_GET_SHST(csts)			(((csts) >> NVME_CSTS_REG_SHST_SHIFT) & NVME_CSTS_REG_SHST_MASK)

#define NVME_AQA_REG_ASQS_SHIFT				(0)
#define NVME_AQA_REG_ASQS_MASK				(0xFFF)
#define NVME_AQA_REG_ACQS_SHIFT				(16)
#define NVME_AQA_REG_ACQS_MASK				(0xFFF)

/* Command field definitions */

#define NVME_CMD_FUSE_SHIFT				(8)
#define NVME_CMD_FUSE_MASK				(0x3)

#define NVME_PWR_ST_MPS_SHIFT				(0)
#define NVME_PWR_ST_MPS_MASK				(0x1)
#define NVME_PWR_ST_NOPS_SHIFT				(1)
#define NVME_PWR_ST_NOPS_MASK				(0x1)
#define NVME_PWR_ST_RRT_SHIFT				(0)
#define NVME_PWR_ST_RRT_MASK				(0x1F)
#define NVME_PWR_ST_RRL_SHIFT				(0)
#define NVME_PWR_ST_RRL_MASK				(0x1F)
#define NVME_PWR_ST_RWT_SHIFT				(0)
#define NVME_PWR_ST_RWT_MASK				(0x1F)
#define NVME_PWR_ST_RWL_SHIFT				(0)
#define NVME_PWR_ST_RWL_MASK				(0x1F)
#define NVME_PWR_ST_IPS_SHIFT				(6)
#define NVME_PWR_ST_IPS_MASK				(0x3)
#define NVME_PWR_ST_APW_SHIFT				(0)
#define NVME_PWR_ST_APW_MASK				(0x7)
#define NVME_PWR_ST_APS_SHIFT				(6)
#define NVME_PWR_ST_APS_MASK				(0x3)

/** Controller Multi-path I/O and Namespace Sharing Capabilities */
/* More then one port */
#define NVME_CTRLR_DATA_MIC_MPORTS_SHIFT		(0)
#define NVME_CTRLR_DATA_MIC_MPORTS_MASK			(0x1)
/* More then one controller */
#define NVME_CTRLR_DATA_MIC_MCTRLRS_SHIFT		(1)
#define NVME_CTRLR_DATA_MIC_MCTRLRS_MASK		(0x1)
/* SR-IOV Virtual Function */
#define NVME_CTRLR_DATA_MIC_SRIOVVF_SHIFT		(2)
#define NVME_CTRLR_DATA_MIC_SRIOVVF_MASK		(0x1)

/** OACS - optional admin command support */
/* supports security send/receive commands */
#define NVME_CTRLR_DATA_OACS_SECURITY_SHIFT		(0)
#define NVME_CTRLR_DATA_OACS_SECURITY_MASK		(0x1)
/* supports format nvm command */
#define NVME_CTRLR_DATA_OACS_FORMAT_SHIFT		(1)
#define NVME_CTRLR_DATA_OACS_FORMAT_MASK		(0x1)
/* supports firmware activate/download commands */
#define NVME_CTRLR_DATA_OACS_FIRMWARE_SHIFT		(2)
#define NVME_CTRLR_DATA_OACS_FIRMWARE_MASK		(0x1)
/* supports namespace management commands */
#define NVME_CTRLR_DATA_OACS_NSMGMT_SHIFT		(3)
#define NVME_CTRLR_DATA_OACS_NSMGMT_MASK		(0x1)
/* supports Device Self-test command */
#define NVME_CTRLR_DATA_OACS_SELFTEST_SHIFT		(4)
#define NVME_CTRLR_DATA_OACS_SELFTEST_MASK		(0x1)
/* supports Directives */
#define NVME_CTRLR_DATA_OACS_DIRECTIVES_SHIFT		(5)
#define NVME_CTRLR_DATA_OACS_DIRECTIVES_MASK		(0x1)
/* supports NVMe-MI Send/Receive */
#define NVME_CTRLR_DATA_OACS_NVMEMI_SHIFT		(6)
#define NVME_CTRLR_DATA_OACS_NVMEMI_MASK		(0x1)
/* supports Virtualization Management */
#define NVME_CTRLR_DATA_OACS_VM_SHIFT			(7)
#define NVME_CTRLR_DATA_OACS_VM_MASK			(0x1)
/* supports Doorbell Buffer Config */
#define NVME_CTRLR_DATA_OACS_DBBUFFER_SHIFT		(8)
#define NVME_CTRLR_DATA_OACS_DBBUFFER_MASK		(0x1)

/** firmware updates */
/* first slot is read-only */
#define NVME_CTRLR_DATA_FRMW_SLOT1_RO_SHIFT		(0)
#define NVME_CTRLR_DATA_FRMW_SLOT1_RO_MASK		(0x1)
/* number of firmware slots */
#define NVME_CTRLR_DATA_FRMW_NUM_SLOTS_SHIFT		(1)
#define NVME_CTRLR_DATA_FRMW_NUM_SLOTS_MASK		(0x7)

/** log page attributes */
/* per namespace smart/health log page */
#define NVME_CTRLR_DATA_LPA_NS_SMART_SHIFT		(0)
#define NVME_CTRLR_DATA_LPA_NS_SMART_MASK		(0x1)

/** AVSCC - admin vendor specific command configuration */
/* admin vendor specific commands use spec format */
#define NVME_CTRLR_DATA_AVSCC_SPEC_FORMAT_SHIFT		(0)
#define NVME_CTRLR_DATA_AVSCC_SPEC_FORMAT_MASK		(0x1)

/** Autonomous Power State Transition Attributes */
/* Autonomous Power State Transitions supported */
#define NVME_CTRLR_DATA_APSTA_APST_SUPP_SHIFT		(0)
#define NVME_CTRLR_DATA_APSTA_APST_SUPP_MASK		(0x1)

/** submission queue entry size */
#define NVME_CTRLR_DATA_SQES_MIN_SHIFT			(0)
#define NVME_CTRLR_DATA_SQES_MIN_MASK			(0xF)
#define NVME_CTRLR_DATA_SQES_MAX_SHIFT			(4)
#define NVME_CTRLR_DATA_SQES_MAX_MASK			(0xF)

/** completion queue entry size */
#define NVME_CTRLR_DATA_CQES_MIN_SHIFT			(0)
#define NVME_CTRLR_DATA_CQES_MIN_MASK			(0xF)
#define NVME_CTRLR_DATA_CQES_MAX_SHIFT			(4)
#define NVME_CTRLR_DATA_CQES_MAX_MASK			(0xF)

/** optional nvm command support */
#define NVME_CTRLR_DATA_ONCS_COMPARE_SHIFT		(0)
#define NVME_CTRLR_DATA_ONCS_COMPARE_MASK		(0x1)
#define NVME_CTRLR_DATA_ONCS_WRITE_UNC_SHIFT		(1)
#define NVME_CTRLR_DATA_ONCS_WRITE_UNC_MASK		(0x1)
#define NVME_CTRLR_DATA_ONCS_DSM_SHIFT			(2)
#define NVME_CTRLR_DATA_ONCS_DSM_MASK			(0x1)
#define NVME_CTRLR_DATA_ONCS_WRZERO_SHIFT		(3)
#define NVME_CTRLR_DATA_ONCS_WRZERO_MASK		(0x1)
#define NVME_CTRLR_DATA_ONCS_SAVEFEAT_SHIFT		(4)
#define NVME_CTRLR_DATA_ONCS_SAVEFEAT_MASK		(0x1)
#define NVME_CTRLR_DATA_ONCS_RESERV_SHIFT		(5)
#define NVME_CTRLR_DATA_ONCS_RESERV_MASK		(0x1)
#define NVME_CTRLR_DATA_ONCS_TIMESTAMP_SHIFT		(6)
#define NVME_CTRLR_DATA_ONCS_TIMESTAMP_MASK		(0x1)

/** Fused Operation Support */
#define NVME_CTRLR_DATA_FUSES_CNW_SHIFT		(0)
#define NVME_CTRLR_DATA_FUSES_CNW_MASK		(0x1)

/** Format NVM Attributes */
#define NVME_CTRLR_DATA_FNA_FORMAT_ALL_SHIFT		(0)
#define NVME_CTRLR_DATA_FNA_FORMAT_ALL_MASK		(0x1)
#define NVME_CTRLR_DATA_FNA_ERASE_ALL_SHIFT		(1)
#define NVME_CTRLR_DATA_FNA_ERASE_ALL_MASK		(0x1)
#define NVME_CTRLR_DATA_FNA_CRYPTO_ERASE_SHIFT		(2)
#define NVME_CTRLR_DATA_FNA_CRYPTO_ERASE_MASK		(0x1)

/** volatile write cache */
#define NVME_CTRLR_DATA_VWC_PRESENT_SHIFT		(0)
#define NVME_CTRLR_DATA_VWC_PRESENT_MASK		(0x1)

/** namespace features */
/* thin provisioning */
#define NVME_NS_DATA_NSFEAT_THIN_PROV_SHIFT		(0)
#define NVME_NS_DATA_NSFEAT_THIN_PROV_MASK		(0x1)
/* NAWUN, NAWUPF, and NACWU fields are valid */
#define NVME_NS_DATA_NSFEAT_NA_FIELDS_SHIFT		(1)
#define NVME_NS_DATA_NSFEAT_NA_FIELDS_MASK		(0x1)
/* Deallocated or Unwritten Logical Block errors supported */
#define NVME_NS_DATA_NSFEAT_DEALLOC_SHIFT		(2)
#define NVME_NS_DATA_NSFEAT_DEALLOC_MASK		(0x1)
/* NGUID and EUI64 fields are not reusable */
#define NVME_NS_DATA_NSFEAT_NO_ID_REUSE_SHIFT		(3)
#define NVME_NS_DATA_NSFEAT_NO_ID_REUSE_MASK		(0x1)

/** formatted lba size */
#define NVME_NS_DATA_FLBAS_FORMAT_SHIFT			(0)
#define NVME_NS_DATA_FLBAS_FORMAT_MASK			(0xF)
#define NVME_NS_DATA_FLBAS_EXTENDED_SHIFT		(4)
#define NVME_NS_DATA_FLBAS_EXTENDED_MASK		(0x1)

/** metadata capabilities */
/* metadata can be transferred as part of data prp list */
#define NVME_NS_DATA_MC_EXTENDED_SHIFT			(0)
#define NVME_NS_DATA_MC_EXTENDED_MASK			(0x1)
/* metadata can be transferred with separate metadata pointer */
#define NVME_NS_DATA_MC_POINTER_SHIFT			(1)
#define NVME_NS_DATA_MC_POINTER_MASK			(0x1)

/** end-to-end data protection capabilities */
/* protection information type 1 */
#define NVME_NS_DATA_DPC_PIT1_SHIFT			(0)
#define NVME_NS_DATA_DPC_PIT1_MASK			(0x1)
/* protection information type 2 */
#define NVME_NS_DATA_DPC_PIT2_SHIFT			(1)
#define NVME_NS_DATA_DPC_PIT2_MASK			(0x1)
/* protection information type 3 */
#define NVME_NS_DATA_DPC_PIT3_SHIFT			(2)
#define NVME_NS_DATA_DPC_PIT3_MASK			(0x1)
/* first eight bytes of metadata */
#define NVME_NS_DATA_DPC_MD_START_SHIFT			(3)
#define NVME_NS_DATA_DPC_MD_START_MASK			(0x1)
/* last eight bytes of metadata */
#define NVME_NS_DATA_DPC_MD_END_SHIFT			(4)
#define NVME_NS_DATA_DPC_MD_END_MASK			(0x1)

/** end-to-end data protection type settings */
/* protection information type */
#define NVME_NS_DATA_DPS_PIT_SHIFT			(0)
#define NVME_NS_DATA_DPS_PIT_MASK			(0x7)
/* 1 == protection info transferred at start of metadata */
/* 0 == protection info transferred at end of metadata */
#define NVME_NS_DATA_DPS_MD_START_SHIFT			(3)
#define NVME_NS_DATA_DPS_MD_START_MASK			(0x1)

/** Namespace Multi-path I/O and Namespace Sharing Capabilities */
/* the namespace may be attached to two or more controllers */
#define NVME_NS_DATA_NMIC_MAY_BE_SHARED_SHIFT		(0)
#define NVME_NS_DATA_NMIC_MAY_BE_SHARED_MASK		(0x1)

/** Reservation Capabilities */
/* Persist Through Power Loss */
#define NVME_NS_DATA_RESCAP_PTPL_SHIFT		(0)
#define NVME_NS_DATA_RESCAP_PTPL_MASK		(0x1)
/* supports the Write Exclusive */
#define NVME_NS_DATA_RESCAP_WR_EX_SHIFT		(1)
#define NVME_NS_DATA_RESCAP_WR_EX_MASK		(0x1)
/* supports the Exclusive Access */
#define NVME_NS_DATA_RESCAP_EX_AC_SHIFT		(2)
#define NVME_NS_DATA_RESCAP_EX_AC_MASK		(0x1)
/* supports the Write Exclusive – Registrants Only */
#define NVME_NS_DATA_RESCAP_WR_EX_RO_SHIFT	(3)
#define NVME_NS_DATA_RESCAP_WR_EX_RO_MASK	(0x1)
/* supports the Exclusive Access - Registrants Only */
#define NVME_NS_DATA_RESCAP_EX_AC_RO_SHIFT	(4)
#define NVME_NS_DATA_RESCAP_EX_AC_RO_MASK	(0x1)
/* supports the Write Exclusive – All Registrants */
#define NVME_NS_DATA_RESCAP_WR_EX_AR_SHIFT	(5)
#define NVME_NS_DATA_RESCAP_WR_EX_AR_MASK	(0x1)
/* supports the Exclusive Access - All Registrants */
#define NVME_NS_DATA_RESCAP_EX_AC_AR_SHIFT	(6)
#define NVME_NS_DATA_RESCAP_EX_AC_AR_MASK	(0x1)
/* Ignore Existing Key is used as defined in revision 1.3 or later */
#define NVME_NS_DATA_RESCAP_IEKEY13_SHIFT	(7)
#define NVME_NS_DATA_RESCAP_IEKEY13_MASK	(0x1)

/** Format Progress Indicator */
/* percentage of the Format NVM command that remains to be completed */
#define NVME_NS_DATA_FPI_PERC_SHIFT		(0)
#define NVME_NS_DATA_FPI_PERC_MASK		(0x7f)
/* namespace supports the Format Progress Indicator */
#define NVME_NS_DATA_FPI_SUPP_SHIFT		(7)
#define NVME_NS_DATA_FPI_SUPP_MASK		(0x1)

/** lba format support */
/* metadata size */
#define NVME_NS_DATA_LBAF_MS_SHIFT			(0)
#define NVME_NS_DATA_LBAF_MS_MASK			(0xFFFF)
/* lba data size */
#define NVME_NS_DATA_LBAF_LBADS_SHIFT			(16)
#define NVME_NS_DATA_LBAF_LBADS_MASK			(0xFF)
/* relative performance */
#define NVME_NS_DATA_LBAF_RP_SHIFT			(24)
#define NVME_NS_DATA_LBAF_RP_MASK			(0x3)

enum nvme_critical_warning_state {
	NVME_CRIT_WARN_ST_AVAILABLE_SPARE		= 0x1,
	NVME_CRIT_WARN_ST_TEMPERATURE			= 0x2,
	NVME_CRIT_WARN_ST_DEVICE_RELIABILITY		= 0x4,
	NVME_CRIT_WARN_ST_READ_ONLY			= 0x8,
	NVME_CRIT_WARN_ST_VOLATILE_MEMORY_BACKUP	= 0x10,
};
#define NVME_CRIT_WARN_ST_RESERVED_MASK			(0xE0)

/* slot for current FW */
#define NVME_FIRMWARE_PAGE_AFI_SLOT_SHIFT		(0)
#define NVME_FIRMWARE_PAGE_AFI_SLOT_MASK		(0x7)

/* CC register SHN field values */
enum shn_value {
	NVME_SHN_NORMAL		= 0x1,
	NVME_SHN_ABRUPT		= 0x2,
};

/* CSTS register SHST field values */
enum shst_value {
	NVME_SHST_NORMAL	= 0x0,
	NVME_SHST_OCCURRING	= 0x1,
	NVME_SHST_COMPLETE	= 0x2,
};

struct nvme_registers
{
	/** controller capabilities */
	uint32_t		cap_lo;
	uint32_t		cap_hi;

	uint32_t		vs;	/* version */
	uint32_t		intms;	/* interrupt mask set */
	uint32_t		intmc;	/* interrupt mask clear */

	/** controller configuration */
	uint32_t		cc;

	uint32_t		reserved1;

	/** controller status */
	uint32_t		csts;

	uint32_t		reserved2;

	/** admin queue attributes */
	uint32_t		aqa;

	uint64_t		asq;	/* admin submission queue base addr */
	uint64_t		acq;	/* admin completion queue base addr */
	uint32_t		reserved3[0x3f2];

	struct {
	    uint32_t		sq_tdbl; /* submission queue tail doorbell */
	    uint32_t		cq_hdbl; /* completion queue head doorbell */
	} doorbell[1] __packed;
} __packed;

_Static_assert(sizeof(struct nvme_registers) == 0x1008, "bad size for nvme_registers");

struct nvme_dsm_range {
	uint32_t attributes;
	uint32_t length;
	uint64_t starting_lba;
} __packed;

/* Largest DSM Trim that can be done */
#define NVME_MAX_DSM_TRIM		4096

_Static_assert(sizeof(struct nvme_dsm_range) == 16, "bad size for nvme_dsm_ranage");

/* command specific status codes */
enum nvme_command_specific_status_code {
	NVME_SC_COMPLETION_QUEUE_INVALID	= 0x00,
	NVME_SC_INVALID_QUEUE_IDENTIFIER	= 0x01,
	NVME_SC_MAXIMUM_QUEUE_SIZE_EXCEEDED	= 0x02,
	NVME_SC_ABORT_COMMAND_LIMIT_EXCEEDED	= 0x03,
	/* 0x04 - reserved */
	NVME_SC_ASYNC_EVENT_REQUEST_LIMIT_EXCEEDED = 0x05,
	NVME_SC_INVALID_FIRMWARE_SLOT		= 0x06,
	NVME_SC_INVALID_FIRMWARE_IMAGE		= 0x07,
	NVME_SC_INVALID_INTERRUPT_VECTOR	= 0x08,
	NVME_SC_INVALID_LOG_PAGE		= 0x09,
	NVME_SC_INVALID_FORMAT			= 0x0a,
	NVME_SC_FIRMWARE_REQUIRES_RESET		= 0x0b,
	NVME_SC_INVALID_QUEUE_DELETION		= 0x0c,
	NVME_SC_FEATURE_NOT_SAVEABLE		= 0x0d,
	NVME_SC_FEATURE_NOT_CHANGEABLE		= 0x0e,
	NVME_SC_FEATURE_NOT_NS_SPECIFIC		= 0x0f,
	NVME_SC_FW_ACT_REQUIRES_NVMS_RESET	= 0x10,
	NVME_SC_FW_ACT_REQUIRES_RESET		= 0x11,
	NVME_SC_FW_ACT_REQUIRES_TIME		= 0x12,
	NVME_SC_FW_ACT_PROHIBITED		= 0x13,
	NVME_SC_OVERLAPPING_RANGE		= 0x14,
	NVME_SC_NS_INSUFFICIENT_CAPACITY	= 0x15,
	NVME_SC_NS_ID_UNAVAILABLE		= 0x16,
	/* 0x17 - reserved */
	NVME_SC_NS_ALREADY_ATTACHED		= 0x18,
	NVME_SC_NS_IS_PRIVATE			= 0x19,
	NVME_SC_NS_NOT_ATTACHED			= 0x1a,
	NVME_SC_THIN_PROV_NOT_SUPPORTED		= 0x1b,
	NVME_SC_CTRLR_LIST_INVALID		= 0x1c,
	NVME_SC_SELT_TEST_IN_PROGRESS		= 0x1d,
	NVME_SC_BOOT_PART_WRITE_PROHIB		= 0x1e,
	NVME_SC_INVALID_CTRLR_ID		= 0x1f,
	NVME_SC_INVALID_SEC_CTRLR_STATE		= 0x20,
	NVME_SC_INVALID_NUM_OF_CTRLR_RESRC	= 0x21,
	NVME_SC_INVALID_RESOURCE_ID		= 0x22,

	NVME_SC_CONFLICTING_ATTRIBUTES		= 0x80,
	NVME_SC_INVALID_PROTECTION_INFO		= 0x81,
	NVME_SC_ATTEMPTED_WRITE_TO_RO_PAGE	= 0x82,
};

/* media error status codes */
enum nvme_media_error_status_code {
	NVME_SC_WRITE_FAULTS			= 0x80,
	NVME_SC_UNRECOVERED_READ_ERROR		= 0x81,
	NVME_SC_GUARD_CHECK_ERROR		= 0x82,
	NVME_SC_APPLICATION_TAG_CHECK_ERROR	= 0x83,
	NVME_SC_REFERENCE_TAG_CHECK_ERROR	= 0x84,
	NVME_SC_COMPARE_FAILURE			= 0x85,
	NVME_SC_ACCESS_DENIED			= 0x86,
	NVME_SC_DEALLOCATED_OR_UNWRITTEN	= 0x87,
};

/* nvme nvm opcodes */
enum nvme_nvm_opcode {
	NVME_OPC_FLUSH				= 0x00,
	NVME_OPC_WRITE				= 0x01,
	NVME_OPC_READ				= 0x02,
	/* 0x03 - reserved */
	NVME_OPC_WRITE_UNCORRECTABLE		= 0x04,
	NVME_OPC_COMPARE			= 0x05,
	/* 0x06 - reserved */
	NVME_OPC_WRITE_ZEROES			= 0x08,
	/* 0x07 - reserved */
	NVME_OPC_DATASET_MANAGEMENT		= 0x09,
	/* 0x0a-0x0c - reserved */
	NVME_OPC_RESERVATION_REGISTER		= 0x0d,
	NVME_OPC_RESERVATION_REPORT		= 0x0e,
	/* 0x0f-0x10 - reserved */
	NVME_OPC_RESERVATION_ACQUIRE		= 0x11,
	/* 0x12-0x14 - reserved */
	NVME_OPC_RESERVATION_RELEASE		= 0x15,
};

enum nvme_dsm_attribute {
	NVME_DSM_ATTR_INTEGRAL_READ		= 0x1,
	NVME_DSM_ATTR_INTEGRAL_WRITE		= 0x2,
	NVME_DSM_ATTR_DEALLOCATE		= 0x4,
};

enum nvme_activate_action {
	NVME_AA_REPLACE_NO_ACTIVATE		= 0x0,
	NVME_AA_REPLACE_ACTIVATE		= 0x1,
	NVME_AA_ACTIVATE			= 0x2,
};

struct nvme_error_information_entry {

	uint64_t		error_count;
	uint16_t		sqid;
	uint16_t		cid;
	uint16_t		status;
	uint16_t		error_location;
	uint64_t		lba;
	uint32_t		nsid;
	uint8_t			vendor_specific;
	uint8_t			reserved[35];
} __packed __aligned(4);

_Static_assert(sizeof(struct nvme_error_information_entry) == 64, "bad size for nvme_error_information_entry");

#define TRTYPE_RDMA 0x1

struct nvmr_discovery_log_entry {
	uint8_t			nvmrdle_trtype;
	uint8_t			nvmrdle_adrfam;
	uint8_t			nvmrdle_subtype;
	uint8_t			nvmrdle_treq;
	uint16_t		nvmrdle_portid;
	uint16_t		nvmrdle_cntlid;
	uint16_t		nvmrdle_asqsz;
	uint8_t			nvmrdle_resv0[22];
	uint8_t			nvmrdle_trsvcid[32];
	uint8_t			nvmrdle_resv1[192];
	uint8_t			nvmrdle_subnqn[256];
	uint8_t			nvmrdle_traddr[256];
	uint8_t			nvmrdle_resv2[256];
} __packed __aligned(4);
_Static_assert(sizeof(struct nvmr_discovery_log_entry) == 1024, "bad size for nvmr_discovery_log_entry");

struct nvmr_discovery_log_page {
	uint64_t			nvmrdlp_genctr;
	uint64_t			nvmrdlp_numrec;
	uint16_t			nvmrdlp_recfmt;
	uint8_t				nvmrdlp_resv0[1006];
	struct nvmr_discovery_log_entry	nvmrdlp_dle[0];
} __packed __aligned(4);
_Static_assert(sizeof(struct nvmr_discovery_log_page) == 1024, "bad size for nvmr_discovery_log_page");

struct nvme_health_information_page {

	uint8_t			critical_warning;
	uint16_t		temperature;
	uint8_t			available_spare;
	uint8_t			available_spare_threshold;
	uint8_t			percentage_used;

	uint8_t			reserved[26];

	/*
	 * Note that the following are 128-bit values, but are
	 *  defined as an array of 2 64-bit values.
	 */
	/* Data Units Read is always in 512-byte units. */
	uint64_t		data_units_read[2];
	/* Data Units Written is always in 512-byte units. */
	uint64_t		data_units_written[2];
	/* For NVM command set, this includes Compare commands. */
	uint64_t		host_read_commands[2];
	uint64_t		host_write_commands[2];
	/* Controller Busy Time is reported in minutes. */
	uint64_t		controller_busy_time[2];
	uint64_t		power_cycles[2];
	uint64_t		power_on_hours[2];
	uint64_t		unsafe_shutdowns[2];
	uint64_t		media_errors[2];
	uint64_t		num_error_info_log_entries[2];
	uint32_t		warning_temp_time;
	uint32_t		error_temp_time;
	uint16_t		temp_sensor[8];

	uint8_t			reserved2[296];
} __packed __aligned(4);

_Static_assert(sizeof(struct nvme_health_information_page) == 512, "bad size for nvme_health_information_page");

struct nvme_firmware_page {

	uint8_t			afi;
	uint8_t			reserved[7];
	uint64_t		revision[7]; /* revisions for 7 slots */
	uint8_t			reserved2[448];
} __packed __aligned(4);

_Static_assert(sizeof(struct nvme_firmware_page) == 512, "bad size for nvme_firmware_page");

struct nvme_ns_list {
	uint32_t		ns[1024];
} __packed __aligned(4);

_Static_assert(sizeof(struct nvme_ns_list) == 4096, "bad size for nvme_ns_list");

struct intel_log_temp_stats
{
	uint64_t	current;
	uint64_t	overtemp_flag_last;
	uint64_t	overtemp_flag_life;
	uint64_t	max_temp;
	uint64_t	min_temp;
	uint64_t	_rsvd[5];
	uint64_t	max_oper_temp;
	uint64_t	min_oper_temp;
	uint64_t	est_offset;
} __packed __aligned(4);

_Static_assert(sizeof(struct intel_log_temp_stats) == 13 * 8, "bad size for intel_log_temp_stats");

#define NVME_TEST_MAX_THREADS	128

struct nvme_io_test {

	enum nvme_nvm_opcode	opc;
	uint32_t		size;
	uint32_t		time;	/* in seconds */
	uint32_t		num_threads;
	uint32_t		flags;
	uint64_t		io_completed[NVME_TEST_MAX_THREADS];
};

enum nvme_io_test_flags {

	/*
	 * Specifies whether dev_refthread/dev_relthread should be
	 *  called during NVME_BIO_TEST.  Ignored for other test
	 *  types.
	 */
	NVME_TEST_FLAG_REFTHREAD =	0x1,
};

#define nvme_completion_is_error(cpl)					\
	(NVME_STATUS_GET_SC((cpl)->status) != 0 || NVME_STATUS_GET_SCT((cpl)->status) != 0)

void	nvme_strvis(uint8_t *dst, const uint8_t *src, int dstlen, int srclen);

#ifdef _KERNEL

struct bio;

struct nvme_namespace;
struct nvme_pci_controller;
struct nvme_consumer;

typedef void *(*nvme_cons_ns_fn_t)(struct nvme_namespace *, void *);
typedef void *(*nvme_cons_ctrlr_fn_t)(struct nvme_controller *);
typedef void (*nvme_cons_async_fn_t)(void *, const struct nvme_completion *,
				     uint32_t, void *, uint32_t);
typedef void (*nvme_cons_fail_fn_t)(void *);

enum nvme_namespace_flags {
	NVME_NS_DEALLOCATE_SUPPORTED	= 0x1,
	NVME_NS_FLUSH_SUPPORTED		= 0x2,
};

/* Admin functions */
void	nvme_ctrlr_cmd_set_feature(struct nvme_pci_controller *pctrlr,
				   uint8_t feature, uint32_t cdw11,
				   void *payload, uint32_t payload_size,
				   nvme_cb_fn_t cb_fn, void *cb_arg);
void	nvme_ctrlr_cmd_get_feature(struct nvme_pci_controller *pctrlr,
				   uint8_t feature, uint32_t cdw11,
				   void *payload, uint32_t payload_size,
				   nvme_cb_fn_t cb_fn, void *cb_arg);

/* NVM I/O functions */
int	nvme_ns_cmd_write(struct nvme_namespace *ns, void *payload,
			  uint64_t lba, uint32_t lba_count, nvme_cb_fn_t cb_fn,
			  void *cb_arg);
int	nvme_ns_cmd_write_bio(struct nvme_namespace *ns, struct bio *bp,
			      nvme_cb_fn_t cb_fn, void *cb_arg);
int	nvme_ns_cmd_read(struct nvme_namespace *ns, void *payload,
			 uint64_t lba, uint32_t lba_count, nvme_cb_fn_t cb_fn,
			 void *cb_arg);
int	nvme_ns_cmd_read_bio(struct nvme_namespace *ns, struct bio *bp,
			      nvme_cb_fn_t cb_fn, void *cb_arg);
int	nvme_ns_cmd_deallocate(struct nvme_namespace *ns, void *payload,
			       uint8_t num_ranges, nvme_cb_fn_t cb_fn,
			       void *cb_arg);
int	nvme_ns_cmd_flush(struct nvme_namespace *ns, nvme_cb_fn_t cb_fn,
			  void *cb_arg);
int	nvme_ns_dump(struct nvme_namespace *ns, void *virt, off_t offset,
		     size_t len);

/* Registration functions */
struct nvme_consumer *	nvme_register_consumer(nvme_cons_ns_fn_t    ns_fn,
					       nvme_cons_ctrlr_fn_t ctrlr_fn,
					       nvme_cons_async_fn_t async_fn,
					       nvme_cons_fail_fn_t  fail_fn);
void		nvme_unregister_consumer(struct nvme_consumer *consumer);

/* Controller helper functions */
device_t	nvme_ctrlr_get_device(struct nvme_pci_controller *pctrlr);
const struct nvme_controller_data *
		nvme_ctrlr_get_data(struct nvme_pci_controller *pctrlr);

/* Namespace helper functions */
uint32_t	nvme_ns_get_max_io_xfer_size(struct nvme_namespace *ns);
uint32_t	nvme_ns_get_sector_size(struct nvme_namespace *ns);
uint64_t	nvme_ns_get_num_sectors(struct nvme_namespace *ns);
uint64_t	nvme_ns_get_size(struct nvme_namespace *ns);
uint32_t	nvme_ns_get_flags(struct nvme_namespace *ns);
const char *	nvme_ns_get_serial_number(struct nvme_namespace *ns);
const char *	nvme_ns_get_model_number(struct nvme_namespace *ns);
const struct nvme_namespace_data *
		nvme_ns_get_data(struct nvme_namespace *ns);
uint32_t	nvme_ns_get_stripesize(struct nvme_namespace *ns);

int	nvme_ns_bio_process(struct nvme_namespace *ns, struct bio *bp,
			    nvme_cb_fn_t cb_fn);

/*
 * Command building helper functions -- shared with CAM
 * These functions assume allocator zeros out cmd structure
 * CAM's xpt_get_ccb and the request allocator for nvme both
 * do zero'd allocations.
 */
static inline
void	nvme_ns_flush_cmd(struct nvme_command *cmd, uint32_t nsid)
{

	cmd->opc = NVME_OPC_FLUSH;
	cmd->nsid = htole32(nsid);
}

static inline
void	nvme_ns_rw_cmd(struct nvme_command *cmd, uint32_t rwcmd, uint32_t nsid,
    uint64_t lba, uint32_t count)
{
	cmd->opc = rwcmd;
	cmd->nsid = htole32(nsid);
	cmd->cdw10 = htole32(lba & 0xffffffffu);
	cmd->cdw11 = htole32(lba >> 32);
	cmd->cdw12 = htole32(count-1);
}

static inline
void	nvme_ns_write_cmd(struct nvme_command *cmd, uint32_t nsid,
    uint64_t lba, uint32_t count)
{
	nvme_ns_rw_cmd(cmd, NVME_OPC_WRITE, nsid, lba, count);
}

static inline
void	nvme_ns_read_cmd(struct nvme_command *cmd, uint32_t nsid,
    uint64_t lba, uint32_t count)
{
	nvme_ns_rw_cmd(cmd, NVME_OPC_READ, nsid, lba, count);
}

static inline
void	nvme_ns_trim_cmd(struct nvme_command *cmd, uint32_t nsid,
    uint32_t num_ranges)
{
	cmd->opc = NVME_OPC_DATASET_MANAGEMENT;
	cmd->nsid = htole32(nsid);
	cmd->cdw10 = htole32(num_ranges - 1);
	cmd->cdw11 = htole32(NVME_DSM_ATTR_DEALLOCATE);
}

extern int nvme_use_nvd;

#endif /* _KERNEL */

static inline
void	nvme_power_state_swapbytes(struct nvme_power_state *s)
{

	s->mp = le16toh(s->mp);
	s->enlat = le32toh(s->enlat);
	s->exlat = le32toh(s->exlat);
	s->idlp = le16toh(s->idlp);
	s->actp = le16toh(s->actp);
}

static inline
void	nvme_controller_data_swapbytes(struct nvme_controller_data *s)
{
	int i;

	s->vid = le16toh(s->vid);
	s->ssvid = le16toh(s->ssvid);
	s->ctrlr_id = le16toh(s->ctrlr_id);
	s->ver = le32toh(s->ver);
	s->rtd3r = le32toh(s->rtd3r);
	s->rtd3e = le32toh(s->rtd3e);
	s->oaes = le32toh(s->oaes);
	s->ctratt = le32toh(s->ctratt);
	s->oacs = le16toh(s->oacs);
	s->wctemp = le16toh(s->wctemp);
	s->cctemp = le16toh(s->cctemp);
	s->mtfa = le16toh(s->mtfa);
	s->hmpre = le32toh(s->hmpre);
	s->hmmin = le32toh(s->hmmin);
	s->rpmbs = le32toh(s->rpmbs);
	s->edstt = le16toh(s->edstt);
	s->kas = le16toh(s->kas);
	s->hctma = le16toh(s->hctma);
	s->mntmt = le16toh(s->mntmt);
	s->mxtmt = le16toh(s->mxtmt);
	s->sanicap = le32toh(s->sanicap);
	s->maxcmd = le16toh(s->maxcmd);
	s->nn = le32toh(s->nn);
	s->oncs = le16toh(s->oncs);
	s->fuses = le16toh(s->fuses);
	s->awun = le16toh(s->awun);
	s->awupf = le16toh(s->awupf);
	s->acwu = le16toh(s->acwu);
	s->sgls = le32toh(s->sgls);
	for (i = 0; i < 32; i++)
		nvme_power_state_swapbytes(&s->power_state[i]);
}

static inline
void	nvme_namespace_data_swapbytes(struct nvme_namespace_data *s)
{
	int i;

	s->nsze = le64toh(s->nsze);
	s->ncap = le64toh(s->ncap);
	s->nuse = le64toh(s->nuse);
	s->nawun = le16toh(s->nawun);
	s->nawupf = le16toh(s->nawupf);
	s->nacwu = le16toh(s->nacwu);
	s->nabsn = le16toh(s->nabsn);
	s->nabo = le16toh(s->nabo);
	s->nabspf = le16toh(s->nabspf);
	s->noiob = le16toh(s->noiob);
	for (i = 0; i < 16; i++)
		s->lbaf[i] = le32toh(s->lbaf[i]);
}

static inline
void	nvme_error_information_entry_swapbytes(struct nvme_error_information_entry *s)
{

	s->error_count = le64toh(s->error_count);
	s->sqid = le16toh(s->sqid);
	s->cid = le16toh(s->cid);
	s->status = le16toh(s->status);
	s->error_location = le16toh(s->error_location);
	s->lba = le64toh(s->lba);
	s->nsid = le32toh(s->nsid);
}

static inline
void	nvme_le128toh(void *p)
{
#if _BYTE_ORDER != _LITTLE_ENDIAN
	/* Swap 16 bytes in place */
	char *tmp = (char*)p;
	char b;
	int i;
	for (i = 0; i < 8; i++) {
		b = tmp[i];
		tmp[i] = tmp[15-i];
		tmp[15-i] = b;
	}
#else
	(void)p;
#endif
}

static inline
void	nvme_health_information_page_swapbytes(struct nvme_health_information_page *s)
{
	int i;

	s->temperature = le16toh(s->temperature);
	nvme_le128toh((void *)s->data_units_read);
	nvme_le128toh((void *)s->data_units_written);
	nvme_le128toh((void *)s->host_read_commands);
	nvme_le128toh((void *)s->host_write_commands);
	nvme_le128toh((void *)s->controller_busy_time);
	nvme_le128toh((void *)s->power_cycles);
	nvme_le128toh((void *)s->power_on_hours);
	nvme_le128toh((void *)s->unsafe_shutdowns);
	nvme_le128toh((void *)s->media_errors);
	nvme_le128toh((void *)s->num_error_info_log_entries);
	s->warning_temp_time = le32toh(s->warning_temp_time);
	s->error_temp_time = le32toh(s->error_temp_time);
	for (i = 0; i < 8; i++)
		s->temp_sensor[i] = le16toh(s->temp_sensor[i]);
}


static inline
void	nvme_firmware_page_swapbytes(struct nvme_firmware_page *s)
{
	int i;

	for (i = 0; i < 7; i++)
		s->revision[i] = le64toh(s->revision[i]);
}

static inline
void	nvme_ns_list_swapbytes(struct nvme_ns_list *s)
{
	int i;

	for (i = 0; i < 1024; i++)
		s->ns[i] = le32toh(s->ns[i]);
}

static inline
void	intel_log_temp_stats_swapbytes(struct intel_log_temp_stats *s)
{

	s->current = le64toh(s->current);
	s->overtemp_flag_last = le64toh(s->overtemp_flag_last);
	s->overtemp_flag_life = le64toh(s->overtemp_flag_life);
	s->max_temp = le64toh(s->max_temp);
	s->min_temp = le64toh(s->min_temp);
	/* omit _rsvd[] */
	s->max_oper_temp = le64toh(s->max_oper_temp);
	s->min_oper_temp = le64toh(s->min_oper_temp);
	s->est_offset = le64toh(s->est_offset);
}

#endif /* __NVME_H__ */
