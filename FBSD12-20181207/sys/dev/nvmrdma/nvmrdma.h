/*
 * Copyright (c) 2019 Dell Inc. or its subsidiaries. All Rights Reserved.
 */

#ifndef _NVDMRDMA_H
#define _NVDMRDMA_H

#include <sys/cdefs.h>
#include <sys/types.h>
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/systm.h>
#include <sys/module.h>

#include <netinet/in.h>

#include "opt_ddb.h"

#ifdef DDB
#include <sys/queue.h>
#include <ddb/ddb.h>
#endif /* DDB */

#include <linux/kernel.h>
#include <linux/netdevice.h>

#include <rdma/rdma_cm.h>
#include <rdma/ib_verbs.h>

#include <dev/nvme/nvme_shared.h>

#include <sys/epoch.h>

#include <dev/nvmrdma/nvmr_spec.h>

struct nvmr_ncmplcont {
	struct ib_cqe			nvmrsp_cqe;
	STAILQ_ENTRY(nvmr_ncmplcont)	nvmrsp_next;
	struct nvme_completion	        nvmrsp_nvmecmpl;
	u64				nvmrsp_dmaddr;
};
typedef struct nvmr_ncmplcont nvmr_ncmplcon_t;

typedef enum {
	NVMRSND_FREE = 0x58F29A,
	NVMRSND_IOQD = 0xA6DA4B,
	NVMRSND_CMPL = 0x8D908E
} nvmr_snd_state_t;

struct nvmr_qpair_tag;

struct nvmr_ncommcont {
	struct ib_cqe		     nvmrsnd_cqe;
	struct ib_cqe		     nvmrsnd_regcqe;
	STAILQ_ENTRY(nvmr_ncommcont) nvmrsnd_nextfree;
	nvmr_stub_t                 *nvmrsnd_nvmecomm;
	struct ib_mr                *nvmrsnd_mr;
	u64			     nvmrsnd_dmaddr;
	uint16_t                     nvmrsnd_cid;
	struct nvme_request         *nvmrsnd_req;
	struct callout               nvmrsnd_to;
	volatile nvmr_snd_state_t    nvmrsnd_state;
	struct nvmr_qpair_tag       *nvmrsnd_q;
	bool                         nvmrsnd_rspndd;
	bool                         nvmrsnd_rkeyvalid;
};
typedef struct nvmr_ncommcont nvmr_ncommcon_t;

#define MAX_SGS	2
#define CTASSERT_MAX_SGS_LARGE_ENOUGH_FOR(data_structure) \
    CTASSERT(MAX_SGS >= ((sizeof(data_structure)/PAGE_SIZE)+1))
CTASSERT_MAX_SGS_LARGE_ENOUGH_FOR(struct nvmrdma_connect_data);

struct nvmr_cntrlr_tag;
typedef void (*nvmr_crtcntrlrcb_t)(struct nvmr_cntrlr_tag *cntrlr);

typedef struct {
	uint32_t nvmrqp_numqueues;
	uint32_t nvmrqp_numqe;
	uint32_t nvmrqp_kato;
} nvmr_qprof_t;

typedef enum {
	NVMR_QTYPE_ADMIN = 0,
	NVMR_QTYPE_IO,
	NVMR_NUM_QTYPES
} nvmr_qndx_t;

typedef struct {
	nvmr_qprof_t nvmrp_qprofs[NVMR_NUM_QTYPES];
	bool         nvmrp_isdiscov;
} nvmr_cntrlrprof_t;

typedef enum {
	NVMRQ_PRE_INIT = 0,
	NVMRQ_PRE_ADDR_RESOLV,
	NVMRQ_ADDR_RESOLV_FAILED,
	NVMRQ_ADDR_RESOLV_SUCCEEDED,
	NVMRQ_PRE_ROUTE_RESOLV,
	NVMRQ_ROUTE_RESOLV_FAILED,
	NVMRQ_ROUTE_RESOLV_SUCCEEDED,
	NVMRQ_PRE_CONNECT,
	NVMRQ_CONNECT_FAILED,
	NVMRQ_CONNECT_SUCCEEDED,
} nvmr_qpair_state_t;

typedef struct nvmr_qpair_tag {
	struct rdma_cm_id             *nvmrq_cmid;
	struct nvmr_cntrlr_tag        *nvmrq_cntrlr; /* Owning Controller     */
	struct ib_pd                  *nvmrq_ibpd;
	struct ib_cq                  *nvmrq_ibcq;
	struct ib_qp                  *nvmrq_ibqp;
	nvmr_ncommcon_t              **nvmrq_commcid; /* CID > nvmr_ncommcont */
	STAILQ_HEAD(, nvmr_ncommcont)  nvmrq_comms;
	STAILQ_HEAD(, nvmr_ncmplcont)  nvmrq_cmpls;
	STAILQ_HEAD(, nvme_request)    nvmrq_defreqs;
	volatile nvmr_qpair_state_t    nvmrq_state;  /* nvmrctr_nvmec.lockc */
	int                            nvmrq_last_cm_status;
	uint16_t                       nvmrq_numsndqe;/* nvmrq_comms alloced  */
	uint16_t                       nvmrq_numrcvqe;/* nvmrq_cmpls alloced  */
	uint16_t                       nvmrq_numFsndqe;/* nvmrq_comms unused  */
	uint16_t                       nvmrq_numFrcvqe;/* nvmrq_cmpls unused  */
	struct scatterlist             nvmrq_scl[MAX_NVME_RDMA_SEGMENTS];
	volatile uint32_t              nvmrq_qediocnt; /*Active+Deferred IO   */
	volatile uint32_t              nvmrq_defiocnt; /*Deferred IO          */
	volatile uint64_t              nvmrq_stat_cmdcnt;

	struct nvme_qpair              nvmrq_gqp;
} *nvmr_qpair_t;


/*
 * An NVMr controller requires multiple memory allocations and n/w
 * interactions to be fully setup.  An initial admin QP is created which
 * is used to interrogate the properties of the remote NVMr controller.
 * Thereafter additional IO QPs are created. As soon as the admin QP is even
 * partly setup it can receive async events from the RDMA stack about
 * disconnects or RNIC device removals.  These removals should trigger a
 * teardown of the controller that could still be initializing.
 *
 * The same events can be generated once the controller has initialized.
 *
 * To deal with all the resulting scenarios a state variable has been introduced
 *
 * A taskqueue has been introduced to handle destroying a controller.
 * Controllers will not be tossed into this queue on a destruction event
 * if it is determined that it is still being initialized (NVMRC_PRE_INIT).
 * However they will be moved to the NVMRC_CONDEMNED state.
 * The NVMRC_CONDEMNED serves two purposes:
 * 1) Destruction/condemn events can work out that a previous condemn event has
 *    happened on a controller and that nothing more has to be done to
 *    condemn it.
 * 2) The initialization logic in nvmr_cntrlr_create() monitors the state to
 *    check if a condemn event has happened on a controller it is initializing.
 *    If so the initialization is aborted and the initialization logic
 *    tosses the controller onto the condemned taskqueue for destruction
 */
typedef enum {
	NVMRC_PRE_INIT = 0,
	NVMRC_INITED,
	NVMRC_CONDEMNED
} nvmr_cntrlr_state_t;

typedef enum {
	NVMR_CNTRLRLST_INVALID = 0,
	NVMR_CNTRLRLST_ACTIVE = 0x45FE129,
	NVMR_CNTRLRLST_CONDEMNED = 0xB9388F
} nvmr_cntrlr_lst_t;

typedef struct nvmr_cntrlr_tag {
	char			very_first_field[NVME_VFFSTRSZ+1];
	nvmrsubnqn_t       nvmrctr_subnqn;
	nvmripv4str_t      nvmrctr_ipv4str;
	nvmr_qpair_t       nvmrctr_adminqp;
	nvmr_qpair_t      *nvmrctr_ioqarr;  /* Array size determined by prof */
	nvmr_cntrlrprof_t *nvmrctr_prof;
	nvmripv4_t         nvmrctr_ipv4;
	uint16_t           nvmrctr_port;
	struct nvme_controller nvmrctr_nvmec;
	volatile nvmr_cntrlr_state_t nvmrctr_state;  /* nvmrctr_nvmec.lockc */
	nvmr_cntrlr_lst_t  nvmrctr_glblst;  /* nvmrctr_nvmec.lockc */
	TAILQ_ENTRY(nvmr_cntrlr_tag) nvmrctr_nxt;
	boolean_t          nvmrctr_nvmereg;
} *nvmr_cntrlr_t;

#define NVMR_NUMSNDSGE (1 + 1) /* NVMe Command  + Inline data */
#define NVMR_NUMRCVSGE 1       /* NVMe Completion */

#define Q_IS_FAILED(q) (atomic_load_acq_int(&q->nvmrq_gqp.qis_enabled) == FALSE)

typedef enum {
	NVMRCD_ENQUEUE = 0,
	NVMRCD_SKIP_ENQUEUE
} nvmr_condemned_disposition_t;

#define NVMR_ALLUNITNUMS (-1)

typedef struct {
	nvmr_portip_t nvmra_pi;
	char         *nvmra_subnqn;
} nvmr_addr_t;

typedef enum {
	NVMR_QCMD_INVALID = 0,
	NVMR_QCMD_RO,
	NVMR_QCMD_WO,
}  nvmr_ksgl_perm_t;

#define NVMRTO 3000

#define NVMR_STRING "NVMe over RDMA"
#define CONFIRMRDMACONTROLLER KASSERT(strncmp(cntrlr->very_first_field, \
    NVMR_STRING, sizeof(cntrlr->very_first_field)) == 0, \
    ("%s@%d NOT an RDMA controller!\n", __func__, __LINE__))
#define KASSERT_NVMR_CNTRLR(c) KASSERT((c)->nvmec_ttype == NVMET_RDMA, \
    ("%s@%d c:%p t:%d\n", __func__, __LINE__, (c), (c)->nvmec_ttype))

#endif /* _NVDMRDMA_H */
