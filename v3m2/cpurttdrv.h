/****************************************************************************/
/*
 * FILE          : cpurttdrv.h
 * DESCRIPTION   : CPU Runtime Test driver for sample code
 * CREATED       : 2021.08.26
 * MODIFIED      : 2021.10.19
 * AUTHOR        : Renesas Electronics Corporation
 * TARGET DEVICE : R-Car V3Mv2
 * TARGET OS     : BareMetal
 * HISTORY       :
 *                 2021.08.26 Create New File for SoC
 *                 2021.10.19 Modify the definition value used in A2 Runtime Test.
 *                            Move definition values that do not need to be shared with the user layer.
 */
/****************************************************************************/
/*
 * Copyright(C) 2021 Renesas Electronics Corporation. All Rights Reserved.
 * RENESAS ELECTRONICS CONFIDENTIAL AND PROPRIETARY
 * This program must be used solely for the purpose for which
 * it was furnished by Renesas Electronics Corporation.
 * No part of this program may be reproduced or disclosed to
 * others, in any form, without the prior written permission
 * of Renesas Electronics Corporation.
 *
 ****************************************************************************/

#ifndef CPURTTDRV_H
#define CPURTTDRV_H

#include "cpurtt_common.h"

#define UDF_CPURTT_UIO_DRIVER_NAME    "fbc_uio_share"     /* cpurtt driver name for uio */

#define DRV_CPURTTKER_CPUNUM_MAX 2U
#define DRV_CPURTTKER_SMONI_BUF_SIZE 128U

/* FieldBIST related defined values */
/* RTTFINISH1 REG */
#define DRV_RTTKER_RTTFINISH1 0xFF830014U

/* RTTEX REG */
#define DRV_RTTKER_IM0_RTTEX 0xFFF80000U
#define DRV_RTTKER_IM1_RTTEX 0xFFF90000U
#define DRV_RTTKER_IMP0_RTTEX 0xFF8D0000U
#define DRV_RTTKER_IMP1_RTTEX 0xFFF50000U
#define DRV_RTTKER_IMP2_RTTEX 0xFFF60000U
#define DRV_RTTKER_IMP3_RTTEX 0xFFF70000U
#define DRV_RTTKER_OCV0_RTTEX 0xFFFA0000U
#define DRV_RTTKER_OCV1_RTTEX 0xFFFB0000U
#define DRV_RTTKER_53D_RTTEX 0xFF87C000U
#define DRV_RTTKER_530_RTTEX 0xFF877000U
#define DRV_RTTKER_531_RTTEX 0xFF878000U

#define DRV_CPURTTKER_OFFSET_RTTEX 0x000U

/* Bit number of RTTFINISH1 register */
#define DRV_RTTKER_IM0_RTTFINISH1_BIT   2U
#define DRV_RTTKER_IM1_RTTFINISH1_BIT   3U
#define DRV_RTTKER_IMP0_RTTFINISH1_BIT  5U
#define DRV_RTTKER_IMP1_RTTFINISH1_BIT  6U
#define DRV_RTTKER_IMP2_RTTFINISH1_BIT  7U
#define DRV_RTTKER_IMP3_RTTFINISH1_BIT  8U
#define DRV_RTTKER_OCV0_RTTFINISH1_BIT  11U
#define DRV_RTTKER_OCV1_RTTFINISH1_BIT  12U
#define DRV_RTTKER_CA53D_RTTFINISH1_BIT 23U
#define DRV_RTTKER_CA530_RTTFINISH1_BIT 24U
#define DRV_RTTKER_CA531_RTTFINISH1_BIT 25U

/* Type of HIERARCHY */
#define DRV_RTTKER_HIERARCHY_CPU   0U
#define DRV_RTTKER_HIERARCHY_OTHER 1U

#define DRV_CPURTTKER_SGIR 0xF1010F00U
#define DRV_CPURTTKER_SGI_HIERARCHY530 0x00010004U
#define DRV_CPURTTKER_SGI_HIERARCHY531 0x00020004U
#define DRV_CPURTTKER_SGI_HIERARCHY53D 0x00020004U

#define DRV_CPURTTKER_ITARGETS_11      0xF101082CU
#define DRV_CPURTTKER_ITARGETS_11_MASK 0x00FF0000U
#define DRV_CPURTTKER_ITARGETS_11_CPU0 0x00010000U
#define DRV_CPURTTKER_ITARGETS_11_CPU1 0x00020000U

#define DRV_RTTKER_FIELD_BIST_INT_CPU 0x01U
#define DRV_RTTKER_AFFINITY_MASK_BIT 0x01U

/* return code */
#define FBIST_CB_CLOSE_REQ 1
#define FBIST_BUSCHECK_ERROR 2

/* A2 RuntimeTest Defined value for synchronization management  */
#define A2SYNC_CPU0_BIT 0x0001
#define A2SYNC_CPU1_BIT 0x0002
#define A2SYNC_ALL      0x0003

#define CB_QUEUE_STATUS_EMPTY   0x00
#define CB_QUEUE_STATUS_ENA     0x01
#define CB_QUEUE_STATUS_FULL    0x02

/* Data definition value to be set in the argument of R_SMONI_API_RuntimeTestA2Execute other than CPU0 */
#define DRV_RTTKER_A2_PARAM_SGI_DATA    0xFFFFFFFFU
#define DRV_RTTKER_A2_PARAM_RTTEX_DATA  0x00000000U

/* Definition of the kernel CPURTT device module name */
#define UDF_CPURTT_DRIVER_NAME        "cpurttdrv"     /* cpurtt driver name */
#define UDF_CPURTT_CLASS_NAME         "cpurttmod"     /* cpurtt driver class name */

/* Structure of RTTFINISH1 */
typedef union
{
    struct
    {
        uint32_t Reserved1 :2;
        uint32_t IM0 :1;
        uint32_t IM1 :1;
        uint32_t Reserved2 :1;
        uint32_t IMP0 :1;
        uint32_t IMP1 :1;
        uint32_t IMP2 :1;
        uint32_t IMP3 :1;
        uint32_t Reserved3 :2;
        uint32_t OCV0 :1;
        uint32_t OCV1 :1;
        uint32_t Reserved4 :10;
        uint32_t CA53D :1;
        uint32_t CA530 :1;
        uint32_t CA531 :1;
        uint32_t Reserved5 :6;
    } BIT;
    uint32_t INT;
} drvRTT_RTTFINISH1_t;

/* Structure of RTTEX */
typedef union
{
    struct
    {
        uint32_t EX :1;
        uint32_t STP :1;
        uint32_t TM :1;
        uint32_t WM :1;
        uint32_t TR :1;
        uint32_t SSZ :1;
        uint32_t Reserved1 :2;
        uint32_t STM :3;
        uint32_t Reserved2 :5;
        uint32_t KCD :8;
        uint32_t Reserved3 :4;
        uint32_t Reserved4 :1;
        uint32_t Reserved5 :1;
        uint32_t Reserved6 :2;
    } BIT;
    uint32_t INT;
} drvRTT_RTTEX_t;

typedef enum
{
    DRV_RTTKER_HIERARCHY_IM0,  /* Video Codec Hierarchy(IMR01) */
    DRV_RTTKER_HIERARCHY_IM1,  /* Video Codec Hierarchy(IMR23) */
    DRV_RTTKER_HIERARCHY_IMP0, /* Image Recognition Hierarchy (IMP core0, IMP core1) */
    DRV_RTTKER_HIERARCHY_IMP1, /* Image Recognition Hierarchy (IMP core2, IMP core3) */
    DRV_RTTKER_HIERARCHY_IMP2, /* Image Recognition Hierarchy (DMAC, PSC) */
    DRV_RTTKER_HIERARCHY_IMP3, /* Image Recognition Hierarchy (CNN) */
    DRV_RTTKER_HIERARCHY_OCV0, /* Image Recognition Hierarchy (OCV core0) */
    DRV_RTTKER_HIERARCHY_OCV1, /* Image Recognition Hierarchy (OCV core1) */
    DRV_RTTKER_HIERARCHY_53D,  /* Cortex-A53 Hierarchy L2 cache */
    DRV_RTTKER_HIERARCHY_530,  /* Cortex-A53 Hierarchy cpu0 */
    DRV_RTTKER_HIERARCHY_531,  /* Cortex-A53 Hierarchy cpu1 */
    DRV_RTTKER_HIERARCHY_MAX
} drvRTT_hierarchy_t;

typedef struct
{
    uint32_t mHierarchyType;
    drvRTT_hierarchy_t mHierarchy;
    uint32_t mRttfinishBit;
    uint32_t mRttexAdr;
} drvCPURTT_FbistHdr_t;

typedef struct
{
    uint8_t head;
    uint8_t pos;
    uint8_t status;
    drvCPURTT_CallbackInfo_t CbInfo[DRV_RTTKER_HIERARCHY_MAX];
}drvCPURTT_CbInfoQueue_t;

typedef int (*A2ThreadTable)(void*);

struct fbc_uio_share_soc_res {
	char **clk_names;
};

/* */
struct fbc_uio_share_platform_data {
	struct platform_device *pdev;
	struct uio_info *uio_info;
	void __iomem *base_addr[1];
	struct clk *clks[3];
	int clk_count;
    spinlock_t lock;
    unsigned long flags;
};

enum {
    UIO_IRQ_DISABLED = 0,
};

#endif
