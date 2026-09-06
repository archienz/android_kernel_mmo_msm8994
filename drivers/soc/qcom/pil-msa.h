/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __MSM_PIL_MSA_H
#define __MSM_PIL_MSA_H

#include <linux/dma-mapping.h>
#include <soc/qcom/subsystem_restart.h>

#include "peripheral-loader.h"

#define VDD_MSS_UV	1000000

struct modem_data {
	struct q6v5_data *q6;
	struct subsys_device *subsys;
	struct subsys_desc subsys_desc;
	void *ramdump_dev;
	bool crash_shutdown;
	bool ignore_errors;
	struct completion stop_ack;
	void __iomem *rmb_base;
	struct clk *xo;
	struct pil_desc desc;
	struct device mba_mem_dev;
	struct dma_attrs attrs_dma;
	/*
	 * Optional fixed, kernel-removed region for the MBA image and the
	 * modem metadata (DT qcom,mba-mem). 0 = allocate from CMA as usual.
	 */
	phys_addr_t mba_region_phys;
	size_t mba_region_size;
	void *lab_mdata_virt;
	dma_addr_t lab_mdata_phys;
	size_t lab_mdata_size;
	/* After META status==3: hive CODE_START + span/filesz for AUTH retry. */
	bool lab_meta_ok;
	phys_addr_t lab_auth_start;
	u32 lab_auth_span_len;
	u32 lab_auth_filesz_len;
};

void *pil_mss_mba_alloc(struct modem_data *md, size_t size, bool mdata,
			dma_addr_t *phys, struct dma_attrs *attrs);
void pil_mss_mba_free(struct modem_data *md, size_t size, void *virt,
		      dma_addr_t phys, struct dma_attrs *attrs);

extern struct pil_reset_ops pil_msa_mss_ops;
extern struct pil_reset_ops pil_msa_mss_ops_selfauth;
extern struct pil_reset_ops pil_msa_femto_mba_ops;

int pil_mss_reset_load_mba(struct pil_desc *pil);
int pil_mss_make_proxy_votes(struct pil_desc *pil);
void pil_mss_remove_proxy_votes(struct pil_desc *pil);
int pil_mss_shutdown(struct pil_desc *pil);
int pil_mss_deinit_image(struct pil_desc *pil);
int __pil_mss_deinit_image(struct pil_desc *pil, bool err_path);
#endif
