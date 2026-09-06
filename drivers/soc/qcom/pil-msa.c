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

#include <linux/module.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/dma-mapping.h>
#include <linux/elf.h>
#include <linux/sizes.h>
#include <linux/ktime.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/pfn.h>
#include <asm/cacheflush.h>
#include <soc/qcom/scm.h>
#include <linux/workqueue.h>

#include "peripheral-loader.h"
#include "pil-q6v5.h"
#include "pil-msa.h"

/* Q6 Register Offsets */
#define QDSP6SS_RST_EVB			0x010

/* AXI Halting Registers */
#define MSS_Q6_HALT_BASE		0x180
#define MSS_MODEM_HALT_BASE		0x200
#define MSS_NC_HALT_BASE		0x280

/* RMB Status Register Values */
#define STATUS_PBL_SUCCESS		0x1
#define STATUS_XPU_UNLOCKED		0x1
#define STATUS_XPU_UNLOCKED_SCRIBBLED	0x2

/* PBL/MBA interface registers */
#define RMB_MBA_IMAGE			0x00
#define RMB_PBL_STATUS			0x04
#define RMB_MBA_COMMAND			0x08
#define RMB_MBA_STATUS			0x0C
#define RMB_PMI_META_DATA		0x10
#define RMB_PMI_CODE_START		0x14
#define RMB_PMI_CODE_LENGTH		0x18
#define RMB_PROTOCOL_VERSION		0x1C
#define RMB_MBA_DEBUG_INFORMATION	0x20

#define POLL_INTERVAL_US		50

#define CMD_META_DATA_READY		0x1
#define CMD_LOAD_READY			0x2
#define CMD_PILFAIL_NFY_MBA		0xffffdead

#define STATUS_META_DATA_AUTH_SUCCESS	0x3
#define STATUS_AUTH_COMPLETE		0x4
#define STATUS_MBA_UNLOCKED		0x6

/* External BHS */
#define EXTERNAL_BHS_ON			BIT(0)
#define EXTERNAL_BHS_STATUS		BIT(4)
#define BHS_TIMEOUT_US			50

#define MSS_RESTART_PARAM_ID		0x2
#define MSS_RESTART_ID			0xA

#define MSS_MAGIC			0XAABADEAD

static int pbl_mba_boot_timeout_ms = 1000;
module_param(pbl_mba_boot_timeout_ms, int, S_IRUGO | S_IWUSR);

static int modem_auth_timeout_ms = 10000;
module_param(modem_auth_timeout_ms, int, S_IRUGO | S_IWUSR);

/* If set to 0xAABADEAD, MBA failures trigger a kernel panic */
static uint modem_trigger_panic;
module_param(modem_trigger_panic, uint, S_IRUGO | S_IWUSR);

/*
 * talkman lab knobs (/sys/module/pil_msa/parameters/). The Lumia 950 boot
 * chain (Windows UEFI + its TZ) answers every MBA staging address tried so
 * far with RMB_PBL_STATUS 0xEF1D0200. Windows' qcpil8994.sys asks TZ to
 * "define relocatable subsystem memory" for the MBA region before it
 * releases the Q6; stock CAF 3.10 never talks to TZ for an MSA modem. These
 * knobs let one boot try the TZ call, other staging addresses and a full
 * MSS reset cycle, with the RMB block dumped before the Q6 is released so a
 * stale status cannot be mistaken for a fresh one.
 *
 * lab_mba_stage / lab_mdata_stage: physical address to stage the MBA /
 *   modem metadata at instead of the DT / CMA default (0 = default).
 * lab_tz_mask: bit0 PAS_MEM_SETUP(pas_id, modem region)
 *              bit1 PAS_MEM_SETUP(pas_id, MBA staging, 1 MB)
 *              bit2 PAS_MEM_SETUP(pas_id, 0x06F00000, 0x180000) (ACPI RMTB)
 *              bit3 also try the same calls with every PAS id 0..7, log only
 *              bit7 re-run the PAS_IS_SUPPORTED probe (ids 0..11)
 * lab_tz_pas_id: TZ processor id for bit0..2 (5 = PAS_MODEM_FW, 0 = PAS_MODEM)
 * lab_pre_reset: assert MSS restart, wait, deassert before the boot attempt
 */
static ulong lab_mba_stage;
module_param(lab_mba_stage, ulong, S_IRUGO | S_IWUSR);
static ulong lab_mdata_stage;
module_param(lab_mdata_stage, ulong, S_IRUGO | S_IWUSR);
static uint lab_tz_mask;
module_param(lab_tz_mask, uint, S_IRUGO | S_IWUSR);
static uint lab_tz_pas_id = 9;
module_param(lab_tz_pas_id, uint, S_IRUGO | S_IWUSR);
static uint lab_pre_reset;
module_param(lab_pre_reset, uint, S_IRUGO | S_IWUSR);
/*
 * TZ PIL SHARE_MEMORY. 0 size disables.
 * lab_share_fnid=0 (default): CAF 4-arg 0x0200020B
 *   {phys, size, flags, 3}. #26 returned -22.
 * lab_share_fnid=1 or 0x02000202: Windows FUN_0040a310 3-word
 *   SHARE {phys, 0, size} (A5). Never PAS 5/6.
 * lab_share_phys 0x07000000 + BoardConfig size 0x5E00000 covers
 *   hash 0x0CA through 0x0CE. Not a reserved-memory node.
 */
static ulong lab_share_phys = 0x07000000;
module_param(lab_share_phys, ulong, S_IRUGO | S_IWUSR);
static ulong lab_share_size = 0x00400000;
module_param(lab_share_size, ulong, S_IRUGO | S_IWUSR);
static uint lab_share_flags = 1;
module_param(lab_share_flags, uint, S_IRUGO | S_IWUSR);
static uint lab_share_fnid;
module_param(lab_share_fnid, uint, S_IRUGO | S_IWUSR);
/*
 * PIL cmd 9: qcpil "unlock subsystem memory, TZ processor ID".
 * Available on this Windows TZ (probe listed cmd 9). 0 disables.
 */
static uint lab_unlock = 1;
module_param(lab_unlock, uint, S_IRUGO | S_IWUSR);
/* 1 = RMB_PMI_CODE_LENGTH is ELF p_paddr span, not filesz sum.
 * BoardConfig keeps this 1 (A4 hive/span). lab_auth_filesz overrides.
 */
static uint lab_code_span;
module_param(lab_code_span, uint, S_IRUGO | S_IWUSR);
/* PAS_INIT_IMAGE(cmd 1) with packed metadata (SCM_RW). 0 disables. */
static uint lab_init_image = 1;
module_param(lab_init_image, uint, S_IRUGO | S_IWUSR);
/* lab: if nonzero, this value is written to RMB_MBA_IMAGE instead of the staging address (0xffffffff writes 0) */
static uint lab_rmb_image;
module_param(lab_rmb_image, uint, S_IRUGO | S_IWUSR);

/*
 * lab_mba_unwrap: the Lumia mba.b00/mba.mbn is an ELF wrapper (one PT_LOAD
 * at file offset 0x1000) around the flat MBN the 8992 PBL expects. Windows'
 * qcpil8994 loads the segment; stock CAF copies the file. 1 = load segment.
 * lab_win_reset: replay the QDSP6SS bring-up qcsubsys8992.sys uses on 8992
 * instead of __pil_q6v55_reset (absolute PWR_CTL writes, GFMUX_CTL 0x102).
 * lab_hold: after a PBL/MBA verdict leave the MSS powered and clocked so
 * lab_peek can inspect the MSS side (RMB, TCM at 0xfcc08000).
 * lab_peek: "<phys> <nwords>" ioremaps and prints 32-bit words.
 */
static uint lab_mba_unwrap = 1;
module_param(lab_mba_unwrap, uint, S_IRUGO | S_IWUSR);
static uint lab_win_reset;
module_param(lab_win_reset, uint, S_IRUGO | S_IWUSR);
static uint lab_hold;
module_param(lab_hold, uint, S_IRUGO | S_IWUSR);
/*
 * Windows FUN_004068d0: copy PT_LOAD into the hive before CMD_META_DATA_READY
 * and keep the metadata MDL until AUTH_COMPLETE. 1 = do that (m18).
 * 0 = ring META first (m36): m35 4ee7 died during preload seg 6
 * (filesz 0x280000 at 0x075c0000) before after-META STATUS could print.
 */
static uint lab_load_before_auth;
module_param(lab_load_before_auth, uint, S_IRUGO | S_IWUSR);
/*
 * After the hive copy, also place PT_LOAD at ELF p_paddr 0x07000000
 * (already memory_hole / ramoops). 0 size disables. Not a DT reserved-memory.
 */
static ulong lab_mirror_phys = 0x07000000;
module_param(lab_mirror_phys, ulong, S_IRUGO | S_IWUSR);
static ulong lab_mirror_size;
module_param(lab_mirror_size, ulong, S_IRUGO | S_IWUSR);
/*
 * RMB_PMI_CODE_START after META. 0 = CMA hive (0x07400000). Windows
 * qcsubsys never mentions 0x07400000 and uses ELF 0x07000000 19 times.
 */
static ulong lab_code_start;
module_param(lab_code_start, ulong, S_IRUGO | S_IWUSR);
/* 1 = load PT_LOAD at ELF p_paddr (0x07000000…) via ioremap, not +4MiB CMA. */
static uint lab_identity = 1;
module_param(lab_identity, uint, S_IRUGO | S_IWUSR);
/*
 * 1 = rewrite reloc PT_LOAD p_paddr in packed metadata to the CMA hive.
 * MBA DEBUG 7 walks those addresses; #26 left them at 0x07000000.
 * Do not combine with lab_identity.
 */
static uint lab_reloc_headers = 1;
module_param(lab_reloc_headers, uint, S_IRUGO | S_IWUSR);
/* Copy hash bytes to ELF hash p_paddr (0x0CA00000) if that is inside CMA. */
static uint lab_place_hash = 1;
module_param(lab_place_hash, uint, S_IRUGO | S_IWUSR);
/*
 * 1 = Windows MBA-visible hash PHDR: leave p_paddr at ELF 0x0CA00000
 *     (4K-aligned, p_align 0x1000). qcpil FUN_004065a8 copies Ehdr+PHDRs
 *     before FUN_004066fc writes the working dest p_paddr = packed+0x334,
 *     so the blob MBA parses still says 0x0CA. #26 wrote packed+0x334
 *     into that PHDR → DEBUG 7 ("unaligned address").
 * 0 = rewrite p_paddr to packed_phys+4K (previous untested workaround).
 */
static uint lab_win_hash = 1;
module_param(lab_win_hash, uint, S_IRUGO | S_IWUSR);
/*
 * After measured META status==3, AUTH CODE_LENGTH:
 *   0 = hive/span (A4 default, 0x07400000 / 0x05A00000 with lab_code_span=1)
 *   1 = filesz sum instead of span (Windows FUN_0040c740 = 0x02BD3367)
 *   2 = first AUTH with span; if MBA returns status<0, rewrite LENGTH to
 *       filesz and ring CMD_LOAD_READY once more (same hive CODE_START).
 * CODE_START stays the CMA hive. Never identity-map 0x070 (m11).
 */
static uint lab_auth_filesz;
module_param(lab_auth_filesz, uint, S_IRUGO | S_IWUSR);

#define TALKMAN_ELF_HOLE	0x07000000UL
#define TALKMAN_HIVE_BASE	0x07400000UL
#define TALKMAN_WIN_FILESZ	0x02BD3367UL

static void *lab_ident_map(phys_addr_t phys, size_t size, void *data)
{
	return ioremap_wc(phys, size);
}

static void lab_ident_unmap(void *virt, size_t size, void *data)
{
	if (virt)
		iounmap(virt);
}

static int lab_peek_set(const char *val, const struct kernel_param *kp)
{
	u64 phys = 0, n = 8;
	void __iomem *v;
	u64 i;
	char line[128];
	int pos;

	if (sscanf(val, "%lli %lli", &phys, &n) < 1)
		return -EINVAL;
	if (n > 256)
		n = 256;
	v = ioremap(phys & ~0xfffULL, ((phys & 0xfff) + n * 4 + 0xfff) & ~0xfffULL);
	if (!v) {
		pr_err("talkman lab: peek ioremap(%#llx) failed\n", phys);
		return -ENOMEM;
	}
	for (i = 0; i < n; i += 8) {
		u64 j;

		pos = scnprintf(line, sizeof(line), "talkman lab: peek %08llx:", phys + i * 4);
		for (j = i; j < n && j < i + 8; j++)
			pos += scnprintf(line + pos, sizeof(line) - pos, " %08x",
					 readl_relaxed(v + (phys & 0xfff) + j * 4));
		pr_info("%s\n", line);
	}
	iounmap(v);
	return 0;
}
static const struct kernel_param_ops lab_peek_ops = { .set = lab_peek_set };
module_param_cb(lab_peek, &lab_peek_ops, NULL, S_IWUSR);

/* QDSP6SS bring-up exactly as qcsubsys8992.sys FUN_00405c30 does it for 8992 */
static int lab_win_q6_reset(struct q6v5_data *drv)
{
	void __iomem *b = drv->reg_base;
	u32 v;

	writel_relaxed(0x20, b + 0x110);		/* STRAP_ACC */
	mb();
	writel_relaxed(1, b + 0x38);			/* XO_CBCR */
	mb();
	writel_relaxed(0x1700000, b + 0x30);		/* PWR_CTL: BHS_ON + clamps */
	mb();
	v = readl_relaxed(b + 0x30) | 0x2000000;	/* LDO_BYP */
	writel_relaxed(v, b + 0x30);
	mb();
	v = readl_relaxed(b + 0x30) & ~0x400000;	/* ~CLAMP_QMC_MEM */
	writel_relaxed(v, b + 0x30);
	mb();
	v = readl_relaxed(b + 0x30) | 0xfffff;		/* all memories on */
	writel_relaxed(v, b + 0x30);
	mb();
	v = readl_relaxed(b + 0x30) & ~0x200000;	/* ~CLAMP_WL */
	writel_relaxed(v, b + 0x30);
	mb();
	v = readl_relaxed(b + 0x30) & ~0x100000;	/* ~CLAMP_IO */
	writel_relaxed(v, b + 0x30);
	mb();
	writel_relaxed(4, b + 0x14);			/* RESET: BUS_ARES_ENA only */
	mb();
	writel_relaxed(0x102, b + 0x20);		/* GFMUX_CTL: CLK_ENA | SWITCH_CLK_OVR */
	mb();
	pr_info("talkman lab: windows q6 reset done, PWR_CTL %08x RESET %08x GFMUX %08x\n",
		readl_relaxed(b + 0x30), readl_relaxed(b + 0x14), readl_relaxed(b + 0x20));
	return 0;
}
/* lab: bus-state dump groups: 1=GCC cbcr, 2=AXI halt regs, 4=QDSP6SS regs */
static uint lab_bus_mask;
module_param(lab_bus_mask, uint, S_IRUGO | S_IWUSR);

#define LAB_PAS_INIT_IMAGE_CMD		1

/*
 * lab_scm: "<fnid> [a0 [a1 [a2 [a3]]]]" (hex or decimal) issues one armv8
 * SIP SMC with value args and logs ret[0..2]. "avail <lo> <hi>" asks
 * SCM_SVC_INFO/IS_CALL_AVAIL for every fnid in [lo, hi]. Read-only lab use.
 */
static int lab_scm_set(const char *val, const struct kernel_param *kp)
{
	char buf[128], *p, *tok;
	u64 a[5] = {0};
	int n = 0, ret;
	struct scm_desc desc = {0};

	strlcpy(buf, val, sizeof(buf));
	p = strim(buf);
	if (!strncmp(p, "avail", 5)) {
		u32 lo, hi, id;

		if (sscanf(p + 5, "%i %i", &lo, &hi) != 2)
			return -EINVAL;
		for (id = lo; id <= hi && id - lo < 256; id++) {
			pr_info("talkman lab: is_call_available(%#x) = %d\n", id,
				scm_is_call_available((id >> 8) & 0xff, id & 0xff));
		}
		return 0;
	}
	while ((tok = strsep(&p, " ")) && n < 5) {
		if (!*tok)
			continue;
		if (kstrtou64(tok, 0, &a[n]))
			return -EINVAL;
		n++;
	}
	if (!n)
		return -EINVAL;
	/* PIL AUTH_AND_RESET (5) / SHUTDOWN (6) — do not probe live. */
	if (((u32)a[0] & 0xff) == 5 || ((u32)a[0] & 0xff) == 6) {
		pr_err("talkman lab: scm %#llx refused (PAS auth/shutdown)\n",
		       a[0]);
		return -EPERM;
	}
	desc.args[0] = a[1];
	desc.args[1] = a[2];
	desc.args[2] = a[3];
	desc.args[3] = a[4];
	desc.arginfo = SCM_ARGS(n - 1);
	ret = scm_call2((u32)a[0], &desc);
	pr_info("talkman lab: scm %#llx(%llx %llx %llx %llx) rc=%d ret=%llx %llx %llx\n",
		a[0], a[1], a[2], a[3], a[4], ret,
		(u64)desc.ret[0], (u64)desc.ret[1], (u64)desc.ret[2]);
	return 0;
}
static const struct kernel_param_ops lab_scm_ops = {
	.set = lab_scm_set,
};
module_param_cb(lab_scm, &lab_scm_ops, NULL, S_IWUSR);
#define LAB_PAS_MEM_SETUP_CMD		2
#define LAB_PAS_IS_SUPPORTED_CMD	7

static int lab_scm_pas_mem_setup(struct device *dev, u32 proc, phys_addr_t addr,
				 size_t len)
{
	struct {
		u32 proc;
		u32 start_addr;
		u32 len;
	} request = { proc, (u32)addr, (u32)len };
	struct scm_desc desc = {0};
	u32 scm_ret = 0;
	int ret;

	if (!is_scm_armv8()) {
		ret = scm_call(SCM_SVC_PIL, LAB_PAS_MEM_SETUP_CMD, &request,
			       sizeof(request), &scm_ret, sizeof(scm_ret));
	} else {
		desc.args[0] = proc;
		desc.args[1] = addr;
		desc.args[2] = len;
		desc.arginfo = SCM_ARGS(3);
		ret = scm_call2(SCM_SIP_FNID(SCM_SVC_PIL, LAB_PAS_MEM_SETUP_CMD),
				&desc);
		scm_ret = desc.ret[0];
	}
	dev_info(dev, "talkman lab: PAS_MEM_SETUP(proc %u, %pa, 0x%zx) rc=%d ret=%u\n",
		 proc, &addr, len, ret, scm_ret);
	return ret ? ret : (int)scm_ret;
}

static void lab_scm_pas_probe(struct device *dev)
{
	int id;

	for (id = 0; id < 12; id++) {
		u32 request = id, ret_val = 0;
		struct scm_desc sdesc = {0};
		int rc;

		if (!is_scm_armv8()) {
			rc = scm_call(SCM_SVC_PIL, LAB_PAS_IS_SUPPORTED_CMD,
				      &request, sizeof(request), &ret_val,
				      sizeof(ret_val));
		} else {
			sdesc.args[0] = id;
			sdesc.arginfo = SCM_ARGS(1);
			rc = scm_call2(SCM_SIP_FNID(SCM_SVC_PIL,
						    LAB_PAS_IS_SUPPORTED_CMD),
				       &sdesc);
			ret_val = sdesc.ret[0];
		}
		dev_info(dev, "talkman lab: pas_supported(%d) rc=%d ret=%u (scm %s)\n",
			 id, rc, ret_val, is_scm_armv8() ? "armv8" : "legacy");
	}
}

#define LAB_PAS_UNLOCK_CMD		9

static int lab_scm_pas_unlock(struct device *dev, u32 proc)
{
	struct scm_desc desc = {0};
	int ret;

	desc.args[0] = proc;
	desc.arginfo = SCM_ARGS(1);
	ret = scm_call2(SCM_SIP_FNID(SCM_SVC_PIL, LAB_PAS_UNLOCK_CMD), &desc);
	dev_info(dev, "talkman lab: PAS_UNLOCK(proc %u) rc=%d ret=%llx\n",
		 proc, ret, (u64)desc.ret[0]);
	return ret;
}

#define LAB_PAS_INIT_IMAGE_CMD		1

static int lab_scm_pas_init_image(struct device *dev, u32 proc,
				  phys_addr_t mdata)
{
	struct scm_desc desc = {0};
	int ret;

	desc.args[0] = proc;
	desc.args[1] = mdata;
	desc.arginfo = SCM_ARGS(2, SCM_VAL, SCM_RW);
	ret = scm_call2(SCM_SIP_FNID(SCM_SVC_PIL, LAB_PAS_INIT_IMAGE_CMD),
			&desc);
	dev_info(dev, "talkman lab: PAS_INIT_IMAGE(proc %u, %pa) rc=%d ret=%llx\n",
		 proc, &mdata, ret, (u64)desc.ret[0]);
	return ret;
}

#define LAB_SHARE_FNID_CAF	0x0200020B
#define LAB_SHARE_FNID_WIN	0x02000202

static void lab_share_memory(struct device *dev)
{
	struct scm_desc desc = {0};
	int ret;
	u32 fnid = lab_share_fnid;
	unsigned int nargs;

	if (!lab_share_size)
		return;

	if (fnid == 1)
		fnid = LAB_SHARE_FNID_WIN;
	if (!fnid)
		fnid = LAB_SHARE_FNID_CAF;
	if ((fnid & 0xff) == 5 || (fnid & 0xff) == 6) {
		dev_err(dev, "talkman lab: SHARE fnid %#x refused (PAS auth/shutdown)\n",
			fnid);
		return;
	}

	if (fnid == LAB_SHARE_FNID_CAF) {
		/*
		 * CAF shared_memory.c: start, size, proc, READ|WRITE (3).
		 * Live #26: every 4-arg combo is -22; 3-arg on 0xB is -12.
		 */
		desc.args[0] = lab_share_phys;
		desc.args[1] = lab_share_size;
		desc.args[2] = lab_share_flags;
		desc.args[3] = 3;
		desc.arginfo = SCM_ARGS(4);
		nargs = 4;
	} else {
		/* A5 FUN_0040a310: SCM 0x02000202, 3 words {phys, 0, size}. */
		desc.args[0] = lab_share_phys;
		desc.args[1] = 0;
		desc.args[2] = lab_share_size;
		desc.arginfo = SCM_ARGS(3);
		nargs = 3;
	}
	ret = scm_call2(fnid, &desc);
	dev_info(dev,
		 "talkman lab: SHARE fnid=%#x args=%llx %llx %llx %llx n=%u rc=%d ret=%llx\n",
		 fnid, (u64)desc.args[0], (u64)desc.args[1],
		 (u64)desc.args[2], (u64)desc.args[3], nargs, ret,
		 (u64)desc.ret[0]);
}

static void lab_tz_before_boot(struct pil_desc *pil, phys_addr_t mba_phys)
{
	phys_addr_t rs = pil_get_region_start(pil);
	phys_addr_t re = pil_get_region_end(pil);
	u32 ids[] = { lab_tz_pas_id };
	int i, n = 1;
	u32 all[] = { 0, 1, 2, 3, 4, 5, 6, 7 };

	if (lab_unlock) {
		lab_scm_pas_unlock(pil->dev, lab_tz_pas_id);
		if (lab_tz_pas_id)
			lab_scm_pas_unlock(pil->dev, 0);
	}

	/*
	 * SHARE waits until PT_LOAD is in place (see pil_msa_auth_modem_mdt).
	 * Measured on #26: only PAS_MEM_SETUP(proc 9) accepts those
	 * windows. proc 9 is PAS_VIDC/Venus on msm8992, not PAS_MODEM
	 * (unsupported on this Windows TZ). SHARE 4-arg was -22; m24
	 * retries the Windows 3-word layout after PIL cmd 9 unlock.
	 */
	if (lab_tz_pas_id) {
		if (lab_mirror_size)
			lab_scm_pas_mem_setup(pil->dev, lab_tz_pas_id,
					      lab_mirror_phys, lab_mirror_size);
		if (lab_mdata_stage)
			lab_scm_pas_mem_setup(pil->dev, lab_tz_pas_id,
					      lab_mdata_stage, SZ_1M);
		if (lab_mba_stage)
			lab_scm_pas_mem_setup(pil->dev, lab_tz_pas_id,
					      lab_mba_stage, SZ_1M);
	}

	if (!lab_tz_mask)
		return;
	if (lab_tz_mask & BIT(7))
		lab_scm_pas_probe(pil->dev);
	for (i = 0; i < ((lab_tz_mask & BIT(3)) ? ARRAY_SIZE(all) : n); i++) {
		u32 proc = (lab_tz_mask & BIT(3)) ? all[i] : ids[i];

		if ((lab_tz_mask & BIT(0)) && re > rs)
			lab_scm_pas_mem_setup(pil->dev, proc, rs, re - rs);
		if (lab_tz_mask & BIT(1))
			lab_scm_pas_mem_setup(pil->dev, proc, mba_phys, SZ_1M);
		if (lab_tz_mask & BIT(2))
			lab_scm_pas_mem_setup(pil->dev, proc, 0x06F00000,
					      0x180000);
	}
}

static u32 lab_last_image = ~0u;
static u32 lab_last_status = ~0u;
static u32 lab_last_debug = ~0u;

/*
 * pr_emerg so STATUS/DEBUG/IMAGE/hash/after-META hit console + ramoops
 * even when logd is dead. Keep the last line and re-print it every 1s
 * for ~10s so a late adb dmesg can still see after-META.
 */
static void lab_emerg_replay(struct work_struct *work);

static char lab_emerg_last[320];
static unsigned int lab_emerg_left;
static DECLARE_DELAYED_WORK(lab_emerg_work, lab_emerg_replay);

static void lab_emerg_replay(struct work_struct *work)
{
	(void)work;
	if (!lab_emerg_last[0] || !lab_emerg_left)
		return;
	pr_emerg("%s", lab_emerg_last);
	lab_emerg_left--;
	if (lab_emerg_left)
		schedule_delayed_work(&lab_emerg_work, HZ);
}

static void talkman_mba_emerg(const char *fmt, ...)
{
	va_list ap;
	int n;

	va_start(ap, fmt);
	n = vscnprintf(lab_emerg_last, sizeof(lab_emerg_last), fmt, ap);
	va_end(ap);
	if (n > 0 && lab_emerg_last[n - 1] != '\n' &&
	    n + 1 < (int)sizeof(lab_emerg_last)) {
		lab_emerg_last[n] = '\n';
		lab_emerg_last[n + 1] = '\0';
	}
	pr_emerg("%s", lab_emerg_last);
	lab_emerg_left = 10;
	mod_delayed_work(system_wq, &lab_emerg_work, HZ);
}

static void lab_log_rmb(struct device *dev, void __iomem *base,
			const char *when)
{
	u32 image = readl_relaxed(base + RMB_MBA_IMAGE);
	u32 pbl = readl_relaxed(base + RMB_PBL_STATUS);
	u32 cmd = readl_relaxed(base + RMB_MBA_COMMAND);
	u32 mba = readl_relaxed(base + RMB_MBA_STATUS);
	u32 meta = readl_relaxed(base + RMB_PMI_META_DATA);
	u32 start = readl_relaxed(base + RMB_PMI_CODE_START);
	u32 len = readl_relaxed(base + RMB_PMI_CODE_LENGTH);
	u32 ver = readl_relaxed(base + RMB_PROTOCOL_VERSION);
	u32 dbg = readl_relaxed(base + RMB_MBA_DEBUG_INFORMATION);

	talkman_mba_emerg("talkman-mba RMB %s: IMAGE=%08x STATUS=%08x DEBUG=%08x pbl=%08x cmd=%08x meta=%08x start=%08x len=%08x ver=%08x\n",
	       when, image, mba, dbg, pbl, cmd, meta, start, len, ver);
	if (image != lab_last_image) {
		talkman_mba_emerg("talkman-mba IMAGE change %08x -> %08x (%s)\n",
		       lab_last_image, image, when);
		lab_last_image = image;
	}
	if (mba != lab_last_status) {
		talkman_mba_emerg("talkman-mba STATUS change %08x -> %08x (%s)\n",
		       lab_last_status, mba, when);
		lab_last_status = mba;
	}
	if (dbg != lab_last_debug) {
		talkman_mba_emerg("talkman-mba DEBUG change %08x -> %08x (%s)\n",
		       lab_last_debug, dbg, when);
		lab_last_debug = dbg;
	}
	(void)dev;
}

static void modem_log_rmb_regs(void __iomem *base)
{
	talkman_mba_emerg("talkman-mba RMB_MBA_IMAGE: %08x\n",
	       readl_relaxed(base + RMB_MBA_IMAGE));
	pr_err("talkman-mba RMB_PBL_STATUS: %08x\n",
	       readl_relaxed(base + RMB_PBL_STATUS));
	pr_err("talkman-mba RMB_MBA_COMMAND: %08x\n",
	       readl_relaxed(base + RMB_MBA_COMMAND));
	talkman_mba_emerg("talkman-mba RMB_MBA_STATUS: %08x\n",
	       readl_relaxed(base + RMB_MBA_STATUS));
	pr_err("talkman-mba RMB_PMI_META_DATA: %08x\n",
	       readl_relaxed(base + RMB_PMI_META_DATA));
	pr_err("talkman-mba RMB_PMI_CODE_START: %08x\n",
	       readl_relaxed(base + RMB_PMI_CODE_START));
	pr_err("talkman-mba RMB_PMI_CODE_LENGTH: %08x\n",
	       readl_relaxed(base + RMB_PMI_CODE_LENGTH));
	pr_err("talkman-mba RMB_PROTOCOL_VERSION: %08x\n",
	       readl_relaxed(base + RMB_PROTOCOL_VERSION));
	talkman_mba_emerg("talkman-mba RMB_MBA_DEBUG_INFORMATION: %08x\n",
	       readl_relaxed(base + RMB_MBA_DEBUG_INFORMATION));

	if (modem_trigger_panic == MSS_MAGIC)
		panic("%s: System ramdump is needed!!!\n", __func__);
}

static int pil_mss_power_up(struct q6v5_data *drv)
{
	int ret = 0;
	u32 regval;

	if (drv->vreg) {
		ret = regulator_enable(drv->vreg);
		if (ret)
			dev_err(drv->desc.dev, "Failed to enable modem regulator.\n");
	}

	if (drv->cxrail_bhs) {
		regval = readl_relaxed(drv->cxrail_bhs);
		regval |= EXTERNAL_BHS_ON;
		writel_relaxed(regval, drv->cxrail_bhs);

		ret = readl_poll_timeout(drv->cxrail_bhs, regval,
			regval & EXTERNAL_BHS_STATUS, 1, BHS_TIMEOUT_US);
	}

	return ret;
}

static int pil_mss_power_down(struct q6v5_data *drv)
{
	u32 regval;

	if (drv->cxrail_bhs) {
		regval = readl_relaxed(drv->cxrail_bhs);
		regval &= ~EXTERNAL_BHS_ON;
		writel_relaxed(regval, drv->cxrail_bhs);
	}

	if (drv->vreg)
		return regulator_disable(drv->vreg);

	return 0;
}

static int pil_mss_enable_clks(struct q6v5_data *drv)
{
	int ret;

	ret = clk_prepare_enable(drv->ahb_clk);
	if (ret)
		goto err_ahb_clk;
	ret = clk_prepare_enable(drv->axi_clk);
	if (ret)
		goto err_axi_clk;
	ret = clk_prepare_enable(drv->rom_clk);
	if (ret)
		goto err_rom_clk;
	ret = clk_prepare_enable(drv->gpll0_mss_clk);
	if (ret)
		goto err_gpll0_mss_clk;

	return 0;

err_gpll0_mss_clk:
	clk_disable_unprepare(drv->rom_clk);
err_rom_clk:
	clk_disable_unprepare(drv->axi_clk);
err_axi_clk:
	clk_disable_unprepare(drv->ahb_clk);
err_ahb_clk:
	return ret;
}

static void pil_mss_disable_clks(struct q6v5_data *drv)
{
	clk_disable_unprepare(drv->gpll0_mss_clk);
	clk_disable_unprepare(drv->rom_clk);
	clk_disable_unprepare(drv->axi_clk);
	if (!drv->ahb_clk_vote)
		clk_disable_unprepare(drv->ahb_clk);
}


/* lab: snapshot of the MSS bus path state */
static void lab_dump_bus(struct q6v5_data *drv, const char *when)
{
	void __iomem *gcc;
	u32 cbcr = 0, rom = 0, bimc_gfx = 0;
	u32 hq[3] = {0}, hm[3] = {0}, hn[3] = {0};
	int i;

	if (!lab_bus_mask)
		return;
	gcc = ioremap(0xfc400000, 0x2000);
	if (gcc && (lab_bus_mask & 1)) {
		cbcr = readl_relaxed(gcc + 0x0284);	/* MSS_Q6_BIMC_AXI_CBCR */
		rom = readl_relaxed(gcc + 0x1c00);	/* BOOT_ROM_AHB_CBCR (8994) */
		bimc_gfx = readl_relaxed(gcc + 0x0400);	/* MMSS_BIMC_GFX_CBCR */
	}
	if (gcc)
		iounmap(gcc);
	if (drv->axi_halt_base && (lab_bus_mask & 2)) {
		for (i = 0; i < 3; i++) {
			hq[i] = readl_relaxed(drv->axi_halt_base + MSS_Q6_HALT_BASE + 4 * i);
			hm[i] = readl_relaxed(drv->axi_halt_base + MSS_MODEM_HALT_BASE + 4 * i);
			hn[i] = readl_relaxed(drv->axi_halt_base + MSS_NC_HALT_BASE + 4 * i);
		}
	}
	dev_info(drv->desc.dev,
		 "talkman lab bus %s: MSS_RESTART=%08x Q6_BIMC_AXI_CBCR=%08x (clk_off=%u en=%u) rom_cbcr=%08x bimc_gfx_cbcr=%08x\n",
		 when, drv->restart_reg ? readl_relaxed(drv->restart_reg) : 0xdead,
		 cbcr, (cbcr >> 31) & 1, cbcr & 1, rom, bimc_gfx);
	dev_info(drv->desc.dev,
		 "talkman lab halt %s: q6 req/ack/idle=%x/%x/%x modem=%x/%x/%x nc=%x/%x/%x\n",
		 when, hq[0], hq[1], hq[2], hm[0], hm[1], hm[2], hn[0], hn[1], hn[2]);
	if (!(lab_bus_mask & 4))
		return;
	dev_info(drv->desc.dev,
		 "talkman lab q6ss %s: RESET=%08x GFMUX=%08x PWR_CTL=%08x XO_CBCR=%08x STRAP_ACC=%08x SLEEP_CBCR=%08x RST_EVB=%08x\n",
		 when, readl_relaxed(drv->reg_base + 0x014), readl_relaxed(drv->reg_base + 0x020),
		 readl_relaxed(drv->reg_base + 0x030), readl_relaxed(drv->reg_base + 0x038),
		 readl_relaxed(drv->reg_base + 0x110), readl_relaxed(drv->reg_base + 0x03c),
		 readl_relaxed(drv->reg_base + 0x010));
}

static int pil_mss_restart_reg(struct q6v5_data *drv, u32 mss_restart)
{
	int ret = 0;
	int scm_ret = 0;
	struct scm_desc desc = {0};

	desc.args[0] = mss_restart;
	desc.args[1] = 0;
	desc.arginfo = SCM_ARGS(2);

	if (drv->restart_reg && !drv->restart_reg_sec) {
		writel_relaxed(mss_restart, drv->restart_reg);
		mb();
		udelay(2);
	} else if (drv->restart_reg_sec) {
		if (!is_scm_armv8()) {
			ret = scm_call(SCM_SVC_PIL, MSS_RESTART_ID,
					&mss_restart, sizeof(mss_restart),
					&scm_ret, sizeof(scm_ret));
		} else {
			ret = scm_call2(SCM_SIP_FNID(SCM_SVC_PIL,
						MSS_RESTART_ID), &desc);
			scm_ret = desc.ret[0];
		}
		if (ret || scm_ret)
			pr_err("Secure MSS restart failed\n");
	}

	return ret;
}

static int pil_msa_wait_for_mba_ready(struct q6v5_data *drv)
{
	struct device *dev = drv->desc.dev;
	int ret;
	u32 status;

	/* Wait for PBL completion. */
	{
		ktime_t t0 = ktime_get();

		ret = readl_poll_timeout(drv->rmb_base + RMB_PBL_STATUS, status,
			status != 0, POLL_INTERVAL_US,
			pbl_mba_boot_timeout_ms * 1000);
		pr_err("talkman-mba PBL STATUS=%08x after %lld us (rc %d)\n",
		       status, ktime_us_delta(ktime_get(), t0), ret);
		lab_log_rmb(dev, drv->rmb_base, "after PBL poll");
	}
	if (ret) {
		dev_err(dev, "PBL boot timed out\n");
		return ret;
	}
	if (status != STATUS_PBL_SUCCESS) {
		dev_err(dev, "PBL returned unexpected status %d\n", status);
		return -EINVAL;
	}

	/* Wait for MBA completion. */
	ret = readl_poll_timeout(drv->rmb_base + RMB_MBA_STATUS, status,
		status != 0, POLL_INTERVAL_US, pbl_mba_boot_timeout_ms * 1000);
	if (ret) {
		dev_err(dev, "MBA boot timed out\n");
		return ret;
	}
	if (status != STATUS_XPU_UNLOCKED &&
	    status != STATUS_XPU_UNLOCKED_SCRIBBLED) {
		talkman_mba_emerg("talkman-mba MBA unexpected STATUS=%08x DEBUG=%08x\n",
		       status,
		       readl_relaxed(drv->rmb_base + RMB_MBA_DEBUG_INFORMATION));
		dev_err(dev, "MBA returned unexpected status %d\n", status);
		lab_log_rmb(dev, drv->rmb_base, "mba unexpected");
		return -EINVAL;
	}
	talkman_mba_emerg("talkman-mba MBA ready STATUS=%08x DEBUG=%08x\n",
	       status,
	       readl_relaxed(drv->rmb_base + RMB_MBA_DEBUG_INFORMATION));
	lab_log_rmb(dev, drv->rmb_base, "mba ready");

	return 0;
}

int pil_mss_shutdown(struct pil_desc *pil)
{
	struct q6v5_data *drv = container_of(pil, struct q6v5_data, desc);
	int ret = 0;

	if (drv->axi_halt_base) {
		pil_q6v5_halt_axi_port(pil,
			drv->axi_halt_base + MSS_Q6_HALT_BASE);
		pil_q6v5_halt_axi_port(pil,
			drv->axi_halt_base + MSS_MODEM_HALT_BASE);
		pil_q6v5_halt_axi_port(pil,
			drv->axi_halt_base + MSS_NC_HALT_BASE);
	}

	if (drv->axi_halt_q6)
		pil_q6v5_halt_axi_port(pil, drv->axi_halt_q6);
	if (drv->axi_halt_mss)
		pil_q6v5_halt_axi_port(pil, drv->axi_halt_mss);
	if (drv->axi_halt_nc)
		pil_q6v5_halt_axi_port(pil, drv->axi_halt_nc);

	ret = pil_mss_restart_reg(drv, 1);

	if (drv->is_booted) {
		pil_mss_disable_clks(drv);
		pil_mss_power_down(drv);
		drv->is_booted = false;
	}

	return ret;
}

int __pil_mss_deinit_image(struct pil_desc *pil, bool err_path)
{
	struct modem_data *drv = dev_get_drvdata(pil->dev);
	struct q6v5_data *q6_drv = container_of(pil, struct q6v5_data, desc);
	int ret = 0;
	s32 status;

	if (err_path) {
		writel_relaxed(CMD_PILFAIL_NFY_MBA,
				drv->rmb_base + RMB_MBA_COMMAND);
		ret = readl_poll_timeout(drv->rmb_base + RMB_MBA_STATUS, status,
			status == STATUS_MBA_UNLOCKED || status < 0,
			POLL_INTERVAL_US, pbl_mba_boot_timeout_ms * 1000);
		if (ret)
			dev_err(pil->dev, "MBA region unlock timed out\n");
		else if (status < 0)
			dev_err(pil->dev, "MBA unlock returned err status: %d\n",
						status);
	}

	ret = pil_mss_shutdown(pil);

	if (q6_drv->ahb_clk_vote)
		clk_disable_unprepare(q6_drv->ahb_clk);

	/* In case of any failure where reclaim MBA memory
	 * could not happen, free the memory here */
	if (drv->q6->mba_virt) {
		pil_mss_mba_free(drv, drv->q6->mba_size, drv->q6->mba_virt,
				 drv->q6->mba_phys, &drv->attrs_dma);
		drv->q6->mba_virt = NULL;
	}
	return ret;
}

int pil_mss_deinit_image(struct pil_desc *pil)
{
	return __pil_mss_deinit_image(pil, true);
}

int pil_mss_make_proxy_votes(struct pil_desc *pil)
{
	int ret;
	struct q6v5_data *drv = container_of(pil, struct q6v5_data, desc);
	int uv = 0;

	ret = of_property_read_u32(pil->dev->of_node, "vdd_mx-uV", &uv);
	if (ret) {
		dev_err(pil->dev, "missing vdd_mx-uV property\n");
		return ret;
	}

	ret = regulator_set_voltage(drv->vreg_mx, uv, INT_MAX);
	if (ret) {
		dev_err(pil->dev, "Failed to request vreg_mx voltage\n");
		return ret;
	}

	ret = regulator_enable(drv->vreg_mx);
	if (ret) {
		dev_err(pil->dev, "Failed to enable vreg_mx\n");
		regulator_set_voltage(drv->vreg_mx, 0, INT_MAX);
		return ret;
	}

	ret = pil_q6v5_make_proxy_votes(pil);
	if (ret) {
		regulator_disable(drv->vreg_mx);
		regulator_set_voltage(drv->vreg_mx, 0, INT_MAX);
	}

	return ret;
}

void pil_mss_remove_proxy_votes(struct pil_desc *pil)
{
	struct q6v5_data *drv = container_of(pil, struct q6v5_data, desc);
	pil_q6v5_remove_proxy_votes(pil);
	regulator_disable(drv->vreg_mx);
	regulator_set_voltage(drv->vreg_mx, 0, INT_MAX);
}

static int pil_mss_reset(struct pil_desc *pil)
{
	struct q6v5_data *drv = container_of(pil, struct q6v5_data, desc);
	phys_addr_t start_addr = pil_get_entry_addr(pil);
	int ret;

	if (drv->mba_phys)
		start_addr = drv->mba_phys;

	/*
	 * Bring subsystem out of reset and enable required
	 * regulators and clocks.
	 */
	ret = pil_mss_power_up(drv);
	if (ret)
		goto err_power;

	/* talkman lab: full MSS reset cycle so the RMB block cannot be stale */
	if (lab_pre_reset) {
		pil_mss_restart_reg(drv, 1);
		udelay(100);
	}

	/* Deassert reset to subsystem and wait for propagation */
	ret = pil_mss_restart_reg(drv, 0);
	if (ret)
		goto err_restart;

	ret = pil_mss_enable_clks(drv);
	if (ret)
		goto err_clks;

	if (drv->self_auth)
		lab_log_rmb(pil->dev, drv->rmb_base, "before image write");

	/* Program Image Address */
	if (drv->self_auth) {
		writel_relaxed(lab_rmb_image ? (lab_rmb_image == 0xffffffff ? 0 : lab_rmb_image) : (u32)start_addr,
			       drv->rmb_base + RMB_MBA_IMAGE);
		/*
		 * Ensure write to RMB base occurs before reset
		 * is released.
		 */
		mb();
		lab_log_rmb(pil->dev, drv->rmb_base, "after image write");
	} else {
		writel_relaxed((start_addr >> 4) & 0x0FFFFFF0,
				drv->reg_base + QDSP6SS_RST_EVB);
	}

	lab_dump_bus(drv, "before q6 release");
	ret = lab_win_reset ? lab_win_q6_reset(drv) : pil_q6v5_reset(pil);
	if (ret)
		goto err_q6v5_reset;

	/* Wait for MBA to start. Check for PBL and MBA errors while waiting. */
	if (drv->self_auth) {
		ret = pil_msa_wait_for_mba_ready(drv);
		if (ret)
			goto err_q6v5_reset;
	}

	talkman_mba_emerg("talkman-mba MBA boot done IMAGE=%08x STATUS=%08x DEBUG=%08x\n",
	       readl_relaxed(drv->rmb_base + RMB_MBA_IMAGE),
	       readl_relaxed(drv->rmb_base + RMB_MBA_STATUS),
	       readl_relaxed(drv->rmb_base + RMB_MBA_DEBUG_INFORMATION));
	lab_log_rmb(pil->dev, drv->rmb_base, "mba boot done");
	drv->is_booted = true;

	return 0;

err_q6v5_reset:
	modem_log_rmb_regs(drv->rmb_base);
	lab_dump_bus(drv, "after pbl");
	if (lab_hold) {
		dev_info(pil->dev, "talkman lab: lab_hold set, leaving MSS powered\n");
		return ret;
	}
	pil_mss_disable_clks(drv);
	if (drv->ahb_clk_vote)
		clk_disable_unprepare(drv->ahb_clk);
err_clks:
	pil_mss_restart_reg(drv, 1);
err_restart:
	pil_mss_power_down(drv);
err_power:
	return ret;
}

/*
 * MBA image / metadata buffers.
 *
 * Default: CMA through mba_mem_dev, as stock CAF does.
 *
 * With a fixed MBA region (DT qcom,mba-mem, memory the kernel does not own)
 * the MBA image is copied to the region base. On the Lumia 950 (MSM8992,
 * TZ set up by the Windows boot chain) the modem PBL only fetches its
 * authenticator from the 1 MB at 0x06F00000 that Windows' qcsubsys8992
 * driver uses; any other address is answered with RMB_PBL_STATUS
 * 0xEF1D0200. The rest of that megabyte is MBA scratch, so the modem
 * metadata goes to the top of the modem region instead: MSS-readable,
 * HLOS-writable, and only overwritten by segment loading, which starts
 * after the metadata has been authenticated and released.
 */
void *pil_mss_mba_alloc(struct modem_data *md, size_t size, bool mdata,
			dma_addr_t *phys, struct dma_attrs *attrs)
{
	phys_addr_t base = 0, mba_lo, mba_hi, end;
	void *virt;

	if (mdata ? lab_mdata_stage : lab_mba_stage) {
		base = mdata ? lab_mdata_stage : lab_mba_stage;
		/*
		 * 0x0CA ioremap_wc of CMA RAM hung m28. Metadata at a
		 * pfn_valid address must use the linear map too.
		 */
		if (mdata && pfn_valid(__phys_to_pfn(base))) {
			*phys = base;
			pr_err("talkman-mba metadata linear %pa+%zx (no ioremap)\n",
			       &base, size);
			return phys_to_virt(base);
		}
		/* arm64 ioremap_wc = PROT_NORMAL_NC (MmNonCached). */
		virt = ioremap_wc(base, size);
		if (virt) {
			*phys = base;
			if (mdata)
				pr_err("talkman-mba metadata map %#llx+%zx (NORMAL_NC)\n",
				       (u64)base, size);
			return virt;
		}
		pr_err("talkman lab: ioremap %s %#llx+%zx failed%s\n",
		       mdata ? "mdata" : "mba", (u64)base, size,
		       mdata ? " (no dma)" : ", dma fallback");
		goto dma;
	}

	if (!md->mba_region_size)
		goto dma;

	mba_lo = lab_mba_stage ? lab_mba_stage : md->mba_region_phys;
	mba_hi = mba_lo + (md->mba_region_size ? md->mba_region_size : SZ_1M);

	if (mdata) {
		end = pil_get_region_end(&md->desc);
		if (!end || size > SZ_1M) {
			pr_err("talkman lab: mdata no region_end=%pa size=%zx, dma fallback\n",
			       &end, size);
			goto dma;
		}
		base = (end - size) & ~(phys_addr_t)(SZ_4K - 1);
		if (mba_lo && base < mba_hi && (base + size) > mba_lo) {
			pr_err("talkman lab: mdata %#llx overlaps MBA %#llx-%#llx, dma fallback\n",
			       (u64)base, (u64)mba_lo, (u64)mba_hi);
			goto dma;
		}
	} else {
		if (size > md->mba_region_size) {
			pr_err("talkman lab: MBA size %zx > region %zx\n",
			       size, (size_t)md->mba_region_size);
			return NULL;
		}
		base = md->mba_region_phys;
	}
	if (mdata && pfn_valid(__phys_to_pfn(base))) {
		*phys = base;
		pr_err("talkman-mba metadata linear %pa+%zx (no ioremap)\n",
		       &base, size);
		return phys_to_virt(base);
	}
	virt = ioremap_wc(base, size);
	if (virt) {
		*phys = base;
		if (mdata)
			pr_err("talkman-mba metadata map %#llx+%zx (NORMAL_NC)\n",
			       (u64)base, size);
		return virt;
	}
	pr_err("talkman lab: ioremap %#llx+%zx failed%s\n",
	       (u64)base, size, mdata ? " (no dma)" : ", dma fallback");
dma:
	if (mdata) {
		pr_err("talkman lab: metadata dma fallback refused\n");
		return NULL;
	}
	return dma_alloc_attrs(&md->mba_mem_dev, size, phys, GFP_KERNEL, attrs);
}

void pil_mss_mba_free(struct modem_data *md, size_t size, void *virt,
		      dma_addr_t phys, struct dma_attrs *attrs)
{
	if (!virt)
		return;
	if (!md->mba_region_size && !lab_mba_stage && !lab_mdata_stage) {
		dma_free_attrs(&md->mba_mem_dev, size, virt, phys, attrs);
		return;
	}
	if (pfn_valid(__phys_to_pfn(phys)) && virt == phys_to_virt(phys))
		return;
	iounmap(virt);
}

int pil_mss_reset_load_mba(struct pil_desc *pil)
{
	struct q6v5_data *drv = container_of(pil, struct q6v5_data, desc);
	struct modem_data *md = dev_get_drvdata(pil->dev);
	const struct firmware *fw;
	char fw_name_legacy[10] = "mba.b00";
	char fw_name[10] = "mba.mbn";
	char *fw_name_p;
	void *mba_virt;
	dma_addr_t mba_phys, mba_phys_end;
	int ret, count;
	const u8 *data;

	fw_name_p = drv->non_elf_image ? fw_name_legacy : fw_name;
	/* Load and authenticate mba image */
	ret = request_firmware(&fw, fw_name_p, pil->dev);
	if (ret) {
		dev_err(pil->dev, "Failed to locate %s\n",
						fw_name_p);
		return ret;
	}

	drv->mba_size = SZ_1M;
	md->mba_mem_dev.coherent_dma_mask =
		DMA_BIT_MASK(sizeof(dma_addr_t) * 8);
	init_dma_attrs(&md->attrs_dma);
	dma_set_attr(DMA_ATTR_STRONGLY_ORDERED, &md->attrs_dma);
	mba_virt = pil_mss_mba_alloc(md, drv->mba_size, false, &mba_phys,
				     &md->attrs_dma);
	if (!mba_virt) {
		dev_err(pil->dev, "MBA metadata buffer allocation failed\n");
		ret = -ENOMEM;
		goto err_dma_alloc;
	}

	drv->mba_phys = mba_phys;
	drv->mba_virt = mba_virt;
	mba_phys_end = mba_phys + drv->mba_size;

	dev_info(pil->dev, "MBA: loading from %pa to %pa\n", &mba_phys,
								&mba_phys_end);
	/* Load the MBA image into memory */
	data = fw ? fw->data : NULL;
	if (!data) {
		dev_err(pil->dev, "MBA data is NULL\n");
		ret = -ENOMEM;
		goto err_mss_reset;
	}
	count = fw->size;
	if (lab_mba_unwrap && count > sizeof(struct elf32_hdr) &&
	    !memcmp(data, ELFMAG, SELFMAG) && data[EI_CLASS] == ELFCLASS32) {
		const struct elf32_hdr *eh = (const struct elf32_hdr *)data;
		const struct elf32_phdr *ph;
		int i;

		if (eh->e_phoff + (size_t)eh->e_phnum * sizeof(*ph) <= fw->size) {
			ph = (const struct elf32_phdr *)(data + eh->e_phoff);
			for (i = 0; i < eh->e_phnum; i++, ph++) {
				if (ph->p_type != PT_LOAD || !ph->p_filesz)
					continue;
				if (ph->p_offset + ph->p_filesz > fw->size)
					break;
				dev_info(pil->dev,
					 "MBA: ELF wrapper, loading segment %d (file %#x, %#x bytes, paddr %#x) as flat MBA\n",
					 i, ph->p_offset, ph->p_filesz, ph->p_paddr);
				data += ph->p_offset;
				count = ph->p_filesz;
				break;
			}
		}
	}
	if (count > drv->mba_size) {
		dev_err(pil->dev, "MBA too large (%d)\n", count);
		ret = -EINVAL;
		goto err_mss_reset;
	}
	memcpy(mba_virt, data, count);
	wmb();
	{
		/* talkman lab: did the staging memory take the image? */
		size_t i;
		int bad = 0;

		for (i = 0; i < count; i += SZ_4K)
			if (memcmp(mba_virt + i, data + i,
				   min_t(size_t, SZ_4K, count - i)))
				bad++;
		dev_info(pil->dev,
			 "MBA staging readback: %d bad pages of %zu, head %08x %08x %08x %08x\n",
			 bad, (size_t)DIV_ROUND_UP(count, SZ_4K),
			 readl_relaxed(mba_virt), readl_relaxed(mba_virt + 4),
			 readl_relaxed(mba_virt + 8), readl_relaxed(mba_virt + 12));
	}

	lab_tz_before_boot(pil, mba_phys);

	ret = pil_mss_reset(pil);
	if (ret) {
		dev_err(pil->dev, "MBA boot failed.\n");
		goto err_mss_reset;
	}

	release_firmware(fw);

	return 0;

err_mss_reset:
	pil_mss_mba_free(md, drv->mba_size, drv->mba_virt, drv->mba_phys,
			 &md->attrs_dma);
	drv->mba_virt = NULL;
err_dma_alloc:
	release_firmware(fw);
	return ret;
}


/* Windows qcpil8994 FUN_004065a8/FUN_004066fc: pack Ehdr+Phdrs+hash.
 * FUN_004066fc then sets the *working* hash p_paddr to packed+0x334;
 * the copy FUN_004065a8 already wrote still has ELF p_paddr 0x0CA00000.
 * Hash bytes also stay at 0x0CA (in-hash ptrs 0x0CA00028/328/428).
 */
#define PIL_HASH_PFLAG	(0x2 << 24)

static const struct elf32_phdr *lab_hash_phdr(const u8 *elf, size_t sz,
					      int *idx)
{
	const struct elf32_hdr *eh;
	const struct elf32_phdr *ph;
	size_t i;

	if (idx)
		*idx = -1;
	if (!elf || sz < sizeof(*eh) || memcmp(elf, ELFMAG, SELFMAG) ||
	    elf[EI_CLASS] != ELFCLASS32)
		return NULL;
	eh = (const struct elf32_hdr *)elf;
	if (!eh->e_phnum || eh->e_phentsize != sizeof(*ph) ||
	    eh->e_phoff + (size_t)eh->e_phnum * sizeof(*ph) > sz)
		return NULL;
	ph = (const struct elf32_phdr *)(elf + eh->e_phoff);
	for (i = 0; i < eh->e_phnum; i++, ph++) {
		if ((ph->p_type == PT_NULL || ph->p_type == PT_LOAD) &&
		    (ph->p_flags & (0x7 << 24)) == PIL_HASH_PFLAG &&
		    ph->p_filesz) {
			if (idx)
				*idx = (int)i;
			return ph;
		}
	}
	return NULL;
}

static u32 lab_hash_paddr(const u8 *elf, size_t sz)
{
	const struct elf32_phdr *ph = lab_hash_phdr(elf, sz, NULL);

	return ph ? ph->p_paddr : 0;
}

static const char *lab_mba_dbg_name(u32 dbg)
{
	switch (dbg) {
	case 4:
		return "MBA_DEBUG_POLICY_META_AUTH_FAILURE";
	case 7:
		return "MBA_META_DATA_AUTH_TLB_FAILURE";
	case 8:
		return "MBA_META_DATA_AUTH_FAILURE";
	case 9:
		return "MBA_INIT_CODE_START_TLB_FAILURE";
	case 10:
		return "MBA_CODE_RANGE_INVALID";
	case 11:
		return "MBA_CODE_SEGMENT_AUTH_IMAGE_LEN_FAILURE";
	case 12:
		return "MBA_CODE_AUTH_RESET_FAILURE";
	case 13:
		return "MBA_CODE_DECRYPT_VERIFY_FAILURE";
	default:
		return "see mba.mbn";
	}
}

static void lab_ring_load_ready(struct pil_desc *pil, phys_addr_t code,
				u32 code_len, const char *why)
{
	struct modem_data *drv = dev_get_drvdata(pil->dev);
	phys_addr_t hive = pil_get_region_start(pil);

	writel_relaxed((u32)code, drv->rmb_base + RMB_PMI_CODE_START);
	writel_relaxed(code_len, drv->rmb_base + RMB_PMI_CODE_LENGTH);
	wmb();
	writel_relaxed(CMD_LOAD_READY, drv->rmb_base + RMB_MBA_COMMAND);
	pr_err("talkman-mba CMD_LOAD_READY %s start %pa (hive %pa) len %#x\n",
	       why, &code, &hive, code_len);
	lab_log_rmb(pil->dev, drv->rmb_base, why);
}

/*
 * AUTH window after META==3. Default hive/span (A4). lab_auth_filesz=1
 * uses PT_LOAD filesz (Windows 0x02BD3367) as CODE_LENGTH instead.
 * Never CODE_START in the 0x070 memory_hole (m11).
 */
static int lab_pick_auth_window(struct pil_desc *pil, phys_addr_t *code_out,
				u32 *len_out, u32 *span_out, u32 *filesz_out)
{
	phys_addr_t hive = 0, code = 0;
	size_t span = 0;
	u32 filesz, len;
	int rc;

	rc = pil_hive_code_window(pil, &hive, &span);
	if (rc) {
		hive = pil_get_region_start(pil);
		span = pil_segs_span(pil, &code);
		if (!hive)
			hive = code;
	}
	filesz = (u32)pil_segs_filesz(pil);
	code = lab_code_start ? (phys_addr_t)lab_code_start : hive;

	if (code >= TALKMAN_ELF_HOLE && code < TALKMAN_HIVE_BASE) {
		dev_err(pil->dev,
			"talkman lab: refuse CODE_START %pa (0x070 hole), using hive %pa\n",
			&code, &hive);
		code = hive;
	}

	if (lab_auth_filesz == 1)
		len = filesz;
	else if (lab_code_span)
		len = (u32)span;
	else
		len = filesz;

	if (span_out)
		*span_out = (u32)span;
	if (filesz_out)
		*filesz_out = filesz;
	if (code_out)
		*code_out = code;
	if (len_out)
		*len_out = len;

	pr_err("talkman-mba AUTH window start %pa len %#x (hive %pa span %#zx filesz %#x win_filesz=%d auth_filesz=%u code_span=%u)\n",
	       &code, len, &hive, span, filesz,
	       filesz == TALKMAN_WIN_FILESZ, lab_auth_filesz, lab_code_span);

	if (!code || !len)
		return -EINVAL;
	return 0;
}

static bool lab_virt_is_linear(void *virt, dma_addr_t phys)
{
	return virt && pfn_valid(__phys_to_pfn(phys)) &&
	       virt == phys_to_virt(phys);
}

static void lab_log_hash_phdr(struct device *dev, const char *when,
			      const u8 *elf, size_t sz, dma_addr_t meta_phys)
{
	int idx = -1;
	const struct elf32_phdr *ph = lab_hash_phdr(elf, sz, &idx);
	u32 pa, off, al, fsz;
	int aligned;

	if (!ph) {
		talkman_mba_emerg("talkman-mba hash PH %s: none (sz %zu)\n", when, sz);
		return;
	}
	pa = ph->p_paddr;
	off = ph->p_offset;
	al = ph->p_align ? ph->p_align : 1;
	fsz = ph->p_filesz;
	aligned = !(pa & (al - 1));
	talkman_mba_emerg("talkman-mba hash PH %s: PH%02d p_paddr=%08x p_off=%08x p_align=%08x filesz=%#x meta=%08x win_working=%08x aligned=%d align_ok=%d\n",
	       when, idx, pa, off, al, fsz, (u32)meta_phys,
	       (u32)meta_phys + 0x334, aligned, aligned);
}

static size_t lab_pack_pmi_metadata(u8 *dst, size_t dst_sz, const u8 *src,
				    size_t src_sz, dma_addr_t dst_phys)
{
	const struct elf32_hdr *eh;
	const struct elf32_phdr *sph;
	struct elf32_phdr *dph;
	size_t phoff, hash_off, packed;
	const u8 *hash;
	u32 hsz, hoff, old_hash_pa;
	int hash_i = -1;

	sph = lab_hash_phdr(src, src_sz, &hash_i);
	if (!sph || sph->p_offset + (size_t)sph->p_filesz > src_sz)
		goto verbatim;

	hoff = sph->p_offset;
	hsz = sph->p_filesz;
	old_hash_pa = sph->p_paddr;
	eh = (const struct elf32_hdr *)src;
	phoff = eh->e_phoff;

	if (lab_win_hash) {
		/*
		 * Keep the on-disk MDT: hash stays at p_offset 0x1000,
		 * p_paddr 0x0CA00000, p_align 0x1000. Do not rewrite the
		 * MBA-visible PHDR to packed+0x334 (#26 DEBUG 7).
		 */
		if (src_sz > dst_sz)
			return 0;
		memcpy(dst, src, src_sz);
		talkman_mba_emerg("talkman-mba win_hash PH%02d keep p_paddr=%08x p_off=%08x filesz=%#x (packed %zu @ %08x; windows working dest %08x)\n",
		       hash_i, old_hash_pa, hoff, hsz, src_sz, (u32)dst_phys,
		       (u32)dst_phys + 0x334);
		return src_sz;
	}

	{
		size_t hdr_len = phoff + (size_t)eh->e_phnum * sizeof(*sph);

		/* Fallback: 4K-align the packed hash and point p_paddr there. */
		hash_off = ALIGN(hdr_len, PAGE_SIZE);
		packed = hash_off + hsz;
		if (packed > dst_sz)
			goto verbatim;
		memcpy(dst, src, hdr_len);
	}
	hash = src + hoff;
	memcpy(dst + hash_off, hash, hsz);
	dph = (struct elf32_phdr *)(dst + phoff) + hash_i;
	dph->p_offset = hash_off;
	dph->p_paddr = dst_phys + hash_off;
	if (lab_place_hash && old_hash_pa) {
		talkman_mba_emerg("talkman-mba hash PHDR packed %#x; ELF hash at %#x\n",
		       dph->p_paddr, old_hash_pa);
	} else if (old_hash_pa && hsz >= 4) {
		u32 new_base = (u32)dst_phys + (u32)hash_off;
		u32 *words = (u32 *)(dst + hash_off);
		size_t nw = hsz / 4, k, nfix = 0;

		for (k = 0; k < nw; k++) {
			if (words[k] >= old_hash_pa &&
			    words[k] < old_hash_pa + hsz) {
				words[k] = words[k] - old_hash_pa + new_base;
				nfix++;
			}
		}
		talkman_mba_emerg("talkman-mba relocated %zu hash ptrs %#x -> %#x\n",
		       nfix, old_hash_pa, new_base);
	}
	return packed;

verbatim:
	if (src_sz > dst_sz)
		return 0;
	memcpy(dst, src, src_sz);
	return src_sz;
}

/* Windows leaves the hash at ELF 0x0CA00000 (inside the 90 MiB window).
 * Linux modem CMA 0x07400000+90MiB includes that address — phys_to_virt,
 * not ioremap and not a reserved-memory node.
 */
static int lab_place_elf_hash(struct pil_desc *pil, const u8 *src, size_t src_sz)
{
	const struct elf32_phdr *ph;
	phys_addr_t hive, hend, pa;
	size_t hsz;
	u32 hoff;
	int idx = -1;
	void *v;
	const u32 *w;

	if (!lab_place_hash)
		return -ENOENT;
	ph = lab_hash_phdr(src, src_sz, &idx);
	if (!ph || ph->p_offset + (size_t)ph->p_filesz > src_sz)
		return -EINVAL;
	hoff = ph->p_offset;
	hsz = ph->p_filesz;
	pa = ph->p_paddr;
	hive = pil_get_region_start(pil);
	hend = pil_get_region_end(pil);
	talkman_mba_emerg("talkman-mba place hash PH%02d elf %pa+%#zx hive %pa-%pa pfn_valid=%d aligned=%d align_ok=%d\n",
	       idx, &pa, hsz, &hive, &hend,
	       pfn_valid(__phys_to_pfn(pa)), !((u32)pa & 0xfff),
	       !((u32)pa & 0xfff));
	if (pa < hive || pa + hsz > hend) {
		talkman_mba_emerg("talkman-mba hash ELF %pa+%#zx outside hive, skip place\n",
		       &pa, hsz);
		return -ENXIO;
	}
	/* 0x0CA is inside modem CMA (0x074+90MiB). ioremap_wc of that
	 * RAM hung m28. Use the linear map only; never ioremap.
	 */
	if (!pfn_valid(__phys_to_pfn(pa))) {
		dev_err(pil->dev,
			 "talkman lab: hash ELF %pa not pfn_valid, skip place\n",
			 &pa);
		return -EFAULT;
	}
	v = phys_to_virt(pa);
	memcpy(v, src + hoff, hsz);
	__flush_dcache_area(v, hsz);
	wmb();
	w = v;
	talkman_mba_emerg("talkman-mba placed hash %#zx at ELF %pa via phys_to_virt head %08x %08x ptrs %08x %08x %08x\n",
	       hsz, &pa, w[0], w[1], w[3], w[6], w[8]);
	return 0;
}

static int pil_msa_auth_modem_mdt(struct pil_desc *pil, const u8 *metadata,
					size_t size)
{
	struct modem_data *drv = dev_get_drvdata(pil->dev);
	void *mdata_virt;
	dma_addr_t mdata_phys;
	s32 status;
	int ret;
	size_t mdt_sz = size;
	size_t packed = 0;
	int place_rc = -ENOENT;
	u32 hash_pa, dbg;
	DEFINE_DMA_ATTRS(attrs);

	drv->lab_meta_ok = false;
	drv->lab_auth_start = 0;
	drv->lab_auth_span_len = 0;
	drv->lab_auth_filesz_len = 0;
	drv->mba_mem_dev.coherent_dma_mask =
		DMA_BIT_MASK(sizeof(dma_addr_t) * 8);
	dma_set_attr(DMA_ATTR_STRONGLY_ORDERED, &attrs);
	/*
	 * Windows qcpil8994 allocates a 16 KB / 4K-aligned PMI metadata
	 * buffer. Hexagon TLB create rejects non-power-of-two sizes
	 * ("TLB create error: incorrect size"); modem.mdt is 11304 bytes,
	 * so map 16 KB and zero-fill the tail.
	 */
	{
		size_t alloc_sz = ALIGN(size, SZ_16K);

		if (alloc_sz < SZ_16K)
			alloc_sz = SZ_16K;
		mdata_virt = pil_mss_mba_alloc(drv, alloc_sz, true,
					       &mdata_phys, &attrs);
		if (!mdata_virt) {
			dev_err(pil->dev,
				"MBA metadata buffer allocation failed (mdt %zu alloc %zu)\n",
				size, alloc_sz);
			ret = -ENOMEM;
			goto fail;
		}
		memset(mdata_virt, 0, alloc_sz);
		packed = lab_pack_pmi_metadata(mdata_virt, alloc_sz,
					       metadata, size, mdata_phys);
		if (!packed) {
			dev_err(pil->dev,
				"MBA metadata pack failed (mdt %zu)\n",
				size);
			pil_mss_mba_free(drv, alloc_sz, mdata_virt,
					 mdata_phys, &attrs);
			ret = -EINVAL;
			goto fail;
		}
		if (lab_reloc_headers && !lab_identity)
			pil_reloc_elf_paddrs_in_mdt(pil, mdata_virt, packed);
		hash_pa = lab_hash_paddr(mdata_virt, packed);
		talkman_mba_emerg("talkman-mba MBA metadata: mdt %zu packed %zu mapped %zu at %pad (hash paddr %08x align_ok=%d win_hash=%u)\n",
		       size, packed, alloc_sz, &mdata_phys, hash_pa,
		       hash_pa && !(hash_pa & 0xfff), lab_win_hash);
		lab_log_hash_phdr(pil->dev, "after-pack/reloc", mdata_virt,
				  packed, mdata_phys);
		/* Keep alloc_sz so the matching free uses the same length. */
		size = alloc_sz;
	}
	/* wmb() drains write-combine; mapping is PROT_NORMAL_NC. */
	wmb();
	if (lab_virt_is_linear(mdata_virt, mdata_phys))
		__flush_dcache_area(mdata_virt, size);

	if (lab_load_before_auth) {
		if (lab_identity) {
			pil->map_fw_mem = lab_ident_map;
			pil->unmap_fw_mem = lab_ident_unmap;
			pil_force_elf_paddrs(pil);
		}
		talkman_mba_emerg("talkman-mba hash-paddr=%08x align_ok=%d win_hash=%u (pre-copy)\n",
		       hash_pa, hash_pa && !(hash_pa & 0xfff), lab_win_hash);
		lab_log_rmb(pil->dev, drv->rmb_base, "pre-copy segs");
		ret = pil_copy_all_segs(pil);
		if (ret) {
			dev_err(pil->dev,
				"talkman lab: preload PT_LOAD failed %d\n", ret);
			pil_mss_mba_free(drv, size, mdata_virt, mdata_phys,
					 &attrs);
			goto fail;
		}
		pil->lab_skip_seg_load = true;
		{
			phys_addr_t hive = pil_get_region_start(pil);

			pr_err("talkman-mba PT_LOAD copied before headers, hive %pa\n",
			       &hive);
			talkman_mba_emerg("talkman-mba hash-paddr=%08x align_ok=%d win_hash=%u (post-copy)\n",
			       hash_pa, hash_pa && !(hash_pa & 0xfff),
			       lab_win_hash);
			lab_log_rmb(pil->dev, drv->rmb_base, "post-copy segs");
		}
		if (lab_mirror_size) {
			int mret;

			mret = pil_mirror_elf_window(pil, lab_mirror_phys,
						     lab_mirror_size);
			if (mret)
				dev_err(pil->dev,
					"talkman lab: ELF-window mirror %#lx+%#lx rc=%d\n",
					lab_mirror_phys, lab_mirror_size, mret);
		}
	} else {
		talkman_mba_emerg("talkman-mba META before preload (load_before_auth=0) hash-paddr=%08x align_ok=%d win_hash=%u\n",
		       hash_pa, hash_pa && !(hash_pa & 0xfff), lab_win_hash);
		lab_log_rmb(pil->dev, drv->rmb_base, "pre-META no-copy");
	}
	/* After PT_LOAD so hash at 0x0CA wins over any overlapping BSS. */
	place_rc = lab_place_elf_hash(pil, metadata, mdt_sz);
	if (lab_virt_is_linear(mdata_virt, mdata_phys))
		__flush_dcache_area(mdata_virt, size);

	/* After any preload so a successful SHARE cannot lock AP out. */
	lab_share_memory(pil->dev);
	if (lab_init_image && lab_tz_pas_id)
		lab_scm_pas_init_image(pil->dev, lab_tz_pas_id, mdata_phys);

	hash_pa = lab_hash_paddr(mdata_virt, packed);
	talkman_mba_emerg("talkman-mba ringing META meta=%pad hash-paddr=%08x align_ok=%d place=%d reloc=%u identity=%u win_hash=%u\n",
	       &mdata_phys, hash_pa, hash_pa && !(hash_pa & 0xfff), place_rc,
	       lab_reloc_headers, lab_identity, lab_win_hash);

	/* Initialize length counter to 0 */
	writel_relaxed(0, drv->rmb_base + RMB_PMI_CODE_LENGTH);

	/* Pass address of meta-data to the MBA and perform authentication */
	writel_relaxed(mdata_phys, drv->rmb_base + RMB_PMI_META_DATA);
	writel_relaxed(CMD_META_DATA_READY, drv->rmb_base + RMB_MBA_COMMAND);
	lab_log_rmb(pil->dev, drv->rmb_base, "after META cmd");
	ret = readl_poll_timeout(drv->rmb_base + RMB_MBA_STATUS, status,
		status == STATUS_META_DATA_AUTH_SUCCESS || status < 0,
		POLL_INTERVAL_US, modem_auth_timeout_ms * 1000);
	dbg = readl_relaxed(drv->rmb_base + RMB_MBA_DEBUG_INFORMATION);
	talkman_mba_emerg("talkman-mba after META poll rc=%d STATUS=%d DEBUG=%08x (%s) hash-paddr=%08x align_ok=%d win_hash=%u\n",
	       ret, status, dbg, lab_mba_dbg_name(dbg), hash_pa,
	       hash_pa && !(hash_pa & 0xfff), lab_win_hash);
	lab_log_rmb(pil->dev, drv->rmb_base, "after META poll");
	if (ret) {
		dev_err(pil->dev, "MBA authentication of headers timed out\n");
	} else if (status < 0) {
		dev_err(pil->dev, "MBA returned error %d for headers\n",
				status);
		ret = -EINVAL;
	}

	if (!ret && status == STATUS_META_DATA_AUTH_SUCCESS &&
	    lab_load_before_auth) {
		phys_addr_t code = 0;
		u32 code_len = 0, span_len = 0, filesz_len = 0;
		const char *why = "after META=3 hive/span";

		if (lab_pick_auth_window(pil, &code, &code_len, &span_len,
					 &filesz_len)) {
			dev_err(pil->dev,
				"talkman lab: AUTH window invalid after META=3\n");
			ret = -EINVAL;
		} else {
			drv->lab_meta_ok = true;
			drv->lab_auth_start = code;
			drv->lab_auth_span_len = span_len;
			drv->lab_auth_filesz_len = filesz_len;
			if (lab_auth_filesz == 1)
				why = "after META=3 filesz";
			else if (!lab_code_span)
				why = "after META=3 filesz (code_span=0)";
			lab_ring_load_ready(pil, code, code_len, why);
		}
	}

	if (ret || !lab_load_before_auth) {
		pil_mss_mba_free(drv, size, mdata_virt, mdata_phys, &attrs);
	} else {
		/* Windows FUN_004064b4: free metadata after AUTH_COMPLETE. */
		drv->lab_mdata_virt = mdata_virt;
		drv->lab_mdata_phys = mdata_phys;
		drv->lab_mdata_size = size;
	}

	if (!ret)
		return ret;

fail:
	modem_log_rmb_regs(drv->rmb_base);
	if (drv->q6) {
		pil_mss_shutdown(pil);
		pil_mss_mba_free(drv, drv->q6->mba_size, drv->q6->mba_virt,
				 drv->q6->mba_phys, &drv->attrs_dma);
		drv->q6->mba_virt = NULL;
	}
	return ret;
}

static int pil_msa_mss_reset_mba_load_auth_mdt(struct pil_desc *pil,
				  const u8 *metadata, size_t size)
{
	int ret;

	ret = pil_mss_reset_load_mba(pil);
	if (ret)
		return ret;

	return pil_msa_auth_modem_mdt(pil, metadata, size);
}

static int pil_msa_mba_verify_blob(struct pil_desc *pil, phys_addr_t phy_addr,
				   size_t size)
{
	struct modem_data *drv = dev_get_drvdata(pil->dev);
	s32 status;
	u32 img_length = readl_relaxed(drv->rmb_base + RMB_PMI_CODE_LENGTH);

	/* Begin image authentication */
	if (img_length == 0) {
		writel_relaxed(phy_addr, drv->rmb_base + RMB_PMI_CODE_START);
		writel_relaxed(CMD_LOAD_READY, drv->rmb_base + RMB_MBA_COMMAND);
	}
	/* Increment length counter */
	img_length += size;
	writel_relaxed(img_length, drv->rmb_base + RMB_PMI_CODE_LENGTH);

	status = readl_relaxed(drv->rmb_base + RMB_MBA_STATUS);
	if (status < 0) {
		dev_err(pil->dev, "MBA returned error %d\n", status);
		modem_log_rmb_regs(drv->rmb_base);
		return -EINVAL;
	}

	return 0;
}

static int pil_msa_mba_auth(struct pil_desc *pil)
{
	struct modem_data *drv = dev_get_drvdata(pil->dev);
	struct q6v5_data *q6_drv = container_of(pil, struct q6v5_data, desc);
	int ret;
	s32 status;
	u32 dbg, first_len;

	/* Wait for all segments to be authenticated or an error to occur */
	ret = readl_poll_timeout(drv->rmb_base + RMB_MBA_STATUS, status,
			status == STATUS_AUTH_COMPLETE || status < 0,
			50, modem_auth_timeout_ms * 1000);
	dbg = readl_relaxed(drv->rmb_base + RMB_MBA_DEBUG_INFORMATION);
	first_len = readl_relaxed(drv->rmb_base + RMB_PMI_CODE_LENGTH);
	talkman_mba_emerg("talkman-mba after AUTH poll rc=%d STATUS=%d DEBUG=%08x (%s) len=%08x\n",
	       ret, status, dbg, lab_mba_dbg_name(dbg), first_len);
	lab_log_rmb(pil->dev, drv->rmb_base, "after AUTH poll");
	if (ret) {
		dev_err(pil->dev, "MBA authentication of image timed out\n");
	} else if (status < 0) {
		dev_err(pil->dev, "MBA returned error %d for image\n", status);
		ret = -EINVAL;
		/*
		 * Gated retry: META already measured ==3, first AUTH used
		 * hive/span, MBA rejected the length (DEBUG 11) or range.
		 * Rewrite CODE_LENGTH to filesz and ring CMD_LOAD_READY once.
		 * Do not retry on timeout (MBA may still be walking span).
		 */
		if (lab_auth_filesz == 2 && drv->lab_meta_ok &&
		    drv->lab_auth_filesz_len &&
		    first_len != drv->lab_auth_filesz_len) {
			s32 status2;
			int ret2;

			talkman_mba_emerg("talkman-mba AUTH retry filesz %#x (was %#x DEBUG %u) after META=3\n",
			       drv->lab_auth_filesz_len, first_len, dbg);
			lab_ring_load_ready(pil, drv->lab_auth_start,
					    drv->lab_auth_filesz_len,
					    "AUTH filesz retry");
			ret2 = readl_poll_timeout(drv->rmb_base + RMB_MBA_STATUS,
				status2,
				status2 == STATUS_AUTH_COMPLETE || status2 < 0,
				50, modem_auth_timeout_ms * 1000);
			dbg = readl_relaxed(drv->rmb_base +
					    RMB_MBA_DEBUG_INFORMATION);
			talkman_mba_emerg("talkman-mba after AUTH filesz retry rc=%d STATUS=%d DEBUG=%08x (%s)\n",
			       ret2, status2, dbg, lab_mba_dbg_name(dbg));
			lab_log_rmb(pil->dev, drv->rmb_base,
				    "after AUTH filesz retry");
			if (!ret2 && status2 == STATUS_AUTH_COMPLETE) {
				ret = 0;
				status = status2;
			} else if (!ret2 && status2 < 0) {
				dev_err(pil->dev,
					"MBA filesz retry error %d for image\n",
					status2);
				ret = -EINVAL;
			} else {
				dev_err(pil->dev,
					"MBA filesz retry timed out\n");
				ret = ret2 ? ret2 : -ETIMEDOUT;
			}
		}
	}

	if (drv->lab_mdata_virt) {
		DEFINE_DMA_ATTRS(mdata_attrs);

		dma_set_attr(DMA_ATTR_STRONGLY_ORDERED, &mdata_attrs);
		pil_mss_mba_free(drv, drv->lab_mdata_size, drv->lab_mdata_virt,
				 drv->lab_mdata_phys, &mdata_attrs);
		drv->lab_mdata_virt = NULL;
	}
	if (drv->q6 && drv->q6->mba_virt) {
		/* Reclaim MBA memory. */
		pil_mss_mba_free(drv, drv->q6->mba_size, drv->q6->mba_virt,
				 drv->q6->mba_phys, &drv->attrs_dma);
		drv->q6->mba_virt = NULL;
	}

	if (ret)
		modem_log_rmb_regs(drv->rmb_base);
	if (q6_drv->ahb_clk_vote)
		clk_disable_unprepare(q6_drv->ahb_clk);

	return ret;
}

/*
 * To be used only if self-auth is disabled, or if the
 * MBA image is loaded as segments and not in init_image.
 */
struct pil_reset_ops pil_msa_mss_ops = {
	.proxy_vote = pil_mss_make_proxy_votes,
	.proxy_unvote = pil_mss_remove_proxy_votes,
	.auth_and_reset = pil_mss_reset,
	.shutdown = pil_mss_shutdown,
};

/*
 * To be used if self-auth is enabled and the MBA is to be loaded
 * in init_image and the modem headers are also to be authenticated
 * in init_image. Modem segments authenticated in auth_and_reset.
 */
struct pil_reset_ops pil_msa_mss_ops_selfauth = {
	.init_image = pil_msa_mss_reset_mba_load_auth_mdt,
	.proxy_vote = pil_mss_make_proxy_votes,
	.proxy_unvote = pil_mss_remove_proxy_votes,
	.verify_blob = pil_msa_mba_verify_blob,
	.auth_and_reset = pil_msa_mba_auth,
	.deinit_image = pil_mss_deinit_image,
	.shutdown = pil_mss_shutdown,
};

/*
 * To be used if the modem headers are to be authenticated
 * in init_image, and the modem segments in auth_and_reset.
 */
struct pil_reset_ops pil_msa_femto_mba_ops = {
	.init_image = pil_msa_auth_modem_mdt,
	.verify_blob = pil_msa_mba_verify_blob,
	.auth_and_reset = pil_msa_mba_auth,
};
