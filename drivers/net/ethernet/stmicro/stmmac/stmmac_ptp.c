// SPDX-License-Identifier: GPL-2.0-only
/*******************************************************************************
  PTP 1588 clock using the STMMAC.

  Copyright (C) 2013  Vayavya Labs Pvt Ltd


  Author: Rayagond Kokatanur <rayagond@vayavyalabs.com>
*******************************************************************************/
#include "stmmac.h"
#include "stmmac_ptp.h"
#include "dwmac4.h"
#include <linux/iopoll.h>
#ifdef CONFIG_X86
#include <asm/intel-family.h>
#endif
/**
 * stmmac_adjust_freq
 *
 * @ptp: pointer to ptp_clock_info structure
 * @ppb: desired period change in parts ber billion
 *
 * Description: this function will adjust the frequency of hardware clock.
 */
static int stmmac_adjust_freq(struct ptp_clock_info *ptp, s32 ppb)
{
	struct stmmac_priv *priv =
	    container_of(ptp, struct stmmac_priv, ptp_clock_ops);
	unsigned long flags;
	u32 diff, addend;
	int neg_adj = 0;
	u64 adj;

	if (ppb < 0) {
		neg_adj = 1;
		ppb = -ppb;
	}

	addend = priv->default_addend;
	adj = addend;
	adj *= ppb;
	diff = div_u64(adj, 1000000000ULL);
	addend = neg_adj ? (addend - diff) : (addend + diff);

	spin_lock_irqsave(&priv->ptp_lock, flags);
	stmmac_config_addend(priv, priv->ptpaddr, addend);
	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	return 0;
}

/**
 * stmmac_adjust_time
 *
 * @ptp: pointer to ptp_clock_info structure
 * @delta: desired change in nanoseconds
 *
 * Description: this function will shift/adjust the hardware clock time.
 */
static int stmmac_adjust_time(struct ptp_clock_info *ptp, s64 delta)
{
	struct stmmac_priv *priv =
	    container_of(ptp, struct stmmac_priv, ptp_clock_ops);
	unsigned long flags;
	u32 sec, nsec;
	u32 quotient, reminder;
	int neg_adj = 0;
	bool xmac;

	xmac = priv->plat->has_gmac4 || priv->plat->has_xgmac;

	if (delta < 0) {
		neg_adj = 1;
		delta = -delta;
	}

	quotient = div_u64_rem(delta, 1000000000ULL, &reminder);
	sec = quotient;
	nsec = reminder;

	spin_lock_irqsave(&priv->ptp_lock, flags);
	stmmac_adjust_systime(priv, priv->ptpaddr, sec, nsec, neg_adj, xmac);
	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	return 0;
}

/**
 * stmmac_get_time
 *
 * @ptp: pointer to ptp_clock_info structure
 * @ts: pointer to hold time/result
 *
 * Description: this function will read the current time from the
 * hardware clock and store it in @ts.
 */
static int stmmac_get_time(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
	struct stmmac_priv *priv =
	    container_of(ptp, struct stmmac_priv, ptp_clock_ops);
	unsigned long flags;
	u64 ns = 0;

	spin_lock_irqsave(&priv->ptp_lock, flags);
	stmmac_get_systime(priv, priv->ptpaddr, &ns);
	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	*ts = ns_to_timespec64(ns);

	return 0;
}

/**
 * stmmac_set_time
 *
 * @ptp: pointer to ptp_clock_info structure
 * @ts: time value to set
 *
 * Description: this function will set the current time on the
 * hardware clock.
 */
static int stmmac_set_time(struct ptp_clock_info *ptp,
			   const struct timespec64 *ts)
{
	struct stmmac_priv *priv =
	    container_of(ptp, struct stmmac_priv, ptp_clock_ops);
	unsigned long flags;

	spin_lock_irqsave(&priv->ptp_lock, flags);
	stmmac_init_systime(priv, priv->ptpaddr, ts->tv_sec, ts->tv_nsec);
	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	return 0;
}

static int stmmac_enable(struct ptp_clock_info *ptp,
			 struct ptp_clock_request *rq, int on)
{
	struct stmmac_priv *priv =
	    container_of(ptp, struct stmmac_priv, ptp_clock_ops);
	void __iomem *ptpaddr = priv->ptpaddr;
	struct stmmac_pps_cfg *cfg;
	int ret = -EOPNOTSUPP;
	unsigned long flags;
	u32 acr_value;

	switch (rq->type) {
	case PTP_CLK_REQ_PEROUT:
		/* Reject requests with unsupported flags */
		if (rq->perout.flags)
			return -EOPNOTSUPP;

		cfg = &priv->pps[rq->perout.index];

		cfg->start.tv_sec = rq->perout.start.sec;
		cfg->start.tv_nsec = rq->perout.start.nsec;
		cfg->period.tv_sec = rq->perout.period.sec;
		cfg->period.tv_nsec = rq->perout.period.nsec;

		spin_lock_irqsave(&priv->ptp_lock, flags);
		ret = stmmac_flex_pps_config(priv, priv->ioaddr,
					     rq->perout.index, cfg, on,
					     priv->sub_second_inc,
					     priv->systime_flags);
		spin_unlock_irqrestore(&priv->ptp_lock, flags);
		break;
	case PTP_CLK_REQ_EXTTS:
		priv->plat->ext_snapshot_en = on;
		acr_value = readl(ptpaddr + PTP_ACR);
		acr_value &= ~PTP_ACR_MASK;
		if (on) {
			/* Enable External snapshot trigger */
			acr_value |= priv->plat->ext_snapshot_num;
			acr_value |= PTP_ACR_ATSFC;
			pr_info("Auxiliary Snapshot %d enabled\n",
				priv->plat->ext_snapshot_num >> 5);
		} else {
			pr_info("Auxiliary Snapshot %d disabled\n",
				priv->plat->ext_snapshot_num >> 5);
		}
		writel(acr_value, ptpaddr + PTP_ACR);
		ret = 0;
		break;
	default:
		break;
	}

	return ret;
}

#ifdef CONFIG_STMMAC_HWTS
static int stmmac_cross_ts_isr(struct stmmac_priv *priv)
{
	return (readl(priv->ioaddr + GMAC_INT_STATUS) & GMAC_INT_TSIE);
}

/**
 * stmmac_get_syncdevicetime - Callback given to timekeeping code
 *                             reads system/device registers
 * @device: current device time
 * @system: system counter value read synchronously with device time
 * @ctx: context provided by timekeeping code
 *
 * Read device and system (ART) clock simultaneously and return the corrected
 * clock values in ns.
 **/
static int stmmac_get_syncdevicetime(ktime_t *device,
				     struct system_counterval_t *system,
				     void *ctx)
{
	struct stmmac_priv *priv = (struct stmmac_priv *)ctx;
	void __iomem *ptpaddr = priv->ptpaddr;
	void __iomem *ioaddr = priv->hw->pcsr;
	unsigned int ebx_numerator;
	unsigned int unused[3];
	unsigned long flags;
	u32 num_snapshot;
	u32 gpio_value;
	u32 acr_value;
	u64 art_time;
	u64 ptp_time;
	int i;

	priv->plat->int_snapshot_en = 1;

	/* Enable Internal snapshot trigger */
	acr_value = readl(ptpaddr + PTP_ACR);
	acr_value &= ~PTP_ACR_MASK;
	switch (priv->plat->int_snapshot_num) {
	case AUX_SNAPSHOT0:
		acr_value |= PTP_ACR_ATSEN0;
		break;
	case AUX_SNAPSHOT1:
		acr_value |= PTP_ACR_ATSEN1;
		break;
	case AUX_SNAPSHOT2:
		acr_value |= PTP_ACR_ATSEN2;
		break;
	case AUX_SNAPSHOT3:
		acr_value |= PTP_ACR_ATSEN3;
		break;
	default:
		return -EINVAL;
	}
	writel(acr_value, ptpaddr + PTP_ACR);

	/* Clear FIFO */
	acr_value = readl(ptpaddr + PTP_ACR);
	acr_value |= PTP_ACR_ATSFC;
	writel(acr_value, ptpaddr + PTP_ACR);

	/** Trigger Internal snapshot signal
	 * Create a rising edge by just toggle the GPO1 to low
	 * and back to high.
	 */
	gpio_value = readl(ioaddr + GMAC_GPIO_STATUS);
	gpio_value &= ~GPO1;
	writel(gpio_value, ioaddr + GMAC_GPIO_STATUS);
	gpio_value |= GPO1;
	writel(gpio_value, ioaddr + GMAC_GPIO_STATUS);

	/* Time sync done Indication - Interrupt method */
	if (!wait_event_timeout(priv->tstamp_busy_wait,
				stmmac_cross_ts_isr(priv), HZ / 100))
		return -ETIMEDOUT;

	num_snapshot = (readl(ioaddr + GMAC_TIMESTAMP_STATUS) &
			GMAC_TIMESTAMP_ATSNS_MASK) >>
			GMAC_TIMESTAMP_ATSNS_SHIFT;

	/* Repeat until the timestamps are from the FIFO last segment */
	for (i = 0; i < num_snapshot; i++) {
		spin_lock_irqsave(&priv->ptp_lock, flags);
		stmmac_get_ptptime(priv, ptpaddr, &ptp_time);
		*device = ns_to_ktime(ptp_time);
		spin_unlock_irqrestore(&priv->ptp_lock, flags);

		stmmac_get_arttime(priv, priv->mii,
				   priv->plat->intel_adhoc_addr, &art_time);
		*system = convert_art_to_tsc(art_time);
	}

	priv->plat->int_snapshot_en = 0;

	/* In the case of PSE Local ART, it might be running different frequency
	 * compared to the PMC ART, so we will need to perform a multiplication
	 * to match the PMC ART frequency.
	 */
	if (priv->plat->is_pse) {
		system->cycles *= priv->plat->pmc_art_to_pse_art_ratio;
		return 0;
	}
#ifdef CONFIG_X86
	/* [REVERTME] Workaround: TigerLake Internal MCP A2 stepping has ART
	 * value of 0.5x of actual ART. As it has same CPU family and model as
	 * the rest of the TigerLak CPU skus, we can only differentiate them
	 * using numerator value define in CPUID Leaf 15H. Note that this
	 * workaround is not required for Tigerlake Internal MCP A6 stepping.
	 */
	cpuid(0x15, unused, &ebx_numerator, unused + 1, unused + 2);
	if (boot_cpu_data.x86_model == INTEL_FAM6_TIGERLAKE_L &&
	    ebx_numerator == 0x5e)
		system->cycles *= 2;
#endif
	return 0;
}

static int stmmac_getcrosststamp(struct ptp_clock_info *ptp,
				 struct system_device_crosststamp *xtstamp)
{
	struct stmmac_priv *priv =
	    container_of(ptp, struct stmmac_priv, ptp_clock_ops);

	if (!boot_cpu_has(X86_FEATURE_ART))
		return -EOPNOTSUPP;

	return get_device_system_crosststamp(stmmac_get_syncdevicetime,
					     priv, NULL, xtstamp);
}
#endif

/* structure describing a PTP hardware clock */
static struct ptp_clock_info stmmac_ptp_clock_ops = {
	.owner = THIS_MODULE,
	.name = "stmmac ptp",
	.max_adj = 62500000,
	.n_alarm = 0,
	.n_ext_ts = 0,
	.n_per_out = 0, /* will be overwritten in stmmac_ptp_register */
	.n_pins = 0,
	.pps = 0,
	.adjfreq = stmmac_adjust_freq,
	.adjtime = stmmac_adjust_time,
	.gettime64 = stmmac_get_time,
	.settime64 = stmmac_set_time,
	.enable = stmmac_enable,
#ifdef CONFIG_STMMAC_HWTS
	.getcrosststamp = stmmac_getcrosststamp,
#endif
};

/**
 * stmmac_ptp_register
 * @priv: driver private structure
 * Description: this function will register the ptp clock driver
 * to kernel. It also does some house keeping work.
 */
void stmmac_ptp_register(struct stmmac_priv *priv)
{
	int aux_snapshot_n;
	int i;

	void __iomem *ioaddr = priv->hw->pcsr;
	u32 gpio_value;

	gpio_value = readl(ioaddr + GMAC_GPIO_STATUS);

	if (priv->plat->is_pse) {
		gpio_value &= ~PTP_PSE_CLK_FREQ_MASK;
		gpio_value |= PTP_PSE_CLK_FREQ_200MHZ;
	} else {
		gpio_value &= ~GPO0;
	}

	writel(gpio_value, ioaddr + GMAC_GPIO_STATUS);

	for (i = 0; i < priv->dma_cap.pps_out_num; i++) {
		if (i >= STMMAC_PPS_MAX)
			break;
		priv->pps[i].available = true;
	}

	if (priv->plat->ptp_max_adj)
		stmmac_ptp_clock_ops.max_adj = priv->plat->ptp_max_adj;

	stmmac_ptp_clock_ops.n_per_out = priv->dma_cap.pps_out_num;
	stmmac_ptp_clock_ops.n_ext_ts = priv->dma_cap.aux_snapshot_n;

	spin_lock_init(&priv->ptp_lock);
	priv->ptp_clock_ops = stmmac_ptp_clock_ops;

	aux_snapshot_n = priv->dma_cap.aux_snapshot_n;

	priv->ptp_clock = ptp_clock_register(&priv->ptp_clock_ops,
					     priv->device);
	if (IS_ERR(priv->ptp_clock)) {
		netdev_err(priv->dev, "ptp_clock_register failed\n");
		priv->ptp_clock = NULL;
	} else if (priv->ptp_clock)
		netdev_info(priv->dev, "registered PTP clock\n");
}

/**
 * stmmac_ptp_unregister
 * @priv: driver private structure
 * Description: this function will remove/unregister the ptp clock driver
 * from the kernel.
 */
void stmmac_ptp_unregister(struct stmmac_priv *priv)
{
	if (priv->ptp_clock) {
		ptp_clock_unregister(priv->ptp_clock);
		priv->ptp_clock = NULL;
		pr_debug("Removed PTP HW clock successfully on %s\n",
			 priv->dev->name);
	}
}
