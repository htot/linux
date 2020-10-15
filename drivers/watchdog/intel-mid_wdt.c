// SPDX-License-Identifier: GPL-2.0-only
/*
 *      intel-mid_wdt: generic Intel MID SCU watchdog driver
 *
 *      Platforms supported so far:
 *      - Merrifield only
 *
 *      Copyright (C) 2014 Intel Corporation. All rights reserved.
 *      Contact: David Cohen <david.a.cohen@linux.intel.com>
 */

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/nmi.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>
#include <linux/platform_data/intel-mid_wdt.h>

#include <asm/cpu_device_id.h>
#include <asm/hw_irq.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel-family.h>
#include <asm/intel-mid.h>
#include <asm/io_apic.h>

#define IPC_WATCHDOG 0xf8

#define MID_WDT_PRETIMEOUT		15
#define MID_WDT_TIMEOUT_MIN		(1 + MID_WDT_PRETIMEOUT)
#define MID_WDT_TIMEOUT_MAX		170
#define MID_WDT_DEFAULT_TIMEOUT		90

/* SCU watchdog messages */
enum {
	SCU_WATCHDOG_START = 0,
	SCU_WATCHDOG_STOP,
	SCU_WATCHDOG_KEEPALIVE,
};

struct mid_wdt {
	struct watchdog_device wd;
	struct device *dev;
	struct intel_scu_ipc_dev *scu;
};

static inline int
wdt_command(struct mid_wdt *mid, int sub, const void *in, size_t inlen, size_t size)
{
	struct intel_scu_ipc_dev *scu = mid->scu;

	return intel_scu_ipc_dev_command_with_size(scu, IPC_WATCHDOG, sub, in,
						   inlen, size, NULL, 0);
}

static int wdt_start(struct watchdog_device *wd)
{
	struct mid_wdt *mid = watchdog_get_drvdata(wd);
	int ret, in_size;
	int timeout = wd->timeout;
	struct ipc_wd_start {
		u32 pretimeout;
		u32 timeout;
	} ipc_wd_start = { timeout - MID_WDT_PRETIMEOUT, timeout };

	/*
	 * SCU expects the input size for watchdog IPC to be 2 which is the
	 * size of the structure in dwords. SCU IPC normally takes bytes
	 * but this is a special case where we specify size to be different
	 * than inlen.
	 */
	in_size = DIV_ROUND_UP(sizeof(ipc_wd_start), 4);

	ret = wdt_command(mid, SCU_WATCHDOG_START, &ipc_wd_start,
			  sizeof(ipc_wd_start), in_size);
	if (ret)
		dev_crit(mid->dev, "error starting watchdog: %d\n", ret);

	return ret;
}

static int wdt_ping(struct watchdog_device *wd)
{
	struct mid_wdt *mid = watchdog_get_drvdata(wd);
	int ret;

	ret = wdt_command(mid, SCU_WATCHDOG_KEEPALIVE, NULL, 0, 0);
	if (ret)
		dev_crit(mid->dev, "Error executing keepalive: %d\n", ret);

	return ret;
}

static int wdt_stop(struct watchdog_device *wd)
{
	struct mid_wdt *mid = watchdog_get_drvdata(wd);
	int ret;

	ret = wdt_command(mid, SCU_WATCHDOG_STOP, NULL, 0, 0);
	if (ret)
		dev_crit(mid->dev, "Error stopping watchdog: %d\n", ret);

	return ret;
}

static irqreturn_t mid_wdt_irq(int irq, void *dev_id)
{
	panic("Kernel Watchdog");

	/* This code should not be reached */
	return IRQ_HANDLED;
}

static const struct watchdog_info mid_wdt_info = {
	.identity = "Intel MID SCU watchdog",
	.options = WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT | WDIOF_MAGICCLOSE,
};

static const struct watchdog_ops mid_wdt_ops = {
	.owner = THIS_MODULE,
	.start = wdt_start,
	.stop = wdt_stop,
	.ping = wdt_ping,
};

#define TANGIER_EXT_TIMER0_MSI 12

static int tangier_probe(struct platform_device *pdev)
{
	struct intel_mid_wdt_pdata *pdata = pdev->dev.platform_data;
	struct irq_alloc_info info;
	int gsi = TANGIER_EXT_TIMER0_MSI;
	int irq;

	/* IOAPIC builds identity mapping between GSI and IRQ */
	ioapic_set_alloc_attr(&info, cpu_to_node(0), IOAPIC_LEVEL, IOAPIC_POL_HIGH);

	irq = mp_map_gsi_to_irq(gsi, IOAPIC_MAP_ALLOC, &info);
	if (irq < 0) {
		dev_warn(&pdev->dev, "can't find interrupt %d in IOAPIC\n", gsi);
		return irq;
	}

	pdata->irq = irq;
	return 0;
}

static struct intel_mid_wdt_pdata tangier_pdata = {
	.probe = tangier_probe,
};

static const struct x86_cpu_id intel_mid_cpu_ids[] = {
	X86_MATCH_INTEL_FAM6_MODEL(ATOM_SILVERMONT_MID, &tangier_pdata),
	{}
};
MODULE_DEVICE_TABLE(x86cpu, intel_mid_cpu_ids);

static int mid_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct watchdog_device *wdt_dev;
	struct intel_mid_wdt_pdata *pdata = dev->platform_data;
	struct mid_wdt *mid;
	int ret;

	if (!pdata) {
		const struct x86_cpu_id *id;

		id = x86_match_cpu(intel_mid_cpu_ids);
		if (id)
			pdata = (struct intel_mid_wdt_pdata *)id->driver_data;
	}
	if (!pdata) {
		dev_err(dev, "missing platform data\n");
		return -EINVAL;
	}

	if (pdata->probe) {
		ret = pdata->probe(pdev);
		if (ret)
			return ret;
	}

	mid = devm_kzalloc(dev, sizeof(*mid), GFP_KERNEL);
	if (!mid)
		return -ENOMEM;

	mid->dev = dev;
	wdt_dev = &mid->wd;

	wdt_dev->info = &mid_wdt_info;
	wdt_dev->ops = &mid_wdt_ops;
	wdt_dev->min_timeout = MID_WDT_TIMEOUT_MIN;
	wdt_dev->max_timeout = MID_WDT_TIMEOUT_MAX;
	wdt_dev->timeout = MID_WDT_DEFAULT_TIMEOUT;
	wdt_dev->parent = dev;

	watchdog_set_nowayout(wdt_dev, WATCHDOG_NOWAYOUT);
	watchdog_set_drvdata(wdt_dev, mid);

	ret = devm_request_irq(dev, pdata->irq, mid_wdt_irq,
			       IRQF_SHARED | IRQF_NO_SUSPEND, "watchdog",
			       wdt_dev);
	if (ret) {
		dev_err(dev, "error requesting warning irq %d\n", pdata->irq);
		return ret;
	}

	mid->scu = devm_intel_scu_ipc_dev_get(dev);
	if (!mid->scu)
		return -EPROBE_DEFER;

	/*
	 * The firmware followed by U-Boot leaves the watchdog running
	 * with the default threshold which may vary. When we get here
	 * we should make a decision to prevent any side effects before
	 * user space daemon will take care of it. The best option,
	 * taking into consideration that there is no way to read values
	 * back from hardware, is to enforce watchdog being run with
	 * deterministic values.
	 */
	ret = wdt_start(wdt_dev);
	if (ret)
		return ret;

	/* Make sure the watchdog is serviced */
	set_bit(WDOG_HW_RUNNING, &wdt_dev->status);

	ret = devm_watchdog_register_device(dev, wdt_dev);
	if (ret)
		return ret;

	dev_info(dev, "Intel MID watchdog device probed\n");

	return 0;
}

static struct platform_driver mid_wdt_driver = {
	.probe		= mid_wdt_probe,
	.driver		= {
		.name	= "intel_mid_wdt",
	},
};

module_platform_driver(mid_wdt_driver);

MODULE_AUTHOR("David Cohen <david.a.cohen@linux.intel.com>");
MODULE_DESCRIPTION("Watchdog Driver for Intel MID platform");
MODULE_LICENSE("GPL");
