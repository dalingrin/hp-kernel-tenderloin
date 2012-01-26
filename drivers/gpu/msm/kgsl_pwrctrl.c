/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#include <linux/interrupt.h>
#include <linux/err.h>
#include <mach/clk.h>
#include <mach/dal_axi.h>
#include <mach/msm_bus.h>

#include "kgsl.h"

int kgsl_pwrctrl_clk(struct kgsl_device *device, unsigned int pwrflag)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	switch (pwrflag) {
	case KGSL_PWRFLAGS_CLK_OFF:
		if (pwr->power_flags & KGSL_PWRFLAGS_CLK_ON) {
			if (pwr->grp_pclk)
				clk_disable(pwr->grp_pclk);
			clk_disable(pwr->grp_clk);
			if (pwr->imem_clk != NULL)
				clk_disable(pwr->imem_clk);
			if (pwr->imem_pclk != NULL)
				clk_disable(pwr->imem_pclk);
			if (pwr->clk_freq[KGSL_MIN_FREQ])
				clk_set_rate(pwr->grp_src_clk,
					pwr->clk_freq[KGSL_MIN_FREQ]);
			pwr->power_flags &=
					~(KGSL_PWRFLAGS_CLK_ON);
			pwr->power_flags |= KGSL_PWRFLAGS_CLK_OFF;
		}
		return KGSL_SUCCESS;
	case KGSL_PWRFLAGS_CLK_ON:
		if (pwr->power_flags & KGSL_PWRFLAGS_CLK_OFF) {
			if (pwr->clk_freq[KGSL_MAX_FREQ])
				clk_set_rate(pwr->grp_src_clk,
					pwr->clk_freq[KGSL_MAX_FREQ]);
			if (pwr->grp_pclk)
				clk_enable(pwr->grp_pclk);
			clk_enable(pwr->grp_clk);
			if (pwr->imem_clk != NULL)
				clk_enable(pwr->imem_clk);
			if (pwr->imem_pclk != NULL)
				clk_enable(pwr->imem_pclk);

			pwr->power_flags &=
				~(KGSL_PWRFLAGS_CLK_OFF);
			pwr->power_flags |= KGSL_PWRFLAGS_CLK_ON;
		}
		return KGSL_SUCCESS;
	default:
		return KGSL_FAILURE;
	}
}

int kgsl_pwrctrl_axi(struct kgsl_device *device, unsigned int pwrflag)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;

	switch (pwrflag) {
	case KGSL_PWRFLAGS_AXI_OFF:
		if (pwr->power_flags & KGSL_PWRFLAGS_AXI_ON) {
			if (pwr->clk_freq[KGSL_AXI_HIGH])
				pm_qos_update_request(
					pwr->pm_qos_req, PM_QOS_DEFAULT_VALUE);
			if (pwr->pcl)
				msm_bus_scale_client_update_request(pwr->pcl,
								BW_INIT);
			pwr->power_flags &=
				~(KGSL_PWRFLAGS_AXI_ON);
			pwr->power_flags |= KGSL_PWRFLAGS_AXI_OFF;
		}
		return KGSL_SUCCESS;
	case KGSL_PWRFLAGS_AXI_ON:
		if (pwr->power_flags & KGSL_PWRFLAGS_AXI_OFF) {
			if (pwr->clk_freq[KGSL_AXI_HIGH])
				pm_qos_update_request(
					pwr->pm_qos_req,
					pwr->clk_freq[KGSL_AXI_HIGH]);
			if (pwr->pcl)
				msm_bus_scale_client_update_request(pwr->pcl,
								BW_MAX);
			pwr->power_flags &=
				~(KGSL_PWRFLAGS_AXI_OFF);
			pwr->power_flags |= KGSL_PWRFLAGS_AXI_ON;
		}
		return KGSL_SUCCESS;
	default:
		return KGSL_FAILURE;
	}
}


int kgsl_pwrctrl_pwrrail(struct kgsl_device *device, unsigned int pwrflag)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	switch (pwrflag) {
	case KGSL_PWRFLAGS_POWER_OFF:
		if (pwr->power_flags & KGSL_PWRFLAGS_POWER_ON) {
			internal_pwr_rail_ctl(pwr->pwr_rail, KGSL_FALSE);
			if (pwr->gpu_reg)
				regulator_disable(pwr->gpu_reg);
			pwr->power_flags &=
					~(KGSL_PWRFLAGS_POWER_ON);
			pwr->power_flags |=
					KGSL_PWRFLAGS_POWER_OFF;
		}
		return KGSL_SUCCESS;
	case KGSL_PWRFLAGS_POWER_ON:
		if (pwr->power_flags & KGSL_PWRFLAGS_POWER_OFF) {
			internal_pwr_rail_mode(pwr->pwr_rail,
					PWR_RAIL_CTL_MANUAL);
			internal_pwr_rail_ctl(pwr->pwr_rail, KGSL_TRUE);
			if (pwr->gpu_reg)
				regulator_enable(pwr->gpu_reg);
			pwr->power_flags &=
					~(KGSL_PWRFLAGS_POWER_OFF);
			pwr->power_flags |=
					KGSL_PWRFLAGS_POWER_ON;
		}
		return KGSL_SUCCESS;
	default:
		return KGSL_FAILURE;
	}
}


int kgsl_pwrctrl_irq(struct kgsl_device *device, unsigned int pwrflag)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	switch (pwrflag) {
	case KGSL_PWRFLAGS_IRQ_ON:
		if (pwr->power_flags & KGSL_PWRFLAGS_IRQ_OFF) {
			enable_irq(pwr->interrupt_num);
			pwr->power_flags &=
				~(KGSL_PWRFLAGS_IRQ_OFF);
			pwr->power_flags |= KGSL_PWRFLAGS_IRQ_ON;
		}
	return KGSL_SUCCESS;
	case KGSL_PWRFLAGS_IRQ_OFF:
		if (pwr->power_flags & KGSL_PWRFLAGS_IRQ_ON) {
			disable_irq(pwr->interrupt_num);
			pwr->power_flags &=
				~(KGSL_PWRFLAGS_IRQ_ON);
			pwr->power_flags |= KGSL_PWRFLAGS_IRQ_OFF;
		}
		return KGSL_SUCCESS;
	default:
		return KGSL_FAILURE;
	}
}

void kgsl_pwrctrl_close(struct kgsl_device *device)
{
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	if (pwr->interrupt_num > 0) {
		if (pwr->have_irq) {
			free_irq(pwr->interrupt_num, NULL);
			pwr->have_irq = 0;
		}
		pwr->interrupt_num = 0;
	}

	pm_qos_remove_request(pwr->pm_qos_req);

	if (pwr->pcl)
		msm_bus_scale_unregister_client(pwr->pcl);

	pwr->pcl = 0;

	if (pwr->gpu_reg) {
		regulator_put(pwr->gpu_reg);
		pwr->gpu_reg = NULL;
	}

	if (pwr->grp_pclk) {
		clk_put(pwr->grp_pclk);
		pwr->grp_pclk = NULL;
	}

	if (pwr->grp_clk) {
		clk_put(pwr->grp_clk);
		pwr->grp_clk = NULL;
	}

	if (pwr->imem_clk != NULL) {
		clk_put(pwr->imem_clk);
		pwr->imem_clk = NULL;
	}

	pwr->grp_src_clk = NULL;
	pwr->power_flags = 0;
}

void kgsl_idle_check(struct work_struct *work)
{
	struct kgsl_device *device = container_of(work, struct kgsl_device,
							idle_check_ws);

	mutex_lock(&device->mutex);
	if (device->state & (KGSL_STATE_ACTIVE | KGSL_STATE_NAP)) {
		if (device->ftbl.device_sleep(device) == KGSL_FAILURE)
			mod_timer(&device->idle_timer,
					jiffies +
					device->pwrctrl.interval_timeout);
	}
	mutex_unlock(&device->mutex);
}

void kgsl_timer(unsigned long data)
{
	struct kgsl_device *device = (struct kgsl_device *) data;
	device->requested_state = KGSL_STATE_SLEEP;
	/* Have work run in a non-interrupt context. */
	schedule_work(&device->idle_check_ws);
}

void kgsl_pre_hwaccess(struct kgsl_device *device)
{
	while (1) {
		if (device == NULL)
			break;
		if (device->state == KGSL_STATE_ACTIVE)
			break;
		if (device->state != KGSL_STATE_SUSPEND) {
			device->ftbl.device_wake(device);
			break;
		}
		mutex_unlock(&device->mutex);
		wait_for_completion(&device->hwaccess_gate);
		mutex_lock(&device->mutex);
	}
}


