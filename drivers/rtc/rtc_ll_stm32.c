/*
 * Copyright (c) 2023 Syslinbit
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_rtc

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/irq.h>

#include <stm32_ll_rtc.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_exti.h>
#include <stm32_ll_pwr.h>

#include "stm32_hsem.h"

LOG_MODULE_REGISTER(rtc_stm32, CONFIG_RTC_LOG_LEVEL);

#if defined(CONFIG_SOC_SERIES_STM32F1X)
#error This driver does not support STM32F1 yet
#endif

struct rtc_stm32_config {
	const struct stm32_pclken *pclken;
	uint32_t async_prescaler;
	uint32_t sync_prescaler;
};

struct rtc_stm32_data {
	struct k_mutex lock;
};


#if CONFIG_RTC_ALARM || CONFIG_RTC_UPDATE
static void rtc_stm32_isr(const struct device *dev) {
	ARG_UNUSED(dev);

	/* TODO */
}
#endif  /* CONFIG_RTC_ALARM */

static void rtc_stm32_wait_synchro()
{
	LL_RTC_DisableWriteProtection(RTC);

	LL_RTC_ClearFlag_RS(RTC);

	LL_RTC_EnableWriteProtection(RTC);

	while(LL_RTC_IsActiveFlag_RS(RTC) == 1) {
		;
	}
}

static int rtc_stm32_enter_initialization_mode()
{
	LL_RTC_EnableInitMode(RTC);
	bool success = WAIT_FOR(LL_RTC_IsActiveFlag_INIT(RTC), 1000000, k_msleep(1));

	if (!success) {
		return -EIO;
	}

	return 0;
}

static void rtc_stm32_leave_initialization_mode()
{
	LL_RTC_DisableInitMode(RTC);
}

static int rtc_stm32_configure(const struct device *dev)
{
	const struct rtc_stm32_config *config = dev->config;

	int err = 0;

	uint32_t hour_format     = LL_RTC_GetHourFormat(RTC);
	uint32_t sync_prescaler  = LL_RTC_GetSynchPrescaler(RTC);
	uint32_t async_prescaler = LL_RTC_GetAsynchPrescaler(RTC);

	LL_RTC_DisableWriteProtection(RTC);

	/* configuration process requires to stop the RTC counter so
	   do it only if needed to avoid creating time drift at each reset
	   Note: We could have used the INITS flag but it wouldn't have worked
		 if config changes from one reset to another
	 */
	if ((hour_format != LL_RTC_HOURFORMAT_24HOUR) ||
	    (sync_prescaler != config->sync_prescaler) ||
	    (async_prescaler != config->async_prescaler)) {

		err = rtc_stm32_enter_initialization_mode();
		if (err == 0) {
			LL_RTC_SetHourFormat(RTC, LL_RTC_HOURFORMAT_24HOUR);
			LL_RTC_SetSynchPrescaler(RTC, config->sync_prescaler);

			LL_RTC_SetAsynchPrescaler(RTC, config->async_prescaler);

			rtc_stm32_leave_initialization_mode();
		}
	}

#ifdef RTC_CR_BYPSHAD
	LL_RTC_EnableShadowRegBypass(RTC);
#endif /* RTC_CR_BYPSHAD */

	LL_RTC_EnableWriteProtection(RTC);

	return err;
}

#if CONFIG_RTC_ALARM || CONFIG_RTC_UPDATE
static void rtc_stm32_setup_irq()
{
#if defined(CONFIG_SOC_SERIES_STM32H7X) && defined(CONFIG_CPU_CORTEX_M4)
	LL_C2_EXTI_EnableIT_0_31(RTC_EXTI_LINE);
	LL_EXTI_EnableRisingTrig_0_31(RTC_EXTI_LINE);
#elif defined(CONFIG_SOC_SERIES_STM32U5X)
	/* in STM32U5 family RTC is not connected to EXTI */
#else
	LL_EXTI_EnableIT_0_31(RTC_EXTI_LINE);
	LL_EXTI_EnableRisingTrig_0_31(RTC_EXTI_LINE);
#endif

	IRQ_CONNECT(DT_INST_IRQN(0),
		    DT_INST_IRQ(0, priority),
		    rtc_stm32_isr, DEVICE_DT_INST_GET(0), 0);

	irq_enable(DT_INST_IRQN(0));
}
#endif /* CONFIG_RTC_ALARM */

static int rtc_stm32_set_time(const struct device *dev, const struct rtc_time *timeptr)
{
	struct rtc_stm32_data *data = dev->data;
	int err = 0;

	/* The year value stored in tm_year is the number of years elapsed since 1900 */
	uint32_t real_year = timeptr->tm_year + 1900;

	if (real_year < 2000) {
		/* RTC does not support years before 2000 */
		return -EINVAL;
	}

	if (timeptr->tm_wday == -1) {
		/* weekday is expected */
		return -EINVAL;
	}

	/* TODO : verify tm validity ?*/

	err = k_mutex_lock(&data->lock, K_NO_WAIT);
	if (err) {
		return err;
	}

	LL_RTC_DisableWriteProtection(RTC);

	err = rtc_stm32_enter_initialization_mode();
	if (err) {
		k_mutex_unlock(&data->lock);
		return err;
	}

	uint32_t hour = bin2bcd(timeptr->tm_hour);
	uint32_t min  = bin2bcd(timeptr->tm_min);
	uint32_t sec  = bin2bcd(timeptr->tm_sec);

	LL_RTC_TIME_Config(RTC, LL_RTC_TIME_FORMAT_AM_OR_24, hour, min, sec);

	/* RTC expects number of years elapsed since 2000 */
	uint32_t year    = bin2bcd(real_year - 2000);
	uint32_t month   = bin2bcd(timeptr->tm_mon + 1);
	uint32_t day     = bin2bcd(timeptr->tm_mday);
	uint32_t weekday = timeptr->tm_wday;

	if (weekday == 0) {
		weekday = 7;
	}

	LL_RTC_DATE_Config(RTC, weekday, day, month, year);

	LL_RTC_DisableInitMode(RTC);

	LL_RTC_EnableWriteProtection(RTC);

	rtc_stm32_wait_synchro();

	k_mutex_unlock(&data->lock);

	return 0;
}

static int rtc_stm32_get_time(const struct device *dev, struct rtc_time *timeptr)
{
	struct rtc_stm32_data *data = dev->data;
	const struct rtc_stm32_config *config = dev->config;

	uint32_t synch_prescaler = config->sync_prescaler;

	int err = k_mutex_lock(&data->lock, K_NO_WAIT);
	if (err) {
		return err;
	}

	/* read subseconds, time and date registers */
	uint32_t rtc_subseconds = LL_RTC_TIME_GetSubSecond(RTC);
	uint32_t rtc_time = LL_RTC_TIME_Get(RTC);
	uint32_t rtc_date = LL_RTC_DATE_Get(RTC);

	/* The year value stored in RTC is the number of years elapsed since 2000 */
	uint32_t real_year = bcd2bin(__LL_RTC_GET_YEAR(rtc_date)) + 2000;

	/* The year value expected in tm_year is the number of years elapsed since 1900 */
	timeptr->tm_year = real_year - 1900;
	timeptr->tm_mon  = bcd2bin(__LL_RTC_GET_MONTH(rtc_date)) - 1;
	timeptr->tm_mday = bcd2bin(__LL_RTC_GET_DAY(rtc_date));
	timeptr->tm_wday = __LL_RTC_GET_WEEKDAY(rtc_date);
	if (timeptr->tm_wday == 7) {
		timeptr->tm_wday = 0;
	}

	timeptr->tm_hour = bcd2bin(__LL_RTC_GET_HOUR(rtc_time));
	timeptr->tm_min  = bcd2bin(__LL_RTC_GET_MINUTE(rtc_time));
	timeptr->tm_sec  = bcd2bin(__LL_RTC_GET_SECOND(rtc_time));

	uint64_t nanosec = ((uint64_t)(synch_prescaler - rtc_subseconds)) * 1000000000;
	nanosec += ((synch_prescaler + 1) / 2); /* just to ensure rounding is done correctly */
	nanosec /= synch_prescaler + 1;
	timeptr->tm_nsec = (int32_t) nanosec;

	/* unknown values */
	timeptr->tm_yday = -1;
	timeptr->tm_isdst = -1;

	k_mutex_unlock(&data->lock);

	return 0;
}

#ifdef CONFIG_RTC_ALARM
static int  rtc_stm32_alarm_get_supported_fields(const struct device *dev, uint16_t id,
						uint16_t *mask)
{
	/* TODO */
	return -ENOTSUP;
}

static int rtc_stm32_alarm_set_time(const struct device *dev, uint16_t id, uint16_t mask,
				   const struct rtc_time *timeptr)
{
	/* TODO */
	return -ENOTSUP;
}

static int rtc_stm32_alarm_get_time(const struct device *dev, uint16_t id, uint16_t *mask,
				   struct rtc_time *timeptr)
{
	/* TODO */
	return -ENOTSUP;
}

static int rtc_stm32_alarm_is_pending(const struct device *dev, uint16_t id)
{

	/* TODO */
	return -ENOTSUP;
}

static int rtc_stm32_alarm_set_callback(const struct device *dev, uint16_t id,
				       rtc_alarm_callback callback, void *user_data)
{
	/* TODO */
	return -ENOTSUP;
}
#endif /* CONFIG_RTC_ALARM */

#ifdef CONFIG_RTC_UPDATE
static int rtc_stm32_update_set_callback(const struct device *dev,
					    rtc_update_callback callback, void *user_data)
{
	/* TODO */
	return -ENOTSUP;
}
#endif /* CONFIG_RTC_UPDATE */

#ifdef CONFIG_RTC_CALIBRATION
static int rtc_stm32_set_calibration(const struct device *dev, int32_t calibration)
{
	ARG_UNUSED(dev);

	/* Note : calibration is considered to be ppb value
	   to apply on period but with opposite sign */

	if ((calibration > 1000000000 / (0x1 << 11)) ||
	    (calibration < - ((int64_t) 511 * 1000000000L) / (0x1 << 20))) {
		/* out of supported range */
		return -EINVAL;
	}

	/*
	   calp : whether or not a clock pulse must be inserted every 2^11 clock cycles
	   calm : number of clock pulse to ignore each 2^20 clock cycles (max: 511)
	*/

	uint32_t calp = LL_RTC_CALIB_INSERTPULSE_NONE;
	uint64_t calm = 0;

	if (calibration > 0) {
		calp = LL_RTC_CALIB_INSERTPULSE_SET;
		calibration -= 1000000000 / (0x1 << 11);
	} else {
		calp = LL_RTC_CALIB_INSERTPULSE_NONE;
	}

	__ASSERT_NO_MSG(calibration <= 0);

	/* calm = -calibration * 10^9 / 2^20 */
	calm = -calibration * (0x1L << 20);
	calm += 500000000; /* just to get proper rounded value */
	calm /= 1000000000;

	__ASSERT_NO_MSG(calm < 512);

	bool ok_to_proceed = WAIT_FOR(LL_RTC_IsActiveFlag_RECALP(RTC) == 0, 1000, k_usleep(100));
	if (!ok_to_proceed) {
		return -EIO;
	}

	LL_RTC_DisableWriteProtection(RTC);

	LL_RTC_CAL_SetPulse(RTC, calp);
	LL_RTC_CAL_SetMinus(RTC, (uint32_t) calm);

	LL_RTC_EnableWriteProtection(RTC);

	return 0;
}

static int rtc_stm32_get_calibration(const struct device *dev, int32_t *calibration)
{
	ARG_UNUSED(dev);

	/*
	   calp : whether or not a clock pulse is inserted every 2^11 clock cycles
	   calm : number of clock pulse ignored each 2^20 clock cycles
	*/

	uint32_t calp = LL_RTC_CAL_IsPulseInserted(RTC);
	uint32_t calm = LL_RTC_CAL_GetMinus(RTC);

	*calibration = (512 * (int32_t) calp - (int32_t) calm) * 1000000000L / (0x1 << 20);

	return 0;
}
#endif /* CONFIG_RTC_CALIBRATION */

static int rtc_stm32_init(const struct device *dev)
{
	struct rtc_stm32_data *data = dev->data;
	const struct rtc_stm32_config *config = dev->config;

	const struct device *const clock = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	if (!device_is_ready(clock)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	/* Enable RTC bus clock */
	if (clock_control_on(clock, (clock_control_subsys_t) &config->pclken[0]) != 0) {
		LOG_ERR("failed to enable clock");
		return -EIO;
	}

	k_mutex_init(&data->lock);

	/* Enable Backup access */
	z_stm32_hsem_lock(CFG_HW_RCC_SEMID, HSEM_LOCK_DEFAULT_RETRY);
#if defined(PWR_CR_DBP) || defined(PWR_CR1_DBP) || \
	defined(PWR_DBPCR_DBP) || defined(PWR_DBPR_DBP)
	LL_PWR_EnableBkUpAccess();
#endif /* PWR_CR_DBP || PWR_CR1_DBP || PWR_DBPR_DBP */

	/* Enable RTC clock source */
	if (clock_control_configure(clock,
				    (clock_control_subsys_t) &config->pclken[1],
				    NULL) != 0) {
		LOG_ERR("failed to configure clock");
		return -EIO;
	}

	LL_RCC_EnableRTC();

	z_stm32_hsem_unlock(CFG_HW_RCC_SEMID);

	int err = rtc_stm32_configure(dev);
	if (err) {
		LOG_ERR("failed to configure RTC");
		return err;
	}

#if CONFIG_RTC_ALARM || CONFIG_RTC_UPDATE
	rtc_stm32_setup_irq();
#endif

	/* Wait for the first time and date registers update to avoid reading incorrect values */
	rtc_stm32_wait_synchro();

	return 0;
}

#if DT_INST_NUM_CLOCKS(0) != 2
#error wrong RTC clock configuration in device tree
#else
const struct stm32_pclken rtc_stm32_pclken[] = STM32_DT_INST_CLOCKS(0);
#endif

const struct rtc_stm32_config rtc_config = {
	.pclken = rtc_stm32_pclken,
#if defined(CONFIG_SOC_SERIES_STM32F1X)
#if DT_INST_CLOCKS_CELL(1, bus) == STM32_SRC_LSI
	/* prescaler values for LSI @ 40 KHz */
	.async_prescaler = 0x9C3F,
#else /* DT_INST_CLOCKS_CELL(1, bus) == STM32_SRC_LSE */
	/* prescaler values for LSE @ 32768 Hz */
	.async_prescaler = 0x7FFF,
#endif /* DT_INST_CLOCKS_CELL(1, bus) == STM32_SRC_LSE */
#else
#if DT_INST_CLOCKS_CELL_BY_IDX(0, 1, bus) == STM32_SRC_LSI
	/* prescaler values for LSI @ 32 KHz */
	.async_prescaler = 0x7F,
	.sync_prescaler = 0x00F9,
#else /* DT_INST_CLOCKS_CELL(1, bus) == STM32_SRC_LSE */
	/* prescaler values for LSE @ 32768 Hz */
	.async_prescaler = 0x7F,
	.sync_prescaler = 0x00FF,
#endif
#endif
};

static struct rtc_stm32_data rtc_data;

struct rtc_driver_api rtc_stm32_driver_api = {
	.set_time = rtc_stm32_set_time,
	.get_time = rtc_stm32_get_time,
#ifdef CONFIG_RTC_ALARM
	.alarm_get_supported_fields = rtc_stm32_alarm_get_supported_fields,
	.alarm_set_time = rtc_stm32_alarm_set_time,
	.alarm_get_time = rtc_stm32_alarm_get_time,
	.alarm_is_pending = rtc_stm32_alarm_is_pending,
	.alarm_set_callback = rtc_stm32_alarm_set_callback,
#endif /* CONFIG_RTC_ALARM */
#ifdef CONFIG_RTC_UPDATE
	.update_set_callback = rtc_stm32_update_set_callback,
#endif /* CONFIG_RTC_UPDATE */
#ifdef CONFIG_RTC_CALIBRATION
	.set_calibration = rtc_stm32_set_calibration,
	.get_calibration = rtc_stm32_get_calibration,
#endif /* CONFIG_RTC_CALIBRATION */
};

DEVICE_DT_INST_DEFINE(0, &rtc_stm32_init, NULL, &rtc_data, &rtc_config,
		      POST_KERNEL, CONFIG_RTC_INIT_PRIORITY, &rtc_stm32_driver_api);
