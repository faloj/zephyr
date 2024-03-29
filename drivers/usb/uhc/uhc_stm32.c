/*
 * Copyright (c) 2023 Syslinbit
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "uhc_common.h"

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>

#include <soc.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_pwr.h>

#include <string.h>
#include <stdint.h>
#include <stddef.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uhc_stm32, CONFIG_UHC_DRIVER_LOG_LEVEL);

#define LOCAL_LOG_DBG(msg, ...) LOG_DBG(msg __VA_OPT__(,) __VA_ARGS__)

#define DT_CLOCKS_PROP_MAX_LEN (2)

#define SETUP_PACKET_SIZE (8U)

#define DEFAULT_EP0_MPS (64U)
#define DEFAULT_ADDR    ( 0U)

#define NB_MAX_PIPE (16U)

BUILD_ASSERT(NB_MAX_PIPE <= UINT8_MAX);

#define UHC_STM32_MAX_ERR (3U)

enum usb_speed {
	USB_SPEED_INVALID = -1,
	/* Values are fixed by maximum-speed enum values order in
	 * usb-controller.yaml dts binding file
	 */
	USB_SPEED_LOW     =  0,
	USB_SPEED_FULL    =  1,
	USB_SPEED_HIGH    =  2,
	USB_SPEED_SUPER   =  3,
};

enum phy_type {
	PHY_INVALID     = -1,
	PHY_EMBEDDED_FS =  0,
#if defined(USB_OTG_HS_EMBEDDED_PHY)
	PHY_EMBEDDED_HS,
#endif
	PHY_EXTERNAL_ULPI,
};

enum uhc_state {
	STM32_UHC_STATE_SPEED_ENUM,
	STM32_UHC_STATE_DISCONNECTED,
	STM32_UHC_STATE_READY,
};

struct uhc_stm32_data {
	enum uhc_state state;
	const struct device * dev;
	bool busy_pipe[NB_MAX_PIPE];
	uint8_t control_pipe;
	HCD_HandleTypeDef hcd;
	struct uhc_transfer *ongoing_xfer;
	struct net_buf_simple_state ongoing_xfer_buf_save;
	size_t ongoing_xfer_err_cpt;
	struct k_work_q work_queue;
	struct k_work on_connect_work;
	struct k_work on_disconnect_work;
	struct k_work on_reset_work;
	struct k_work on_xfer_update_work;
	struct k_work on_new_xfer_work;
	struct k_work_delayable delayed_reset_work;
};

struct uhc_stm32_config {
	uint32_t irq;
	enum phy_type phy;
	size_t num_host_channels;
	struct stm32_pclken clocks[DT_CLOCKS_PROP_MAX_LEN];
	size_t num_clock;
	enum usb_speed max_speed;
	const struct pinctrl_dev_config *pcfg;
	struct gpio_dt_spec ulpi_reset_gpio;
	struct gpio_dt_spec vbus_enable_gpio;
};

#if defined(USB_DRD_FS)
#define SPEED_FROM_DRD_CORE_ST_SPEED_DEF(drd_core_speed) (     \
	(drd_core_speed == USB_DRD_SPEED_LS) ? USB_SPEED_FULL : (  \
	(drd_core_speed == USB_DRD_SPEED_FS) ? USB_SPEED_HIGH : (  \
	USB_SPEED_INVALID))                                        \
)
#endif

#if defined(USB_OTG_FS) || defined(USB_OTG_HS)
#define SPEED_FROM_OTG_CORE_ST_SPEED_DEF(otg_core_speed) (            \
	(otg_core_speed == HPRT0_PRTSPD_LOW_SPEED) ? USB_SPEED_FULL : (   \
	(otg_core_speed == HPRT0_PRTSPD_FULL_SPEED) ? USB_SPEED_HIGH : (  \
	(otg_core_speed == HPRT0_PRTSPD_HIGH_SPEED) ? USB_SPEED_HIGH : (  \
	USB_SPEED_INVALID)))                                              \
)
#endif


static inline enum usb_speed priv_get_current_speed(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	uint32_t hal_speed = HAL_HCD_GetCurrentSpeed(&(priv->hcd));

	enum usb_speed speed = USB_SPEED_INVALID;

#if defined(USB_DRD_FS)
	speed = SPEED_FROM_DRD_CORE_ST_SPEED_DEF(hal_speed);
#endif
#if defined(USB_OTG_FS) || defined(USB_OTG_HS)
	speed = SPEED_FROM_OTG_CORE_ST_SPEED_DEF(hal_speed);
#endif

	if (speed == USB_SPEED_INVALID) {
		LOG_DBG("Invalid USB speed returned by \"HAL_HCD_GetCurrentSpeed\" (%d)", hal_speed);
		__ASSERT_NO_MSG(0);

		/* Falling back to low speed */
		speed = USB_SPEED_LOW;
	}

	return speed;
}

static void uhc_stm32_irq(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	HAL_HCD_IRQHandler((HCD_HandleTypeDef *)&priv->hcd);

	/* TODO: check WKUPINT to trigger UHC_EVT_RWUP */
}

void HAL_HCD_SOF_Callback(HCD_HandleTypeDef *hhcd)
{
	ARG_UNUSED(hhcd);

	//const struct device *dev = (const struct device *)hhcd->pData;
	//struct uhc_stm32_data *priv = uhc_get_private(dev);
}

void HAL_HCD_Connect_Callback(HCD_HandleTypeDef *hhcd)
{
	const struct device *dev = (const struct device *)hhcd->pData;
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	k_work_submit_to_queue(&priv->work_queue, &priv->on_connect_work);
}

void HAL_HCD_Disconnect_Callback(HCD_HandleTypeDef *hhcd)
{
	const struct device *dev = (const struct device *)hhcd->pData;
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	k_work_submit_to_queue(&priv->work_queue, &priv->on_disconnect_work);
}

void HAL_HCD_PortEnabled_Callback(HCD_HandleTypeDef *hhcd)
{
	const struct device *dev = (const struct device *)hhcd->pData;
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	k_work_submit_to_queue(&priv->work_queue, &priv->on_reset_work);
}

void HAL_HCD_PortDisabled_Callback(HCD_HandleTypeDef *hhcd)
{
	ARG_UNUSED(hhcd);

	//const struct device *dev = (const struct device *)hhcd->pData;
	//struct uhc_stm32_data *priv = uhc_get_private(dev);
}

void HAL_HCD_HC_NotifyURBChange_Callback(HCD_HandleTypeDef *hhcd, uint8_t chnum,
					 HCD_URBStateTypeDef urb_state)
{
	const struct device *dev = (const struct device *)hhcd->pData;
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	k_work_submit_to_queue(&priv->work_queue, &priv->on_xfer_update_work);
}

static inline void priv_hcd_prepare(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	const struct uhc_stm32_config *config = dev->config;

	HCD_TypeDef *instance = priv->hcd.Instance;
	memset(&priv->hcd, 0, sizeof(priv->hcd));
	priv->hcd.Instance = instance;

	priv->hcd.Init.Host_channels = config->num_host_channels;
	priv->hcd.Init.dma_enable = 0;
	priv->hcd.Init.battery_charging_enable = DISABLE;
	priv->hcd.Init.use_external_vbus = DISABLE;
	priv->hcd.Init.vbus_sensing_enable = DISABLE;

	/* use pData to pass a reference to the device handler
	 * to be able to retrieve it from the HAL callbacks
	 */
	priv->hcd.pData = (void *)dev;

#if defined(USB_DRD_FS)
// TODO: verify if there is possibility of multiple instances
	if (priv->hcd.Instance == USB_DRD_FS) {
		priv->hcd.Init.speed = HCD_SPEED_FULL;
	}
#endif

#if defined(USB_OTG_HS)
// TODO: verify if there is possibility of multiple instances
	if (priv->hcd.Instance == USB_OTG_HS) {
		switch (config->phy) {
			case PHY_EMBEDDED_FS: {
				priv->hcd.Init.phy_itface = HCD_PHY_EMBEDDED;
			} break;
#if defined(USB_OTG_HS_EMBEDDED_PHY)
			case PHY_EMBEDDED_HS: {
				priv->hcd.Init.phy_itface = USB_OTG_HS_EMBEDDED_PHY;
			} break;
#endif
			case PHY_EXTERNAL_ULPI: {
				priv->hcd.Init.phy_itface = USB_OTG_ULPI_PHY;
				priv->hcd.Init.use_external_vbus = ENABLE;
			} break;
			case PHY_INVALID: {
				__ASSERT_NO_MSG(0);
			} break;
		}
		if (config->max_speed >= USB_SPEED_HIGH) {
			priv->hcd.Init.speed = HCD_SPEED_HIGH;
		} else {
			priv->hcd.Init.speed = HCD_SPEED_FULL;
		}
	}
#endif

#if defined(USB_OTG_FS)
// TODO: verify if there is possibility of multiple instances
	if (priv->hcd.Instance == USB_OTG_FS) {
		if (config->phy != PHY_EMBEDDED_FS) {
			LOG_DBG("Unsupported PHY."
				"USB controller will default to embedded full-speed only PHY.");
		}
		priv->hcd.Init.phy_itface = HCD_PHY_EMBEDDED;
		priv->hcd.Init.speed = HCD_SPEED_FULL;
	}
#endif
}

static int uhc_stm32_lock(const struct device *dev)
{
	return uhc_lock_internal(dev, K_FOREVER);
}

static int uhc_stm32_unlock(const struct device *dev)
{
	return uhc_unlock_internal(dev);
}

/* TODO : Factorize if possible. priv_clock_enable and priv_clock_disable are just a slightly
 * modified version taken from udc_stm32.c (under Copyright (c) 2023 Linaro Limited)
 */
/***********************************************************************/
static int priv_clock_enable(const struct device *dev)
{
	const struct uhc_stm32_config *config = dev->config;
	const struct device *const clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	if (!device_is_ready(clk)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	if (config->num_clock > 1) {
		if (clock_control_configure(clk, (clock_control_subsys_t *)&config->clocks[1],
					    NULL) != 0) {
			LOG_ERR("Could not select USB domain clock");
			return -EIO;
		}
	}

	if (clock_control_on(clk, (clock_control_subsys_t *)&config->clocks[0]) != 0) {
		LOG_ERR("Unable to enable USB clock");
		return -EIO;
	}

	uint32_t usb_clock_rate;

	if (clock_control_get_rate(clk, (clock_control_subsys_t *)&config->clocks[1],
				   &usb_clock_rate) != 0) {
		LOG_ERR("Failed to get USB domain clock rate");
		return -EIO;
	}

	if (usb_clock_rate != MHZ(48)) {
		LOG_ERR("USB Clock is not 48MHz (%d)", usb_clock_rate);
		return -ENOTSUP;
	}

/* Previous check won't work in case of F1. Add build time check */
#if defined(RCC_CFGR_OTGFSPRE) || defined(RCC_CFGR_USBPRE)

#if (MHZ(48) == CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC) && !defined(STM32_PLL_USBPRE)
/* PLL output clock is set to 48MHz, it should not be divided */
#warning USBPRE/OTGFSPRE should be set in rcc node
#endif

#endif /* RCC_CFGR_OTGFSPRE / RCC_CFGR_USBPRE */

	return 0;
}

static int priv_clock_disable(const struct device *dev)
{
	const struct uhc_stm32_config *config = dev->config;
	const struct device *clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	if (clock_control_off(clk, (clock_control_subsys_t *)&config->clocks[0]) != 0) {
		LOG_ERR("Unable to disable USB clock");
		return -EIO;
	}

	return 0;
}
/**********************************************************************/

static int uhc_stm32_init(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	const struct uhc_stm32_config *config = dev->config;

	int err = 0;

	err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		LOG_ERR("USB pinctrl setup failed (%d)", err);
		return err;
	}

	if (config->vbus_enable_gpio.port) {
		if (!gpio_is_ready_dt(&config->vbus_enable_gpio)) {
			LOG_ERR("vbus enable gpio not ready");
			return -ENODEV;
		}
		err = gpio_pin_configure_dt(&config->vbus_enable_gpio, GPIO_OUTPUT_INACTIVE);
		if (err) {
			LOG_ERR("Failed to configure vbus enable gpio (%d)", err);
			return err;
		}
	}

	if (config->phy == PHY_EXTERNAL_ULPI && config->ulpi_reset_gpio.port) {
		if (!gpio_is_ready_dt(&config->ulpi_reset_gpio)) {
			LOG_ERR("ULPI reset gpio is not ready");
			return -ENODEV;
		}
		err = gpio_pin_configure_dt(&config->ulpi_reset_gpio, GPIO_OUTPUT_INACTIVE);
		if (err) {
			LOG_ERR("Failed to configure ULPI reset gpio (%d)", err);
			return err;
		}
	}

#if defined(CONFIG_SOC_SERIES_STM32U5X)
	/* TODO */
#elif defined(CONFIG_SOC_SERIES_STM32H5X)
	/* TODO : test if it works */
	LL_PWR_EnableUSBVoltageDetector();
	while (LL_PWR_IsActiveFlag_VDDUSB() == 0) {
		;
	}
#elif defined(CONFIG_SOC_SERIES_STM32H7X)
	/* TODO: useless with an ULPI phy ? */
	LL_PWR_EnableUSBVoltageDetector();
	while (LL_PWR_IsActiveFlag_USB() == 0) {
		;
	}
#endif

	err = priv_clock_enable(dev);
	if (err != 0) {
		LOG_ERR("Error enabling clock(s)");
		return -EIO;
	}

#if defined(CONFIG_SOC_SERIES_STM32H7X)
	// TODO: some devices have two USB : handle this case
	LL_AHB1_GRP1_EnableClockSleep(LL_AHB1_GRP1_PERIPH_USB1OTGHS);
	if (config->phy == PHY_EXTERNAL_ULPI) {
		// TODO: same here
		LL_AHB1_GRP1_EnableClockSleep(LL_AHB1_GRP1_PERIPH_USB1OTGHSULPI);
	} else {
		// TODO: same here
		LL_AHB1_GRP1_DisableClockSleep(LL_AHB1_GRP1_PERIPH_USB1OTGHSULPI);
	}
#endif

	priv_hcd_prepare(dev);

	HAL_StatusTypeDef status = HAL_HCD_Init(&(priv->hcd));

	if (status != HAL_OK) {
		LOG_ERR("HCD_Init failed, %d", (int)status);
		return -EIO;
	}

	return 0;
}

/**
  * @brief  Initialize a host channel.
  * @param  hhcd HCD handle
  * @param  ch_num Channel number.
  *         This parameter can be a value from 1 to 15
  * @param  epnum Endpoint number.
  *          This parameter can be a value from 1 to 15
  * @param  dev_address Current device address
  *          This parameter can be a value from 0 to 255
  * @param  speed Current device speed.
  *          This parameter can be one of these values:
  *            HCD_DEVICE_SPEED_HIGH: High speed mode,
  *            HCD_DEVICE_SPEED_FULL: Full speed mode,
  *            HCD_DEVICE_SPEED_LOW: Low speed mode
  * @param  ep_type Endpoint Type.
  *          This parameter can be one of these values:
  *            EP_TYPE_CTRL: Control type,
  *            EP_TYPE_ISOC: Isochronous type,
  *            EP_TYPE_BULK: Bulk type,
  *            EP_TYPE_INTR: Interrupt type
  * @param  mps Max Packet Size.
  *          This parameter can be a value from 0 to32K
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_HCD_HC_Init(HCD_HandleTypeDef *hhcd, uint8_t ch_num, uint8_t epnum,
                                  uint8_t dev_address, uint8_t speed, uint8_t ep_type, uint16_t mps);

static int uhc_stm32_open_pipe(const struct device *dev, uint8_t *pipe_id, uint8_t ep,
							   uint8_t dev_addr, uint8_t speed, uint8_t ep_type, uint16_t mps)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	const struct uhc_stm32_config *config = dev->config;

	bool found = false;

	size_t i;
	for(i = 0; i < config->num_host_channels; i++) {
		if (priv->busy_pipe[i] == false) {
			*pipe_id = i;
			found = true;
			break;
		}
	}

	if (found == false) {
		return -ENODEV;
	}

	HAL_StatusTypeDef status = HAL_HCD_HC_Init(&(priv->hcd),
		*pipe_id, USB_EP_GET_IDX(ep),
        dev_addr, speed,
		ep_type, mps
	);
	if (status != HAL_OK) {
		return -EIO;
	}

	priv->busy_pipe[i] = true;
	return 0;
}

static int uhc_stm32_close_pipe(const struct device *dev, uint8_t pipe_id)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	const struct uhc_stm32_config *config = dev->config;

	if (pipe_id > config->num_host_channels) {
		return -EINVAL;
	}

	if (priv->busy_pipe[pipe_id] != true) {
		return -EALREADY;
	}

	HAL_StatusTypeDef status = HAL_HCD_HC_Halt(&(priv->hcd), pipe_id);
	if (status != HAL_OK) {
		return -EIO;
	}

	priv->busy_pipe[pipe_id] = false;

	return 0;
}

static inline int uhc_stm32_retreive_pipe_id(const struct device *dev,
												const uint8_t ep,
												const uint8_t addr,
												uint8_t * const pipe_id)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	const struct uhc_stm32_config *config = dev->config;

	bool found = false;

	size_t i;
	for(i = 0; i < config->num_host_channels; i++) {
		if (priv->busy_pipe[i] == true) {
			if ((priv->hcd.hc[i].ep_num == USB_EP_GET_IDX(ep)) &&
				(priv->hcd.hc[i].dev_addr == addr)) {
				*pipe_id = i;
				found = true;
				break;
			}
		}
	}

	if (!found) {
		return -ENODEV;
	}

	return 0;
}

static inline void uhc_stm32_init_pipes(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	const struct uhc_stm32_config *config = dev->config;

	size_t i;
	for(i = 0; i < config->num_host_channels; i++) {
		priv->busy_pipe[i] = false;
	}
}

static inline void uhc_stm32_close_all_pipes(const struct device *dev)
{
	const struct uhc_stm32_config *config = dev->config;

	size_t i;
	for(i = 0; i < config->num_host_channels; i++) {
		uhc_stm32_close_pipe(dev, i);
	}
}

static inline void uhc_stm32_deinit_pipes(const struct device *dev)
{
	return uhc_stm32_close_all_pipes(dev);
}

static int uhc_stm32_enable(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	const struct uhc_stm32_config *config = dev->config;

	irq_enable(config->irq);

	HAL_StatusTypeDef status = HAL_HCD_Start(&priv->hcd);

	if (status != HAL_OK) {
		LOG_ERR("HCD_Start failed (%d)", (int)status);
		return -EIO;
	}

	if (config->vbus_enable_gpio.port) {
		int err = gpio_pin_set_dt(&config->vbus_enable_gpio, 1);

		if (err) {
			LOG_ERR("Failed to enable vbus power (%d)", err);
			return err;
		}
	}

	uhc_stm32_init_pipes(dev);

	return 0;
}

static int uhc_stm32_disable(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	const struct uhc_stm32_config *config = dev->config;

	uhc_stm32_deinit_pipes(dev);

	if (config->vbus_enable_gpio.port) {
		int err = gpio_pin_set_dt(&config->vbus_enable_gpio, 0);

		if (err) {
			LOG_ERR("Failed to disable vbus power (%d)", err);
			return err;
		}
	}

	HAL_StatusTypeDef status = HAL_HCD_Stop(&priv->hcd);

	if (status != HAL_OK) {
		LOG_ERR("HCD_Stop failed (%d)", (int)status);
		return -EIO;
	}

	irq_disable(config->irq);

	return 0;
}

static int uhc_stm32_shutdown(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	const struct uhc_stm32_config *config = dev->config;

	HAL_StatusTypeDef status = HAL_HCD_DeInit(&priv->hcd);

	if (status != HAL_OK) {
		LOG_ERR("HAL_HCD_DeInit failed (%d)", (int)status);
		return -EIO;
	}

	int err = priv_clock_disable(dev);

	if (err) {
		LOG_ERR("Failed to disable USB clock (%d)", err);
		return err;
	}

	if (irq_is_enabled(config->irq)) {
		irq_disable(config->irq);
	}

	return 0;
}

// TODO : ensure atomicity of operations

static int uhc_stm32_bus_reset(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	HAL_HCD_ResetPort(&priv->hcd);

	return 0;
}

static int uhc_stm32_sof_enable(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);

  	uint32_t USBx_BASE = (uint32_t)priv->hcd.Instance;

	if (!READ_BIT(USBx_HPRT0, USB_OTG_HPRT_PSUSP) &&
	    !READ_BIT(USBx_HPRT0, USB_OTG_HPRT_PRES)) {
		return 0;
		return -EALREADY;
	}

	/* TODO */
	return 0;
}

static int uhc_stm32_bus_suspend(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);

  	uint32_t USBx_BASE = (uint32_t)priv->hcd.Instance;

	if (READ_BIT(USBx_HPRT0, USB_OTG_HPRT_PSUSP)) {
		return -EALREADY;
	}

	SET_BIT(USBx_HPRT0, USB_OTG_HPRT_PSUSP);

	uhc_submit_event(dev, UHC_EVT_SUSPENDED, 0);

	return 0;
}

static int uhc_stm32_bus_resume(const struct device *dev)
{
	// TODO : handle WKUPIN into IRQ handler
	struct uhc_stm32_data *priv = uhc_get_private(dev);

  	uint32_t USBx_BASE = (uint32_t)priv->hcd.Instance;

	if (READ_BIT(USBx_HPRT0, USB_OTG_HPRT_PRES)) {
		return -EBUSY;
	}

	SET_BIT(USBx_HPRT0, USB_OTG_HPRT_PRES);

	// USB specs says resume must be driven at least 20ms
	k_msleep(20 + 1);

	CLEAR_BIT(USBx_HPRT0, USB_OTG_HPRT_PRES);

	uhc_submit_event(dev, UHC_EVT_RESUMED, 0);

	return 0;
}

static int uhc_stm32_ep_enqueue(const struct device *dev, struct uhc_transfer *const xfer)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	xfer->stage = UHC_CONTROL_STAGE_SETUP;

	int err = uhc_xfer_append(dev, xfer);
	if (err) {
		return err;
	}

	k_work_submit_to_queue(&priv->work_queue, &priv->on_new_xfer_work);

	return 0;
}

static int uhc_stm32_ep_dequeue(const struct device *dev, struct uhc_transfer *const xfer)
{
	/* TODO */
	return 0;
}

/** TODO: remove/transform
  * @brief  Submit a new URB for processing.
  * @param  hhcd HCD handle
  * @param  ch_num Channel number.
  *         This parameter can be a value from 1 to 15
  * @param  direction Channel number.
  *          This parameter can be one of these values:
  *           0 : Output / 1 : Input
  * @param  ep_type Endpoint Type.
  *          This parameter can be one of these values:
  *            EP_TYPE_CTRL: Control type/
  *            EP_TYPE_ISOC: Isochronous type/
  *            EP_TYPE_BULK: Bulk type/
  *            EP_TYPE_INTR: Interrupt type/
  * @param  token Endpoint Type.
  *          This parameter can be one of these values:
  *            0: HC_PID_SETUP / 1: HC_PID_DATA1
  * @param  pbuff pointer to URB data
  * @param  length Length of URB data
  * @param  do_ping activate do ping protocol (for high speed only).
  *          This parameter can be one of these values:
  *           0 : do ping inactive / 1 : do ping active
  * @retval HAL status
  */
static inline int uhc_stm32_submit_request(const struct device *dev,
										   uint8_t chan_num,
										   uint8_t direction,
										   uint8_t ep_type,
										   uint8_t token,
										   uint8_t *buf,
										   uint16_t length)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	HAL_StatusTypeDef status = HAL_HCD_HC_SubmitRequest(&priv->hcd,
		chan_num, direction, ep_type, token, buf, length, 0
	);

	if (status == HAL_OK) {
		return 0;
	} else {
		return -ECANCELED;
	}
}

static inline int uhc_stm32_send_control_setup(const struct device *dev,
											   const uint8_t chan_num,
											   uint8_t *buf,
											   uint16_t length)
{
	if (length != SETUP_PACKET_SIZE) {
		return -EINVAL;
	}

	return uhc_stm32_submit_request(dev, chan_num, 0, EP_TYPE_CTRL, 0, buf, length);
}

static int uhc_stm32_send_data(const struct device *dev,
							   const uint8_t chan_num,
							   struct net_buf *const buf,
							   const uint8_t ep_type,
							   const uint8_t maximum_packet_size)
{
	size_t tx_size = MIN(buf->len, maximum_packet_size);

	int err = uhc_stm32_submit_request(dev, chan_num, 0, ep_type, 1, buf->data, tx_size);
	if (err) {
		return err;
	}

	net_buf_pull(buf, tx_size);

	return 0;
}

static int uhc_stm32_receive_data(const struct device *dev,
								  const uint8_t chan_num,
								  struct net_buf *const buf,
								  const uint8_t ep_type,
								  const uint8_t maximum_packet_size)
{
	size_t rx_size = MIN(net_buf_tailroom(buf), maximum_packet_size);
	LOCAL_LOG_DBG("rx_size = %d", rx_size);
	if (rx_size == 0) {
		return -ENOMEM;
	}

	void* buffer_tail = net_buf_add(buf, rx_size);

	int err = uhc_stm32_submit_request(dev, chan_num, 1, ep_type, 1, buffer_tail, rx_size);
	if (err) {
		net_buf_remove_mem(buf, rx_size);
		return err;
	}

	return 0;
}

static int uhc_stm32_send_control_status(const struct device *dev, const uint8_t chan_num)
{
	return uhc_stm32_submit_request(dev, chan_num, 0, EP_TYPE_CTRL, 1, NULL, 0);
}

static int uhc_stm32_receive_control_status(const struct device *dev, const uint8_t chan_num)
{
	return uhc_stm32_submit_request(dev, chan_num, 1, EP_TYPE_CTRL, 1, NULL, 0);
}

static int uhc_stm32_xfer_control(const struct device *dev, struct uhc_transfer *const xfer)
{
	uint8_t pipe_id = 0;
	int err = uhc_stm32_retreive_pipe_id(dev, xfer->ep, xfer->addr, &pipe_id);
	if (err) {
		// No opened pipe found for this endpoint at this device address
		return err;
	}

	if (xfer->stage == UHC_CONTROL_STAGE_SETUP) {
		LOCAL_LOG_DBG("Handle SETUP stage");
		err = uhc_stm32_send_control_setup(dev, pipe_id, xfer->setup_pkt, sizeof(xfer->setup_pkt));
		LOCAL_LOG_DBG("SETUP stage DONE ! %d", err);
		return err;
	}

	if (xfer->buf != NULL && xfer->stage == UHC_CONTROL_STAGE_DATA) {
		if (USB_EP_DIR_IS_IN(xfer->ep)) {
			LOCAL_LOG_DBG("Handle DATA stage: receive");
			err = uhc_stm32_receive_data(dev, pipe_id, xfer->buf, USB_EP_TYPE_CONTROL, xfer->mps);
			LOCAL_LOG_DBG("rx stage err = %d", err);
			return err;
		} else {
			LOCAL_LOG_DBG("Handle DATA stage: send");
			return uhc_stm32_send_data(dev, pipe_id, xfer->buf, USB_EP_TYPE_CONTROL, xfer->mps);
		}
	}

	if (xfer->stage == UHC_CONTROL_STAGE_STATUS) {
		if (USB_EP_DIR_IS_IN(xfer->ep)) {
			LOCAL_LOG_DBG("Handle STATUS stage: send");
			err = uhc_stm32_send_control_status(dev, pipe_id);
			LOCAL_LOG_DBG("status stage err = %d", err);
			return err;
		} else {
			LOCAL_LOG_DBG("Handle STATUS stage: receive");
			return uhc_stm32_receive_control_status(dev, pipe_id);
		}
	}

	return -EINVAL;
}

static int uhc_stm32_xfer_bulk(const struct device *dev, struct uhc_transfer *const xfer)
{
	//struct uhc_stm32_data *priv = uhc_get_private(dev);

	return 0;
}

static int uhc_stm32_schedule_xfer(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	//LOCAL_LOG_DBG("schedule");

	if (priv->ongoing_xfer == NULL) {
		priv->ongoing_xfer = uhc_xfer_get_next(dev);
		if (priv->ongoing_xfer == NULL) {
			LOG_DBG("Nothing to transfer");
			return 0;
		}
		priv->ongoing_xfer_err_cpt = 0;
		// TODO: see if it is a valid way of doing things
		net_buf_simple_save(&(priv->ongoing_xfer->buf->b), &(priv->ongoing_xfer_buf_save));
	}

	/* TODO: support all sort of transfers */

	if (USB_EP_GET_IDX(priv->ongoing_xfer->ep) == 0) {
		return uhc_stm32_xfer_control(dev, priv->ongoing_xfer);
	}

	return uhc_stm32_xfer_bulk(dev, priv->ongoing_xfer);
}

static HCD_URBStateTypeDef uhc_stm32_get_ongoing_xfer_urb_state(const struct device * dev) {
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	uint8_t pipe_id = 0;
	int err = uhc_stm32_retreive_pipe_id(dev,
		priv->ongoing_xfer->ep,
		priv->ongoing_xfer->addr,
		&pipe_id
	);
	if (err) {
		LOG_ERR("Pipe ID not found for ongoing XFER");
		__ASSERT_NO_MSG(0);
	}

	return HAL_HCD_HC_GetURBState(&(priv->hcd), pipe_id);
}

static void uhc_stm32_xfer_end(const struct device *dev, const int err) {
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	LOCAL_LOG_DBG("end with err = %d", err);

	uhc_xfer_return(dev, priv->ongoing_xfer, err);
	priv->ongoing_xfer = NULL;
	priv->ongoing_xfer_err_cpt = 0;
}

static int uhc_stm32_xfer_update(const struct device *dev) {
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	if (priv->ongoing_xfer == NULL) {
		LOCAL_LOG_DBG("Nothing");
		return 0;
	}

	HCD_URBStateTypeDef urb_state = uhc_stm32_get_ongoing_xfer_urb_state(dev);

	if (urb_state == URB_DONE) {
		LOCAL_LOG_DBG("DONE!");
		priv->ongoing_xfer_err_cpt = 0;
		if (USB_EP_GET_IDX(priv->ongoing_xfer->ep) == 0) {
			if (priv->ongoing_xfer->stage == UHC_CONTROL_STAGE_SETUP) {
				if (priv->ongoing_xfer->buf != NULL) {
					priv->ongoing_xfer->stage = UHC_CONTROL_STAGE_DATA;
				} else {
					priv->ongoing_xfer->stage = UHC_CONTROL_STAGE_STATUS;
				}
			} else if (priv->ongoing_xfer->stage == UHC_CONTROL_STAGE_DATA) {
				if ((USB_EP_DIR_IS_IN(priv->ongoing_xfer->ep)
						&& net_buf_tailroom(priv->ongoing_xfer->buf) == 0)
				    || (USB_EP_DIR_IS_OUT(priv->ongoing_xfer->ep)
						&& priv->ongoing_xfer->buf->len == 0)) {
					priv->ongoing_xfer->stage = UHC_CONTROL_STAGE_STATUS;
				}
			} else if (priv->ongoing_xfer->stage == UHC_CONTROL_STAGE_STATUS) {
				uhc_stm32_xfer_end(dev, 0);
			} else {
				__ASSERT_NO_MSG(0);
			}
		}
	} else if (urb_state != URB_IDLE) {
		LOCAL_LOG_DBG("ERR!");
		priv->ongoing_xfer_err_cpt++;
		if (priv->ongoing_xfer_err_cpt >= UHC_STM32_MAX_ERR) {
			uhc_stm32_xfer_end(dev, 1); // TODO: proper err value
			return 0;
		}
		priv->ongoing_xfer->stage = UHC_CONTROL_STAGE_SETUP;
		// restore net buf to it's pristine state
		net_buf_simple_restore(&(priv->ongoing_xfer->buf->b), &(priv->ongoing_xfer_buf_save));
		// must retry
		//k_work_submit_to_queue(&priv->work_queue, &priv->on_xfer_update_work);
	} else { // urb_state == URB_IDLE
		LOCAL_LOG_DBG("NOPE!");
		// must retry later
		k_work_submit_to_queue(&priv->work_queue, &priv->on_xfer_update_work);
		/* Nothing to do */
		return 0;
	}

	if (priv->ongoing_xfer != NULL) {
		return uhc_stm32_schedule_xfer(dev);
	}

	return 0;
}

// TODO : lock when needed
void priv_on_port_connect(struct k_work *item)
{
    struct uhc_stm32_data *priv = CONTAINER_OF(item, struct uhc_stm32_data, on_connect_work);

	LOG_DBG("Event : Connection");

	/* PHY is high speed capable and connected device may also be so we must
	   schedule a reset to allow determining the real device speed. */
	int ret = k_work_reschedule_for_queue(&priv->work_queue, &priv->delayed_reset_work, K_MSEC(200));
	if (ret < 0) {
		// TODO
		__ASSERT_NO_MSG(0);
	} else {
		// TODO
	}
	priv->state = STM32_UHC_STATE_SPEED_ENUM;
}

void priv_on_reset(struct k_work *work)
{
    struct uhc_stm32_data *priv = CONTAINER_OF(work, struct uhc_stm32_data, on_reset_work);

	LOG_DBG("Event : Reset");

	// TODO: see what happen when this function is called but without any device connected
	enum usb_speed current_speed = priv_get_current_speed(priv->dev);

	if (priv->state == STM32_UHC_STATE_SPEED_ENUM) {
		priv->state = STM32_UHC_STATE_READY;
		if (current_speed == USB_SPEED_LOW) {
			LOG_DBG("LS");
			uhc_submit_event(priv->dev, UHC_EVT_DEV_CONNECTED_LS, 0);
		} else if (current_speed == USB_SPEED_FULL) {
			LOG_DBG("FS");
			uhc_submit_event(priv->dev, UHC_EVT_DEV_CONNECTED_FS, 0);
		} else {
			LOG_DBG("HS");
			uhc_submit_event(priv->dev, UHC_EVT_DEV_CONNECTED_HS, 0);
		}
	} else {
		/* Let higher level code know a reset occurred */
		uhc_submit_event(priv->dev, UHC_EVT_RESETED, 0);
	}

	if (priv->state != STM32_UHC_STATE_DISCONNECTED) {
		int err = 0;

		// TODO : this open pipe thing is temporary
		uint8_t pipe_speed = USB_SPEED_LOW;
		if (current_speed == USB_SPEED_LOW) {
			pipe_speed = HCD_DEVICE_SPEED_LOW;
		} else if (current_speed == USB_SPEED_FULL) {
			pipe_speed = HCD_DEVICE_SPEED_FULL;
		} else if (current_speed == USB_SPEED_HIGH) {
			pipe_speed = HCD_DEVICE_SPEED_HIGH;
		}
		err = uhc_stm32_open_pipe(priv->dev,
			&(priv->control_pipe),
			0, DEFAULT_ADDR,
			pipe_speed,
			EP_TYPE_CTRL,
			DEFAULT_EP0_MPS
		);
		if (err) {
			LOG_ERR("Failed to open control pipe out channel");
			__ASSERT_NO_MSG(0);
		}
	}
}

void priv_on_port_disconnect(struct k_work *work)
{
    struct uhc_stm32_data *priv = CONTAINER_OF(work, struct uhc_stm32_data, on_disconnect_work);

	LOG_DBG("Event : Disconnection");

	priv->state = STM32_UHC_STATE_DISCONNECTED;

	// TODO : empty workqueue ?

	uhc_stm32_close_all_pipes(priv->dev);

	/* Let higher level code know a disconnection occurred */
	uhc_submit_event(priv->dev, UHC_EVT_DEV_REMOVED, 0);
}

void priv_on_xfer_update(struct k_work *work)
{
    struct uhc_stm32_data *priv = CONTAINER_OF(work, struct uhc_stm32_data, on_xfer_update_work);

	LOG_DBG("Event : Update");
	int err = uhc_stm32_xfer_update(priv->dev);
	if (err) {
		// TODO
		LOG_DBG("AARRGGG 3, %d", err);
	}
}

void priv_on_new_xfer(struct k_work *work)
{
    struct uhc_stm32_data *priv = CONTAINER_OF(work, struct uhc_stm32_data, on_new_xfer_work);

	LOG_DBG("Event : New XFER");
	// TODO: this is just for tests must handle it properly
	int err = uhc_stm32_schedule_xfer(priv->dev);
	if (err) {
		// TODO
		LOG_DBG("AARRGGG 1, %d", err);
	}
}

void priv_delayed_reset(struct k_work * work)
{
	struct k_work_delayable *delayable_work = k_work_delayable_from_work(work);
	struct uhc_stm32_data *priv =
		CONTAINER_OF(delayable_work, struct uhc_stm32_data, delayed_reset_work);

	if (priv->state == STM32_UHC_STATE_SPEED_ENUM) {
		// TODO: this function takes time and will delay other submited work
		uhc_stm32_bus_reset(priv->dev);
	}
}

static const struct uhc_api uhc_stm32_api = {
	.lock = uhc_stm32_lock,
	.unlock = uhc_stm32_unlock,

	.init = uhc_stm32_init,
	.enable = uhc_stm32_enable,
	.disable = uhc_stm32_disable,
	.shutdown = uhc_stm32_shutdown,

	.bus_reset = uhc_stm32_bus_reset,
	.sof_enable = uhc_stm32_sof_enable,
	.bus_suspend = uhc_stm32_bus_suspend,
	.bus_resume = uhc_stm32_bus_resume,

	.ep_enqueue = uhc_stm32_ep_enqueue,
	.ep_dequeue = uhc_stm32_ep_dequeue,
};

K_THREAD_STACK_DEFINE(work_thread_stack, CONFIG_UHC_STM32_DRV_THREAD_STACK_SIZE);

static void uhc_stm32_driver_preinit_common(const struct device *dev)
{
	struct uhc_data *data = dev->data;
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	const struct uhc_stm32_config *config = dev->config;

	if (config->max_speed >= USB_SPEED_HIGH) {
		data->caps.hs = 1;
	} else {
		data->caps.hs = 0;
	}

	priv->dev = dev;
	priv->state = STM32_UHC_STATE_DISCONNECTED;

	k_work_queue_init(&priv->work_queue);
	k_work_init(&priv->on_connect_work, priv_on_port_connect);
	k_work_init(&priv->on_disconnect_work, priv_on_port_disconnect);
	k_work_init(&priv->on_reset_work, priv_on_reset);
	k_work_init(&priv->on_xfer_update_work, priv_on_xfer_update);
	k_work_init(&priv->on_new_xfer_work, priv_on_new_xfer);
	k_work_init_delayable(&priv->delayed_reset_work, priv_delayed_reset);

	k_work_queue_start(&priv->work_queue,
					   work_thread_stack, K_THREAD_STACK_SIZEOF(work_thread_stack),
					   K_PRIO_COOP(2), NULL);
}

#define DT_PHY(node_id)           DT_PHANDLE(node_id, phys)
#define DT_PHY_COMPAT_EMBEDDED_FS usb_nop_xceiv
#define DT_PHY_COMPAT_EMBEDDED_HS st_stm32_usbphyc
#define DT_PHY_COMPAT_ULPI        usb_ulpi_phy

#define DT_MAX_SPEED_OR(node_id, default_speed) \
	DT_ENUM_IDX_OR(node_id, maximum_speed, default_speed)

#define GET_MAX_SPEED(node_id, phy_max_speed) \
	MIN(DT_MAX_SPEED_OR(node_id, phy_max_speed), phy_max_speed)

#define UHC_STM32_INIT_FUNC_NAME(node_id) uhc_stm32_driver_init_##node_id

#define UHC_STM32_INIT_FUNC(node_id)                                   \
static int UHC_STM32_INIT_FUNC_NAME(node_id)(const struct device *dev) \
{                                                                      \
	uhc_stm32_driver_preinit_common(dev);                              \
                                                                       \
	IRQ_CONNECT(DT_IRQN(node_id),                                      \
		DT_IRQ(node_id, priority), uhc_stm32_irq,                      \
		DEVICE_DT_GET(node_id), 0                                      \
	);                                                                 \
                                                                       \
	return 0;                                                          \
}

// TODO: use IF_ENABLED macro

#if defined(USB_OTG_HS_EMBEDDED_PHY)
#define GET_PHY_TYPE(node_id) ( \
	DT_NODE_HAS_COMPAT(DT_PHY(node_id), DT_PHY_COMPAT_EMBEDDED_FS) ? PHY_EMBEDDED_FS : ( \
	DT_NODE_HAS_COMPAT(DT_PHY(node_id), DT_PHY_COMPAT_EMBEDDED_HS) ? PHY_EMBEDDED_HS : ( \
	DT_NODE_HAS_COMPAT(DT_PHY(node_id), DT_PHY_COMPAT_ULPI) ? PHY_EXTERNAL_ULPI: (       \
	PHY_INVALID))) \
)
#define GET_PHY_MAX_SPEED_FROM_TYPE(phy_type) ( \
	(phy_type == PHY_EMBEDDED_FS) ? USB_SPEED_FULL : (  \
	(phy_type == PHY_EMBEDDED_HS) ? USB_SPEED_HIGH : (  \
	(phy_type == PHY_EXTERNAL_ULPI) ? USB_SPEED_HIGH: ( \
	USB_SPEED_INVALID))) \
)
#else
#define GET_PHY_TYPE(node_id) ( \
	DT_NODE_HAS_COMPAT(DT_PHY(node_id), DT_PHY_COMPAT_EMBEDDED_FS) ? PHY_EMBEDDED_FS : ( \
	DT_NODE_HAS_COMPAT(DT_PHY(node_id), DT_PHY_COMPAT_ULPI) ? PHY_EXTERNAL_ULPI: ( \
	PHY_INVALID)) \
)
#define GET_PHY_MAX_SPEED_FROM_TYPE(phy_type) ( \
	(phy_type == PHY_EMBEDDED_FS) ? USB_SPEED_FULL :  ( \
	(phy_type == PHY_EXTERNAL_ULPI) ? USB_SPEED_HIGH: ( \
	USB_SPEED_INVALID)) \
)
#endif

#define GET_PHY_MAX_SPEED(node_id) GET_PHY_MAX_SPEED_FROM_TYPE(GET_PHY_TYPE(node_id))


#define STM32_OTGHS_INIT(node_id)                                                                  \
                                                                                                   \
PINCTRL_DT_DEFINE(node_id);                                                                        \
								                                                                   \
BUILD_ASSERT(DT_NUM_CLOCKS(node_id) <= DT_CLOCKS_PROP_MAX_LEN,                                     \
	"Invalid number of element in \"clock\" property in device tree");                             \
BUILD_ASSERT(DT_PROP(node_id, num_host_channels) <= NB_MAX_PIPE,                                   \
	"Invalid number of host channels");                                                            \
BUILD_ASSERT(GET_PHY_TYPE(node_id) != PHY_INVALID,                                                 \
	"Unsupported or incompatible USB PHY defined in device tree");                                 \
BUILD_ASSERT(GET_MAX_SPEED(node_id, GET_PHY_MAX_SPEED(node_id)) != USB_SPEED_INVALID,              \
	"Invalid usb speed");                                                                          \
                                                                                                   \
static const struct uhc_stm32_config uhc_config_##node_id = {                                      \
	.irq = DT_IRQN(node_id),                                                                       \
	.pcfg = PINCTRL_DT_DEV_CONFIG_GET(node_id),                                                    \
	.vbus_enable_gpio = GPIO_DT_SPEC_GET_OR(node_id, vbus_gpios, {0}),                             \
	.clocks = STM32_DT_CLOCKS(node_id),                                                            \
	.num_clock = DT_NUM_CLOCKS(node_id),                                                           \
	.num_host_channels = DT_PROP(node_id, num_host_channels),                                      \
	.phy = GET_PHY_TYPE(node_id),                                                                  \
	.max_speed = GET_MAX_SPEED(node_id, GET_PHY_MAX_SPEED(node_id)),                               \
	.ulpi_reset_gpio = GPIO_DT_SPEC_GET_OR(DT_PHY(node_id), reset_gpios, {0}),                     \
};                                                                                                 \
                                                                                                   \
static struct uhc_stm32_data uhc_priv_data_##node_id = {                                           \
	.hcd.Instance = (HCD_TypeDef *) DT_REG_ADDR(node_id),                                          \
};                                                                                                 \
                                                                                                   \
static struct uhc_data uhc_data_##node_id = {                                                      \
	.mutex = Z_MUTEX_INITIALIZER(uhc_data_##node_id.mutex),                                        \
	.priv = &uhc_priv_data_##node_id,                                                              \
};                                                                                                 \
                                                                                                   \
UHC_STM32_INIT_FUNC(node_id);                                                                      \
                                                                                                   \
DEVICE_DT_DEFINE(node_id, UHC_STM32_INIT_FUNC_NAME(node_id), NULL,                                 \
				 &uhc_data_##node_id, &uhc_config_##node_id,                                       \
				 POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                                  \
				 &uhc_stm32_api);


#define STM32_OTGFS_INIT(node_id)                                                 \
                                                                                  \
PINCTRL_DT_DEFINE(node_id);                                                       \
								                                                  \
BUILD_ASSERT(DT_NUM_CLOCKS(node_id) <= DT_CLOCKS_PROP_MAX_LEN,                    \
	"Invalid number of element in \"clock\" property in device tree");            \
BUILD_ASSERT(DT_PROP(node_id, num_host_channels) <= NB_MAX_PIPE,                  \
	"Invalid number of host channels");                                           \
BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_PHY(node_id), DT_PHY_COMPAT_EMBEDDED_FS),      \
	"Invalid PHY defined in device tree");                                        \
BUILD_ASSERT(GET_MAX_SPEED(node_id, GET_PHY_MAX_SPEED(node_id)) != USB_SPEED_INVALID,              \
	"Invalid usb speed");                                                                          \
                                                                                  \
static const struct uhc_stm32_config uhc_config_##node_id = {                     \
	.irq = DT_IRQN(node_id),                                                      \
	.pcfg = PINCTRL_DT_DEV_CONFIG_GET(node_id),                                   \
	.vbus_enable_gpio = GPIO_DT_SPEC_GET_OR(node_id, vbus_gpios, {0}),            \
	.clocks = STM32_DT_CLOCKS(node_id),                                           \
	.num_clock = DT_NUM_CLOCKS(node_id),                                                           \
	.num_host_channels = DT_PROP(node_id, num_host_channels),                     \
	.max_speed = GET_MAX_SPEED(node_id, USB_SPEED_FULL),                  \
	.phy = PHY_EMBEDDED_FS,                                                       \
};                                                                                \
                                                                                  \
static struct uhc_stm32_data uhc_priv_data_##node_id = {                                           \
	.hcd.Instance = (HCD_TypeDef *) DT_REG_ADDR(node_id),                                          \
};   \
                                                                                  \
static struct uhc_data uhc_data_##node_id = {                                     \
	.mutex = Z_MUTEX_INITIALIZER(uhc_data_##node_id.mutex),                       \
	.priv = &uhc_priv_data_##node_id,                                             \
};                                                                                                 \
																								   \
UHC_STM32_INIT_FUNC(node_id);                                                                      \
																								   \
																								   \
DEVICE_DT_DEFINE(node_id, UHC_STM32_INIT_FUNC_NAME(node_id), NULL,                                 \
				 &uhc_data_##node_id, &uhc_config_##node_id,                                       \
				 POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                                  \
				 &uhc_stm32_api);


#define STM32_USB_INIT(node_id)                                                   \
                                                                                  \
PINCTRL_DT_DEFINE(node_id);                                                       \
								                                                  \
BUILD_ASSERT(DT_NUM_CLOCKS(node_id) <= DT_CLOCKS_PROP_MAX_LEN,                    \
	"Invalid number of element in \"clock\" property in device tree");            \
BUILD_ASSERT(DT_PROP(node_id, num_host_channels) <= NB_MAX_PIPE,                  \
	"Invalid number of host channels");                                           \
BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_PHY(node_id), DT_PHY_COMPAT_EMBEDDED_FS),      \
	"Invalid PHY defined in device tree");                                        \
BUILD_ASSERT(GET_MAX_SPEED(node_id, GET_PHY_MAX_SPEED(node_id)) != USB_SPEED_INVALID,              \
	"Invalid usb speed");                                                                          \
                                                                                  \
static const struct uhc_stm32_config uhc_config_##node_id = {                     \
	.irq = DT_IRQN(node_id),                                                      \
	.pcfg = PINCTRL_DT_DEV_CONFIG_GET(node_id),                                   \
	.vbus_enable_gpio = GPIO_DT_SPEC_GET_OR(node_id, vbus_gpios, {0}),            \
	.clocks = STM32_DT_CLOCKS(node_id),                                           \
	.num_clock = DT_NUM_CLOCKS(node_id),                                                           \
	.num_host_channels = DT_PROP(node_id, num_host_channels),                     \
	.max_speed = GET_MAX_SPEED(node_id, USB_SPEED_FULL),                  \
	.phy = PHY_EMBEDDED_FS,                                                       \
};                                                                                \
                                                                                  \
static struct uhc_stm32_data uhc_priv_data_##node_id = {                                           \
	.hcd.Instance = (HCD_TypeDef *) DT_REG_ADDR(node_id),                                          \
};                                                                                     \
                                                                                  \
static struct uhc_data uhc_data_##node_id = {                                     \
	.mutex = Z_MUTEX_INITIALIZER(uhc_data_##node_id.mutex),                       \
	.priv = &uhc_priv_data_##node_id,                                             \
};                                                                                                 \
																								   \
UHC_STM32_INIT_FUNC(node_id);                                                                      \
																								   \
																								   \
DEVICE_DT_DEFINE(node_id, UHC_STM32_INIT_FUNC_NAME(node_id), NULL,                                 \
				 &uhc_data_##node_id, &uhc_config_##node_id,                                       \
				 POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                                  \
				 &uhc_stm32_api);


DT_FOREACH_STATUS_OKAY(st_stm32_otghs, STM32_OTGHS_INIT)
DT_FOREACH_STATUS_OKAY(st_stm32_otgfs, STM32_OTGFS_INIT)
DT_FOREACH_STATUS_OKAY(st_stm32_usb, STM32_USB_INIT)
