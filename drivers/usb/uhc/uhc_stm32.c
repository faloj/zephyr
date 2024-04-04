/*
 * Copyright (c) 2024 Syslinbit
 * Copyright (c) 2023 Linaro Limited
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

/* TODO: factorize core init part with udc_stm32 driver if possible */

// TODO : remove
#define LOCAL_LOG_DBG(msg, ...) LOG_DBG(msg __VA_OPT__(,) __VA_ARGS__)

// TODO: rename ?
#define DT_CLOCKS_PROP_MAX_LEN (2)

#define SETUP_PACKET_SIZE (8U)

#define DEFAULT_ADDR    ( 0U)
#define USB_CONTROL_EP  ( 0U)

#define NB_MAX_PIPE (16U)

BUILD_ASSERT(NB_MAX_PIPE <= UINT8_MAX);

#define USB_NB_MAX_XFER_ATTEMPTS (3U)


#define _IS_DRIVER_OF_INSTANCE(dev, instance) \
	(((struct uhc_stm32_data *) uhc_get_private(dev))->hcd_ptr->Instance == instance)

#if defined(USB_OTG_FS)
#define IS_USB_OTG_FS_DEVICE(dev) _IS_DRIVER_OF_INSTANCE(dev, USB_OTG_FS)
#else
#define IS_USB_OTG_FS_DEVICE(dev) (0)
#endif

#if defined(USB_OTG_HS)
#define IS_USB_OTG_HS_DEVICE(dev) _IS_DRIVER_OF_INSTANCE(dev, USB_OTG_HS)
#else
#define IS_USB_OTG_HS_DEVICE(dev) (0)
#endif

#if defined(USB_DRD_FS)
#define IS_USB_DRD_FS_DEVICE(dev) _IS_DRIVER_OF_INSTANCE(dev, USB_DRD_FS)
#else
#define IS_USB_DRD_FS_DEVICE(dev) (0)
#endif


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

enum phy_interface {
	PHY_INVALID       = -1,
	PHY_EXTERNAL_ULPI = HCD_PHY_ULPI,
	PHY_EMBEDDED_FS   = HCD_PHY_EMBEDDED,
#if defined(USB_OTG_HS_EMBEDDED_PHY)
	PHY_EMBEDDED_HS   = USB_OTG_HS_EMBEDDED_PHY,
#endif
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
	HCD_HandleTypeDef *hcd_ptr;
	struct uhc_transfer *ongoing_xfer;
	struct net_buf_simple_state ongoing_xfer_buf_save;
	size_t ongoing_xfer_attempts;
	uint8_t ongoing_xfer_pipe_id;
	struct k_work_q work_queue;
	struct k_work on_connect_work;
	struct k_work on_disconnect_work;
	struct k_work on_reset_work;
	struct k_work on_sof_work;
	struct k_work on_xfer_update_work;
	struct k_work on_schedule_new_xfer_work;
	struct k_work_delayable delayed_reset_work;
};

struct uhc_stm32_config {
	uint32_t irq;
	enum phy_interface phy;
	size_t num_host_channels;
	struct stm32_pclken clocks[DT_CLOCKS_PROP_MAX_LEN];
	size_t num_clock;
	enum usb_speed max_speed;
	const struct pinctrl_dev_config *pcfg;
	struct gpio_dt_spec ulpi_reset_gpio;
	struct gpio_dt_spec vbus_enable_gpio;
};

#if defined(USB_DRD_FS)
#define SPEED_FROM_HAL_SPEED(drd_core_speed) (     \
	(drd_core_speed == USB_DRD_SPEED_LS) ? USB_SPEED_FULL : (  \
	(drd_core_speed == USB_DRD_SPEED_FS) ? USB_SPEED_HIGH : (  \
	USB_SPEED_INVALID))                                        \
)
#endif

#if defined(USB_OTG_FS) || defined(USB_OTG_HS)
#define SPEED_FROM_HAL_SPEED(otg_core_speed) (            \
	(otg_core_speed == HPRT0_PRTSPD_LOW_SPEED) ? USB_SPEED_FULL : (   \
	(otg_core_speed == HPRT0_PRTSPD_FULL_SPEED) ? USB_SPEED_HIGH : (  \
	(otg_core_speed == HPRT0_PRTSPD_HIGH_SPEED) ? USB_SPEED_HIGH : (  \
	USB_SPEED_INVALID)))                                              \
)
#endif

#define SPEED_TO_HCD_INIT_SPEED(usb_speed) (           \
	(usb_speed >= USB_SPEED_HIGH) ? HCD_SPEED_HIGH : ( \
	(usb_speed == USB_SPEED_FULL) ? HCD_SPEED_FULL : ( \
	(usb_speed == USB_SPEED_LOW) ? HCD_SPEED_LOW : (   \
	HCD_SPEED_LOW)))                                   \
)


static inline enum usb_speed priv_get_current_speed(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	enum usb_speed speed = SPEED_FROM_HAL_SPEED(
		HAL_HCD_GetCurrentSpeed(priv->hcd_ptr)
	);

	if (speed == USB_SPEED_INVALID) {
		LOG_ERR("Invalid USB speed returned by \"HAL_HCD_GetCurrentSpeed\"");
		__ASSERT_NO_MSG(0);

		/* Falling back to low speed */
		speed = USB_SPEED_LOW;
	}

	return speed;
}

static void uhc_stm32_irq(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	/* Call the ST IRQ handler which will, in turn, call the corresponding HAL_HCD_* callbacks */
	HAL_HCD_IRQHandler(priv->hcd_ptr);
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
}

void HAL_HCD_SOF_Callback(HCD_HandleTypeDef *hhcd)
{
	const struct device *dev = (const struct device *)hhcd->pData;
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	k_work_submit_to_queue(&priv->work_queue, &priv->on_sof_work);
}

void HAL_HCD_HC_NotifyURBChange_Callback(HCD_HandleTypeDef *hhcd, uint8_t chnum,
					 HCD_URBStateTypeDef urb_state)
{
	const struct device *dev = (const struct device *)hhcd->pData;
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	k_work_submit_to_queue(&priv->work_queue, &priv->on_xfer_update_work);
}


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


#if defined(CONFIG_SOC_SERIES_STM32H7X)
	if(IS_USB_OTG_HS_DEVICE(dev)) {
		LL_AHB1_GRP1_EnableClockSleep(LL_AHB1_GRP1_PERIPH_USB1OTGHS);
		if (config->phy == PHY_EXTERNAL_ULPI) {
			LL_AHB1_GRP1_EnableClockSleep(LL_AHB1_GRP1_PERIPH_USB1OTGHSULPI);
		} else {
			/* ULPI clock is activated by default in sleep mode. If we use an
			   other PHY this will prevent the USB_OTG_HS controller to work
			   properly when the MCU enters sleep mode so we must disable it.
			*/
			LL_AHB1_GRP1_DisableClockSleep(LL_AHB1_GRP1_PERIPH_USB1OTGHSULPI);
		}
	}
#if defined(USB_OTG_FS)
	if (IS_USB_OTG_FS_DEVICE(dev)) {
		LL_AHB1_GRP1_EnableClockSleep(LL_AHB1_GRP1_PERIPH_USB2OTGHS);
		/* USB_OTG_FS cannot be connected to an external ULPI PHY but
		   ULPI clock is still activated by default in sleep mode
		   (in run mode it is already disabled by default). This prevents
		   the USB_OTG_FS controller to work properly when the MCU enters
		   sleep mode so we must disable it.
		 */
		LL_AHB1_GRP1_DisableClockSleep(LL_AHB1_GRP1_PERIPH_USB2OTGHSULPI);
	}
#endif
#endif

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

static int priv_open_pipe(const struct device *dev, uint8_t *pipe_id, uint8_t ep,
						  uint8_t dev_addr, enum usb_speed speed, uint8_t ep_type, uint16_t mps)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	const struct uhc_stm32_config *config = dev->config;

	uint8_t pipe_speed = HCD_DEVICE_SPEED_LOW;
	if (speed == USB_SPEED_LOW) {
		pipe_speed = HCD_DEVICE_SPEED_LOW;
	} else if (speed == USB_SPEED_FULL) {
		pipe_speed = HCD_DEVICE_SPEED_FULL;
	} else if (speed == USB_SPEED_HIGH) {
		pipe_speed = HCD_DEVICE_SPEED_HIGH;
	} else {
		__ASSERT_NO_MSG(0);
	}

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

	HAL_StatusTypeDef status = HAL_HCD_HC_Init(priv->hcd_ptr,
		*pipe_id, USB_EP_GET_IDX(ep),
        dev_addr, pipe_speed,
		ep_type, mps
	);
	if (status != HAL_OK) {
		return -EIO;
	}

	priv->busy_pipe[i] = true;
	return 0;
}

static int priv_close_pipe(const struct device *dev, uint8_t pipe_id)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	const struct uhc_stm32_config *config = dev->config;

	if (pipe_id > config->num_host_channels) {
		return -EINVAL;
	}

	if (priv->busy_pipe[pipe_id] != true) {
		return -EALREADY;
	}

	HAL_HCD_HC_Halt(priv->hcd_ptr, pipe_id);

	priv->busy_pipe[pipe_id] = false;

	return 0;
}

static int priv_retreive_pipe_id(const struct device *dev, const uint8_t ep,
										const uint8_t addr, uint8_t * const pipe_id)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	const struct uhc_stm32_config *config = dev->config;

	bool found = false;

	size_t i;
	for(i = 0; i < config->num_host_channels; i++) {
		if (priv->busy_pipe[i] == true) {
			if ((priv->hcd_ptr->hc[i].ep_num == USB_EP_GET_IDX(ep)) &&
				(priv->hcd_ptr->hc[i].dev_addr == addr)) {
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

static void priv_init_pipes(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	const struct uhc_stm32_config *config = dev->config;

	size_t i;
	for(i = 0; i < config->num_host_channels; i++) {
		priv->busy_pipe[i] = false;
	}
}

static void priv_close_all_pipes(const struct device *dev)
{
	const struct uhc_stm32_config *config = dev->config;

	size_t i;
	for(i = 0; i < config->num_host_channels; i++) {
		priv_close_pipe(dev, i);
	}
}

static inline void priv_deinit_pipes(const struct device *dev)
{
	return priv_close_all_pipes(dev);
}

static inline int priv_submit_request(const struct device *dev, uint8_t chan_num, uint8_t direction,
									  uint8_t ep_type, uint8_t token, uint8_t *buf, uint16_t length)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	HAL_StatusTypeDef status = HAL_HCD_HC_SubmitRequest(priv->hcd_ptr,
		chan_num, direction, ep_type, token, buf, length, 0
	);

	if (status == HAL_OK) {
		return 0;
	} else {
		return -EIO;
	}
}

static inline int priv_send_control_setup(const struct device *dev, const uint8_t chan_num,
										  uint8_t *buf, uint16_t length)
{
	if (length != SETUP_PACKET_SIZE) {
		return -EINVAL;
	}

	return priv_submit_request(dev, chan_num, 0, EP_TYPE_CTRL, 0, buf, length);
}

static inline int priv_send_control_status(const struct device *dev, const uint8_t chan_num)
{
	return priv_submit_request(dev, chan_num, 0, EP_TYPE_CTRL, 1, NULL, 0);
}

static int priv_receive_control_status(const struct device *dev, const uint8_t chan_num)
{
	return priv_submit_request(dev, chan_num, 1, EP_TYPE_CTRL, 1, NULL, 0);
}

static int priv_send_data(const struct device *dev, const uint8_t chan_num,
						  struct net_buf *const buf, const uint8_t ep_type,
						  const uint8_t maximum_packet_size)
{
	size_t tx_size = MIN(buf->len, maximum_packet_size);

	int err = priv_submit_request(dev, chan_num, 0, ep_type, 1, buf->data, tx_size);
	if (err) {
		return err;
	}

	net_buf_pull(buf, tx_size);

	return 0;
}

static int priv_receive_data(const struct device *dev, const uint8_t chan_num,
							 struct net_buf *const buf, const uint8_t ep_type,
							 const uint8_t maximum_packet_size)
{
	size_t rx_size = MIN(net_buf_tailroom(buf), maximum_packet_size);
	if (rx_size == 0) {
		return -ENOMEM;
	}

	void* buffer_tail = net_buf_add(buf, rx_size);

	int err = priv_submit_request(dev, chan_num, 1, ep_type, 1, buffer_tail, rx_size);
	if (err) {
		net_buf_remove_mem(buf, rx_size);
		return err;
	}

	return 0;
}

static int priv_ongoing_xfer_control_run(const struct device *dev, struct uhc_transfer *const xfer)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	if (xfer->stage == UHC_CONTROL_STAGE_SETUP) {
		LOCAL_LOG_DBG("Handle control SETUP stage");
		return priv_send_control_setup(dev,
			priv->ongoing_xfer_pipe_id, xfer->setup_pkt, sizeof(xfer->setup_pkt)
		);
	}

	if (xfer->buf != NULL && xfer->stage == UHC_CONTROL_STAGE_DATA) {
		if (USB_EP_DIR_IS_IN(xfer->ep)) {
			LOCAL_LOG_DBG("Handle control DATA stage: receive");
			return priv_receive_data(dev,
				priv->ongoing_xfer_pipe_id, xfer->buf, USB_EP_TYPE_CONTROL, xfer->mps
			);
		} else {
			LOCAL_LOG_DBG("Handle control DATA stage: send");
			return priv_send_data(dev,
				priv->ongoing_xfer_pipe_id, xfer->buf, USB_EP_TYPE_CONTROL, xfer->mps
			);
		}
	}

	if (xfer->stage == UHC_CONTROL_STAGE_STATUS) {
		if (USB_EP_DIR_IS_IN(xfer->ep)) {
			LOCAL_LOG_DBG("Handle control STATUS stage: send");
			return priv_send_control_status(dev, priv->ongoing_xfer_pipe_id);
		} else {
			LOCAL_LOG_DBG("Handle STATUS stage: receive");
			return priv_receive_control_status(dev, priv->ongoing_xfer_pipe_id);
		}
	}

	return -EINVAL;
}

static int priv_ongoing_xfer_bulk_run(const struct device *dev, struct uhc_transfer *const xfer)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(xfer);

	/* TODO */

	return 0;
}

static int priv_ongoing_xfer_run(const struct device * dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	if (USB_EP_GET_IDX(priv->ongoing_xfer->ep) == USB_CONTROL_EP) {
		return priv_ongoing_xfer_control_run(dev, priv->ongoing_xfer);
	} else {
		/* TODO: For now all other xfer are considered as bulk transfers,
		   add support for all type of transfer
		 */
		return priv_ongoing_xfer_bulk_run(dev, priv->ongoing_xfer);
	}
}

static int priv_ongoing_xfer_start_next(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	if (priv->ongoing_xfer != NULL) {
		/* A transfer is already ongoing */
		return 0;
	}

	priv->ongoing_xfer = uhc_xfer_get_next(dev);

	if (priv->ongoing_xfer == NULL) {
		/* There is no xfer enqueued */
		return 0;
	}

	priv->ongoing_xfer_attempts = 0;

	/* Note: net_buf API does not offer a proper way to directly save the net_buf state
	   so it's internal normally hidden net_buf_simple is saved instead
	*/
	net_buf_simple_save(&(priv->ongoing_xfer->buf->b), &(priv->ongoing_xfer_buf_save));

	/* Retreive/open a pipe. This is temporary as the upper code does not manage pipes yet. */
	int err = priv_retreive_pipe_id(dev,
		priv->ongoing_xfer->ep,
		priv->ongoing_xfer->addr,
		&(priv->ongoing_xfer_pipe_id)
	);

	if (err) {
		__ASSERT_NO_MSG(err == -ENODEV);
		uint8_t ep_type = EP_TYPE_BULK;
		if (USB_EP_GET_IDX(priv->ongoing_xfer->ep) == USB_CONTROL_EP) {
			/* TODO : handle other type of transfers */
			ep_type = EP_TYPE_CTRL;
		}
		enum usb_speed speed = priv_get_current_speed(priv->dev);
		err = priv_open_pipe(priv->dev,
			&(priv->ongoing_xfer_pipe_id),
			priv->ongoing_xfer->ep,
			priv->ongoing_xfer->addr,
			speed, ep_type,
			priv->ongoing_xfer->mps
		);
		if (err) {
			return err;
		}
	}

	return priv_ongoing_xfer_run(dev);
}

static void priv_ongoing_xfer_end(const struct device *dev, const int err)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	if (priv->ongoing_xfer == NULL) {
		return;
	}

	/* Close pipe if it is not the control pipe as it is supposed to stay oppened.
	   This is temporary as the upper code does not manage pipes yet.
	 */
	if (USB_EP_GET_IDX(priv->ongoing_xfer->ep) != USB_CONTROL_EP) {
		int ret = priv_close_pipe(dev, priv->ongoing_xfer_pipe_id);
		(void) ret;
		__ASSERT_NO_MSG(ret == 0);
	}

	uhc_xfer_return(dev, priv->ongoing_xfer, err);

	priv->ongoing_xfer = NULL;

	/* There may be more xfer to handle, submit a work to handle the next one */
	k_work_submit_to_queue(&priv->work_queue, &priv->on_schedule_new_xfer_work);
}

static void priv_ongoing_xfer_handle_timeout(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	if (priv->ongoing_xfer == NULL) {
		return;
	}

	if (priv->ongoing_xfer > 0) {
		priv->ongoing_xfer->timeout -= 1;
	}

	if (priv->ongoing_xfer->timeout == 0) {
		priv_ongoing_xfer_end(priv->dev, -ETIMEDOUT);
	}
}

static inline void priv_ongoing_xfer_control_update_stage(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	/* The last transfer block succeded, reset the failed attempts counter. */
	priv->ongoing_xfer_attempts = 0;

	if (priv->ongoing_xfer->stage == UHC_CONTROL_STAGE_SETUP) {
		if (priv->ongoing_xfer->buf != NULL) {
			/* Next state is data stage */
			priv->ongoing_xfer->stage = UHC_CONTROL_STAGE_DATA;
		} else {
			/* There is no data so jump directly to the status stage */
			priv->ongoing_xfer->stage = UHC_CONTROL_STAGE_STATUS;
		}
	} else if (priv->ongoing_xfer->stage == UHC_CONTROL_STAGE_DATA) {
		if (USB_EP_DIR_IS_IN(priv->ongoing_xfer->ep) &&
			net_buf_tailroom(priv->ongoing_xfer->buf) == 0) {
			 /* No more data left to receive. Go to the status stage */
			 priv->ongoing_xfer->stage = UHC_CONTROL_STAGE_STATUS;
		} else if (USB_EP_DIR_IS_OUT(priv->ongoing_xfer->ep) &&
				   priv->ongoing_xfer->buf->len == 0) {
			/* No more data left to send. Go to the status stage */
			priv->ongoing_xfer->stage = UHC_CONTROL_STAGE_STATUS;
		} else {
			/* The transmission succeded so far. Save the actual net_buf state in case the
			   following partial data transmission fails so we can try to send it again. */
			net_buf_simple_save(&(priv->ongoing_xfer->buf->b), &(priv->ongoing_xfer_buf_save));
		}
	} else if (priv->ongoing_xfer->stage == UHC_CONTROL_STAGE_STATUS) {
		/* Transfer is completed */
		priv_ongoing_xfer_end(dev, 0);
	} else {
		/* This is not supposed to happen */
		__ASSERT_NO_MSG(0);
	}
}

static int priv_ongoing_xfer_update(const struct device *dev) {
	struct uhc_stm32_data *priv = uhc_get_private(dev);

	if (priv->ongoing_xfer == NULL) {
		/* There is no ongoing transfer */
		return 0;
	}

	HCD_URBStateTypeDef urb_state =
		HAL_HCD_HC_GetURBState(priv->hcd_ptr, priv->ongoing_xfer_pipe_id);

	if (urb_state == URB_IDLE) {
		/* This is not supposed to happen */
		__ASSERT_NO_MSG(0);
	}

	if (urb_state == URB_DONE) {
		LOG_DBG("URB_DONE");
		priv->ongoing_xfer_attempts = 0;
		if (USB_EP_GET_IDX(priv->ongoing_xfer->ep) == 0) {
			priv_ongoing_xfer_control_update_stage(dev);
		} else {
			/* Transmission succeded */
			priv_ongoing_xfer_end(dev, 0);
		}
	} else {
		LOG_DBG("URB_ERR");
		/* The transmission failed */
		priv->ongoing_xfer_attempts++;
		if (priv->ongoing_xfer_attempts >= USB_NB_MAX_XFER_ATTEMPTS) {
			/* Transmission failed too many times, cancel it. */
			priv_ongoing_xfer_end(dev, -EPIPE);
			return 0;
		}
		/* Restore net buf to the last state, to be able to re-run the transmission */
		net_buf_simple_restore(&(priv->ongoing_xfer->buf->b), &(priv->ongoing_xfer_buf_save));
	}

	if (priv->ongoing_xfer != NULL) {
		/* Transfer is not completed yet, continue the transmission */
		return priv_ongoing_xfer_run(dev);
	}

	return 0;
}

static int uhc_stm32_lock(const struct device *dev)
{
	return uhc_lock_internal(dev, K_FOREVER);
}

static int uhc_stm32_unlock(const struct device *dev)
{
	return uhc_unlock_internal(dev);
}

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

#if defined(CONFIG_SOC_SERIES_STM32H7X)
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

	HAL_StatusTypeDef status = HAL_HCD_Init(priv->hcd_ptr);

	if (status != HAL_OK) {
		LOG_ERR("HCD_Init failed, %d", (int)status);
		return -EIO;
	}

	return 0;
}

static int uhc_stm32_enable(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	const struct uhc_stm32_config *config = dev->config;

	irq_enable(config->irq);

	HAL_StatusTypeDef status = HAL_HCD_Start(priv->hcd_ptr);

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

	priv_init_pipes(dev);

	return 0;
}

static int uhc_stm32_disable(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);
	const struct uhc_stm32_config *config = dev->config;

	/* abort a potentially ongoing transfer */
	priv_ongoing_xfer_end(priv->dev, -ECANCELED);

	priv_deinit_pipes(dev);

	if (config->vbus_enable_gpio.port) {
		int err = gpio_pin_set_dt(&config->vbus_enable_gpio, 0);

		if (err) {
			LOG_ERR("Failed to disable vbus power (%d)", err);
			return err;
		}
	}

	HAL_StatusTypeDef status = HAL_HCD_Stop(priv->hcd_ptr);

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

	HAL_StatusTypeDef status = HAL_HCD_DeInit(priv->hcd_ptr);

	if (status != HAL_OK) {
		LOG_ERR("HAL_HCD_DeInit failed (%d)", (int)status);
		return -EIO;
	}

	int err = priv_clock_disable(dev);

	if (err) {
		LOG_ERR("Failed to disable USB clock (%d)", err);
		return err;
	}

	return 0;
}

static int uhc_stm32_bus_reset(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);

#if defined(USB_OTG_FS) || defined(USB_OTG_HS)
	if (IS_USB_OTG_FS_DEVICE(dev) || IS_USB_OTG_HS_DEVICE(dev)) {
		/* define USBx_BASE as USBx_HPRT0 is a special ST defined macro that depends on it */
		uint32_t USBx_BASE = (uint32_t)priv->hcd_ptr->Instance;

		__IO uint32_t hprt0_value = 0U;

		hprt0_value = USBx_HPRT0;

		/* avoid interfering with some special bits */
		hprt0_value &= ~(
			USB_OTG_HPRT_PENA |
			USB_OTG_HPRT_PCDET |
		    USB_OTG_HPRT_PENCHNG |
			USB_OTG_HPRT_POCCHNG |
			USB_OTG_HPRT_PSUSP
		);

		/* set PRST bit */
		USBx_HPRT0 = (USB_OTG_HPRT_PRST | hprt0_value);
		k_msleep(100);
		/* clear PRST bit */
		USBx_HPRT0 = ((~USB_OTG_HPRT_PRST) & hprt0_value);
		k_msleep(10);
	}
#endif

#if defined(USB_DRD_FS)
	if (IS_USB_DRD_FS_DEVICE(dev)) {
		((USB_DRD_TypeDef *) priv->hcd_ptr->Instance)->CNTR |= USB_CNTR_USBRST;
		k_msleep(100);
		((USB_DRD_TypeDef *) priv->hcd_ptr->Instance)->CNTR &= ~USB_CNTR_USBRST;
		k_msleep(30);
	}
#endif

	return 0;
}

static int uhc_stm32_sof_enable(const struct device *dev)
{
	/* TODO */
	return 0;
}

static int uhc_stm32_bus_suspend(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);

#if defined(USB_OTG_FS) || defined(USB_OTG_HS)
	if (IS_USB_OTG_FS_DEVICE(dev) || IS_USB_OTG_HS_DEVICE(dev)) {
		/* define USBx_BASE as USBx_HPRT0 is a special ST defined macro that depends on it */
		uint32_t USBx_BASE = (uint32_t)priv->hcd_ptr->Instance;

		if (READ_BIT(USBx_HPRT0, USB_OTG_HPRT_PSUSP)) {
			return -EALREADY;
		}

		__IO uint32_t hprt0_value = 0U;

		hprt0_value = USBx_HPRT0;

		/* avoid interfering with some special bits */
		hprt0_value &= ~(
			USB_OTG_HPRT_PENA |
			USB_OTG_HPRT_PCDET |
			USB_OTG_HPRT_PENCHNG |
			USB_OTG_HPRT_POCCHNG
		);

		/* set PSUP bit */
		USBx_HPRT0 = (USB_OTG_HPRT_PSUSP | hprt0_value);
	}
#endif

#if defined(USB_DRD_FS)
	if (IS_USB_DRD_FS_DEVICE(dev)) {
		/* TODO */
		return -ENOSYS;
	}
#endif

	uhc_submit_event(dev, UHC_EVT_SUSPENDED, 0);

	return 0;
}

static int uhc_stm32_bus_resume(const struct device *dev)
{
	struct uhc_stm32_data *priv = uhc_get_private(dev);

#if defined(USB_OTG_FS) || defined(USB_OTG_HS)
	if (IS_USB_OTG_FS_DEVICE(dev) || IS_USB_OTG_HS_DEVICE(dev)) {
		/* define USBx_BASE as USBx_HPRT0 is a special ST defined macro that depends on it */
		uint32_t USBx_BASE = (uint32_t)priv->hcd_ptr->Instance;

		if (READ_BIT(USBx_HPRT0, USB_OTG_HPRT_PRES)) {
			return -EBUSY;
		}

		__IO uint32_t hprt0_value = 0U;

		hprt0_value = USBx_HPRT0;

		/* avoid interfering with some special bits */
		hprt0_value &= ~(
			USB_OTG_HPRT_PENA |
			USB_OTG_HPRT_PCDET |
			USB_OTG_HPRT_PENCHNG |
			USB_OTG_HPRT_POCCHNG |
			USB_OTG_HPRT_PSUSP
		);

		/* set PRES bit */
		USBx_HPRT0 = (USB_OTG_HPRT_PRES | hprt0_value);

		/* USB specifications says resume must be driven at least 20ms */
		k_msleep(20 + 1);

		/* clear PRES bit */
		USBx_HPRT0 = ((~USB_OTG_HPRT_PRES) & hprt0_value);
		k_msleep(10);
	}
#endif

#if defined(USB_DRD_FS)
	if (IS_USB_DRD_FS_DEVICE(dev)) {
		/* TODO */
		return -ENOSYS;
	}
#endif

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

	k_work_submit_to_queue(&priv->work_queue, &priv->on_schedule_new_xfer_work);

	return 0;
}

static int uhc_stm32_ep_dequeue(const struct device *dev, struct uhc_transfer *const xfer)
{
	/* TODO */
	return 0;
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

void priv_on_port_connect(struct k_work *item)
{
    struct uhc_stm32_data *priv = CONTAINER_OF(item, struct uhc_stm32_data, on_connect_work);

	uhc_stm32_lock(priv->dev);

	LOG_DBG("Event : Connection");

	/* PHY is high speed capable and connected device may also be so we must
	   schedule a reset to allow determining the real device speed. */
	int ret = k_work_reschedule_for_queue(&priv->work_queue, &priv->delayed_reset_work, K_MSEC(200));
	if (ret != 1) {
		__ASSERT_NO_MSG(0);
	}

	priv->state = STM32_UHC_STATE_SPEED_ENUM;

	uhc_stm32_unlock(priv->dev);
}

void priv_on_reset(struct k_work *work)
{
    struct uhc_stm32_data *priv = CONTAINER_OF(work, struct uhc_stm32_data, on_reset_work);

	uhc_stm32_lock(priv->dev);

	if (priv->state == STM32_UHC_STATE_SPEED_ENUM) {
		priv->state = STM32_UHC_STATE_READY;
		enum usb_speed current_speed = priv_get_current_speed(priv->dev);
		if (current_speed == USB_SPEED_LOW) {
			LOG_DBG("Low speed device connected");
			uhc_submit_event(priv->dev, UHC_EVT_DEV_CONNECTED_LS, 0);
		} else if (current_speed == USB_SPEED_FULL) {
			LOG_DBG("Full speed device connected");
			uhc_submit_event(priv->dev, UHC_EVT_DEV_CONNECTED_FS, 0);
		} else {
			LOG_DBG("High speed device connected");
			uhc_submit_event(priv->dev, UHC_EVT_DEV_CONNECTED_HS, 0);
		}
	} else {
		/* Let higher level code know a reset occurred */
		uhc_submit_event(priv->dev, UHC_EVT_RESETED, 0);
	}

	uhc_stm32_unlock(priv->dev);
}

void priv_on_sof(struct k_work *work)
{
    struct uhc_stm32_data *priv = CONTAINER_OF(work, struct uhc_stm32_data, on_sof_work);

	uhc_stm32_lock(priv->dev);

	priv_ongoing_xfer_handle_timeout(priv->dev);

	uhc_stm32_unlock(priv->dev);
}

void priv_on_port_disconnect(struct k_work *work)
{
    struct uhc_stm32_data *priv = CONTAINER_OF(work, struct uhc_stm32_data, on_disconnect_work);

	LOG_DBG("Event : Disconnection");

	uhc_stm32_lock(priv->dev);

	priv->state = STM32_UHC_STATE_DISCONNECTED;

	/* abort a potentially ongoing transfer */
	priv_ongoing_xfer_end(priv->dev, -ECANCELED);

	priv_close_all_pipes(priv->dev);

	/* Let higher level code know a disconnection occurred */
	uhc_submit_event(priv->dev, UHC_EVT_DEV_REMOVED, 0);

	uhc_stm32_unlock(priv->dev);
}

void priv_on_xfer_update(struct k_work *work)
{
    struct uhc_stm32_data *priv = CONTAINER_OF(work, struct uhc_stm32_data, on_xfer_update_work);

	LOG_DBG("Event : Update");

	uhc_stm32_lock(priv->dev);

	int err = priv_ongoing_xfer_update(priv->dev);
	if (err) {
		/* something went wrong, cancel the ongoing transfer and notify the upper layer */
		priv_ongoing_xfer_end(priv->dev, -ECANCELED);
		uhc_submit_event(priv->dev, UHC_EVT_ERROR, err);
	}

	uhc_stm32_unlock(priv->dev);
}

void priv_on_schedule_new_xfer(struct k_work *work)
{
    struct uhc_stm32_data *priv =
		CONTAINER_OF(work, struct uhc_stm32_data, on_schedule_new_xfer_work);

	//LOG_DBG("Event : New XFER");

	uhc_stm32_lock(priv->dev);

	if (priv->state != STM32_UHC_STATE_READY) {
		/* There is no device connected (or speed enumeration is ongoing) */
		uhc_stm32_unlock(priv->dev);
		return;
	}

	int err = priv_ongoing_xfer_start_next(priv->dev);
	if (err) {
		/* something went wrong, cancel the ongoing transfer and notify the upper layer */
		priv_ongoing_xfer_end(priv->dev, -ECANCELED);
		uhc_submit_event(priv->dev, UHC_EVT_ERROR, err);
	}

	uhc_stm32_unlock(priv->dev);
}

void priv_delayed_reset(struct k_work * work)
{
	struct k_work_delayable *delayable_work = k_work_delayable_from_work(work);
	struct uhc_stm32_data *priv =
		CONTAINER_OF(delayable_work, struct uhc_stm32_data, delayed_reset_work);

	uhc_stm32_lock(priv->dev);

	if (priv->state == STM32_UHC_STATE_SPEED_ENUM) {
		/* Note: This function takes time and will delay other submited work */
		uhc_stm32_bus_reset(priv->dev);
	}

	uhc_stm32_unlock(priv->dev);
}

static void uhc_stm32_driver_init_common(const struct device *dev)
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
	k_work_init(&priv->on_sof_work, priv_on_sof);
	k_work_init(&priv->on_xfer_update_work, priv_on_xfer_update);
	k_work_init(&priv->on_schedule_new_xfer_work, priv_on_schedule_new_xfer);
	k_work_init_delayable(&priv->delayed_reset_work, priv_delayed_reset);
}


#define UHC_STM32_INIT_FUNC_NAME(node_id) uhc_stm32_driver_init_##node_id

#define UHC_STM32_INIT_FUNC(node_id)                                                               \
	static int UHC_STM32_INIT_FUNC_NAME(node_id)(const struct device *dev)                         \
	{                                                                                              \
		struct uhc_stm32_data *priv = uhc_get_private(dev);                                        \
		                                                                                           \
		uhc_stm32_driver_init_common(dev);                                                         \
		                                                                                           \
		k_work_queue_start(&priv->work_queue,                                                      \
		   uhc_work_thread_stack_##node_id,                                                        \
		   K_THREAD_STACK_SIZEOF(uhc_work_thread_stack_##node_id),                                 \
		   K_PRIO_COOP(2),                                                                         \
		   NULL                                                                                    \
		);                                                                                         \
	                                                                                               \
		IRQ_CONNECT(DT_IRQN(node_id),                                                              \
			DT_IRQ(node_id, priority), uhc_stm32_irq,                                              \
			DEVICE_DT_GET(node_id), 0                                                              \
		);                                                                                         \
	                                                                                               \
		return 0;                                                                                  \
	}


#define DT_PHY(node_id) DT_PHANDLE(node_id, phys)
#define DT_MAX_SPEED_OR(node_id, default_speed) \
	DT_ENUM_IDX_OR(node_id, maximum_speed, default_speed)


#define DT_PHY_COMPAT_EMBEDDED_FS usb_nop_xceiv
#define DT_PHY_COMPAT_EMBEDDED_HS st_stm32_usbphyc
#define DT_PHY_COMPAT_ULPI        usb_ulpi_phy


#if defined(USB_OTG_HS_EMBEDDED_PHY)
#define GET_PHY_INTERFACE(node_id) ( \
	DT_NODE_HAS_COMPAT(DT_PHY(node_id), DT_PHY_COMPAT_EMBEDDED_FS) ? PHY_EMBEDDED_FS : ( \
	DT_NODE_HAS_COMPAT(DT_PHY(node_id), DT_PHY_COMPAT_EMBEDDED_HS) ? PHY_EMBEDDED_HS : ( \
	DT_NODE_HAS_COMPAT(DT_PHY(node_id), DT_PHY_COMPAT_ULPI) ? PHY_EXTERNAL_ULPI: (       \
	PHY_INVALID))) \
)
#define _GET_PHY_MAX_SPEED(phy_interface) ( \
	(phy_interface == PHY_EMBEDDED_FS) ? USB_SPEED_FULL : (  \
	(phy_interface == PHY_EMBEDDED_HS) ? USB_SPEED_HIGH : (  \
	(phy_interface == PHY_EXTERNAL_ULPI) ? USB_SPEED_HIGH: ( \
	USB_SPEED_INVALID))) \
)
#else
#define GET_PHY_INTERFACE(node_id) ( \
	DT_NODE_HAS_COMPAT(DT_PHY(node_id), DT_PHY_COMPAT_EMBEDDED_FS) ? PHY_EMBEDDED_FS : ( \
	DT_NODE_HAS_COMPAT(DT_PHY(node_id), DT_PHY_COMPAT_ULPI) ? PHY_EXTERNAL_ULPI: ( \
	PHY_INVALID)) \
)
#define _GET_PHY_MAX_SPEED(phy_interface) ( \
	(phy_interface == PHY_EMBEDDED_FS) ? USB_SPEED_FULL :  ( \
	(phy_interface == PHY_EXTERNAL_ULPI) ? USB_SPEED_HIGH: ( \
	USB_SPEED_INVALID)) \
)
#endif

#define GET_PHY_MAX_SPEED(node_id) _GET_PHY_MAX_SPEED(GET_PHY_INTERFACE(node_id))

#define GET_MAX_SPEED(node_id, core_max_speed) \
	MIN(MIN(DT_MAX_SPEED_OR(node_id, core_max_speed), core_max_speed), GET_PHY_MAX_SPEED(node_id))


#define STM32_INIT_COMMON(node_id, core_max_speed)                                                 \
	PINCTRL_DT_DEFINE(node_id);                                                                    \
	                                                                                               \
	BUILD_ASSERT(DT_NUM_CLOCKS(node_id) <= DT_CLOCKS_PROP_MAX_LEN,                                 \
		"Invalid number of element in \"clock\" property in device tree");                         \
	BUILD_ASSERT(DT_PROP(node_id, num_host_channels) <= NB_MAX_PIPE,                               \
		"Invalid number of host channels");                                                        \
	BUILD_ASSERT(GET_PHY_INTERFACE(node_id) != PHY_INVALID,                                        \
		"Unsupported or incompatible USB PHY defined in device tree");                             \
	BUILD_ASSERT(GET_MAX_SPEED(node_id, core_max_speed) != USB_SPEED_INVALID,                      \
		"Invalid usb speed");                                                                      \
	                                                                                               \
	static const struct uhc_stm32_config uhc_config_##node_id = {                                  \
		.irq = DT_IRQN(node_id),                                                                   \
		.pcfg = PINCTRL_DT_DEV_CONFIG_GET(node_id),                                                \
		.vbus_enable_gpio = GPIO_DT_SPEC_GET_OR(node_id, vbus_gpios, {0}),                         \
		.clocks = STM32_DT_CLOCKS(node_id),                                                        \
		.num_clock = DT_NUM_CLOCKS(node_id),                                                       \
		.num_host_channels = DT_PROP(node_id, num_host_channels),                                  \
		.phy = GET_PHY_INTERFACE(node_id),                                                         \
		.max_speed = GET_MAX_SPEED(node_id, core_max_speed),                                       \
		.ulpi_reset_gpio = GPIO_DT_SPEC_GET_OR(DT_PHY(node_id), reset_gpios, {0}),                 \
	};                                                                                             \
	                                                                                               \
	static HCD_HandleTypeDef uhc_stm32_hcd_##node_id = {                                           \
		.pData = (void*) DEVICE_DT_GET(node_id),                                                   \
		.Instance = (HCD_TypeDef *) DT_REG_ADDR(node_id),                                          \
		.Init = {                                                                                  \
			.Host_channels = DT_PROP(node_id, num_host_channels),                                  \
			.dma_enable = 0,                                                                       \
			.battery_charging_enable = DISABLE,                                                    \
			.use_external_vbus = DISABLE,                                                          \
			.vbus_sensing_enable = (                                                               \
				(GET_PHY_INTERFACE(node_id) == PHY_EXTERNAL_ULPI) ? ENABLE : DISABLE               \
			),                                                                                     \
			.speed = SPEED_TO_HCD_INIT_SPEED(GET_MAX_SPEED(node_id, core_max_speed)),              \
			.phy_itface = uhc_config_##node_id.phy,                                                \
		},                                                                                         \
	};                                                                                             \
	                                                                                               \
	static struct uhc_stm32_data uhc_priv_data_##node_id = {                                       \
		.hcd_ptr = &(uhc_stm32_hcd_##node_id),                                                     \
	};                                                                                             \
	                                                                                               \
	static struct uhc_data uhc_data_##node_id = {                                                  \
		.mutex = Z_MUTEX_INITIALIZER(uhc_data_##node_id.mutex),                                    \
		.priv = &uhc_priv_data_##node_id,                                                          \
	};                                                                                             \
	                                                                                               \
	static K_THREAD_STACK_DEFINE(                                                                  \
		uhc_work_thread_stack_##node_id,                                                           \
		CONFIG_UHC_STM32_DRV_THREAD_STACK_SIZE                                                     \
	);                                                                                             \
	                                                                                               \
	UHC_STM32_INIT_FUNC(node_id);                                                                  \
	                                                                                               \
	DEVICE_DT_DEFINE(node_id, UHC_STM32_INIT_FUNC_NAME(node_id), NULL,                             \
	                 &uhc_data_##node_id, &uhc_config_##node_id,                                   \
	                 POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                              \
	                 &uhc_stm32_api);


#define STM32_USB_INIT(node_id)                                                                    \
	BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_PHY(node_id), DT_PHY_COMPAT_EMBEDDED_FS),                   \
		"Invalid PHY defined in device tree");                                                     \
	                                                                                               \
	STM32_INIT_COMMON(node_id, USB_SPEED_FULL)

#define STM32_OTGFS_INIT(node_id)                                                                  \
	BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_PHY(node_id), DT_PHY_COMPAT_EMBEDDED_FS),                   \
		"Invalid PHY defined in device tree");                                                     \                                                                               \
	                                                                                               \
   STM32_INIT_COMMON(node_id, USB_SPEED_FULL)

#define STM32_OTGHS_INIT(node_id)                                                                  \
	STM32_INIT_COMMON(node_id, USB_SPEED_HIGH)


DT_FOREACH_STATUS_OKAY(st_stm32_usb, STM32_USB_INIT)
DT_FOREACH_STATUS_OKAY(st_stm32_otgfs, STM32_OTGFS_INIT)
DT_FOREACH_STATUS_OKAY(st_stm32_otghs, STM32_OTGHS_INIT)
