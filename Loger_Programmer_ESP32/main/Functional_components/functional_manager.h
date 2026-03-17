
#pragma once

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "util.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tusb.h"
#include "msc.h"
#include "serial_handler.h"
#include "serial_bridge.h"
#include "hal/usb_phy_types.h"
#include "rom/gpio.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_mac.h"
#include "esp_private/usb_phy.h"
#include "eub_vendord.h"
#include "debug_probe.h"
#include "usb_defs.h"
#include "led_io.h"
#include "wifi_control.h"
#include "spi_handler_v2.h"
#include "logger_handler.h"

#define BRIDGE 0
#define LOGGER 1
#define WEB_FACE 2

#define COMP_SET(mask)      (g_initialized |= (mask))
#define COMP_CLEAR(mask)    (g_initialized &= ~(mask))
#define COMP_IS_INIT(mask)  ((g_initialized & (mask)) == (mask))

typedef enum {
    COMP_NONE       = 0,
    COMP_TUSB       = (1 << 0),  // TinyUSB — всегда нужен
    COMP_BRIDGE     = (1 << 1),  // serial_bridge + serial_handler
    COMP_MSC        = (1 << 2),  // MSC flasher
    COMP_DEBUG_PROB = (1 << 3),  // JTAG debug probe
    COMP_SPI        = (1 << 4),  // SPI slave
    COMP_LOGGER     = (1 << 5),  // Ring buffer + logger task
    COMP_WIFI       = (1 << 6),  // Wi-Fi + HTTP server
} ComponentMask_t;

typedef enum{
    SET_BRIDGE,
    SET_LOGGER,
    SET_WEB,
    RESET_BRIDGE,
    RESET_LOGGER,
    RESET_WEB,
}SetCommand_t;

typedef enum{
    Rx_DATA_NOP,
    RX_DATA_CMPT
}RX_USB_data_state_t;


void Function_Init(void);