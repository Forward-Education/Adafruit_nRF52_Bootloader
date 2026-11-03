/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Ha Thach for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


/*
 * Forward Education MicroRGB REV_A (nRF52833, hw-microbit-clone, ordered 2026-10-31).
 * Pins from hw-microbit-clone tools/pinmap.json. P0.11 to P0.14 are MATRIX_EN, MATRIX_DIN,
 * JD_PWR_EN and EDGE_P20 here, so the DK's board (pca10100) must not run on REV_A.
 */

#ifndef MICRORGB_REVA_H
#define MICRORGB_REVA_H

/*------------------------------------------------------------------*/
/* LED
 *------------------------------------------------------------------*/
// STATUS_LED (D34 through R22 to GND): active high
#define LEDS_NUMBER       1
#define LED_PRIMARY_PIN   PINNUM(0, 1)
#define LED_STATE_ON      1

// Bootloader status colour on the 5x5 XL-1615 matrix: MATRIX_DIN, with MATRIX_EN switching on the
// MT3608 boost. JD_PWR_EN stays low (100k pull-down), so the Jacdac port stays off (invariant 9).
#define LED_NEOPIXEL         PINNUM(0, 12)
#define NEOPIXEL_POWER_PIN   PINNUM(0, 11)
#define NEOPIXELS_NUMBER     25
#define BOARD_RGB_BRIGHTNESS 0x040404

/*------------------------------------------------------------------*/
/* BUTTON
 *------------------------------------------------------------------*/
// Button A (EDGE_P5, 10k pull-up R19) held at reset: stay in the bootloader. No BLE OTA button:
// OTA in this build is untested with S113.
#define BUTTON_DFU     PINNUM(1, 9)
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

/*------------------------------------------------------------------*/
/* Power
 *------------------------------------------------------------------*/
// L1 is the 10 uH, 300 mA part (C6980427): DC/DC allowed
#define ENABLE_DCDC_1  1   // REG1 (DCDCEN); DCDC_0 is the nRF52840's REG0

//--------------------------------------------------------------------+
// BLE OTA
//--------------------------------------------------------------------+
#define BLEDIS_MANUFACTURER "Forward Education"
#define BLEDIS_MODEL        "MicroRGB REV_A"

//--------------------------------------------------------------------+
// USB
//--------------------------------------------------------------------+
// pid.codes TEST IDs, not for shipping (architecture.md open decision 2). The application uses
// 0x1209:0x0001.
#define USB_DESC_VID          0x1209
#define USB_DESC_UF2_PID      0x0002
#define USB_DESC_CDC_ONLY_PID 0x0002

#define UF2_PRODUCT_NAME      "Forward MicroRGB"
#define UF2_VOLUME_LABEL      "MICRORGB"
#define UF2_BOARD_ID          "MicroRGB-REV_A"
#define UF2_INDEX_URL         "https://forwardedu.com"

#endif // MICRORGB_REVA_H
