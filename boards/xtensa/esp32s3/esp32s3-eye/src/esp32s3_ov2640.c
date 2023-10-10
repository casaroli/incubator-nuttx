/****************************************************************************
 * boards/arm/sama5/sama5d3x-ek/src/sam_ov2640.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/fb.h>
#include <nuttx/video/ov2640.h>

#include "esp32s3_clockconfig.h"
#include "esp32s3_gpio.h"
#include "esp32s3_i2c.h"
#include "esp32s3_periph.h"

#include "xtensa.h"
#include "hardware/esp32s3_system.h"
#include "hardware/esp32s3_gpio_sigmap.h"
#include "hardware/esp32s3_lcd_cam.h"

#include "esp32s3-eye.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_ESP32S3_DEFAULT_CPU_FREQ_240 && (OV2640_FREQUENCY_MHZ % 3) == 0
  /* Use PLL=240MHz as clock resource */
#  define ESP32S3_CAM_CLK_SEL   2
#  define ESP32S3_CAM_CLK_MHZ   240
#else
  /* Use PLL=160MHz as clock resource */
#  define ESP32S3_CAM_CLK_SEL     3
#  define ESP32S3_CAM_CLK_MHZ     160
#endif

#define ESP32S3_CAM_CLK_N         (ESP32S3_CAM_CLK_MHZ / \
                                   OV2640_FREQUENCY_MHZ)
#define ESP32S3_CAM_CLK_RES       (ESP32S3_CAM_CLK_MHZ % \
                                   OV2640_FREQUENCY_MHZ)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: max_common_divisor
 *
 * Description:
 *   Calculate maxium common divisor.
 *
 * Input Parameters:
 *   a - Calculation parameter a
 *   b - Calculation parameter b
 *
 * Returned Value:
 *   Maximum common divisor.
 *
 ****************************************************************************/

static inline uint32_t max_common_divisor(uint32_t a, uint32_t b)
{
  uint32_t c = a % b;

  while (c)
    {
      a = b;
      b = c;
      c = a % b;
    }

  return b;
}

static int ov2640_config_gpio(void)
{
  /* XCLK out */
  esp32s3_configgpio(15, OUTPUT);
  esp32s3_gpio_matrix_out(15, CAM_CLK_IDX, 0, 0);

  /* PCLK, HREF, VSYNC */
  esp32s3_configgpio(13, INPUT);
  esp32s3_gpio_matrix_in(13, CAM_PCLK_IDX, 0);

  esp32s3_configgpio(7, INPUT);
  esp32s3_gpio_matrix_in(7, CAM_H_ENABLE_IDX, 0);

  esp32s3_configgpio(6, INPUT);
  esp32s3_gpio_matrix_in(6, CAM_V_SYNC_IDX, 0);

  /* data pins */
  int data_pins[8] = {
    11, 9, 8, 10, 12, 18, 17, 16
  };
  for (int i = 0; i < 8; i++) {
    esp32s3_configgpio(data_pins[i], INPUT);
    esp32s3_gpio_matrix_in(data_pins[i], CAM_DATA_IN0_IDX + i, 0);
  }
}

static int ov2640_config_cam(void) {
  /* Enable clock to peripheral */
  esp32s3_periph_module_enable(PERIPH_LCD_CAM_MODULE);

  /* Config XCLK */
  uint32_t regval;
  uint32_t clk_a;
  uint32_t clk_b;

#if ESP32S3_CAM_CLK_RES != 0
  uint32_t divisor = max_common_divisor(ESP32S3_CAM_CLK_RES,
                                        CONFIG_ESP32S3_CAM_CLOCK_MHZ);
  clk_b = ESP32S3_CAM_CLK_RES / divisor;
  clk_a = CONFIG_ESP32S3_CAM_CLOCK_MHZ / divisor;

  ginfo("divisor=%d\n", divisor);
#else
  clk_b = clk_a = 0;
#endif

  ginfo("XCLK=%d/(%d + %d/%d)\n", ESP32S3_CAM_CLK_MHZ,
          ESP32S3_CAM_CLK_N, clk_b, clk_a);

  regval = (ESP32S3_CAM_CLK_SEL << LCD_CAM_CAM_CLK_SEL_S) |
          (ESP32S3_CAM_CLK_N << LCD_CAM_CAM_CLKM_DIV_NUM_S) |
          (clk_a << LCD_CAM_CAM_CLKM_DIV_A_S) |
          (clk_b << LCD_CAM_CAM_CLKM_DIV_B_S);
  ginfo("%" PRIx32 " <-%" PRIx32 "\n", LCD_CAM_CAM_CTRL_REG, regval);
  putreg32(regval, LCD_CAM_CAM_CTRL_REG);

}

int ov2460_start_cam(void)
{
  uint32_t regval;

  /* Update CAM parameters before start */
  regval  = getreg32(LCD_CAM_CAM_CTRL_REG);
  ginfo("%" PRIx32 " ->%" PRIx32 "\n", LCD_CAM_CAM_CTRL_REG, regval);
  regval |= LCD_CAM_CAM_UPDATE_REG_M;
  ginfo("%" PRIx32 " <-%" PRIx32 "\n", LCD_CAM_CAM_CTRL_REG, regval);
  putreg32(regval, LCD_CAM_CAM_CTRL_REG);

  /* Start CAM */
  regval  = getreg32(LCD_CAM_CAM_CTRL1_REG);
  ginfo("%" PRIx32 " ->%" PRIx32 "\n", LCD_CAM_CAM_CTRL_REG, regval);
  regval |= LCD_CAM_CAM_START_M;
  ginfo("%" PRIx32 " <-%" PRIx32 "\n", LCD_CAM_CAM_CTRL_REG, regval);
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);
}

int ov2640_camera_initialize(void)
{

  ov2640_config_gpio();

  ov2640_config_cam();

  ov2460_start_cam();

  /* Init I2C */
  struct i2c_master_s *i2c = esp32s3_i2cbus_initialize(OV2640_BUS);
  if (!i2c)
  {
    gerr("ERROR: Failed to initialize TWI%d\n", OV2640_BUS);
    return EXIT_FAILURE;
  }

  /* Init OV2640 */
  int ret = ov2640_initialize(i2c);
  if (ret < 0)
    {
      gerr("ERROR: Failed to initialize the OV2640: %d\n", ret);
      return EXIT_FAILURE;
    }
  return EXIT_SUCCESS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
