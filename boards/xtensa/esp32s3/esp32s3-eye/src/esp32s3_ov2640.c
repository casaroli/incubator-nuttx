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
#include <nuttx/spinlock.h>

#include <stdlib.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/fb.h>
#include <nuttx/video/ov2640.h>

#include "esp32s3_clockconfig.h"
#include "esp32s3_gpio.h"
#include "esp32s3_dma.h"
#include "esp32s3_i2c.h"
#include "esp32s3_irq.h"
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

/****************************************************************************
 * Name: cam_interrupt
 *
 * Description:
 *   Start sending next frame to LCD.
 *
 * Input Parameters:
 *   irq     - The IRQ number of the interrupt.
 *   context - The register state save array at the time of the interrupt.
 *   arg     - Not used
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int IRAM_ATTR cam_interrupt(int irq, void *context, void *arg)
{
  uint32_t regval;
  // struct esp32s3_lcd_s *priv = &g_lcd_priv;
  uint32_t status = getreg32(LCD_CAM_LC_DMA_INT_ST_REG);

  putreg32(status, LCD_CAM_LC_DMA_INT_CLR_REG);

  if (status & LCD_CAM_CAM_VSYNC_INT_ST_M)
    {
      // ginfo("VSYNC ISR!\n");
    }
  else {
      gerr("Unknown ISR status%x!\n", status);

  }
#if 0
  if (status & LCD_CAM_LCD_VSYNC_INT_ST_M)
    {
      /* Stop TX */

      regval  = esp32s3_lcd_getreg(LCD_CAM_LCD_USER_REG);
      regval &= ~LCD_CAM_LCD_START_M;
      esp32s3_lcd_putreg(LCD_CAM_LCD_USER_REG, regval);

      regval  = esp32s3_lcd_getreg(LCD_CAM_LCD_USER_REG);
      regval |= LCD_CAM_LCD_UPDATE_REG_M;
      esp32s3_lcd_putreg(LCD_CAM_LCD_USER_REG, regval);

      /* Clear TX fifo */

      regval  = esp32s3_lcd_getreg(LCD_CAM_LCD_MISC_REG);
      regval |= LCD_CAM_LCD_AFIFO_RESET_M;
      esp32s3_lcd_putreg(LCD_CAM_LCD_MISC_REG, regval);

#if ESP32S3_LCD_LAYERS > 1
      priv->cur_layer = (priv->cur_layer + 1) % ESP32S3_LCD_LAYERS;

      esp32s3_dma_load(CURRENT_LAYER(priv)->dmadesc,
                       priv->dma_channel,
                       true);
#endif

#ifndef CONFIG_FB_UPDATE
      /* Write framebuffer data from D-cache to PSRAM */

      cache_writeback_addr(CURRENT_LAYER(priv)->framebuffer,
                           ESP32S3_LCD_FB_SIZE);
#endif

      /* Enable DMA TX */

      esp32s3_dma_enable(priv->dma_channel, true);

      /* Update LCD parameters and start TX */

      regval  = esp32s3_lcd_getreg(LCD_CAM_LCD_USER_REG);
      regval |= LCD_CAM_LCD_UPDATE_REG_M;
      esp32s3_lcd_putreg(LCD_CAM_LCD_USER_REG, regval);

      regval  = esp32s3_lcd_getreg(LCD_CAM_LCD_USER_REG);
      regval |= LCD_CAM_LCD_START_M;
      esp32s3_lcd_putreg(LCD_CAM_LCD_USER_REG, regval);
    }
#endif
  return 0;
}


static int ov2640_gpio_config(void)
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

static int ov2640_cam_config(void) {
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

  regval = (4 << LCD_CAM_CAM_VSYNC_FILTER_THRES_S) |
          (ESP32S3_CAM_CLK_SEL << LCD_CAM_CAM_CLK_SEL_S) |
          (ESP32S3_CAM_CLK_N << LCD_CAM_CAM_CLKM_DIV_NUM_S) |
          (clk_a << LCD_CAM_CAM_CLKM_DIV_A_S) |
          (clk_b << LCD_CAM_CAM_CLKM_DIV_B_S);
  ginfo("%" PRIx32 " <-%" PRIx32 "\n", LCD_CAM_CAM_CTRL_REG, regval);
  putreg32(regval, LCD_CAM_CAM_CTRL_REG);

  regval = LCD_CAM_CAM_VSYNC_FILTER_EN_M |
          ((4092-1) << LCD_CAM_CAM_REC_DATA_BYTELEN_S) |
          (ESP32S3_CAM_CLK_N << LCD_CAM_CAM_CLKM_DIV_NUM_S) |
          (clk_a << LCD_CAM_CAM_CLKM_DIV_A_S) |
          (clk_b << LCD_CAM_CAM_CLKM_DIV_B_S);
  ginfo("%" PRIx32 " <-%" PRIx32 "\n", LCD_CAM_CAM_CTRL1_REG, regval);
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  // TODO: converter?

}

static int ov2460_cam_start(void)
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

static int ov2460_cam_init_isr(void)
{
  spinlock_t lock;
  int flags = spin_lock_irqsave(&lock);

  int cpu = up_cpu_index();

  // TODO: DMA
  // 	esp_err_t ret = ESP_OK;
  //   ret = esp_intr_alloc_intrstatus(gdma_periph_signals.groups[0].pairs[cam->dma_num].rx_irq_id,
  //                                    ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM,
  //                                    (uint32_t)&GDMA.channel[cam->dma_num].in.int_st, GDMA_IN_SUC_EOF_CH0_INT_ST_M,
  //                                    ll_cam_dma_isr, cam, &cam->dma_intr_handle);
  //   if (ret != ESP_OK) {
  //       ESP_LOGE(TAG, "DMA interrupt allocation of camera failed");
	// 	return ret;
	// }

  /* VSYNC */
  int cpuint = esp32s3_setup_irq(cpu,
                                   ESP32S3_PERIPH_LCD_CAM,
                                   ESP32S3_INT_PRIO_DEF,
                                   ESP32S3_CPUINT_LEVEL);
  DEBUGASSERT(cpuint >= 0);

  int attach = irq_attach(ESP32S3_IRQ_LCD_CAM, cam_interrupt, 0);
  DEBUGASSERT(attach == 0);

  spin_unlock_irqrestore(&lock, flags);

  uint32_t regval = LCD_CAM_CAM_VSYNC_INT_ENA_M;
  putreg32(regval, LCD_CAM_LC_DMA_INT_ENA_REG);
}

static int ov2640_dma_init(void) {
  // struct esp32s3_lcd_s *priv = &g_lcd_priv;

  esp32s3_dma_init();

  int dma_channel = esp32s3_dma_request(ESP32S3_DMA_PERIPH_LCDCAM,
                                          10, 1, true);
  DEBUGASSERT(dma_channel >= 0);


  // esp32s3_dma_set_ext_memblk(dma_channel,
  //                            true,
  //                            ESP32S3_DMA_EXT_MEMBLK_64B);

  // for (int i = 0; i < ESP32S3_LCD_LAYERS; i++)
  //   {
  //     struct esp32s3_layer_s *layer = &priv->layer[i];

  //     layer->framebuffer = memalign(64, ESP32S3_LCD_FB_SIZE);
  //     DEBUGASSERT(layer->framebuffer != NULL);
  //     memset(layer->framebuffer, 0, ESP32S3_LCD_FB_SIZE);

  //     esp32s3_dma_setup(layer->dmadesc,
  //                       ESP32S3_LCD_DMADESC_NUM,
  //                       layer->framebuffer,
  //                       ESP32S3_LCD_FB_SIZE,
  //                       true);
  //   }


}

int ov2640_camera_initialize(void)
{

  ov2640_gpio_config();

  ov2640_cam_config();

  ov2460_cam_start();

  // TODO: dma init

  ov2460_cam_init_isr();

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
