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
// printf
#include <stdio.h>

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
#include "hardware/esp32s3_dma.h"

#include "esp32s3-eye.h"

#undef CONFIG_ESP32S3_EYE_CAM_XCLK_MHZ
#define CONFIG_ESP32S3_EYE_CAM_XCLK_MHZ 15
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_ESP32S3_DEFAULT_CPU_FREQ_240 && \
  (CONFIG_ESP32S3_EYE_CAM_XCLK_MHZ % 3) == 0
  /* Use PLL=240MHz as clock resource */
#  define ESP32S3_CAM_CLK_SEL   2
#  define ESP32S3_CAM_CLK_MHZ   240
#else
  /* Use PLL=160MHz as clock resource */
#  define ESP32S3_CAM_CLK_SEL     3
#  define ESP32S3_CAM_CLK_MHZ     160
#endif

#define ESP32S3_CAM_CLK_N         (ESP32S3_CAM_CLK_MHZ / \
                                   CONFIG_ESP32S3_EYE_CAM_XCLK_MHZ)
#define ESP32S3_CAM_CLK_RES       (ESP32S3_CAM_CLK_MHZ % \
                                   CONFIG_ESP32S3_EYE_CAM_XCLK_MHZ)

#define ESP32S3_CAM_FB_SIZE (320*300*2)

#undef ESP32S3_DMA_DATALEN_MAX
#define ESP32S3_DMA_DATALEN_MAX 1279

#define ESP32S3_CAM_DMADESC_NUM   (ESP32S3_CAM_FB_SIZE / \
                                   ESP32S3_DMA_DATALEN_MAX + 1)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int dma_channel;
static void *framebuffer;
struct esp32s3_dmadesc_s dma_descriptors[ESP32S3_CAM_DMADESC_NUM];

static sem_t g_sem;

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static int ov2640_cam_stop(void);
static int ov2640_cam_start(void);

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

#ifdef OV2640_REGDEBUG
void ov2640_putreg32(const char *func, uint32_t value, uint32_t addr) {
  ginfo("%s: %" PRIx32 " <- %" PRIx32 "\n", func, addr, value);
  putreg32(value, addr);
}

uint32_t ov2640_getreg32(const char *func, uint32_t addr) {
  uint32_t value;
  value = getreg32(addr);
  ginfo("%s: %" PRIx32 " -> %" PRIx32 "\n", func, addr, value);
  return value;
}

#undef putreg32
#undef getreg32
#define putreg32(v, a) ov2640_putreg32(__func__, v, a)
#define getreg32(a) ov2640_getreg32(__func__, a)

#endif

/****************************************************************************
 * Name: dma_isr
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

static int IRAM_ATTR dma_isr(int irq, void *context, void *arg)
{
  uint32_t status = getreg32(DMA_IN_INT_ST_CH0_REG + (0xc0 * dma_channel));
  putreg32(status, DMA_IN_INT_CLR_CH0_REG + (0xc0 * dma_channel));

  if (status & DMA_IN_SUC_EOF_CH0_INT_ST_M)
  {
    ginfo("DMA ISR EOF!\n");

    uint32_t regvalue = getreg32(DMA_IN_SUC_EOF_DES_ADDR_CH0_REG + (0xc0 * dma_channel));

    ginfo("Last descriptor is %08x!\n", regvalue);

    if ((void *)regvalue == &dma_descriptors[(320*240*2/1280)-1])
      {
        ginfo("LAST ISR");
        ov2640_cam_stop();
        sem_post(&g_sem);
      }
  }
  if (status & DMA_IN_DONE_CH0_INT_ST_M)
    {
      ginfo("DMA ISR DONE!\n");
    }

  return 0;
}

/****************************************************************************
 * Name: cam_isr
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

static int IRAM_ATTR cam_isr(int irq, void *context, void *arg)
{
  uint32_t status = getreg32(LCD_CAM_LC_DMA_INT_ST_REG);

  putreg32(status, LCD_CAM_LC_DMA_INT_CLR_REG);

  if (status & LCD_CAM_CAM_HS_INT_ST_M)
    {
      ginfo("HS ISR!\n");
    }

  if (status & LCD_CAM_CAM_VSYNC_INT_ST_M)
    {
      ginfo("VSYNC ISR! .. starting DMA\n");

      uint32_t regval;

      /* Reset Rx and FIFO */

      regval  = getreg32(LCD_CAM_CAM_CTRL1_REG);
      regval |= LCD_CAM_CAM_RESET_M | LCD_CAM_CAM_AFIFO_RESET_M;
      putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

      /* Reset DMA channel */

      regval = DMA_IN_RST_CH0_M | DMA_IN_DATA_BURST_EN_CH0;
      putreg32(regval, DMA_IN_CONF0_CH0_REG + (0xc0 * dma_channel));
      putreg32(regval & ~DMA_IN_RST_CH0_M, DMA_IN_CONF0_CH0_REG + (0xc0 * dma_channel));

      /* Load DMA */

      esp32s3_dma_load(dma_descriptors, dma_channel, false);

      /* Start DMA */
      
      esp32s3_dma_enable(dma_channel, false);
    }

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

  return 0;
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
                                        CONFIG_ESP32S3_EYE_CAM_XCLK_MHZ);
  clk_b = ESP32S3_CAM_CLK_RES / divisor;
  clk_a = CONFIG_ESP32S3_EYE_CAM_XCLK_MHZ / divisor;

  ginfo("divisor=%d\n", divisor);
#else
  clk_b = clk_a = 0;
#endif

  ginfo("XCLK=%d/(%d + %d/%d)\n", ESP32S3_CAM_CLK_MHZ,
          ESP32S3_CAM_CLK_N, clk_b, clk_a);

  regval = 
          // LCD_CAM_CAM_VS_EOF_EN_M | 
          LCD_CAM_CAM_LINE_INT_EN_M |
          (4 << LCD_CAM_CAM_VSYNC_FILTER_THRES_S) |
          (ESP32S3_CAM_CLK_SEL << LCD_CAM_CAM_CLK_SEL_S) |
          (ESP32S3_CAM_CLK_N << LCD_CAM_CAM_CLKM_DIV_NUM_S) |
          (clk_a << LCD_CAM_CAM_CLKM_DIV_A_S) |
          (clk_b << LCD_CAM_CAM_CLKM_DIV_B_S);
  putreg32(regval, LCD_CAM_CAM_CTRL_REG);

  regval = LCD_CAM_CAM_VSYNC_FILTER_EN_M |
         ((3840-1) << LCD_CAM_CAM_REC_DATA_BYTELEN_S) |
         ((5) << LCD_CAM_CAM_LINE_INT_NUM_S) |
        //  ((40000-1) << LCD_CAM_CAM_REC_DATA_BYTELEN_S) |
          (ESP32S3_CAM_CLK_N << LCD_CAM_CAM_CLKM_DIV_NUM_S) |
          (clk_a << LCD_CAM_CAM_CLKM_DIV_A_S) |
          (clk_b << LCD_CAM_CAM_CLKM_DIV_B_S);
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  // TODO: converter?

  /* Update CAM parameters before start */
  regval  = getreg32(LCD_CAM_CAM_CTRL_REG);
  regval |= LCD_CAM_CAM_UPDATE_REG_M;
  putreg32(regval, LCD_CAM_CAM_CTRL_REG);

  /* Reset Rx and FIFO */
  regval  = getreg32(LCD_CAM_CAM_CTRL1_REG);
  regval |= LCD_CAM_CAM_RESET_M | LCD_CAM_CAM_AFIFO_RESET_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  return 0;
}

static int ov2640_cam_init_isr(void)
{
  spinlock_t lock;
  int flags = spin_lock_irqsave(&lock);

  int cpu = up_cpu_index();

  int cpuint;
  int attach;

  /* DMA */

  cpuint = esp32s3_setup_irq(cpu,
                             ESP32S3_PERIPH_DMA_IN_CH0 + dma_channel,
                             ESP32S3_INT_PRIO_DEF,
                             ESP32S3_CPUINT_LEVEL);

  DEBUGASSERT(cpuint >= 0);

  attach = irq_attach(ESP32S3_IRQ_DMA_IN_CH0 + dma_channel, dma_isr, 0);
  DEBUGASSERT(attach == 0);

  /* VSYNC */

  cpuint = esp32s3_setup_irq(cpu,
                             ESP32S3_PERIPH_LCD_CAM,
                             ESP32S3_INT_PRIO_DEF,
                             ESP32S3_CPUINT_LEVEL);
  DEBUGASSERT(cpuint >= 0);

  attach = irq_attach(ESP32S3_IRQ_LCD_CAM, cam_isr, 0);
  DEBUGASSERT(attach == 0);

  spin_unlock_irqrestore(&lock, flags);

  uint32_t regval;

  /* DMA */

  regval = DMA_IN_SUC_EOF_CH0_INT_ENA_M;
  // regval = DMA_IN_DONE_CH0_INT_ENA_M;
  // regval = DMA_IN_SUC_EOF_CH0_INT_ENA_M | DMA_IN_DONE_CH0_INT_ENA_M;
  putreg32(regval, DMA_IN_INT_ENA_CH0_REG);

  /* VSYNC */

  // regval = LCD_CAM_CAM_VSYNC_INT_ENA_M | LCD_CAM_CAM_HS_INT_ENA_M;
  regval = LCD_CAM_CAM_VSYNC_INT_ENA_M;
  putreg32(regval, LCD_CAM_LC_DMA_INT_ENA_REG);

  return 0;
}

static int ov2640_dma_init(void) {

  esp32s3_dma_init();

  dma_channel = esp32s3_dma_request(ESP32S3_DMA_PERIPH_LCDCAM,
                                    10, 1, false);

  ginfo("Allocated DMA channel %d\n", dma_channel);
  DEBUGASSERT(dma_channel >= 0);

  // esp32s3_dma_set_ext_memblk(dma_channel,
  //                            true,
  //                            ESP32S3_DMA_EXT_MEMBLK_64B);

  framebuffer = memalign(64, ESP32S3_CAM_FB_SIZE);
  DEBUGASSERT(framebuffer != NULL);
  memset(framebuffer, 0xff, ESP32S3_CAM_FB_SIZE);

  // esp32s3_dma_setup(dma_descriptors,
  //                   ESP32S3_CAM_DMADESC_NUM,
  //                   framebuffer,
  //                   ESP32S3_CAM_FB_SIZE,
  //                   false);

  esp32s3_dma_setup2(dma_descriptors,
                    ESP32S3_CAM_FB_SIZE/(ESP32S3_DMA_DATALEN_MAX+1),
                    framebuffer,
                    ESP32S3_CAM_FB_SIZE,
                    false, ESP32S3_DMA_DATALEN_MAX+1);

  return 0;
}

static int ov2640_cam_stop(void)
{
  uint32_t regval;

  /* Stop CAM */

  regval  = getreg32(LCD_CAM_CAM_CTRL1_REG);
  regval &= ~LCD_CAM_CAM_START_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  return 0;
}


static int ov2640_cam_start(void)
{
  uint32_t regval;

  /* Stop CAM */

  regval  = getreg32(LCD_CAM_CAM_CTRL1_REG);
  regval &= ~LCD_CAM_CAM_START_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  /* Reset Rx and FIFO */

  regval  = getreg32(LCD_CAM_CAM_CTRL1_REG);
  regval |= LCD_CAM_CAM_RESET_M | LCD_CAM_CAM_AFIFO_RESET_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  // /* Reset DMA */

  // regval = DMA_IN_RST_CH0_M;
  // putreg32(regval, DMA_IN_CONF0_CH0_REG);
  // putreg32(regval & ~DMA_IN_RST_CH0_M, DMA_IN_CONF0_CH0_REG);

  /* Start CAM */

  regval  = getreg32(LCD_CAM_CAM_CTRL1_REG);
  regval |= LCD_CAM_CAM_START_M;
  putreg32(regval, LCD_CAM_CAM_CTRL1_REG);

  ginfo("%03d (0x%08x)\n", 0, dma_descriptors->ctrl);
  ginfo("%03d (0x%08x)\n", ESP32S3_CAM_DMADESC_NUM-1, dma_descriptors[ESP32S3_CAM_DMADESC_NUM-1].ctrl);

  // /* Load DMA */

  // esp32s3_dma_load(dma_descriptors, dma_channel, false);

  // /* Start DMA */
        
  // esp32s3_dma_enable(dma_channel, false);


  return 0;
}

int ov2640_snap(struct esp32s3_dmadesc_s ** desc)
{
  sem_init(&g_sem, 0, 0);
 
  ov2640_cam_start();

  sem_wait(&g_sem);
 
  *desc = dma_descriptors;

  return 0;
}

int ov2640_camera_initialize(void)
{
  int ret;

  ret = ov2640_gpio_config();
  if (ret > 0)
  {
    gerr("ERROR: Failed on ov2640_gpio_config\n");
    return EXIT_FAILURE;
  }

  ret = ov2640_cam_config();
  if (ret > 0)
  {
    gerr("ERROR: Failed on ov2640_cam_config\n");
    return EXIT_FAILURE;
  }

  /* Init I2C */
  struct i2c_master_s *i2c = esp32s3_i2cbus_initialize(ESP32S3_EYE_CAM_I2C_BUS);
  if (!i2c)
  {
    gerr("ERROR: Failed to initialize I2C %d\n", ESP32S3_EYE_CAM_I2C_BUS);
    return EXIT_FAILURE;
  }

  /* Init OV2640 */
  ret = ov2640_initialize(i2c);
  if (ret < 0)
    {
      gerr("ERROR: Failed to initialize the OV2640: %d\n", ret);
      return EXIT_FAILURE;
    }

  ret = ov2640_dma_init();
  if (ret > 0)
  {
    gerr("ERROR: Failed on ov2640_dma_init\n");
    return EXIT_FAILURE;
  }

  ret = ov2640_cam_init_isr();
  if (ret > 0)
  {
    gerr("ERROR: Failed on ov2640_cam_init_isr\n");
    return EXIT_FAILURE;
  }

  // ginfo("will start camera ov2640_cam_start\n");
  // ret = ov2640_cam_start();
  // if (ret > 0)
  // {
  //   gerr("ERROR: Failed on ov2640_cam_start\n");
  //   return EXIT_FAILURE;
  // }

  return EXIT_SUCCESS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
