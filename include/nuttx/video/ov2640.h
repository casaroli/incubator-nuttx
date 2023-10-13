/****************************************************************************
 * include/nuttx/video/ov2640.h
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

#ifndef __INCLUDE_NUTTX_VIDEO_OV2640_H
#define __INCLUDE_NUTTX_VIDEO_OV2640_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 7-bit I2C address.  Default: 0x21 */

#ifndef CONFIG_OV2640_I2CADDR
#  define CONFIG_OV2640_I2CADDR 0x21
#endif

#ifdef CONFIG_OV2640_JPEG
#  undef CONFIG_OV2640_QCIF_RESOLUTION
#  undef CONFIG_OV2640_QVGA_RESOLUTION
#  undef CONFIG_OV2640_CIF_RESOLUTION
#  undef CONFIG_OV2640_VGA_RESOLUTION
#  undef CONFIG_OV2640_SVGA_RESOLUTION
#  undef CONFIG_OV2640_XVGA_RESOLUTION
#  undef CONFIG_OV2640_SXGA_RESOLUTION
#  undef CONFIG_OV2640_UXGA_RESOLUTION

#else
#  undef CONFIG_OV2640_JPEG_QCIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_CIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_VGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_XVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SXVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_UXGA_RESOLUTION
#endif

#if defined(CONFIG_OV2640_QCIF_RESOLUTION) || \
    defined(CONFIG_OV2640_JPEG_QCIF_RESOLUTION)

#  define OV2640_IMAGE_WIDTH  176
#  define OV2640_IMAGE_HEIGHT 144

#  undef CONFIG_OV2640_QVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QVGA_RESOLUTION
#  undef CONFIG_OV2640_CIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_CIF_RESOLUTION
#  undef CONFIG_OV2640_VGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_VGA_RESOLUTION
#  undef CONFIG_OV2640_SVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SVGA_RESOLUTION
#  undef CONFIG_OV2640_XVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_XVGA_RESOLUTION
#  undef CONFIG_OV2640_SXGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SXVGA_RESOLUTION
#  undef CONFIG_OV2640_UXGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_UXGA_RESOLUTION

#elif defined(CONFIG_OV2640_QVGA_RESOLUTION) || \
      defined(CONFIG_OV2640_JPEG_QVGA_RESOLUTION)

#  define OV2640_IMAGE_WIDTH  320
#  define OV2640_IMAGE_HEIGHT 240

#  undef CONFIG_OV2640_QCIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QCIF_RESOLUTION
#  undef CONFIG_OV2640_CIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_CIF_RESOLUTION
#  undef CONFIG_OV2640_VGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_VGA_RESOLUTION
#  undef CONFIG_OV2640_SVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SVGA_RESOLUTION
#  undef CONFIG_OV2640_XVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_XVGA_RESOLUTION
#  undef CONFIG_OV2640_SXGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SXVGA_RESOLUTION
#  undef CONFIG_OV2640_UXGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_UXGA_RESOLUTION

#elif defined(CONFIG_OV2640_CIF_RESOLUTION) || \
      defined(CONFIG_OV2640_JPEG_CIF_RESOLUTION)

#  define OV2640_IMAGE_WIDTH  352
#  define OV2640_IMAGE_HEIGHT  288

#  undef CONFIG_OV2640_QCIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QCIF_RESOLUTION
#  undef CONFIG_OV2640_QVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QVGA_RESOLUTION
#  undef CONFIG_OV2640_VGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_VGA_RESOLUTION
#  undef CONFIG_OV2640_SVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SVGA_RESOLUTION
#  undef CONFIG_OV2640_XVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_XVGA_RESOLUTION
#  undef CONFIG_OV2640_SXGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SXVGA_RESOLUTION
#  undef CONFIG_OV2640_UXGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_UXGA_RESOLUTION

#elif defined(CONFIG_OV2640_VGA_RESOLUTION) || \
      defined(CONFIG_OV2640_JPEG_VGA_RESOLUTION)

#  define OV2640_IMAGE_WIDTH  640
#  define OV2640_IMAGE_HEIGHT 480

#  undef CONFIG_OV2640_QCIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QCIF_RESOLUTION
#  undef CONFIG_OV2640_QVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QVGA_RESOLUTION
#  undef CONFIG_OV2640_CIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_CIF_RESOLUTION
#  undef CONFIG_OV2640_SVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SVGA_RESOLUTION
#  undef CONFIG_OV2640_XVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_XVGA_RESOLUTION
#  undef CONFIG_OV2640_SXGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SXVGA_RESOLUTION
#  undef CONFIG_OV2640_UXGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_UXGA_RESOLUTION

#elif defined(CONFIG_OV2640_SVGA_RESOLUTION) || \
      defined(CONFIG_OV2640_JPEG_SVGA_RESOLUTION)

#  define OV2640_IMAGE_WIDTH  800
#  define OV2640_IMAGE_HEIGHT 600

#  undef CONFIG_OV2640_QCIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QCIF_RESOLUTION
#  undef CONFIG_OV2640_QVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QVGA_RESOLUTION
#  undef CONFIG_OV2640_CIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_CIF_RESOLUTION
#  undef CONFIG_OV2640_VGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_VGA_RESOLUTION
#  undef CONFIG_OV2640_XVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_XVGA_RESOLUTION
#  undef CONFIG_OV2640_SXGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SXVGA_RESOLUTION
#  undef CONFIG_OV2640_UXGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_UXGA_RESOLUTION

#elif defined(CONFIG_OV2640_XVGA_RESOLUTION) || \
      defined(CONFIG_OV2640_JPEG_XVGA_RESOLUTION)

#  define OV2640_IMAGE_WIDTH  1024
#  define OV2640_IMAGE_HEIGHT 768

#  undef CONFIG_OV2640_QCIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QCIF_RESOLUTION
#  undef CONFIG_OV2640_QVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QVGA_RESOLUTION
#  undef CONFIG_OV2640_CIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_CIF_RESOLUTION
#  undef CONFIG_OV2640_VGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_VGA_RESOLUTION
#  undef CONFIG_OV2640_SVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SVGA_RESOLUTION
#  undef CONFIG_OV2640_SXGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SXVGA_RESOLUTION
#  undef CONFIG_OV2640_UXGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_UXGA_RESOLUTION

#elif defined(CONFIG_OV2640_SXGA_RESOLUTION) || \
      defined(CONFIG_OV2640_JPEG_SXVGA_RESOLUTION)

#  define OV2640_IMAGE_WIDTH  1280
#  define OV2640_IMAGE_HEIGHT 1024

#  undef CONFIG_OV2640_QCIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QCIF_RESOLUTION
#  undef CONFIG_OV2640_QVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QVGA_RESOLUTION
#  undef CONFIG_OV2640_CIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_CIF_RESOLUTION
#  undef CONFIG_OV2640_VGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_VGA_RESOLUTION
#  undef CONFIG_OV2640_SVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SVGA_RESOLUTION
#  undef CONFIG_OV2640_XVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_XVGA_RESOLUTION
#  undef CONFIG_OV2640_UXGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_UXGA_RESOLUTION

#elif defined(CONFIG_OV2640_UXGA_RESOLUTION) || \
      defined(CONFIG_OV2640_JPEG_UXGA_RESOLUTION)

#  define OV2640_IMAGE_WIDTH  1600
#  define OV2640_IMAGE_HEIGHT 1200

#  undef CONFIG_OV2640_QCIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QCIF_RESOLUTION
#  undef CONFIG_OV2640_QVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_QVGA_RESOLUTION
#  undef CONFIG_OV2640_CIF_RESOLUTION
#  undef CONFIG_OV2640_JPEG_CIF_RESOLUTION
#  undef CONFIG_OV2640_VGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_VGA_RESOLUTION
#  undef CONFIG_OV2640_SVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SVGA_RESOLUTION
#  undef CONFIG_OV2640_XVGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_XVGA_RESOLUTION
#  undef CONFIG_OV2640_SXGA_RESOLUTION
#  undef CONFIG_OV2640_JPEG_SXVGA_RESOLUTION

#else
#  error Unknown Resolution
#endif

/* Chip identification */

#define OVR2640_MANUFACTURER_IDL 0xa2
#define OVR2640_MANUFACTURER_IDH 0x7f
#define OVR2640_PRODUCT_IDL      0x42
#define OVR2640_PRODUCT_IDH      0x26

/****************************************************************************
 * Name: ov2640_initialize
 *
 * Description:
 *   Initialize the OV2640 camera.
 *
 * Input Parameters:
 *   i2c - Reference to the I2C driver structure
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

struct i2c_master_s;
int ov2640_initialize(FAR struct i2c_master_s *i2c);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_VIDEO_OV2640_H */
