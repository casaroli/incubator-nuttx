/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_memorymap.h
 *
 * Generated from rp23xx.svd originally provided by
 *   Raspberry Pi (Trading) Ltd.
 *
 * Copyright 2020 (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_MEMORYMAP_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm_internal.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RP23XX_FLASH_BASE                0x10000000 /* -0x001fffff: FLASH memory space (2048KB) */
#define RP23XX_SRAM_BASE                 0x20000000 /* -0x20041fff: SRAM memory space (264KB) */

#define RP23XX_XIP_CTRL_BASE             0x14000000  /* QSPI flash execute-in-place block */
#define RP23XX_XIP_SSI_BASE              0x18000000
#define RP23XX_SYSINFO_BASE              0x40000000
#define RP23XX_SYSCFG_BASE               0x40008000  /* Register block for various chip control signals */
#define RP23XX_CLOCKS_BASE               0x40010000
#define RP23XX_RESETS_BASE               0x40020000
#define RP23XX_PSM_BASE                  0x40018000
#define RP23XX_IO_BANK0_BASE             0x40028000
#define RP23XX_IO_QSPI_BASE              0x40030000
#define RP23XX_PADS_BANK0_BASE           0x40038000
#define RP23XX_PADS_QSPI_BASE            0x40040000
#define RP23XX_XOSC_BASE                 0x40048000  /* Controls the crystal oscillator */
#define RP23XX_PLL_SYS_BASE              0x40050000
#define RP23XX_PLL_USB_BASE              0x40058000
#define RP23XX_BUSCTRL_BASE              0x40068000  /* Register block for busfabric control signals and performance counters */
#define RP23XX_UART0_BASE                0x40070000
#define RP23XX_UART1_BASE                0x40078000
#define RP23XX_UART_BASE(n)              (0x40070000 + (n) * 0x8000)
#define RP23XX_SPI0_BASE                 0x40080000
#define RP23XX_SPI1_BASE                 0x40088000
#define RP23XX_SPI_BASE(n)               (0x40080000 + (n) * 0x8000)
#define RP23XX_I2C0_BASE                 0x40090000  /* DW_apb_i2c address block */
#define RP23XX_I2C1_BASE                 0x40098000  /* DW_apb_i2c address block */
#define RP23XX_I2C_BASE(n)               (0x40090000 + (n) * 0x8000)
#define RP23XX_ADC_BASE                  0x400a0000  /* Control and data interface to SAR ADC */
#define RP23XX_PWM_BASE                  0x400a8000  /* Simple PWM */
#define RP23XX_TIMER_BASE                0x400b0000  /* Controls time and alarms time is a 64 bit value indicating the time in usec since power-on timeh is the top 32 bits of time & timel is the bottom 32 bits to change time write to timelw before timehw to read time read from timelr before timehr An alarm is set by setting alarm_enable and writing to the corresponding alarm register When an alarm is pending, the corresponding alarm_running signal will be high An alarm can be cancelled before it has finished by clearing the alarm_enable When an alarm fires, the corresponding alarm_irq is set and alarm_running is cleared To clear the interrupt write a 1 to the corresponding alarm_irq */
#define RP23XX_WATCHDOG_BASE             0x400d8000
#define RP23XX_RTC_BASE                  0x4005c000  /* Register block to control RTC */
#define RP23XX_ROSC_BASE                 0x400e8000
#define RP23XX_VREG_AND_CHIP_RESET_BASE  0x40064000  /* control and status for on-chip voltage regulator and chip level reset subsystem */
#define RP23XX_TBMAN_BASE                0x40160000  /* Testbench manager. Allows the programmer to know what platform their software is running on. */
#define RP23XX_DMA_BASE                  0x50000000  /* DMA with separate read and write masters */
#define RP23XX_USBCTRL_DPSRAM_BASE       0x50100000  /* USB Dual Port SRAM */
#define RP23XX_USBCTRL_REGS_BASE         0x50110000  /* USB FS/LS controller device registers */
#define RP23XX_PIO0_BASE                 0x50200000  /* Programmable IO block */
#define RP23XX_PIO1_BASE                 0x50300000  /* Programmable IO block */
#define RP23XX_PIO_BASE(n)               (0x50200000 + (n) * 0x100000)
#define RP23XX_SIO_BASE                  0xd0000000  /* Single-cycle IO block Provides core-local and inter-core hardware for the two processors, with single-cycle access. */
#define RP23XX_PPB_BASE                  0xe0000000

#define RP23XX_ATOMIC_XOR_REG_OFFSET         0x1000
#define RP23XX_ATOMIC_SET_REG_OFFSET         0x2000
#define RP23XX_ATOMIC_CLR_REG_OFFSET         0x3000

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

#  define xorbits_reg32(v,a)   putreg32(v, (a) | RP23XX_ATOMIC_XOR_REG_OFFSET)
#  define setbits_reg32(v,a)   putreg32(v, (a) | RP23XX_ATOMIC_SET_REG_OFFSET)
#  define clrbits_reg32(v,a)   putreg32(v, (a) | RP23XX_ATOMIC_CLR_REG_OFFSET)
#  define modbits_reg32(v,m,a) xorbits_reg32((getreg32(a) ^ (v)) & (m), a)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_MEMORYMAP_H */
