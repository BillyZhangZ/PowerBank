#!/usr/bin/env python

# Copyright (c) 2014 Freescale Semiconductor, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
# o Redistributions of source code must retain the above copyright notice, this list
#   of conditions and the following disclaimer.
#
# o Redistributions in binary form must reproduce the above copyright notice, this
#   list of conditions and the following disclaimer in the documentation and/or
#   other materials provided with the distribution.
#
# o Neither the name of Freescale Semiconductor, Inc. nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
# ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import sys, os
import bltestconfig


tmpboard = bltestconfig.target[1]

# For system restoration test case
# uart0 and i2c0 pinmux
if tmpboard.find('freedom') != -1:
    pinmuxRegAddr_uart_rx = 0x40049004  #PTA1
    pinmuxRegAddr_uart_tx = 0x40049008  #PTA2
 
    pinmuxRegAddr_i2c_scl = 0x4004B020  #PTC8
    pinmuxRegAddr_i2c_sda = 0x4004B024  #PTC9
elif tmpboard.find('tower') != -1:
    pinmuxRegAddr_uart_rx = 0x4004903C  #PTA15
    pinmuxRegAddr_uart_tx = 0x40049038  #PTA14
 
    pinmuxRegAddr_i2c_scl = 0x4004D060  #PTE24
    pinmuxRegAddr_i2c_sda = 0x4004D064  #PTE25
else:
    raise ValueError('Unsupported target build.')

# spi0 pinmux
pinmuxRegAddr_spi_pcs = 0x4004C000  #PTD0
pinmuxRegAddr_spi_sck = 0x4004C004  #PTD1
pinmuxRegAddr_spi_miso = 0x4004C008  #PTD3
pinmuxRegAddr_spi_mosi = 0x4004C00C  #PTD2

# uart0 control regs
controlRegAddr_lpuart_CTRL = None
controlRegAddr_uart_C1 = 0x4006A002
controlRegAddr_uart_C2 = 0x4006A003
controlRegAddr_uart_C3 = 0x4006A006
controlRegAddr_uart_C4 = None
controlRegAddr_uart0_C4 = 0x4006A00A
controlRegAddr_uart0_C5 = 0x4006A00B
# spi0 control regs
controlRegAddr_spi_C1 = 0x40076000
controlRegAddr_spi_C2 = 0x40076001
controlRegAddr_spi_C3 = None
# i2c0 control regs
controlRegAddr_i2c_C1 = 0x40066002
controlRegAddr_i2c_C2 = 0x40066005
# usb0 control regs
controlRegAddr_usb_CTL0 = None
controlRegAddr_usb_CTL1 = None
controlRegAddr_usb_CTL = 0x40072094
controlRegAddr_usb_OTGCTL = 0x4007201C


