#/**
# Copyright (c) 2014 Cory Xie.
# All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
#
# @file objects.mk
# @author Cory Xie (cory.xie@gmail.com)
# @brief list of i.MX61 board objects.
# */

board-objs-y+=brd_defterm.o
board-objs-y+=brd_main.o
board-objs-$(CONFIG_SMP)+=brd_smp.o
board-objs-$(CONFIG_SD_ONE_GUEST_EBMP_DTS)+=dts/sd/one_guest_ebmp.o

board-dtbs-y+=dts/sd/one_guest_ebmp.dtb
