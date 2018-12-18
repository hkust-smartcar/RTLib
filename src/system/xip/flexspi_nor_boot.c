/*
 * The Clear BSD License
 * Copyright 2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * flexspi_nor_boot.cpp
 *
 *  Created on: Aug 3, 2018
 *      Author: LeeChunHei
 */

#include "system/xip/flexspi_nor_boot.h"

__attribute__((section(".boot_hdr.ivt")))
/*************************************
 *  IVT Data
 *************************************/
const ivt image_vector_table = {
IVT_HEADER, /* IVT Header */
IMAGE_ENTRY_ADDRESS, /* Image Entry Function */
IVT_RSVD, /* Reserved = 0 */
(uint32_t) DCD_ADDRESS, /* Address where DCD information is stored */
(uint32_t)BOOT_DATA_ADDRESS, /* Address where BOOT Data Structure is stored */
(uint32_t) & image_vector_table, /* Pointer to IVT Self (absolute address */
(uint32_t) CSF_ADDRESS, /* Address where CSF file is stored */
IVT_RSVD /* Reserved = 0 */
};

__attribute__((section(".boot_hdr.boot_data")))
/*************************************
 *  Boot Data
 *************************************/
const BOOT_DATA_T boot_data = {
		0x00000000,//FLASH_BASE, //0x00000000, /* boot start location */
		0x20000,//FLASH_SIZE, //0x20000, /* size */
		PLUGIN_FLAG, /* Plugin flag*/
		0xFFFFFFFF /* empty - extra data word */
};

