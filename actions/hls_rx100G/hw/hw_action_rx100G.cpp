/*
 * Copyright 2017 International Business Machines
 * Copyright 2019 Paul Scherrer Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <string.h>
#include "ap_int.h"
#include "hw_action_rx100G.h"

packed_pedeG0_t packed_pedeG0[NMODULES * 512 * 1024 / 32];

void copy_data(snap_membus_t *din_gmem, snap_HBMbus_t *d_hbm0, snap_HBMbus_t *d_hbm1, size_t in_addr) {
	for (size_t i = 0; i < NMODULES * 512 * 1024 / 32; i ++) {
		ap_uint<512> tmp = din_gmem[in_addr+i];
		d_hbm0[i] = tmp(255,0);
	}
	for (size_t i = 0; i < NMODULES * 512 * 1024 / 32; i ++) {
		ap_uint<512> tmp = din_gmem[in_addr+i];
		d_hbm1[i] = tmp(511,256);
	}
}

void load_data_to_hbm(snap_membus_t *din_gmem, uint64_t in_gain_pedestal_addr,
		snap_HBMbus_t *d_hbm_p0, snap_HBMbus_t *d_hbm_p1,
		snap_HBMbus_t *d_hbm_p2, snap_HBMbus_t *d_hbm_p3,
		snap_HBMbus_t *d_hbm_p4, snap_HBMbus_t *d_hbm_p5,
		snap_HBMbus_t *d_hbm_p6, snap_HBMbus_t *d_hbm_p7,
		snap_HBMbus_t *d_hbm_p8, snap_HBMbus_t *d_hbm_p9,
		snap_HBMbus_t *d_hbm_p10, snap_HBMbus_t *d_hbm_p11) {
	size_t offset = in_gain_pedestal_addr;
	for (size_t i = 0; i < NPIXEL * 2 / 64; i ++) {
#pragma HLS PIPELINE
ap_uint<512> tmp = din_gmem[offset+i];
d_hbm_p0[i] = tmp(255,0);
d_hbm_p1[i] = tmp(511,256);
	}
	offset += NPIXEL * 2 / 64;
	for (size_t i = 0; i < NPIXEL * 2 / 64; i ++) {
#pragma HLS PIPELINE
		ap_uint<512> tmp = din_gmem[offset+i];
		d_hbm_p2[i] = tmp(255,0);
		d_hbm_p3[i] = tmp(511,256);
	}
	offset += NPIXEL * 2 / 64;
	for (size_t i = 0; i < NPIXEL * 2 / 64; i ++) {
#pragma HLS PIPELINE
		ap_uint<512> tmp = din_gmem[offset+i];
		d_hbm_p4[i] = tmp(255,0);
		d_hbm_p5[i] = tmp(511,256);
	}
	offset += NPIXEL * 2 / 64;
	for (size_t i = 0; i < NPIXEL * 2 / 64; i ++) {
#pragma HLS PIPELINE
		ap_uint<512> tmp = din_gmem[offset+i];
		d_hbm_p6[i] = tmp(255,0);
		d_hbm_p7[i] = tmp(511,256);
	}
	offset += NPIXEL * 2 / 64;
	for (size_t i = 0; i < NPIXEL * 2 / 64; i ++) {
#pragma HLS PIPELINE
		ap_uint<512> tmp = din_gmem[offset+i];
		d_hbm_p8[i] = tmp(255,0);
		d_hbm_p9[i] = tmp(511,256);
	}
	// p10 and p11 are used for statistics and need to be zeroed out
	for (size_t i = 0; i < NPIXEL * 2 / 64; i ++) {
#pragma HLS PIPELINE
		d_hbm_p10[i] = 0;
		d_hbm_p11[i] = 0;
	}
}

void save_pedestal(snap_membus_t *dout_gmem, size_t offset) {
	for (size_t i = 0; i < NPIXEL * 2 / 64 / 8; i ++) {
#pragma HLS PIPELINE II = 8

		packed_pedeG0_t tmp[8];
		ap_uint<512> tmp2[8];

		for (int j = 0; j < 8; j++) tmp[j] = packed_pedeG0[8*i+j];
		for (int j = 0; j < 8; j++) pack_pede(tmp[j],tmp2[j]);
		memcpy(dout_gmem+offset+8*i, tmp2, 8*64);
	}
}

void load_pedestal(snap_membus_t *din_gmem, size_t offset) {
	for (size_t i = 0; i < NPIXEL * 2 / 64 / 8; i ++) {
#pragma HLS PIPELINE II = 8
		packed_pedeG0_t tmp[8];
		ap_uint<512> tmp2[8];
		memcpy(tmp2, din_gmem+offset+8*i, 8*64);
		for (int j = 0; j < 8; j++) {
			pedeG1G2_t in[32];
			pedeG0_t out[32];
			unpack_pedeG1G2(tmp2[j],in);
			for (int k = 0; k < 32; k++ ) out[k] = in[k];
			pack_pedeG0(tmp[j], out);
		}
		for (int j = 0; j < 8; j++) packed_pedeG0[8*i+j] = tmp[j];
	}
}

// Taken from HBM_memcopy action
//convert buffer 256b to 512b
static void HBMbus_to_membus(snap_HBMbus_t *data_in, snap_membus_t *data_out,
                             uint64_t size_in_words_512)
{
#pragma HLS INLINE off
        static snap_membus_t data_entry = 0;

        hbm2mem_loop:
        for (int k=0; k<size_in_words_512; k++) {
#pragma HLS PIPELINE II=2
            for (int j = 0; j < 2; j++) {
                data_entry |= ((snap_membus_t)(data_in[k*2+j])) << j*MEMDW/2;
            }
            data_out[k] = data_entry;
            data_entry = 0;
        }
 }


void collect_data(AXI_STREAM &din_eth,
		eth_settings_t eth_settings,
		snap_membus_t *dout_gmem,
		size_t out_frame_buffer_addr, size_t out_frame_status_addr,
		snap_HBMbus_t *d_hbm_p0, snap_HBMbus_t *d_hbm_p1,
		snap_HBMbus_t *d_hbm_p2, snap_HBMbus_t *d_hbm_p3,
		snap_HBMbus_t *d_hbm_p4, snap_HBMbus_t *d_hbm_p5,
		snap_HBMbus_t *d_hbm_p6, snap_HBMbus_t *d_hbm_p7,
		snap_HBMbus_t *d_hbm_p8, snap_HBMbus_t *d_hbm_p9,
		snap_HBMbus_t *d_hbm_p10, snap_HBMbus_t *d_hbm_p11,
		ap_uint<8> mode) {
#pragma HLS DATAFLOW
DATA_STREAM raw;
DATA_STREAM converted;
#pragma HLS STREAM variable=raw depth=512
#pragma HLS STREAM variable=converted depth=512
read_eth_packet(din_eth, raw, eth_settings, d_hbm_p10);
convert_data(raw, converted,
		d_hbm_p0, d_hbm_p1,
		d_hbm_p2, d_hbm_p3,
		d_hbm_p4, d_hbm_p5,
		d_hbm_p6, d_hbm_p7,
		d_hbm_p8, d_hbm_p9,
		mode);
write_data(converted, dout_gmem, out_frame_buffer_addr, out_frame_status_addr, d_hbm_p11);
}

//----------------------------------------------------------------------
//--- MAIN PROGRAM -----------------------------------------------------
//----------------------------------------------------------------------
static int process_action(snap_membus_t *din_gmem,
		snap_membus_t *dout_gmem,
		snap_HBMbus_t *d_hbm_p0, snap_HBMbus_t *d_hbm_p1,
		snap_HBMbus_t *d_hbm_p2, snap_HBMbus_t *d_hbm_p3,
		snap_HBMbus_t *d_hbm_p4, snap_HBMbus_t *d_hbm_p5,
		snap_HBMbus_t *d_hbm_p6, snap_HBMbus_t *d_hbm_p7,
		snap_HBMbus_t *d_hbm_p8, snap_HBMbus_t *d_hbm_p9,
		snap_HBMbus_t *d_hbm_p10, snap_HBMbus_t *d_hbm_p11,
		AXI_STREAM &din_eth,
		AXI_STREAM &dout_eth,
		action_reg *act_reg)
{

	// HBM and in-memory order:
	// p0,p1 - gain G0
	// p2,p3 - gain G1
	// p4,p5 - gain G2
	// p6,p7 - pedestal G1
	// p8,p9 - pedestal G2

	send_gratious_arp(dout_eth, act_reg->Data.fpga_mac_addr, act_reg->Data.fpga_ipv4_addr);

	size_t in_gain_pedestal_addr = act_reg->Data.in_gain_pedestal_data.addr >> ADDR_RIGHT_SHIFT;
	size_t out_frame_buffer_addr = act_reg->Data.out_frame_buffer.addr >> ADDR_RIGHT_SHIFT;
	size_t out_frame_status_addr = act_reg->Data.out_frame_status.addr >> ADDR_RIGHT_SHIFT;

	uint64_t bytes_written = 0;

	eth_settings_t eth_settings;
	eth_settings.fpga_mac_addr = act_reg->Data.fpga_mac_addr;
	eth_settings.fpga_ipv4_addr = act_reg->Data.fpga_ipv4_addr;
	eth_settings.frame_number_to_stop = act_reg->Data.packets_to_read;
	eth_settings.frame_number_to_stop = act_reg->Data.packets_to_read + 5;

	// Load constants
	load_data_to_hbm(din_gmem, in_gain_pedestal_addr,
			d_hbm_p0, d_hbm_p1,
			d_hbm_p2, d_hbm_p3,
			d_hbm_p4, d_hbm_p5,
			d_hbm_p6, d_hbm_p7,
			d_hbm_p8, d_hbm_p9,
			d_hbm_p10, d_hbm_p11);

	switch (act_reg->Data.mode) {
	case MODE_PEDEG0:
	case MODE_CONV:
		load_pedestal(din_gmem, in_gain_pedestal_addr + 5 * NPIXEL * 2L / 64);
		break;
	case MODE_PEDEG1:
		load_pedestal(din_gmem, in_gain_pedestal_addr + 3 * NPIXEL * 2L / 64);
		break;
	case MODE_PEDEG2:
		load_pedestal(din_gmem, in_gain_pedestal_addr + 4 * NPIXEL * 2L / 64);
		break;
	}

	collect_data(din_eth, eth_settings, dout_gmem, out_frame_buffer_addr, out_frame_status_addr,
			d_hbm_p0, d_hbm_p1,
			d_hbm_p2, d_hbm_p3,
			d_hbm_p4, d_hbm_p5,
			d_hbm_p6, d_hbm_p7,
			d_hbm_p8, d_hbm_p9,
			d_hbm_p10, d_hbm_p11,
			act_reg->Data.mode);

	// Save calculated pedestal back to memory
	switch (act_reg->Data.mode) {
	case MODE_PEDEG0:
	case MODE_CONV:
		save_pedestal(dout_gmem, in_gain_pedestal_addr + 5 * NPIXEL * 2L / 64);
		break;
	case MODE_PEDEG1:
		save_pedestal(dout_gmem, in_gain_pedestal_addr + 3 * NPIXEL * 2L / 64);
		break;
	case MODE_PEDEG2:
		save_pedestal(dout_gmem, in_gain_pedestal_addr + 4 * NPIXEL * 2L / 64);
		break;
	}

       // Save status - currently copy the whole HBM content
       HBMbus_to_membus(d_hbm_p10,  dout_gmem + out_frame_status_addr + 64, 256*1024L*1024L/64);
       HBMbus_to_membus(d_hbm_p11,  dout_gmem + out_frame_status_addr + 64 + 256*1024L*1024L/64, 256*1024L*1024L/64);

	act_reg->Control.Retc = SNAP_RETC_SUCCESS;

	return 0;
}

//--- TOP LEVEL MODULE -------------------------------------------------
void hls_action(snap_membus_t *din_gmem, snap_membus_t *dout_gmem,
		snap_HBMbus_t *d_hbm_p0, snap_HBMbus_t *d_hbm_p1,
		snap_HBMbus_t *d_hbm_p2, snap_HBMbus_t *d_hbm_p3,
		snap_HBMbus_t *d_hbm_p4, snap_HBMbus_t *d_hbm_p5,
		snap_HBMbus_t *d_hbm_p6, snap_HBMbus_t *d_hbm_p7,
		snap_HBMbus_t *d_hbm_p8, snap_HBMbus_t *d_hbm_p9,
		snap_HBMbus_t *d_hbm_p10, snap_HBMbus_t *d_hbm_p11,
		AXI_STREAM &din_eth, AXI_STREAM &dout_eth,
		action_reg *act_reg,
		action_RO_config_reg *Action_Config)
{
	// Host Memory AXI Interface - CANNOT BE REMOVED - NO CHANGE BELOW
#pragma HLS INTERFACE m_axi port=din_gmem bundle=host_mem offset=slave depth=512 \
		max_read_burst_length=64  max_write_burst_length=64 latency=16
#pragma HLS INTERFACE s_axilite port=din_gmem bundle=ctrl_reg offset=0x030

#pragma HLS INTERFACE m_axi port=dout_gmem bundle=host_mem offset=slave depth=512 \
		max_read_burst_length=64  max_write_burst_length=64 latency=16
#pragma HLS INTERFACE s_axilite port=dout_gmem bundle=ctrl_reg offset=0x040

	/*  // DDR memory Interface - CAN BE COMMENTED IF UNUSED
	 * #pragma HLS INTERFACE m_axi port=d_ddrmem bundle=card_mem0 offset=slave depth=512 \
	 *   max_read_burst_length=64  max_write_burst_length=64
	 * #pragma HLS INTERFACE s_axilite port=d_ddrmem bundle=ctrl_reg offset=0x050
	 */
	// Host Memory AXI Lite Master Interface - NO CHANGE BELOW
#pragma HLS DATA_PACK variable=Action_Config
#pragma HLS INTERFACE s_axilite port=Action_Config bundle=ctrl_reg offset=0x010
#pragma HLS DATA_PACK variable=act_reg
#pragma HLS INTERFACE s_axilite port=act_reg bundle=ctrl_reg offset=0x100
#pragma HLS INTERFACE s_axilite port=return bundle=ctrl_reg

#pragma HLS INTERFACE m_axi port=d_hbm_p0 bundle=card_hbm_p0 offset=slave depth=512 \
		max_read_burst_length=64  max_write_burst_length=64 latency=8
#pragma HLS INTERFACE m_axi port=d_hbm_p1 bundle=card_hbm_p1 offset=slave depth=512 \
		max_read_burst_length=64  max_write_burst_length=64 latency=8
#pragma HLS INTERFACE m_axi port=d_hbm_p2 bundle=card_hbm_p2 offset=slave depth=512 \
		max_read_burst_length=64  max_write_burst_length=64 latency=8
#pragma HLS INTERFACE m_axi port=d_hbm_p3 bundle=card_hbm_p3 offset=slave depth=512 \
		max_read_burst_length=64  max_write_burst_length=64 latency=8
#pragma HLS INTERFACE m_axi port=d_hbm_p4 bundle=card_hbm_p4 offset=slave depth=512 \
		max_read_burst_length=64  max_write_burst_length=64 latency=8
#pragma HLS INTERFACE m_axi port=d_hbm_p5 bundle=card_hbm_p5 offset=slave depth=512 \
		max_read_burst_length=64  max_write_burst_length=64 latency=8
#pragma HLS INTERFACE m_axi port=d_hbm_p6 bundle=card_hbm_p6 offset=slave depth=512 \
		max_read_burst_length=64  max_write_burst_length=64 latency=8
#pragma HLS INTERFACE m_axi port=d_hbm_p7 bundle=card_hbm_p7 offset=slave depth=512 \
		max_read_burst_length=64  max_write_burst_length=64 latency=8
#pragma HLS INTERFACE m_axi port=d_hbm_p8 bundle=card_hbm_p8 offset=slave depth=512 \
		max_read_burst_length=64  max_write_burst_length=64 latency=8
#pragma HLS INTERFACE m_axi port=d_hbm_p9 bundle=card_hbm_p9 offset=slave depth=512 \
		max_read_burst_length=64  max_write_burst_length=64 latency=8
#pragma HLS INTERFACE m_axi port=d_hbm_p10 bundle=card_hbm_p10 offset=slave depth=512 \
		max_read_burst_length=64  max_write_burst_length=64 latency=8
#pragma HLS INTERFACE m_axi port=d_hbm_p11 bundle=card_hbm_p11 offset=slave depth=512 \
		max_read_burst_length=64  max_write_burst_length=64 latency=8

#pragma HLS INTERFACE axis register off port=din_eth
#pragma HLS INTERFACE axis register off port=dout_eth

#pragma HLS RESOURCE variable=packed_pedeG0 core=RAM_1P_URAM latency=1
#pragma HLS ARRAY_PARTITION variable=packed_pedeG0 cyclic factor=8 dim=1

	/* Required Action Type Detection - NO CHANGE BELOW */
	//	NOTE: switch generates better vhdl than "if" */
	// Test used to exit the action if no parameter has been set.
	// Used for the discovery phase of the cards */
	switch (act_reg->Control.flags) {
	case 0:
		Action_Config->action_type = RX100G_ACTION_TYPE; //TO BE ADAPTED
		Action_Config->release_level = RELEASE_LEVEL;
		act_reg->Control.Retc = 0xe00f;
		return;
		break;
	default:
		/* process_action(din_gmem, dout_gmem, d_ddrmem, act_reg); */
		// process_action(din_gmem, dout_gmem, din_eth, dout_eth, act_reg);
		process_action(din_gmem, dout_gmem, d_hbm_p0, d_hbm_p1, d_hbm_p2, d_hbm_p3, d_hbm_p4, d_hbm_p5, d_hbm_p6, d_hbm_p7, d_hbm_p8, d_hbm_p9, d_hbm_p10, d_hbm_p11, din_eth, dout_eth, act_reg);
		break;
	}
}

