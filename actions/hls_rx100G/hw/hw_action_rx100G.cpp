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

inline void mask_tkeep(ap_uint<512> &data, ap_uint<64> keep) {
	for (int i = 0; i < 64; i++) {
#pragma HLS unroll
		if (keep[i] == 0) data(i*8+7,i*8) = 0;
	}
}

inline void copy_data(snap_membus_t *din_gmem, snap_HBMbus_t *d_hbm0, snap_HBMbus_t *d_hbm1, size_t in_addr) {
	for (int i = 0; i < NMODULES * 512 * 1024 / 32; i ++) {
#pragma HLS pipeline
		ap_uint<512> tmp;

		memcpy(&tmp, din_gmem + in_addr + i, 64);
		ap_uint<256> tmp0 = tmp(255,0);
		ap_uint<256> tmp1 = tmp(511,256);

		memcpy(d_hbm0+i, &tmp0, 32);
		memcpy(d_hbm1+i, &tmp1, 32);
		}
}

void process_frames(AXI_STREAM &din_eth,
		eth_settings_t eth_settings, eth_stat_t &eth_stat,
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
	read_eth_packet(din_eth, raw, eth_settings, eth_stat);
	convert_data(raw, converted,
			d_hbm_p0, d_hbm_p1,
			d_hbm_p2, d_hbm_p3,
			d_hbm_p4, d_hbm_p5,
			d_hbm_p6, d_hbm_p7,
			d_hbm_p8, d_hbm_p9,
			d_hbm_p10, d_hbm_p11,
			mode);
	write_data(converted, dout_gmem, out_frame_buffer_addr, out_frame_status_addr);
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

       send_gratious_arp(dout_eth, act_reg->Data.fpga_mac_addr, act_reg->Data.fpga_ipv4_addr);

	size_t in_gain_pedestal_addr = act_reg->Data.in_gain_pedestal_data.addr >> ADDR_RIGHT_SHIFT;
	size_t out_frame_buffer_addr = act_reg->Data.out_frame_buffer.addr >> ADDR_RIGHT_SHIFT;
	size_t out_frame_status_addr = act_reg->Data.out_frame_status.addr >> ADDR_RIGHT_SHIFT;

	uint64_t bytes_written = 0;

	eth_settings_t eth_settings;
	eth_settings.fpga_mac_addr = act_reg->Data.fpga_mac_addr;
	eth_settings.fpga_ipv4_addr = act_reg->Data.fpga_ipv4_addr;
	eth_settings.expected_packets = act_reg->Data.packets_to_read;

	eth_stat_t eth_stats;
	eth_stats.bad_packets = 0;
	eth_stats.good_packets = 0;
	eth_stats.ignored_packets = 0;

	// Load constants
	// Copy pede G1
	copy_data(din_gmem, d_hbm_p0, d_hbm_p1, in_gain_pedestal_addr);
	// Copy pede G2
	copy_data(din_gmem, d_hbm_p2, d_hbm_p3, in_gain_pedestal_addr + (NMODULES * 512 * 1024 / 32));
	// Copy gain G0
	copy_data(din_gmem, d_hbm_p4, d_hbm_p5, in_gain_pedestal_addr + (NMODULES * 512 * 1024 / 32) * 2);
	// Copy gain G1
	copy_data(din_gmem, d_hbm_p6, d_hbm_p7, in_gain_pedestal_addr + (NMODULES * 512 * 1024 / 32) * 3);
	// Copy gain G2
	copy_data(din_gmem, d_hbm_p8, d_hbm_p9, in_gain_pedestal_addr + (NMODULES * 512 * 1024 / 32) * 4);
	// Copy pede G0 RMS
	copy_data(din_gmem, d_hbm_p10, d_hbm_p11, in_gain_pedestal_addr + (NMODULES * 512 * 1024 / 32) * 5);

	process_frames(din_eth, eth_settings, eth_stats, dout_gmem, out_frame_buffer_addr, out_frame_status_addr, 
                       d_hbm_p0, d_hbm_p1, 
                       d_hbm_p2, d_hbm_p3, 
                       d_hbm_p4, d_hbm_p5, 
                       d_hbm_p6, d_hbm_p7, 
                       d_hbm_p8, d_hbm_p9, 
                       d_hbm_p10, d_hbm_p11, 
                       act_reg->Data.mode);

	act_reg->Data.good_packets = eth_stats.good_packets;
	act_reg->Data.bad_packets = eth_stats.bad_packets;
	//	act_reg->Data.ignored_packets = eth_stats.ignored_packets;

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

