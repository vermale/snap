#ifndef __HW_ACTION_RX100G_H__
#define __HW_ACTION_RX100G_H__

/*
 * Copyright 2017 International Business Machines
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

#include <stdint.h>
#include <string.h>
#include <ap_int.h>
#include <hls_stream.h>

#include "hls_snap.H"
#include "../include/action_rx100G.h" /* HelloWorld Job definition */

#define PEDE_G0_PRECISION 24
#define PEDESTAL_WINDOW_SIZE 128

//--------------------------------------------------------------------
// 1: simplify the data casting style
#define RELEASE_LEVEL		0x00000003

typedef char word_t[BPERDW];
//---------------------------------------------------------------------
// This is generic. Just adapt names for a new action
// CONTROL is defined and handled by SNAP 
// helloworld_job_t is user defined in hls_helloworld/include/action_change_case.h
typedef struct {
	CONTROL Control;	/*  16 bytes */
	rx100G_job_t Data;	/* up to 108 bytes */
	uint8_t padding[SNAP_HLS_JOBSIZE - sizeof(rx100G_job_t)];
} action_reg;

// Based on https://forums.xilinx.com/t5/High-Level-Synthesis-HLS/ap-axiu-parameters/td-p/635138
struct ap_axiu_for_eth {
	ap_uint<512>     data;
	ap_uint<64>      keep;
	ap_uint<1>       user;
	ap_uint<1>       last;
};

struct eth_settings_t {
	uint64_t expected_packets;
	uint64_t fpga_mac_addr;
	uint32_t fpga_ipv4_addr;
	uint32_t fpga_udp_port;
};

struct eth_stat_t {
	uint64_t good_packets;
	uint64_t bad_packets;
	uint64_t ignored_packets;
};

struct data_packet_t {
	ap_uint<512> data;
	ap_uint<64> frame_number; //
	ap_uint<4> module; // 0..16
	ap_uint<8> eth_packet; // 0..128
	ap_uint<8> axis_packet; // 0..128
        ap_uint<1> axis_user; // TUSER from AXIS
	ap_uint<1> exit; // exit
	ap_uint<1> trigger; // debug flag on
};

struct packet_header_t {
	ap_uint<48> dest_mac ;
	ap_uint<48> src_mac  ;
	ap_uint<16> ether_type ;

	ap_uint<4> ip_version ;
	ap_uint<4> ipv4_header_len ;
	ap_uint<8> ipv4_protocol  ;
	ap_uint<32> ipv4_total_len ;
	// uint32_t ipv4_header_checksum;
	ap_uint<32> ipv4_source_ip ;
	ap_uint<32> ipv4_dest_ip ;

	ap_uint<16> udp_src_port;
	ap_uint<16> udp_dest_port;
	ap_uint<16> udp_len;
	ap_uint<16> udp_checksum;

	ap_uint<64> jf_frame_number;
	ap_uint<32> jf_exptime;
	ap_uint<32> jf_packet_number;
	ap_uint<64> jf_bunch_id;
	ap_uint<64> jf_timestamp;
	ap_uint<16> jf_module_id;
	ap_uint<16> jf_xcoord;
	ap_uint<16> jf_ycoord;
	ap_uint<16> jf_zcoord;
	ap_uint<32> jf_debug;
	ap_uint<16> jf_round_robin;
	ap_uint<8> jf_detector_type;
	ap_uint<8> jf_header_version_type;
};

typedef ap_ufixed<PEDE_G0_PRECISION,14, SC_RND_CONV> pedeG0_t;
typedef ap_ufixed<16,2, SC_RND_CONV>  gainG0_t;
typedef ap_ufixed<16,12, SC_RND_CONV> pedeG0RMS_t;

typedef ap_ufixed<16,14, SC_RND_CONV> pedeG1G2_t;
typedef ap_ufixed<16,3, SC_RND_CONV>  gainG1G2_t;

typedef ap_uint<PEDE_G0_PRECISION*32> packed_pedeG0_t;

typedef hls::stream<ap_axiu_for_eth> AXI_STREAM;
typedef hls::stream<data_packet_t> DATA_STREAM;

void decode_eth_1(ap_uint<512> val_in, packet_header_t &header_out);
void decode_eth_2(ap_uint<512> val_in, packet_header_t &header_out);

void pack_pedeG0(packed_pedeG0_t& out, pedeG0_t in[32]);
void unpack_pedeG0(packed_pedeG0_t in, pedeG0_t out[32]);
void unpack_gainG0(ap_uint<512> in, gainG0_t outg[32]);
void unpack_pedeG0RMS(ap_uint<512> in, pedeG0RMS_t[32]);
void unpack_pedeG1G2(ap_uint<512> in, pedeG1G2_t outp[32]);
void unpack_gainG1G2(ap_uint<512> in, gainG1G2_t outp[32]);
void status_pack(ap_uint<512> &out, ap_uint<8> in[64]) ;

void unpack_gainG0(ap_uint<256> in_1, ap_uint<256> in_2, gainG0_t outg[32]);
void unpack_pedeG0RMS(ap_uint<256> in_1, ap_uint<256> in_2, pedeG0RMS_t outr[32]);
void unpack_pedeG1G2(ap_uint<256> in_1, ap_uint<256> in_2, pedeG1G2_t outp[32]);
void unpack_gainG1G2(ap_uint<256> in_1, ap_uint<256> in_2, gainG1G2_t outg[32]);

void data_shuffle(ap_uint<512> &out, ap_int<16> in[32]);
void data_pack(ap_uint<512> &out, ap_int<16> in[32]);

void send_gratious_arp(AXI_STREAM &out, ap_uint<48> mac, ap_uint<32> ipv4_address);

void read_eth_packet(AXI_STREAM &deth_in, DATA_STREAM &raw_out, eth_settings_t eth_settings, eth_stat_t &eth_stat);

void filter_packets(DATA_STREAM &in, DATA_STREAM &out);

void convert_data(DATA_STREAM &in, DATA_STREAM &out,
		snap_HBMbus_t *d_hbm_p0, snap_HBMbus_t *d_hbm_p1,
		snap_HBMbus_t *d_hbm_p2, snap_HBMbus_t *d_hbm_p3,
		snap_HBMbus_t *d_hbm_p4, snap_HBMbus_t *d_hbm_p5,
		snap_HBMbus_t *d_hbm_p6, snap_HBMbus_t *d_hbm_p7,
		snap_HBMbus_t *d_hbm_p8, snap_HBMbus_t *d_hbm_p9,
		snap_HBMbus_t *d_hbm_p10, snap_HBMbus_t *d_hbm_p11,
		bool saw_raw);


void write_data(DATA_STREAM &in, snap_membus_t *dout_gmem,
		size_t out_frame_buffer_addr, size_t out_frame_status_addr);

void pedestal_update(ap_uint<512> data_in, packed_pedeG0_t& packed_pede, ap_uint<32> &mask, ap_uint<2> exp_gain, uint64_t frame_number);

void convert_and_shuffle(ap_uint<512> data_in, ap_uint<512>& data_out,
		packed_pedeG0_t& packed_pedeG0,
		ap_uint<256> packed_pedeG0RMS_1, ap_uint<256> packed_pedeG0RMS_2,
		ap_uint<256> packed_gainG0_1, ap_uint<256> packed_gainG0_2,
		ap_uint<256> packed_pedeG1_1, ap_uint<256> packed_pedeG1_2,
		ap_uint<256> packed_gainG1_1, ap_uint<256> packed_gainG1_2,
		ap_uint<256> packed_pedeG2_1, ap_uint<256> packed_pedeG2_2,
		ap_uint<256> packed_gainG2_1, ap_uint<256> packed_gainG2_2);

void convert_and_shuffle(ap_uint<512> data_in, ap_uint<512>& data_out,
		packed_pedeG0_t& packed_pedeG0, ap_uint<512> packed_pedeG0RMS, ap_uint<512> packed_gainG0,
		ap_uint<512> packed_pedeG1, ap_uint<512> packed_gainG1,
		ap_uint<512> packed_pedeG2, ap_uint<512> packed_gainG2);

#endif  /* __ACTION_RX100G_H__*/
