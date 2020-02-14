/*
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

#include "hw_action_rx100G.h"

inline void unpack_pedeG1G2(ap_uint<512> in, pedeG1G2_t outp[32]) {
	for (int i = 0; i < 32; i ++) {
		for (int j = 0; j < 16; j ++) {
			outp[i][j] = in[i*16+j];
		}
	}
}

inline void unpack_gainG1G2(ap_uint<512> in, gainG1G2_t outg[32]) {
	for (int i = 0; i < 32; i ++) {
		for (int j = 0; j < 16; j ++) {
			outg[i][j] = in[i*16+j];
		}
	}
}


void pedestal_update(ap_uint<512> data_in, packed_pedeG0_t& packed_pede, ap_uint<32> &mask, ap_uint<2> exp_gain, uint64_t frame_number) {
	pedeG0_t pedestal[32];
	unpack_pedeG0(packed_pede, pedestal);
	ap_uint<16> in_val[32];

	Loop0: for (int i = 0; i < 512; i++) in_val[i/16][i%16] = data_in[i];

	Loop1: for (int i = 0; i < 32; i++) {
		ap_uint<2> gain = in_val[i](15,14);
		if (gain != exp_gain) mask[i] = 1;
		else {
			pedeG0_t tmp = in_val[i](13,0);
			if (frame_number < PEDESTAL_WINDOW_SIZE)
				// Plain average
				pedestal[i] += tmp / PEDESTAL_WINDOW_SIZE;
			else
				// Rolling average
				pedestal[i] += (tmp - pedestal[i]) / PEDESTAL_WINDOW_SIZE;
		}
	}
	pack_pedeG0(packed_pede, pedestal);
}

void convert_data(DATA_STREAM &in, DATA_STREAM &out,
		snap_membus_t *d_hbm_p0, snap_membus_t *d_hbm_p1,
		snap_membus_t *d_hbm_p2, snap_membus_t *d_hbm_p3,
		snap_membus_t *d_hbm_p4, snap_membus_t *d_hbm_p5,

		bool save_raw) {

	data_packet_t packet_in, packet_out;
	in.read(packet_in);
	packed_pedeG0_t packed_pedeG0[NMODULES * 512 * 1024 / 32];

#pragma HLS RESOURCE variable=packed_pedeG0 core=RAM_1P_URAM
	//#pragma HLS ARRAY_PARTITION variable=packed_pedeG0 cyclic factor=8 dim=1

	if (save_raw == 1) {
		Just_forward: while (packet_in.exit != 1) {
#pragma HLS pipeline
			out.write(packet_in);
			in.read(packet_in);
		}
	} else {
		while (packet_in.exit != 1) {
			Convert_and_forward: while ((packet_in.exit != 1) && (packet_in.axis_packet % 8 == 0)) {
#pragma HLS pipeline II = 8
				size_t offset = packet_in.module * 128 * 128 + 128 * packet_in.eth_packet + packet_in.axis_packet;
				for (int i = 0; i < 8; i ++) {

					ap_uint<512> packed_pedeG0RMS;
					ap_uint<512> packed_pedeG1;
					ap_uint<512> packed_pedeG2;
					ap_uint<512> packed_gainG0;
					ap_uint<512> packed_gainG1;
					ap_uint<512> packed_gainG2;

					memcpy(&packed_pedeG1, d_hbm_p0+offset+i, 64);
					memcpy(&packed_pedeG2, d_hbm_p1+offset+i, 64);
					memcpy(&packed_gainG0, d_hbm_p2+offset+i, 64);
					memcpy(&packed_gainG1, d_hbm_p3+offset+i, 64);
					memcpy(&packed_gainG2, d_hbm_p4+offset+i, 64);
					memcpy(&packed_pedeG0RMS, d_hbm_p5+offset+i, 64);
					convert_and_shuffle(packet_in.data, packet_out.data, packed_pedeG0[offset+i], packed_pedeG0RMS, packed_gainG0, packed_pedeG1, packed_gainG1, packed_pedeG2, packed_gainG2);

					packet_out.exit = packet_in.exit;
					packet_out.axis_packet = packet_in.axis_packet;
					packet_out.eth_packet = packet_in.eth_packet;
					packet_out.axis_user = packet_in.axis_user;
					packet_out.frame_number = packet_in.frame_number;
					packet_out.module = packet_in.module;
					packet_out.trigger = packet_in.trigger;

					in.read(packet_in);
					out.write(packet_out);
				}
			}
			while ((packet_in.exit != 1) && (packet_in.axis_packet % 8 != 0)) in.read(packet_in);
		}
	}

	out.write(packet_in);
}


void convert_and_shuffle(ap_uint<512> data_in, ap_uint<512>& data_out,
		packed_pedeG0_t& packed_pedeG0, ap_uint<512> packed_pedeG0RMS, ap_uint<512> packed_gainG0,
		ap_uint<512> packed_pedeG1, ap_uint<512> packed_gainG1,
		ap_uint<512> packed_pedeG2, ap_uint<512> packed_gainG2) {
#pragma HLS PIPELINE
	const ap_fixed<18,16, SC_RND_CONV> half = 0.5f;

	ap_uint<16> in_val[32];
	ap_int<16> out_val[32];
	Loop0: for (int i = 0; i < 512; i++) in_val[i/16][i%16] = data_in[i];

	pedeG0_t pedeG0[32];
	pedeG0RMS_t pedeG0RMS[32];
	gainG0_t gainG0[32];
	pedeG1G2_t pedeG1[32];
	pedeG1G2_t pedeG2[32];
	gainG1G2_t gainG1[32];
	gainG1G2_t gainG2[32];

	unpack_pedeG0(packed_pedeG0, pedeG0);

	unpack_pedeG0RMS(packed_pedeG0RMS, pedeG0RMS);
	unpack_gainG0(packed_gainG0, gainG0);
	unpack_pedeG1G2(packed_pedeG1,pedeG1);
	unpack_pedeG1G2(packed_pedeG2,pedeG2);
	unpack_gainG1G2(packed_gainG1,gainG1);
	unpack_gainG1G2(packed_gainG2,gainG2);

	Loop1: for (int i = 0; i < 32; i++) {

		if (in_val[i] == 0xffff) out_val[i] = 32766; // can saturate G2 - overload
		else if (in_val[i] == 0xc000) out_val[i] = -32700; //cannot saturate G1
		else {
			ap_fixed<18,16, SC_RND_CONV> val_diff;
			ap_fixed<18,16, SC_RND_CONV> val_result;
			ap_uint<2> gain = in_val[i] >>14;
			ap_uint<14> adu = in_val[i]; // take first two bits
			switch (gain) {
			case 0: {
				ap_ufixed<24,14, SC_RND_CONV> val_pede;
				val_pede = pedeG0[i];
				if (adu - val_pede < pedeG0RMS[i]) {
					val_pede += (adu - val_pede) / 128;
					pedeG0[i] = val_pede;
				}
				val_diff = adu - val_pede;
				val_result = val_diff * (gainG0[i] / 512);
				if (val_result >= 0)
					out_val[i] = val_result + half;
				else  out_val[i] = val_result - half;

				break;
			}
			case 1: {
				val_diff     = pedeG1[i] - adu;
				val_result   =  val_diff * gainG1[i];

				if (val_result >= 0)
					out_val[i] = val_result + half;
				else  out_val[i] = val_result - half;
				break;
			}
			case 2:
				out_val[i] = -32700;
				break;
			case 3: {
				val_diff     = pedeG2[i] - adu;
				val_result   = val_diff * gainG2[i];

				if (val_result >= 0)
					out_val[i] = val_result + half;
				else  out_val[i] = val_result - half;

				break;
			}
			}
		}
	}
	// pack_pedeG0(packed_pedeG0, pedeG0);
	data_shuffle(data_out, out_val);
}
