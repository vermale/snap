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

inline void load(ap_uint<512> &out, snap_HBMbus_t *d_hbm_p0, snap_HBMbus_t *d_hbm_p1, size_t offset) {
	out(511,256) = d_hbm_p1[offset];
	out(255,0) = d_hbm_p0[offset];
}

void pedestal_update(ap_uint<512> data_in, packed_pedeG0_t& packed_pede, ap_uint<2> exp_gain) {
	pedeG0_t pedestal[32];
	unpack_pedeG0(packed_pede, pedestal);
	ap_uint<16> in_val[32];

	Loop0: for (int i = 0; i < 512; i++) in_val[i/16][i%16] = data_in[i];

	Loop1: for (int i = 0; i < 32; i++) {
		ap_uint<2> gain = in_val[i](15,14);
		pedeG0_t tmp = in_val[i](13,0);
		if (gain == exp_gain) {
			// Rolling average
			pedestal[i] += (tmp - pedestal[i]) / PEDESTAL_WINDOW_SIZE;
		}
	}
	pack_pedeG0(packed_pede, pedestal);
}

#define BURST_SIZE 16

void convert_data(DATA_STREAM &in, DATA_STREAM &out,
		snap_HBMbus_t *d_hbm_p0, snap_HBMbus_t *d_hbm_p1,
		snap_HBMbus_t *d_hbm_p2, snap_HBMbus_t *d_hbm_p3,
		snap_HBMbus_t *d_hbm_p4, snap_HBMbus_t *d_hbm_p5,
		snap_HBMbus_t *d_hbm_p6, snap_HBMbus_t *d_hbm_p7,
		snap_HBMbus_t *d_hbm_p8, snap_HBMbus_t *d_hbm_p9,
		conversion_settings_t conversion_settings) {

	data_packet_t packet_in, packet_out;
	in.read(packet_in);

	while (packet_in.exit != 1) {
		Convert_and_forward: while ((packet_in.exit != 1) && (packet_in.axis_packet % BURST_SIZE == 0)) {
#pragma HLS pipeline II = 16
			size_t offset = packet_in.module * 128 * 128 + 128 * packet_in.eth_packet + packet_in.axis_packet;
			// HBM Order:
			// p0,p1 - gain G0
			// p2,p3 - gain G1
			// p4,p5 - gain G2
			// p6,p7 - pedestal G1
			// p8,p9 - pedestal G2

			ap_uint<256> packed_gainG0_1[BURST_SIZE], packed_gainG0_2[BURST_SIZE];
			ap_uint<256> packed_gainG1_1[BURST_SIZE], packed_gainG1_2[BURST_SIZE];
			ap_uint<256> packed_gainG2_1[BURST_SIZE], packed_gainG2_2[BURST_SIZE];
			ap_uint<256> packed_pedeG1_1[BURST_SIZE], packed_pedeG1_2[BURST_SIZE];
			ap_uint<256> packed_pedeG2_1[BURST_SIZE], packed_pedeG2_2[BURST_SIZE];

			memcpy(packed_gainG0_1,d_hbm_p0+offset, BURST_SIZE*32);
			memcpy(packed_gainG0_2,d_hbm_p1+offset, BURST_SIZE*32);
			memcpy(packed_gainG1_1,d_hbm_p2+offset, BURST_SIZE*32);
			memcpy(packed_gainG1_2,d_hbm_p3+offset, BURST_SIZE*32);
			memcpy(packed_gainG2_1,d_hbm_p4+offset, BURST_SIZE*32);
			memcpy(packed_gainG2_2,d_hbm_p5+offset, BURST_SIZE*32);
			memcpy(packed_pedeG1_1,d_hbm_p6+offset, BURST_SIZE*32);
			memcpy(packed_pedeG1_2,d_hbm_p7+offset, BURST_SIZE*32);
			memcpy(packed_pedeG2_1,d_hbm_p8+offset, BURST_SIZE*32);
			memcpy(packed_pedeG2_2,d_hbm_p9+offset, BURST_SIZE*32);


			for (int z = 0; z < BURST_SIZE/8; z++) {
				data_packet_t packet_buffer[8];

				packet_buffer[0] = packet_in;

				for (int i = 1; i < 8; i ++) {
					in.read(packet_buffer[i]);
				}
				for (int i = 0; i < 8; i ++) {
					ap_uint<512> tmp;
					ap_uint<8> conversion_mode = conversion_settings.conversion_mode;
					if ((conversion_mode == MODE_CONV) && (packet_in.frame_number < conversion_settings.pedestalG0_frames))
						conversion_mode = MODE_PEDEG0;
					convert_and_shuffle(packet_buffer[i].data, tmp, packed_pedeG0[offset+z*8+i],
							packed_gainG0_1[i+z*8], packed_gainG0_2[i+z*8],
							packed_pedeG1_1[i+z*8], packed_pedeG1_2[i+z*8], packed_gainG1_1[i+z*8], packed_gainG1_2[i+z*8],
							packed_pedeG2_1[i+z*8], packed_pedeG2_2[i+z*8], packed_gainG2_1[i+z*8], packed_gainG2_2[i+z*8],
							conversion_mode);

					if (conversion_settings.conversion_mode == MODE_CONV) packet_buffer[i].data = tmp;
				}
				for (int i = 0; i < 8; i ++) {
					out.write(packet_buffer[i]);
				}
				in.read(packet_in);
			}
		}
		while ((packet_in.exit != 1) && (packet_in.axis_packet % BURST_SIZE != 0)) in.read(packet_in);
	}
	out.write(packet_in);
}


void convert_and_shuffle(ap_uint<512> data_in, ap_uint<512> &data_out,
		packed_pedeG0_t& packed_pedeG0,
		ap_uint<256> packed_gainG0_1, ap_uint<256> packed_gainG0_2,
		ap_uint<256> packed_pedeG1_1, ap_uint<256> packed_pedeG1_2,
		ap_uint<256> packed_gainG1_1, ap_uint<256> packed_gainG1_2,
		ap_uint<256> packed_pedeG2_1, ap_uint<256> packed_pedeG2_2,
		ap_uint<256> packed_gainG2_1, ap_uint<256> packed_gainG2_2,
		const ap_uint<8> mode) {
#pragma HLS PIPELINE
#pragma HLS INLINE off
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

	//unpack_pedeG0RMS(packed_pedeG0RMS_1, packed_pedeG0RMS_2, pedeG0RMS);
	unpack_gainG0  (packed_gainG0_1, packed_gainG0_2, gainG0);
	unpack_pedeG1G2(packed_pedeG1_1, packed_pedeG1_2, pedeG1);
	unpack_pedeG1G2(packed_pedeG2_1, packed_pedeG2_2, pedeG2);
	unpack_gainG1G2(packed_gainG1_1, packed_gainG1_2, gainG1);
	unpack_gainG1G2(packed_gainG2_1, packed_gainG2_2, gainG2);

	Convert: for (int i = 0; i < 32; i++) {

		if (in_val[i] == 0x3fff) out_val[i] = 32766; // can saturate G2 - overload
		else if (in_val[i] == 0xffff) out_val[i] = -32700; //error
		else if (in_val[i] == 0xc000) out_val[i] = -32700; //cannot saturate G1 - error
		else {
			ap_fixed<18,16, SC_RND_CONV> val_diff;
			ap_fixed<18,16, SC_RND_CONV> val_result = 0;
			ap_uint<2> gain = in_val[i] >>14;
			ap_uint<14> adu = in_val[i]; // take first two bits
			switch (gain) {
			case 0: {

				val_diff = adu - pedeG0[i];
				if (mode == MODE_PEDEG0) pedeG0[i] += ((pedeG0_signed_t) val_diff) / 128;
				val_result = val_diff * (gainG0[i] / 512);
				if (val_result >= 0)
					out_val[i] = val_result + half;
				else  out_val[i] = val_result - half;
				break;
			}
			case 1: {
				val_diff     = pedeG1[i] - adu;
				if (mode == MODE_PEDEG1) {
					pedeG0[i] += ((pedeG0_signed_t) val_diff) / -128;
				}

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
				if (mode == MODE_PEDEG2) {
					pedeG0[i] += ((pedeG0_signed_t) val_diff) / -128;
				}

				val_result   = val_diff * gainG2[i];
				if (val_result >= 0)
					out_val[i] = val_result + half;
				else  out_val[i] = val_result - half;
				break;
			}
			}
		}
	}
	pack_pedeG0(packed_pedeG0, pedeG0);
	data_shuffle(data_out, out_val);
	//data_pack(data_out, out_val);
}

