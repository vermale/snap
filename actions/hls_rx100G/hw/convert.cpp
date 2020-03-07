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

#define URAM_PARITITION 8
#define HBM_BURST_G0 64
#define HBM_BURST_G1G2 4

void update_pedestal(ap_uint<512> data_in, ap_uint<18*32> &data_out, packed_pedeG0_t &packed_pede, ap_uint<1> accumulate, ap_uint<8> mode) {
#pragma HLS PIPELINE II=1
#pragma HLS INLINE off
	// Load current pedestal
	pedeG0_t pedestal[32];
	unpack_pedeG0(packed_pede, pedestal);

	for (int j = 0; j < 32; j++) {
		ap_uint<2> gain = data_in(16 * j + 15,16 * j + 14);
		ap_uint<14> adu = data_in(16 * j + 13,16 * j);
		ap_fixed<18,16> val_diff = adu - pedestal[j];

		// Correct pedestal based on gain
		if (((gain == 0x0) && (mode == MODE_PEDEG0)) ||
				((gain == 0x1) && (mode == MODE_PEDEG1)) ||
				((gain == 0x3) && (mode == MODE_PEDEG2))) {

			if (accumulate)
				pedestal[j] += ((pedeG0_t) adu) / PEDESTAL_WINDOW_SIZE;
			else
				pedestal[j] += val_diff / PEDESTAL_WINDOW_SIZE;
		}

		// Calculate G0 pedestal correction - anyway - it will be overwritten on next steps

		for (int k = 0; k < 18; k++)
			data_out[j * 18 + k] = val_diff[k];

	}
	// Save pedestal
	pack_pedeG0(packed_pede, pedestal);
}


void pedestalG0(DATA_STREAM &in, DATA_STREAM &out, conversion_settings_t conversion_settings) {
	data_packet_t packet_in, packet_out;

	in >> packet_in;
	while (packet_in.exit == 0) {
		while ((packet_in.exit == 0) && (packet_in.axis_packet % URAM_PARITITION == 0)) {

			ap_uint<8> mode = conversion_settings.conversion_mode;
			ap_uint<28> frame_number = packet_in.frame_number;
			if ((mode == MODE_CONV) && (conversion_settings.pedestalG0_frames < frame_number)) mode = MODE_PEDEG0;

			ap_uint<1> accumulate_pede;
			if (frame_number < PEDESTAL_WINDOW_SIZE) accumulate_pede = 1;
			else accumulate_pede = 0;
#pragma HLS PIPELINE II=8
			size_t offset = packet_in.module * 128 * 128 + 128 * packet_in.eth_packet + packet_in.axis_packet;
			for (int i = 0; i < URAM_PARITITION; i++) {
				// Copy old packet
				packet_out = packet_in;

				update_pedestal(packet_in.data, packet_out.conv_data,packed_pedeG0[offset+i], accumulate_pede, mode);
				// Send packet out
				out << packet_out;
				in >> packet_in;
			}
		}
		while ((packet_in.exit == 0) && (packet_in.axis_packet % URAM_PARITITION != 0)) in >> packet_in;
	}
	out << packet_in;
}

void gainG0_correction(ap_uint<32*18> data_in, ap_uint<32*18> &data_out, ap_uint<256> packed_gainG0_1, ap_uint<256> packed_gainG0_2) {
#pragma HLS PIPELINE II=1
	gainG0_t gainG0[32];
	unpack_gainG0 (packed_gainG0_1, packed_gainG0_2, gainG0);
	for (int j = 0; j < 32; j++) {
		ap_fixed<18,16> val_diff;
		ap_fixed<18,16> val_result;
		// Unpack differences
		for (int k = 0; k < 18; k++)
			val_diff[k] = data_in[j * 18 + k];
		// Multiply by gain factor
		val_result = val_diff * (gainG0[j] / 512);

		// Write outcome
		for (int k = 0; k < 18; k++)
			data_out[j * 18 + k] = val_result[k];
	}

}

void gainG0(DATA_STREAM &in, DATA_STREAM &out, snap_HBMbus_t *d_hbm_p0, snap_HBMbus_t *d_hbm_p1) {
	data_packet_t packet_in, packet_out;
	in >> packet_in;
	while (packet_in.exit == 0) {
		while ((packet_in.exit == 0) && (packet_in.axis_packet % HBM_BURST_G0 == 0)) {
#pragma HLS PIPELINE II=4
			ap_uint<256> packed_gainG0_1[HBM_BURST_G0], packed_gainG0_2[HBM_BURST_G0];
			size_t offset = packet_in.module * 128 * 128 + 128 * packet_in.eth_packet + packet_in.axis_packet;
			memcpy(packed_gainG0_1,d_hbm_p0+offset, HBM_BURST_G0*32);
			memcpy(packed_gainG0_2,d_hbm_p1+offset, HBM_BURST_G0*32);

			for (int i = 0; i < HBM_BURST_G0; i ++) {
				packet_out.axis_packet = packet_in.axis_packet;
				packet_out.axis_user = packet_in.axis_user;
				packet_out.data = packet_in.data;
				packet_out.eth_packet = packet_in.eth_packet;
				packet_out.exit = packet_in.exit;
				packet_out.frame_number = packet_in.frame_number;
				packet_out.module = packet_in.module;
				packet_out.trigger = packet_in.trigger;
				gainG0_correction(packet_in.conv_data, packet_out.conv_data, packed_gainG0_1[i], packed_gainG0_2[i]);
				out << packet_out;
				in >> packet_in;
			}
		}
		while ((packet_in.exit == 0) && (packet_in.axis_packet % HBM_BURST_G0 != 0)) in >> packet_in;
	}
	out << packet_in;
}

void gainG1_correction(ap_uint<512> data_in, ap_uint<32*18> old_data, ap_uint<32*18> &data_out, ap_uint<256> packed_gain_1, ap_uint<256> packed_gain_2,
		ap_uint<256> packed_pede_1, ap_uint<256> packed_pede_2) {
#pragma HLS PIPELINE II=1

	ap_uint<32*18> mask = 0;
	ap_uint<32*18> new_val = 0;

	gainG1G2_t gainG1G2[32];
	pedeG1G2_t pedeG1G2[32];

	unpack_gainG1G2 (packed_gain_1, packed_gain_2, gainG1G2);
	unpack_pedeG1G2 (packed_pede_1, packed_pede_2, pedeG1G2);

	for (int i = 0; i < 32; i++) {
		ap_uint<2> gain = data_in(16 * i + 15,16 * i + 14);
		ap_uint<14> adu = data_in(16 * i + 13,16 * i);

		ap_fixed<18,16> val_diff;
		ap_fixed<18,16> val_result;
		if (adu == 0x0) val_result = -32768;
		else if (adu == 0x3fff) val_result = -32768;
		else
			val_result = gainG1G2[i] * (pedeG1G2[i] - (pedeG1G2_t)adu);
		// Write outcome
		for (int k = 0; k < 18; k++) {
			if (gain == 0x1)
				data_out[i * 18 + k] = val_result[k];
			else data_out[i * 18 + k] = old_data[i* 18 + k];
		}
	}

}

void gainG2_correction(ap_uint<512> data_in, ap_uint<32*18> old_data, ap_uint<32*18> &data_out, ap_uint<256> packed_gain_1, ap_uint<256> packed_gain_2,
		ap_uint<256> packed_pede_1, ap_uint<256> packed_pede_2) {
#pragma HLS PIPELINE II=1

	ap_uint<32*18> mask = 0;
	ap_uint<32*18> new_val = 0;

	gainG1G2_t gainG1G2[32];
	pedeG1G2_t pedeG1G2[32];

	unpack_gainG1G2 (packed_gain_1, packed_gain_2, gainG1G2);
	unpack_pedeG1G2 (packed_pede_1, packed_pede_2, pedeG1G2);

	for (int i = 0; i < 32; i++) {

		ap_uint<2> gain = data_in(16 * i + 15,16 * i + 14);
		ap_uint<14> adu = data_in(16 * i + 13,16 * i);

		ap_fixed<18,16> val_result;
		if (adu == 0x0) val_result = 32768;
		else if (adu == 0x3fff) val_result = -32768+1;
		else
		val_result = gainG1G2[i] * (pedeG1G2[i] - (pedeG1G2_t)adu);

		// Write outcome
		for (int k = 0; k < 18; k++) {
			if (gain == 0x3)
				data_out[i * 18 + k] = val_result[k];
			else data_out[i * 18 + k] = old_data[i* 18 + k];
		}
	}
}


void correctG1(DATA_STREAM &in, DATA_STREAM &out, snap_HBMbus_t *d_hbm_p0, snap_HBMbus_t *d_hbm_p1,
		snap_HBMbus_t *d_hbm_p2, snap_HBMbus_t *d_hbm_p3) {
	data_packet_t packet_in, packet_out;
	in >> packet_in;
	while (packet_in.exit == 0) {
		while ((packet_in.exit == 0) && (packet_in.axis_packet % HBM_BURST_G1G2 == 0)) {
#pragma HLS PIPELINE II=4
			size_t offset = packet_in.module * 128 * 128 + 128 * packet_in.eth_packet + packet_in.axis_packet;

			ap_uint<256> packed_gain_1[HBM_BURST_G1G2], packed_gain_2[HBM_BURST_G1G2];
			ap_uint<256> packed_pede_1[HBM_BURST_G1G2], packed_pede_2[HBM_BURST_G1G2];

			memcpy(packed_gain_1,d_hbm_p0+offset, HBM_BURST_G1G2*32);
			memcpy(packed_gain_2,d_hbm_p1+offset, HBM_BURST_G1G2*32);
			memcpy(packed_pede_1,d_hbm_p2+offset, HBM_BURST_G1G2*32);
			memcpy(packed_pede_2,d_hbm_p3+offset, HBM_BURST_G1G2*32);

			for (int i = 0; i < HBM_BURST_G1G2; i ++) {
				packet_out.axis_packet = packet_in.axis_packet;
				packet_out.axis_user = packet_in.axis_user;
				packet_out.data = packet_in.data;
				packet_out.eth_packet = packet_in.eth_packet;
				packet_out.exit = packet_in.exit;
				packet_out.frame_number = packet_in.frame_number;
				packet_out.module = packet_in.module;
				packet_out.trigger = packet_in.trigger;

				gainG1_correction(packet_in.data, packet_in.conv_data, packet_out.conv_data, packed_gain_1[i], packed_gain_2[i],
						packed_pede_1[i], packed_pede_2[i]);
				out << packet_out;
				in >> packet_in;
			}
		}
		while ((packet_in.exit == 0) && (packet_in.axis_packet % HBM_BURST_G1G2 != 0)) in >> packet_in;
	}
	out << packet_in;
}

void correctG2(DATA_STREAM &in, DATA_STREAM &out, snap_HBMbus_t *d_hbm_p0, snap_HBMbus_t *d_hbm_p1,
		snap_HBMbus_t *d_hbm_p2, snap_HBMbus_t *d_hbm_p3) {
	data_packet_t packet_in;
	in >> packet_in;
	while (packet_in.exit == 0) {
		while ((packet_in.exit == 0) && (packet_in.axis_packet % HBM_BURST_G1G2 == 0)) {
#pragma HLS PIPELINE II=4
			size_t offset = packet_in.module * 128 * 128 + 128 * packet_in.eth_packet + packet_in.axis_packet;

			ap_uint<256> packed_gain_1[HBM_BURST_G1G2], packed_gain_2[HBM_BURST_G1G2];
			ap_uint<256> packed_pede_1[HBM_BURST_G1G2], packed_pede_2[HBM_BURST_G1G2];

			memcpy(packed_gain_1,d_hbm_p0+offset, HBM_BURST_G1G2*32);
			memcpy(packed_gain_2,d_hbm_p1+offset, HBM_BURST_G1G2*32);
			memcpy(packed_pede_1,d_hbm_p2+offset, HBM_BURST_G1G2*32);
			memcpy(packed_pede_2,d_hbm_p3+offset, HBM_BURST_G1G2*32);

			for (int i = 0; i < HBM_BURST_G1G2; i ++) {
				data_packet_t packet_out;
				packet_out.axis_packet = packet_in.axis_packet;
				packet_out.axis_user = packet_in.axis_user;
				packet_out.data = packet_in.data;
				packet_out.eth_packet = packet_in.eth_packet;
				packet_out.exit = packet_in.exit;
				packet_out.frame_number = packet_in.frame_number;
				packet_out.module = packet_in.module;
				packet_out.trigger = packet_in.trigger;

				gainG2_correction(packet_in.data, packet_in.conv_data, packet_out.conv_data, packed_gain_1[i], packed_gain_2[i],
						packed_pede_1[i], packed_pede_2[i]);
				out << packet_out;
				in >> packet_in;
			}
		}
		while ((packet_in.exit == 0) && (packet_in.axis_packet % HBM_BURST_G1G2 != 0)) in >> packet_in;
	}
	out << packet_in;
}

void merge_converted_stream(DATA_STREAM &in, DATA_STREAM &out, ap_uint<2> convert) {
	const ap_fixed<18,16> half = 0.5f;
	data_packet_t packet_in, packet_out;
	in >> packet_in;
	while (packet_in.exit == 0) {
#pragma HLS PIPELINE II=1
		packet_out = packet_in;
		if (convert == 1) {
			for (int i = 0; i < 32; i++) {
				ap_int<16> rounded;
				ap_fixed<18,16, AP_TRN_ZERO> original;
				for (int j = 0; j < 18; j ++)
					original[j] = packet_in.conv_data[i * 18 + j];
				rounded = original + half;
				packet_out.data(i * 16 + 15, i * 16) = rounded;
			}
		} else if (convert == 2) {
			for (int i = 0; i < 32; i++) {
				ap_int<16> rounded;
				ap_fixed<18,16, AP_TRN_ZERO> original;
				for (int j = 0; j < 18; j ++)
					original[j] = packet_in.conv_data[i * 18 + j];
				rounded = original + half;

				for (int j = 0; j < 16; j ++) {
					packet_out.data[j * 32 + i] = rounded[j];
				}
			}
		}
		out << packet_out;
		in >> packet_in;
	}
	out << packet_in;
}
