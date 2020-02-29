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

void write_data(DATA_STREAM &in, snap_membus_t *dout_gmem, size_t out_frame_buffer_addr, size_t out_frame_status_addr, snap_HBMbus_t *d_hbm_stat) {
	data_packet_t packet_in;
	in.read(packet_in);

	int counter_ok = 0;
	int counter_wrong = 0;

	ap_uint<256> hbm_cache[NMODULES]; // Cache for HBM statistics
	ap_uint<28> hbm_cache_addr[NMODULES];
	for (int i = 0; i < NMODULES; i++) {
#pragma HLS UNROLL
		hbm_cache[i] = 0;
		hbm_cache_addr[i] = 0;
	}

	uint64_t head[NMODULES]; // number of the newest packet received for the frame
#pragma HLS RESOURCE variable=head core=RAM_1P_LUTRAM
	for (int i = 0; i < NMODULES; i++) {
#pragma HLS UNROLL
		head[i] = 0L;
	}

	while (packet_in.exit == 0) {

		Loop_good_packet: while ((packet_in.exit == 0) && (packet_in.axis_packet == 0)) {

			// TODO: accounting which packets were converted
#pragma HLS PIPELINE II=129
			size_t out_frame_addr = out_frame_buffer_addr +
					       (packet_in.frame_number % FRAME_BUF_SIZE) * (NMODULES * MODULE_COLS * MODULE_LINES / 32) +
							packet_in.module * (MODULE_COLS * MODULE_LINES/32) +
							packet_in.eth_packet * (4096/32);

			ap_uint<512> buffer[128];

			ap_uint<24> frame_number0 = packet_in.frame_number;
			ap_uint<4> module0 = packet_in.module;
			ap_uint<8> eth_packet0 = packet_in.eth_packet;

			ap_uint<28> hbm_cell_addr = (packet_in.frame_number / 2) * NMODULES + packet_in.module;
			ap_uint<8> hbm_bit_addr = (packet_in.frame_number % 2) *128 + packet_in.eth_packet;

			if (hbm_cache_addr[packet_in.module] != hbm_cell_addr) {
				d_hbm_stat[hbm_cache_addr[packet_in.module]] = hbm_cache[packet_in.module];
				hbm_cache_addr[packet_in.module] = hbm_cell_addr;
				hbm_cache[packet_in.module] = d_hbm_stat[hbm_cell_addr];
			}

			if (packet_in.frame_number > head[packet_in.module]) {
				//is_head = true;
				//for (int i = 0; i < NMODULES; i++)
				//	if (head[i] >= packet_in.frame_number) is_head = false;

				head[packet_in.module] = packet_in.frame_number;
				ap_uint<512> statistics = 0;

				statistics(31,0) = counter_ok;
				statistics(63,32) = counter_wrong;

				for (int i = 0; i < NMODULES; i++) {
					statistics(64 + i * 64 + 63, 64 + i * 64) = head[i];
				}

				// Status info is filled only every NMODULES frames, but interleaved between modules.
				if (packet_in.frame_number % (NMODULES) == packet_in.module)
					memcpy(dout_gmem+out_frame_status_addr, &statistics, BPERDW);
			}

			ap_uint<1> last_axis_user;

			buffer[0] = packet_in.data;
			for (int i = 1; i < 128; i++) {
				in.read(packet_in);
				buffer[i] = packet_in.data;
			}

			memcpy(dout_gmem + out_frame_addr, buffer, 128*64);

			if ((packet_in.axis_packet == 127) && (packet_in.axis_user == 0)) {
				hbm_cache[module0][hbm_bit_addr] = 1;
				counter_ok++;
			} else counter_wrong++;

			in.read(packet_in);
		}
		// forward, to get to a beginning of a meaningful packet:
		Loop_err_packet: while ((packet_in.exit == 0) && (packet_in.axis_packet != 0))
			in.read(packet_in);
	}
	ap_uint<512> statistics = 0;

	for (int i = 0; i < NMODULES; i++)
		d_hbm_stat[hbm_cache_addr[i]] = hbm_cache[i];


	statistics(31,0) = counter_ok;
	statistics(63,32) = counter_wrong;

	for (int i = 0; i < NMODULES; i++) {
		statistics(64 + i * 64 + 63, 64 + i * 64) = head[i];
	}
	memcpy(dout_gmem+out_frame_status_addr, &statistics, BPERDW);

}
