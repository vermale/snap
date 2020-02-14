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

void filter_packets(DATA_STREAM &in, DATA_STREAM &out) {
	data_packet_t packet_in[128];
#pragma HLS RESOURCE variable=packet_in core=RAM_2P_LUTRAM
#pragma HLS ARRAY_PARTITION variable=packet_in complete dim=1
	int index = 0;
	int packets_to_write = 0;

	in.read(packet_in[0]);

	while (packet_in[index].exit != 1) {
#pragma HLS pipeline
		if ((packet_in[index].axis_packet == 127) && (packet_in[index].axis_user == 0)) {
			packets_to_write = 128;
		}
		index = (index + 1) % 128;
		if (packets_to_write > 0) {
			out.write(packet_in[index]);
			packets_to_write--;
		}
		in.read(packet_in[index]);
	}
	while (packets_to_write > 0) {
		index = (index + 1) % 128;
		out.write(packet_in[index]);
		packets_to_write--;
	}
	out.write(packet_in[index]);
}
