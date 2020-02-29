#ifndef __ACTION_RX100G_H__
#define __ACTION_RX100G_H__

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

#include <snap_types.h>

#define FRAME_BUF_SIZE        16384L
#define FRAME_STATUS_BUF_SIZE 16384

#define NMODULES 4

#define MODULE_COLS 1024L
#define MODULE_LINES 512L

#define NPIXEL (NMODULES*MODULE_COLS*MODULE_LINES)

#define MODE_RAW            0
#define MODE_CONV           1
#define MODE_PEDEG0         2
#define MODE_PEDEG1         3
#define MODE_PEDEG2         4
#define MODE_LOAD_CONSTANTS 5

struct online_statistics_t {
	uint32_t good_packets;
	uint32_t err_packets;
	uint32_t head[NMODULES];
	uint32_t trigger_position;
};

struct header_info_t {
	uint64_t jf_frame_number;
	uint16_t udp_src_port;
	uint16_t udp_dest_port;
	uint32_t jf_debug;
	uint64_t jf_timestamp;
	uint64_t jf_bunch_id;
};


#ifdef __cplusplus
extern "C" {
#endif

/* This number is unique and is declared in ~snap/ActionTypes.md */
#define RX100G_ACTION_TYPE 0x52320100

/* Data structure used to exchange information between action and application */
/* Size limit is 108 Bytes */
typedef struct rx100G_job {
	uint64_t in_gain_pedestal_data_addr;
	uint64_t out_frame_buffer_addr;
	uint64_t out_frame_status_addr;
	uint64_t out_jf_packet_headers_addr;
    uint64_t expected_frames;
    uint64_t pedestalG0_frames;
    uint64_t fpga_mac_addr;
    uint32_t fpga_ipv4_addr;
    uint32_t mode;
} rx100G_job_t;

#ifdef __cplusplus
}
#endif

#endif	/* __ACTION_CHANGECASE_H__ */
