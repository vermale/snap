#ifndef _JFRECEIVER_H
#define _JFRECEIVER_H

#include <stdlib.h>
#include <string>
#include <pthread.h>
#include <lz4.h>
#include <infiniband/verbs.h>

#include "../include/action_rx100G.h"

#define RECEIVING_DELAY 5
#define RDMA_BUFFER_MAX_ELEM_SIZE (LZ4_compressBound(NPIXEL * 2) + 20)
#define RDMA_SQ_SIZE 8192L // Maximum receive ring length without processing

#define PTHREAD_ERROR(ret,func) if (ret) printf("%s(%d) %s: err = %d\n",__FILE__,__LINE__, #func, ret), exit(ret)

#define TIMEOUT 600

// Settings
extern uint32_t card_no;

extern uint64_t nframes_to_collect;
extern uint64_t nframes_to_write;

extern uint32_t compression_threads;
extern uint64_t pedestalG0;
extern uint8_t  conversion_mode;
extern uint64_t fpga_mac_addr;
extern uint64_t fpga_ip_addr;

// Experimental info
extern double energy_in_keV;

// File paths
extern std::string gainFileName[NMODULES];
extern std::string pedestalFileName;
extern std::string file_server_host;
extern std::string ib_dev_name;
extern int fpga_card_number;

// Buffers size
extern size_t frame_buffer_size;
extern size_t status_buffer_size;
extern size_t gain_pedestal_data_size;
extern size_t jf_packet_headers_size;
extern size_t ib_outgoing_buffer_size;

// Buffers for communication with the FPGA
extern int16_t *frame_buffer;
extern online_statistics_t *online_statistics;
extern header_info_t *jf_packet_headers;
extern char *status_buffer;
extern uint16_t *gain_pedestal_data;
extern char *packet_counter;
extern char *ib_outgoing_buffer;

// IB data structures
extern ibv_mr *ib_outgoing_buffer_mr;

struct ThreadArg {
	uint16_t ThreadID;
};

// Threaded 
extern uint32_t trigger_frame;
extern pthread_mutex_t trigger_frame_mutex; 

void *SnapThread(void *in_threadarg);
void *CompressAndSendThread(void *in_threadarg);

int parse_input(int argc, char **argv);

#endif
