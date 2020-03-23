#ifndef _JFRECEIVER_H
#define _JFRECEIVER_H

#include "JFApp.h"

#define FRAME_LIMIT 1000000L

#define RECEIVING_DELAY 5

#define RDMA_SQ_PSN 532
#define RDMA_SQ_SIZE 1024L // Maximum number of write elements

#define PTHREAD_ERROR(ret,func) if (ret) printf("%s(%d) %s: err = %d\n",__FILE__,__LINE__, #func, ret), exit(ret)

#define TIMEOUT 600

extern experiment_settings_t experiment_settings;

// Settings only necessary for receiver
struct receiver_settings_t {
	uint32_t card_number;
	uint64_t fpga_mac_addr;
	uint64_t fpga_ip_addr;
	uint32_t compression_threads;
	uint16_t tcp_port;
	std::string gain_file_name[NMODULES];
	std::string pedestal_file_name;
	std::string ib_dev_name;
};
extern receiver_settings_t receiver_settings;



// Buffers for communication with the FPGA
extern int16_t *frame_buffer;
extern size_t frame_buffer_size;
extern char *status_buffer;
extern size_t status_buffer_size;
extern uint16_t *gain_pedestal_data;
extern size_t gain_pedestal_data_size;
extern char *packet_counter;
extern size_t jf_packet_headers_size;

// Useful pointers to buffers above
extern online_statistics_t *online_statistics;
extern header_info_t *jf_packet_headers;

// Settings for Infiniband
extern ib_settings_t ib_settings;

// IB buffer
extern size_t ib_buffer_size;
extern char *ib_buffer;

// TCP/IP socket
extern int sockfd;
extern int accepted_socket; // There is only one accepted socket at the time

// Thread information
struct ThreadArg {
	uint16_t ThreadID;
	receiver_settings_t receiver_settings;
};

// Last frame with trigger - for consistency measured only for a single module, protected by mutex
extern uint32_t trigger_frame;
extern pthread_mutex_t trigger_frame_mutex; 
extern pthread_cond_t  trigger_frame_cond;

// IB buffer usage
extern int16_t ib_buffer_occupancy[RDMA_SQ_SIZE];
extern pthread_mutex_t ib_buffer_occupancy_mutex;
extern pthread_cond_t ib_buffer_occupancy_cond;


int setup_snap(uint32_t card_number);
void close_snap();

void *snap_thread(void *in_threadarg);
void *poll_cq_thread(void *in_threadarg);
void *send_thread(void *in_threadarg);

int parse_input(int argc, char **argv);

#endif
