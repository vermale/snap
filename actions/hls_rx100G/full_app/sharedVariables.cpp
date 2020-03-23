#include "JFReceiver.h"

size_t frame_buffer_size = 0;
size_t status_buffer_size = 0;
size_t gain_pedestal_data_size = 0;
size_t jf_packet_headers_size = 0;
size_t ib_buffer_size = RDMA_BUFFER_MAX_ELEM_SIZE * RDMA_SQ_SIZE;

receiver_settings_t receiver_settings;
ib_settings_t ib_settings;
experiment_settings_t experiment_settings;

// Last frame with trigger - for consistency measured only for a single module, protected by mutex
uint32_t trigger_frame = 0;
pthread_mutex_t trigger_frame_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  trigger_frame_cond = PTHREAD_COND_INITIALIZER;

// IB buffer usage
int16_t ib_buffer_occupancy[RDMA_SQ_SIZE];
pthread_mutex_t ib_buffer_occupancy_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t ib_buffer_occupancy_cond = PTHREAD_COND_INITIALIZER;

// TCP/IP socket
int sockfd;
int accepted_socket; // There is only one accepted socket at the time

// Buffers for communication with the FPGA
int16_t *frame_buffer = NULL;
online_statistics_t *online_statistics = NULL;
header_info_t *jf_packet_headers = NULL;
char *status_buffer = NULL;
uint16_t *gain_pedestal_data = NULL;
char *packet_counter = NULL;
char *ib_buffer = NULL;



