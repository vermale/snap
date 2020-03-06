#include "JFReceiver.h"

size_t frame_buffer_size = 0;
size_t status_buffer_size = 0;
size_t gain_pedestal_data_size = 0;
size_t jf_packet_headers_size = 0;
size_t ib_outgoing_buffer_size = RDMA_BUFFER_MAX_ELEM_SIZE * RDMA_SQ_SIZE;

// Settings
uint32_t card_no = 0;

uint64_t nframes_to_collect = 100;
uint64_t nframes_to_write   = 100;

uint64_t compression_threads = 16;
uint64_t pedestalG0 = 0;
uint8_t  conversion_mode = MODE_RAW;
uint64_t fpga_mac_addr = 0xAABBCCDDEEF1;
uint64_t fpga_ip_addr = 0x0A013205;

// Experimental info
double energy_in_keV = 12.4;

// File paths
std::string gainFileName[NMODULES];
std::string pedestalFileName = "pedestal.dat";



// Buffers for communication with the FPGA
int16_t *frame_buffer = NULL;
online_statistics_t *online_statistics = NULL;
header_info_t *jf_packet_headers = NULL;
char *status_buffer = NULL;
uint16_t *gain_pedestal_data = NULL;
char *packet_counter = NULL;
char *ib_outgoing_buffer = NULL;
ibv_mr *ib_outgoing_buffer_mr = NULL;



