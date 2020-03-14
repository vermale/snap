#include "JFReceiver.h"

size_t frame_buffer_size = 0;
size_t status_buffer_size = 0;
size_t gain_pedestal_data_size = 0;
size_t jf_packet_headers_size = 0;
size_t ib_outgoing_buffer_size = RDMA_BUFFER_MAX_ELEM_SIZE * RDMA_SQ_SIZE;

// Settings
uint32_t card_no = 1;

uint64_t nframes_to_collect = FRAME_BUF_SIZE;
uint64_t nframes_to_write   = 8000;

uint32_t compression_threads = 16;
uint64_t pedestalG0 = 2000;
uint8_t  conversion_mode = MODE_RAW;
uint64_t fpga_mac_addr = 0xAABBCCDDEEF1;
uint64_t fpga_ip_addr = 0x0A013205;

// Experimental info
double energy_in_keV = 12.4;

// File paths
std::string gainFileName[NMODULES] = {
"/home/jungfrau/JF2M_X06SA_200310/gainMaps_M351_2020-01-20.bin",
"/home/jungfrau/JF2M_X06SA_200310/gainMaps_M312_2020-01-20.bin",
"/home/jungfrau/JF2M_X06SA_200310/gainMaps_M261_2019-07-29.bin",
"/home/jungfrau/JF2M_X06SA_200310/gainMaps_M373_2020-01-31.bin",
};

std::string pedestalFileName = "pedestal.dat";

std::string file_server_host = "mx-jungfrau-1";
std::string ib_dev_name = "mlx5_0";
int fpga_card_number = 1;

uint32_t trigger_frame;
pthread_mutex_t trigger_frame_mutex = PTHREAD_MUTEX_INITIALIZER; 

// Buffers for communication with the FPGA
int16_t *frame_buffer = NULL;
online_statistics_t *online_statistics = NULL;
header_info_t *jf_packet_headers = NULL;
char *status_buffer = NULL;
uint16_t *gain_pedestal_data = NULL;
char *packet_counter = NULL;
char *ib_outgoing_buffer = NULL;
ibv_mr *ib_outgoing_buffer_mr = NULL;



