#ifndef JFWRITER_H_
#define JFWRITER_H_

#include <hdf5.h>
#include "JFApp.h"

#define RDMA_RQ_SIZE 8192L // Maximum number of receive elements
#define NCARDS       1

// HDF5 - data file (should be array for multiple files)
hid_t data_file;
hid_t data_group;
hid_t data_dataspace;
hid_t data_dataset;
hid_t data_dcpl;

// Settings only necessary for writer
struct writer_settings_t {
	std::string HDF5_prefix;
	int images_per_file;
};
extern writer_settings_t writer_settings;

struct writer_connection_settings_t {
	int sockfd;                 // TCP/IP socket
	std::string receiver_host;  // Receiver host
	uint16_t receiver_tcp_port; // Receiver TCP port
	std::string ib_dev_name;    // IB device name
	ib_settings_t ib_settings;  // IB settings
};

void *writer_thread(void* threadArg);

struct gain_pedestal_t {
	uint16_t gainG0[NPIXEL];
	uint16_t gainG1[NPIXEL];
	uint16_t gainG2[NPIXEL];
	uint16_t pedeG1[NPIXEL];
	uint16_t pedeG2[NPIXEL];
	uint16_t pedeG0[NPIXEL];
};

extern gain_pedestal_t gain_pedestal[NCARDS];
extern online_statistics_t online_statistics[NCARDS];

extern experiment_settings_t experiment_settings;
extern writer_connection_settings_t writer_connection_settings[NCARDS];

extern uint8_t writers_done_per_file;
extern pthread_mutex_t writers_done_per_file_mutex;
extern pthread_cond_t writers_done_per_file_cond;

#endif // JFWRITER_H_
