#ifndef JFAPP_H_
#define JFAPP_H_

#include <stdlib.h>
#include <string>
#include <pthread.h>
#include <lz4.h>
#include <infiniband/verbs.h>

#include "../include/action_rx100G.h"

#define RDMA_BUFFER_MAX_ELEM_SIZE (LZ4_compressBound(NPIXEL * 2) + 16)

#define TCPIP_CONN_MAGIC_NUMBER 123434L
#define TCPIP_DONE_MAGIC_NUMBER  56789L

#define H5Z_FILTER_JF  52320

// Settings exchanged between writer and receiver
struct experiment_settings_t {
	uint64_t nframes_to_collect;
	uint64_t nframes_to_write;
	uint8_t conversion_mode;
	uint64_t pedestalG0_frames;
	double energy_in_keV;
};

// Settings for IB connection
struct ib_comm_settings_t {
	uint16_t dlid;
	uint32_t qp_num;
	uint32_t rq_psn;
	uint32_t frame_buffer_rkey;
	uint64_t frame_buffer_remote_addr;
};

// IB context
struct ib_settings_t {
	// IB data structures
	ibv_mr *buffer_mr;
	ibv_context *context;
	ibv_pd *pd;
	ibv_cq *cq;
	ibv_qp *qp;
	ibv_port_attr port_attr;
};

// IB Verbs function wrappers
int setup_ibverbs(ib_settings_t &settings, std::string ib_device_name, size_t send_queue_size, size_t receive_queue_size);
int switch_to_rtr(ib_settings_t &settings, uint32_t rq_psn, uint16_t dlid, uint32_t dest_qp_num);
int switch_to_rts(ib_settings_t &settings, uint32_t sq_psn);
int close_ibverbs(ib_settings_t &settings);

#endif // JFAPP_H_
