#include <iostream>
#include "IB_Transport.h"

// IB data structures
ibv_context *ib_context;
ibv_pd *ib_pd;
ibv_cq *ib_cq;
ibv_qp *ib_qp;
ibv_port_attr ib_port_attr;

int setup_ibverbs(std::string ib_device_name, size_t send_queue_size, size_t receive_queue_size) {
	struct ibv_device **dev_list;
	int num_devices;
	int selected = -1;

	dev_list = ibv_get_device_list(&num_devices);

	if (dev_list == NULL) {
		std::cerr << "Failed to get IB devices list." << std::endl;
		return 1;
	}

	for (int i = 0; i < num_devices; i++) {
		if (strncmp(ib_device_name.c_str(), ibv_get_device_name(dev_list[i]),6) == 0) selected = i;
	}

	if (selected == -1) {
		std::cerr << "IB device not found." << std::endl;
		return 1;
	}

	ib_context = ibv_open_device(dev_list[selected]);
	if (ib_context == NULL) {
		std::cerr << "Failed to open context for IB device." << std::endl;
		return 1;
	}
	ibv_free_device_list(dev_list);

	ib_pd = ibv_alloc_pd(ib_context);
	if (ib_pd == NULL) {
		std::cerr << "Failed to allocate IB protection domain." << std::endl;
		return 1;
	}

	ib_cq = ibv_create_cq(ib_context, send_queue_size + receive_queue_size, NULL, NULL, 0);
	if (ib_cq == NULL) {
		std::cerr << "Failed to create IB completion queue." << std::endl;
		return 1;
	}

	ibv_qp_init_attr qp_init_attr;
	memset(&qp_init_attr, 0, sizeof (qp_init_attr));
	qp_init_attr.send_cq = ib_cq;
	qp_init_attr.recv_cq = ib_cq;
	qp_init_attr.qp_type = IBV_QPT_RC;
	qp_init_attr.cap.max_send_wr = send_queue_size;
	qp_init_attr.cap.max_recv_wr = receive_queue_size;
	qp_init_attr.cap.max_send_sge = 1; // No scatter/gather operations

	ib_qp = ibv_create_qp(ib_pd, &qp_init_attr);
	if (ib_qp == NULL) {
		std::cerr << "Failed to create IB queue pair." << std::endl;
		return 1;
	}

	struct ibv_qp_attr qp_attr;
	int qp_flags;

	memset(&qp_attr, 0, sizeof(qp_attr));

	qp_flags = IBV_QP_STATE | IBV_QP_PORT |IBV_QP_PKEY_INDEX | IBV_QP_ACCESS_FLAGS;
	qp_attr.qp_state = IBV_QPS_INIT;
	qp_attr.port_num = 1;
	qp_attr.qp_access_flags = 0;
	qp_attr.pkey_index = 0;

	int ret = ibv_modify_qp(ib_qp, &qp_attr, qp_flags);
	if (ret < 0)
	{
		std::cerr << "Failed modify IB queue pair to init." << std::endl;
		return 1;
	}

	ibv_query_port(ib_context, 1, &ib_port_attr);
	
	return 0;
}

int switch_to_rtr(uint32_t rq_psn, uint16_t dlid, uint32_t dest_qp_num) {	
	int qp_flags = IBV_QP_STATE | 
			IBV_QP_AV| 
			IBV_QP_PATH_MTU | 
			IBV_QP_DEST_QPN | 
			IBV_QP_RQ_PSN |
			IBV_QP_MAX_DEST_RD_ATOMIC |
			IBV_QP_MIN_RNR_TIMER;
	
	struct ibv_qp_attr qp_attr;
	memset(&qp_attr, 0, sizeof(qp_attr));
	qp_attr.qp_state           = IBV_QPS_RTR;
	qp_attr.path_mtu           = IBV_MTU_2048;
	qp_attr.dest_qp_num        = dest_qp_num;
	qp_attr.rq_psn             = rq_psn;
	qp_attr.max_dest_rd_atomic = 1;
	qp_attr.min_rnr_timer      = 12; // recommended from Mellanox
	qp_attr.ah_attr.dlid       = dlid;
	int ret = ibv_modify_qp(ib_qp, &qp_attr, qp_flags);
	if (ret) {
		std::cerr << "Failed to set IB queue pair to ready to receive" << std::endl;
	}
	return 0;	
}

int switch_to_rts(uint32_t sq_psn) {
	int qp_flags = IBV_QP_STATE | 
			IBV_QP_SQ_PSN |
			IBV_QP_TIMEOUT |
			IBV_QP_RETRY_CNT |
			IBV_QP_RNR_RETRY |
			IBV_QP_MAX_QP_RD_ATOMIC;

	struct ibv_qp_attr qp_attr;
	memset(&qp_attr, 0, sizeof(qp_attr));
	qp_attr.qp_state         = IBV_QPS_RTS;
	qp_attr.sq_psn           = sq_psn;
	qp_attr.timeout          = 14; // recommended from Mellanox
	qp_attr.retry_cnt        = 7;  // recommended from Mellanox
	qp_attr.rnr_retry        = 7;  // recommended from Mellanox
	qp_attr.max_rd_atomic    = 1;
	
	int ret = ibv_modify_qp(ib_qp, &qp_attr, qp_flags);
	
	if (ret) {
		std::cerr << "Failed to set IB queue pair to ready to send" << std::endl;
	}
	return 0;
}

int close_ibverbs() {
	ibv_destroy_qp(ib_qp);
	ibv_destroy_cq(ib_cq);
	ibv_dealloc_pd(ib_pd);
	ibv_close_device(ib_context);
    return 0;
}
