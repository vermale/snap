#include <infiniband/verbs.h>

extern char *ib_device_name;

extern ibv_context *ib_context;
extern ibv_pd *ib_pd;
extern ibv_cq *ib_cq;
extern ibv_qp *ib_qp;
//extern ibv_mr *ib_buffer_mr;
extern ibv_port_attr ib_port_attr;

int setup_ibverbs(std::string ib_device_name, size_t send_queue_size, size_t receive_queue_size);
int switch_to_rtr(uint32_t rq_psn, uint16_t dlid, uint32_t dest_qp_num);
int switch_to_rts(uint32_t sq_psn);
int close_ibverbs();

int exchange_ib_info_client(std::string hostname, unsigned short port, uint16_t local_dlid, uint32_t local_dest_qp_num, 
		uint16_t &remote_dlid, uint32_t &remote_dest_qp_num, uint32_t sq_psn);
int exchange_ib_info_server(unsigned short port, uint16_t local_dlid, uint32_t local_dest_qp_num, 
		uint16_t &remote_dlid, uint32_t &remote_dest_qp_num, uint32_t &rq_psn);