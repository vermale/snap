#include <iostream>
#include <unistd.h>
#include <sys/mman.h>

#include <hdf5.h>

#include "IB_Transport.h"
#include "JFReceiver.h"

size_t nframes_to_write;

int main(int argc, char **argv) {
	nframes_to_write = 50;

	size_t ib_incoming_buffer_size = ((size_t) RDMA_BUFFER_MAX_ELEM_SIZE) * RDMA_SQ_SIZE;
	char *ib_incoming_buffer      = (char *) mmap (NULL, ib_incoming_buffer_size, PROT_READ | PROT_WRITE, MAP_PRIVATE|MAP_ANONYMOUS, -1, 0);
	memset(ib_incoming_buffer, 0x0, ib_incoming_buffer_size);


	// Establish RDMA link
	if (setup_ibverbs("mlx5_0", RDMA_SQ_SIZE, 0) == 1) exit(EXIT_FAILURE);
	std::cout << "IB link ready" << std::endl;

	// Register memory regions
	ibv_mr *ib_incoming_buffer_mr = ibv_reg_mr(ib_pd, ib_incoming_buffer, ib_incoming_buffer_size, 0);
	if (ib_incoming_buffer_mr == NULL) {
		std::cerr << "Failed to register IB memory region." << std::endl;
		return 1;
	}

	// Start receiving
	struct ibv_sge ib_sg_entry;
	struct ibv_recv_wr ib_wr, *ib_bad_recv_wr;

	/* pointer to packet buffer size and memory key of each packet buffer */
	ib_sg_entry.length = RDMA_BUFFER_MAX_ELEM_SIZE;
	ib_sg_entry.lkey = ib_incoming_buffer_mr->lkey;

	ib_wr.num_sge = 1;
    ib_wr.sg_list = &ib_sg_entry;
    ib_wr.next = NULL;

    for (size_t i = 0; i < RDMA_SQ_SIZE; i++)
    {
    	ib_sg_entry.addr = (uint64_t)ib_incoming_buffer + RDMA_BUFFER_MAX_ELEM_SIZE*i;
    	ib_wr.wr_id = i;
    	ibv_post_recv(ib_qp, &ib_wr, &ib_bad_recv_wr);
    }

    // Exchange setup information
    uint16_t remote_dlid;
    uint32_t remote_dest_qp_num;
    uint32_t rq_psn;

    exchange_ib_info_server(5232, ib_port_attr.lid, ib_qp->qp_num,
			remote_dlid, remote_dest_qp_num, rq_psn);

	// Start receiving
	if (switch_to_rtr(rq_psn, remote_dlid, remote_dest_qp_num) == 1) exit(EXIT_FAILURE);
	std::cout << "IB Ready to receive" << std::endl;

	for (size_t frame = 0; frame < nframes_to_write; frame ++) {
		// Poll CQ
		ibv_wc ib_wc;
		int num_comp = 0;
		while (num_comp == 0) {
			num_comp = ibv_poll_cq(ib_cq, 1, &ib_wc);

			if (num_comp < 0) {
				std::cerr << "Failed polling IB Verbs completion queue" << std::endl;
				exit(EXIT_FAILURE);
			}

			if (ib_wc.status != IBV_WC_SUCCESS) {
				std::cerr << "Failed status " << ibv_wc_status_str(ib_wc.status) << " of IB Verbs send request #" << (int)ib_wc.wr_id << std::endl;
				exit(EXIT_FAILURE);
			}
			usleep(1000);
		}
		// Output is in ib_wc.wr_id - do something
		// e.g. write with direct chunk writer

		if (nframes_to_write - frame > RDMA_SQ_SIZE) {
			// Make new work request with the same ID
			// If there is need of new work request
			ib_sg_entry.addr = (uint64_t)ib_incoming_buffer + RDMA_BUFFER_MAX_ELEM_SIZE*ib_wc.wr_id;
			ib_wr.wr_id = ib_wc.wr_id;
			ibv_post_recv(ib_qp, &ib_wr, &ib_bad_recv_wr);
		}
	}

	ibv_dereg_mr(ib_incoming_buffer_mr);

	// Close RDMA
	close_ibverbs();

	// Deallocate memory
	munmap(ib_incoming_buffer, ib_incoming_buffer_size);
}
