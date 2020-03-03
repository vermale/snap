#include <iostream>
#include <unistd.h>

#include <hdf5.h>

#include "IB_Transport.h"
#include "JFReceiver.h"

int main(int argc, char **argv) {

	uint16_t remote_dlid;
	uint32_t remote_dest_qp_num;
	uint32_t rq_psn;

    // Establish RDMA link
    if (setup_ibverbs("mlx5_0", RDMA_SQ_SIZE, 0) == 1) exit(EXIT_FAILURE);
    std::cout << "IB link ready" << std::endl;

	exchange_ib_info_server(5232, ib_port_attr.lid, ib_qp->qp_num,
			remote_dlid, remote_dest_qp_num, rq_psn);

	if (switch_to_rtr(rq_psn, remote_dlid, remote_dest_qp_num) == 1) exit(EXIT_FAILURE);
	std::cout << "IB Ready to receive" << std::endl;


}
