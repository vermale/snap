#include <iostream>
#include <infiniband/verbs.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>

#include <sys/mman.h>

#include "JFReceiver.h"
#include "IB_Transport.h"

int allocate_memory() {
    frame_buffer_size       = FRAME_BUF_SIZE * NPIXEL * 2; // can store FRAME_BUF_SIZE frames
    status_buffer_size      = nframes_to_collect*NMODULES*128/8+64;   // can store 1 bit per each ETH packet expected
    gain_pedestal_data_size = 6 * 2 * NPIXEL;  // each entry to in_parameters_array is 2 bytes and there are 6 constants per pixel
    jf_packet_headers_size  = nframes_to_collect * NMODULES * sizeof(header_info_t);
    ib_outgoing_buffer_size = ((size_t) RDMA_BUFFER_MAX_ELEM_SIZE) * RDMA_SQ_SIZE;

    // Arrays are allocated with mmap for the higest possible performance. Output is page aligned, so it will be also 64b aligned.
    frame_buffer       = (int16_t *)  mmap (NULL, frame_buffer_size, PROT_READ | PROT_WRITE, MAP_PRIVATE|MAP_ANONYMOUS, -1, 0) ;
    status_buffer      = (char *) mmap (NULL, status_buffer_size, PROT_READ | PROT_WRITE, MAP_PRIVATE|MAP_ANONYMOUS, -1, 0);
    gain_pedestal_data = (uint16_t *) mmap (NULL, gain_pedestal_data_size, PROT_READ | PROT_WRITE, MAP_PRIVATE|MAP_ANONYMOUS, -1, 0);
    jf_packet_headers  = (header_info_t *) mmap (NULL, jf_packet_headers_size, PROT_READ | PROT_WRITE, MAP_PRIVATE|MAP_ANONYMOUS, -1, 0);
    ib_outgoing_buffer        = (char *) mmap (NULL, ib_outgoing_buffer_size, PROT_READ | PROT_WRITE, MAP_PRIVATE|MAP_ANONYMOUS, -1, 0);

    if ((frame_buffer == NULL) || (status_buffer == NULL) ||
        (gain_pedestal_data == NULL) || (jf_packet_headers == NULL) ||
		(ib_outgoing_buffer == NULL)) {
    	std::cerr << "Memory allocation error" << std::endl;
    	return 1;
    }

    // Fill output arrays with zeros
    memset(frame_buffer, 0x0, frame_buffer_size);
    memset(status_buffer, 0x0, status_buffer_size);
    memset(gain_pedestal_data, 0x0, gain_pedestal_data_size);
    memset(jf_packet_headers, 0x0, jf_packet_headers_size);
    memset(ib_outgoing_buffer, 0x0, ib_outgoing_buffer_size);

    packet_counter = (char *) (status_buffer + 64);
    online_statistics = (online_statistics_t *) status_buffer;

    return 0;
}

void deallocate_memory() {
    munmap(frame_buffer, frame_buffer_size);
    munmap(status_buffer, status_buffer_size);
    munmap(gain_pedestal_data, gain_pedestal_data_size);
    munmap(jf_packet_headers, jf_packet_headers_size);
}



int main(int argc, char **argv) {
    int ret;

    std::cout << "JF Receiver " << std::endl;
    // Parse input parameters

    // Allocate memory
    if (allocate_memory() == 1) exit(EXIT_FAILURE);
    std::cout << "Memory allocated" << std::endl; 

    // Load gain and pedestal data

    // Establish RDMA link
    if (setup_ibverbs("mlx5_0", RDMA_SQ_SIZE, 0) == 1) exit(EXIT_FAILURE);
    std::cout << "IB link ready" << std::endl; 

    // Register memory regions
    ibv_mr *ib_incoming_buffer_mr = ibv_reg_mr(ib_pd, ib_outgoing_buffer, ib_outgoing_buffer_size, 0);
    if (ib_incoming_buffer_mr == NULL) {
    	std::cerr << "Failed to register IB memory region." << std::endl;
    	return 1;
    }

    ibv_mr *ib_status_buffer_mr = ibv_reg_mr(ib_pd, status_buffer, status_buffer_size, 0);
    if (ib_status_buffer_mr == NULL) {
        	std::cerr << "Failed to register IB memory region." << std::endl;
        	return 1;
    }

    // Start threads
    pthread_t snapThread1;
//    ret = pthread_create(&snapThread1, NULL, SnapThread, NULL);
//    PTHREAD_ERROR(ret,pthread_create);

    pthread_t compressionThread[compression_threads];
    ThreadArg args[compression_threads];

    for (int i = 0; i < compression_threads ; i++) {
        args[i].ThreadID = i;
//        ret = pthread_create(&compressionThread[i], NULL, CompressAndSendThread, &args[i]);
//        PTHREAD_ERROR(ret,pthread_create);
    }

    uint16_t remote_dlid;
    uint32_t remote_dest_qp_num;
    uint32_t sq_psn = 354;

    exchange_ib_info_client("mx-jungfrau-1", 5232, ib_port_attr.lid, ib_qp->qp_num,
    		remote_dlid, remote_dest_qp_num, sq_psn);

    if (switch_to_rtr(0, remote_dlid, remote_dest_qp_num) == 1) exit(EXIT_FAILURE);
    std::cout << "IB Ready to receive" << std::endl;

    if (switch_to_rts(sq_psn) == 1) exit(EXIT_FAILURE);
    std::cout << "IB Ready to send" << std::endl;

    // Check for threads completion
//    ret = pthread_join(snapThread1, NULL);
//    PTHREAD_ERROR(ret,pthread_join);

//    for	(int i = 0; i <	compression_threads ; i++) {
//        ret = pthread_join(compressionThread[i], NULL);
//        PTHREAD_ERROR(ret,pthread_join);
//    }

    // Send pedestal, header data and collection statistics

    // Save pedestal

    // Print statistics
    std::cout << "Data received: " << ((double) online_statistics->good_packets) / (128.0 * NMODULES * nframes_to_collect) << "%" << std::endl;

    // Deregister memory region
    ibv_dereg_mr(ib_incoming_buffer_mr);
    ibv_dereg_mr(ib_status_buffer_mr);

    // Close RDMA
    close_ibverbs();

    // Deallocate memory
    deallocate_memory();

    // Quit peacefully
}
