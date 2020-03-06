#include <unistd.h>
#include <malloc.h>
#include <iostream>
#include "JFReceiver.h"
#include "IB_Transport.h"

uint32_t lastModuleFrameNumber() {
    uint32_t retVal = online_statistics->head[0];
    for (int i = 1; i < NMODULES; i++) {
        if (online_statistics->head[i] < retVal) retVal = online_statistics->head[i];
    }
    return retVal;
}

void *CompressAndSendThread(void *in_threadarg) {
    ThreadArg *arg = (ThreadArg *) in_threadarg;

    // TODO: Wait for trigger received by the detector
    
    size_t buffer_id;
    // There are still more frames to compress
    for (size_t frame = arg->ThreadID; frame < nframes_to_write; frame += compression_threads) {
    	if (frame < RDMA_SQ_SIZE) buffer_id = frame;
    	else {
    		// Poll CQ to reuse ID
    		ibv_wc ib_wc;
    		int num_comp = 0;
    		while (num_comp == 0) {
    			num_comp = ibv_poll_cq(ib_cq, 1, &ib_wc);

    			if (num_comp < 0) {
    				std::cerr << "Failed polling IB Verbs completion queue" << std::endl;
    				pthread_exit(0);
    			}

    			if (ib_wc.status != IBV_WC_SUCCESS) {
    				std::cerr << "Failed status " << ibv_wc_status_str(ib_wc.status) << " of IB Verbs send request #" << (int)ib_wc.wr_id << std::endl;
    				pthread_exit(0);
    			}
    			usleep(1000);
    		}
    		buffer_id = ib_wc.wr_id;
    	}

        // TODO: usleep can be smart to know how many frames are missing, so to wait good time
    	// Wait till receiving is 5 frames after receiver
        while (lastModuleFrameNumber() < frame + RECEIVING_DELAY) usleep(10000);

        // Compress frame with LZ4 and save to outgoing buffer
        size_t compressedSize = LZ4_compress_default((char *)(frame_buffer + (frame % FRAME_BUF_SIZE) * NPIXEL * 2),
        		ib_outgoing_buffer + RDMA_BUFFER_MAX_ELEM_SIZE * buffer_id + 8,
        		NPIXEL * 2, RDMA_BUFFER_MAX_ELEM_SIZE) ;
        
        // Save metadata - frame number and compressed size
        memcpy(ib_outgoing_buffer + RDMA_BUFFER_MAX_ELEM_SIZE * buffer_id, &frame, 4);
        memcpy(ib_outgoing_buffer + RDMA_BUFFER_MAX_ELEM_SIZE * buffer_id + 4, &compressedSize, 4);
        
        // Send the frame via RDMA
        ibv_sge ib_sg;
        ibv_send_wr ib_wr;
        ibv_send_wr *ib_bad_wr;

        memset(&ib_sg, 0, sizeof(ib_sg));
        ib_sg.addr	  = (uintptr_t)(ib_outgoing_buffer + RDMA_BUFFER_MAX_ELEM_SIZE * buffer_id);
        ib_sg.length  = compressedSize + 8;
        ib_sg.lkey	  = ib_outgoing_buffer_mr->lkey;

        memset(&ib_wr, 0, sizeof(ib_wr));
        ib_wr.wr_id      = buffer_id;
        ib_wr.sg_list    = &ib_sg;
        ib_wr.num_sge    = 1;
        ib_wr.opcode     = IBV_WR_SEND;
        ib_wr.send_flags = IBV_SEND_SIGNALED;

        if (ibv_post_send(ib_qp, &ib_wr, &ib_bad_wr)) {
        	std::cerr << "Sending with IB Verbs failed" << std::endl;
        	pthread_exit(0);
        }
    }

    pthread_exit(0);
}
