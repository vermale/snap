#include <unistd.h>
#include <malloc.h>
#include <iostream>
#include <arpa/inet.h>

#include "JFReceiver.h"
#include "IB_Transport.h"

// Taken from bshuf
/* Write a 64 bit unsigned integer to a buffer in big endian order. */
void bshuf_write_uint64_BE(void* buf, uint64_t num) {
    int ii;
    uint8_t* b = (uint8_t*) buf;
    uint64_t pow28 = 1 << 8;
    for (ii = 7; ii >= 0; ii--) {
        b[ii] = num % pow28;
        num = num / pow28;
    }
}
/* Write a 32 bit unsigned integer to a buffer in big endian order. */
void bshuf_write_uint32_BE(void* buf, uint32_t num) {
    int ii;
    uint8_t* b = (uint8_t*) buf;
    uint32_t pow28 = 1 << 8;
    for (ii = 3; ii >= 0; ii--) {
        b[ii] = num % pow28;
        num = num / pow28;
    }
}

uint32_t lastModuleFrameNumber() {
    uint32_t retVal = online_statistics->head[0];
    for (int i = 1; i < NMODULES; i++) {
        if (online_statistics->head[i] < retVal) retVal = online_statistics->head[i];
    }
    return retVal;
}

void *CompressAndSendThread(void *in_threadarg) {
    ThreadArg *arg = (ThreadArg *) in_threadarg;

    // There are still more frames to compress

    std::cout << "Starting thread #" << arg->ThreadID << std::endl;

    if (arg->ThreadID == 0) {
        while (online_statistics->trigger_position < pedestalG0) usleep(1000);
        trigger_frame = online_statistics->trigger_position;
        pthread_mutex_unlock(&trigger_frame_mutex);
        std::cout << "Trigger observed at" << trigger_frame << std::endl;
    } else {
	pthread_mutex_lock(&trigger_frame_mutex);
        // Check if trigger_frame was set by thread ID = 0
	pthread_mutex_unlock(&trigger_frame_mutex);
    }

    size_t buffer_id;

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
        while (lastModuleFrameNumber() < trigger_frame + frame + RECEIVING_DELAY) usleep(1000);

        uint64_t uncompressed_size = NPIXEL * 2;
        uint32_t block_size = NPIXEL * 2;

        // Compress frame with LZ4 and save to outgoing buffer
        size_t compressed_size = LZ4_compress_default((char *)(frame_buffer + ((trigger_frame + frame) % FRAME_BUF_SIZE) * NPIXEL * 2),
        		ib_outgoing_buffer + RDMA_BUFFER_MAX_ELEM_SIZE * buffer_id + 20,
        		NPIXEL * 2, RDMA_BUFFER_MAX_ELEM_SIZE) ;
        
        // Save metadata - frame number
        memcpy(ib_outgoing_buffer + RDMA_BUFFER_MAX_ELEM_SIZE * buffer_id, &frame, 4);

        // LZ4 plugin header
        bshuf_write_uint64_BE(ib_outgoing_buffer + RDMA_BUFFER_MAX_ELEM_SIZE * buffer_id + 4, uncompressed_size);
        bshuf_write_uint32_BE(ib_outgoing_buffer + RDMA_BUFFER_MAX_ELEM_SIZE * buffer_id + 12, block_size);
        bshuf_write_uint32_BE(ib_outgoing_buffer + RDMA_BUFFER_MAX_ELEM_SIZE * buffer_id + 16, compressed_size);

        // Send the frame via RDMA
        ibv_sge ib_sg;
        ibv_send_wr ib_wr;
        ibv_send_wr *ib_bad_wr;

        memset(&ib_sg, 0, sizeof(ib_sg));
        ib_sg.addr	  = (uintptr_t)(ib_outgoing_buffer + RDMA_BUFFER_MAX_ELEM_SIZE * buffer_id);
        ib_sg.length  = compressed_size + 20;
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
    std::cout << "Exiting thread" << std::endl;
}
