#include <unistd.h>
#include <malloc.h>
#include "JFReceiver.h"

uint32_t lastModuleFrameNumber() {
    uint32_t retVal = 10000000;
    for (int i = 0; i < NMODULES; i++) {
        if (online_statistics->head[i] < retVal) retVal = online_statistics->head[i];
    }
}

void *CompressAndSendThread(void *in_threadarg) {
    ThreadArg *arg = (ThreadArg *) in_threadarg;

    size_t framesStillToWrite = nframes_to_write / compression_threads;
    if (nframes_to_write % compression_threads > arg->ThreadID) framesStillToWrite++;
 
    size_t nextFrameIndex = arg->ThreadID;

    const size_t compressionBufferSize = LZ4_compressBound(NPIXEL * 2);
    char *compressionBuffer = (char *) malloc(compressionBufferSize);

    // TODO: Wait for trigger received by the detector
    
    // There are still more frames to compress
    while (framesStillToWrite > 0) {
        // Receiver hasn't yet got this frame

        // TODO: usleep can be smart to know how many frames are missing, so to wait good time
        while (lastModuleFrameNumber() < nextFrameIndex) usleep(10000);
        size_t compressedSize = LZ4_compress_default((char *)(frame_buffer + nextFrameIndex % FRAME_BUF_SIZE * NPIXEL * 2), compressionBuffer, NPIXEL * 2, compressionBufferSize) ;
        
        // TODO: Send the frame via RDMA
        
        nextFrameIndex += compression_threads;
        framesStillToWrite--;
    }
    free(compressionBuffer);

    pthread_exit(0);
}
