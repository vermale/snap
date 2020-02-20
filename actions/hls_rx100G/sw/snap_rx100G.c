/*
 * Copyright 2017 International Business Machines
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <getopt.h>
#include <malloc.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <assert.h>

#include <snap_tools.h>
#include <libsnap.h>
#include <action_rx100G.h>
#include <snap_hls_if.h>

#define NFRAMES 1
#define NPACKETS

/* main program of the application for the hls_helloworld example        */
/* This application will always be run on CPU and will call either       */
/* a software action (CPU executed) or a hardware action (FPGA executed) */
int main()
{
	int rc = 0; // return code

	// Card parameters
	int card_no = 0;
	struct snap_card *card = NULL;
	struct snap_action *action = NULL;
	char device[128];

	// Control register
	struct snap_job cjob;
	struct rx100G_job mjob;

	unsigned long timeout = 600;
	struct timeval etime, stime;

	fprintf(stderr, "  prepare rx100G job of %ld bytes size\n", sizeof(mjob));

	assert(sizeof(mjob) <= SNAP_JOBSIZE);
	memset(&mjob, 0, sizeof(mjob));

    uint64_t out_data_buffer_size = FRAME_BUF_SIZE * NPIXEL * 2; // can store FRAME_BUF_SIZE frames
    uint64_t out_status_buffer_size = (FRAME_STATUS_BUF_SIZE+1)*64;           // can store FRAME_STATUS_BUF_SIZE frames
    uint64_t in_parameters_array_size = (6 * NPIXEL * 2); // each entry to in_parameters_array is 2 bytes and there are 6 constants per pixel

    // Arrays are allocated with mmap for the higest possible performance. Output is page aligned, so it will be also 64b aligned.

    void *out_data_buffer  = mmap (NULL, out_data_buffer_size, PROT_READ | PROT_WRITE, MAP_PRIVATE|MAP_ANONYMOUS| MAP_POPULATE, -1, 0) ;
    if (out_data_buffer == NULL) goto out_error;

    void *out_status_buffer = mmap (NULL, out_status_buffer_size, PROT_READ | PROT_WRITE, MAP_PRIVATE|MAP_ANONYMOUS| MAP_POPULATE, -1, 0);
    if (out_status_buffer == NULL) goto out_error;

    void *in_parameters_array = mmap (NULL, in_parameters_array_size, PROT_READ | PROT_WRITE, MAP_PRIVATE|MAP_ANONYMOUS| MAP_POPULATE, -1, 0);
    if (in_parameters_array == NULL) goto out_error;

	memset(out_data_buffer, 0x0, out_data_buffer_size);
	memset(out_status_buffer, 0x0, out_status_buffer_size);
	memset(in_parameters_array, 0x0, in_parameters_array_size);

    mjob.packets_to_read = NFRAMES*NMODULES*128;
    mjob.fpga_mac_addr = 0xAABBCCDDEEF1;   // AA:BB:CC:DD:EE:F1
    mjob.fpga_ipv4_addr = 0x0A013205;      // 10.1.50.5

    // Setting input parameters:
    snap_addr_set(&mjob.in_gain_pedestal_data, in_parameters_array, in_parameters_array_size, SNAP_ADDRTYPE_HOST_DRAM,
    		SNAP_ADDRFLAG_ADDR | SNAP_ADDRFLAG_SRC);

    // Setting output parameters:
    snap_addr_set(&mjob.out_frame_buffer, out_data_buffer, out_data_buffer_size, SNAP_ADDRTYPE_HOST_DRAM,
    		SNAP_ADDRFLAG_ADDR | SNAP_ADDRFLAG_DST);

    snap_addr_set(&mjob.out_frame_status, out_status_buffer, out_status_buffer_size, SNAP_ADDRTYPE_HOST_DRAM,
    		SNAP_ADDRFLAG_ADDR | SNAP_ADDRFLAG_DST | SNAP_ADDRFLAG_END);

	int exit_code = EXIT_SUCCESS;

	// default is interrupt mode enabled (vs polling)
	snap_action_flag_t action_irq = (SNAP_ACTION_DONE_IRQ | SNAP_ATTACH_IRQ);

	// Allocate the card that will be used
	snprintf(device, sizeof(device)-1, "/dev/cxl/afu%d.0s", card_no);
	card = snap_card_alloc_dev(device, SNAP_VENDOR_ID_IBM,
				   SNAP_DEVICE_ID_SNAP);
	if (card == NULL) {
		fprintf(stderr, "err: failed to open card %u: %s\n",
			card_no, strerror(errno));
		goto out_error;
	}

	// Attach the action that will be used on the allocated card
	action = snap_attach_action(card, RX100G_ACTION_TYPE, action_irq, 60);
	if (action == NULL) {
		fprintf(stderr, "err: failed to attach action %u: %s\n",
			card_no, strerror(errno));
		goto out_error1;
	}

	// Fill the stucture of data exchanged with the action
	snap_job_set(&cjob, &mjob, sizeof(mjob), NULL, 0);

	// uncomment to dump the job structure
	__hexdump(stderr, &mjob, sizeof(mjob));


	// Collect the timestamp BEFORE the call of the action
	gettimeofday(&stime, NULL);

	// Call the action will:
	//    write all the registers to the action (MMIO) 
	//  + start the action 
	//  + wait for completion
	//  + read all the registers from the action (MMIO) 
	rc = snap_action_sync_execute_job(action, &cjob, timeout);

	// Collect the timestamp AFTER the call of the action
	gettimeofday(&etime, NULL);
	if (rc != 0) {
		fprintf(stderr, "err: job execution %d: %s!\n", rc,
			strerror(errno));
		goto out_error2;
	}
    __hexdump(stdout, out_data_buffer, 130*64);
    __hexdump(stdout, out_status_buffer, 8192);

	printf(" Good packets %ld\n", mjob.good_packets);
	printf(" Bad packets %ld\n", mjob.bad_packets);

	// test return code
	(cjob.retc == SNAP_RETC_SUCCESS) ? fprintf(stdout, "SUCCESS\n") : fprintf(stdout, "FAILED\n");
	if (cjob.retc != SNAP_RETC_SUCCESS) {
		fprintf(stderr, "err: Unexpected RETC=%x!\n", cjob.retc);
		goto out_error2;
	}

	// Display the time of the action call (MMIO registers filled + execution)
	fprintf(stdout, "RX100G took %lld usec\n",
		(long long)timediff_usec(&etime, &stime));

	// Detach action + disallocate the card
	snap_detach_action(action);
	snap_card_free(card);

	// Memory deallocation
	munmap(out_data_buffer, out_data_buffer_size);
	munmap(out_status_buffer, out_status_buffer_size);
	munmap(in_parameters_array, in_parameters_array_size);
	exit(exit_code);

 out_error2:
	snap_detach_action(action);
 out_error1:
	snap_card_free(card);
 out_error:
    printf("Error\n");
	exit(EXIT_FAILURE);
}
