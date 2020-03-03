#include <iostream>

#include <snap_tools.h>
#include <libsnap.h>
#include <snap_hls_if.h>

#include "JFReceiver.h"

void *SnapThread(void *in_threadarg) {

	int rc = 0;
	
	struct snap_card *card = NULL;
	struct snap_action *action = NULL;
	char device[128];
	
	// Control register
	struct snap_job cjob;
	struct rx100G_job mjob;

	// Time out
	// TODO: Check if this will cause problems later
	unsigned long timeout = TIMEOUT;

	mjob.expected_frames = nframes_to_collect;
	mjob.pedestalG0_frames = pedestalG0;
	mjob.mode = conversion_mode;
	mjob.fpga_mac_addr = fpga_mac_addr;   // AA:BB:CC:DD:EE:F1
	mjob.fpga_ipv4_addr = fpga_ip_addr;      // 10.1.50.5

	mjob.in_gain_pedestal_data_addr = (uint64_t) gain_pedestal_data;
	mjob.out_frame_buffer_addr = (uint64_t) frame_buffer;
	mjob.out_frame_status_addr = (uint64_t) online_statistics;
	mjob.out_jf_packet_headers_addr = (uint64_t) jf_packet_headers;

	// default is interrupt mode enabled (vs polling)
	snap_action_flag_t action_irq = (snap_action_flag_t) (SNAP_ACTION_DONE_IRQ | SNAP_ATTACH_IRQ);

	// Allocate the card that will be used
	snprintf(device, sizeof(device)-1, "/dev/cxl/afu%d.0s", card_no);
	card = snap_card_alloc_dev(device, SNAP_VENDOR_ID_IBM,
				   SNAP_DEVICE_ID_SNAP);
	if (card == NULL) {
		fprintf(stderr, "err: failed to open card %u: %s\n",
			card_no, strerror(errno));
		pthread_exit(0);
	}

	// Attach the action that will be used on the allocated card
	action = snap_attach_action(card, RX100G_ACTION_TYPE, action_irq, 60);
	if (action == NULL) {
		fprintf(stderr, "err: failed to attach action %u: %s\n",
			card_no, strerror(errno));
		snap_card_free(card);
		pthread_exit(0);
	}

	// Fill the stucture of data exchanged with the action
	snap_job_set(&cjob, &mjob, sizeof(mjob), NULL, 0);

	// Call the action will:
	//    write all the registers to the action (MMIO) 
	//  + start the action 
	//  + wait for completion
	//  + read all the registers from the action (MMIO) 
	rc = snap_action_sync_execute_job(action, &cjob, timeout);
	
	// Detach action + disallocate the card
	snap_detach_action(action);
	snap_card_free(card);

	pthread_exit(0);
}
