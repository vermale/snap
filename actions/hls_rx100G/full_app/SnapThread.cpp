#include <iostream>

#include <snap_tools.h>
#include <libsnap.h>
#include <snap_hls_if.h>

#include "JFReceiver.h"

struct snap_card *card = NULL;
struct snap_action *action = NULL;

int setup_snap(uint32_t card_number) {
	char device[128];

	// default is interrupt mode enabled (vs polling)
	snap_action_flag_t action_irq = (snap_action_flag_t) (SNAP_ACTION_DONE_IRQ | SNAP_ATTACH_IRQ);

	// Allocate the card that will be used
	snprintf(device, sizeof(device)-1, "/dev/cxl/afu%d.0s", card_number);
	card = snap_card_alloc_dev(device, SNAP_VENDOR_ID_IBM,
			SNAP_DEVICE_ID_SNAP);
	if (card == NULL) {
		std::cerr << "Failed to open card #" << card_number << " " << strerror(errno) << std::endl;
		return 1;
	}

	// Attach the action that will be used on the allocated card
	action = snap_attach_action(card, RX100G_ACTION_TYPE, action_irq, 60);
	if (action == NULL) {
		std::cerr << "Failed to attacj action for card #" << card_number << " " << strerror(errno) << std::endl;
		snap_card_free(card);
		return 1;
	}
	return 0;
}

void close_snap() {
	// Detach action + deallocate the card
	snap_detach_action(action);
	snap_card_free(card);
}

void *snap_thread(void *in_threadarg) {
	int rc = 0;

	// Control register
	struct snap_job cjob;
	struct rx100G_job mjob;

	mjob.expected_frames = experiment_settings.nframes_to_collect;
	mjob.pedestalG0_frames = experiment_settings.pedestalG0_frames;
	mjob.mode = experiment_settings.conversion_mode;
	mjob.fpga_mac_addr = receiver_settings.fpga_mac_addr;   // AA:BB:CC:DD:EE:F1
	mjob.fpga_ipv4_addr = receiver_settings.fpga_ip_addr;      // 10.1.50.5

	mjob.in_gain_pedestal_data_addr = (uint64_t) gain_pedestal_data;
	mjob.out_frame_buffer_addr      = (uint64_t) frame_buffer;
	mjob.out_frame_status_addr      = (uint64_t) online_statistics;
	mjob.out_jf_packet_headers_addr = (uint64_t) jf_packet_headers;

	// Fill the stucture of data exchanged with the action
	snap_job_set(&cjob, &mjob, sizeof(mjob), NULL, 0);

	std::cout << "SNAP Thread: Receiving can start" << std::endl;

	// Call the action will:
	//    write all the registers to the action (MMIO) 
	//  + start the action 
	//  + wait for completion
	//  + read all the registers from the action (MMIO) 
	rc = snap_action_sync_execute_job(action, &cjob, TIMEOUT);

	if (rc) std::cerr << "Action failed" << std::endl;

	// Reset Ethernet CMAC
	std::cout << "Resetting 100G CMAC" << std::endl;
	mjob.mode = MODE_RESET;
	snap_job_set(&cjob, &mjob, sizeof(mjob), NULL, 0);

	rc = snap_action_sync_execute_job(action, &cjob, TIMEOUT);

	pthread_exit(0);
}
