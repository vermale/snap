#include <fstream>
#include <iostream>
#include <infiniband/verbs.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <unistd.h>

#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>

#include "JFReceiver.h"
#include "IB_Transport.h"

int parse_input(int argc, char **argv) {
	int opt;
	receiver_settings.card_number = 1;
	receiver_settings.compression_threads = 2;
	receiver_settings.ib_dev_name = "mlx5_0";
	receiver_settings.fpga_mac_addr = 0xAABBCCDDEEF1;
	receiver_settings.fpga_ip_addr = 0x0A013205;
	receiver_settings.tcp_port = 52320;
	receiver_settings.pedestal_file_name = "pedestal.dat";

	receiver_settings.gain_file_name[0] =
			"/home/jungfrau/JF2M_X06SA_200310/gainMaps_M351_2020-01-20.bin";
	receiver_settings.gain_file_name[1] =
			"/home/jungfrau/JF2M_X06SA_200310/gainMaps_M312_2020-01-20.bin";
	receiver_settings.gain_file_name[2] =
			"/home/jungfrau/JF2M_X06SA_200310/gainMaps_M261_2019-07-29.bin";
	receiver_settings.gain_file_name[3] =
			"/home/jungfrau/JF2M_X06SA_200310/gainMaps_M373_2020-01-31.bin";

	while ((opt = getopt(argc,argv,":C:T:I:P:p:0:1:2:3:")) != EOF)
		switch(opt)
		{
		case 'C':
			receiver_settings.card_number = atoi(optarg);
			break;
		case 'T':
			receiver_settings.compression_threads = atoi(optarg);
			break;
		case 'I':
			receiver_settings.ib_dev_name = std::string(optarg);
			break;
		case 'P':
			receiver_settings.tcp_port = atoi(optarg);
			break;
		case 'p':
			receiver_settings.pedestal_file_name = std::string(optarg);
			break;
		case 0:
			receiver_settings.gain_file_name[0] = std::string(optarg);
			break;
		case 1:
			receiver_settings.gain_file_name[1] = std::string(optarg);
			break;
		case 2:
			receiver_settings.gain_file_name[2] = std::string(optarg);
			break;
		case 3:
			receiver_settings.gain_file_name[3] = std::string(optarg);
			break;
		}
	return 0;
}

int allocate_memory() {
	frame_buffer_size       = FRAME_BUF_SIZE * NPIXEL * 2; // can store FRAME_BUF_SIZE frames
	status_buffer_size      = FRAME_LIMIT*NMODULES*128/8+64;   // can store 1 bit per each ETH packet expected
	gain_pedestal_data_size = 6 * 2 * NPIXEL;  // each entry to in_parameters_array is 2 bytes and there are 6 constants per pixel
	jf_packet_headers_size  = FRAME_LIMIT * NMODULES * sizeof(header_info_t);
	ib_buffer_size          = ((size_t) RDMA_BUFFER_MAX_ELEM_SIZE) * RDMA_SQ_SIZE;

	// Arrays are allocated with mmap for the higest possible performance. Output is page aligned, so it will be also 64b aligned.
	frame_buffer       = (int16_t *)  mmap (NULL, frame_buffer_size, PROT_READ | PROT_WRITE, MAP_PRIVATE|MAP_ANONYMOUS, -1, 0) ;
	status_buffer      = (char *) mmap (NULL, status_buffer_size, PROT_READ | PROT_WRITE, MAP_PRIVATE|MAP_ANONYMOUS, -1, 0);
	gain_pedestal_data = (uint16_t *) mmap (NULL, gain_pedestal_data_size, PROT_READ | PROT_WRITE, MAP_PRIVATE|MAP_ANONYMOUS, -1, 0);
	jf_packet_headers  = (header_info_t *) mmap (NULL, jf_packet_headers_size, PROT_READ | PROT_WRITE, MAP_PRIVATE|MAP_ANONYMOUS, -1, 0);
	ib_buffer          = (char *) mmap (NULL, ib_buffer_size, PROT_READ | PROT_WRITE, MAP_PRIVATE|MAP_ANONYMOUS, -1, 0);

	if ((frame_buffer == NULL) || (status_buffer == NULL) ||
			(gain_pedestal_data == NULL) || (jf_packet_headers == NULL) ||
			(ib_buffer == NULL)) {
		std::cerr << "Memory allocation error" << std::endl;
		return 1;
	}

	// Fill output arrays with zeros
	memset(frame_buffer, 0x0, frame_buffer_size);
	memset(status_buffer, 0x0, status_buffer_size);
	memset(gain_pedestal_data, 0x0, gain_pedestal_data_size);
	memset(jf_packet_headers, 0x0, jf_packet_headers_size);
	memset(ib_buffer, 0x0, ib_buffer_size);

	packet_counter = (char *) (status_buffer + 64);
	online_statistics = (online_statistics_t *) status_buffer;

	return 0;
}

void deallocate_memory() {
	munmap(frame_buffer, frame_buffer_size);
	munmap(status_buffer, status_buffer_size);
	munmap(gain_pedestal_data, gain_pedestal_data_size);
	munmap(jf_packet_headers, jf_packet_headers_size);
	munmap(ib_buffer, ib_buffer_size);
}

int load_bin_file(std::string fname, char *dest, size_t size) {
	std::cout << "Loading " << fname.c_str() << std::endl;
	std::fstream file10(fname.c_str(), std::fstream::in | std::fstream::binary);
	if (!file10.is_open()) {
		std::cerr << "Error opening file " << fname.c_str() << std::endl;
		return 1;
	} else {
		file10.read(dest, size);
		file10.close();
		return 0;
	}
}

void load_gain(std::string fname, int module, double energy_in_keV) {
	double *tmp_gain = (double *) calloc (3 * MODULE_COLS * MODULE_LINES, sizeof(double));
	load_bin_file(fname, (char *) tmp_gain, 3 * MODULE_COLS * MODULE_LINES * sizeof(double));

	size_t offset = module * MODULE_COLS * MODULE_LINES;

	// 14-bit fractional part
	for (int i = 0; i < MODULE_COLS * MODULE_LINES; i ++)
		gain_pedestal_data[offset+i] =  (uint16_t) ((512.0 / (tmp_gain[i] * energy_in_keV)) * 16384 + 0.5);

	// 13-bit fractional part
	offset += NPIXEL;
	for (int i = 0; i < MODULE_COLS * MODULE_LINES; i ++)
		gain_pedestal_data[offset+i] =  (uint16_t) (-1.0 / ( tmp_gain[i + MODULE_COLS * MODULE_LINES] * energy_in_keV) * 8192 + 0.5);

	// 13-bit fractional part
	offset += NPIXEL;
	for (int i = 0; i < MODULE_COLS * MODULE_LINES; i ++)
		gain_pedestal_data[offset+i] =  (uint16_t) (-1.0 / ( tmp_gain[i + 2 * MODULE_COLS * MODULE_LINES] * energy_in_keV) * 8192 + 0.5);

	free(tmp_gain);
}

void load_pedestal(std::string fname) {
	load_bin_file(fname, (char *)(gain_pedestal_data + 3 * NPIXEL), 3 * NPIXEL * sizeof(uint16_t));
}

void save_pedestal(std::string fname) {
	std::cout << "Saving " << fname.c_str() << std::endl;
	std::fstream file10(fname.c_str(), std::ios::out | std::ios::binary);
	if (!file10.is_open()) {
		std::cerr << "Error opening file " << fname.c_str() << std::endl;
	} else {
		file10.write((char *) (gain_pedestal_data + 3 * NPIXEL), 3 * NPIXEL * sizeof(uint16_t));
		file10.close();
	}
}

int TCP_server(uint16_t port) {
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd == 0) {
		std::cerr << "TCP/IP socket error" << std::endl;
		exit(EXIT_FAILURE);
		return 1;
	}

	struct sockaddr_in srv_address;
	srv_address.sin_family = AF_INET;
	srv_address.sin_addr.s_addr = INADDR_ANY;
	srv_address.sin_port = htons( port );

	if (bind(sockfd, (struct sockaddr *)&srv_address, sizeof(srv_address)) < 0) {
		std::cerr << "TCP/IP bind error" << std::endl;
		return 1;
	}

	listen(sockfd, 1);
	return 0;
}

int TCP_accept_connection() {
	struct sockaddr_in client_address;
	// Accept TCP/IP connection
	socklen_t addrlen = sizeof(client_address);
	accepted_socket = accept(sockfd, (struct sockaddr *)&client_address, &addrlen);
	uint64_t magic_number = TCPIP_CONN_MAGIC_NUMBER;

	// Send parameters
	send(accepted_socket, &magic_number, sizeof(uint64_t), 0);
	// Receive parameters
	read(accepted_socket, &magic_number, sizeof(uint64_t));

	if (magic_number != TCPIP_CONN_MAGIC_NUMBER) return 1;
	else return 0;
}

void TCP_close_connection() {
	close(sockfd);
}

void TCP_exchange_IB_parameters(ib_comm_settings_t *remote) {
	ib_comm_settings_t local;
	local.qp_num = ib_settings.qp->qp_num;
	local.dlid = ib_settings.port_attr.lid;
	local.rq_psn = RDMA_SQ_PSN;

	// Send parameters
	send(accepted_socket, &local, sizeof(ib_comm_settings_t), 0);

	// Receive parameters
	read(accepted_socket, remote, sizeof(ib_comm_settings_t));
}
/*
int save_images() {
	if (conversion_mode == MODE_CONV) {
		std::ofstream data_file("output_data.dat",std::ios::out | std::ios::binary);
		data_file.write((char *) (frame_buffer + NPIXEL * online_statistics->trigger_position), nframes_to_write * NPIXEL * 2);
		data_file.close();

		int32_t *frame_sum = (int32_t *) calloc (NPIXEL, sizeof(int32_t));
		for (uint64_t j = 0; j < nframes_to_write; j++) {
			for (int i = 0; i < NPIXEL; i++) {
				int16_t tmp = frame_buffer[NPIXEL * (online_statistics->trigger_position + (uint64_t)j) + (uint64_t)i];
				if ((tmp < 30000) && (tmp > -30000)) frame_sum[i] += tmp;
			}
		}
		std::ofstream sum_file("sum_data.dat",std::ios::out | std::ios::binary);
		sum_file.write((char *) (frame_sum), NPIXEL * sizeof(int32_t));
		sum_file.close();
		free (frame_sum);
	} else {
		std::ofstream data_file("output_data.dat",std::ios::out | std::ios::binary);
		data_file.write((char *) (frame_buffer), nframes_to_collect * NPIXEL * 2);
		data_file.close();
	}

	std::ofstream status_file("status_data.dat",std::ios::out | std::ios::binary);
	status_file.write((char *) status_buffer, status_buffer_size);
	status_file.close();
} */

int main(int argc, char **argv) {
	int ret;

	std::cout << "JF Receiver " << std::endl;

	// Parse input parameters
	if (parse_input(argc, argv) == 1) exit(EXIT_FAILURE);

	// Allocate memory
	if (allocate_memory() == 1) exit(EXIT_FAILURE);
	std::cout << "Memory allocated" << std::endl;

	// Load gain files (double, per module)
	for (int i = 0; i < NMODULES; i++)
		load_gain(receiver_settings.gain_file_name[i], i, experiment_settings.energy_in_keV);

	// Load pedestal file
	load_pedestal(receiver_settings.pedestal_file_name);

	// Establish RDMA link
	if (setup_ibverbs(ib_settings, receiver_settings.ib_dev_name.c_str(), RDMA_SQ_SIZE, 0) == 1) exit(EXIT_FAILURE);
	std::cout << "IB link ready" << std::endl;

	// Register memory regions
	ib_settings.buffer_mr = ibv_reg_mr(ib_settings.pd, ib_buffer, ib_buffer_size, 0);
	if (ib_settings.buffer_mr == NULL) {
		std::cerr << "Failed to register IB memory region." << std::endl;
		return 1;
	}

	// Connect to FPGA board
	setup_snap(receiver_settings.card_number);

	// Establish TCP/IP server
	if (TCP_server(receiver_settings.tcp_port) == 1) exit(EXIT_FAILURE);

	// Accept TCP/IP communication
	while (TCP_accept_connection() != 0)
		std::cout << "Bogus TCP/IP connection" << std::endl;

	// Exchange IB information
	ib_comm_settings_t remote;
	TCP_exchange_IB_parameters(&remote);

	// Switch to ready to send state for IB
	if (switch_to_rtr(ib_settings, 0, remote.dlid, remote.qp_num) == 1) exit(EXIT_FAILURE);
	std::cout << "IB Ready to receive" << std::endl;
	if (switch_to_rts(ib_settings, RDMA_SQ_PSN) == 1) exit(EXIT_FAILURE);
	std::cout << "IB Ready to send" << std::endl;

	// Receive experimental settings via TCP/IP
	read(accepted_socket, &experiment_settings, sizeof(experiment_settings_t));

	// Start compression threads
	pthread_t compressionThread[receiver_settings.compression_threads];
	ThreadArg args[receiver_settings.compression_threads];

	for (int i = 0; i < receiver_settings.compression_threads ; i++) {
		args[i].ThreadID = i;
		ret = pthread_create(&compressionThread[i], NULL, send_thread, &args[i]);
		PTHREAD_ERROR(ret,pthread_create);
	}


	pthread_t poll_cq_thread_1;
	ret = pthread_create(&poll_cq_thread_1, NULL, poll_cq_thread, NULL);
	PTHREAD_ERROR(ret, pthread_create);

	// Check for thread completion
	ret = pthread_join(poll_cq_thread_1, NULL);
	PTHREAD_ERROR(ret, pthread_join);

	// Start SNAP thread
	//pthread_t snapThread1;
	//ret = pthread_create(&snapThread1, NULL, snap_thread, NULL);
	//PTHREAD_ERROR(ret,pthread_create);

	// Check for threads completion
	//ret = pthread_join(snapThread1, NULL);
	//PTHREAD_ERROR(ret,pthread_join);

	for	(int i = 0; i <	receiver_settings.compression_threads ; i++) {
		ret = pthread_join(compressionThread[i], NULL);
		PTHREAD_ERROR(ret,pthread_join);
	}

	// Acknowledge frames transferred
	//uint64_t frames_written = experiment_settings.nframes_to_write;
	//end(accepted_socket, &frames_written, sizeof(uint64_t), 0);

	// Send pedestal, header data and collection statistics
	send(accepted_socket, online_statistics, sizeof(online_statistics_t), 0);
	send(accepted_socket, &gain_pedestal_data, gain_pedestal_data_size, 0);

	// Save pedestal
	save_pedestal(receiver_settings.pedestal_file_name);

	// Disconnect client
	close(accepted_socket);

	// Deregister memory region
	ibv_dereg_mr(ib_settings.buffer_mr);

	// Close RDMA
	close_ibverbs(ib_settings);

	// Close SNAP
	close_snap();

	// Close TCP/IP socket
	TCP_close_connection();

	// Deallocate memory
	deallocate_memory();

	// Quit peacefully
	std::cout << "Done" << std::endl;
}
