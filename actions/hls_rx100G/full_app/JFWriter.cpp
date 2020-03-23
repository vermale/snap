#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <unistd.h>
#include <sys/mman.h>
#include <algorithm>
#include <hdf5.h>

#include <endian.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>

#include "JFWriter.h"

extern "C" {
extern H5Z_class_t H5Z_JF[1];
}

writer_settings_t writer_settings;
gain_pedestal_t gain_pedestal[NCARDS];
online_statistics_t online_statistics[NCARDS];

experiment_settings_t experiment_settings;
writer_connection_settings_t writer_connection_settings[NCARDS];

uint8_t writers_done_per_file;
pthread_mutex_t writers_done_per_file_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t writers_done_per_file_cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t hdf5_mutex = PTHREAD_MUTEX_INITIALIZER;

int addStringAttribute(hid_t location, std::string name, std::string val) {
	/* https://support.hdfgroup.org/ftp/HDF5/current/src/unpacked/examples/h5_attribute.c */
	hid_t aid = H5Screate(H5S_SCALAR);
	hid_t atype = H5Tcopy(H5T_C_S1);
	H5Tset_size(atype, val.length());
	H5Tset_strpad(atype,H5T_STR_NULLTERM);
	hid_t attr = H5Acreate2(location, name.c_str(), atype, aid, H5P_DEFAULT, H5P_DEFAULT);
	herr_t ret = H5Awrite(attr, atype, val.c_str());
	ret = H5Sclose(aid);
	ret = H5Tclose(atype);
	ret = H5Aclose(attr);

	return 0;

}

hid_t createGroup(hid_t master_file_id, std::string group, std::string nxattr) {
	hid_t group_id = H5Gcreate(master_file_id, group.c_str(), H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
	if (nxattr != "") addStringAttribute(group_id, "NX_class", nxattr);
	return group_id;
}

int open_HDF5_file(uint32_t file_id) {

	int32_t frames = experiment_settings.nframes_to_write - writer_settings.images_per_file * file_id;
	if (frames >  writer_settings.images_per_file) frames =  writer_settings.images_per_file;
	if (frames <= 0) return 1;

	// generate filename for data file
	char buff[12];
	snprintf(buff,12,"data_%06d", file_id+1);
	std::string filename = writer_settings.HDF5_prefix+"_"+std::string(buff)+".h5";

	// Create data file
	data_file = H5Fcreate(filename.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
	if (data_file < 0) {
		std::cerr << "HDF5 data file error" << std::endl;
	}

	hid_t grp = createGroup(data_file, "/entry", "NXentry");
	H5Gclose(grp);

	data_group = createGroup(data_file, "/entry/data","NXdata");

	herr_t h5ret;

	// https://support.hdfgroup.org/ftp/HDF5/current/src/unpacked/examples/h5_crtdat.c
	hsize_t dims[3], chunk[3];

	dims[0] = frames;
	dims[1] = 514 * NMODULES * NCARDS;
	dims[2] = 1030;

	chunk[0] = 1;
	chunk[1] = 514 * NMODULES;
	chunk[2] = 1030;

	// Create the data space for the dataset.
	data_dataspace = H5Screate_simple(3, dims, NULL);

	data_dcpl = H5Pcreate (H5P_DATASET_CREATE);
	h5ret = H5Pset_chunk (data_dcpl, 3, chunk);

	// Set appropriate compression filter
	// SetCompressionFilter(data_dcpl_id);

	// Create the dataset.
	data_dataset = H5Dcreate2(data_group, "data", H5T_STD_I16LE, data_dataspace,
			H5P_DEFAULT, data_dcpl, H5P_DEFAULT);

	// Add attributes
	int tmp = file_id *  writer_settings.images_per_file + 1;
	hid_t aid = H5Screate(H5S_SCALAR);
	hid_t attr = H5Acreate2(data_dataset, "image_nr_low", H5T_STD_I32LE, aid, H5P_DEFAULT, H5P_DEFAULT);
	h5ret = H5Awrite(attr, H5T_NATIVE_INT, &tmp);
	h5ret = H5Sclose(aid);
	h5ret = H5Aclose(attr);

	tmp = tmp+frames-1;
	aid = H5Screate(H5S_SCALAR);
	attr = H5Acreate2(data_dataset, "image_nr_high", H5T_STD_I32LE, aid, H5P_DEFAULT, H5P_DEFAULT);
	h5ret = H5Awrite(attr, H5T_NATIVE_INT, &tmp);
	h5ret = H5Sclose(aid);
	h5ret = H5Aclose(attr);
	return 0;
}

void close_HDF5_file() {
	H5Pclose (data_dcpl);

	/* End access to the dataset and release resources used by it. */
	H5Dclose(data_dataset);

	/* Terminate access to the data space. */
	H5Sclose(data_dataspace);

	H5Gclose(data_group);
	H5Fclose(data_file);
}

int exchange_magic_number(int sockfd) {
	uint64_t magic_number;

	// Receive parameters
	read(sockfd, &magic_number, sizeof(uint64_t));
	if (magic_number != TCPIP_CONN_MAGIC_NUMBER) {
		std::cerr << "Mismatch in TCP/IP communication" << std::endl;
		return 1;
	}

	magic_number = TCPIP_CONN_MAGIC_NUMBER;
	// Send parameters
	send(sockfd, &magic_number, sizeof(uint64_t), 0);
	return 0;
}

int TCP_connect(int &sockfd, std::string hostname, uint16_t port) {
	// Use socket to exchange connection information
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd == 0) {
		std::cout << "Socket error" << std::endl;
		return 1;
	}

	addrinfo *host_data;

	char txt_buffer[6];
	snprintf(txt_buffer, 6, "%d", port);

	if (getaddrinfo(hostname.c_str(), txt_buffer,  NULL, &host_data)) {
		std::cout << "Host not found" << std::endl;
		return 1;
	}
	if (host_data == NULL) {
		std::cout << "Host " << hostname << " not found" << std::endl;
		return 1;
	}
	if (connect(sockfd, host_data[0].ai_addr, host_data[0].ai_addrlen) < 0) {
		std::cout << "Cannot connect to server" << std::endl;
		return 1;
	}

	return exchange_magic_number(sockfd);
}

void TCP_exchange_IB_parameters(int sockfd, ib_settings_t &ib_settings, ib_comm_settings_t *remote) {
	ib_comm_settings_t local;
	local.qp_num = ib_settings.qp->qp_num;
	local.dlid = ib_settings.port_attr.lid;

	// Receive parameters
	read(sockfd, remote, sizeof(ib_comm_settings_t));

	// Send parameters
	send(sockfd, &local, sizeof(ib_comm_settings_t), 0);
}

int parse_input(int argc, char **argv) {
	int opt;
	experiment_settings.energy_in_keV      = 12.4;
	experiment_settings.pedestalG0_frames  = 0;
	experiment_settings.conversion_mode    = MODE_CONV;
	experiment_settings.nframes_to_collect = 16384;
	experiment_settings.nframes_to_write   = 0;

	writer_settings.HDF5_prefix = "";
	writer_settings.images_per_file = 1000;

	//These parameters are not changeable at the moment
	writer_connection_settings[0].ib_dev_name = "mlx5_0";
	writer_connection_settings[0].receiver_host = "mx-ic922-1";
	writer_connection_settings[0].receiver_tcp_port = 52320;

	if (NCARDS == 2) {
		writer_connection_settings[1].ib_dev_name = "mlx5_2";
		writer_connection_settings[1].receiver_host = "mx-ic922-1";
		writer_connection_settings[1].receiver_tcp_port = 52321;
	}

	while ((opt = getopt(argc,argv,":E:P:012BRc:w:f:i:h")) != EOF)
		switch(opt)
		{
		case 'E':
			experiment_settings.energy_in_keV = atof(optarg);
			break;
		case 'P':
			experiment_settings.pedestalG0_frames  = atoi(optarg);
			break;
		case '0':
			experiment_settings.conversion_mode = MODE_PEDEG0;
			break;
		case '1':
			experiment_settings.conversion_mode = MODE_PEDEG1;
			break;
		case '2':
			experiment_settings.conversion_mode = MODE_PEDEG2;
			break;
		case 'B':
			experiment_settings.conversion_mode = MODE_CONV_BSHUF;
			break;
		case 'c':
			experiment_settings.nframes_to_collect = atoi(optarg);
			break;
		case 'w':
			experiment_settings.nframes_to_write   = atoi(optarg);
			break;
		case 'f':
			writer_settings.HDF5_prefix = std::string(optarg);
			break;
		case 'i':
			writer_settings.images_per_file = atoi(optarg);
			break;
		case ':':
			break;
		case '?':
			break;
		}
	return 0;
}

void *writer_thread(void* threadArg) {
	int threadID = 0;
	int sockfd;
	ib_settings_t ib_settings;
	if (TCP_connect(sockfd, writer_connection_settings[threadID].receiver_host,
			writer_connection_settings[threadID].receiver_tcp_port) == 1)
		pthread_exit(0);

	// Setup Infiniband connection
	setup_ibverbs(ib_settings,
			writer_connection_settings[threadID].ib_dev_name, 0, RDMA_RQ_SIZE);

	// Exchange information with remote host
	ib_comm_settings_t remote;
	TCP_exchange_IB_parameters(sockfd, ib_settings, &remote);

	// Switch to ready to receive
	switch_to_rtr(ib_settings, remote.rq_psn, remote.dlid, remote.qp_num);

	// IB buffer
	char *ib_buffer = (char *) calloc(RDMA_RQ_SIZE, RDMA_BUFFER_MAX_ELEM_SIZE);

	if (ib_buffer == NULL) {
		std::cerr << "Memory allocation error" << std::endl;
		pthread_exit(0);
	}
	ibv_mr *ib_buffer_mr = ibv_reg_mr(ib_settings.pd,
			ib_buffer, RDMA_RQ_SIZE * RDMA_BUFFER_MAX_ELEM_SIZE, 0);
	if (ib_buffer_mr == NULL) {
		std::cerr << "Failed to register IB memory region." << std::endl;
		pthread_exit(0);
	}

	// Post WRs
	// Start receiving
	struct ibv_sge ib_sg_entry;
	struct ibv_recv_wr ib_wr, *ib_bad_recv_wr;

	/* pointer to packet buffer size and memory key of each packet buffer */
	ib_sg_entry.length = RDMA_BUFFER_MAX_ELEM_SIZE;
	ib_sg_entry.lkey = ib_buffer_mr->lkey;

	ib_wr.num_sge = 1;
	ib_wr.sg_list = &ib_sg_entry;
	ib_wr.next = NULL;

	for (size_t i = 0; (i < RDMA_RQ_SIZE) && (i < experiment_settings.nframes_to_write); i++)
	{
		ib_sg_entry.addr = (uint64_t)ib_buffer + RDMA_BUFFER_MAX_ELEM_SIZE*i;
		ib_wr.wr_id = i;
		ibv_post_recv(ib_settings.qp, &ib_wr, &ib_bad_recv_wr);
	}

	// Provide experiment settings to receiver
	send(sockfd, &experiment_settings, sizeof(experiment_settings_t), 0);

	// Receive data and write to HDF5 file
	for (size_t frame = 0; frame < experiment_settings.nframes_to_write; frame ++) {
		// Poll CQ for finished receive requests
		ibv_wc ib_wc;

		int num_comp;
		do {
			num_comp = ibv_poll_cq(ib_settings.cq, 1, &ib_wc);

			if (num_comp < 0) {
				std::cerr << "Failed polling IB Verbs completion queue" << std::endl;
				exit(EXIT_FAILURE);
			}

			if (ib_wc.status != IBV_WC_SUCCESS) {
				std::cerr << "Failed status " << ibv_wc_status_str(ib_wc.status) << " of IB Verbs send request #" << (int)ib_wc.wr_id << std::endl;
				exit(EXIT_FAILURE);
			}
			usleep(1000);
		} while (num_comp == 0);

		// Output is in ib_wc.wr_id - do something
		// e.g. write with direct chunk writer
		// TODO: Write to HDF5

		pthread_mutex_lock(&hdf5_mutex);

		uint64_t *be_frame_number = (uint64_t *) (ib_buffer + RDMA_BUFFER_MAX_ELEM_SIZE*ib_wc.wr_id);
		uint32_t *be_compressed_size = (uint32_t *) (ib_buffer + RDMA_BUFFER_MAX_ELEM_SIZE*ib_wc.wr_id + 8);
		uint32_t compressed_size = be32toh(*be_compressed_size);

		hsize_t offset[] = {be64toh(*be_frame_number), 514 * NMODULES * threadID,0};

		//herr_t h5ret = H5Dwrite_chunk(data_dataset, H5P_DEFAULT, 0, offset, compressed_size,
		//		ib_buffer + RDMA_BUFFER_MAX_ELEM_SIZE*ib_wc.wr_id + 8);


		pthread_mutex_unlock(&hdf5_mutex);

		// Exchange file to write frames
		if (frame % writer_settings.images_per_file ==
				writer_settings.images_per_file - 1) {
			pthread_mutex_lock(&writers_done_per_file_mutex);
			writers_done_per_file--;
			if (writers_done_per_file == 0) {
				// Last part of the file finished - all the other threads are waiting on conditional below
				close_HDF5_file();
				open_HDF5_file(frame / writer_settings.images_per_file);
				pthread_cond_broadcast(&writers_done_per_file_cond);
				writers_done_per_file = NCARDS;
			} else pthread_cond_wait(&writers_done_per_file_cond, &writers_done_per_file_mutex);
			pthread_mutex_unlock(&writers_done_per_file_mutex);
		}

		// Post new WRs
		if (experiment_settings.nframes_to_write - frame > RDMA_RQ_SIZE) {
			// Make new work request with the same ID
			// If there is need of new work request
			ib_sg_entry.addr = (uint64_t)ib_buffer + RDMA_BUFFER_MAX_ELEM_SIZE*ib_wc.wr_id;
			ib_wr.wr_id = ib_wc.wr_id;
			ibv_post_recv(ib_settings.qp, &ib_wr, &ib_bad_recv_wr);
		}
	}

	// Send pedestal, header data and collection statistics
	read(sockfd, &(online_statistics[threadID]), sizeof(online_statistics_t));
	read(sockfd, &(gain_pedestal[threadID]), NPIXEL * 6 * sizeof(uint16_t));

	if (exchange_magic_number(sockfd) == 1) exit(EXIT_FAILURE);

	// Close IB connection
	ibv_dereg_mr(ib_buffer_mr);
	close_ibverbs(ib_settings);

	// Free memory buffer
	free(ib_buffer);

	// Close TCP/IP socket
	close(sockfd);

	pthread_exit(0);
}


int main(int argc, char **argv) {
	int ret;

	// Register JF specific filter
	H5Zregister(&H5Z_JF);

	// Parse input
	if (parse_input(argc, argv) == 1) exit(EXIT_FAILURE);

	open_HDF5_file(0);

	pthread_t writer[NCARDS];

	for (int i = 0; i < NCARDS; i++) {
		ret = pthread_create(&(writer[i]), NULL, writer_thread, NULL);
	}

	for (int i = 0; i < NCARDS; i++) {
		ret = pthread_join(writer[i], NULL);
	}
	close_HDF5_file();
}
