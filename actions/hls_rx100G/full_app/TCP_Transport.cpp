#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>

#include "IB_Transport.h"

struct tcp_ib_exchange_t {
	uint16_t dlid;
	uint32_t dest_qp_num; 
	uint32_t rq_psn;
};

// Server = IB Receiver
int exchange_ib_info_server(unsigned short port, uint16_t local_dlid, uint32_t local_dest_qp_num, 
		uint16_t &remote_dlid, uint32_t &remote_dest_qp_num, uint32_t &rq_psn) {
	tcp_ib_exchange_t tcp_ib_exchange_in, tcp_ib_exchange_out;
	tcp_ib_exchange_out.dlid = local_dlid;
	tcp_ib_exchange_out.dest_qp_num = local_dest_qp_num;

	// Use socket to exchange keys
	int sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd == 0) {
		std::cout << "Socket error" << std::endl;
		exit(EXIT_FAILURE);
		return 1;
	}

	struct sockaddr_in srv_address, client_address;
	srv_address.sin_family = AF_INET; 
	srv_address.sin_addr.s_addr = INADDR_ANY; 
	srv_address.sin_port = htons( port );

	if (bind(sockfd, (struct sockaddr *)&srv_address, sizeof(srv_address)) < 0) {		
		std::cout << "Bind error" << std::endl;
		return 1;
	}

	listen(sockfd, 1);

	// Accept connection
	socklen_t addrlen = sizeof(client_address);	        
	int incoming_socket = accept(sockfd, (struct sockaddr *)&client_address, &addrlen);

	// Send parameters
	send(incoming_socket, &tcp_ib_exchange_out, sizeof(tcp_ib_exchange_t), 0);
	// Receive parameters
	read(incoming_socket, &tcp_ib_exchange_in, sizeof(tcp_ib_exchange_t));

	remote_dlid = tcp_ib_exchange_in.dlid;
	remote_dest_qp_num = tcp_ib_exchange_in.dest_qp_num;
	rq_psn = tcp_ib_exchange_in.rq_psn;

	close(sockfd);
	return 0;
}

// Client = IB Sender
int exchange_ib_info_client(std::string hostname, unsigned short port, uint16_t local_dlid, uint32_t local_dest_qp_num, 
		uint16_t &remote_dlid, uint32_t &remote_dest_qp_num, uint32_t sq_psn) {
	tcp_ib_exchange_t tcp_ib_exchange_in, tcp_ib_exchange_out;
	tcp_ib_exchange_out.dlid = local_dlid;
	tcp_ib_exchange_out.dest_qp_num = local_dest_qp_num;
	tcp_ib_exchange_out.rq_psn = sq_psn;

	// Use socket to exchange connection information
	int sockfd = socket(AF_INET, SOCK_STREAM, 0);
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
		std::cout << "Host not found" << std::endl;
		return 1;
	}
	if (connect(sockfd, host_data[0].ai_addr, host_data[0].ai_addrlen) < 0) {
		std::cout << "Cannot connect to server" << std::endl;
		return 1;
	}

	// Send parameters
	send(sockfd, &tcp_ib_exchange_out, sizeof(tcp_ib_exchange_t), 0);
	// Receive parameters
	read(sockfd, &tcp_ib_exchange_in, sizeof(tcp_ib_exchange_t));

	// Save data
	remote_dlid = tcp_ib_exchange_in.dlid;
	remote_dest_qp_num = tcp_ib_exchange_in.dest_qp_num;

	close(sockfd);
	return 0;
}