#include <unistd.h>
#include "JungfrauFile.h"

#include "action_test.h"
#include "snap_hls_if.h"

#define NFRAMES 256
#define MODULE 0

void hls_action(snap_membus_t *din_gmem, snap_membus_t *dout_gmem,
		snap_membus_t *d_hbm_p0, snap_membus_t *d_hbm_p1,
		snap_membus_t *d_hbm_p2, snap_membus_t *d_hbm_p3,
		snap_membus_t *d_hbm_p4, snap_membus_t *d_hbm_p5,
		AXI_STREAM &din_eth, AXI_STREAM &dout_eth,
		action_reg *act_reg, action_RO_config_reg *Action_Config);

// From snap_tools.h - see LICENSE there
static inline void __hexdump(FILE *fp, const void *buff, unsigned int size)
{
        unsigned int i;
        const uint8_t *b = (uint8_t *)buff;
        char ascii[17];
        char str[2] = { 0x0, };

        if (size == 0)
                return;

        for (i = 0; i < size; i++) {
                if ((i & 0x0f) == 0x00) {
                        fprintf(fp, " %08x:", i);
                        memset(ascii, 0, sizeof(ascii));
                }
                fprintf(fp, " %02x", b[i]);
                str[0] = isalnum(b[i]) ? b[i] : '.';
                str[1] = '\0';
                strncat(ascii, str, sizeof(ascii) - 1);

                if ((i & 0x0f) == 0x0f)
                        fprintf(fp, " | %s\n", ascii);
        }
        // print trailing up to a 16 byte boundary.
        for (; i < ((size + 0xf) & ~0xf); i++) {
                fprintf(fp, "   ");
                str[0] = ' ';
                str[1] = '\0';
                strncat(ascii, str, sizeof(ascii) - 1);

                if ((i & 0x0f) == 0x0f)
                        fprintf(fp, " | %s\n", ascii);
        }
        fprintf(fp, "\n");
}

void make_packet(AXI_STREAM &din_eth, uint64_t frame_number, uint32_t eth_packet, uint16_t *data) {
	void *buff = calloc(130,64);
	RAW_JFUDP_Packet *packet = (RAW_JFUDP_Packet *)buff;
	ap_uint<512> *obuff = (ap_uint<512> *)buff;
	packet->ether_type = 0x0008;
	packet->sour_mac[0] = 0x00; // module 0

	packet->dest_mac[0] = 0xAA; // Big endian in IP header!
	packet->dest_mac[1] = 0xBB;
	packet->dest_mac[2] = 0xCC;
	packet->dest_mac[3] = 0xDD;
	packet->dest_mac[4] = 0xEE;
	packet->dest_mac[5] = 0xF1;
	packet->ipv4_header_h = 0x45; // Big endian in IP header!
    packet->ipv4_header_total_length = 0x4C20; // Big endian in IP header!
	packet->ipv4_header_dest_ip = 0x0532010A; // Big endian in IP header!
	packet->ipv4_header_ttl_protocol = 0x1100;
	packet->udp_dest_port = MODULE * 0x0100; // module number
	packet->framenum = frame_number;
	packet->packetnum = eth_packet;

	for (int i = 0; i < 4096; i++) packet->data[i] = data[i];

	//__hexdump(stdout, obuff,64*130);
	ap_axiu_for_eth packet_in;

	for (int i = 0; i < 130; i++) {
		if (i == 129) packet_in.last = 1;
		else packet_in.last = 0;
		packet_in.keep = 0xFFFFFFFFFFFFFFFF;
		packet_in.user = 0; // TODO: Check 1
		packet_in.data = obuff[i];
		din_eth.write(packet_in);
	}

}


int main(int argc, char *argv[]) {
	int retval = 0;

	snap_membus_t *din_gmem = 0;
	snap_membus_t *dout_gmem = 0;

	snap_membus_t *d_hbm_p0 = (snap_membus_t *) calloc(1024*1024*256, 1);
	snap_membus_t *d_hbm_p1 = (snap_membus_t *) calloc(1024*1024*256, 1);
	snap_membus_t *d_hbm_p2 = (snap_membus_t *) calloc(1024*1024*256, 1);
	snap_membus_t *d_hbm_p3 = (snap_membus_t *) calloc(1024*1024*256, 1);
	snap_membus_t *d_hbm_p4 = (snap_membus_t *) calloc(1024*1024*256, 1);
	snap_membus_t *d_hbm_p5 = (snap_membus_t *) calloc(1024*1024*256, 1);

    void* in_gain = snap_malloc(6*(NMODULES * 512 * 1024)*sizeof(uint16_t));
	uint16_t *out_frame_buffer = (uint16_t *) snap_malloc(FRAME_BUF_SIZE*NMODULES*MODULE_COLS*MODULE_LINES*sizeof(uint16_t));
	uint8_t *out_frame_buffer_status = (uint8_t *) snap_malloc((NMODULES*100000+64)*sizeof(uint8_t));

	for (int i = 0; i < 100000; i++)
			for (int j = 0; j < NMODULES; j++)
				out_frame_buffer_status[i*NMODULES+j] = 0;
	AXI_STREAM din_eth;
	AXI_STREAM dout_eth;

	action_reg action_register;
	action_RO_config_reg Action_Config;

	action_register.Data.packets_to_read = NFRAMES * 128L;
	action_register.Data.save_raw = 1;
	action_register.Control.flags = 1;
	action_register.Data.fpga_mac_addr = 0xAABBCCDDEEF1;
	action_register.Data.fpga_ipv4_addr = 0x0A013205; // 10.1.50.5
	action_register.Data.in_gain_pedestal_data.addr = (uint64_t) in_gain;
	action_register.Data.out_frame_buffer.addr = (uint64_t) out_frame_buffer;
	action_register.Data.out_frame_status.addr = (uint64_t) out_frame_buffer_status;

	uint16_t *frame = (uint16_t *) calloc(NCH * NFRAMES, sizeof(uint16_t));

	JungfrauFile file("/mnt/zfs/e16371/20181004/NA102I_Lyso5/NA102I_Lyso5_12p4_100dps_tr1p0_720_1_d4_f000000000000_0.raw", 0);

	for (int i = 0; i < NFRAMES; i++) {
		file.ReadFrame(frame + NCH * i, 0);
		for (int j = 0; j < 128; j++) {
			make_packet(din_eth, i+1, j, frame + NCH * i + 4096 * j);
		}
	}

    hls_action(din_gmem, dout_gmem, d_hbm_p0, d_hbm_p1, d_hbm_p2, d_hbm_p3, d_hbm_p4, d_hbm_p5, din_eth, dout_eth, &action_register, &Action_Config);

	ap_axiu_for_eth packet_out;

	dout_eth.read(packet_out);

	for (int i = 0; i < 100000; i++) {
		for (int j = 0; j < NMODULES; j++) {
			// if (out_frame_buffer_status[i*NMODULES+j+64] != 0) printf("%d %d %d \n",i,j, out_frame_buffer_status[i*NMODULES+j+64]);
			if ((i < NFRAMES ) && (j == MODULE) && (out_frame_buffer_status[i*NMODULES+j+64] != 128)) {
				printf("Status missing for %d %d\n",i,j,out_frame_buffer_status[i*NMODULES+j+64]);
				retval = 1;
			}
			if (((i >= NFRAMES) || (j != MODULE)) && (out_frame_buffer_status[i*NMODULES+j+64] != 0)) {
				printf("Status wrong for %d %d\n",i,j,out_frame_buffer_status[i*NMODULES+j+64]);
				retval = 1;
			}
		}
	}

	for (int i = 0; i < NFRAMES; i++) {
		for (int j = 0; j < NCH; j++) {
			if (frame[i*NCH+j] != out_frame_buffer[i*2*NCH+j+MODULE*NCH]) retval = 2;
		}
	}

	free(d_hbm_p0);
	free(d_hbm_p1);
	free(d_hbm_p2);
	free(d_hbm_p3);
	free(d_hbm_p4);
	free(d_hbm_p5);
	free(frame);

    __hexdump(stdout, out_frame_buffer_status, 64);
	printf("Good packets %d\n",action_register.Data.good_packets);
	printf("Bad packets %d\n",action_register.Data.bad_packets);

	return retval;
}
