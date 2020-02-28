#include <unistd.h>
#include "JungfrauFile.h"

#include "action_test.h"
#include "snap_hls_if.h"
#include "bitshuffle_core.h"

#define NFRAMES 10
#define MODULE 0

struct frame_header_t {
	uint64_t frame_number;
	uint16_t udp_src_port;
	uint16_t udp_dest_port;
	uint32_t jf_debug;
	uint64_t jf_timestamp;
	uint64_t jf_bunchid;
};

void hls_action(snap_membus_t *din_gmem, snap_membus_t *dout_gmem,
		snap_HBMbus_t *d_hbm_p0, snap_HBMbus_t *d_hbm_p1,
		snap_HBMbus_t *d_hbm_p2, snap_HBMbus_t *d_hbm_p3,
		snap_HBMbus_t *d_hbm_p4, snap_HBMbus_t *d_hbm_p5,
		snap_HBMbus_t *d_hbm_p6, snap_HBMbus_t *d_hbm_p7,
		snap_HBMbus_t *d_hbm_p8, snap_HBMbus_t *d_hbm_p9,
		snap_HBMbus_t *d_hbm_p10, snap_HBMbus_t *d_hbm_p11,

		AXI_STREAM &din_eth, AXI_STREAM &dout_eth, ap_uint<1> & eth_reset,
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
	packet->udp_dest_port = MODULE + 0xC0CC; // module number
	packet->udp_sour_port = 0xACDF;
	packet->timestamp = 0xA0A0A0A0;
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

inline void loadBinFile(std::string name, char *dest, size_t size) {
	printf("Loading %s\n", name.c_str());
	std::fstream file10 = std::fstream(name.c_str(), std::ios::in | std::ios::binary);
	if (!file10.is_open()) {
		printf ("Error opening file %s\n",name.c_str());
	} else {
		file10.read(dest, size);
		file10.close();
	}
}

void load_gain(void *in_gain, std::string fname, int module, double energy_in_keV) {
	double *tmp_gain = (double *) calloc (3 * MODULE_COLS * MODULE_LINES, sizeof(double));
	std::fstream infile;

	loadBinFile(fname, (char *) tmp_gain, 3 * MODULE_COLS * MODULE_LINES * sizeof(double));

	gainG0_t   *in_gain_in_G0   = (gainG0_t *)   in_gain;
	gainG1G2_t *in_gain_in_G1G2 = (gainG1G2_t *) in_gain;
	size_t offset = module * MODULE_COLS * MODULE_LINES;

	for (int i = 0; i < MODULE_COLS * MODULE_LINES; i ++) {
		in_gain_in_G0[offset+i] =  (gainG0_t) (512.0 / ( tmp_gain[i] * energy_in_keV));
	}

	offset += NPIXEL;
	for (int i = 0; i < MODULE_COLS * MODULE_LINES; i ++) {
		in_gain_in_G1G2[offset+i] =  (gainG1G2_t) (-1.0 / ( tmp_gain[i + MODULE_COLS * MODULE_LINES] * energy_in_keV));
	}

	offset += NPIXEL;
	for (int i = 0; i < MODULE_COLS * MODULE_LINES; i ++) {
		in_gain_in_G1G2[offset+i] =  (gainG1G2_t) (-1.0 / ( tmp_gain[i + 2 * MODULE_COLS * MODULE_LINES] * energy_in_keV));
	}

	free(tmp_gain);
}

void load_pedestal(void *in_gain, std::string fname, int module, int array_offset) {
	float *tmp_pede = (float *) calloc (MODULE_COLS * MODULE_LINES, sizeof(float));

	loadBinFile(fname, (char *)  tmp_pede, MODULE_COLS * MODULE_LINES * sizeof(float));

	pedeG1G2_t *in_gain_inpede = (pedeG1G2_t *) in_gain;
	size_t offset = module * MODULE_COLS * MODULE_LINES + array_offset * NPIXEL;

	for (int i = 0; i < MODULE_COLS * MODULE_LINES; i ++) {
		in_gain_inpede[offset+i] =  tmp_pede[i];

	}
	free(tmp_pede);
}

int main(int argc, char *argv[]) {
	int retval = 0;

	snap_membus_t *din_gmem = 0;
	snap_membus_t *dout_gmem = 0;

	snap_HBMbus_t *d_hbm_p0 = (snap_HBMbus_t *) calloc(1024*1024*256, 1);
	snap_HBMbus_t *d_hbm_p1 = (snap_HBMbus_t *) calloc(1024*1024*256, 1);
	snap_HBMbus_t *d_hbm_p2 = (snap_HBMbus_t *) calloc(1024*1024*256, 1);
	snap_HBMbus_t *d_hbm_p3 = (snap_HBMbus_t *) calloc(1024*1024*256, 1);
	snap_HBMbus_t *d_hbm_p4 = (snap_HBMbus_t *) calloc(1024*1024*256, 1);
	snap_HBMbus_t *d_hbm_p5 = (snap_HBMbus_t *) calloc(1024*1024*256, 1);
	snap_HBMbus_t *d_hbm_p6 = (snap_HBMbus_t *) calloc(1024*1024*256, 1);
	snap_HBMbus_t *d_hbm_p7 = (snap_HBMbus_t *) calloc(1024*1024*256, 1);
	snap_HBMbus_t *d_hbm_p8 = (snap_HBMbus_t *) calloc(1024*1024*256, 1);
	snap_HBMbus_t *d_hbm_p9 = (snap_HBMbus_t *) calloc(1024*1024*256, 1);
	snap_HBMbus_t *d_hbm_p10 = (snap_HBMbus_t *) calloc(1024*1024*256, 1);
	snap_HBMbus_t *d_hbm_p11 = (snap_HBMbus_t *) calloc(1024*1024*256, 1);


    void* in_gain = snap_malloc(6*(NPIXEL)*sizeof(uint16_t));
    memset(in_gain, 0x0, 6*(NPIXEL)*sizeof(uint16_t));

    int16_t *out_frame_buffer = (int16_t *) snap_malloc(FRAME_BUF_SIZE*NPIXEL*sizeof(uint16_t));
    memset(out_frame_buffer, 0x0, FRAME_BUF_SIZE*NPIXEL*sizeof(uint16_t));

    uint8_t  *out_frame_buffer_status = (uint8_t *) snap_malloc(NFRAMES*NMODULES*128/8+64); 
    memset(out_frame_buffer_status, 0x0, NFRAMES*NMODULES*128/8+64);

	frame_header_t* jf_frame_headers = (frame_header_t *)  snap_malloc(NFRAMES*NMODULES*sizeof(frame_header_t));
    memset(jf_frame_headers,0x0, NFRAMES*NMODULES*sizeof(frame_header_t));

    load_gain(in_gain, "test_data/gainMaps_M049.bin", MODULE, 12.4);

    load_pedestal(in_gain, "test_data/mod5_pedeG1.bin", MODULE, 3);
    load_pedestal(in_gain, "test_data/mod5_pedeG2.bin", MODULE, 4);
    load_pedestal(in_gain, "test_data/mod5_pedeG0.bin", MODULE, 5);

	AXI_STREAM din_eth;
	AXI_STREAM dout_eth;

	action_reg action_register;
	action_RO_config_reg Action_Config;

	ap_uint<1> eth_reset;

	action_register.Data.expected_frames = NFRAMES;
	action_register.Data.mode = MODE_CONV;
	action_register.Control.flags = 1;
	action_register.Data.fpga_mac_addr = 0xAABBCCDDEEF1;
	action_register.Data.fpga_ipv4_addr = 0x0A013205; // 10.1.50.5
	action_register.Data.in_gain_pedestal_data_addr = (uint64_t) in_gain;
	action_register.Data.out_frame_buffer_addr = (uint64_t) out_frame_buffer;
	action_register.Data.out_frame_status_addr = (uint64_t) out_frame_buffer_status;
	action_register.Data.out_jf_packet_headers_addr = (uint64_t) jf_frame_headers;
	action_register.Data.pedestalG0_frames = 0;

	uint16_t *frame = (uint16_t *) calloc(MODULE_LINES * MODULE_COLS * NFRAMES, sizeof(uint16_t));
	int16_t *frame_converted = (int16_t *) calloc(MODULE_LINES * MODULE_COLS * NFRAMES, sizeof(uint16_t));

	for (int i = 0; i < NFRAMES; i++) {
		loadBinFile("test_data/mod5_raw" + std::to_string(i) + ".bin", (char *)  (frame + NCH * i), MODULE_LINES * MODULE_COLS * sizeof(uint16_t));
		loadBinFile("test_data/mod5_conv" + std::to_string(i) + ".bin", (char *)  (frame_converted + NCH * i), MODULE_LINES * MODULE_COLS * sizeof(uint16_t));
		for (int j = 0; j < 128; j++) {
			make_packet(din_eth, i+1, j, frame + NCH * i + 4096 * j);
		}
	}

	make_packet(din_eth, NFRAMES+100, 0, frame);

    hls_action(din_gmem, dout_gmem, d_hbm_p0, d_hbm_p1, d_hbm_p2, d_hbm_p3, d_hbm_p4, d_hbm_p5,
    		d_hbm_p6, d_hbm_p7, d_hbm_p8, d_hbm_p9, d_hbm_p10, d_hbm_p11,
    		din_eth, dout_eth, eth_reset, &action_register, &Action_Config);

	ap_axiu_for_eth packet_out;

	dout_eth.read(packet_out);

	int16_t *out_frame_buffer_unshuf = (int16_t *) snap_malloc(FRAME_BUF_SIZE*NPIXEL*sizeof(uint16_t));

	bshuf_bitunshuffle(out_frame_buffer, out_frame_buffer_unshuf, NFRAMES*NPIXEL, 2, 32);

	if (action_register.Data.mode == MODE_CONV) {
		double mean_error = 0.0;
		for (int i = 0; i < NFRAMES; i++) {
			for (int j = 0; j < NCH; j++) {
				if (!((frame_converted[i*NCH+j]  - out_frame_buffer_unshuf[i*NMODULES*NCH+j+MODULE*NCH] > 20000) || (frame_converted[i*NCH+j]  - out_frame_buffer_unshuf[i*NMODULES*NCH+j+MODULE*NCH] < -20000)))
					mean_error += (frame_converted[i*NCH+j] - out_frame_buffer_unshuf[i*NMODULES*NCH+j+MODULE*NCH])*(frame_converted[i*NCH+j] - out_frame_buffer_unshuf[i*NMODULES*NCH+j+MODULE*NCH]);

			}
		}
		mean_error = sqrt(mean_error/ (NFRAMES*NCH));
		std::cout << "Mean error " << mean_error << std::endl;
		if (mean_error > 0.33) retval = 2;
	} else {
		for (int i = 0; i < NFRAMES; i++) {
			for (int j = 0; j < NCH; j++) {
				if (frame[i*NCH+j] != out_frame_buffer[i*NMODULES*NCH+j+MODULE*NCH])  retval = 2;
			}
		}
	}
	__hexdump(stdout, out_frame_buffer, 16*64);
	__hexdump(stdout, out_frame_buffer_unshuf, 16*64);
	__hexdump(stdout, frame_converted, 16*64);

	__hexdump(stdout, jf_frame_headers, NFRAMES*NMODULES*sizeof(frame_header_t));
	__hexdump(stdout, out_frame_buffer_status, NFRAMES*NMODULES*128/8+64);

	free(d_hbm_p0);
	free(d_hbm_p1);
	free(d_hbm_p2);
	free(d_hbm_p3);
	free(d_hbm_p4);
	free(d_hbm_p5);
	free(d_hbm_p6);
	free(d_hbm_p7);
	free(d_hbm_p8);
	free(d_hbm_p9);
	free(d_hbm_p10);
	free(d_hbm_p11);

	free(frame);
	free(frame_converted);

	return retval;
}
