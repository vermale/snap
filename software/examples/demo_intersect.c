/*
 * Copyright 2016, International Business Machines
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

/* Function: Two steps:
 * 1) Copy two or more tables from Host to DDR (memcopy)
 * 2) Do intersection on the tables, and return the result to Host memory
 *      
 *      Assume the table elements have same data struct
 *      The size of intersection result is MIN(Table1, Table2, ...Table N)
 * Only count the time elapsed at step 2. 
 *
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

#include <donut_tools.h>
#include <action_intersect.h>
#include <libdonut.h>
#include <snap_s_regs.h>

#define ACTION_REDAY_IRQ 4

int verbose_flag = 0;

static const char *version = GIT_VERSION;
//static const char *mem_tab[] = { "HOST_DRAM", "CARD_DRAM", "TYPE_NVME" };

/**
 * @brief	prints valid command line options
 *
 * @param prog	current program's name
 */
static void usage(const char *prog)
{
	printf("\nUsage: \n%s [-h] [-v, --verbose] [-V, --version]\n"
	       "  -C, --card     <cardno>   can be (0...3)\n"
	       "  -t, --timeout  <seconds>  timeout seconds.\n"
           "----------------------------------------------\n"
	       "  -i, --input1    <file1.bin> input file 1.\n"
	       "  -j, --input2    <file2.bin> input file 2.\n"
	       "  -o, --output   <result.bin> output file.\n"
           "----------------------------------------------\n"
	       "  -n, --num      <int>      How many elements in the table for random generated array.\n"
	       "  -l, --len      <int>      length of the random string.\n"
	       "  -s, --software            Use software methods.\n"
	       "  -m, --method   <0/1>      0: compare one by one (Only in SW).\n"
           "                            1: Use hash table\n"
           "                            2: Sort and do intersection\n"
           "  -I, --irq                 Enable Interrupts\n"      
	       "\n"
	       "Example:\n"
	       "HW:  sudo ./demo_intersect ...\n"
	       "SW:  DNUT_CONFIG=1 ./demo_intersect ...\n"
	       "\n",
	       prog);
}

static void dnut_prepare_intersect(struct dnut_job *cjob, 
                 struct intersect_job *ijob_i, 
                 struct intersect_job *ijob_o,
                 uint32_t step,
                 uint32_t method,

                 value_t * input_addrs_host[],
                 uint32_t input_sizes[], 
                 value_t * output_addr_host, 
                 uint32_t actual_output_size)
{
    uint64_t ddr_addr = 0x0ull; 

    if (step == 1)
    {
        //Memcopy, source 
        dnut_addr_set( &ijob_i->src_tables_host[0], input_addrs_host[0], input_sizes[0],DNUT_TARGET_TYPE_HOST_DRAM ,
            DNUT_TARGET_FLAGS_ADDR | DNUT_TARGET_FLAGS_SRC);
        dnut_addr_set( &ijob_i->src_tables_host[1], input_addrs_host[1], input_sizes[1],DNUT_TARGET_TYPE_HOST_DRAM ,
            DNUT_TARGET_FLAGS_ADDR | DNUT_TARGET_FLAGS_SRC);
        
        //Memcopy, target 
        ddr_addr = 0;
        dnut_addr_set( &ijob_i->src_tables_ddr[0], (void *)ddr_addr, input_sizes[0], DNUT_TARGET_TYPE_CARD_DRAM ,
            DNUT_TARGET_FLAGS_ADDR | DNUT_TARGET_FLAGS_DST | DNUT_TARGET_FLAGS_END);
        
        ddr_addr = MAX_TABLE_SIZE;
        dnut_addr_set( &ijob_i->src_tables_ddr[1], (void *)ddr_addr, input_sizes[1], DNUT_TARGET_TYPE_CARD_DRAM ,
            DNUT_TARGET_FLAGS_ADDR | DNUT_TARGET_FLAGS_DST | DNUT_TARGET_FLAGS_END);

        //No relation to result_table
    }
    else if (step == 2)
    {
        
        //Memcopy, source 
        ddr_addr = 0;
        dnut_addr_set( &ijob_i->src_tables_ddr[0], (void *)ddr_addr, input_sizes[0],DNUT_TARGET_TYPE_CARD_DRAM ,
            DNUT_TARGET_FLAGS_ADDR | DNUT_TARGET_FLAGS_SRC);
        
        ddr_addr = MAX_TABLE_SIZE;
        dnut_addr_set( &ijob_i->src_tables_ddr[1], (void *)ddr_addr, input_sizes[1],DNUT_TARGET_TYPE_CARD_DRAM ,
            DNUT_TARGET_FLAGS_ADDR | DNUT_TARGET_FLAGS_SRC);
        
        //Memcopy, target 
        dnut_addr_set( &ijob_i->src_tables_host[0], input_addrs_host[0], input_sizes[0],DNUT_TARGET_TYPE_HOST_DRAM ,
            DNUT_TARGET_FLAGS_ADDR | DNUT_TARGET_FLAGS_DST | DNUT_TARGET_FLAGS_END);
        dnut_addr_set( &ijob_i->src_tables_host[1], input_addrs_host[1], input_sizes[1],DNUT_TARGET_TYPE_HOST_DRAM ,
            DNUT_TARGET_FLAGS_ADDR | DNUT_TARGET_FLAGS_DST | DNUT_TARGET_FLAGS_END);

        //No relation to result_table
    }
    else if (step == 3)
    {
        ddr_addr = 0;
        dnut_addr_set( &ijob_i->src_tables_ddr[0], (void *)ddr_addr, input_sizes[0],DNUT_TARGET_TYPE_CARD_DRAM ,
            DNUT_TARGET_FLAGS_ADDR | DNUT_TARGET_FLAGS_SRC);

        ddr_addr = MAX_TABLE_SIZE;
        dnut_addr_set( &ijob_i->src_tables_ddr[1], (void *)ddr_addr, input_sizes[1],DNUT_TARGET_TYPE_CARD_DRAM ,
            DNUT_TARGET_FLAGS_ADDR | DNUT_TARGET_FLAGS_SRC);

        //result_table in DDR 
        // 128 is a dummy value. HW will update this field when finished.
        ddr_addr = 2*MAX_TABLE_SIZE;
        dnut_addr_set (&ijob_i->result_table, (void *)ddr_addr, 128, DNUT_TARGET_TYPE_CARD_DRAM ,
                DNUT_TARGET_FLAGS_ADDR | DNUT_TARGET_FLAGS_DST |
		        DNUT_TARGET_FLAGS_END);

    }
    else if (step == 5)
    {
        //Memcopy, source 
        // reuse src_tables_ddr[0] for the result. 
        ddr_addr = 2*MAX_TABLE_SIZE;
        dnut_addr_set( &ijob_i->src_tables_ddr[0], (void *)ddr_addr, actual_output_size,DNUT_TARGET_TYPE_CARD_DRAM ,
            DNUT_TARGET_FLAGS_ADDR | DNUT_TARGET_FLAGS_SRC);
        
        //Memcopy, target 
        dnut_addr_set (&ijob_i->result_table, output_addr_host, actual_output_size, DNUT_TARGET_TYPE_HOST_DRAM ,
                DNUT_TARGET_FLAGS_ADDR | DNUT_TARGET_FLAGS_DST |
		        DNUT_TARGET_FLAGS_END);
    }
    ijob_i->step = step;
    ijob_i->method = method;
	dnut_job_set(cjob, HLS_INTERSECT_ID, ijob_i, sizeof(*ijob_i),
		    ijob_o, sizeof(*ijob_o));
}


static int gen_random_table(value_t table[], uint32_t num,uint32_t len)
{
    
    //Parameters: 
    // num: how many elements in the table
    //Value
    uint32_t i,j;
    value_t pattern;


    if(len <=0 || len >= sizeof(value_t))
    {
        printf(" Error length when generating a random table.\n");
        return -1;
    }
    for (i = 0; i < num; i++)
    {
    
        for(j = 0; j < len; j++)
        {
            pattern[j] = (char)(rand()%26+97); //generate letters. 
        }
        for(j = len; j < sizeof(value_t)-1; j++)
        {
            pattern[j] = 32; //space 
        }
        
        pattern[j] = '\0'; 
        copyvalue(table[i], pattern);
    }

    return 0;

}

static void dump_table(value_t* table, uint32_t num)
{
    uint32_t i;
    printf("Table: \n");
    for (i = 0; i < num; i++)
    {
        printf("%d: %s,\n", i, table[i] );
    }
    printf("\n");
}

static int run_one_step(struct dnut_kernel *kernel, struct dnut_job *cjob, unsigned long timeout, int action_irq, uint64_t step)
{

    int rc;
    struct timeval etime, stime;
    gettimeofday(&stime, NULL);
 	if (action_irq) {
		dnut_kernel_mmio_write32(kernel, 0x8, 1);
		dnut_kernel_mmio_write32(kernel, 0x4, 1);
	}
	rc = dnut_kernel_sync_execute_job(kernel, cjob, timeout, action_irq);
    if (action_irq) {
		dnut_kernel_mmio_write32(kernel, 0xc, 1);
		dnut_kernel_mmio_write32(kernel, 0x4, 0);
	}
	if (rc != 0) {
		fprintf(stderr, "err: job execution %d: %s!\n\n\n", rc,
			strerror(errno));
		return rc;
	}
	gettimeofday(&etime, NULL);
	fprintf(stdout, "Step %ld took %lld usec\n",
		step, (long long)timediff_usec(&etime, &stime));

    return rc;
}

/**
 * Read accelerator specific registers. Must be called as root!
 */
int main(int argc, char *argv[])
{
    //General variables for donut call
	int ch; 
    int rc = 0;
	int card_no = 0;
	struct dnut_kernel *kernel = NULL;
	char device[128];
	uint32_t page_size = sysconf(_SC_PAGESIZE);
    int exit_code = EXIT_SUCCESS;
    unsigned long timeout = 1000;
	struct dnut_job cjob;
    int attach_flags = SNAP_CCR_DIRECT_MODE;
    int action_irq = 0;
    struct timeval etime, stime;

    //Function specific
    //long long time_us;
    struct intersect_job ijob_i, ijob_o;
    value_t * src_tables[NUM_TABLES];
    uint32_t  src_sizes[NUM_TABLES];
    FILE *fp;

    value_t * result_table = NULL;
    value_t * temp_ptr;
    uint32_t  init_result_size;
    uint32_t  actual_result_size;
    uint32_t result_num;
    uint32_t i;
    
    uint32_t min_num = -1; //MAX for unsigned.


    //For random generated table....
    uint32_t num = 20; 
    uint32_t len = 1;
   
    
    //Several global variables. 
    uint32_t method = 1;
    uint32_t sw = 0;
    
    const char *input[NUM_TABLES];
    for(i = 0; i < NUM_TABLES; i++)
        input[i] = NULL;
	
    const char *output = NULL;

	
    while (1) {
		int option_index = 0;
		static struct option long_options[] = {
			{ "card",	 required_argument, NULL, 'C' },
			{ "input1",	 required_argument, NULL, 'i' },
			{ "input2",	 required_argument, NULL, 'j' },
			{ "output",	 required_argument, NULL, 'o' },
			{ "num",	 required_argument, NULL, 'n' },
			{ "len",	 required_argument, NULL, 'l' },
			{ "method",	 required_argument, NULL, 'm' },
			{ "software",required_argument, NULL, 's' },
			{ "timeout", required_argument, NULL, 't' },
			{ "version", no_argument,	    NULL, 'V' },
			{ "verbose", no_argument,	    NULL, 'v' },
			{ "irq",     no_argument,	    NULL, 'I' },
			{ "help",	 no_argument,	    NULL, 'h' },
			{ 0,		 no_argument,	    NULL, 0   },
		};

		ch = getopt_long(argc, argv,
				 "C:i:j:o:m:n:l:t:VIvhs",
				 long_options, &option_index);
		if (ch == -1)
			break;

		switch (ch) {
		case 'C':
			card_no = strtol(optarg, (char **)NULL, 0);
			break;
		case 'i':
			input[0] = optarg;
			break;
		case 'j':
			input[1] = optarg;
			break;
		case 'o':
			output = optarg;
			break;
		case 'n':
			num = __str_to_num(optarg);
			break;
		case 'l':
			len = __str_to_num(optarg);
			break;
		case 't':
			timeout = strtol(optarg, (char **)NULL, 0);
			break;
		case 'm':
			method = strtol(optarg, (char **)NULL, 0);
			break;
		case 's':
			sw = 1;
			break;
        
        /* service */
		case 'V':
			printf("%s\n", version);
			exit(EXIT_SUCCESS);
		case 'v':
			verbose_flag = 1;
			break;
		case 'h':
			usage(argv[0]);
			exit(EXIT_SUCCESS);
			break;
        case 'I': /* irq */
            attach_flags |= SNAP_CCR_IRQ_ATTACH;
            action_irq = ACTION_REDAY_IRQ;
            break;
		default:
			usage(argv[0]);
			exit(EXIT_FAILURE);
		}
	}

	if (optind != argc) {
		usage(argv[0]);
		exit(EXIT_FAILURE);
	}


    //Create Input tables
    
    if (input[0] == NULL || input[1] == NULL) 
    {
        //Randomly generate the Table data
        for (i = 0; i < NUM_TABLES; i++)
        {
            src_sizes[i] = num*sizeof(value_t); //All tables are of same size.
            
            src_tables[i] = memalign (page_size, src_sizes[i]); 
            if(!src_tables[i])
                goto out_error2;

            rc |= gen_random_table(src_tables[i], num, len);
            
            printf("Source table address is %p\n",src_tables[i]);

            if(1)
                dump_table(src_tables[i], num);
        }



    }
    else
    {

        int filesize[2];
        uint32_t j;
        value_t temp_data;

        for (i = 0; i < NUM_TABLES; i++)
        {
            filesize[i] = __file_size(input[i]);
            if (filesize[i] < 0)
                goto out_error;

            num = filesize[i]/sizeof(value_t);// We Assume the input file is formated !!
            src_sizes[i] = num * sizeof(value_t);
            if(num < min_num)
                min_num = num;
            
            src_tables[i] = memalign(page_size, src_sizes[i]);
            if(!src_tables[i])
                goto out_error2;
            
            fp = fopen(input[i], "rb");
            if(!fp)
            {
                fprintf(stderr, "Err: cannot open file!\n");
                goto out_error2;
            }

            for( j = 0; j < num; j++)
            {
                fgets(temp_data, sizeof(value_t)+1, fp);
                temp_data[sizeof(value_t)-1] = '\0';
                copyvalue(src_tables[i][j], temp_data);
            }

            
            fclose(fp);

            fprintf(stdout, "reading input data %d elements from %s\n",
                num, input[i]);

            if(0)
                dump_table(src_tables[i], num);
  
            if (rc < 0)
			    goto out_error;
        }

    }
    

    // Apply result_table. 
    init_result_size = min_num * sizeof(value_t);
    result_table = memalign(page_size, init_result_size);
    if (!result_table)
        goto out_error2;

    
    /////////////////////////////////////////////////////////////////
    //    Open Device ... and start 
    /////////////////////////////////////////////////////////////////
    snprintf(device, sizeof(device)-1, "/dev/cxl/afu%d.0s", card_no);
	kernel = dnut_kernel_attach_dev(device,
					0x1014,
					0xcafe,
					HLS_INTERSECT_ID&0xFFFF);
	if (kernel == NULL) {
		fprintf(stderr, "err: failed to open card %u: %s\n", card_no,
			strerror(errno));
		goto out_error;
	}

#if 1				/* FIXME Circumvention should go away */
	pr_info("FIXME Wait a sec ...\n");
	sleep(1);
#endif
    rc = dnut_attach_action((void*)kernel, HLS_INTERSECT_ID, attach_flags, timeout);
	if (rc != 0) {
		fprintf(stderr, "err: job Attach %d: %s!\n", rc,
			strerror(errno));
		goto out_error2;
	}

#if 1				/* FIXME Circumvention should go away */
	pr_info("FIXME temporary setting to define memory base address\n");
	dnut_kernel_mmio_write32(kernel, 0x00030, 0);
	dnut_kernel_mmio_write32(kernel, 0x00034, 0);
	dnut_kernel_mmio_write32(kernel, 0x00040, 0);
	dnut_kernel_mmio_write32(kernel, 0x00044, 0);
	dnut_kernel_mmio_write32(kernel, 0x00050, 0);
	dnut_kernel_mmio_write32(kernel, 0x00054, 0);
#endif

    //------------------------------------
    // Action begin (1) 
    printf("Start Step1 (Copy source data from Host to DDR) ..............\n");
	dnut_prepare_intersect(&cjob, &ijob_i, &ijob_o,
                 1, method, src_tables, src_sizes,result_table,99);
    
    rc |= run_one_step(kernel, &cjob, timeout, action_irq, 1);
	if (rc != 0)
		goto out_error;
    
    if(sw)
    {
        //------------------------------------
        // Action begin (2)
        printf("Start Step2 (Copy source data from DDR to Host) ..............\n");
        dnut_prepare_intersect(&cjob, &ijob_i, &ijob_o,
                     2, method, src_tables, src_sizes,result_table,99);
        
        rc |= run_one_step(kernel, &cjob, timeout, action_irq, 2);
        if (rc != 0)
            goto out_error;

        //------------------------------------
        // Action begin (4)
        printf("Start Step4 (Do interesction by software) ..............\n");
        gettimeofday(&stime, NULL);
        result_num = run_sw_intersection (method, src_tables[0], src_sizes[0]/sizeof(value_t),
                                                                            src_tables[1], src_sizes[1]/sizeof(value_t), result_table);
	    gettimeofday(&etime, NULL);
	    fprintf(stdout, "Step 4 took %lld usec\n", (long long)timediff_usec(&etime, &stime));
        printf("SW: result_num = %d\n", result_num);


    }
    else
    {
        //------------------------------------
        // Action begin (3)
        printf("Start Step3 (Do intersection in DDR) ..............\n");
        dnut_prepare_intersect(&cjob, &ijob_i, &ijob_o,
                     3, method, src_tables, src_sizes, result_table, 99);
        
        rc |= run_one_step(kernel, &cjob, timeout, action_irq, 3);
        if (rc != 0)
            goto out_error;
        
        actual_result_size = ijob_o.result_table.size;  //in bytes
        result_num = actual_result_size/sizeof(value_t);
        printf("HW: result_num = %d\n", result_num);


        //------------------------------------
        // Action begin (5)
        printf("Start Step5 (Copy result from DDR to Host) ..............\n");
        dnut_prepare_intersect(&cjob, &ijob_i, &ijob_o,
                     5, method, src_tables, src_sizes, result_table, actual_result_size);
        
        rc |= run_one_step(kernel, &cjob, timeout, action_irq, 5);
        if (rc != 0)
            goto out_error;
    }
        
    

    /// Print the results
    temp_ptr = result_table;
    for(i = 0;( i< result_num && verbose_flag); i++)
    {
        printf("%s;\n", *temp_ptr);
        temp_ptr ++;
    }
    printf("\n");



	dnut_kernel_free(kernel);

    for(i = 0; i < NUM_TABLES; i++)
	    __free(src_tables[i]);
	__free(result_table);

	exit(exit_code);

 out_error2:
	dnut_kernel_free(kernel);

 out_error:
    for(i = 0; i < NUM_TABLES; i++)
	    __free(src_tables[i]);
	__free(result_table);
	exit(EXIT_FAILURE);
}