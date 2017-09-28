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

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <err.h>
#include <pthread.h>
#include <semaphore.h>

#include <sys/stat.h>
#include <sys/mman.h>

#include "snap_internal.h"
#include "libsnap.h"
#include "snap_hls_if.h"
#include "capiblock.h"

/*
 * FIXME The following stuff most likely neesd to go in a header file 
 * which can be accessed outside this code, or the functionalty in this
 * file needs to be provided in a usable fashion outside this code.
 *
 * We need to use the HDL example here, since that is the only one at
 * this point in time which can trigger NVMe memory moves as well as
 * transfers of DRAM from/to host and card DRAM.
 * 
 * Fix the lun_size since we fake here the device to be 1 GiB.
 * Having multiple NVMe requests in flight will hopefully help to
 * improve the performance.
 */

static inline void __free(void *ptr)
{
	if (ptr == NULL)
		return;
	free(ptr);
}

typedef struct atomic_t {
	pthread_mutex_t lock;
	unsigned long count;
} atomic_t;

static inline void atomic_init(atomic_t *a, unsigned long v)
{
	pthread_mutex_init(&a->lock, NULL);
	a->count = v;
}

static inline unsigned long atomic_inc(atomic_t *a)
{
	unsigned long v;
	pthread_mutex_lock(&a->lock);
	v = a->count++;
	pthread_mutex_unlock(&a->lock);
	return v;
}

#define SNAP_FLASHGT_NVME_SIZE (1ull * 1024 * 1024 * 1024) /* FIXME n TiB */
#define __CBLK_BLOCK_SIZE 4096

enum cblk_status {
	CBLK_IDLE = 0,
	CBLK_READING = 1,
	CBLK_WRITING = 2,
	CBLK_READY = 3,
	CBLK_ERROR = 4,
};

static const char *status_str[] =
	{ "IDLE", "READING", "WRITING", "READY", "ERROR" };
		

struct cblk_req {
	unsigned int num;
	uint8_t slot;
	enum cblk_status status;
	sem_t wait_sem;
	uint8_t *buf;
};

static inline void cblk_set_status(struct cblk_req *req,
				enum cblk_status status)
{
	block_trace("  [%s] req slot %d new status is %s\n", __func__,
		req->slot, status_str[status]);
	req->status = status;
}

#define CBLK_WIDX_MAX		1	/* Just one for now */
#define CBLK_RIDX_MAX		15	/* 15 read slots */
#define CBLK_IDX_MAX		(CBLK_WIDX_MAX + CBLK_RIDX_MAX)

#define CBLK_NBLOCKS_MAX	32	/* 128 KiB / 4KiB */
#define CBLK_NBLOCKS_WRITE_MAX	2	/* writing is just 1 or 2 blocks */

struct cblk_dev {
	struct snap_card *card;
	struct snap_action *act;
	pthread_mutex_t dev_lock;

	unsigned int drive;
	size_t nblocks; /* total size of the device in blocks */
	int timeout;
	uint8_t *buf;

	unsigned int w_in_flight;	/* write transfers in flight */
	unsigned int r_in_flight;	/* read transfers in flight */

	unsigned int widx;
	unsigned int ridx;

	struct cblk_req req[CBLK_IDX_MAX];

	sem_t busy_sem;			/* wait here if there is no free slot */
	sem_t idle_sem;			/* wait here if there is no work */
	pthread_t done_tid;
};

/* Use just one for now ... */
static struct cblk_dev chunk = {
	.dev_lock = PTHREAD_MUTEX_INITIALIZER,
};

static inline unsigned int work_in_flight(struct cblk_dev *c)
{
	return c->w_in_flight + c->r_in_flight;
}

/* Action related definitions. Used to access the hardware */

/*
* Die neue NVMe Action ist im Branch copy4k8k und stellt zwei 
* Funktionen zur Verfügung. Host Memory nach NVMe (action reg 0x30=3)
* und NVMe nach Host Memory ( reg 0x30=4).
* 
* Source, desination und size Register- Zuordnung haben sich nicht 
* geändert. Size ist entweder 4k oder 8k. Ein Read or Write Transfer
* wird mit action start getriggert. Unterstützt werden bis zu 15 Reads
* und 1 Write zur einer Zeit. Alle Transfers müssen eine unterschiedliche
* ID haben. ID 0 ist für den Write Transfer reserviert. ID1 bis 15 für
* die Reads. Die ID muss in Register 0x30 bit 8 bis 11 mit angegeben
* werden.
* 
* Über Register 0x4c kann der Transfer- Status abgefragt
* werden. Bit 4 zeigt an, ob ein Transfer abgeschlossen wurde.
* Bit 0 bis 3 gibt die zugehörige ID an.
* 
* Beispiel:
*  reg 0x4c = 0x10 -> write transfer completed
*  reg 0x4c = 0x1f -> read transfer with ID 15 completed
*  reg 0x4c = 0x00 -> no completion
*/ 

#define ACTION_TYPE_NVME_EXAMPLE	0x10140001	/* Action Type */

#define ACTION_CONFIG		0x30
#define  ACTION_CONFIG_COPY_HN	0x03	/* Memcopy Host DRAM to NVMe */
#define  ACTION_CONFIG_COPY_NH	0x04	/* Memcopy NVMe to Host DRAM */
#define  ACTION_CONFIG_MAX	0x05

#define NVME_DRIVE1		0x10	/* Select Drive 1 for 0a and 0b */

static const char *action_name[] = {
	/* 0         1          2          3          4     */
	"UNKNOWN", "UNKNOWN", "UNKNOWN", "COPY_HN", "COPY_NH",
};

#define ACTION_SRC_LOW		0x34	/* LBA for 03, 04 */
#define ACTION_SRC_HIGH		0x38
#define ACTION_DEST_LOW		0x3c	/* LBA for 03, 04 */
#define ACTION_DEST_HIGH	0x40
#define ACTION_CNT		0x44	/* Count Register or # of 512 Byte Blocks for NVME */

#define ACTION_STATUS		0x4c
#define  ACTION_STATUS_WRITE_COMPLETED	0x10
#define  ACTION_STATUS_READ_COMPLETED	0x1f /* mask id 0x0f */
#define  ACTION_STATUS_NO_COMPLETION	0x00 /* no completion seen */
#define  ACTION_STATUS_COMPLETION_MASK	0x0f /* mask completion bits */

/* defaults */
#define ACTION_WAIT_TIME	10	/* Default timeout in sec */

#define KILO_BYTE		(1024ull)
#define MEGA_BYTE		(1024 * KILO_BYTE)
#define GIGA_BYTE		(1024 * MEGA_BYTE)
#define DDR_MEM_SIZE		(4 * GIGA_BYTE)	  /* Default End of FPGA Ram */
#define DDR_MEM_BASE_ADDR	0x00000000	  /* Default Start of FPGA Ram */
#define HOST_BUFFER_SIZE	(256 * KILO_BYTE) /* Default Size for Host Buffers */
#define NVME_LB_SIZE		512		  /* NVME Block Size */
#define NVME_DRIVE_SIZE		(4 * GIGA_BYTE)	  /* NVME Drive Size */
#define NVME_MAX_TRANSFER_SIZE	(32 * MEGA_BYTE)  /* NVME limit to Transfer in one chunk */

/* NVME lba cache */
#define CACHE_MASK	0x00000003
#define CACHE_WAYS	4
#define CACHE_ENTRIES	(CACHE_MASK + 1)

enum cache_block_status {
	CACHE_BLOCK_UNUSED = 0,	/* not in use yset */
	CACHE_BLOCK_VALID = 1,	/* data is valid  for specified lba */
	CACHE_BLOCK_DIRTY = 2,	/* data was written */
};

static const char *block_status_str[] = { "UNUSED", "VALID", "DIRTY"};

struct cache_way {
	enum cache_block_status status;
	off_t lba;		/* lba this cache entry is for */
	size_t nblocks;		/* use 1 to keep things simple */
	unsigned int used;	/* # times this block was used */
	unsigned int count;	/* eviction counter */
	void *buf;		/* data if status is CBLK_BLOCK_VALID */
};

struct cache_entry {
	pthread_mutex_t way_lock;
	unsigned int count;
	struct cache_way way[CACHE_WAYS];
};

typedef uint8_t cache_block_t[__CBLK_BLOCK_SIZE];

static struct cache_entry cache_entries[CACHE_ENTRIES];
static cache_block_t *cache_blocks = NULL;

static int cache_init(void)
{
	int rc;
	unsigned int i, j;

	rc = posix_memalign((void **)&cache_blocks, __CBLK_BLOCK_SIZE,
		CACHE_ENTRIES * CACHE_WAYS * __CBLK_BLOCK_SIZE);
	if (rc != 0) {
		perror("err: posix_memalign");
		return rc;
	}

	for (i = 0; i < CACHE_ENTRIES; i++) {
		struct cache_entry *entry = &cache_entries[i];
		struct cache_way *way = entry->way;

		pthread_mutex_init(&entry->way_lock, NULL);
		for (j = 0; j < CACHE_WAYS; j++) {
			way[j].status = CACHE_BLOCK_UNUSED;
			way[i].count = 0;
			way[j].buf = &cache_blocks[i * CACHE_WAYS + j];
		}
	}
	return 0;
}

static void cache_done(void)
{
	unsigned int i, j;

	block_trace("CACHE\n");
	for (i = 0; i < CACHE_ENTRIES; i++) {
		struct cache_entry *entry = &cache_entries[i];
		struct cache_way *way = entry->way;

		block_trace("  entry[%d]\n", i);
		for (j = 0; j < CACHE_WAYS; j++) {
			block_trace("    way[%d].status=%s buf=%p lba=%lld\n", j,
				block_status_str[way[j].status],
				way[j].buf, (long long)way[j].lba);
		}
	}

	__free(cache_blocks);
	cache_blocks = NULL;
}

static inline int cache_read(off_t lba, void *buf)
{
	unsigned int i = lba & CACHE_MASK, j;
	struct cache_entry *entry = &cache_entries[i];
	struct cache_way *way = entry->way;

	block_trace("[%s]\n", __func__);
	block_trace("  searching read entry[%d] lba=%lld\n", i, (long long)lba);

	pthread_mutex_lock(&entry->way_lock);
	for (j = 0; j < CACHE_WAYS; j++) {
		if (lba == way[j].lba) {
			way[j].count = entry->count++;
			memcpy(buf, way[j].buf, __CBLK_BLOCK_SIZE);
			pthread_mutex_unlock(&entry->way_lock);
			return 0;
		}
		block_trace("    way[%d].status=%s buf=%p lba=%lld\n",
			j, block_status_str[way[j].status],
			way[j].buf, (long long)way[j].lba);
	}
	pthread_mutex_unlock(&entry->way_lock);

	block_trace("  not found lba=%lld\n", (long long)lba);
	return -1; /* not found */
}

static inline int cache_write(off_t lba, const void *buf)
{
	unsigned int i = lba & CACHE_MASK, j;
	struct cache_entry *entry = &cache_entries[i];
	struct cache_way *way = entry->way;
	uint8_t evict_idx;
	unsigned int min_count;

	block_trace("[%s]\n", __func__);
	block_trace("  searching write entry[%d] lba=%lld\n", i, (long long)lba);

	pthread_mutex_lock(&entry->way_lock);
	evict_idx = 0;
	min_count = way[0].count;

	for (j = 0; j < CACHE_WAYS; j++) {
		if (way[j].lba == lba) { /* entry is already in cache */
			evict_idx = j;
			break;
		} else if (way[j].status == CACHE_BLOCK_UNUSED) {
			evict_idx = j;
			break;		/* assume fill from 0 to n */
		} else if (way[j].count < min_count)
			evict_idx = j;	/* replace entry with smallest count */
	}

	j = evict_idx;
	memcpy(way[j].buf, buf, __CBLK_BLOCK_SIZE);
	way[j].lba = lba;
	way[j].status = CACHE_BLOCK_VALID;
	way[j].count = entry->count++;

	block_trace("    way[%d].status=%s buf=%p lba=%lld replaced\n",
		j, block_status_str[way[j].status],
		way[j].buf, (long long)way[j].lba);

	pthread_mutex_unlock(&entry->way_lock);
	return 0; /* always write */
}

/* Action or Kernel Write and Read are 32 bit MMIO */
static int __cblk_write(struct cblk_dev *c, uint32_t addr, uint32_t data)
{
	int rc;

	rc = snap_mmio_write32(c->card, (uint64_t)addr, data);
	if (0 != rc)
		fprintf(stderr, "err: Write MMIO 32 Err %d\n", rc);

	return rc;
}


/* Action or Kernel Write and Read are 32 bit MMIO */
static int __cblk_read(struct cblk_dev *c, uint32_t addr, uint32_t *data)
{
	int rc;

	rc = snap_mmio_read32(c->card, (uint64_t)addr, data);
	if (0 != rc)
		fprintf(stderr, "err: Read MMIO 32 Err %d\n", rc);

	return rc;
}

/*
 * NVMe: For NVMe transfers n is representing a NVME_LB_SIZE (512)
 *       byte block.
 */
static inline void start_memcpy(struct cblk_dev *c,
				uint32_t action,
				uint64_t dest,
				uint64_t src,
				uint32_t n)
{
	uint8_t action_code = action & 0x00ff;

	block_trace("  %12s memcpy_%x(dest=0x%llx, src=0x%llx n=%lld bytes)\n",
		action_name[action_code % ACTION_CONFIG_MAX], action,
		(long long)dest, (long long)src, (long long)n);

	__cblk_write(c, ACTION_CONFIG, action);
	__cblk_write(c, ACTION_DEST_LOW, (uint32_t)(dest & 0xffffffff));
	__cblk_write(c, ACTION_DEST_HIGH, (uint32_t)(dest >> 32));
	__cblk_write(c, ACTION_SRC_LOW, (uint32_t)(src & 0xffffffff));
	__cblk_write(c, ACTION_SRC_HIGH, (uint32_t)(src >> 32));
	__cblk_write(c, ACTION_CNT, n);

	/* Wait for Action to go back to Idle */
	snap_action_start(c->act);

}

int cblk_init(void *arg __attribute__((unused)),
	      uint64_t flags __attribute__((unused)))
{
	block_trace("[%s] arg=%p flags=%llx\n", __func__, arg,
		(long long)flags);
	return 0;
}

int cblk_term(void *arg __attribute__((unused)),
	      uint64_t flags __attribute__((unused)))
{
	block_trace("[%s] arg=%p flags=%llx\n", __func__, arg,
		(long long)flags);
	return 0;
}

/**
 * Check action results and kick potential waiting threads.
 */
static inline int cblk_wait_done(struct cblk_dev *c, int timeout __attribute__((unused)))
{
	int rc = ETIME;
	uint32_t status = 0x0;
	int slot = -1;

#if 1
	rc = snap_action_completed(c->act, NULL, timeout);
	if (rc == 0) {
		fprintf(stderr, "err: Timeout while Waiting for Idle %d\n", rc);
		return -1;
	}
#endif

	rc = __cblk_read(c, ACTION_STATUS, &status);
	if (rc != 0) {
		fprintf(stderr, "err: MMIO32 read ACTION_STATUS %d\n", rc);
		return -2;
	}

	if (status == ACTION_STATUS_NO_COMPLETION) {
		block_trace("  NO COMPLETION %02x\n", status);
		return -3;
	}

	slot = status & ACTION_STATUS_COMPLETION_MASK;
	if (status == ACTION_STATUS_WRITE_COMPLETED) {
		block_trace("  WRITE_COMPLETED %02x SLOT: %d\n", status, slot);
	} else {
		block_trace("  READ_COMPLETED %02x SLOT: %d\n", status, slot);
	}
	return slot;
}

static void done_thread_cleanup(void *arg)
{
	block_trace("[%s] p=%p\n", __func__, arg);
}

static void *done_thread(void *arg)
{
	struct cblk_dev *c = (struct cblk_dev *)arg;

	block_trace("[%s] arg=%p enter\n", __func__, arg);

	pthread_cleanup_push(done_thread_cleanup, c);

	while (1) {
		block_trace("  [%s] waiting for work ... WORK NEEDED WORK NEEDED\n",
			__func__);
		sem_wait(&c->idle_sem);	/* wait until work is to be done */
		block_trace("  [%s] worken up ...\n", __func__);
		pthread_testcancel();	/* go home if requested */

		/* FIXME Do we need the device lock here? */
		while (work_in_flight(c)) {
			int slot;

			slot = cblk_wait_done(c, c->timeout);
			if (slot < 0) {
				block_trace("  [%s] wait_done returned slot=%d\n",
					__func__, slot);
			} else {
				if ((c->req[slot].status == CBLK_READING) ||
				    (c->req[slot].status == CBLK_WRITING)) {
					block_trace("  [%s] waking up slot %d\n",
						__func__, slot);
					cblk_set_status(&c->req[slot], CBLK_READY);
					sem_post(&c->req[slot].wait_sem);
				} else
					block_trace("  [%s] slot %d status is %s, in_flight=%d/%d "
						"NOTHING TO BE DONE\n",
						__func__, slot, status_str[slot],
						c->w_in_flight, c->r_in_flight);
			}
			pthread_testcancel();	/* go home if requested */
		}
	}

	pthread_cleanup_pop(1);

	block_trace("[%s] arg=%p exit\n", __func__, arg);
	return NULL;
}

chunk_id_t cblk_open(const char *path,
		     int max_num_requests __attribute__((unused)),
		     int mode, uint64_t ext_arg __attribute__((unused)),
		     int flags)
{
	int rc;
	unsigned int i;
	int timeout = ACTION_WAIT_TIME;
	unsigned long have_nvme = 0;
	snap_action_flag_t attach_flags =
		(SNAP_ACTION_DONE_IRQ | SNAP_ATTACH_IRQ);
	struct cblk_dev *c = &chunk;

	block_trace("[%s] opening (%s) ...\n", __func__, path);

	pthread_mutex_lock(&c->dev_lock);

	if (flags & CBLK_OPN_VIRT_LUN) {
		fprintf(stderr, "virtual luns not supported in capi stub\n");
		goto out_err0;
	}

	if (mode != O_RDWR) {
		fprintf(stderr, "err: Only O_RDWR file mode is supported in capi stub\n");
		goto out_err0;
	}

	if (c->card != NULL) { /* already initialized */
		pthread_mutex_unlock(&c->dev_lock);
		return 0;
	}

	/* path must match the following scheme: "/dev/cxl/afu%d.0m" */
	c->card = snap_card_alloc_dev(path, SNAP_VENDOR_ID_IBM,
					 SNAP_DEVICE_ID_SNAP);
	if (NULL == c->card) {
		fprintf(stderr, "err: Cannot open SNAP device %s\n", path);
		goto out_err0;
	}

	/* Check if i do have NVME on this card */
	snap_card_ioctl(c->card, GET_NVME_ENABLED, (unsigned long)&have_nvme);
	if (0 == have_nvme) {
		fprintf(stderr, "err: SNAP NVMe is not enabled on card %s!\n",
			path);
		goto out_err1;
	}

	c->act = snap_attach_action(c->card, ACTION_TYPE_NVME_EXAMPLE,
					attach_flags, timeout);
	if (NULL == c->act) {
		fprintf(stderr, "err: Cannot Attach Action: %x\n",
			ACTION_TYPE_NVME_EXAMPLE);
		goto out_err1;
	}

	c->buf = snap_malloc(CBLK_IDX_MAX * __CBLK_BLOCK_SIZE * CBLK_NBLOCKS_MAX);
	if (c->buf == NULL) {
		fprintf(stderr, "err: Cannot alloc temporary buffer\n");
		goto out_err2;
	}
	
	c->drive = 0;
	c->nblocks = SNAP_FLASHGT_NVME_SIZE / __CBLK_BLOCK_SIZE;
	c->timeout = timeout;
	c->done_tid = 0;
	c->widx = 0;
	c->ridx = 0;
	sem_init(&c->busy_sem, 0, 0);
	sem_init(&c->idle_sem, 0, 0);

	for (i = 0; i < ARRAY_SIZE(c->req); i++) {
		c->req[i].slot = i;
		c->req[i].num = 0;
		c->req[i].buf = c->buf + i * CBLK_NBLOCKS_MAX * __CBLK_BLOCK_SIZE;
		cblk_set_status(&c->req[i], CBLK_IDLE);
		sem_init(&c->req[i].wait_sem, 0, 0);
	}

	rc = pthread_create(&c->done_tid, NULL, &done_thread, &chunk);
	if (rc != 0)
		goto out_err3;

	rc = cache_init();
	if (rc != 0)
		goto out_err4;
	
	pthread_mutex_unlock(&c->dev_lock);
	return 0;

 out_err4:
	pthread_cancel(c->done_tid);
	pthread_join(c->done_tid, NULL);
	c->done_tid = 0;
 out_err3:
	__free(c->buf);
	c->buf = NULL;
 out_err2:
	snap_detach_action(c->act);
	c->act = NULL;
 out_err1:
	snap_card_free(c->card);
	c->card = NULL;
 out_err0:
 	for (i = 0; i < ARRAY_SIZE(c->req); i++) {
		c->req[i].status = CBLK_IDLE;
		sem_destroy(&c->req[i].wait_sem);
	}
	pthread_mutex_unlock(&c->dev_lock);
	return (chunk_id_t)(-1);
}

int cblk_close(chunk_id_t id __attribute__((unused)),
	       int flags __attribute__((unused)))
{
	unsigned int i;
	struct cblk_dev *c = &chunk;

	if (c->card == NULL) {
		errno = EINVAL;
		return -1;
	}

	if (c->done_tid) {
		pthread_cancel(c->done_tid);
		pthread_join(c->done_tid, NULL);
		c->done_tid = 0;
	}

	block_trace("[%s] id=%d ...\n", __func__, (int)id);

	for (i = 0; i < ARRAY_SIZE(c->req); i++) {
		block_trace("  req[%2d]: %s %d\n", i,
			status_str[c->req[i].status],
			c->req[i].num);
		c->req[i].status = CBLK_IDLE;
		sem_destroy(&c->req[i].wait_sem);
	}

	snap_detach_action(c->act);
	snap_card_free(c->card);
	__free(c->buf);

	c->act = NULL;
	c->card = NULL;
	c->nblocks = 0;
	c->timeout = 0;
	c->drive = -1;

	cache_done();
	return 0;
}

int cblk_get_lun_size(chunk_id_t id __attribute__((unused)),
		      size_t *size, int flags __attribute__((unused)))
{
	struct cblk_dev *c = &chunk;

	block_trace("[%s] lun_size=%zu block of %d bytes ...\n",
		__func__, c->nblocks, __CBLK_BLOCK_SIZE);
	if (size)
		*size = c->nblocks;
	return 0;
}

int cblk_get_size(chunk_id_t id, size_t *size, int flags)
{
	cblk_get_lun_size(id, size, flags);
	return 0;
}

int cblk_set_size(chunk_id_t id __attribute__((unused)),
		  size_t size __attribute__((unused)),
		  int flags __attribute__((unused)))
{
	fprintf(stderr, "err: Cannot change size of physical luns\n");
	return -1;
}

static int block_read(struct cblk_dev *c, void *buf, off_t lba,
		size_t nblocks)
{
	int slot, count;
	struct cblk_req *req;
	uint32_t mem_size = __CBLK_BLOCK_SIZE * nblocks;

	block_trace("[%s] reading (%p lba=%zu nblocks=%zu) ...\n",
		__func__, buf, lba, nblocks);

	if (nblocks > CBLK_NBLOCKS_MAX) {
		fprintf(stderr, "err: temp buffer too small!\n");
		errno = EFAULT;
		return -1;
	}

	pthread_mutex_lock(&c->dev_lock);

	while (c->r_in_flight == CBLK_WIDX_MAX) { /* block in case of busy hardware */
		pthread_mutex_unlock(&c->dev_lock);
		block_trace("  [%s] wait for free slot\n", __func__);
		sem_wait(&c->idle_sem);
		block_trace("  [%s] check if we have a free slot\n", __func__);
		pthread_mutex_lock(&c->dev_lock);
	}

	slot = CBLK_WIDX_MAX + c->ridx;
	req = &c->req[slot];

	if (req->status != CBLK_IDLE) {	/* FATAL error scenario */
			fprintf(stderr, "err: req[%d].status not CBLK_IDLE!\n", slot);
		errno = EBUSY;
		pthread_mutex_unlock(&c->dev_lock);
		goto __exit1;
	}

	cblk_set_status(req, CBLK_READING);
	c->r_in_flight++;
	start_memcpy(c,
		ACTION_CONFIG_COPY_NH | (slot << 8), /* NVMe to Host DDR */
		(uint64_t)req->buf,			/* dst */
		lba * __CBLK_BLOCK_SIZE/NVME_LB_SIZE,	/* src */
		mem_size);				/* size */

	/* Kick the thread if we are the 1st one */
	if (c->r_in_flight == 1)
		sem_post(&c->idle_sem);

	count = 0;
	while (req->status == CBLK_READING) {
		block_trace("  [%s] sleeping %d. slot %d status: %s\n",
			__func__, count++, slot, status_str[req->status]);
		sem_wait(&req->wait_sem);
		block_trace("  [%s] continuing slot %d\n", __func__, slot);
	}

	memcpy(buf, req->buf, nblocks * __CBLK_BLOCK_SIZE);
	cblk_set_status(req, CBLK_IDLE);
	c->r_in_flight--;
	c->ridx = (c->ridx + 1) % CBLK_RIDX_MAX; /* pick next slot */

	sem_post(&c->idle_sem);		/* kick potential waiters */
	pthread_mutex_unlock(&c->dev_lock);

	block_trace("[%s] exit lba=%zu nblocks=%zu\n", __func__, lba, nblocks);
	return nblocks;

 __exit1:
	cblk_set_status(req, CBLK_ERROR);
	pthread_mutex_unlock(&c->dev_lock);
	return -1;
}

int cblk_read(chunk_id_t id __attribute__((unused)),
	      void *buf, off_t lba, size_t nblocks,
	      int flags  __attribute__((unused)))
{
	int rc;
	size_t i;

	rc = block_read(&chunk, buf, lba, nblocks);
	if (rc <= 0)
		return rc;

	for (i = 0; i < nblocks; i++) {
		cache_write(lba + i, buf + i * __CBLK_BLOCK_SIZE);
		cache_read(lba + i, buf + i * __CBLK_BLOCK_SIZE);
	}
	return rc;
}

static int block_write(struct cblk_dev *c, void *buf, off_t lba,
		size_t nblocks)
{
	int slot;
	uint32_t mem_size = __CBLK_BLOCK_SIZE * nblocks;
	struct cblk_req *req;

	block_trace("[%s] writing (%p lba=%zu nblocks=%zu) ...\n",
		__func__, buf, lba, nblocks);

	if (nblocks > CBLK_NBLOCKS_WRITE_MAX) {
		fprintf(stderr, "err: just 1 and 2 supported for NBLOCKS!\n");
		errno = EFAULT;
		return -1;
	}

	pthread_mutex_lock(&c->dev_lock);

	while (c->w_in_flight == CBLK_WIDX_MAX) { /* block in case of busy hardware */
		pthread_mutex_unlock(&c->dev_lock);
		block_trace("  [%s] wait for free slot\n", __func__);
		sem_wait(&c->idle_sem);
		block_trace("  [%s] check if we have a free slot\n", __func__);
		pthread_mutex_lock(&c->dev_lock);
	}

	slot = c->widx;
	req = &c->req[slot];

	if (req->status != CBLK_IDLE) {	/* FATAL error scenario */
		fprintf(stderr, "err: req[%d].status not CBLK_IDLE!\n", slot);
		errno = EBUSY;
		pthread_mutex_unlock(&c->dev_lock);
		goto __exit1;
	}

	cblk_set_status(req, CBLK_WRITING);
	memcpy(req->buf, buf, nblocks * __CBLK_BLOCK_SIZE);
	c->w_in_flight++;
	start_memcpy(c,
		ACTION_CONFIG_COPY_HN | (slot << 8),	/* Host DDR to NVMe */
		lba * __CBLK_BLOCK_SIZE/NVME_LB_SIZE,	/* dst */
		(uint64_t)req->buf,			/* src */
		mem_size);				/* size */

	/* Kick the thread if we are the 1st one */
	if (c->w_in_flight == 1)
		sem_post(&c->idle_sem);

	while (req->status == CBLK_WRITING) {
		block_trace("  [%s] sleeping slot %d\n", __func__, slot);
		sem_wait(&req->wait_sem);
		block_trace("  [%s] continuing slot %d\n", __func__, slot);
	}
	
	cblk_set_status(req, CBLK_IDLE);
	c->w_in_flight--;
	c->widx = (c->widx + 1) % CBLK_WIDX_MAX;	 /* pick next slot */

	sem_post(&c->idle_sem);		/* kick potential waiters */
	pthread_mutex_unlock(&c->dev_lock);

	block_trace("[%s] exit lba=%zu nblocks=%zu\n", __func__, lba, nblocks);
	return nblocks;

 __exit1:
	cblk_set_status(req, CBLK_ERROR);
	pthread_mutex_unlock(&c->dev_lock);
	return -1;
}

int cblk_write(chunk_id_t id __attribute__((unused)),
	       void *buf, off_t lba, size_t nblocks,
	       int flags __attribute__((unused)))
{
	return block_write(&chunk, buf, lba, nblocks);
}

static void _init(void) __attribute__((constructor));

static void _init(void)
{
	block_trace("[%s] init\n", __func__);

}

static void _done(void) __attribute__((destructor));

static void _done(void)
{
	block_trace("[%s] exit\n", __func__);
	cblk_close(0, 0);
}