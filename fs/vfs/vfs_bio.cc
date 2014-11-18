/*
 * Copyright (c) 2005-2007, Kohsuke Ohtani
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of any co-contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * vfs_bio.c - buffered I/O operations
 */

/*
 * References:
 *	Bach: The Design of the UNIX Operating System (Prentice Hall, 1986)
 */

#include <osv/prex.h>
#include <osv/buf.h>
#include <osv/bio.h>
#include <osv/device.h>

#include <errno.h>
#include <limits.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <semaphore.h>

#include "vfs.h"
#include <boost/intrusive/list.hpp>

/* macros to clear/set/test flags. */
#define	SET(t, f)	(t) |= (f)
#define	CLR(t, f)	(t) &= ~(f)
#define	ISSET(t, f)	((t) & (f))

static int
rw_buf(struct buf *bp, int rw)
{
	struct bio *bio;
	int ret;

	bio = alloc_bio();
	if (!bio)
		return ENOMEM;

	// XXX ROB: Why does this have to be malloc'd?
	bio->bio_data = malloc(BSIZE);
	if (rw) {
		bio->bio_cmd = BIO_WRITE;
		memcpy(bio->bio_data, bp->b_data, BSIZE);
	}
	else {
		bio->bio_cmd = BIO_READ;
	}

	bio->bio_dev = bp->b_dev;
	bio->bio_offset = bp->b_blkno << 9;
	bio->bio_bcount = BSIZE;

	bio->bio_dev->driver->devops->strategy(bio);
	ret = bio_wait(bio);

	if (!rw) {
		memcpy(bp->b_data, bio->bio_data, BSIZE);
	}

	free(bio->bio_data);
	destroy_bio(bio);
	return ret;
}

/*
 * Assign a buffer for the given block.
 *
 * The block is selected from the buffer list with LRU
 * algorithm.  If the appropriate block already exists in the
 * block list, return it.  Otherwise, the least recently used
 * block is used.
 */
void
getblk(struct device *dev, int blkno, struct buf *bp)
{
	DPRINTF(VFSDB_BIO, ("getblk: dev=%x blkno=%d\n", dev, blkno));
	bp->b_flags = B_BUSY;
	bp->b_dev = dev;
	bp->b_blkno = blkno;
	DPRINTF(VFSDB_BIO, ("getblk: done bp=%x\n", bp));
}

/*
 * Release a buffer, with no I/O implied.
 */
void
brelse(struct buf *bp)
{
}

/*
 * Block read with cache.
 * @dev:   device id to read from.
 * @blkno: block number.
 * @buf:   buffer pointer to be returned.
 *
 * An actual read operation is done only when the cached
 * buffer is dirty.
 */
int
bread(struct device *dev, int blkno, struct buf *bp)
{
	DPRINTF(VFSDB_BIO, ("bread: dev=%x blkno=%d\n", dev, blkno));
	getblk(dev, blkno, bp);

	auto error = rw_buf(bp, 0);
	if (error) {
		DPRINTF(VFSDB_BIO, ("bread: i/o error\n"));
		return error;
	}
	CLR(bp->b_flags, B_INVAL);
	SET(bp->b_flags, (B_READ | B_DONE));
	DPRINTF(VFSDB_BIO, ("bread: done bp=%x\n\n", bp));
	return 0;
}

/*
 * Block write with cache.
 * @buf:   buffer to write.
 *
 * The data is copied to the buffer.
 * Then release the buffer.
 */
int
bwrite(struct buf *bp)
{
	ASSERT(ISSET(bp->b_flags, B_BUSY));
	DPRINTF(VFSDB_BIO, ("bwrite: dev=%x blkno=%d\n", bp->b_dev,
			    bp->b_blkno));
	CLR(bp->b_flags, (B_READ | B_DONE | B_DELWRI));

	auto error = rw_buf(bp, 1);
	if (error)
		return error;
	SET(bp->b_flags, B_DONE);
	return 0;
}

/*
 * Delayed write.
 *
 * The buffer is marked dirty, but an actual I/O is not
 * performed.  This routine should be used when the buffer
 * is expected to be modified again soon.
 */
void
bdwrite(struct buf *bp)
{
}

/*
 * Flush write-behind block
 */
void
bflush(struct buf *bp)
{
}

/*
 * Invalidate buffer for specified device.
 * This is called when unmount.
 */
void
binval(struct device *dev)
{
}

/*
 * Invalidate all buffers.
 * This is called when unmount.
 */
void
bio_sync(void)
{
}

/*
 * Initialize the buffer I/O system.
 */
void
bio_init(void)
{
}
