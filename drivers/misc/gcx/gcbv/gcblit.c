/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2012 Vivante Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * BSD LICENSE
 *
 * Copyright(c) 2012 Vivante Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Vivante Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "gcbv.h"

#define GCZONE_NONE		0
#define GCZONE_ALL		(~0U)
#define GCZONE_BLEND		(1 << 0)
#define GCZONE_SURF		(1 << 1)
<<<<<<< HEAD
#define GCZONE_BLIT		(1 << 3)
=======
#define GCZONE_BLIT		(1 << 2)
>>>>>>> omapzoom/p-android-omap-3.4

GCDBG_FILTERDEF(blit, GCZONE_NONE,
		"blend",
		"surf",
		"blit")


static enum bverror do_blit_end(struct bvbltparams *bvbltparams,
				struct gcbatch *batch)
{
	enum bverror bverror;
	struct gcblit *gcblit;
	struct gcmobltconfig *gcmobltconfig;
	struct gcmostartde *gcmostartde;

	GCENTER(GCZONE_BLIT);

	/* Get a shortcut to the operation specific data. */
	gcblit = &batch->op.blit;

	GCDBG(GCZONE_BLIT, "finalizing the blit, scrcount = %d\n",
	      gcblit->srccount);

	/***********************************************************************
	 * Configure the operation.
	 */

	/* Allocate command buffer. */
	bverror = claim_buffer(bvbltparams, batch,
			       sizeof(struct gcmobltconfig),
			       (void **) &gcmobltconfig);
	if (bverror != BVERR_NONE)
		goto exit;

	/* Configure multi-source control. */
	gcmobltconfig->multisource_ldst = gcmobltconfig_multisource_ldst;
	gcmobltconfig->multisource.raw = 0;
	gcmobltconfig->multisource.reg.srccount = gcblit->srccount - 1;

	GCDBG(GCZONE_BLIT, "blockenable = %d\n", gcblit->blockenable);
	if (gcblit->blockenable) {
		gcmobltconfig->multisource.reg.horblock
			= GCREG_DE_MULTI_SOURCE_HORIZONTAL_BLOCK_PIXEL16;
		gcmobltconfig->multisource.reg.verblock
			= GCREG_DE_MULTI_SOURCE_VERTICAL_BLOCK_LINE64;
	} else {
		gcmobltconfig->multisource.reg.horblock
			= GCREG_DE_MULTI_SOURCE_HORIZONTAL_BLOCK_PIXEL128;
		gcmobltconfig->multisource.reg.verblock
			= GCREG_DE_MULTI_SOURCE_VERTICAL_BLOCK_LINE1;
	}

	/* Set destination configuration. */
	GCDBG(GCZONE_BLIT, "  swizzle code = %d\n", gcblit->swizzle);
	GCDBG(GCZONE_BLIT, "  format code = %d\n", gcblit->format);

	gcmobltconfig->dstconfig_ldst = gcmobltconfig_dstconfig_ldst;
	gcmobltconfig->dstconfig.raw = 0;
	gcmobltconfig->dstconfig.reg.swizzle = gcblit->swizzle;
	gcmobltconfig->dstconfig.reg.format = gcblit->format;
<<<<<<< HEAD
=======
	gcmobltconfig->dstconfig.reg.endian = gcblit->endian;
>>>>>>> omapzoom/p-android-omap-3.4
	gcmobltconfig->dstconfig.reg.command = gcblit->multisrc
		? GCREG_DEST_CONFIG_COMMAND_MULTI_SOURCE_BLT
		: GCREG_DEST_CONFIG_COMMAND_BIT_BLT;

	/***********************************************************************
	 * Start the operation.
	 */

	/* Allocate command buffer. */
	bverror = claim_buffer(bvbltparams, batch,
			       sizeof(struct gcmostartde),
			       (void **) &gcmostartde);
	if (bverror != BVERR_NONE)
		goto exit;

	/* Set START_DE command. */
	gcmostartde->startde.cmd.fld = gcfldstartde;

	/* Set destination rectangle. */
	gcmostartde->rect.left = gcblit->dstrect.left;
	gcmostartde->rect.top = gcblit->dstrect.top;
	gcmostartde->rect.right = gcblit->dstrect.right;
	gcmostartde->rect.bottom = gcblit->dstrect.bottom;

	GCDBG(GCZONE_BLIT, "dstrect = (%d,%d)-(%d,%d)\n",
	      gcmostartde->rect.left, gcmostartde->rect.top,
	      gcmostartde->rect.right, gcmostartde->rect.bottom);

	/* Reset the finalizer. */
	batch->batchend = do_end;

<<<<<<< HEAD
	gc_debug_blt(gcblit->srccount,
		     abs(gcblit->dstrect.right - gcblit->dstrect.left),
		     abs(gcblit->dstrect.bottom - gcblit->dstrect.top));
=======
	gcbv_debug_blt(gcblit->srccount,
		       abs(gcblit->dstrect.right - gcblit->dstrect.left),
		       abs(gcblit->dstrect.bottom - gcblit->dstrect.top));
>>>>>>> omapzoom/p-android-omap-3.4

exit:
	GCEXITARG(GCZONE_BLIT, "bv%s = %d\n",
		  (bverror == BVERR_NONE) ? "result" : "error", bverror);
	return bverror;
}

enum bverror do_blit(struct bvbltparams *bvbltparams,
		     struct gcbatch *batch,
<<<<<<< HEAD
		     struct surfaceinfo *srcinfo)
=======
		     struct gcsurface *srcinfo)
>>>>>>> omapzoom/p-android-omap-3.4
{
	enum bverror bverror = BVERR_NONE;
	struct gccontext *gccontext = get_context();

	struct gcmosrc0 *gcmosrc0;
	struct gcmosrc *gcmosrc;
	struct gcblit *gcblit;

	unsigned int index;
	struct bvbuffmap *dstmap = NULL;
	struct bvbuffmap *srcmap = NULL;

<<<<<<< HEAD
	struct surfaceinfo *dstinfo;
=======
	struct gcsurface *dstinfo;
>>>>>>> omapzoom/p-android-omap-3.4
	int dstshiftX, dstshiftY;
	int dstpixalign, dstbyteshift;
	int dstoffsetX, dstoffsetY;

<<<<<<< HEAD
	int srcshiftX, srcshiftY, srctopedge;
	struct gcrect srcclipped;
	int srcsurfwidth, srcsurfheight;
	unsigned int physwidth, physheight;
	int orthogonal;
	int multisrc;
=======
	int adjust, srcshiftX, srcshiftY;
	unsigned int physwidth, physheight;
	bool orthogonal;
	bool multisrc;
	unsigned int batchfinalize;

	int srcleftedge, srctopedge;
	int dstleftedge, dsttopedge;

	struct gcrect *srcorig, *srcclip, *srcadj;
	struct gcrect *dstorig, *dstclip, *dstadj;
>>>>>>> omapzoom/p-android-omap-3.4

	GCENTER(GCZONE_BLIT);

	/* 3-plane source not supported. */
	if ((srcinfo->format.type == BVFMT_YUV) &&
	    (srcinfo->format.cs.yuv.planecount == 3)) {
		BVSETBLTERROR((srcinfo->index == 0)
					? BVERR_SRC1GEOM_FORMAT
					: BVERR_SRC2GEOM_FORMAT,
			      "unsupported source%d format.",
			      srcinfo->index + 1);
		goto exit;
	}

<<<<<<< HEAD
	/* Get a shortcut to the destination surface. */
	dstinfo = &batch->dstinfo;

=======
	/* Zero-fill for source is not supported. */
	if (srcinfo->format.zerofill) {
		BVSETBLTERROR((srcinfo->index == 0)
					? BVERR_SRC1GEOM_FORMAT
					: BVERR_SRC2GEOM_FORMAT,
			      "0 filling is not supported.");
		goto exit;
	}

	/***********************************************************************
	 * Parse destination.
	 */

	/* Get a shortcut to the destination surface. */
	dstinfo = &batch->dstinfo;

	/* Get destination rectangle shortcuts. */
	dstorig = &dstinfo->rect.orig;
	dstclip = &dstinfo->rect.clip;
	dstadj = &dstinfo->rect.adj;

>>>>>>> omapzoom/p-android-omap-3.4
	/* Parse destination parameters. */
	bverror = parse_destination(bvbltparams, batch);
	if (bverror != BVERR_NONE)
		goto exit;

<<<<<<< HEAD
	/* Setup rotation. */
	process_dest_rotation(bvbltparams, batch);
=======
	/* Ignore the blit if destination rectangle is empty. */
	if (null_rect(dstclip)) {
		GCDBG(GCZONE_SURF, "empty destination rectangle.\n");
		goto exit;
	}

	/* Adjust surface angles if necessary. */
	adjust_angle(srcinfo, dstinfo);
>>>>>>> omapzoom/p-android-omap-3.4


	/***********************************************************************
	 * Determine source surface alignment offset.
	 */

<<<<<<< HEAD
=======
	/* Assume multi-source is possible. */
	multisrc = true;

	/* Assume no additional shift is necessary. */
	srcshiftX = 0;
	srcshiftY = 0;

	/* Get source rectangle shortcuts. */
	srcorig = &srcinfo->rect.orig;
	srcclip = &srcinfo->rect.clip;
	srcadj = &srcinfo->rect.adj;

>>>>>>> omapzoom/p-android-omap-3.4
	/* Determine whether the source and the destination are orthogonal
	 * to each other. */
	orthogonal = (srcinfo->angle % 2) != (dstinfo->angle % 2);

	/* Compute clipped source rectangle. */
<<<<<<< HEAD
	srcclipped.left   = srcinfo->rect.left   + batch->clipdelta.left;
	srcclipped.top    = srcinfo->rect.top    + batch->clipdelta.top;
	srcclipped.right  = srcinfo->rect.right  + batch->clipdelta.right;
	srcclipped.bottom = srcinfo->rect.bottom + batch->clipdelta.bottom;
	GCPRINT_RECT(GCZONE_SURF, "clipped source", &srcclipped);

	/* Validate the source rectangle. */
	if (!valid_rect(srcinfo->geom, &srcclipped)) {
=======
	srcclip->left   = srcorig->left   + batch->clipdelta.left;
	srcclip->top    = srcorig->top    + batch->clipdelta.top;
	srcclip->right  = srcorig->right  + batch->clipdelta.right;
	srcclip->bottom = srcorig->bottom + batch->clipdelta.bottom;
	GCPRINT_RECT(GCZONE_SURF, "clipped source", srcclip);

	/* Validate the source rectangle. */
	if (!valid_rect(srcinfo, srcclip)) {
>>>>>>> omapzoom/p-android-omap-3.4
		BVSETBLTERROR((srcinfo->index == 0)
					? BVERR_SRC1RECT
					: BVERR_SRC2RECT,
			      "invalid source rectangle.");
		goto exit;
	}

<<<<<<< HEAD
	/* Compute the source surface shift. */
	switch (srcinfo->angle) {
	case ROT_ANGLE_0:
		srctopedge = srcclipped.top;
		srcshiftX = srcclipped.left - batch->dstadjusted.left;
		srcshiftY = srctopedge - batch->dstadjusted.top;
		break;

	case ROT_ANGLE_90:
		srctopedge = srcinfo->geom->width - srcclipped.left;
		srcshiftX = srcclipped.top - batch->dstadjusted.top;
		srcshiftY = srctopedge
			  - (batch->dstwidth - batch->dstadjusted.left);
		srctopedge += 1;
		break;

	case ROT_ANGLE_180:
		srctopedge = srcinfo->geom->height - srcclipped.top;
		srcshiftX = (srcinfo->geom->width - srcclipped.left)
			  - (batch->dstwidth - batch->dstadjusted.left);
		srcshiftY = srctopedge
			  - (batch->dstheight - batch->dstadjusted.top);
		srctopedge += 1;
		break;

	case ROT_ANGLE_270:
		srctopedge = srcclipped.left;
		srcshiftX = (srcinfo->geom->height - srcclipped.top)
			  - (batch->dstheight - batch->dstadjusted.top);
		srcshiftY = srctopedge - batch->dstadjusted.left;
		break;

	default:
		srctopedge = 0;
		srcshiftX = 0;
		srcshiftY = 0;
	}

	/* We cannot be in the middle of a sample, currently only YUV formats
	 * can have subsamples. Adjust vertical position as necessary.
	 * Horizontal position will be adjusted based on the byte offset and
	 * base address alignment requirement. This assumes that if we are
	 * aligned on the base address, then we are also aligned at the
	 * beginning of a sample. */
	if (srcinfo->format.type == BVFMT_YUV) {
		int mody = (srctopedge + srcshiftY)
			 % srcinfo->format.cs.yuv.ysample;

		if (mody < 0)
			mody = srcinfo->format.cs.yuv.ysample + mody;

		srcshiftY -= mody;
		srcinfo->ypixalign = -mody;
	} else {
		srcinfo->ypixalign = 0;
	}

	/* Compute the source surface offset in bytes. */
	srcinfo->bytealign = srcshiftY * (int) srcinfo->geom->virtstride
			   + srcshiftX * (int) srcinfo->format.bitspp / 8;

	/* Compute the source offset in pixels needed to compensate
	 * for the surface base address misalignment if any. */
	srcinfo->xpixalign = get_pixel_offset(srcinfo, srcinfo->bytealign);

	GCDBG(GCZONE_SURF, "source surface %d:\n", srcinfo->index + 1);
	GCDBG(GCZONE_SURF, "  surface offset (pixels) = %d,%d\n",
	      srcshiftX, srcshiftY);
	GCDBG(GCZONE_SURF, "  surface offset (bytes) = 0x%08X\n",
	      srcinfo->bytealign);
	GCDBG(GCZONE_SURF, "  srcpixalign = %d,%d\n",
	      srcinfo->xpixalign, srcinfo->ypixalign);

	/* Apply the source alignment. */
	srcinfo->bytealign += srcinfo->xpixalign
			   * (int) srcinfo->format.bitspp / 8;
	srcshiftX += srcinfo->xpixalign;

	/* NOTE: at this point the source is ready to be presented,
	 * srcinfo->xpixalign and srcinfo->ypixalign represent additional
	 * adjustments for the DESTINATION. */

	GCDBG(GCZONE_SURF, "  adjusted surface offset (pixels) = %d,%d\n",
	      srcshiftX, srcshiftY);
	GCDBG(GCZONE_SURF, "  adjusted surface offset (bytes) = 0x%08X\n",
	      srcinfo->bytealign);
=======
	/* Ignore the blit if source rectangle is empty. */
	if (null_rect(srcclip)) {
		GCDBG(GCZONE_SURF, "empty source rectangle.\n");
		goto exit;
	}

	/* Determine source and destination physical origin. */
	switch (srcinfo->angle) {
	case ROT_ANGLE_0:
		srcleftedge = srcclip->left;
		srctopedge  = srcclip->top;
		dstleftedge = dstadj->left;
		dsttopedge  = dstadj->top;
		break;

	case ROT_ANGLE_90:
		srcleftedge = srcclip->top;
		srctopedge  = srcinfo->width - srcclip->left;
		dstleftedge = dstadj->top;
		dsttopedge  = dstinfo->adjwidth - dstadj->left;
		break;

	case ROT_ANGLE_180:
		srcleftedge = srcinfo->width - srcclip->left;
		srctopedge  = srcinfo->height - srcclip->top;
		dstleftedge = dstinfo->adjwidth - dstadj->left;
		dsttopedge  = dstinfo->adjheight - dstadj->top;
		break;

	case ROT_ANGLE_270:
		srcleftedge = srcinfo->height - srcclip->top;
		srctopedge  = srcclip->left;
		dstleftedge = dstinfo->adjheight - dstadj->top;
		dsttopedge  = dstadj->left;
		break;

	default:
		srcleftedge = 0;
		srctopedge  = 0;
		dstleftedge = 0;
		dsttopedge  = 0;
	}

	/* Compute the source surface shift. */
	srcinfo->xpixalign = srcleftedge - dstleftedge;
	srcinfo->ypixalign = srctopedge  - dsttopedge;

	/* Compute the source surface offset in bytes. */
	srcinfo->bytealign1
		= srcinfo->ypixalign * (int) srcinfo->stride1
		+ srcinfo->xpixalign * (int) srcinfo->format.bitspp / 8;

	GCDBG(GCZONE_SURF, "source surface %d:\n", srcinfo->index + 1);
	GCDBG(GCZONE_SURF, "  surface offset (pixels) = %d,%d\n",
	      srcinfo->xpixalign, srcinfo->ypixalign);
	GCDBG(GCZONE_SURF, "  surface offset (bytes) = 0x%08X\n",
	      srcinfo->bytealign1);

	/* Compute the source offset in pixels needed to compensate
	 * for the surface base address misalignment if any. */
	adjust = get_pixel_offset(srcinfo, srcinfo->bytealign1);

	/* Account for the newly created misalignment if any. */
	srcinfo->bytealign1 += adjust * (int) srcinfo->format.bitspp / 8;
	srcinfo->xpixalign += adjust;
	srcshiftX += adjust;

	GCDBG(GCZONE_SURF, "  horizontal alignment adjustment (pixels) = %d\n",
	      adjust);
	GCDBG(GCZONE_SURF, "  adjusted surface offset (pixels) = %d,%d\n",
	      srcinfo->xpixalign, srcinfo->ypixalign);
	GCDBG(GCZONE_SURF, "  additional surface offset (pixels) = %d,%d\n",
	      srcshiftX, srcshiftY);
	GCDBG(GCZONE_SURF, "  adjusted surface offset (bytes) = 0x%08X\n",
	      srcinfo->bytealign1);
>>>>>>> omapzoom/p-android-omap-3.4

	/* Compute U/V plane offsets. */
	if ((srcinfo->format.type == BVFMT_YUV) &&
	    (srcinfo->format.cs.yuv.planecount > 1))
<<<<<<< HEAD
		set_computeyuv(srcinfo, srcshiftX, srcshiftY);
=======
		set_computeyuv(srcinfo, srcinfo->xpixalign, srcinfo->ypixalign);
>>>>>>> omapzoom/p-android-omap-3.4

	/* Set precomputed destination adjustments based on the destination
	 * base address misalignment only. */
	dstshiftX = dstinfo->xpixalign;
	dstshiftY = dstinfo->ypixalign;

	/* Apply source adjustemnts. */
	if (srcinfo->angle == dstinfo->angle) {
<<<<<<< HEAD
		dstshiftX += srcinfo->xpixalign;
		dstshiftY += srcinfo->ypixalign;
	} else if (((srcinfo->angle + 3) % 4) == dstinfo->angle) {
		dstshiftY += srcinfo->xpixalign;
	} else if (((srcinfo->angle + 1) % 4) == dstinfo->angle) {
		dstshiftX += srcinfo->ypixalign;
	}

	/* Compute the destination surface offset in bytes. */
	dstbyteshift = dstshiftY * (int) dstinfo->geom->virtstride
=======
		dstshiftX += srcshiftX;
		dstshiftY += srcshiftY;
	} else if (((srcinfo->angle + 3) % 4) == dstinfo->angle) {
		dstshiftY += srcshiftX;
	} else if (((srcinfo->angle + 1) % 4) == dstinfo->angle) {
		dstshiftX += srcshiftY;
	}

	/* Compute the destination surface offset in bytes. */
	dstbyteshift = dstshiftY * (int) dstinfo->stride1
>>>>>>> omapzoom/p-android-omap-3.4
		     + dstshiftX * (int) dstinfo->format.bitspp / 8;

	/* Compute the destination offset in pixels needed to compensate
	 * for the surface base address misalignment if any. If dstpixalign
	 * comes out anything other than zero, multisource blit cannot be
	 * performed. */
	dstpixalign = get_pixel_offset(dstinfo, dstbyteshift);
<<<<<<< HEAD
=======
	if (dstpixalign != 0) {
		GCDBG(GCZONE_SURF,
		      "  disabling multi-source, "
		      "destination needs to be realigned again.\n");
		multisrc = false;
	}
>>>>>>> omapzoom/p-android-omap-3.4

	GCDBG(GCZONE_SURF, "destination surface:\n");
	GCDBG(GCZONE_SURF, "  surface offset (pixels) = %d,%d\n",
	      dstshiftX, dstshiftY);
	GCDBG(GCZONE_SURF, "  surface offset (bytes) = 0x%08X\n",
	      dstbyteshift);
	GCDBG(GCZONE_SURF, "  realignment = %d\n",
	      dstpixalign);

<<<<<<< HEAD
	if ((dstpixalign != 0) ||
	    ((srcinfo->xpixalign != 0) && (srcinfo->angle == dstinfo->angle))) {
		/* Adjust the destination to match the source geometry. */
		switch (srcinfo->angle) {
		case ROT_ANGLE_0:
			/* Adjust coordinates. */
			srcclipped.left -= srcshiftX;
			srcclipped.top  -= srcshiftY;

			/* Determine source size. */
			srcsurfwidth = srcinfo->geom->width
				     - srcinfo->xpixalign;
			srcsurfheight = srcinfo->geom->height;
			break;

		case ROT_ANGLE_90:
			/* Adjust top coordinate. */
			srcclipped.top -= srcshiftX;

			/* Determine source size. */
			srcsurfwidth = srcinfo->geom->height
				     - srcinfo->xpixalign;
			srcsurfheight = srcinfo->geom->width;
			break;

		case ROT_ANGLE_180:
			/* Determine source size. */
			srcsurfwidth = srcinfo->geom->width
				     - srcinfo->xpixalign;
			srcsurfheight = srcinfo->geom->height;
			break;

		case ROT_ANGLE_270:
			/* Adjust coordinates. */
			srcclipped.left -= srcshiftY;

			/* Determine source size. */
			srcsurfwidth = srcinfo->geom->height
				     - srcinfo->xpixalign;
			srcsurfheight = srcinfo->geom->width;
			break;

		default:
			srcsurfwidth = 0;
			srcsurfheight = 0;
		}

		GCDBG(GCZONE_SURF, "srcrect origin = %d,%d\n",
		      srcclipped.left, srcclipped.top);
		GCDBG(GCZONE_SURF, "source physical size = %dx%d\n",
		      srcsurfwidth, srcsurfheight);

		/* Overwrite destination byte offset. */
		dstbyteshift = dstinfo->bytealign;

		/* No adjustment necessary for single-source. */
		dstoffsetX = 0;
		dstoffsetY = 0;

		/* Set the physical destination size. */
		physwidth = dstinfo->physwidth;
		physheight = dstinfo->physheight;

		/* Disable multi source for the cases where the destination
		 * and the source address alignments do not match. */
		multisrc = 0;
		GCDBG(GCZONE_SURF, "multi-source disabled.\n");
	} else {
		/* Source origin is not used in multi-source setup. */
		srcclipped.left = 0;
		srcclipped.top = 0;
=======
	if (multisrc) {
		GCDBG(GCZONE_SURF, "multi-source enabled.\n");

		/* Source origin is not used in multi-source setup. */
		srcadj->left = 0;
		srcadj->top = 0;

		/* Set new surface shift. */
		if (dstinfo->bytealign1 != dstbyteshift) {
			GCDBG(GCZONE_SURF,
			      "destination alignment changed.\n");
			dstinfo->bytealign1 = dstbyteshift;
			dstinfo->surfdirty = true;
		}
>>>>>>> omapzoom/p-android-omap-3.4

		/* Adjust the destination to match the source geometry. */
		switch (srcinfo->angle) {
		case ROT_ANGLE_0:
			/* Adjust the destination horizontally. */
<<<<<<< HEAD
			dstoffsetX = srcinfo->xpixalign;
			dstoffsetY = srcinfo->ypixalign;

			/* Apply the source alignment. */
			if ((dstinfo->angle % 2) == 0) {
				physwidth  = dstinfo->physwidth
					   - srcinfo->xpixalign;
				physheight = dstinfo->physheight
					   - srcinfo->ypixalign;
			} else {
				physwidth  = dstinfo->physwidth
					   - srcinfo->ypixalign;
				physheight = dstinfo->physheight
					   - srcinfo->xpixalign;
=======
			dstoffsetX = srcshiftX;
			dstoffsetY = srcshiftY;

			/* Apply the source alignment. */
			if ((dstinfo->angle % 2) == 0) {
				physwidth  = dstinfo->physwidth  - srcshiftX;
				physheight = dstinfo->physheight - srcshiftY;
			} else {
				physwidth  = dstinfo->physwidth  - srcshiftY;
				physheight = dstinfo->physheight - srcshiftX;
>>>>>>> omapzoom/p-android-omap-3.4
			}
			break;

		case ROT_ANGLE_90:
			/* Adjust the destination vertically. */
<<<<<<< HEAD
			dstoffsetX = srcinfo->ypixalign;
			dstoffsetY = srcinfo->xpixalign;

			/* Apply the source alignment. */
			if ((dstinfo->angle % 2) == 0) {
				physwidth  = dstinfo->physwidth
					   - srcinfo->ypixalign;
				physheight = dstinfo->physheight
					   - srcinfo->xpixalign;
			} else {
				physwidth  = dstinfo->physwidth
					   - srcinfo->xpixalign;
				physheight = dstinfo->physheight
					   - srcinfo->ypixalign;
=======
			dstoffsetX = srcshiftY;
			dstoffsetY = srcshiftX;

			/* Apply the source alignment. */
			if ((dstinfo->angle % 2) == 0) {
				physwidth  = dstinfo->physwidth  - srcshiftY;
				physheight = dstinfo->physheight - srcshiftX;
			} else {
				physwidth  = dstinfo->physwidth  - srcshiftX;
				physheight = dstinfo->physheight - srcshiftY;
>>>>>>> omapzoom/p-android-omap-3.4
			}
			break;

		case ROT_ANGLE_180:
			/* No adjustment necessary. */
			dstoffsetX = 0;
			dstoffsetY = 0;

			/* Apply the source alignment. */
			if ((dstinfo->angle % 2) == 0) {
<<<<<<< HEAD
				physwidth  = dstinfo->physwidth
					   - srcinfo->xpixalign;
				physheight = dstinfo->physheight
					   - srcinfo->ypixalign;
			} else {
				physwidth  = dstinfo->physwidth
					   - srcinfo->ypixalign;
				physheight = dstinfo->physheight
					   - srcinfo->xpixalign;
=======
				physwidth  = dstinfo->physwidth  - srcshiftX;
				physheight = dstinfo->physheight - srcshiftY;
			} else {
				physwidth  = dstinfo->physwidth  - srcshiftY;
				physheight = dstinfo->physheight - srcshiftX;
>>>>>>> omapzoom/p-android-omap-3.4
			}
			break;

		case ROT_ANGLE_270:
			/* No adjustment necessary. */
			dstoffsetX = 0;
			dstoffsetY = 0;

			/* Apply the source alignment. */
			if ((dstinfo->angle % 2) == 0) {
<<<<<<< HEAD
				physwidth  = dstinfo->physwidth
					   - srcinfo->ypixalign;
				physheight = dstinfo->physheight
					   - srcinfo->xpixalign;
			} else {
				physwidth  = dstinfo->physwidth
					   - srcinfo->xpixalign;
				physheight = dstinfo->physheight
					   - srcinfo->ypixalign;
=======
				physwidth  = dstinfo->physwidth  - srcshiftY;
				physheight = dstinfo->physheight - srcshiftX;
			} else {
				physwidth  = dstinfo->physwidth  - srcshiftX;
				physheight = dstinfo->physheight - srcshiftY;
>>>>>>> omapzoom/p-android-omap-3.4
			}
			break;

		default:
			physwidth = 0;
			physheight = 0;
			dstoffsetX = 0;
			dstoffsetY = 0;
		}

		/* Source geometry is now the same as the destination. */
		if (orthogonal) {
<<<<<<< HEAD
			srcsurfwidth = physheight;
			srcsurfheight = physwidth;
		} else {
			srcsurfwidth = physwidth;
			srcsurfheight = physheight;
		}

		/* Enable multi-source. */
		multisrc = 1;
		GCDBG(GCZONE_SURF, "multi-source enabled.\n");
	}

	/* Misaligned source may cause the destination parameters
	 * to change, verify whether this has happened. */
	if ((batch->dstbyteshift != dstbyteshift) ||
	    (batch->dstphyswidth != physwidth) ||
	    (batch->dstphysheight != physheight) ||
	    (batch->dstoffsetX != dstoffsetX) ||
	    (batch->dstoffsetY != dstoffsetY)) {
		/* Set new values. */
		batch->dstbyteshift = dstbyteshift;
		batch->dstphyswidth = physwidth;
		batch->dstphysheight = physheight;
		batch->dstoffsetX = dstoffsetX;
		batch->dstoffsetY = dstoffsetY;

		/* Now we need to end the current batch and program
		 * the hardware with the new destination. */
		batch->batchflags |= BVBATCH_DST;
	}

	/* Check if we need to finalize existing batch. */
	if ((batch->batchend != do_blit_end) ||
	    (batch->op.blit.srccount == 4) ||
	    (batch->op.blit.multisrc == 0) ||
	    (multisrc == 0) ||
	    ((batch->batchflags & (BVBATCH_DST |
				   BVBATCH_CLIPRECT |
				   BVBATCH_DESTRECT)) != 0)) {
=======
			srcinfo->physwidth  = physheight;
			srcinfo->physheight = physwidth;
		} else {
			srcinfo->physwidth  = physwidth;
			srcinfo->physheight = physheight;
		}

		/* Set new surface size. */
		if ((physwidth  != dstinfo->physwidth) ||
		    (physheight != dstinfo->physheight)) {
			dstinfo->physwidth  = physwidth;
			dstinfo->physheight = physheight;
			dstinfo->surfdirty = true;
		}

		/* Set new offset. */
		if ((batch->dstoffsetX != dstoffsetX) ||
		    (batch->dstoffsetY != dstoffsetY)) {
			batch->dstoffsetX = dstoffsetX;
			batch->dstoffsetY = dstoffsetY;
			dstinfo->surfdirty = true;
		}
	} else {
		GCDBG(GCZONE_SURF, "multi-source disabled.\n");

		/* Determine surface size and render rectangle. */
		process_rotation(srcinfo);

		/* No adjustment necessary for single-source. */
		dstoffsetX = 0;
		dstoffsetY = 0;
	}

	batchfinalize = 0;

	/* Reached maximum number of sources? */
	if (batch->op.blit.srccount == gccontext->gccaps.maxsource)
		batchfinalize |= GCBV_BATCH_FINALIZE_SRCCOUNT;

	/* Previous operation was not blit? */
	if (batch->batchend != do_blit_end)
		batchfinalize |= GCBV_BATCH_FINALIZE_OPERATION;

	/* Previous blit was not multi-sourced? */
	else if (!batch->op.blit.multisrc)
		batchfinalize |= GCBV_BATCH_FINALIZE_MULTISRC;

	/* Current blit is not multi-sourced? */
	if (!multisrc)
		batchfinalize |= GCBV_BATCH_FINALIZE_ALIGN;

	/* Destination has changed? */
	if (dstinfo->surfdirty)
		batchfinalize |= GCBV_BATCH_FINALIZE_FLAGS_DST;

	if (dstinfo->cliprectdirty)
		batchfinalize |= GCBV_BATCH_FINALIZE_FLAGS_CLIPRECT;

	if (dstinfo->destrectdirty)
		batchfinalize |= GCBV_BATCH_FINALIZE_FLAGS_DESTRECT;

	/* Check if we need to finalize existing batch. */
	if (batchfinalize) {
		if (batch->batchend == do_blit_end)
			gcbv_debug_finalize_batch(batchfinalize);

>>>>>>> omapzoom/p-android-omap-3.4
		/* Finalize existing batch if any. */
		bverror = batch->batchend(bvbltparams, batch);
		if (bverror != BVERR_NONE)
			goto exit;

		/* Blit batch. */
		batch->batchend = do_blit_end;

		/* Initialize the new batch. */
		gcblit = &batch->op.blit;
<<<<<<< HEAD
		gcblit->blockenable = 0;
=======
		gcblit->blockenable = false;
>>>>>>> omapzoom/p-android-omap-3.4
		gcblit->srccount = 0;
		gcblit->multisrc = multisrc;

		/* Set the destination format. */
		gcblit->format  = dstinfo->format.format;
		gcblit->swizzle = dstinfo->format.swizzle;
<<<<<<< HEAD

		/* Set the destination coordinates. */
		gcblit->dstrect.left   = batch->dstadjusted.left   - dstoffsetX;
		gcblit->dstrect.top    = batch->dstadjusted.top    - dstoffsetY;
		gcblit->dstrect.right  = batch->dstadjusted.right  - dstoffsetX;
		gcblit->dstrect.bottom = batch->dstadjusted.bottom - dstoffsetY;

=======
		gcblit->endian = dstinfo->format.endian;

		/* Set the destination coordinates. */
		gcblit->dstrect.left   = dstadj->left   - dstoffsetX;
		gcblit->dstrect.top    = dstadj->top    - dstoffsetY;
		gcblit->dstrect.right  = dstadj->right  - dstoffsetX;
		gcblit->dstrect.bottom = dstadj->bottom - dstoffsetY;
	}

	if (dstinfo->surfdirty) {
>>>>>>> omapzoom/p-android-omap-3.4
		/* Map the destination. */
		bverror = do_map(dstinfo->buf.desc, batch, &dstmap);
		if (bverror != BVERR_NONE) {
			bvbltparams->errdesc = gccontext->bverrorstr;
			goto exit;
		}

		/* Set the new destination. */
		bverror = set_dst(bvbltparams, batch, dstmap);
		if (bverror != BVERR_NONE)
			goto exit;
<<<<<<< HEAD

		/* Reset the modified flag. */
		batch->batchflags &= ~(BVBATCH_DST |
				       BVBATCH_CLIPRECT |
				       BVBATCH_DESTRECT);
=======
>>>>>>> omapzoom/p-android-omap-3.4
	}

	/* Map the source. */
	bverror = do_map(srcinfo->buf.desc, batch, &srcmap);
	if (bverror != BVERR_NONE) {
		bvbltparams->errdesc = gccontext->bverrorstr;
		goto exit;
	}

	/***********************************************************************
	** Configure source.
	*/

	/* We need to walk in blocks if the source and the destination
	 * surfaces are orthogonal to each other. */
<<<<<<< HEAD
	batch->op.blit.blockenable |= orthogonal;
=======
	if (orthogonal)
		batch->op.blit.blockenable = true;
>>>>>>> omapzoom/p-android-omap-3.4

	/* Shortcut to the register index. */
	index = batch->op.blit.srccount;

	/* Set surface parameters. */
	if (index == 0) {
		/* Allocate command buffer. */
		bverror = claim_buffer(bvbltparams, batch,
				       sizeof(struct gcmosrc0),
				       (void **) &gcmosrc0);
		if (bverror != BVERR_NONE)
			goto exit;

		add_fixup(bvbltparams, batch, &gcmosrc0->address,
<<<<<<< HEAD
			  srcinfo->bytealign);

		gcmosrc0->config_ldst = gcmosrc0_config_ldst;
		gcmosrc0->address = GET_MAP_HANDLE(srcmap);
		gcmosrc0->stride = srcinfo->geom->virtstride;
		gcmosrc0->rotation.raw = 0;
		gcmosrc0->rotation.reg.surf_width = srcsurfwidth;
		gcmosrc0->config.raw = 0;
		gcmosrc0->config.reg.swizzle = srcinfo->format.swizzle;
		gcmosrc0->config.reg.format = srcinfo->format.format;
		gcmosrc0->origin.reg.x = srcclipped.left;
		gcmosrc0->origin.reg.y = srcclipped.top;
		gcmosrc0->size.reg = gcregsrcsize_max;

		gcmosrc0->rotation_ldst = gcmosrc0_rotation_ldst;
		gcmosrc0->rotationheight.reg.height = srcsurfheight;
=======
			  srcinfo->bytealign1);

		gcmosrc0->config_ldst = gcmosrc0_config_ldst;
		gcmosrc0->address = GET_MAP_HANDLE(srcmap);
		gcmosrc0->stride = srcinfo->stride1;
		gcmosrc0->rotation.raw = 0;
		gcmosrc0->rotation.reg.surf_width = srcinfo->physwidth;
		gcmosrc0->config.raw = 0;
		gcmosrc0->config.reg.swizzle = srcinfo->format.swizzle;
		gcmosrc0->config.reg.format = srcinfo->format.format;
		gcmosrc0->config.reg.endian = srcinfo->format.endian;
		gcmosrc0->origin.reg.x = srcadj->left;
		gcmosrc0->origin.reg.y = srcadj->top;
		gcmosrc0->size.reg = gcregsrcsize_max;

		gcmosrc0->rotation_ldst = gcmosrc0_rotation_ldst;
		gcmosrc0->rotationheight.reg.height = srcinfo->physheight;
>>>>>>> omapzoom/p-android-omap-3.4
		gcmosrc0->rotationangle.raw = 0;
		gcmosrc0->rotationangle.reg.src = rotencoding[srcinfo->angle];
		gcmosrc0->rotationangle.reg.dst = rotencoding[dstinfo->angle];
		gcmosrc0->rotationangle.reg.src_mirror = srcinfo->mirror;
		gcmosrc0->rotationangle.reg.dst_mirror = GCREG_MIRROR_NONE;

		gcmosrc0->rop_ldst = gcmosrc0_rop_ldst;
		gcmosrc0->rop.raw = 0;
		gcmosrc0->rop.reg.type = GCREG_ROP_TYPE_ROP3;
		gcmosrc0->rop.reg.fg = (unsigned char) srcinfo->rop;

		gcmosrc0->mult_ldst = gcmosrc0_mult_ldst;
		gcmosrc0->mult.raw = 0;
<<<<<<< HEAD
		gcmosrc0->mult.reg.srcglobalpremul
		= GCREG_COLOR_MULTIPLY_MODES_SRC_GLOBAL_PREMULTIPLY_DISABLE;
=======
		gcmosrc0->mult.reg.srcglobalpremul = srcinfo->srcglobalpremul;
>>>>>>> omapzoom/p-android-omap-3.4

		if (srcinfo->format.premultiplied)
			gcmosrc0->mult.reg.srcpremul
			= GCREG_COLOR_MULTIPLY_MODES_SRC_PREMULTIPLY_DISABLE;
		else
			gcmosrc0->mult.reg.srcpremul
			= GCREG_COLOR_MULTIPLY_MODES_SRC_PREMULTIPLY_ENABLE;

		if (dstinfo->format.premultiplied) {
			gcmosrc0->mult.reg.dstpremul
			= GCREG_COLOR_MULTIPLY_MODES_SRC_PREMULTIPLY_DISABLE;

			gcmosrc0->mult.reg.dstdemul
			= GCREG_COLOR_MULTIPLY_MODES_DST_DEMULTIPLY_DISABLE;
		} else {
			gcmosrc0->mult.reg.dstpremul
			= GCREG_COLOR_MULTIPLY_MODES_SRC_PREMULTIPLY_ENABLE;

			gcmosrc0->mult.reg.dstdemul
			= GCREG_COLOR_MULTIPLY_MODES_DST_DEMULTIPLY_ENABLE;
		}

		/* Program blending. */
		bverror = set_blending(bvbltparams, batch, srcinfo);
		if (bverror != BVERR_NONE)
			goto exit;

		/* Program YUV source. */
		if (srcinfo->format.type == BVFMT_YUV) {
			bverror = set_yuvsrc(bvbltparams, batch,
					     srcinfo, srcmap);
			if (bverror != BVERR_NONE)
				goto exit;
		}
	} else {
		/* Allocate command buffer. */
		bverror = claim_buffer(bvbltparams, batch,
				       sizeof(struct gcmosrc),
				       (void **) &gcmosrc);
		if (bverror != BVERR_NONE)
			goto exit;

		add_fixup(bvbltparams, batch, &gcmosrc->address,
<<<<<<< HEAD
			  srcinfo->bytealign);
=======
			  srcinfo->bytealign1);
>>>>>>> omapzoom/p-android-omap-3.4

		gcmosrc->address_ldst = gcmosrc_address_ldst[index];
		gcmosrc->address = GET_MAP_HANDLE(srcmap);
		gcmosrc->stride_ldst = gcmosrc_stride_ldst[index];
<<<<<<< HEAD
		gcmosrc->stride = srcinfo->geom->virtstride;

		gcmosrc->rotation_ldst = gcmosrc_rotation_ldst[index];
		gcmosrc->rotation.raw = 0;
		gcmosrc->rotation.reg.surf_width = srcsurfwidth;
=======
		gcmosrc->stride = srcinfo->stride1;

		gcmosrc->rotation_ldst = gcmosrc_rotation_ldst[index];
		gcmosrc->rotation.raw = 0;
		gcmosrc->rotation.reg.surf_width = srcinfo->physwidth;
>>>>>>> omapzoom/p-android-omap-3.4

		gcmosrc->config_ldst = gcmosrc_config_ldst[index];
		gcmosrc->config.raw = 0;
		gcmosrc->config.reg.swizzle = srcinfo->format.swizzle;
		gcmosrc->config.reg.format = srcinfo->format.format;
<<<<<<< HEAD

		gcmosrc->origin_ldst = gcmosrc_origin_ldst[index];
		gcmosrc->origin.reg.x = srcclipped.left;
		gcmosrc->origin.reg.y = srcclipped.top;
=======
		gcmosrc->config.reg.endian = srcinfo->format.endian;

		gcmosrc->origin_ldst = gcmosrc_origin_ldst[index];
		gcmosrc->origin.reg.x = srcadj->left;
		gcmosrc->origin.reg.y = srcadj->top;
>>>>>>> omapzoom/p-android-omap-3.4

		gcmosrc->size_ldst = gcmosrc_size_ldst[index];
		gcmosrc->size.reg = gcregsrcsize_max;

		gcmosrc->rotationheight_ldst
			= gcmosrc_rotationheight_ldst[index];
<<<<<<< HEAD
		gcmosrc->rotationheight.reg.height = srcsurfheight;
=======
		gcmosrc->rotationheight.reg.height = srcinfo->physheight;
>>>>>>> omapzoom/p-android-omap-3.4

		gcmosrc->rotationangle_ldst
			= gcmosrc_rotationangle_ldst[index];
		gcmosrc->rotationangle.raw = 0;
		gcmosrc->rotationangle.reg.src = rotencoding[srcinfo->angle];
		gcmosrc->rotationangle.reg.dst = rotencoding[dstinfo->angle];
		gcmosrc->rotationangle.reg.src_mirror = srcinfo->mirror;
		gcmosrc->rotationangle.reg.dst_mirror = GCREG_MIRROR_NONE;

		gcmosrc->rop_ldst = gcmosrc_rop_ldst[index];
		gcmosrc->rop.raw = 0;
		gcmosrc->rop.reg.type = GCREG_ROP_TYPE_ROP3;
		gcmosrc->rop.reg.fg = (unsigned char) srcinfo->rop;

		gcmosrc->mult_ldst = gcmosrc_mult_ldst[index];
		gcmosrc->mult.raw = 0;
<<<<<<< HEAD
		gcmosrc->mult.reg.srcglobalpremul
		= GCREG_COLOR_MULTIPLY_MODES_SRC_GLOBAL_PREMULTIPLY_DISABLE;
=======
		gcmosrc->mult.reg.srcglobalpremul = srcinfo->srcglobalpremul;
>>>>>>> omapzoom/p-android-omap-3.4

		if (srcinfo->format.premultiplied)
			gcmosrc->mult.reg.srcpremul
			= GCREG_COLOR_MULTIPLY_MODES_SRC_PREMULTIPLY_DISABLE;
		else
			gcmosrc->mult.reg.srcpremul
			= GCREG_COLOR_MULTIPLY_MODES_SRC_PREMULTIPLY_ENABLE;

		if (dstinfo->format.premultiplied) {
			gcmosrc->mult.reg.dstpremul
			= GCREG_COLOR_MULTIPLY_MODES_SRC_PREMULTIPLY_DISABLE;

			gcmosrc->mult.reg.dstdemul
			= GCREG_COLOR_MULTIPLY_MODES_DST_DEMULTIPLY_DISABLE;
		} else {
			gcmosrc->mult.reg.dstpremul
			= GCREG_COLOR_MULTIPLY_MODES_SRC_PREMULTIPLY_ENABLE;

			gcmosrc->mult.reg.dstdemul
			= GCREG_COLOR_MULTIPLY_MODES_DST_DEMULTIPLY_ENABLE;
		}

		/* Program blending. */
		bverror = set_blending_index(bvbltparams, batch,
					     srcinfo, index);
		if (bverror != BVERR_NONE)
			goto exit;

		/* Program YUV source. */
		if (srcinfo->format.type == BVFMT_YUV) {
			bverror = set_yuvsrc_index(bvbltparams, batch,
						   srcinfo, srcmap, index);
			if (bverror != BVERR_NONE)
				goto exit;
		}
	}

	batch->op.blit.srccount += 1;

exit:
	GCEXITARG(GCZONE_BLIT, "bv%s = %d\n",
		  (bverror == BVERR_NONE) ? "result" : "error", bverror);
	return bverror;
}
