/*
 * image_kind.h - the device-kind stamp in an image's binary info (WP IMGKIND)
 *
 * Every DSPi image (main.c) and the alink sender image (DSPi-Sender,
 * sender_main.c) carry one BINARY_INFO_ID_AND_STRING entry, tag 'DS', id
 * DSPI_IMAGE_KIND_ID, whose text says what the image is for: "dsp" or
 * "alink_sender".  The console's firmware guard (dspi-core firmware_guard.rs,
 * image_kind) reads it from a UF2 before an install and refuses an image whose
 * kind is not the target's (a DSP image on a sender, a sender image on a DSP),
 * whatever is typed.  Same tag as core_vreg.c's voltage stamp, another id; the
 * sender has no core_vreg.c, so the kind stamp lives here.  Upstream images
 * carry neither.  Change the id or the texts only together with the console's
 * parser.
 */
#ifndef DSPI_IMAGE_KIND_H
#define DSPI_IMAGE_KIND_H

#include "pico/binary_info.h"

#define DSPI_IMAGE_KIND_ID 0x0B7A4B1D
#define DSPI_IMAGE_KIND_DSP "dsp"
#define DSPI_IMAGE_KIND_ALINK_SENDER "alink_sender"

#define DSPI_IMAGE_KIND_STAMP(kind) \
    bi_decl(bi_string(BINARY_INFO_MAKE_TAG('D', 'S'), DSPI_IMAGE_KIND_ID, kind))

#endif
