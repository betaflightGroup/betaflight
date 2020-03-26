/*-
 * Copyright (c) 2020 Dominic Clifton
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
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
 * SP Racing Pixel OSD Library by Dominic Clifton.
 */

//
// Horizontal
//

#ifndef SPRACING_PIXEL_OSD_HORIZONTAL_RESOLUTION
#define SPRACING_PIXEL_OSD_HORIZONTAL_RESOLUTION 360
#endif

//
// Vertical
//

#ifndef SPRACING_PIXEL_OSD_PAL_VISIBLE_LINES
#define SPRACING_PIXEL_OSD_PAL_VISIBLE_LINES 288  // MAX7456 = 16 rows * 18 character height
#endif

#ifndef SPRACING_PIXEL_OSD_NTSC_VISIBLE_LINES
#define SPRACING_PIXEL_OSD_NTSC_VISIBLE_LINES 234 // MAX7456 (13 rows * 18 character height)
#endif
