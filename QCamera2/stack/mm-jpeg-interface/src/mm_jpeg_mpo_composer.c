/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#define ATRACE_TAG ATRACE_TAG_CAMERA

#include <pthread.h>
#include "mm_jpeg_dbg.h"
#include "mm_jpeg_mpo.h"

#define M_APP0    0xe0
#define M_APP1    0xe1
#define M_APP2    0xe2

pthread_mutex_t g_mpo_lock = PTHREAD_MUTEX_INITIALIZER;

/** mm_jpeg_mpo_write_long_little_endian
 *
 *  Arguments:
 *    @buffer_addr: image start addr
 *    @buff_offset: offset in the buffer
 *    @buffer_size: Size of the buffer
 *    @value: Value to write
 *    @overflow : Overflow flag
 *
 *  Return:
 *       Start offset of the specified app marker
 *
 *  Description:
 *       Gets the start offset of the given app marker
 *
 **/
void mm_jpeg_mpo_write_long_little_endian(uint8_t *buff_addr, uint32_t buff_offset,
  uint32_t buffer_size, int value, uint8_t *overflow)
{
  if (buff_offset + 3 >= buffer_size) {
    *overflow = TRUE;
  }

  if (!(*overflow)) {
    buff_addr[buff_offset + 3] = (uint8_t)((value >> 24) & 0xFF);
    buff_addr[buff_offset + 2] = (uint8_t)((value >> 16) & 0xFF);
    buff_addr[buff_offset + 1] = (uint8_t)((value >> 8) & 0xFF);
    buff_addr[buff_offset] = (uint8_t)(value & 0xFF);
  }
}

/** mm_jpeg_mpo_get_app_marker
 *
 *  Arguments:
 *    @buffer_addr: Jpeg image start addr
 *    @app_marker: app_marker to find
 *
 *  Return:
 *       Start offset of the specified app marker
 *
 *  Description:
 *       Gets the start offset of the given app marker
 *
 **/
uint8_t *mm_jpeg_mpo_get_app_marker(uint8_t *buffer_addr, int app_marker)
{
  int32_t byte;
  uint8_t *p_current_addr = NULL, *p_start_offset = NULL;

  p_current_addr = buffer_addr;
  do {
      do {
        byte = *(p_current_addr);
        p_current_addr++;
      }
      while (byte != 0xFF);
      //Read the next byte after 0xFF
      byte = *(p_current_addr);

      if (byte == app_marker) {
        p_start_offset = ++p_current_addr;
        break;
      }
  }
  while (byte == 0);

  return p_start_offset;
}

/** mm_jpeg_mpo_get_mp_header
 *
 *  Arguments:
 *    @app2_marker: app2_marker start offset
 *
 *  Return:
 *       Start offset of the MP header
 *
 *  Description:
 *       Get the start offset of the MP header (before the MP
 *       Endian field). All offsets in the MP header need to be
 *       specified wrt this start offset.
 *
 **/
uint8_t *mm_jpeg_mpo_get_mp_header(uint8_t *app2_start_offset)
{
  uint8_t *mp_headr_start_offset = NULL;

  if (app2_start_offset != NULL) {
    mp_headr_start_offset = app2_start_offset + MP_APP2_FIELD_LENGTH_BYTES +
      MP_FORMAT_IDENTIFIER_BYTES;
  }

  return mp_headr_start_offset;
}

/** mm_jpeg_mpo_update_header
 *
 *  Arguments:
 *    @mpo_info: MPO Info
 *
 *  Return:
 *       0 - Success
 *       -1 - otherwise
 *
 *  Description:
 *      Update the MP Index IFD of the first image with onfo
 *      about abot all other images.
 *
 **/
int mm_jpeg_mpo_update_header(mm_jpeg_mpo_info_t *mpo_info)
{
  uint8_t *app2_start_off_addr = NULL, *mp_headr_start_off_addr = NULL;
  uint32_t mp_index_ifd_offset = 0, current_offset = 0, mp_entry_val_offset = 0;
  uint8_t *aux_start_addr = NULL;
  uint8_t overflow_flag;
  int i = 0;
  int rc = -1;

  //Get the addr of the App Marker
  app2_start_off_addr = mm_jpeg_mpo_get_app_marker(
    mpo_info->output_buff.buf_vaddr, M_APP2);
  if (!app2_start_off_addr) {
    CDBG_ERROR("%s %d:] Cannot find App2 marker. MPO composition failed",
      __func__, __LINE__ );
    return rc;
  }

  //Get the addr of the MP Headr start offset.
  //All offsets in the MP header are wrt to this addr
  mp_headr_start_off_addr = mm_jpeg_mpo_get_mp_header(app2_start_off_addr);
  if (!mp_headr_start_off_addr) {
    CDBG_ERROR("%s %d:] mp headr start offset is NULL. MPO composition failed",
      __func__, __LINE__ );
    return rc;
  }

  current_offset = mp_headr_start_off_addr - mpo_info->output_buff.buf_vaddr;
  //TODo:Get Endianess

  //offset to first ifd
  current_offset += MP_ENDIAN_BYTES;
  //To do: Read the value to get MP Index IFD. Right now, we know it is immediately
  //after
  current_offset += 8;
  mp_index_ifd_offset = current_offset;

  //Traverse to MP Entry value
  //ToDo: read count value
  current_offset += MP_INDEX_COUNT_BYTES + MP_INDEX_VERSION_BYTES +
    MP_INDEX_NUMBER_OF_IMAGES_BYTES +
    MP_INDEX_ENTRY_BYTES + MP_INDEX_IMAGE_UNIQUE_ID_LIST_BYTES +
    MP_INDEX_TOTAL_CAPURED_FRAMES + MP_INDEX_OFFSET_OF_NEXT_IFD_BYTES;

  mp_entry_val_offset = current_offset;

  //Update image size for primary image
  current_offset += MP_INDEX_ENTRY_INDIVIDUAL_IMAGE_ATTRIBUTE_BYTES;
  mm_jpeg_mpo_write_long_little_endian(mpo_info->output_buff.buf_vaddr,
    current_offset,  mpo_info->output_buff_size,
    mpo_info->primary_image.buf_filled_len, &overflow_flag);

  aux_start_addr = mpo_info->output_buff.buf_vaddr +
    mpo_info->primary_image.buf_filled_len;

  for (i = 1; i < mpo_info->num_of_images; i++) {

    //Go to MP Entry val for each image
    mp_entry_val_offset += MP_INDEX_ENTRY_VALUE_BYTES;
    current_offset = mp_entry_val_offset;

    //Update image size
    current_offset += MP_INDEX_ENTRY_INDIVIDUAL_IMAGE_ATTRIBUTE_BYTES;
    mm_jpeg_mpo_write_long_little_endian(mpo_info->output_buff.buf_vaddr,
      current_offset, mpo_info->output_buff_size,
      mpo_info->aux_images[i].buf_filled_len, &overflow_flag);

    //Update the offset
    current_offset += MP_INDEX_ENTRY_INDIVIDUAL_IMAGE_SIZE_BYTES;
    mm_jpeg_mpo_write_long_little_endian(mpo_info->output_buff.buf_vaddr,
      current_offset, mpo_info->output_buff_size,
      aux_start_addr - mp_headr_start_off_addr, &overflow_flag);
    aux_start_addr += mpo_info->aux_images[i].buf_filled_len;
  }
  if (!overflow_flag) {
    rc = 0;
  }
  return rc;
}

/** mm_jpeg_mpo_compose
 *
 *  Arguments:
 *    @mpo_info: MPO Info
 *
 *  Return:
 *       0 - Success
 *      -1 - otherwise
 *
 *  Description:
 *      Compose MPO image from multiple JPEG images
 *
 **/
int mm_jpeg_mpo_compose(mm_jpeg_mpo_info_t *mpo_info)
{
  uint8_t *aux_write_offset = NULL;
  int i = 0, rc = -1;

  pthread_mutex_lock(&g_mpo_lock);

  //Primary image needs to be copied to the o/p buffer if its not already
  if (mpo_info->output_buff.buf_filled_len == 0) {
    if (mpo_info->primary_image.buf_filled_len < mpo_info->output_buff_size) {
      memcpy(mpo_info->output_buff.buf_vaddr, mpo_info->primary_image.buf_vaddr,
        mpo_info->primary_image.buf_filled_len);
      mpo_info->output_buff.buf_filled_len +=
        mpo_info->primary_image.buf_filled_len;
    } else {
      CDBG_ERROR("%s %d: O/P buffer not large enough. MPO composition failed",
        __func__, __LINE__);
      pthread_mutex_unlock(&g_mpo_lock);
      return rc;
    }
  }
  //Append each Aux image to the buffer
  for (i = 1; i < mpo_info->num_of_images; i++) {
    if ((mpo_info->output_buff.buf_filled_len +
      mpo_info->aux_images[i].buf_filled_len) < mpo_info->output_buff_size) {
      aux_write_offset = mpo_info->output_buff.buf_vaddr +
        mpo_info->output_buff.buf_filled_len;
      memcpy(aux_write_offset, mpo_info->aux_images[i].buf_vaddr,
        mpo_info->aux_images[i].buf_filled_len);
      mpo_info->output_buff.buf_filled_len +=
        mpo_info->aux_images[i].buf_filled_len;
    } else {
      CDBG_ERROR("%s %d: O/P buffer not large enough. MPO composition failed",
          __func__, __LINE__);
      pthread_mutex_unlock(&g_mpo_lock);
      return rc;
    }
  }

  rc = mm_jpeg_mpo_update_header(mpo_info);
  pthread_mutex_unlock(&g_mpo_lock);

  return rc;
}
