#include "en_dc.h"
#include <stdlib.h>

/*****************************************************************************
 * Defines
 ****************************************************************************/

#ifndef FALSE
#define FALSE (0)
#endif

#ifndef TRUE
#define TRUE (!FALSE)
#endif

/*****************************************************************************
 * Functions
 ****************************************************************************/

/* Encode (COBS - Consistent Overhead Byte Stuffing)
 *
 * Fixes applied:
 * 1. ENCODE_OK typo fixed (was ECODE_OK)
 * 2. dst_buf_end_ptr correctly set to dst_buf_start_ptr + dst_buf_len
 * 3. dst_code_write_ptr declared and initialized to dst_buf_start_ptr
 * 4. dst_write_ptr initialized to dst_buf_start_ptr + 1u (one past code byte)
 * 5. Null-pointer check moved to before any pointer arithmetic
 * 6. Loop changed from broken for-loop to correct while loop over source bytes
 * 7. Zero-byte detection changed from 0xFF to 0x00 (COBS stuffs zeros)
 * 8. search_len==0u wraparound check correctly flushes at 0xFF block boundary
 */
encode_result frame_encode(void *dst_buf_ptr, size_t dst_buf_len,
                            const void *src_ptr, size_t src_len) {
  encode_result result = {0u, ENCODE_OK};

  /* Fix 1: null check must come before any pointer use */
  if ((dst_buf_ptr == NULL) || (src_ptr == NULL)) {
    result.status = ENCODE_NULL_POINTER;
    return result;
  }

  const uint8_t *src_read_ptr      = (const uint8_t *)src_ptr;
  const uint8_t *src_end_ptr       = src_read_ptr + src_len;
  uint8_t       *dst_buf_start_ptr = (uint8_t *)dst_buf_ptr;
  /* Fix 2: end pointer must point to actual end of destination buffer */
  uint8_t       *dst_buf_end_ptr   = dst_buf_start_ptr + dst_buf_len;
  /* Fix 3: code write pointer starts at the very first output byte */
  uint8_t       *dst_code_write_ptr = dst_buf_start_ptr;
  /* Fix 4: data write pointer starts one byte after the first code slot */
  uint8_t       *dst_write_ptr     = dst_buf_start_ptr + 1u;
  uint8_t        src_byte          = 0u;
  uint8_t        search_len        = 1u;

  if (src_len != 0u) {
    /* Fix 5: iterate over every source byte (not a fixed count) */
    while (src_read_ptr < src_end_ptr) {

      /* Check destination buffer space before writing */
      if (dst_write_ptr >= dst_buf_end_ptr) {
        result.status |= ENCODE_OUT_BUFFER_OVERFLOW;
        dst_write_ptr = dst_buf_end_ptr;
        break;
      }

      src_byte = *src_read_ptr++;

      /* Fix 6: COBS stuffs 0x00 bytes, not 0xFF bytes */
      if (src_byte == 0x00u) {
        /* Flush current block: write code byte and open a new block */
        *dst_code_write_ptr = search_len;
        dst_code_write_ptr  = dst_write_ptr++;
        search_len          = 1u;
      } else {
        *dst_write_ptr++ = src_byte;
        search_len++;

        /* Fix 7: uint8_t wraps at 256; 0 means 255 non-zero bytes written */
        if (search_len == 0u) {
          *dst_code_write_ptr = 0xFFu;
          dst_code_write_ptr  = dst_write_ptr++;
          search_len          = 1u;
        }
      }
    }
  }

  /* Fix 8: finalize last code byte against correct end pointer */
  if (dst_code_write_ptr >= dst_buf_end_ptr) {
    result.status |= ENCODE_OUT_BUFFER_OVERFLOW;
    dst_write_ptr = dst_buf_end_ptr;
  } else {
    *dst_code_write_ptr = search_len;
  }

  result.out_len = (size_t)(dst_write_ptr - dst_buf_start_ptr);
  return result;
}

/* Decode (COBS - Consistent Overhead Byte Stuffing)
 *
 * Fixes applied:
 * 1. Outer loop changed from broken for(i=0;i<len_code;i++) with
 *    uninitialised len_code to while(src_read_ptr < src_end_ptr)
 * 2. len_code is now read fresh at the top of each outer-loop iteration
 * 3. Decoded bytes are actually written: *dst_write_ptr++ = src_byte
 * 4. Implicit 0x00 re-inserted between blocks (except after last block)
 */
decode_result frame_decode(void *dst_buf_ptr, size_t dst_buf_len,
                            const void *src_ptr, size_t src_len) {
  decode_result  result         = {0u, DECODE_OK};
  const uint8_t *src_read_ptr  = (const uint8_t *)src_ptr;
  const uint8_t *src_end_ptr   = src_read_ptr + src_len;
  uint8_t       *dst_buf_start_ptr = (uint8_t *)dst_buf_ptr;
  uint8_t       *dst_buf_end_ptr   = dst_buf_start_ptr + dst_buf_len;
  uint8_t       *dst_write_ptr     = (uint8_t *)dst_buf_ptr;
  size_t         remaining_bytes;
  uint8_t        src_byte;
  uint8_t        i;
  uint8_t        len_code;

  if ((dst_buf_ptr == NULL) || (src_ptr == NULL)) {
    result.status = DECODE_NULL_POINTER;
    return result;
  }

  if (src_len != 0u) {
    /* Fix 1 & 2: correct outer loop — read one len_code per iteration */
    while (src_read_ptr < src_end_ptr) {
      len_code = *src_read_ptr++;

      if (len_code == 0u) {
        result.status |= DECODE_ZERO_BYTE_IN_INPUT;
        break;
      }
      len_code--;   /* number of literal data bytes that follow */

      /* Clamp to available source bytes */
      remaining_bytes = (size_t)(src_end_ptr - src_read_ptr);
      if (len_code > remaining_bytes) {
        result.status |= DECODE_INPUT_TOO_SHORT;
        len_code = (uint8_t)remaining_bytes;
      }

      /* Clamp to available destination space */
      remaining_bytes = (size_t)(dst_buf_end_ptr - dst_write_ptr);
      if (len_code > remaining_bytes) {
        result.status |= DECODE_OUT_BUFFER_OVERFLOW;
        len_code = (uint8_t)remaining_bytes;
      }

      /* Copy the literal bytes */
      for (i = len_code; i != 0u; i--) {
        src_byte = *src_read_ptr++;
        if (src_byte == 0u) {
          result.status |= DECODE_ZERO_BYTE_IN_INPUT;
        }
        /* Fix 3: actually write the decoded byte */
        *dst_write_ptr++ = src_byte;
      }

      /* Fix 4: re-insert implicit 0x00 between blocks (not after last block) */
      if (src_read_ptr < src_end_ptr) {
        if (dst_write_ptr < dst_buf_end_ptr) {
          *dst_write_ptr++ = 0x00u;
        } else {
          result.status |= DECODE_OUT_BUFFER_OVERFLOW;
        }
      }
    }
  }

  result.out_len = (size_t)(dst_write_ptr - dst_buf_start_ptr);
  return result;
}