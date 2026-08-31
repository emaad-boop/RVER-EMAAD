#ifndef EN_DC_H_
#define EN_DC_H_




#include <stdint.h>
#include <stdlib.h>




#define ENCODE_DST_BUF_LEN_MAX(SRC_LEN)            (((SRC_LEN) == 0u) ? 1u : ((SRC_LEN) + (((SRC_LEN) + 253u) / 254u)))
#define DECODE_DST_BUF_LEN_MAX(SRC_LEN)            (((SRC_LEN) == 0u) ? 0u : ((SRC_LEN) - 1u))

#define ENCODE_SRC_OFFSET(SRC_LEN)                 (((SRC_LEN) + 253u)/254u)



typedef enum
{
    ENCODE_OK                  = 0x00,
    ENCODE_NULL_POINTER        = 0x01,
    ENCODE_OUT_BUFFER_OVERFLOW = 0x02
} encode_status;

typedef struct
{
    size_t              out_len;
    encode_status  status;
} encode_result;


typedef enum
{
    DECODE_OK                  = 0x00,
    DECODE_NULL_POINTER        = 0x01,
    DECODE_OUT_BUFFER_OVERFLOW = 0x02,
    DECODE_ZERO_BYTE_IN_INPUT  = 0x04,
    DECODE_INPUT_TOO_SHORT     = 0x08
} decode_status;

typedef struct
{
    size_t              out_len;
    decode_status  status;
} decode_result;




#ifdef __cplusplus
extern "C" {
#endif

encode_result frame_encode(void * dst_buf_ptr, size_t dst_buf_len,
                               const void * src_ptr, size_t src_len);

decode_result frame_decode(void * dst_buf_ptr, size_t dst_buf_len,
                               const void * src_ptr, size_t src_len);

#ifdef __cplusplus
} 
#endif


#endif 
