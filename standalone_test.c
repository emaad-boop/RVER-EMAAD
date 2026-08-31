/*
 * standalone_test.c  -  Single-threaded test runner (no pthreads needed)
 * Works with plain MinGW GCC on Windows.
 *
 * Simulates the full pipeline for each testcase:
 *   read (x,y) -> encode -> decode -> drive_to_target -> write result
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/*  en_dc types (inlined to avoid pthread dependency in read.h)        */
/* ------------------------------------------------------------------ */
#define max_size 256
#define ENCODE_DST_BUF_LEN_MAX(N) (((N)==0u)?1u:((N)+(((N)+253u)/254u)))
#define DECODE_DST_BUF_LEN_MAX(N) (((N)==0u)?0u:((N)-1u))

typedef enum { ENCODE_OK=0x00, ENCODE_NULL_POINTER=0x01, ENCODE_OUT_BUFFER_OVERFLOW=0x02 } encode_status;
typedef struct { size_t out_len; encode_status status; } encode_result;
typedef enum { DECODE_OK=0x00, DECODE_NULL_POINTER=0x01, DECODE_OUT_BUFFER_OVERFLOW=0x02,
               DECODE_ZERO_BYTE_IN_INPUT=0x04, DECODE_INPUT_TOO_SHORT=0x08 } decode_status;
typedef struct { size_t out_len; decode_status status; } decode_result;

/* ------------------------------------------------------------------ */
/*  COBS encoder (fixed version)                                        */
/* ------------------------------------------------------------------ */
encode_result frame_encode(void *dst_buf_ptr, size_t dst_buf_len,
                            const void *src_ptr, size_t src_len) {
  encode_result result = {0u, ENCODE_OK};
  if (!dst_buf_ptr || !src_ptr) { result.status=ENCODE_NULL_POINTER; return result; }
  const uint8_t *src_read_ptr      = (const uint8_t *)src_ptr;
  const uint8_t *src_end_ptr       = src_read_ptr + src_len;
  uint8_t       *dst_buf_start_ptr = (uint8_t *)dst_buf_ptr;
  uint8_t       *dst_buf_end_ptr   = dst_buf_start_ptr + dst_buf_len;
  uint8_t       *dst_code_write_ptr= dst_buf_start_ptr;
  uint8_t       *dst_write_ptr     = dst_buf_start_ptr + 1u;
  uint8_t        src_byte=0u, search_len=1u;

  if (src_len != 0u) {
    while (src_read_ptr < src_end_ptr) {
      if (dst_write_ptr >= dst_buf_end_ptr) {
        result.status |= ENCODE_OUT_BUFFER_OVERFLOW; dst_write_ptr=dst_buf_end_ptr; break;
      }
      src_byte = *src_read_ptr++;
      if (src_byte == 0x00u) {
        *dst_code_write_ptr=search_len; dst_code_write_ptr=dst_write_ptr++; search_len=1u;
      } else {
        *dst_write_ptr++=src_byte; search_len++;
        if (search_len==0u) {
          *dst_code_write_ptr=0xFFu; dst_code_write_ptr=dst_write_ptr++; search_len=1u;
        }
      }
    }
  }
  if (dst_code_write_ptr>=dst_buf_end_ptr) {
    result.status|=ENCODE_OUT_BUFFER_OVERFLOW; dst_write_ptr=dst_buf_end_ptr;
  } else { *dst_code_write_ptr=search_len; }
  result.out_len=(size_t)(dst_write_ptr-dst_buf_start_ptr);
  return result;
}

/* ------------------------------------------------------------------ */
/*  COBS decoder (fixed version)                                        */
/* ------------------------------------------------------------------ */
decode_result frame_decode(void *dst_buf_ptr, size_t dst_buf_len,
                            const void *src_ptr, size_t src_len) {
  decode_result result={0u,DECODE_OK};
  if (!dst_buf_ptr||!src_ptr) { result.status=DECODE_NULL_POINTER; return result; }
  const uint8_t *src_read_ptr  = (const uint8_t *)src_ptr;
  const uint8_t *src_end_ptr   = src_read_ptr + src_len;
  uint8_t *dst_buf_start_ptr   = (uint8_t *)dst_buf_ptr;
  uint8_t *dst_buf_end_ptr     = dst_buf_start_ptr + dst_buf_len;
  uint8_t *dst_write_ptr       = (uint8_t *)dst_buf_ptr;
  size_t remaining; uint8_t src_byte,i,len_code;

  if (src_len!=0u) {
    while (src_read_ptr < src_end_ptr) {
      len_code=*src_read_ptr++;
      if (len_code==0u) { result.status|=DECODE_ZERO_BYTE_IN_INPUT; break; }
      len_code--;
      remaining=(size_t)(src_end_ptr-src_read_ptr);
      if (len_code>remaining) { result.status|=DECODE_INPUT_TOO_SHORT; len_code=(uint8_t)remaining; }
      remaining=(size_t)(dst_buf_end_ptr-dst_write_ptr);
      if (len_code>remaining) { result.status|=DECODE_OUT_BUFFER_OVERFLOW; len_code=(uint8_t)remaining; }
      for (i=len_code;i!=0u;i--) {
        src_byte=*src_read_ptr++;
        if (src_byte==0u) result.status|=DECODE_ZERO_BYTE_IN_INPUT;
        *dst_write_ptr++=src_byte;
      }
      if (src_read_ptr<src_end_ptr) {
        if (dst_write_ptr<dst_buf_end_ptr) *dst_write_ptr++=0x00u;
        else result.status|=DECODE_OUT_BUFFER_OVERFLOW;
      }
    }
  }
  result.out_len=(size_t)(dst_write_ptr-dst_buf_start_ptr);
  return result;
}

/* ------------------------------------------------------------------ */
/*  Rover controller (fixed drive_to_target)                            */
/* ------------------------------------------------------------------ */
#define PI_F             3.14159265358979323846f
#define WHEEL_RADIUS     0.15f
#define WHEEL_SEPARATION 0.77f
#define MAX_LINEAR_VELOCITY  1.0f
#define MAX_ANGULAR_VELOCITY 2.0f
#define MAX_WHEEL_VELOCITY  10.0f
#define HEADING_GAIN     1.25f
#define TARGET_TOLERANCE 0.10f
#define DRIVE_DT_SECONDS 0.02f
#define MAX_DRIVE_STEPS  6000

struct coordinate { float latitude; float longitude; float altitude; };
struct rover_state { struct coordinate position; float heading_rad; };
struct wheel_velocity { float left; float right; };
enum drive_status { DRIVE_REACHED_TARGET=0, DRIVE_INVALID_INPUT=-1,
                    DRIVE_INVALID_COMMAND=-2, DRIVE_MAX_STEPS_EXCEEDED=-3 };

static float normalize_angle(float a) {
  while (a> PI_F) a-=2.0f*PI_F;
  while (a<-PI_F) a+=2.0f*PI_F;
  return a;
}
static bool apply_wheel_velocities(struct rover_state *rover, struct wheel_velocity v) {
  if (!isfinite(v.left)||!isfinite(v.right)||
      fabsf(v.left)>MAX_WHEEL_VELOCITY||fabsf(v.right)>MAX_WHEEL_VELOCITY) return false;
  float lin = WHEEL_RADIUS*(v.left+v.right)/2.0f;
  float ang = WHEEL_RADIUS*(v.right-v.left)/WHEEL_SEPARATION;
  rover->heading_rad=normalize_angle(rover->heading_rad+ang*DRIVE_DT_SECONDS);
  rover->position.longitude+=lin*cosf(rover->heading_rad)*DRIVE_DT_SECONDS;
  rover->position.latitude +=lin*sinf(rover->heading_rad)*DRIVE_DT_SECONDS;
  return true;
}
enum drive_status drive_to_target(struct rover_state *rover, const struct coordinate *target) {
  if (!rover||!target) return DRIVE_INVALID_INPUT;
  if (!isfinite(rover->position.latitude)||!isfinite(rover->position.longitude)||
      !isfinite(rover->heading_rad)||!isfinite(target->latitude)||!isfinite(target->longitude))
    return DRIVE_INVALID_INPUT;
  for (int step=0;step<MAX_DRIVE_STEPS;step++) {
    float dx=target->longitude-rover->position.longitude;
    float dy=target->latitude -rover->position.latitude;
    if (hypotf(dx,dy)<=TARGET_TOLERANCE) return DRIVE_REACHED_TARGET;
    float desired=atan2f(dy,dx);
    float err=normalize_angle(desired-rover->heading_rad);
    float ang=HEADING_GAIN*err;
    if (ang> MAX_ANGULAR_VELOCITY) ang= MAX_ANGULAR_VELOCITY;
    if (ang<-MAX_ANGULAR_VELOCITY) ang=-MAX_ANGULAR_VELOCITY;
    float cos_err=cosf(err);
    float lin=MAX_LINEAR_VELOCITY*(cos_err>0.0f?cos_err:0.0f);
    float hs=WHEEL_SEPARATION/2.0f;
    float vl=(lin-ang*hs)/WHEEL_RADIUS;
    float vr=(lin+ang*hs)/WHEEL_RADIUS;
    if (vl> MAX_WHEEL_VELOCITY) vl= MAX_WHEEL_VELOCITY;
    if (vl<-MAX_WHEEL_VELOCITY) vl=-MAX_WHEEL_VELOCITY;
    if (vr> MAX_WHEEL_VELOCITY) vr= MAX_WHEEL_VELOCITY;
    if (vr<-MAX_WHEEL_VELOCITY) vr=-MAX_WHEEL_VELOCITY;
    struct wheel_velocity vel={vl,vr};
    if (!apply_wheel_velocities(rover,vel)) return DRIVE_INVALID_COMMAND;
  }
  return DRIVE_MAX_STEPS_EXCEEDED;
}

/* ------------------------------------------------------------------ */
/*  Run one testcase: read coords, encode->decode->drive, write result  */
/* ------------------------------------------------------------------ */
void run_testcase(const char *input_path, const char *result_path, int tc_num) {
  printf("=== Testcase %d: %s ===\n", tc_num, input_path);

  FILE *fin = fopen(input_path, "r");
  if (!fin) { printf("  ERROR: cannot open %s\n", input_path); return; }
  FILE *fout = fopen(result_path, "w");
  if (!fout) { printf("  ERROR: cannot open %s for write\n", result_path); fclose(fin); return; }

  /* Rover starts at origin, heading east */
  struct rover_state rover = {{0.0f,0.0f,0.0f},0.0f};

  float x,y;
  int count=0;
  while (fscanf(fin,"%f %f",&x,&y)==2) {
    count++;

    /* --- Encode --- */
    uint8_t raw[sizeof(float)*2];
    memcpy(raw,            &x,sizeof(float));
    memcpy(raw+sizeof(float),&y,sizeof(float));
    uint8_t enc_buf[ENCODE_DST_BUF_LEN_MAX(sizeof(float)*2)+4];
    size_t enc_buf_len = ENCODE_DST_BUF_LEN_MAX(sizeof(float)*2);
    encode_result enc=frame_encode(enc_buf,enc_buf_len,raw,sizeof(raw));
    if (enc.status!=ENCODE_OK) {
      printf("  [%d] Encode error: 0x%02X\n",count,enc.status); continue;
    }

    /* --- Decode --- */
    uint8_t dec_buf[sizeof(float)*2+4];
    decode_result dec=frame_decode(dec_buf,sizeof(dec_buf),enc_buf,enc.out_len);
    if (dec.status!=DECODE_OK) {
      printf("  [%d] Decode error: 0x%02X\n",count,dec.status); continue;
    }

    /* Verify round-trip */
    if (dec.out_len>=sizeof(float)*2) {
      float rx,ry;
      memcpy(&rx,dec_buf,             sizeof(float));
      memcpy(&ry,dec_buf+sizeof(float),sizeof(float));
      if (fabsf(rx-x)>1e-5f||fabsf(ry-y)>1e-5f)
        printf("  [%d] WARNING: round-trip mismatch! orig=(%.3f,%.3f) got=(%.3f,%.3f)\n",count,x,y,rx,ry);
    }

    /* --- Drive --- */
    struct coordinate target={x,y,0.0f};
    enum drive_status ds=drive_to_target(&rover,&target);

    float dx=target.latitude -rover.position.latitude;
    float dy=target.longitude-rover.position.longitude;
    float error=hypotf(dx,dy);
    int   status=(ds==DRIVE_REACHED_TARGET&&error<=0.7f)?0:1;

    /* Print to console */
    printf("  [%d] target=(%.2f,%.2f)  final=(%.2f,%.2f)  err=%.2f  status=%d  drive=%s\n",
           count,x,y,
           rover.position.latitude,rover.position.longitude,
           error,status,
           ds==0?"REACHED":ds==-1?"INVALID_INPUT":ds==-2?"INVALID_CMD":"MAX_STEPS");

    /* Write to result file */
    fprintf(fout,"%.2f %.2f %.2f %d \n",
            rover.position.latitude,rover.position.longitude,error,status);
  }

  fclose(fin);
  fclose(fout);
  printf("  -> Written to %s (%d entries)\n\n", result_path, count);
}

/* ------------------------------------------------------------------ */
/*  Compare result file with expected                                   */
/* ------------------------------------------------------------------ */
void compare_files(const char *result, const char *expected, int tc_num) {
  FILE *fr=fopen(result,"r");
  FILE *fe=fopen(expected,"r");
  if (!fr||!fe) { printf("TC%d: cannot open files for compare\n",tc_num); return; }

  int line=0,mismatches=0;
  float rx,ry,re; int rs;
  float ex,ey,ee; int es;
  while (fscanf(fr,"%f %f %f %d",&rx,&ry,&re,&rs)==4 &&
         fscanf(fe,"%f %f %f %d",&ex,&ey,&ee,&es)==4) {
    line++;
    int ok=(fabsf(rx-ex)<0.02f&&fabsf(ry-ey)<0.02f&&fabsf(re-ee)<0.02f&&rs==es);
    if (!ok) {
      mismatches++;
      printf("  TC%d Line%d MISMATCH: got(%.2f %.2f %.2f %d) expected(%.2f %.2f %.2f %d)\n",
             tc_num,line,rx,ry,re,rs,ex,ey,ee,es);
    }
  }
  fclose(fr); fclose(fe);
  if (mismatches==0)
    printf("  TC%d: ALL %d lines MATCH expected output ✓\n", tc_num, line);
  else
    printf("  TC%d: %d/%d lines differ\n", tc_num, mismatches, line);
}

/* ------------------------------------------------------------------ */
int main(void) {
  /* Make sure result dir exists */
  system("if not exist result mkdir result");

  const char *inputs[]  = {"input/testcase1.txt","input/testcase2.txt",
                            "input/testcase3.txt","input/testcase4.txt"};
  const char *results[] = {"result/result1.txt","result/result2.txt",
                            "result/result3.txt","result/result4.txt"};
  const char *expected[]= {"result/expected_result1.txt","result/expected_result2.txt",
                            "result/expected_result3.txt","result/expected_result4.txt"};

  printf("==========================================\n");
  printf("  RVER-EMAAD  Standalone Test Runner\n");
  printf("==========================================\n\n");

  for (int i=0;i<4;i++) {
    run_testcase(inputs[i],results[i],i+1);
  }

  printf("==========================================\n");
  printf("  Comparing against expected outputs\n");
  printf("==========================================\n");
  for (int i=0;i<4;i++) {
    compare_files(results[i],expected[i],i+1);
  }
  printf("\nDone.\n");
  return 0;
}