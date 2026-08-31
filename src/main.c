#include <stdio.h>
#include <pthread.h>
#include <stdint.h>
#include <math.h>
#include <semaphore.h>
#include <string.h>
#include <unistd.h>
#include "read.h"
#include "en_dc.h"
#include "read_file.h"
#include "drive.h"

#define NUM_PRODUCERS 1
#define NUM_CONSUMERS 3

/* -------------------------------------------------------------------------
 * Globals
 * -----------------------------------------------------------------------*/
Message_Queue   queue;
Shared_Buffer   shared_buffer;
ReadWrite_Lock  lock;
pthread_mutex_t message_mutex;
pthread_cond_t  message_available;
unsigned long   message_generation = 0;
int             producer_finished  = 0;

/* -------------------------------------------------------------------------
 * Producer thread
 *
 * Reads (x, y) coordinate pairs from the input file, encodes each pair as a
 * COBS transport message, stores it in the shared buffer and notifies the
 * consumer threads.
 * -----------------------------------------------------------------------*/
void *producer(void *arg)
{
    InputFile  input;
    FileArgs  *args = (FileArgs *)arg;

    if (input_file_open(&input, args->filename) != 0) {
        return NULL;
    }

    float x_coord;
    float y_coord;

    while (input_file_read(&input, &x_coord, &y_coord)) {

        Message msg = {0};

        /* --- TODO resolved: Convert coordinates into a transport message --- */
        /* Pack the two floats into a raw byte array and COBS-encode them.    */
        uint8_t raw[sizeof(float) * 2];
        memcpy(raw,                   &x_coord, sizeof(float));
        memcpy(raw + sizeof(float),   &y_coord, sizeof(float));

        size_t dst_buf_len = ENCODE_DST_BUF_LEN_MAX(sizeof(raw));
        encode_result enc = frame_encode(msg.data, dst_buf_len,
                                         raw, sizeof(raw));
        if (enc.status != ENCODE_OK) {
            /* Encoding failed — skip this coordinate */
            continue;
        }
        msg.length = enc.out_len;

        /* --- TODO resolved: Store in shared buffer safely (no duplicates) ---
         * Use the writer lock so only one message is live at a time.
         * Increment message_generation to let consumers know a new message is
         * available; the generation counter acts as a ticket so each consumer
         * processes each message exactly once.
         */
        pthread_mutex_lock(&message_mutex);
        writer_enter(&lock);
        memcpy(shared_buffer.data, msg.data, msg.length);
        shared_buffer.length = msg.length;
        writer_exit(&lock);
        message_generation++;
        /* --- TODO resolved: Notify waiting consumers --- */
        pthread_cond_broadcast(&message_available);
        pthread_mutex_unlock(&message_mutex);
    }

    /* --- TODO resolved: Signal that production has finished --- */
    pthread_mutex_lock(&message_mutex);
    producer_finished = 1;
    pthread_cond_broadcast(&message_available);
    pthread_mutex_unlock(&message_mutex);

    input_file_close(&input);
    return NULL;
}

/* -------------------------------------------------------------------------
 * Consumer thread
 *
 * Waits for the producer to post a new message, reads it from the shared
 * buffer, decodes it, and pushes the decoded message onto the drive queue.
 * Each message is forwarded exactly once (generation-counter guard).
 * -----------------------------------------------------------------------*/
void *consumer(void *arg)
{
    int id = *(int *)arg;
    (void)id;   /* suppress unused-variable warning */

    unsigned long last_seen = 0;

    while (1) {
        /* --- TODO resolved: Wait for a new message --- */
        pthread_mutex_lock(&message_mutex);
        while (message_generation == last_seen && !producer_finished) {
            pthread_cond_wait(&message_available, &message_mutex);
        }

        /* Exit if producer is done and no new message arrived */
        if (producer_finished && message_generation == last_seen) {
            pthread_mutex_unlock(&message_mutex);
            break;
        }

        /* Record this generation so we don't process it twice */
        last_seen = message_generation;
        pthread_mutex_unlock(&message_mutex);

        /* --- TODO resolved: Safely retrieve message from shared buffer --- */
        Message local_msg = {0};
        reader_enter(&lock);
        memcpy(local_msg.data, shared_buffer.data, shared_buffer.length);
        local_msg.length = shared_buffer.length;
        reader_exit(&lock);

        /* --- TODO resolved: Decode the message --- */
        uint8_t decoded[DECODE_DST_BUF_LEN_MAX(max_size)];
        decode_result dec = frame_decode(decoded, sizeof(decoded),
                                          local_msg.data, local_msg.length);
        if (dec.status != DECODE_OK) {
            continue;
        }

        /* --- TODO resolved: Forward to drive queue (only once per message) ---
         * Wrap the decoded bytes back into a Message and push to the queue.
         */
        Message drive_msg = {0};
        memcpy(drive_msg.data, decoded, dec.out_len);
        drive_msg.length = dec.out_len;
        message_queue_push(&queue, &drive_msg);
    }

    return NULL;
}

/* -------------------------------------------------------------------------
 * Drive-writer thread
 *
 * Receives decoded coordinate messages from the queue, reconstructs the
 * target coordinate, maintains rover state, invokes drive_to_target() and
 * writes the result to the output file.
 * -----------------------------------------------------------------------*/
void *drive_write(void *arg){
    InputFile  input;
    FileArgs  *args = (FileArgs *) arg;

    if (input_file_open_write(&input, args->result_filename) != 0) {
        printf("Failed to open %s\n", args->result_filename);
        return NULL;
    }

    /* Rover starts at origin, heading east */
    struct rover_state rover = {
        .position   = {0.0f, 0.0f, 0.0f},
        .heading_rad = 0.0f
    };

    for (int i = 0; i < 10; i++) {
        /* Receive the decoded coordinate message from the queue */
        Message msg = {0};
        message_queue_pop(&queue, &msg);

        /* Reconstruct the target coordinate from raw bytes */
        float x_coord = 0.0f, y_coord = 0.0f;
        if (msg.length >= sizeof(float) * 2) {
            memcpy(&x_coord, msg.data,                 sizeof(float));
            memcpy(&y_coord, msg.data + sizeof(float), sizeof(float));
        }

        struct coordinate coordinate_target = {
            .latitude  = x_coord,
            .longitude = y_coord,
            .altitude  = 0.0f
        };

        /* Drive the rover to the target */
        enum drive_status result_status = drive_to_target(&rover, &coordinate_target);

        /* Compute final distance error */
        float dx    = coordinate_target.latitude  - rover.position.latitude;
        float dy    = coordinate_target.longitude - rover.position.longitude;
        float error = hypotf(dx, dy);
        int   status;

        if (result_status == DRIVE_REACHED_TARGET && error <= 0.7f) {
            status = 0;
        } else {
            status = 1;
        }

        input_file_write(&input,
                         &rover.position.latitude,
                         &rover.position.longitude,
                         &error, &status);
    }

    input_file_close(&input);
    return NULL;
}


/* -------------------------------------------------------------------------
 * main
 * -----------------------------------------------------------------------*/
int main(){
    pthread_t producers[NUM_PRODUCERS];
    pthread_t consumers[NUM_CONSUMERS];
    pthread_t drive_writers[NUM_PRODUCERS];
    int consumer_id[NUM_CONSUMERS] = {1, 2, 3};

    const char *testcases[] = {
        "input/testcase1.txt",
        "input/testcase2.txt",
        "input/testcase3.txt",
        "input/testcase4.txt"
    };
    const char *result_tc[] = {
        "result/result1.txt",
        "result/result2.txt",
        "result/result3.txt",
        "result/result4.txt"
    };

    if (rwlock_init(&lock) != 0) {
        printf("Reader writer synchronization failed\n");
        return 1;
    }
    if (message_queue_init(&queue) != 0) {
        printf("Queue Initialization failed\n");
        return 1;
    }
    if (pthread_mutex_init(&message_mutex, NULL) != 0) {
        printf("Mutex Initialization failed\n");
        return 1;
    }
    if (pthread_cond_init(&message_available, NULL) != 0) {
        printf("Condition variable Initialization failed\n");
        return 1;
    }

    for (int i = 0; i < 4; i++) {
        printf("Input : %d \n", i + 1);
        printf("\n");

        FileArgs file_args = {
            .id              = 1,
            .filename        = testcases[i],
            .result_filename = result_tc[i]
        };
        message_generation = 0;
        producer_finished  = 0;

        for (int j = 0; j < NUM_PRODUCERS; j++) {
            pthread_create(&producers[j], NULL, producer, &file_args);
        }
        for (int j = 0; j < NUM_CONSUMERS; j++) {
            pthread_create(&consumers[j], NULL, consumer, &consumer_id[j]);
        }
        for (int j = 0; j < NUM_PRODUCERS; j++) {
            pthread_create(&drive_writers[j], NULL, drive_write, &file_args);
        }

        for (int j = 0; j < NUM_PRODUCERS; j++) {
            pthread_join(producers[j], NULL);
        }
        for (int j = 0; j < NUM_CONSUMERS; j++) {
            pthread_join(consumers[j], NULL);
        }
        for (int j = 0; j < NUM_PRODUCERS; j++) {
            pthread_join(drive_writers[j], NULL);
        }
    }

    pthread_cond_destroy(&message_available);
    pthread_mutex_destroy(&message_mutex);
    rwlock_destroy(&lock);
    message_destroy(&queue);

    return 0;
}