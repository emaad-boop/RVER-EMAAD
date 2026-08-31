#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include "read.h"

int message_queue_init(Message_Queue *queue){
  queue->head = 0;
  queue->tail = 0;
  queue->current = 0;
  if(pthread_mutex_init(&queue->mutex, NULL) != 0){
    return -1;
  }
  if(sem_init(&queue->empty, 0, 50) != 0){
    pthread_mutex_destroy(&queue->mutex);
    return -1;
  }
  if(sem_init(&queue->full, 0, 0) != 0){
    sem_destroy(&queue->empty);
    pthread_mutex_destroy(&queue->mutex);
    return -1;
  }
  return 0;
};

void message_destroy(Message_Queue *queue){
  pthread_mutex_destroy(&queue->mutex);
  sem_destroy(&queue->full);
  sem_destroy(&queue->empty);
};

/* message_queue_push
 *
 * Adds a message to the circular queue.
 * Blocks when the queue is full (sem_wait on empty).
 * Uses a mutex to protect the shared tail index.
 * Uses modulo arithmetic so no extra memory is ever wasted.
 */
int message_queue_push(Message_Queue *queue, const Message *msg){
  /* Block until there is at least one empty slot */
  sem_wait(&queue->empty);

  pthread_mutex_lock(&queue->mutex);

  /* Copy message into the next free slot */
  queue->buffer[queue->tail] = *msg;

  /* Advance tail using modulo to keep it circular (no wasted memory) */
  queue->tail = (queue->tail + 1) % 50;

  pthread_mutex_unlock(&queue->mutex);

  /* Signal that one more item is ready to be consumed */
  sem_post(&queue->full);
  return 0;
};

int message_queue_pop(Message_Queue *queue, Message *msg){
  sem_wait(&queue->full);
  pthread_mutex_lock(&queue->mutex);
  *msg = queue->buffer[queue->head];
  queue->head = (queue->head + 1) % 50;
  pthread_mutex_unlock(&queue->mutex);
  sem_post(&queue->empty);
  return 0;
};