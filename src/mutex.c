#include <stdio.h>
#include <semaphore.h>
#include <pthread.h>
#include <stdint.h>
#include <unistd.h>

#include "read.h"

int rwlock_init(ReadWrite_Lock *rw){
    rw->reader = 0;

    if (pthread_mutex_init(&rw->reader_count, NULL) != 0)
        return -1;

    if (pthread_mutex_init(&rw->writer_count, NULL) != 0) {
        pthread_mutex_destroy(&rw->reader_count);
        return -1;
    }

    if (sem_init(&rw->resource, 0, 1) != 0) {
        pthread_mutex_destroy(&rw->reader_count);
        pthread_mutex_destroy(&rw->writer_count);
        return -1;
    }

    return 0;
}

/*
 * Reader Entry
 *
 * Multiple readers can access the resource concurrently.
 * The FIRST reader acquires the resource semaphore to block any writer.
 * Subsequent readers increment the count without touching the semaphore.
 * reader_count is protected by its own mutex so increments are atomic.
 */
void reader_enter(ReadWrite_Lock *lock){
    pthread_mutex_lock(&lock->reader_count);
    lock->reader++;
    if (lock->reader == 1) {
        /* First reader: block writers from acquiring the resource */
        sem_wait(&lock->resource);
    }
    pthread_mutex_unlock(&lock->reader_count);
}

/*
 * Reader Exit
 *
 * Decrements the reader count.
 * The LAST reader releases the resource semaphore so a waiting writer
 * can proceed.
 */
void reader_exit(ReadWrite_Lock *lock){
    pthread_mutex_lock(&lock->reader_count);
    lock->reader--;
    if (lock->reader == 0) {
        /* Last reader: allow writers to proceed */
        sem_post(&lock->resource);
    }
    pthread_mutex_unlock(&lock->reader_count);
}

/*
 * Writer Entry
 *
 * Writers require exclusive access: acquire the resource semaphore.
 * This blocks if any reader or another writer holds it.
 */
void writer_enter(ReadWrite_Lock *lock){
    sem_wait(&lock->resource);
}

/*
 * Writer Exit
 *
 * Release the resource semaphore so another reader or writer can proceed.
 */
void writer_exit(ReadWrite_Lock *lock){
    sem_post(&lock->resource);
}

/*
 * Destroy all synchronization primitives in the lock.
 */
void rwlock_destroy(ReadWrite_Lock *rw){
    pthread_mutex_destroy(&rw->reader_count);
    pthread_mutex_destroy(&rw->writer_count);
    sem_destroy(&rw->resource);
}