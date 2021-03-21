/**
 * Author: Jude Alnas
 * Date: Jan 30 2021
 * Description: A generic, thread-safe FIFO buffer. Data members of nodes are of type void*.
 *  FIFO is implemented as a doubly linked list with a sentinel node
 *  that points to the first and last elements of the list.
 * 
 *  To be thread-safe, mutexes and conditional variales are used to control access to the buffer.
 *  
 *  fifoPull always reads from the tail of the buffer (i.e., from sentinel->prev).
 *  Hence, high priority nodes are placed closer to the tail of the buffer.
 *  Negative priorities are treated as the highest priority and placed at the tail of the buffer
 *  Otherwise, the node is placed behind nodes of equal or higher priority, whichever occurs first
 *  when traversing the buffer from head to tail.
 * 
 * Usage:
 *  1) Instantiate buffer using fifoBufferInit()
 *  2) Call fifoPush, fifoPull, fifoFlush from desired threads
 *  3) When done, call fifoBufferClose
 * **/

#ifndef _FIFO_H_
#define _FIFO_H_

    #include <pthread.h>
    #include <stdlib.h>
    #include <stdbool.h>
    #include <stdio.h>
    #include <errno.h>

    typedef struct Node {
        void* data;
        struct Node *next;
        struct Node *prev;
        int priority;
    } fifo_node_t;

    typedef struct Buffer {
        pthread_mutex_t lock;
        pthread_cond_t cond_nonfull;
        pthread_cond_t cond_nonempty;
        int max_buffer_size;
        int buffer_occupancy;
        fifo_node_t *sentinel;
    } fifo_buffer_t;

    //Buffer interaction
    int fifoPush(fifo_buffer_t* buffer, void* data, int priority, bool blocking);
    void* fifoPull(fifo_buffer_t* buffer, bool blocking);
    void** fifoFlush(fifo_buffer_t* buffer, bool blocking);
    /////////////////////////////////////////////////////////////////

    fifo_buffer_t* fifoBufferInit(int max_buffer_size); //buffer instantiation
    void** fifoBufferClose(fifo_buffer_t*); //buffer destructor
    void fifoPrint(fifo_buffer_t* buffer); //for debugging
    int fifoUpdateOccupancy(fifo_buffer_t* buffer); //determines buffer occupancy by traversing list; not used for performance reasons
#endif