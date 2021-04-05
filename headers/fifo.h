/**
 * Author: Jude Alnas
 * Date: March 27 2021
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
 *  The data member of each node is a void pointer. Data is pushed 
 * 
 * Usage:
 *  1) Instantiate buffer using fifoBufferInit()
 *  2) Push data into buffer using fifoPush()
 *     a) A new fifo_node_t is allocated and assigned the void* data passed to fifoPull
 *     b) This new node is placed at beginning of list (sentinel->next)
 *  3) Pull data from buffer using fifoPull()
 *     a) The node at end of fifo (sentinel->prev) is removed from list
 *     b) The void* data is saved
 *     c) The extracted node is deallocated
 *     d) void* data is returned
 *  4) When done, call fifoBufferClose
 * 
 *  WARNING: The pointer passed to fifoPush() is copied into the fifo node. Hence, it is possible that
 *           the thread calling fifoPush deallocates the data prior to reception by the thread
 *           calling fifoPull, causing a segmentation fault. It is the responsiblity of the higher-level
 *           application using this library to ensure the lifetime of data.
 **/

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

    /********* Buffer interaction *********/
    //Push data into FIFO pointed to by buffer. If blocking is false and buffer is full,
    //this function returns -1.
    int fifoPush(fifo_buffer_t* buffer, void* data, int priority, bool blocking);

    //Pull next data from FIFO pointed to by buffer. If blocking is false and buffer is empty,
    //this function returns -1.
    void* fifoPull(fifo_buffer_t* buffer, bool blocking);

    // Empties FIFO, returning the contents in a NULL terminated array in first-out order (i.e. index 0 is first out) 
    // If blocking is false and buffer is full, this function returns -1.
    void** fifoFlush(fifo_buffer_t* buffer, bool blocking);
    /**************************************/
    /////////////////////////////////////////////////////////////////

    //Returns pointer to initialized FIFO with capacity of max_buffer_size
    fifo_buffer_t* fifoBufferInit(int max_buffer_size); //buffer instantiation
    
    //Frees resources allocated for FIFO. Returns contents in a NULL terminated array in first-out order
    void** fifoBufferClose(fifo_buffer_t* buffer); //buffer destructor

    //Debugging function that prints the contents of the FIFO buffer
    void fifoPrint(fifo_buffer_t* buffer); //for debugging

    //Alternative, UNUSED method of determingin buffer occupancy via traversing entirety of list.
    //This was not used due to overhead concerns
    int fifoUpdateOccupancy(fifo_buffer_t* buffer); //determines buffer occupancy by traversing list; not used for performance reasons
#endif