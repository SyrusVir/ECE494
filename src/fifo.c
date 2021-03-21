/**
 * Author: Jude Alnas
 * Date: Jan 30 2021
 * Description: A generic, thread-safe FIFO buffer. Buffers data pointers. Includes priority functionality
 * **/
#include "fifo.h"

//linked list manipulation functions
void addNodeAfter(fifo_node_t* curr_node, fifo_node_t* new_node) {
    new_node->next = curr_node->next;
    new_node->prev = curr_node;
    curr_node->next->prev = new_node;
    curr_node->next = new_node;
}

fifo_node_t* removeNode(fifo_buffer_t* buffer, fifo_node_t* node) {
    if (node == buffer->sentinel) {
        perror("Sentinel node cannot be removed from FIFO.");
        return NULL;
    }
    
    node->prev->next = node->next;
    node->next->prev = node->prev;
    node->next = NULL;
    node->prev = NULL;

    return node;
}

void addToHead(fifo_buffer_t* buffer, fifo_node_t* new_node) {
    addNodeAfter(buffer->sentinel, new_node);
}

void addToTail(fifo_buffer_t* buffer, fifo_node_t* new_node) {
    addNodeAfter(buffer->sentinel->prev, new_node);
}

//Constructors & Destructors
fifo_node_t* fifoNodeCreate(void* data, int priority) {
    /** Returns pointer to a FIFO buffer node in allocated memory.
     * Next and Prev are intialized to NULL. Pair with fifoNodeDestroy
     * to ensure memory is freed and contined data is preserved**/

    fifo_node_t *node_out = (fifo_node_t*) malloc(sizeof(fifo_node_t));
    node_out->priority = priority;
    node_out->data = data;
    node_out->next = NULL;
    node_out->prev = NULL;

    return node_out;
}

void* fifoNodeDestroy(fifo_node_t* node) {
    /** Returns data pointer contained in node and frees allocated memory **/
    void* out = node->data;
    free(node);
    return out;
}

fifo_buffer_t* fifoBufferInit(int max_buffer_size) {
    fifo_buffer_t *buffer = (fifo_buffer_t*) malloc(sizeof(fifo_buffer_t));

    pthread_mutex_init(&buffer->lock,NULL);
    pthread_cond_init(&buffer->cond_nonfull,NULL);
    pthread_cond_init(&buffer->cond_nonempty,NULL);
    buffer->max_buffer_size = max_buffer_size;
    buffer->buffer_occupancy = 0;
    buffer->sentinel = fifoNodeCreate(NULL,0);
    buffer->sentinel->next = buffer->sentinel;
    buffer->sentinel->prev = buffer->sentinel;
    return buffer;
}

void** fifoBufferClose(fifo_buffer_t* buffer) {
    /**Closes all references to the buffer, returning the data 
     * conents remaining in a NULL-terminated array of pointers**/
    
    void* *out = fifoFlush(buffer,true);
    pthread_mutex_destroy(&buffer->lock);
    pthread_cond_destroy(&buffer->cond_nonempty); 
    pthread_cond_destroy(&buffer->cond_nonfull);
    free(buffer);

    return out; 
}

//Higher-level methods
int fifoLockBuffer(fifo_buffer_t* buffer, bool blocking) {
    int lock_status;
    if (blocking) {
        lock_status = pthread_mutex_lock(&buffer->lock);
    }
    else {
        lock_status = pthread_mutex_trylock(&buffer->lock);
    }

    return lock_status;
}

int fifoPush(fifo_buffer_t* buffer, void* data, int priority, bool blocking) {
    /**Push data into buffer. If blocking == true, this function will wait until
     * the buffer is available and non-full. 0 is returned if push is successful.
     * If priority < 0, data will be appended at the end of FIFO and will be the 
     * next node retrieved by fifoPUll. Otherwise, the node is inserted behind the 
     * first node of equal or greater priority.  **/

    int lock_status = fifoLockBuffer(buffer,blocking);

    //Push node into buffer if mutex acquired ///////////////////////
    if (lock_status == 0) {
        if(buffer->buffer_occupancy >= buffer->max_buffer_size) {
            if (blocking) {
                int cond_status;
                cond_status = pthread_cond_wait(&buffer->cond_nonfull, &buffer->lock);
                if (cond_status != 0) return cond_status; 
            } //if blocking set, wait until nonfull signal is emitted
            else {
                return -1;
            } //otw return immediately with -1;
        } //If buffer full, wait or return

        //This point reached only if mutex is obtained and nonfull signal has been emitted
        fifo_node_t* new_node = fifoNodeCreate(data, priority); //initialize new buffer node

        if (priority < 0) {
            addNodeAfter(buffer->sentinel->prev,new_node);
        } //Negative priorities are considered higher than any existing. Append to tail
        else {
            //loop through buffer until a node of equal or higher priority is found.
            fifo_node_t* p;
            for (p = buffer->sentinel->next; p != buffer->sentinel;p = p->next) {
                if (p->priority >= new_node->priority || p->priority < 0) break;
            }
            addNodeAfter(p->prev,new_node);
        } //For non-negative priorities, find node of equal or greater priority. Insert new node before.

        buffer->buffer_occupancy++;

        pthread_cond_signal(&buffer->cond_nonempty);
        pthread_mutex_unlock(&buffer->lock);
    } // continue if lock successfully obtained
    ///////////////////////////////////////////////////////////

    return lock_status;
}   

void* fifoPull(fifo_buffer_t* buffer, bool blocking) {
    /**Returns the next data pointer in the FIFO pointed to by buffer.
     * If blocking == true, this call will block until the buffer is
     * available and non-empty. A non-null pointer is returned if the 
     * pull was successful.   **/

    int lock_status = fifoLockBuffer(buffer,blocking);

    if (lock_status == 0){
        if(buffer->buffer_occupancy <= 0) {
            if (blocking) {
                int cond_status;
                cond_status = pthread_cond_wait(&buffer->cond_nonempty, &buffer->lock); 
                if (cond_status != 0) return NULL;
            } //if blocking set, wait until nonempty signal is emitted
            else {
                return NULL;
            } //otw return immediately with NULL;
        } //If buffer empty, wait or return

        //This point is reached if the buffer is available and nonempty
        fifo_node_t* rec = removeNode(buffer, buffer->sentinel->prev);  //remove node at buffer tail
        
        buffer->buffer_occupancy--;
        
        pthread_cond_signal(&buffer->cond_nonfull);
        pthread_mutex_unlock(&buffer->lock);
        
        return fifoNodeDestroy(rec); //deallocate memory and return data pointer
    } //Pull from buffer if lock acquired 
    else return NULL;
}

void** fifoFlush(fifo_buffer_t* buffer, bool blocking) {
    /**Empties the buffer, returning a NULL-terminated 
     * array of pointers pointing to the data contents.
     * If blocking == true, this function call will block 
     * until the buffer is available. NULL is returned if
     * flush is unsuccessful. Otherwise, a pointer to an
     * array containing the remaining buffer contents is 
     * returned **/
    int lock_status = fifoLockBuffer(buffer,blocking);
    
    if (lock_status == 0) {
        void** out = (void*) calloc(buffer->buffer_occupancy + 1,sizeof(void*));
        fifo_node_t* p_tmp;
        fifo_node_t* p = buffer->sentinel->next;
        int i = 0;
        while (p != buffer->sentinel) {
            p_tmp = p; //store pointer to current node
            p = p->next; //get pointer to next node
            out[i] = fifoNodeDestroy(p_tmp); //destroy current node and retrieve data
            i++;
        }
        
        out[i] = NULL; //Append NULL termination
        
        //empty buffer configuration
        buffer->sentinel->prev = buffer->sentinel;
        buffer->sentinel->next = buffer->sentinel;
        
        pthread_cond_signal(&buffer->cond_nonfull);
        pthread_mutex_unlock(&buffer->lock);
        return out;
    }
    else return NULL;
}

void fifoPrint(fifo_buffer_t* buffer) {
    fifo_node_t* p;
    int i = 0;
    
    for (p = buffer->sentinel->next; p != buffer->sentinel; p = p->next) {
        printf(" Node %d:  Address=%p  Priority=%d  Next=%p  Prev=%p\n",i,p,p->priority,p->next,p->prev);
        i++;
    }
}

int fifoUpdateOccupancy(fifo_buffer_t* buffer) {
    /**A more accurate means of determining buffer occupancy.
     * The buffer is iterated through its entirety, counting the entries.**/

    int cnt = 0;
    fifo_node_t* p;
    for (p = buffer->sentinel->next; p != buffer->sentinel; p = p->next) cnt++;
    
    buffer->buffer_occupancy = cnt;
    return cnt;
}
