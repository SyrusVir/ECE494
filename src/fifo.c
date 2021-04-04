/**
 * Author: Jude Alnas
 * Date: Jan 30 2021
 * Description: A generic, thread-safe FIFO buffer. Buffers data pointers. Includes priority functionality
 * **/
#include "fifo.h"

//linked list manipulation functions
void addNodeAfter(fifo_node_t* curr_node, fifo_node_t* new_node) 
{
    new_node->next = curr_node->next;
    new_node->prev = curr_node;
    curr_node->next->prev = new_node;
    curr_node->next = new_node;
}

fifo_node_t* removeNode(fifo_buffer_t* buffer, fifo_node_t* node) 
{
    if (node == buffer->sentinel) 
    {
        perror("Sentinel node cannot be removed from FIFO.");
        return NULL;
    }
    
    node->prev->next = node->next;
    node->next->prev = node->prev;
    node->next = NULL;
    node->prev = NULL;

    return node;
}

/** Returns pointer to a FIFO buffer node in allocated memory.
 * Next and Prev are intialized to NULL. Pair with fifoNodeDestroy
 * to ensure memory is freed and contined data is preserved
 */
fifo_node_t* fifoNodeCreate(void* data, int priority) 
{

    fifo_node_t *node_out = (fifo_node_t*) malloc(sizeof(fifo_node_t));
    node_out->priority = priority;
    node_out->data = data;
    node_out->next = NULL;
    node_out->prev = NULL;

    return node_out;
}

/** Returns data pointer contained in node and frees allocated memory **/
void* fifoNodeDestroy(fifo_node_t* node) 
{
    void* out = node->data;
    free(node);
    return out;
}

fifo_buffer_t* fifoBufferInit(int max_buffer_size) 
{
    fifo_buffer_t *buffer = (fifo_buffer_t*) malloc(sizeof(fifo_buffer_t));
    
    pthread_mutexattr_t mutex_attr;
    pthread_mutexattr_init(&mutex_attr);
    pthread_mutexattr_settype(&mutex_attr, PTHREAD_MUTEX_ERRORCHECK);
    pthread_mutex_init(&buffer->lock,&mutex_attr);
    pthread_cond_init(&buffer->cond_nonfull,NULL);
    pthread_cond_init(&buffer->cond_nonempty,NULL);
    buffer->max_buffer_size = max_buffer_size;
    buffer->buffer_occupancy = 0;
    buffer->sentinel = fifoNodeCreate(NULL,0);
    buffer->sentinel->next = buffer->sentinel;
    buffer->sentinel->prev = buffer->sentinel;
    return buffer;
}

/**Closes all references to the buffer, returning the data 
 * conents remaining in a NULL-terminated array of pointers**/
void** fifoBufferClose(fifo_buffer_t* buffer) 
{
    
    void* *out = fifoFlush(buffer,true);
    pthread_mutex_destroy(&buffer->lock);
    pthread_cond_destroy(&buffer->cond_nonempty); 
    pthread_cond_destroy(&buffer->cond_nonfull);
    free(buffer);

    return out; 
}

//Higher-level methods
int fifoLockBuffer(fifo_buffer_t* buffer, bool blocking) 
{
    if (blocking) 
    {
        return pthread_mutex_lock(&buffer->lock);
    }
    else 
    {
        return pthread_mutex_trylock(&buffer->lock);
    }
}

/**Push data into buffer. If blocking == true, this function will wait until
 * the buffer is available and non-full. 0 is returned if push is successful.
 * If priority < 0, data will be appended at the end of FIFO and will be the 
 * next node retrieved by fifoPUll. Otherwise, the node is inserted behind the 
 * first node of equal or greater priority.  **/
int fifoPush(fifo_buffer_t* buffer, void* data, int priority, bool blocking) 
{

    int lock_status = fifoLockBuffer(buffer,blocking);

    //Push node into buffer if mutex acquired ///////////////////////
    if (lock_status == 0) 
    {
        if(buffer->buffer_occupancy >= buffer->max_buffer_size) 
        {
            if (blocking) 
            {
                int cond_status;
                cond_status = pthread_cond_wait(&buffer->cond_nonfull, &buffer->lock);
                if (cond_status != 0) return cond_status; 
            } //if blocking set, wait until nonfull signal is emitted
            else 
            {
                pthread_mutex_unlock(&buffer->lock);
                return -1;
            } //otw return immediately with -1;
        } //If buffer full, wait or return

        //This point reached only if mutex is obtained and nonfull signal has been emitted
        fifo_node_t* new_node = fifoNodeCreate(data, priority); //initialize new buffer node

        if (priority < 0) 
        {
            addNodeAfter(buffer->sentinel->prev,new_node);
        } //Negative priorities are considered higher than any existing. Append to tail
        else 
        {
            //loop through buffer until a node of equal or higher priority is found.
            fifo_node_t* p;
            for (p = buffer->sentinel->next; p != buffer->sentinel;p = p->next) 
            {
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

void* fifoPull(fifo_buffer_t* buffer, bool blocking) 
{
    /**Returns the next data pointer in the FIFO pointed to by buffer.
     * If blocking == true, this call will block until the buffer is
     * available and non-empty. A non-null pointer is returned if the 
     * pull was successful.   **/

    int lock_status = fifoLockBuffer(buffer,blocking);

    if (lock_status == 0)
    {
        if(buffer->buffer_occupancy <= 0) 
        {
            if (blocking) 
            {
                int cond_status;
                cond_status = pthread_cond_wait(&buffer->cond_nonempty, &buffer->lock); 
                if (cond_status != 0) return NULL;
            } //if blocking set, wait until nonempty signal is emitted
            else 
            {
                pthread_mutex_unlock(&buffer->lock);
                return NULL;
            } //otw unlock acquired mutex and return with NULL;
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

void** fifoFlush(fifo_buffer_t* buffer, bool blocking) 
{
    /**Empties the buffer, returning a NULL-terminated 
     * array of pointers pointing to the data contents.
     * If blocking == true, this function call will block 
     * until the buffer is available. NULL is returned if
     * flush is unsuccessful. Otherwise, a pointer to a
     * NULL terminated array containing the remaining buffer 
     * contents is returned 
     */
    int lock_status = fifoLockBuffer(buffer,blocking);
    
    if (lock_status == 0) //if mutex obtained
    {
        //allocate output array of nodes. +1 for NULL terminator
        void** out = (void*) calloc(buffer->buffer_occupancy + 1,sizeof(void*));

        //iterate over current FIFO nodes
        fifo_node_t* p_tmp; //temporary pointer
        fifo_node_t* p = buffer->sentinel->next; //first node in FIFO
        int i = 0;
        while (buffer->sentinel->prev != buffer->sentinel) 
        {
            //NOTE: fifoPull is not used here because that function requires access to the mutex
            //      Using here would cause a deadlock. Direct list manipulation is done to make
            //      fifoFlush an atomic operation.
            out[i] = fifoNodeDestroy(removeNode(buffer,buffer->sentinel->prev));
            i++;
        }
        
        out[i] = NULL; //Append NULL termination
        
        //empty buffer configuration
        buffer->sentinel->prev = buffer->sentinel;
        buffer->sentinel->next = buffer->sentinel;
        buffer->buffer_occupancy = 0;
        
        pthread_cond_signal(&buffer->cond_nonfull);
        pthread_mutex_unlock(&buffer->lock);
        return out;
    } //end if (lock_status == 0) i.e. if mutex obtained
    else return NULL; //if failed to obtain mutex, return NULL
}

void fifoPrint(fifo_buffer_t* buffer) 
{
    fifo_node_t* p;
    int i = 0;
    
    for (p = buffer->sentinel->next; p != buffer->sentinel; p = p->next) 
    {
        printf(" Node %d:  Address=%p  Priority=%d  Next=%p  Prev=%p\n",i,p,p->priority,p->next,p->prev);
        i++;
    }
}

int fifoUpdateOccupancy(fifo_buffer_t* buffer) 
{
    /**A more accurate means of determining buffer occupancy.
     * The buffer is iterated through its entirety, counting the entries.**/

    int cnt = 0;
    fifo_node_t* p;
    for (p = buffer->sentinel->next; p != buffer->sentinel; p = p->next) cnt++;
    
    buffer->buffer_occupancy = cnt;
    return cnt;
}
