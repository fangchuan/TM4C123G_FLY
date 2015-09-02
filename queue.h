#ifndef __QUEUE__
#define __QUEUE__

typedef long queue_element_type;
#define QUEUE_MAX_SIZE 40

typedef struct _queue_t
{
    queue_element_type buf[QUEUE_MAX_SIZE];
    int size;
    int tailIdx, headIdx;
}queue_t;

extern void queue_init(queue_t *queue, int size);
extern void queue_push(queue_t *queue, queue_element_type element);
extern int queue_pop(queue_t *queue, queue_element_type *element);
extern void queue_release(queue_t *queue);

#define QUEUE_ENUM_BEG(queue, i, element)	\
	for(i = (queue.headIdx + 1) % queue.size; i != queue.tailIdx; i++, i %= queue.size)\
	{\
		element = queue.buf[i];\

#define QUEUE_ENUM_END	}

#endif