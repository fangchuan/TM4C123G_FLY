#include "queue.h"
#include <stdlib.h>

void queue_init(queue_t *queue, int size)
{
	queue->size = size + 2;
        queue->size = queue->size < QUEUE_MAX_SIZE ? queue->size : QUEUE_MAX_SIZE;
	//queue->buf = (queue_element_type *)malloc(queue->size * sizeof(queue_element_type));
	queue->tailIdx = 0;
	queue->headIdx = queue->size - 1;
}

void queue_release(queue_t *queue)
{
	queue->size = 0;
}

void queue_push(queue_t *queue, queue_element_type element)
{
	queue->buf[queue->tailIdx] = element;
	queue->tailIdx++;
	queue->tailIdx %= queue->size;
	if(queue->tailIdx == queue->headIdx)
	{
		queue->headIdx++;
		queue->headIdx %= queue->size;
	}
}

int queue_pop(queue_t *queue, queue_element_type *element)
{
	if((queue->headIdx + 1) % queue->size != queue->tailIdx)
	{
		queue->headIdx++;
		queue->headIdx %= queue->size;
		*element = queue->buf[queue->headIdx];
		return 1;
	}
	else
	{
		return 0;
	}
}