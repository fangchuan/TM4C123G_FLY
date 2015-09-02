#include "AP_BufferFloat.h"

void ap_bufferfloat_init(AP_BufferFloat *ap_buffer_float, unsigned char size)
{
	ap_buffer_float->size = size;
	ap_bufferfloat_clear(ap_buffer_float);
}

void ap_bufferfloat_clear(AP_BufferFloat *ap_buffer_float)
{
	ap_buffer_float->num_items = 0;
	ap_buffer_float->head = 0;
}

// add - adds an item to the buffer.  returns TRUE if successfully added
int ap_bufferfloat_add(AP_BufferFloat *ap_buffer_float, float item)
{
	// determine position of new item
	unsigned char tail = ap_buffer_float->head + ap_buffer_float->num_items;
	if( tail >= ap_buffer_float->size ) {
		tail -= ap_buffer_float->size;
	}

	// add item to buffer
	ap_buffer_float->buff[tail] = item;

	// increment number of items
	if( ap_buffer_float->num_items < ap_buffer_float->size ) {
		ap_buffer_float->num_items++;
	}else{
		// no room for new items so drop oldest item
		ap_buffer_float->head++;
		if( ap_buffer_float->head >= ap_buffer_float->size ) {
			ap_buffer_float->head = 0;
		}
	}

	// indicate success
	return 1;
}

// get - returns the next value in the buffer
float ap_bufferfloat_get(AP_BufferFloat *ap_buffer_float)
{
	float result;

	// return zero if buffer is empty
	if( ap_buffer_float->num_items == 0 ) {
		return 0;
	}

	// get next value in buffer
	result = ap_buffer_float->buff[ap_buffer_float->head];

	// increment to next point
	ap_buffer_float->head++;
	if( ap_buffer_float->head >= ap_buffer_float->size )
		ap_buffer_float->head = 0;

	// reduce number of items
	ap_buffer_float->num_items--;

	// return item
	return result;
}

// peek - check what the next value in the buffer is but don't pull it off
float ap_bufferfloat_peek(AP_BufferFloat *ap_buffer_float, unsigned char position)
{
	unsigned char j = ap_buffer_float->head + position;

	// return zero if position is out of range
	if( position >= ap_buffer_float->num_items ) {
		return 0;
	}

	// wrap around if necessary
	if( j >= ap_buffer_float->size )
		j -= ap_buffer_float->size;

	// return desired value
	return ap_buffer_float->buff[j];
}
