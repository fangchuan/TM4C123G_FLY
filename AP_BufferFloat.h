#ifndef __AP_BUFFERFLOAT__
#define __AP_BUFFERFLOAT__

#define AP_BUFFER_FLOAT_MAX_SIZE 20

typedef struct _AP_BufferFloat
{
	unsigned char num_items;
	unsigned char size;
	unsigned char head;
	float buff[AP_BUFFER_FLOAT_MAX_SIZE];
} AP_BufferFloat;

void ap_bufferfloat_init(AP_BufferFloat *ap_buffer_float, unsigned char size);
void ap_bufferfloat_clear(AP_BufferFloat *ap_buffer_float);
// add - adds an item to the buffer.  returns TRUE if successfully added
int ap_bufferfloat_add(AP_BufferFloat *ap_buffer_float, float item);
// get - returns the next value in the buffer
float ap_bufferfloat_get(AP_BufferFloat *ap_buffer_float);
// peek - check what the next value in the buffer is but don't pull it off
float ap_bufferfloat_peek(AP_BufferFloat *ap_buffer_float, unsigned char position);

#endif
