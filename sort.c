#include "sort.h"

void sort_simple_rise(sort_element_type *data, int num)
{
	int i, j;
	int min_index;
	sort_element_type tmp;
	for(i = 0; i < num; i++)
	{
		min_index = i;
		for(j = i + 1; j < num; j++)
		{
			if(data[j] < data[min_index])
			{
				min_index = j;
			}
		}
		tmp = data[min_index];
		data[min_index] = data[i];
		data[i] = tmp;
	}
}