#include "sample_counter.h"

#include "sct.h"

uint32_t sample_counter_read(void)
{
	return SCT_COUNT;
}
