#include "fun.h"

extern uint16_t adc_value[2];
extern uint16_t convertarr[CONVERT_NUM];

void adc_task(void)
{
    convertarr[0] = adc_value[0];
    dac_data_set(DAC0, DAC_OUT0, DAC_ALIGN_12B_R, convertarr[0]);
}

