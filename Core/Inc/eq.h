#ifndef EQ_H
#define EQ_H

#include <stdint.h>

/* Design the filter stages for the given sample rate, clear all filter state
   and set up the bypass button (PA0) and the EQ-active LED (PD13).
   Safe to call more than once. */
void EQ_Init(float fs);

/* Process 'frames' stereo frames in place (interleaved L,R, signed 16-bit).
   Called from the I2S DMA interrupt, so it must stay short. */
void EQ_Process(int16_t *buf, uint32_t frames);

#endif /* EQ_H */
