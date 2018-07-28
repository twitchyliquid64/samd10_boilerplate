#include "samd10.h"

// TODO: Verify exact timing on a scope.
#define CYCLES_PER_MS F_CPU / 128 / 1000
#define NOP_SLED_8 __asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");__asm("NOP");
#define NOP_SLED_32 NOP_SLED_8; NOP_SLED_8;

void delay_ms_busyloop(uint32_t ms) {
  for (uint32_t z = 0; z < ms; z++) {
    for(int i = 0; i < CYCLES_PER_MS; i++) {
      NOP_SLED_32;
      NOP_SLED_32;
      NOP_SLED_32;
      NOP_SLED_8; NOP_SLED_8; NOP_SLED_8;
      __asm("NOP");
      __asm("NOP");
      __asm("NOP");
      __asm("NOP");
      __asm("NOP");
      NOP_SLED_8;
    }
  }
}
