This folder contains FreeRTOSv8_2_0 PLUS the following modifications:

1) src/portable/GCC/ARM_CM3/port.c had what we believe to be a bug in it:
portVECTACTIVE_MASK should be 0x1FF not 0xFF

2) A port for NRF51 was added.