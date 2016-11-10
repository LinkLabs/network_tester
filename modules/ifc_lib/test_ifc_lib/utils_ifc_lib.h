#ifndef UTILS_IFC_LIB_H
#define UTILS_IFC_LIB_H

#include <stdint.h>

#define TRANSPORT_LEN_MAX       256
#define TRANSPORT_INVOKE_MAX    16


void ifc_utils_setup(void);
void ifc_utils_teardown(void);

uint16_t compute_checksum(uint8_t const * buf, uint16_t len);
void transport_expect(
        uint8_t opcode,
        uint8_t const * wr_buf, uint16_t wr_len,
        uint8_t ack,
        uint8_t const * rd_buf, uint16_t rd_len);

#endif