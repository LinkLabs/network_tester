#include "ll_ifc.h"
#include "ll_ifc_no_mac.h"
#include "utils_ifc_lib.h"

#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include "cmockery.h"

void setUp(void **state)
{
    (void) state;
    ifc_utils_setup();
    ll_reset_state();
}

void tearDown(void **state)
{
    (void) state;
    ifc_utils_teardown();
}

void test_myself_transport_write(void **state)
{
    (void) state;
    uint8_t cmd_payload[] = {1, 2, 3, 4};
    uint8_t cmd[] = {0xff, 0xff, 0xff, 0xff, 0xc4, 1, 0, 0, 4, 1, 2, 3, 4, 0, 0};
    uint8_t rsp_payload[] = {5, 6, 7, 8};
    uint8_t rsp[TRANSPORT_LEN_MAX];

    uint16_t crc = compute_checksum(cmd + 4, sizeof(cmd) - 6);
    cmd[sizeof(cmd) - 2] = (crc >> 8) & 0xff;
    cmd[sizeof(cmd) - 1] = (crc >> 0) & 0xff;

    transport_expect(1, cmd_payload, sizeof(cmd_payload), 0, rsp_payload, sizeof(rsp_payload));
    assert_int_equal(0, transport_write(cmd, sizeof(cmd)));
    assert_int_equal(0, transport_read(rsp, sizeof(rsp_payload) + 8));
    assert_memory_equal(rsp_payload, rsp + 6, sizeof(rsp_payload));
}

void test_timestamp_get(void **state)
{
    (void) state;
    uint8_t cmd[] = {0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t rsp[] = {0x11, 0x22, 0x33, 0x44};

    transport_expect(OP_TIMESTAMP, cmd, sizeof(cmd), 0, rsp, sizeof(rsp));

    uint32_t actual_timestamp_us = 0;
    assert_int_equal(0, ll_timestamp_get(&actual_timestamp_us));
    assert_int_equal(0x11223344, actual_timestamp_us);
}

void test_timestamp_get_with_response_error(void **state)
{
    (void) state;
    uint8_t cmd[] = {0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t rsp[] = {0x11, 0x22, 0x33}; // too short

    transport_expect(OP_TIMESTAMP, cmd, sizeof(cmd), 0, rsp, sizeof(rsp));

    uint32_t actual_timestamp_us = 0;
    assert_int_equal(LL_IFC_ERROR_INCORRECT_RESPONSE_LENGTH, ll_timestamp_get(&actual_timestamp_us));
}

void test_timestamp_get_twice_to_check_message_identifier(void **state)
{
    (void) state;
    uint8_t cmd[] = {0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t rsp[] = {0x11, 0x22, 0x33, 0x44};

    transport_expect(OP_TIMESTAMP, cmd, sizeof(cmd), 0, rsp, sizeof(rsp));
    transport_expect(OP_TIMESTAMP, cmd, sizeof(cmd), 0, rsp, sizeof(rsp));

    uint32_t actual_timestamp_us = 0;
    assert_int_equal(0, ll_timestamp_get(&actual_timestamp_us));
    assert_int_equal(0x11223344, actual_timestamp_us);

    assert_int_equal(0, ll_timestamp_get(&actual_timestamp_us));
    assert_int_equal(0x11223344, actual_timestamp_us);
}

void test_timestamp_set(void **state)
{
    (void) state;
    uint8_t cmd[] = {0x01, 0x10, 0x20, 0x30, 0x40};
    uint8_t rsp[] = {0x11, 0x22, 0x33, 0x44};

    transport_expect(OP_TIMESTAMP, cmd, sizeof(cmd), 0, rsp, sizeof(rsp));

    uint32_t actual_timestamp_us = 0;
    assert_int_equal(0, ll_timestamp_set(1, 0x10203040, &actual_timestamp_us));
    assert_int_equal(0x11223344, actual_timestamp_us);
}

void test_timestamp_set_with_nack(void **state)
{
    (void) state;
    uint8_t cmd[] = {0x01, 0x10, 0x20, 0x30, 0x40};

    transport_expect(OP_TIMESTAMP, cmd, sizeof(cmd), LL_IFC_NACK_BUSY_TRY_AGAIN, 0, 0);

    uint32_t actual_timestamp_us = 0;
    assert_int_equal(-LL_IFC_NACK_BUSY_TRY_AGAIN, ll_timestamp_set(1, 0x10203040, &actual_timestamp_us));
}

void test_nack_with_payload(void **state)
{
    (void) state;
    uint8_t cmd[] = {0x01, 0x10, 0x20, 0x30, 0x40};
    uint8_t rsp[] = {1};

    transport_expect(OP_TIMESTAMP, cmd, sizeof(cmd), LL_IFC_NACK_BUSY_TRY_AGAIN, rsp, sizeof(rsp));

    uint32_t actual_timestamp_us = 0;
    // todo : indicating a CRC error does not seem correct here
    assert_int_equal(-104, ll_timestamp_set(1, 0x10203040, &actual_timestamp_us));
}

void test_timestamp_set_with_invalid_operation(void **state)
{
    (void) state;
    //TODO: not sure what this is supposed to be testing
//    uint32_t actual_timestamp_us = 0;
//    printf("negative\n");
//    assert_int_equal(LL_IFC_ERROR_INCORRECT_PARAMETER,
//            ll_timestamp_set((ll_timestamp_operation_t) -1,
//                             0x10203040, &actual_timestamp_us));
//    printf("too big\n");
//    assert_int_equal(LL_IFC_ERROR_INCORRECT_PARAMETER,
//            ll_timestamp_set((ll_timestamp_operation_t) LL_TIMESTAMP_SYNC + 1,
//                             0x10203040, &actual_timestamp_us));
}

void test_packet_send_timestamp(void **state)
{
    (void) state;
    uint8_t cmd1[] = {0x10, 0x20, 0x30, 0x40};

    uint8_t cmd2[] = {1, 2, 3, 4, 5, 6, 7, 8};
    uint8_t rsp2[] = {0};

    transport_expect(OP_SEND_TIMESTAMP, cmd1, sizeof(cmd1), 0, 0, 0);
    transport_expect(OP_PKT_SEND_QUEUE, cmd2, sizeof(cmd2), 0, rsp2, sizeof(rsp2));
    assert_int_equal(0, ll_packet_send_timestamp(0x10203040, cmd2, sizeof(cmd2)));
}



