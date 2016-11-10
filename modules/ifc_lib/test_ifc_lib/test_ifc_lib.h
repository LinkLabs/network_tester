
void setUp(void **state);
void tearDown(void **state);

void test_myself_transport_write(void **state);
void test_timestamp_get(void **state);
void test_timestamp_get_with_response_error(void **state);
void test_timestamp_get_twice_to_check_message_identifier(void **state);
void test_timestamp_set(void **state);
void test_timestamp_set_with_nack(void **state);
void test_nack_with_payload(void **state);
void test_timestamp_set_with_invalid_operation(void **state);
void test_packet_send_timestamp(void **state);

#define GEN_IFC_LIB_TESTS \
        unit_test_setup_teardown(test_myself_transport_write, setUp, tearDown), \
        unit_test_setup_teardown(test_timestamp_get, setUp, tearDown), \
        unit_test_setup_teardown(test_timestamp_get_with_response_error, setUp, tearDown), \
        unit_test_setup_teardown(test_timestamp_get_twice_to_check_message_identifier, setUp, tearDown), \
        unit_test_setup_teardown(test_timestamp_set, setUp, tearDown), \
        unit_test_setup_teardown(test_timestamp_set_with_nack, setUp, tearDown), \
        unit_test_setup_teardown(test_nack_with_payload, setUp, tearDown), \
        unit_test_setup_teardown(test_timestamp_set_with_invalid_operation, setUp, tearDown), \
        unit_test_setup_teardown(test_packet_send_timestamp, setUp, tearDown),

