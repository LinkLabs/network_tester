
void setUpFtpInit(void **state);
void setUpFtpSegment(void **state);
void setUpFtpApply(void **state);
void tearDownFtp(void **state);

void test_ftp_INIT_init_reject(void **state);
void test_ftp_INIT_init_accept(void **state);
void test_ftp_INIT_seg_reject(void **state);
void test_ftp_INIT_seg_accept(void **state);
void test_ftp_SEGMENT_cancel_accept(void **state);
void test_ftp_SEGMENT_cancel_reject(void **state);
void test_ftp_SEGMENT_init_accept(void **state);
void test_ftp_SEGMENT_init_reject(void **state);
void test_ftp_SEGMENT_new_seg_accept_init_accept(void **state);
void test_ftp_SEGMENT_new_seg_accept_init_reject(void **state);
void test_ftp_SEGMENT_new_seg_accept_write_fail(void **state);
void test_ftp_SEGMENT_new_seg_accept_write(void **state);
void test_ftp_SEGMENT_new_seg_complete_short(void **state);
void test_ftp_SEGMENT_new_seg_complete_long(void **state);
void test_ftp_SEGMENT_new_seg_complete_long_bad_crc(void **state);
void test_ftp_SEGMENT_apply_incomplete(void **state);
void test_ftp_SEGMENT_timeout_request_segs(void **state);
void test_ftp_SEGMENT_timeout_request_segs_clear_timeout(void **state);
void test_ftp_SEGMENT_timeout_cancel(void **state);
void test_ftp_APPLY_init_accept(void **state);
void test_ftp_APPLY_init_reject(void **state);
void test_ftp_APPLY_apply_accept(void **state);
void test_ftp_APPLY_apply_reject(void **state);
void test_ftp_num_segments_get_fail(void **state);
void test_ftp_num_segments_get_success(void **state);

#define FTP_IFC_LIB_TESTS \
        unit_test_setup_teardown(test_ftp_INIT_init_reject, setUpFtpInit, tearDownFtp), \
        unit_test_setup_teardown(test_ftp_INIT_init_accept, setUpFtpInit, tearDownFtp), \
        unit_test_setup_teardown(test_ftp_INIT_seg_reject, setUpFtpInit, tearDownFtp), \
        unit_test_setup_teardown(test_ftp_INIT_seg_accept, setUpFtpInit, tearDownFtp), \
        unit_test_setup_teardown(test_ftp_SEGMENT_cancel_accept, setUpFtpSegment, tearDownFtp), \
        unit_test_setup_teardown(test_ftp_SEGMENT_cancel_reject, setUpFtpSegment, tearDownFtp), \
        unit_test_setup_teardown(test_ftp_SEGMENT_init_accept, setUpFtpSegment, tearDownFtp), \
        unit_test_setup_teardown(test_ftp_SEGMENT_init_reject, setUpFtpSegment, tearDownFtp), \
        unit_test_setup_teardown(test_ftp_SEGMENT_new_seg_accept_init_accept, setUpFtpSegment, tearDownFtp), \
        unit_test_setup_teardown(test_ftp_SEGMENT_new_seg_accept_init_reject, setUpFtpSegment, tearDownFtp), \
        unit_test_setup_teardown(test_ftp_SEGMENT_new_seg_accept_write_fail, setUpFtpSegment, tearDownFtp), \
        unit_test_setup_teardown(test_ftp_SEGMENT_new_seg_accept_write, setUpFtpSegment, tearDownFtp), \
        unit_test_setup_teardown(test_ftp_SEGMENT_new_seg_complete_short, setUpFtpSegment, tearDownFtp), \
        unit_test_setup_teardown(test_ftp_SEGMENT_new_seg_complete_long, setUpFtpSegment, tearDownFtp), \
        unit_test_setup_teardown(test_ftp_SEGMENT_new_seg_complete_long_bad_crc, setUpFtpSegment, tearDownFtp), \
        unit_test_setup_teardown(test_ftp_SEGMENT_apply_incomplete, setUpFtpSegment, tearDownFtp), \
        unit_test_setup_teardown(test_ftp_SEGMENT_timeout_request_segs, setUpFtpSegment, tearDownFtp), \
        unit_test_setup_teardown(test_ftp_SEGMENT_timeout_request_segs_clear_timeout, setUpFtpSegment, tearDownFtp), \
        unit_test_setup_teardown(test_ftp_SEGMENT_timeout_cancel, setUpFtpSegment, tearDownFtp), \
        unit_test_setup_teardown(test_ftp_APPLY_init_accept, setUpFtpApply, tearDownFtp), \
        unit_test_setup_teardown(test_ftp_APPLY_init_reject, setUpFtpApply, tearDownFtp), \
        unit_test_setup_teardown(test_ftp_APPLY_apply_accept, setUpFtpApply, tearDownFtp), \
        unit_test_setup_teardown(test_ftp_APPLY_apply_reject, setUpFtpApply, tearDownFtp), \
        unit_test(test_ftp_num_segments_get_fail), \
        unit_test(test_ftp_num_segments_get_success), \
