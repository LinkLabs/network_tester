#include "ll_ifc_utils.h"

void setUpRandom(void **state);
void tearDown(void **state);

void test_utils_difftime_random_positive(void **state);
void test_utils_difftime_random_negative(void **state);
void test_utils_difftime_limits(void **state);
void test_utils_difftime_now_random_positive(void **state);
void test_utils_difftime_now_random_negative(void **state);
void test_utils_difftime_now_limits(void **state);

#define UTILS_IFC_LIB_TESTS \
        unit_test_setup_teardown(test_utils_difftime_random_positive, setUpRandom, tearDown), \
        unit_test_setup_teardown(test_utils_difftime_random_negative, setUpRandom, tearDown), \
        unit_test(test_utils_difftime_limits), \
        unit_test(test_utils_difftime_now_limits)


//        unit_test_setup_teardown(test_utils_difftime_now_random_positive, setUpRandom, tearDown), \
//        unit_test_setup_teardown(test_utils_difftime_now_random_negative, setUpRandom, tearDown), \
