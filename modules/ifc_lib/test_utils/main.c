#include "test_ifc_lib_utils.h"

#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include "cmockery.h"


int main(int argc, char* argv[]) {
    (void) argc;
    (void) argv;

    srand(time(NULL));

    UnitTest tests[] = {
        UTILS_IFC_LIB_TESTS
    };

    return run_tests(tests);
}
