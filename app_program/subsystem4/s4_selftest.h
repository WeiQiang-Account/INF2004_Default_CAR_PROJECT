/* s4_selftest.h - checks of Buddy 4's pure logic, shared by the on-target test
 * (subsystem4_test.c) and the PC host test (testing/subsystem4/host_tests). */
#ifndef S4_SELFTEST_H
#define S4_SELFTEST_H
#include <stdbool.h>
bool s4_st_math(void);          /* S4-U01 */
bool s4_st_calibration(void);   /* S4-U02 */
bool s4_st_safety(void);        /* S4-U03 */
bool s4_st_pitch(void);         /* S4-U04 */
bool s4_st_hump(void);          /* S4-U05 */
bool s4_st_hump_reject(void);   /* S4-U06 */
bool s4_st_class(void);         /* S4-U07 */
#endif
