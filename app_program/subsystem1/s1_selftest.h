/* s1_selftest.h - checks of subsystem 1's PURE logic (s1_json.c).
 * Run on the robot by subsystem1_test.c (make TEST=1) AND on a PC by
 * testing/subsystem1/host_tests, so both mean the same thing.            */
#ifndef S1_SELFTEST_H
#define S1_SELFTEST_H
#include <stdbool.h>
bool s1_st_telemetry(void);	/* S1-U01 */
bool s1_st_parse(void);		/* S1-U02 */
bool s1_st_rules(void);		/* S1-U03 */
bool s1_st_events(void);	/* S1-U04 */
bool s1_st_small_buf(void);	/* S1-U05 */
#endif
