/* PC test for Buddy 4's pure logic: make -C testing/subsystem4/host_tests */
#include <stdio.h>
#include "s4_selftest.h"

static int fails;
static void check(const char *id, bool ok, const char *what)
{
	printf("[HOST] %s %s  %s\n", id, ok ? "PASS" : "FAIL", what);
	if (!ok) { fails++; }
}

int main(void)
{
	check("S4-U01", s4_st_math(),        "sqrt / sin / atan2 without libm");
	check("S4-U02", s4_st_calibration(), "static offsets; moving robot refused");
	check("S4-U03", s4_st_safety(),      "impact on 1 sample, tilt after 60 ms, release after 1 s, slope is not a tilt");
	check("S4-U04", s4_st_pitch(),       "pitch sign and complementary filter");
	check("S4-U05", s4_st_hump(),        "20 / 10 / 23 mm (peak, no flat top) humps to +-1 mm, run maximum kept");
	check("S4-U06", s4_st_hump_reject(), "40 ms spike ignored, long climb = ramp");
	check("S4-U07", s4_st_class(),       "stationary / moving / turning / accelerating, held 60 ms");
	printf("%d failed\n", fails);
	return fails;
}
