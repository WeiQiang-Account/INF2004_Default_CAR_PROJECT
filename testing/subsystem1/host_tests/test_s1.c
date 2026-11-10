/* PC test for Buddy 1's pure logic: make -C testing/subsystem1/host_tests */
#include <stdio.h>
#include "s1_selftest.h"
#include "s1_json.h"

static int fails;
static void check(const char *id, bool ok, const char *what)
{
	printf("[HOST] %s %s  %s\n", id, ok ? "PASS" : "FAIL", what);
	if (!ok) { fails++; }
}

int main(void)
{
	char b[512];
	s1_snapshot_t s = {0};
	uint16_t n;

	check("S1-U01", s1_st_telemetry(), "telemetry JSON: flat, design fields, longest case < 700 B");
	check("S1-U02", s1_st_parse(),     "command parser: good / unknown / missing seq / too long");
	check("S1-U03", s1_st_rules(),     "state rules: start READY, resume+cancel PAUSED, reset FAULT+still, stop any");
	check("S1-U04", s1_st_events(),    "events once each, watchdog reset first, cmd result");
	check("S1-U05", s1_st_small_buf(), "too-small buffer gives 0, never half a JSON");

	s.vc_state = S1_VC_RUNNING; s.obs_mm = 0xFFFFu; s.bar = -1;
	n = s1_json_telemetry(b, sizeof b, &s);
	printf("example telemetry (%u bytes): %.*s\n", (unsigned)n, (int)n, b);
	{
		static const struct { uint8_t st, cause, mc, ir; } ex[] = {
			{ S1_VC_INIT, 0, 0, 0 }, { S1_VC_READY, 0, 0, 1 }, { S1_VC_RUNNING, 0, 0, 1 },
			{ S1_VC_FAULT, 7, 2, 1 }, { S1_VC_FAULT, 1, 0, 1 }, { S1_VC_FAULT, 3, 0, 1 } };
		unsigned i;
		for (i = 0; i < sizeof ex / sizeof ex[0]; i++) {
			s1_snapshot_t z = {0};
			z.vc_state = ex[i].st; z.vc_cause = ex[i].cause; z.m_cause = ex[i].mc; z.ir_cal = ex[i].ir;
			z.m_state = 1; z.imu_state = 1; z.spd_l = 198; z.spd_r = 203; z.line = 5;
			n = s1_status_text(b, sizeof b, &z);
			printf("status: %.*s\n", (int)n, b);
		}
	}
	printf("%d failed\n", fails);
	return fails;
}
