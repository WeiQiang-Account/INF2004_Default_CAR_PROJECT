/*
 *	subsystem1_test.c
 *	On-target tests for subsystem 1.  Built only with: make TEST=1
 *	Record results in testing/subsystem1/TEST_LOG.md.
 *
 *	S1-U01..U05 run the SAME checks as the PC host test (s1_selftest.c).
 *	S1-U06..U08 use the live kernel side. They never send START, so the
 *	robot cannot drive off during the test. MQTT itself is checked by hand
 *	(S1-I01.. in TEST_LOG.md) with an MQTT build and MQTT Explorer.
 */
#include "subsystem1.h"
#include "s1_bridge.h"
#include "s1_selftest.h"
#include "../test_report.h"

#if APP_TEST

IMPORT UW   s1_accepted_count(void);
IMPORT UW   s1_rejected_count(void);
IMPORT void s1_reset_seq_for_test(void);
IMPORT int  s5_hook_remote_poll(void);

LOCAL uint16_t slen(const char *s) { uint16_t i = 0; while (s[i] != '\0') { i++; } return i; }

EXPORT void subsystem1_test(void)
{
	char		reply[112];
	static s1_snapshot_t snap;	/* static: 1 KB initial-task stack */
	static char	big[1024];
	UW		a0, r0;

	TEST_INIT();

	/* ---- pure logic (the same code the PC test checks) ---- */
	TEST_CHECK("S1-U01", s1_st_telemetry(), "telemetry JSON: flat, design fields, < 960 B");
	TEST_CHECK("S1-U02", s1_st_parse(),     "command parser: good / unknown / missing seq / too long");
	TEST_CHECK("S1-U03", s1_st_rules(),     "state rules for start / stop / resume / cancel / reset");
	TEST_CHECK("S1-U04", s1_st_events(),    "events reported once, watchdog reset first");
	TEST_CHECK("S1-U05", s1_st_small_buf(), "too-small buffer gives 0");

	/* ---- live ---- */
	TEST_CHECK("S1-U06", flg_remote_cmd > 0, "remote command event flag created");

	/* duplicate seq is refused; a refused command never reaches the VC */
	s1_reset_seq_for_test();
	a0 = s1_accepted_count();
	r0 = s1_rejected_count();
	{
		const char *stop = "{\"cmd\":\"stop\",\"seq\":5}";
		const char *bad  = "{\"cmd\":\"dance\",\"seq\":6}";
		(void)s1_cmd_submit(stop, slen(stop), reply, sizeof(reply));
		(void)s1_cmd_submit(stop, slen(stop), reply, sizeof(reply));	/* replay */
		(void)s1_cmd_submit(bad,  slen(bad),  reply, sizeof(reply));
		(void)s5_hook_remote_poll();					/* drop anything left */
	}
	TEST_CHECK("S1-U07", (s1_accepted_count() - a0 == 1u) && (s1_rejected_count() - r0 == 2u),
		   "seq replay and unknown command rejected (cmd_rejected event)");
	s1_reset_seq_for_test();

	/* the live snapshot builds a telemetry message that fits */
	s1_snapshot_take(&snap);
	{
		uint16_t n = s1_json_telemetry(big, sizeof(big), &snap);
		tm_printf((UB*)"[TEST] S1-U08 info: %d bytes: %s\n", (INT)n, (UB*)((n > 0u) ? (big[n] = '\0', big) : "-"));
		TEST_CHECK("S1-U08", n > 0u, "live snapshot -> telemetry JSON");
	}

	TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
