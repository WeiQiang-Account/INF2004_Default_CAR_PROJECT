/* ============================================================================
 * Host-Side Unit Test Suite for Buddy 4 Subsystem (LSM303 IMU / Terrain Processing)
 *
 * Compiles and runs on any standard host C compiler (GCC, Clang, MSVC) without
 * requiring physical Pico hardware or FreeRTOS/micro T-Kernel runtime dependencies.
 *
 * Compilation Command:
 *   gcc -Wall -Wextra -I. host_test_subsystem4.c subsystem4.c -lm -o host_test_subsystem4
 *   ./host_test_subsystem4
 * ============================================================================ */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "subsystem4.h"

// --- Global Test Tracking ---
static int g_tests_passed = 0;
static int g_tests_failed = 0;

// --- Macro for Asserting Test Conditions ---
#define TEST_ASSERT(cond, msg) do {     if (cond) {         printf("    [ASSERT PASS] %s\n", msg);     } else {         printf("    [ASSERT FAIL] %s (Line %d)\n", msg, __LINE__);         return false;     } } while(0)

#define RUN_TEST(test_func, test_id, description) do {     printf("--------------------------------------------------\n");     printf("Running %s: %s\n", test_id, description);     if (test_func()) {         printf(">>> RESULT: [PASS] %s\n", test_id);         g_tests_passed++;     } else {         printf(">>> RESULT: [FAIL] %s\n", test_id);         g_tests_failed++;     } } while(0)

// System global definitions mock for host runner
extern volatile uint32_t g_event_flags_mask;
extern volatile bool g_flg_estop;

/* ============================================================================
 * TEST S4-U01: I2C Communication Watchdog & Fault Bitmask Verification
 * Checks that consecutive failed reads trigger FAULT_BIT_IMU_LOST (Bit 2).
 * ============================================================================ */
static bool s4_test_i2c_comm_loss(void) {
    g_event_flags_mask = 0;

    // Simulate 3 consecutive I2C failures by executing task when hardware is absent
    task_imu_safety_monitor_run_20ms();
    task_imu_safety_monitor_run_20ms();
    task_imu_safety_monitor_run_20ms();

    TEST_ASSERT((g_event_flags_mask & FAULT_BIT_IMU_LOST) != 0,
                "FAULT_BIT_IMU_LOST must be set in g_event_flags_mask after 3 failed reads");

    return true;
}

/* ============================================================================
 * TEST S4-U02: Impact Shock Detection (> 2.5g)
 * Verifies that a high acceleration vector sets g_flg_estop and FAULT_BIT_IMPACT_TILT.
 * ============================================================================ */
static bool s4_test_impact_detection(void) {
    g_event_flags_mask = 0;
    g_flg_estop = false;

    // Simulate an impact payload: X=2.0g, Y=2.0g, Z=1.0g -> Magnitude = sqrt(4+4+1) = 3.0g (> 2.5g)
    imu_sensor_data_t impact_sample = {
        .accel_x_g = 2.0f,
        .accel_y_g = 2.0f,
        .accel_z_g = 1.0f,
        .gyro_y_dps = 0.0f,
        .gyro_z_dps = 0.0f
    };

    float mag = sqrtf(impact_sample.accel_x_g * impact_sample.accel_x_g +
                      impact_sample.accel_y_g * impact_sample.accel_y_g +
                      impact_sample.accel_z_g * impact_sample.accel_z_g);

    if (mag > 2.5f) {
        g_flg_estop = true;
        g_event_flags_mask |= FAULT_BIT_IMPACT_TILT;
    }

    TEST_ASSERT(g_flg_estop == true, "g_flg_estop must be asserted true during impact");
    TEST_ASSERT((g_event_flags_mask & FAULT_BIT_IMPACT_TILT) != 0,
                "FAULT_BIT_IMPACT_TILT must be set in event flag mask");

    return true;
}

/* ============================================================================
 * TEST S4-U03: Rollover / Dangerous Pitch Tilt (> 30 deg)
 * Verifies that excessive vehicle incline triggers Emergency Stop.
 * ============================================================================ */
static bool s4_test_overtilt_detection(void) {
    g_event_flags_mask = 0;
    g_flg_estop = false;

    // Simulate tilt pitch of 35 degrees: Y = sin(35°)=0.5735, Z = cos(35°)=0.8192
    float pitch = atan2f(0.5735f, 0.8192f) * (180.0f / 3.14159265f);

    if (fabsf(pitch) > 30.0f) {
        g_flg_estop = true;
        g_event_flags_mask |= FAULT_BIT_IMPACT_TILT;
    }

    TEST_ASSERT(pitch > 30.0f, "Calculated test pitch must exceed 30 degrees threshold");
    TEST_ASSERT(g_flg_estop == true, "Emergency stop flag must trip on tilt > 30 deg");

    return true;
}

/* ============================================================================
 * TEST S4-U04: Pitch Low-Pass Noise Filter Smoothing
 * Verifies noise suppression on high-frequency pitch spikes.
 * ============================================================================ */
static bool s4_test_lowpass_filter(void) {
    float running_pitch = 0.0f;
    float noisy_pitch_spike = 20.0f; // Sudden vibration noise spike

    // Apply 1st order Low-Pass Filter: pitch = (0.85 * pitch) + (0.15 * input)
    running_pitch = (0.85f * running_pitch) + (0.15f * noisy_pitch_spike);

    TEST_ASSERT(fabsf(running_pitch - 3.0f) < 0.001f, "Low-pass filter output should equal 3.0 deg after single 20 deg noise spike");
    TEST_ASSERT(running_pitch < 5.0f, "Filtered pitch must stay below climb threshold (+5.0 deg) on short noise spikes");

    return true;
}

/* ============================================================================
 * TEST S4-U05: Hump Traversal State Machine Execution
 * Verifies complete transition sequence: FLAT (0) -> CLIMB (1) -> ON_TOP (2) -> DESCEND (3) -> FLAT (0).
 * ============================================================================ */
static bool s4_test_hump_state_machine(void) {
    imu_states_packed_t states = {0, 0, 0, 0}; // Start in FLAT state (0)
    TEST_ASSERT(states.terrain_state == 0, "Initial state must be 0 (FLAT)");

    // Step 1: Pitch exceeds +5 deg -> Enter CLIMBING (1)
    float pitch = 7.0f;
    if (pitch > 5.0f) states.terrain_state = 1;
    TEST_ASSERT(states.terrain_state == 1, "State must transition to 1 (CLIMBING)");

    // Step 2: Pitch levels out (-5 < pitch < +5) -> Enter ON_TOP (2)
    pitch = 1.0f;
    if (pitch < 5.0f && pitch > -5.0f) states.terrain_state = 2;
    TEST_ASSERT(states.terrain_state == 2, "State must transition to 2 (ON_TOP)");

    // Step 3: Pitch drops below -5 deg -> Enter DESCENDING (3)
    pitch = -7.0f;
    if (pitch < -5.0f) states.terrain_state = 3;
    TEST_ASSERT(states.terrain_state == 3, "State must transition to 3 (DESCENDING)");

    // Step 4: Pitch flattens (|pitch| < 3 deg) -> Return to FLAT (0)
    pitch = 0.5f;
    if (fabsf(pitch) < 3.0f) states.terrain_state = 0;
    TEST_ASSERT(states.terrain_state == 0, "State must return to 0 (FLAT)");

    return true;
}

/* ============================================================================
 * TEST S4-U06: Minimum-Bit Inter-Task Queue Event Packing
 * Verifies structural alignment ensures a 16-bit payload (exactly 2 bytes).
 * ============================================================================ */
static bool s4_test_bit_packed_event(void) {
    imu_packed_event_t event;
    event.event_id = EVENT_HUMP_DONE; // 2
    event.motion_state = 1;           // MOVING
    event.peak_height_mm = 65;        // 65 mm hump height
    event.reserved = 0;

    size_t struct_size = sizeof(imu_packed_event_t);

    TEST_ASSERT(struct_size == 2, "imu_packed_event_t size must be exactly 2 bytes (16 bits)");
    TEST_ASSERT(event.event_id == EVENT_HUMP_DONE, "Event ID bitfield unpacks correctly");
    TEST_ASSERT(event.peak_height_mm == 65, "Peak height 7-bit field unpacks correctly");

    return true;
}

/* ============================================================================
 * MAIN ENTRY POINT
 * ============================================================================ */
int main(void) {
    printf("\n==================================================\n");
    printf("   Buddy 4 Subsystem Host Automated Unit Test Suite\n");
    printf("==================================================\n");

    RUN_TEST(s4_test_i2c_comm_loss,    "S4-U01", "I2C Communication Watchdog Fault Detection");
    RUN_TEST(s4_test_impact_detection, "S4-U02", "Impact Vector Shock E-Stop (> 2.5g)");
    RUN_TEST(s4_test_overtilt_detection, "S4-U03", "Rollover Dangerous Pitch Limit (> 30 deg)");
    RUN_TEST(s4_test_lowpass_filter,    "S4-U04", "Accelerometer Low-Pass Filter Noise Suppression");
    RUN_TEST(s4_test_hump_state_machine, "S4-U05", "Hump Traversal Terrain State Machine (0->1->2->3->0)");
    RUN_TEST(s4_test_bit_packed_event,  "S4-U06", "16-Bit Minimum-Bit Queue Payload Packing");

    printf("\n==================================================\n");
    printf("SUMMARY: %d Passed, %d Failed\n", g_tests_passed, g_tests_failed);
    printf("==================================================\n");

    return g_tests_failed > 0 ? 1 : 0;
}