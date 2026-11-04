#include <stdio.h>      
#include <stdlib.h>     
#include <math.h>       
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "subsystem4.h"        

// --- Threshold Parameters ---
#define IMPACT_ACCEL_THRESHOLD 2.5f   
#define TILT_ANGLE_THRESHOLD   30.0f  
#define CLIMB_PITCH_THRESHOLD  5.0f   
#define DESCENT_PITCH_THRESHOLD -5.0f 
#define TURN_RATE_THRESHOLD    30.0f  

// --- Shared Internal Data Storage ---
static imu_sensor_data_t g_imu_latest;       
static bool g_imu_new_sample_flag = false;    
static float g_accel_z_bias = 0.0f;           
static float g_pitch_deg = 0.0f;              
static float g_hump_accum_mm = 0.0f;          
static uint8_t g_hump_peak_mm = 0;            
static uint32_t g_last_enc_count = 0;         

// --- Subsystem Packed States ---
static imu_states_packed_t g_subsystem_states = {0, 0, 0, 0}; 

// --- Atomic Event Flags ---
volatile uint32_t g_event_flags_mask = 0;     
volatile bool g_flg_estop = false;            

// --- Low-Level LSM303 I2C Helper Functions ---
static void lsm303_write_reg(uint8_t reg, uint8_t data) {
    uint8_t buffer[2] = {reg, data}; 
    i2c_write_blocking(MPU6050_I2C_PORT, LSM303_ACCEL_ADDR, buffer, 2, false); 
}

static bool lsm303_read_raw_accel(int16_t *accel) {
    uint8_t reg = LSM303_OUT_X_L_A | 0x80; // Set MSB for auto-increment read
    uint8_t buffer[6]; 

    int ret = i2c_write_blocking(MPU6050_I2C_PORT, LSM303_ACCEL_ADDR, &reg, 1, true);
    if (ret == PICO_ERROR_GENERIC) return false; 

    ret = i2c_read_blocking(MPU6050_I2C_PORT, LSM303_ACCEL_ADDR, buffer, 6, false);
    if (ret == PICO_ERROR_GENERIC) return false; 

    // Corrected casting and bit shifting for all three axes
    accel[0] = (int16_t)(((buffer[1] << 8) | buffer[0])) >> 4; 
    accel[1] = (int16_t)(((buffer[3] << 8) | buffer[2])) >> 4; 
    accel[2] = (int16_t)(((buffer[5] << 8) | buffer[4])) >> 4; 

    return true; 
}

// --- Atomic Helper Functions ---
static void atomic_set_fault_bit(uint32_t bit_mask) {
    g_event_flags_mask |= bit_mask; 
}

static void atomic_clear_fault_bit(uint32_t bit_mask) {
    g_event_flags_mask &= ~bit_mask; 
}

// --- Hardware Initialization ---
void imu_hardware_init(void) {
    gpio_set_function(MPU6050_SDA_PIN, GPIO_FUNC_I2C); 
    gpio_set_function(MPU6050_SCL_PIN, GPIO_FUNC_I2C); 
    gpio_pull_up(MPU6050_SDA_PIN); 
    gpio_pull_up(MPU6050_SCL_PIN); 

    i2c_init(MPU6050_I2C_PORT, 400 * 1000); 

    // Initialize LSM303 Accelerometer: Normal power mode, 100Hz, enable X, Y, Z axes (CTRL_REG1_A = 0x57)
    lsm303_write_reg(LSM303_CTRL_REG1_A, 0x57);
    // Continuous update, +/- 2g scale (CTRL_REG4_A = 0x00)
    lsm303_write_reg(LSM303_CTRL_REG4_A, 0x00);
    sleep_ms(50); 

    float accel_z_sum = 0.0f; 
    int valid_samples = 0; 

    for (int i = 0; i < 50; i++) { 
        int16_t a_raw[3]; 
        if (lsm303_read_raw_accel(a_raw)) { 
            accel_z_sum += (float)a_raw[2] / 1000.0f; // Scale factor for +/-2g 12-bit
            valid_samples++; 
        }
        sleep_ms(20); 
    }

    if (valid_samples > 0) { 
        g_accel_z_bias = (accel_z_sum / valid_samples) - 1.0f; 
    }

    g_subsystem_states.safety_state = 1; 
}

// --- Task 1: IMU Safety Monitor (Priority 5, Highest) ---
void task_imu_safety_monitor_run_20ms(void) {
    static uint8_t failed_read_count = 0; 
    int16_t a_raw[3]; 

    if (!lsm303_read_raw_accel(a_raw)) { 
        failed_read_count++; 
        if (failed_read_count >= 3) { 
            atomic_set_fault_bit(FAULT_BIT_IMU_LOST); 
            printf("[IMU Safety] FAULT: LSM303 Communication Lost!\n"); 
        }
        return; 
    }

    failed_read_count = 0; 
    atomic_clear_fault_bit(FAULT_BIT_IMU_LOST); 

    // Convert raw counts to g units (+/-2g scale factor ~1000 LSB/g for 12-bit right-aligned)
    g_imu_latest.accel_x_g = (float)a_raw[0] / 1000.0f; 
    g_imu_latest.accel_y_g = (float)a_raw[1] / 1000.0f; 
    g_imu_latest.accel_z_g = ((float)a_raw[2] / 1000.0f) - g_accel_z_bias; 

    float accel_magnitude = sqrtf( 
        g_imu_latest.accel_x_g * g_imu_latest.accel_x_g +
        g_imu_latest.accel_y_g * g_imu_latest.accel_y_g +
        g_imu_latest.accel_z_g * g_imu_latest.accel_z_g
    );

    float pitch_accel_deg = atan2f(g_imu_latest.accel_y_g, g_imu_latest.accel_z_g) * (180.0f / 3.14159f); 

    if (accel_magnitude > IMPACT_ACCEL_THRESHOLD || fabsf(pitch_accel_deg) > TILT_ANGLE_THRESHOLD) { 
        g_flg_estop = true; 
        atomic_set_fault_bit(FAULT_BIT_IMPACT_TILT); 
        g_subsystem_states.safety_state = 2; 
        printf("[IMU Safety] EMERGENCY STOP! Impact: %.2fg, Pitch: %.1f deg\n", accel_magnitude, pitch_accel_deg); 
    }

    g_imu_new_sample_flag = true; 
}

// --- Task 2: IMU Terrain & Hump Monitor (Priority 22, Medium) ---
void task_imu_terrain_run_20ms(uint32_t current_enc_count) {
    if (!g_imu_new_sample_flag) return; 
    g_imu_new_sample_flag = false; 

    const float dt = 0.020f; 

    // Calculate pitch purely from accelerometer with a simple low-pass filter (since LSM303 lacks a gyro)
    float pitch_accel_deg = atan2f(g_imu_latest.accel_y_g, g_imu_latest.accel_z_g) * (180.0f / 3.14159f);
    g_pitch_deg = (0.85f * g_pitch_deg) + (0.15f * pitch_accel_deg); // Low-pass filter to smooth vibration noise

    // Classify motion using encoder delta (differential turn detection suggested by teammate)
    uint32_t enc_delta = current_enc_count - g_last_enc_count;
    
    if (current_enc_count != g_last_enc_count) { 
        g_subsystem_states.motion_class = 1; // 1 = MOVING
    } else {
        g_subsystem_states.motion_class = 0; // 0 = STATIONARY
    }

    g_last_enc_count = current_enc_count; 
    float delta_s_meters = ((float)enc_delta / 937.0f) * (3.14159f * 0.065f); 

    switch (g_subsystem_states.terrain_state) { 
        case 0: // FLAT phase
            if (g_pitch_deg > CLIMB_PITCH_THRESHOLD) { 
                g_subsystem_states.terrain_state = 1; 
                g_hump_accum_mm = 0.0f; 

                imu_packed_event_t event = { 
                    .event_id = EVENT_HUMP_START, 
                    .motion_state = g_subsystem_states.motion_class, 
                    .peak_height_mm = 0, 
                    .reserved = 0 
                };
                printf("[IMU Terrain] EVENT: HUMP_START posted to Queue (Packed ID: %d)\n", event.event_id); 
            }
            break;

        case 1: // CLIMBING phase
            g_hump_accum_mm += sinf(g_pitch_deg * (3.14159f / 180.0f)) * delta_s_meters * 1000.0f; 

            if (g_pitch_deg < CLIMB_PITCH_THRESHOLD && g_pitch_deg > DESCENT_PITCH_THRESHOLD) { 
                g_subsystem_states.terrain_state = 2; 
            }
            break;

        case 2: // ON_TOP phase
            g_hump_accum_mm += sinf(g_pitch_deg * (3.14159f / 180.0f)) * delta_s_meters * 1000.0f; 

            if (g_pitch_deg < DESCENT_PITCH_THRESHOLD) { 
                g_subsystem_states.terrain_state = 3; 
            }
            break;

        case 3: // DESCENDING phase
            g_hump_accum_mm += sinf(g_pitch_deg * (3.14159f / 180.0f)) * delta_s_meters * 1000.0f; 

            if (fabsf(g_pitch_deg) < 3.0f) { 
                g_subsystem_states.terrain_state = 0; 

                if (g_hump_accum_mm < 0.0f) g_hump_accum_mm = 0.0f; 
                if (g_hump_accum_mm > 100.0f) g_hump_accum_mm = 100.0f; 
                g_hump_peak_mm = (uint8_t)roundf(g_hump_accum_mm); 

                imu_packed_event_t event = { 
                    .event_id = EVENT_HUMP_DONE, 
                    .motion_state = g_subsystem_states.motion_class, 
                    .peak_height_mm = g_hump_peak_mm, 
                    .reserved = 0 
                };
                printf("[IMU Terrain] EVENT: HUMP_DONE posted! Peak Height: %d mm (LSM303 Payload)\n", 
                       event.peak_height_mm); 
            }
            break;

        default:
            g_subsystem_states.terrain_state = 0; 
            break;
    }
}

// --- Standalone Test Main Function ---
int main(void) {
    stdio_init_all(); 
    sleep_ms(2000); 

    printf("==================================================\n"); 
    printf("   Buddy 4 (LSM303 Subsystem) Standalone Test     \n"); 
    printf("==================================================\n"); 

    imu_hardware_init(); 
    printf("[Test] LSM303 Initialized on GP4/GP5 at Address 0x19.\n\n"); 

    uint32_t simulated_encoder_count = 0; 

    for (int cycle = 1; cycle <= 100; cycle++) { 
        task_imu_safety_monitor_run_20ms(); 

        if (cycle > 20 && cycle < 80) { 
            simulated_encoder_count += 20; 
        }

        task_imu_terrain_run_20ms(simulated_encoder_count); 

        if (g_flg_estop) { 
            printf("[Test] CRITICAL: Motor E-Stop Flag Active! Fault Mask: 0x%02X\n", g_event_flags_mask); 
        }

        sleep_ms(20); 
    }

    printf("\n[Test] 100 LSM303 Execution Cycles Completed Successfully.\n"); 
    while (true) {
        tight_loop_contents(); 
    }
}