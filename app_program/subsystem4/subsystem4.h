#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>

// --- LSM303 I2C Hardware Definitions & Pin Mapping ---
#define MPU6050_I2C_PORT      i2c0        // Dedicated I2C0 peripheral on Pico W
#define MPU6050_SDA_PIN       4           // GPIO 4 (SDA)
#define MPU6050_SCL_PIN       5           // GPIO 5 (SCL)
#define LSM303_ACCEL_ADDR     0x19        // LSM303 Accelerometer I2C address
#define LSM303_MAG_ADDR       0x1E        // LSM303 Magnetometer I2C address

// LSM303 Accelerometer Registers
#define LSM303_CTRL_REG1_A    0x20        // Power mode and enable axes
#define LSM303_CTRL_REG4_A    0x23        // Block data update, scale selection
#define LSM303_OUT_X_L_A      0x28        // Output registers start address (with auto-increment bit 0x80)

// --- System Fault Bitmask Flags ---
#define FAULT_BIT_MOTOR_STALL (1 << 0)    
#define FAULT_BIT_ENCODER     (1 << 1)    
#define FAULT_BIT_IMU_LOST    (1 << 2)    
#define FAULT_BIT_IMPACT_TILT (1 << 3)    
#define FAULT_BIT_NO_ECHO     (1 << 4)    
#define FAULT_BIT_WIFI_LOSS   (1 << 5)    

// --- Minimum-Bit Event Types for Queue Messages ---
typedef enum {
    EVENT_HUMP_START = 1, 
    EVENT_HUMP_DONE  = 2, 
    EVENT_IMPACT     = 3, 
    EVENT_IMU_LOST   = 4  
} imu_event_type_t;

// --- Bit-Packed Event Queue Structure (16 Bits Total) ---
typedef struct __attribute__((packed)) {
    uint16_t event_id : 3;      
    uint16_t motion_state : 2;  
    uint16_t peak_height_mm : 7;
    uint16_t reserved : 4;      
} imu_packed_event_t;

// --- Bit-Packed Raw Sensor Data Sample Shared Struct ---
typedef struct {
    float accel_x_g;            
    float accel_y_g;            
    float accel_z_g;            
    float gyro_y_dps;           // Simulated from accel/encoder derivative for compatibility
    float gyro_z_dps;           // Simulated from encoder turning differential
} imu_sensor_data_t;

// --- Subsystem States Bit-Packed ---
typedef struct __attribute__((packed)) {
    uint8_t safety_state : 2;   
    uint8_t terrain_state : 2;  
    uint8_t motion_class  : 2;  
    uint8_t reserved      : 2;  
} imu_states_packed_t;

// --- Public Subsystem APIs ---
void imu_hardware_init(void);
void task_imu_safety_monitor_run_20ms(void);
void task_imu_terrain_run_20ms(uint32_t current_enc_count);

#endif // IMU_H