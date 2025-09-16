/*
 * Delta-Sigma ADC Reader for NIOS-V
 * Reads ADC data via Avalon-MM and converts to voltage
 */

#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>
#include "system.h"
#include "io.h"
#include "alt_types.h"

 // ADC Configuration
#define ADC_BITS           16      // 16-bit ADC resolution
#define ADC_MAX_VALUE      65535   // 2^16 - 1
#define VREF               1.2f    // 1.2V reference (Agilex 5 I/O standard)

// ADC register offsets
#define ADC_DATA_REG       0x00    // ADC data register
#define ADC_STATUS_REG     0x04    // Status register  

// Status register bits
#define ADC_DATA_READY     0x01    // Data ready flag
#define ADC_VALID          0x02    // ADC valid flag

// ADC base address - From Platform Designer and VHDL address decoder
// Platform Designer maps bridge at 0x00100000-0x0013ffff
// VHDL addr_decoder maps ADC at relative offset 0x0000-0x0FFF
#define ADC_AVALON_BASE    0x00100000  // Bridge base (ADC at offset 0x0000)

// About module base address (from VHDL address decoder)
#define ABOUT_BASE_ADDR    0x00101000  // Bridge base + 0x1000

// About module register offsets (from about_pkg.vhd)
#define ABOUT_IMAGE_TYPE      0x00
#define ABOUT_IMAGE_ID        0x04  
#define ABOUT_REV_MAJOR       0x08
#define ABOUT_REV_MINOR       0x0C
#define ABOUT_REV_PATCH       0x10
#define ABOUT_REV_BUILDNUMBER 0x14
#define ABOUT_GIT_HASH_MSB    0x18
#define ABOUT_GIT_HASH_LSB    0x1C
#define ABOUT_GIT_DIRTY       0x20
#define ABOUT_YYMMDD          0x24
#define ABOUT_HHMMSS          0x28
#define ABOUT_ADC_FEATURES    0x2C

// Heartbeat timing (in microseconds)
#define HEARTBEAT_INTERVAL_US   2000000    // 2 seconds

float adc_to_voltage(uint16_t adc_raw) {
    // Convert raw ADC value to voltage
    return ((float)adc_raw / (float)ADC_MAX_VALUE) * VREF;
}

void print_about_info(void) {
    printf("\n=== System Information ===\n");

    uint32_t image_type = IORD_32DIRECT(ABOUT_BASE_ADDR, ABOUT_IMAGE_TYPE);
    uint32_t image_id = IORD_32DIRECT(ABOUT_BASE_ADDR, ABOUT_IMAGE_ID);
    uint32_t rev_major = IORD_32DIRECT(ABOUT_BASE_ADDR, ABOUT_REV_MAJOR);
    uint32_t rev_minor = IORD_32DIRECT(ABOUT_BASE_ADDR, ABOUT_REV_MINOR);
    uint32_t rev_patch = IORD_32DIRECT(ABOUT_BASE_ADDR, ABOUT_REV_PATCH);
    uint32_t build_num = IORD_32DIRECT(ABOUT_BASE_ADDR, ABOUT_REV_BUILDNUMBER);
    uint32_t build_date = IORD_32DIRECT(ABOUT_BASE_ADDR, ABOUT_YYMMDD);
    uint32_t build_time = IORD_32DIRECT(ABOUT_BASE_ADDR, ABOUT_HHMMSS);
    uint32_t adc_features = IORD_32DIRECT(ABOUT_BASE_ADDR, ABOUT_ADC_FEATURES);

    printf("Project Type: 0x%04X ", image_type);
    if (image_type == 0xADCF) {
        printf("(ADC FPGA)\n");
    }
    else {
        printf("(Unknown)\n");
    }

    printf("Project ID: 0x%04X ", image_id);
    if (image_id == 0x5000) {
        printf("(AXE5000)\n");
    }
    else {
        printf("(Unknown)\n");
    }

    printf("Version: %lu.%lu.%lu (Build %lu)\n", rev_major, rev_minor, rev_patch, build_num);
    printf("Build Date: %06lX (%02lu/%02lu/20%02lu)\n",
        build_date,
        (build_date >> 8) & 0xFF,   // Month
        build_date & 0xFF,          // Day  
        (build_date >> 16) & 0xFF); // Year
    printf("Build Time: %06lX (%02lu:%02lu:%02lu)\n",
        build_time,
        (build_time >> 16) & 0xFF, // Hour
        (build_time >> 8) & 0xFF,  // Minute
        build_time & 0xFF);        // Second

    printf("ADC Features: 0x%08lX\n", adc_features);
    if (adc_features & 0x01) {
        printf("  - Delta-Sigma ADC Enabled\n");
    }
}

void adc_init(void) {
    printf("Delta-Sigma ADC Reader Started\n");
    printf("==============================\n");

    // Print system information first
    print_about_info();

    printf("\n=== ADC Configuration ===\n");
    printf("ADC Config: %d-bit, %.1fV reference\n", ADC_BITS, VREF);
    printf("Base Address: 0x%08X\n", ADC_AVALON_BASE);
    printf("Resolution: %.1f µV per LSB\n", (VREF * 1000000.0f) / ADC_MAX_VALUE);
}

uint16_t adc_read_data(void) {
    uint32_t data = IORD_32DIRECT(ADC_AVALON_BASE, ADC_DATA_REG);
    return (uint16_t)(data & 0xFFFF);
}

uint32_t adc_read_status(void) {
    return IORD_32DIRECT(ADC_AVALON_BASE, ADC_STATUS_REG);
}

void send_heartbeat(void) {
    static uint32_t heartbeat_count = 0;
    printf("Active (heartbeat #%lu)\n", heartbeat_count++);
}

int main(void) {
    uint16_t adc_value;
    uint32_t status;
    uint32_t sample_count = 0;

    // Timing variables for heartbeat
    clock_t last_heartbeat = clock();
    clock_t current_time;

    // Initialize ADC
    adc_init();

    printf("Starting main loop...\n");
    printf("- ADC samples will be displayed when available\n");
    printf("- Heartbeat message every %d seconds\n", HEARTBEAT_INTERVAL_US / 1000000);
    printf("- Press Ctrl+C to stop\n\n");

    while (1) {
        current_time = clock();

        // Check for heartbeat timing (approximately every second)
        if (((current_time - last_heartbeat) * 1000000 / CLOCKS_PER_SEC) >= HEARTBEAT_INTERVAL_US) {
            send_heartbeat();
            last_heartbeat = current_time;
        }

        // Check if new ADC data is available
        status = adc_read_status();

        if (status & ADC_DATA_READY) {
            // Read ADC data (clears data ready flag automatically)
            adc_value = adc_read_data();

            // Convert to voltage
            float voltage = adc_to_voltage(adc_value);

            // Send data to UART with voltage conversion
            printf("Sample %lu: Raw=0x%04X (%d) | Voltage=%.3fV\n",
                sample_count++, adc_value, adc_value, voltage);

            // Short delay after processing sample
            usleep(1000);   // 1ms delay
        }
        else {
            // No new data, very short delay to prevent busy waiting
            usleep(100);    // 100us delay
        }
    }

    return 0;
}
