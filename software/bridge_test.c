/*
 * Memory-Mapped Module Test for NIOS-V
 * Tests ADC and About modules via Avalon-MM Clock Crossing Bridge
 */

#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>
#include "system.h"
#include "io.h"
#include "alt_types.h"

// Base addresses from Platform Designer
// Bridge maps to 0x00020000, modules are offset from there
#define BRIDGE_BASE       0x00020000

// Module base addresses (as configured in address decoder)  
#define ADC_BASE_ADDR     (BRIDGE_BASE + 0x0000)    // ADC at base
#define ABOUT_BASE_ADDR   (BRIDGE_BASE + 0x1000)    // About at 4KB offset

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

// ADC placeholder registers
#define ADC_DATA_REG      0x00
#define ADC_STATUS_REG    0x04

void print_about_info(void) {
    printf("\n=== About Module Information ===\n");
    
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
    } else {
        printf("(Unknown)\n");
    }
    
    printf("Project ID: 0x%04X ", image_id);
    if (image_id == 0x5000) {
        printf("(AXE5000)\n");
    } else {
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
    printf("ADC Features: 0x%04X\n", adc_features);
    if (adc_features & 0x0001) {
        printf("  - Delta-Sigma ADC Enabled\n");
    }
}

void test_adc_module(void) {
    printf("\n=== ADC Module Test ===\n");
    
    uint32_t adc_data = IORD_32DIRECT(ADC_BASE_ADDR, ADC_DATA_REG);
    uint32_t adc_status = IORD_32DIRECT(ADC_BASE_ADDR, ADC_STATUS_REG);
    
    printf("ADC Data: 0x%08lX\n", adc_data);
    printf("ADC Status: 0x%08lX\n", adc_status);
}

void module_connectivity_test(void) {
    printf("\n=== Module Connectivity Test ===\n");
    printf("Testing memory-mapped access via Clock Crossing Bridge...\n");
    
    // Test multiple reads to verify stability
    printf("About module stability test (5 reads):\n");
    for (int i = 0; i < 5; i++) {
        uint32_t image_type = IORD_32DIRECT(ABOUT_BASE_ADDR, ABOUT_IMAGE_TYPE);
        printf("  Read %d: Image Type = 0x%04X\n", i+1, image_type);
        usleep(100000); // 100ms delay
    }
    
    printf("ADC module stability test (5 reads):\n");
    for (int i = 0; i < 5; i++) {
        uint32_t adc_data = IORD_32DIRECT(ADC_BASE_ADDR, ADC_DATA_REG);
        printf("  Read %d: ADC Data = 0x%08X\n", i+1, adc_data);
        usleep(100000); // 100ms delay
    }
}

int main() {
    printf("Delta-Sigma ADC Project - Module Integration Test\n");
    printf("==================================================\n");
    printf("Bridge Base: 0x%08X\n", BRIDGE_BASE);
    printf("ADC Base: 0x%08X\n", ADC_BASE_ADDR);
    printf("About Base: 0x%08X\n", ABOUT_BASE_ADDR);
    
    // Test the modules
    print_about_info();
    test_adc_module();
    module_connectivity_test();
    
    printf("\n=== Starting Continuous Monitoring ===\n");
    
    uint32_t loop_count = 0;
    while (1) {
        printf("\n--- Loop %lu ---\n", loop_count++);
        
        // Read about info briefly
        uint32_t version = IORD_32DIRECT(ABOUT_BASE_ADDR, ABOUT_REV_MAJOR);
        printf("About Version: %lu\n", version);
        
        // Read ADC data
        uint32_t adc_data = IORD_32DIRECT(ADC_BASE_ADDR, ADC_DATA_REG);
        printf("ADC Data: 0x%08X\n", adc_data);
        
        sleep(2); // 2 second delay
    }
    
    return 0;
}
