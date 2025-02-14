#ifndef PULSE_OXIMETER_H
#define PULSE_OXIMETER_H

#include "i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Pulse Oximeter MAX30102 Registers
// #define PUL_OXI_ADDR (uint8_t)0b1010111
// #define PUL_OXI_WRT_ADD 0xAE
// #define PUL_OXI_RD_ADD 0xAF
#define PUL_OXI_INT_STAT1 0x00
#define PUL_OXI_INT_STAT2 0x01
#define PUL_OXI_INT_ENABLE1 0x02
#define PUL_OXI_INT_ENABLE2 0x03
#define PUL_OXI_FIFO_WR_PTR 0x04
#define PUL_OXI_OVF_COUNTER 0x05
#define PUL_OXI_FIFO_RD_PTR 0x06
#define PUL_OXI_FIFO_DATA 0x07
#define PUL_OXI_FIFO_CONFIG 0x08
#define PUL_OXI_MODE_CONFIG 0x09
#define PUL_OXI_SPO2_CONFIG 0x0A
#define PUL_OXI_LED1_PA 0x0C
#define PUL_OXI_LED2_PA 0x0D
#define PUL_OXI_MULTI_LED_CTRL1 0x11
#define PUL_OXI_MULTI_LED_CTRL2 0x12
#define PUL_OXI_TEMP_INT 0x1F
#define PUL_OXI_TEMP_FRAC 0x20
#define PUL_OXI_TEMP_CONFIG 0x21


// Pulse Oximeter MAX30102 Configuration
#define PUL_OXI_FIFO_ROLLOVER_EN 0b1 // FIFO Rollover enabled
#define PUL_OXI_FIFO_ROLLOVER_DIS 0b0 // FIFO Rollover disabled
#define PUL_OXI_SHDN_OFF 0b0 // Not in shutdown mode
#define PUL_OXI_SHDN_ON 0b1 // In shutdown mode
#define PUL_OXI_RESET_OFF 0b0 // Not reset
#define PUL_OXI_RESET_ON 0b1 // Reset

// Pulse Oximeter MAX30102 FIFO Configuration
#define PUL_OXI_FIFO_A_FULL 0b000 // FIFO Almost Full 0

// Pulse Oximeter MAX30102 Sample Averaging
#define PUL_OXI_SMP_AVE_1 0b000 // No averaging
#define PUL_OXI_SMP_AVE_2 0b001 // 2 samples averaged
#define PUL_OXI_SMP_AVE_4 0b010 // 4 samples averaged
#define PUL_OXI_SMP_AVE_8 0b011 // 8 samples averaged
#define PUL_OXI_SMP_AVE_16 0b100 // 16 samples averaged
#define PUL_OXI_SMP_AVE_32 0b101 // 32 samples averaged

// MAX30102 Mode Configuration
#define PUL_OXI_MODE_SPO2 0b011 // SPO2 Mode
#define PUL_OXI_MODE_HR 0b010 // Heart Rate Mode
#define PUL_OXI_MODE_BOTH 0b111 // Multi-LED Mode

// Pulse Oximeter MAX30102 ADC Range
#define PUL_OXI_ADC_RGE_2048 0b00 // 2048 nA
#define PUL_OXI_ADC_RGE_4096 0b01 // 4096 nA
#define PUL_OXI_ADC_RGE_8192 0b10 // 8192 nA
#define PUL_OXI_ADC_RGE_16384 0b11 // 16384 nA

// Pulse Oximeter MAX30102 Sample Rate
#define PUL_OXI_SPO2_SR_50 0b000 // 50 SPS
#define PUL_OXI_SPO2_SR_100 0b001 // 100 SPS
#define PUL_OXI_SPO2_SR_200 0b010 // 200 SPS
#define PUL_OXI_SPO2_SR_400 0b011 // 400 SPS
#define PUL_OXI_SPO2_SR_800 0b100 // 800 SPS
#define PUL_OXI_SPO2_SR_1000 0b101 // 1000 SPS
#define PUL_OXI_SPO2_SR_1600 0b110 // 1600 SPS
#define PUL_OXI_SPO2_SR_3200 0b111 // 3200 SPS

// Pulse Oximeter MAX30102 LED Pulse Width
#define PUL_OXI_LED_PW_69 0b00 // 69 us
#define PUL_OXI_LED_PW_118 0b01 // 118 us
#define PUL_OXI_LED_PW_215 0b10 // 215 us
#define PUL_OXI_LED_PW_411 0b11 // 411 us

// Pulse Oximeter MAX30102 LED Current
#define PUL_OXI_LED_CURR_0 0x00 // 0 mA
#define PUL_OXI_LED_CURR_25 0x7F // 25.4 mA
#define PUL_OXI_LED_CURR_50 0xFF // 51 mA

// Pulse Oximeter MAX30102 Multi-LED Mode Control
#define PUL_OXI_SLOT1 0b000 // Slot 1
#define PUL_OXI_SLOT2 0b001 // Slot 2
#define PUL_OXI_SLOT3 0b010 // Slot 3
#define PUL_OXI_SLOT4 0b011 // Slot 4

// Pulse Oximeter MAX30102 Temperature Configuration
#define PUL_OXI_TEMP_EN 0b1 // Temperature Enable
#define PUL_OXI_TEMP_DIS 0b0 // Temperature Disable

class PulseOximeter
{
    
private:
    /* data */
    i2c* i2c_module;     
    SemaphoreHandle_t *i2c_mutex;

    enum PUL_OXI_DEV_ADDR {
        PUL_OXI_ADDR = 0x57,
        PUL_OXI_WRT_ADD = 0xAE,
        PUL_OXI_RD_ADD = 0xAF
    };   

    esp_err_t i2c_read(PUL_OXI_DEV_ADDR device_addres, uint8_t reg,uint8_t* buffer,size_t data_size);
	esp_err_t i2c_write(PUL_OXI_DEV_ADDR device_addr, uint8_t reg,uint8_t* buffer,size_t data_size);

public:
    PulseOximeter( i2c *i2c_dev, SemaphoreHandle_t *i2c_mutex);
    esp_err_t config();
    esp_err_t reset();
    ~PulseOximeter();
};

#endif // PULSE_OXIMETER_H