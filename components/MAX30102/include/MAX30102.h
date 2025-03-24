#ifndef MAX30102_H_
#define MAX30102_H_

#include "i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "math.h"
#include "esp_log.h"

// MAX30102 Registers
#define MAX30102_ADDR 0x57
#define MAX30102_INT_STAT1 0x00
#define MAX30102_INT_STAT2 0x01
#define MAX30102_INT_ENABLE1 0x02
#define MAX30102_INT_ENABLE2 0x03
#define MAX30102_FIFO_WR_PTR 0x04
#define MAX30102_OVF_COUNTER 0x05
#define MAX30102_FIFO_RD_PTR 0x06
#define MAX30102_FIFO_DATA 0x07
#define MAX30102_FIFO_CONFIG 0x08
#define MAX30102_MODE_CONFIG 0x09
#define MAX30102_SPO2_CONFIG 0x0A
#define MAX30102_LED1_PA 0x0C
#define MAX30102_LED2_PA 0x0D
#define MAX30102_MULTI_LED_CTRL1 0x11
#define MAX30102_MULTI_LED_CTRL2 0x12
#define MAX30102_TEMP_INT 0x1F
#define MAX30102_TEMP_FRAC 0x20
#define MAX30102_TEMP_CONFIG 0x21

// MAX30102 Configuration
#define MAX30102_FIFO_ROLLOVER_EN 0b1 // FIFO Rollover enabled
#define MAX30102_FIFO_ROLLOVER_DIS 0b0 // FIFO Rollover disabled
#define MAX30102_SHDN_OFF 0b0 // Not in shutdown mode
#define MAX30102_SHDN_ON 0b1 // In shutdown mode
#define MAX30102_RESET_OFF 0b0 // Not reset
#define MAX30102_RESET_ON 0b1 // Reset

// MAX30102 FIFO Configuration
#define MAX30102_FIFO_A_FULL 0b0000 // FIFO Almost Full 0

// MAX30102 Sample Averaging
#define MAX30102_SMP_AVE_1 0b000 // No averaging
#define MAX30102_SMP_AVE_2 0b001 // 2 samples averaged
#define MAX30102_SMP_AVE_4 0b010 // 4 samples averaged
#define MAX30102_SMP_AVE_8 0b011 // 8 samples averaged
#define MAX30102_SMP_AVE_16 0b100 // 16 samples averaged
#define MAX30102_SMP_AVE_32 0b101 // 32 samples averaged

// MAX30102 Mode Configuration
#define MAX30102_MODE_SPO2 0b011 // SPO2 Mode
#define MAX30102_MODE_HR 0b010 // Heart Rate Mode
#define MAX30102_MODE_BOTH 0b111 // Multi-LED Mode

// MAX30102 LED ADC Range
#define MAX30102_ADC_RGE_2048 0b00 // 2048 nA
#define MAX30102_ADC_RGE_4096 0b01 // 4096 nA
#define MAX30102_ADC_RGE_8192 0b10 // 8192 nA
#define MAX30102_ADC_RGE_16384 0b11 // 16384 nA

// MAX30102 SPO2 Sample Rate
#define MAX30102_SPO2_SR_50 0b000 // 50 SPS
#define MAX30102_SPO2_SR_100 0b001 // 100 SPS
#define MAX30102_SPO2_SR_200 0b010 // 200 SPS
#define MAX30102_SPO2_SR_400 0b011 // 400 SPS
#define MAX30102_SPO2_SR_800 0b100 // 800 SPS
#define MAX30102_SPO2_SR_1000 0b101 // 1000 SPS
#define MAX30102_SPO2_SR_1600 0b110 // 1600 SPS
#define MAX30102_SPO2_SR_3200 0b111 // 3200 SPS

// MAX30102 LED Pulse Width
#define MAX30102_LED_PW_69 0b00 // 69 us
#define MAX30102_LED_PW_118 0b01 // 118 us
#define MAX30102_LED_PW_215 0b10 // 215 us
#define MAX30102_LED_PW_411 0b11 // 411 us

// MAX30102 LED Current
#define MAX30102_LED_CURR_0 0x00 // 0 mA
#define MAX30102_LED_CURR_25 0x7F // 25.4 mA
#define MAX30102_LED_CURR_50 0xFF // 51 mA

// MAX30102 Multi-LED Mode Control
#define MAX30102_SLOT1 0b001 // Slot 1
#define MAX30102_SLOT2 0b010 // Slot 2
#define MAX30102_SLOT3 0b000 // Slot 3
#define MAX30102_SLOT4 0b000 // Slot 4

// MAX30102 Temperature Configuration
#define MAX30102_TEMP_EN 0b1 // Temperature enabled
#define MAX30102_TEMP_DIS 0b0 // Temperature disabled

// MAX30102 Data Buffer Size
#define MAX30102_BUFFER_SIZE 32

#define MAX30102_HR_THRESHOLD 100
#define MAX30102_SAMP_DELAY 40
#define MAX30102_PEAK_MIN_DIST 4

typedef struct MAX30102_data
{
    int32_t spo2;
    int32_t heart_rate;
    int32_t temperature;
} MAX30102_data_t;

typedef struct MAX30102_config
{
    union 
    {
        uint8_t int_enable_1;
        struct 
        {
            uint8_t pwr_int_en: 1;
            uint8_t reserved: 4;
            uint8_t alc_ovf_en: 1;
            uint8_t ppg_rdy_en: 1;
            uint8_t a_full_en: 1;
        } INT_ENABLE_1;
    };
    union 
    {
        uint8_t int_enable_2;
        struct 
        {
            uint8_t reserved: 1;
            uint8_t die_temp_rdy_en: 1;
            uint8_t reserved2: 6;
        } INT_ENABLE_2;
    };
    union 
    {
        uint8_t fifo_config;
        struct 
        {
            uint8_t fifo_a_full: 4;
            uint8_t fifo_roll_over: 1;
            uint8_t smp_ave: 3;
        } FIFO_CONFIG;
    };
    union 
    {
        uint8_t mode_config;
        struct 
        {
            uint8_t mode: 3;
            uint8_t reserved: 3;
            uint8_t reset: 1;
            uint8_t shdn: 1;
        } MODE_CONFIG;
    };
    union 
    {
        uint8_t spo2_config;
        struct 
        {
            uint8_t led_pw: 2;
            uint8_t spo2_sr: 3;
            uint8_t spo2_adc_rge: 2;
            uint8_t reserved: 1;
        } SPO2_CONFIG;
    };
    uint8_t led1_pa;
    uint8_t led2_pa;
    union 
    {
        uint8_t multi_led_ctrl1;
        struct 
        {
            uint8_t slot1: 3;
            uint8_t reserved: 1;
            uint8_t slot2: 3;
            uint8_t reserved2: 1;
        } MULTI_LED_CTRL1;
    };
    union 
    {
        uint8_t multi_led_ctrl2;
        struct 
        {
            uint8_t slot3: 3;
            uint8_t reserved: 1;
            uint8_t slot4: 3;
            uint8_t reserved2: 1;
        } MULTI_LED_CTRL2;
    };
    union 
    {
        uint8_t temp_config;
        struct 
        {
            uint8_t temp_en: 1;
            uint8_t reserved: 7;
        } TEMP_CONFIG;
    };
} MAX30102_config_t;


#define  TAG "MAX30102"

class MAX30102
{
    private:
        i2c *i2c_module;
        SemaphoreHandle_t *i2c_mutex;

        int32_t ir_led_data[MAX30102_BUFFER_SIZE];
        int32_t red_led_data[MAX30102_BUFFER_SIZE];

        MAX30102_data_t data;
        MAX30102_config_t config_data;

        esp_err_t i2c_read(uint8_t reg, uint8_t *buffer, size_t data_size);
        esp_err_t i2c_write(uint8_t reg, uint8_t *buffer, size_t data_size);
        int32_t rms_value(int32_t *led_data);
        int32_t mean_value(int32_t *led_data);
        esp_err_t remove_dc_offset(int32_t *led_data, int32_t mean);
        int32_t measure_threshold(int32_t *led_data);
        int8_t find_peak(int32_t *led_data, int32_t threshold, int32_t *peaks_location, int32_t min_dist);
        esp_err_t clear_fifo();
        esp_err_t read_fifo();
        int32_t measure_heart_rate(uint8_t peaks, int32_t *peaks_location);
        int32_t measure_spo2();
        int32_t measure_temperature();
    public:
        MAX30102();
        ~MAX30102();
        esp_err_t init(i2c *i2c_module, SemaphoreHandle_t *i2c_mutex);
        esp_err_t config();
        esp_err_t reset();
        esp_err_t sleep();
        esp_err_t wake();
        MAX30102_data_t measure_data();
};

#endif /* MAX30102_H_ */

