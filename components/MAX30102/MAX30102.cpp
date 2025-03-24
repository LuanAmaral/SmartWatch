#include "MAX30102.h"

MAX30102::MAX30102()
{
}

MAX30102::~MAX30102()
{
}

esp_err_t MAX30102::init(i2c *i2c_module, SemaphoreHandle_t *i2c_mutex)
{
    this->i2c_module = i2c_module;
    this->i2c_mutex = i2c_mutex;


    config_data.INT_ENABLE_1.a_full_en = 1;
    config_data.INT_ENABLE_1.ppg_rdy_en = 1;
    config_data.INT_ENABLE_1.alc_ovf_en = 0;
    config_data.INT_ENABLE_1.pwr_int_en = 0;
    config_data.INT_ENABLE_2.die_temp_rdy_en = 0;

    config_data.FIFO_CONFIG.fifo_a_full = MAX30102_FIFO_A_FULL;
    config_data.FIFO_CONFIG.fifo_roll_over = MAX30102_FIFO_ROLLOVER_EN;
    config_data.FIFO_CONFIG.smp_ave = MAX30102_SMP_AVE_4;

    config_data.MODE_CONFIG.mode = MAX30102_MODE_BOTH;
    config_data.MODE_CONFIG.reset = MAX30102_RESET_OFF;
    config_data.MODE_CONFIG.shdn = MAX30102_SHDN_OFF;

    config_data.SPO2_CONFIG.led_pw = MAX30102_LED_PW_69;
    config_data.SPO2_CONFIG.spo2_sr = MAX30102_SPO2_SR_100;
    config_data.SPO2_CONFIG.spo2_adc_rge = MAX30102_ADC_RGE_16384;
    config_data.SPO2_CONFIG.led_pw = MAX30102_LED_PW_215;

    config_data.led1_pa = MAX30102_LED_CURR_25;
    config_data.led2_pa = MAX30102_LED_CURR_25;

    config_data.MULTI_LED_CTRL1.slot1 = MAX30102_SLOT1;
    config_data.MULTI_LED_CTRL1.slot2 = MAX30102_SLOT2;
    config_data.MULTI_LED_CTRL2.slot3 = MAX30102_SLOT3;
    config_data.MULTI_LED_CTRL2.slot4 = MAX30102_SLOT4;

    config_data.TEMP_CONFIG.temp_en = MAX30102_TEMP_EN;

    return ESP_OK;
}

esp_err_t MAX30102::i2c_read(uint8_t reg, uint8_t *buffer, size_t data_size)
{
    esp_err_t ret;
    xSemaphoreTake(*(this->i2c_mutex), portMAX_DELAY);
    ret = i2c_module->read(MAX30102_ADDR, reg, buffer, data_size);
    xSemaphoreGive(*(this->i2c_mutex));
    return ret;
}

esp_err_t MAX30102::i2c_write(uint8_t reg, uint8_t *buffer, size_t data_size)
{
    esp_err_t ret;
    ESP_LOGI(TAG,"Attempting to write to register 0x%02X", reg); 
    ESP_LOGI(TAG,"Data size: %d", data_size);
    ESP_LOG_BUFFER_HEX(TAG, buffer, data_size);

    xSemaphoreTake(*(this->i2c_mutex), portMAX_DELAY);
    ret = i2c_module->write(MAX30102_ADDR, reg, buffer, data_size);
    xSemaphoreGive(*(this->i2c_mutex));

    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error writing to register 0x%02X", reg);
        return ret;
    }
    ESP_LOGI(TAG, "Write successful");
    return ret;
}

int32_t MAX30102::rms_value(int32_t *led_data)
{
    int32_t sum = 0;
    for (int i = 0; i < MAX30102_BUFFER_SIZE; i++)
    {
        sum += led_data[i] * led_data[i];
    }
    return sqrt(sum / MAX30102_BUFFER_SIZE);
}

int32_t MAX30102::mean_value(int32_t *led_data)
{
    int32_t sum = 0;
    for (int i = 0; i < MAX30102_BUFFER_SIZE; i++)
    {
        sum += led_data[i];
    }
    return sum / MAX30102_BUFFER_SIZE;
}

esp_err_t MAX30102::remove_dc_offset(int32_t *led_data, int32_t mean)
{
    for (int i = 0; i < MAX30102_BUFFER_SIZE; i++)
    {
        led_data[i] -= mean;
    }
    return ESP_OK;
}

int32_t MAX30102::measure_threshold(int32_t *led_data)
{
    int32_t mean = mean_value(led_data);
    if (mean < 30) mean = 30;
    if (mean > 60) mean = 60;
    return mean;
}

int8_t MAX30102::find_peak(int32_t *led_data, int32_t threshold, int32_t *peaks_location, int32_t min_dist)
{
    int8_t peak = 0;
    for (int i = 0; i < MAX30102_BUFFER_SIZE; i++)
    {
        if (led_data[i] > threshold && i - peak > min_dist)
        {
            peak++;
            peaks_location[peak] = i;
        }
    }
    return peak;
}

esp_err_t MAX30102::clear_fifo()
{
    uint8_t tx = 0b00000000;
    esp_err_t err =  i2c_write(MAX30102_FIFO_WR_PTR, &tx, 1);
    if (err != ESP_OK) return err;
    err = i2c_write(MAX30102_OVF_COUNTER, &tx, 1);
    if (err != ESP_OK) return err;
    return i2c_write(MAX30102_FIFO_RD_PTR, &tx, 1);
}

esp_err_t MAX30102::read_fifo()
{
    uint8_t rx[6];
    esp_err_t err;
    err = clear_fifo();
    if (err != ESP_OK)
    {
        ESP_LOGE("MAX30102", "Error clearing FIFO");
        return err;
    }
    for (int i = 0; i < MAX30102_BUFFER_SIZE; i++)
    {
        err = i2c_read(MAX30102_FIFO_DATA, rx, 6);
        ESP_LOG_BUFFER_HEX("MAX30102", rx, 6);
        if (err != ESP_OK)
        {
            ESP_LOGE("MAX30102", "Error reading from MAX30102_FIFO_DATA");
            return err;
        }
        ir_led_data[i] = (rx[0] << 16) | (rx[1] << 8) | rx[2];
        red_led_data[i] = (rx[3] << 16) | (rx[4] << 8) | rx[5];
        printf("IR: %ld,\t RED: %ld\n", ir_led_data[i], red_led_data[i]);
        vTaskDelay(MAX30102_SAMP_DELAY / portTICK_PERIOD_MS);
    }
    return ESP_OK;
}


esp_err_t MAX30102::config()
{
    esp_err_t err;

    //  Config interrupt
    err = i2c_write(MAX30102_INT_ENABLE1, &config_data.int_enable_1, 1);
    if (err != ESP_OK){
        ESP_LOGE("MAX30102", "Error writing to MAX30102_INT_ENABLE");
        return err;
    } 

    err = i2c_write(MAX30102_INT_ENABLE2, &config_data.int_enable_2, 1);
    if (err != ESP_OK){
        ESP_LOGE("MAX30102", "Error writing to MAX30102_INT_ENABLE2");
        return err;
    }

    // Config FIFO
    err = i2c_write(MAX30102_FIFO_CONFIG, &config_data.fifo_config, 1);
    if (err != ESP_OK){
        ESP_LOGE("MAX30102", "Error writing to MAX30102_FIFO_CONFIG");
        return err;
    }

    // Config Mode
    err = i2c_write(MAX30102_MODE_CONFIG, &config_data.mode_config, 1);
    if (err != ESP_OK){
        ESP_LOGE("MAX30102", "Error writing to MAX30102_MODE_CONFIG");
        return err;
    }

    err = i2c_write(MAX30102_SPO2_CONFIG, &config_data.spo2_config, 1);
    if (err != ESP_OK){
        ESP_LOGE("MAX30102", "Error writing to MAX30102_SPO2_CONFIG");
        return err;
    }

    // Config LED
    err = i2c_write(MAX30102_LED1_PA, &config_data.led1_pa, 1);
    if (err != ESP_OK){
        ESP_LOGE("MAX30102", "Error writing to MAX30102_LED1_PA");
        return err;
    }
    err = i2c_write(MAX30102_LED2_PA, &config_data.led2_pa, 1);
    if (err != ESP_OK){
        ESP_LOGE("MAX30102", "Error writing to MAX30102_LED2_PA");
        return err;
    }

    // Config Multi-LED Mode
    err = i2c_write(MAX30102_MULTI_LED_CTRL1, &config_data.multi_led_ctrl1, 1);
    if (err != ESP_OK){
        ESP_LOGE("MAX30102", "Error writing to MAX30102_MULTI_LED_CTRL1");
        return err;
    }

    err = i2c_write(MAX30102_MULTI_LED_CTRL2, &config_data.multi_led_ctrl2, 1);
    if (err != ESP_OK){
        ESP_LOGE("MAX30102", "Error writing to MAX30102_MULTI_LED_CTRL2");
        return err;
    }

    // Config Temperature
    err = i2c_write(MAX30102_TEMP_CONFIG, &config_data.temp_config, 1);
    if (err != ESP_OK){
        ESP_LOGE("MAX30102", "Error writing to MAX30102_TEMP_CONFIG");
        return err;
    }    

    return ESP_OK;
}

esp_err_t MAX30102::reset()
{   
    uint8_t rx;
    esp_err_t err = i2c_read(MAX30102_MODE_CONFIG, &rx, 1);
    if (err != ESP_OK) return err;

    uint8_t tx = rx | 0b01000000;
    return i2c_write(MAX30102_MODE_CONFIG, &tx, 1);
}

esp_err_t MAX30102::sleep()
{
    uint8_t rx;
    esp_err_t err = i2c_read(MAX30102_MODE_CONFIG, &rx, 1);
    if (err != ESP_OK) return err;

    uint8_t tx = rx | 0b10000000;
    return i2c_write(MAX30102_MODE_CONFIG, &tx, 1);
}

esp_err_t MAX30102::wake()
{
    uint8_t rx;
    esp_err_t err = i2c_read(MAX30102_MODE_CONFIG, &rx, 1);
    if (err != ESP_OK) return err;

    uint8_t tx = rx & 0b01111111;
    return i2c_write(MAX30102_MODE_CONFIG, &tx, 1);
}

int32_t MAX30102::measure_heart_rate(uint8_t peaks, int32_t *peaks_location)
{
    int32_t heart_rate = 0;
    int32_t time = 0;
    if (peaks > 1)
    {
        for (int i = 1; i < peaks; i++)
        {
            time += peaks_location[i] - peaks_location[i - 1];
        }
        time = time / (peaks - 1);
        heart_rate = 60000 / (time * MAX30102_SAMP_DELAY);
    }
    else { heart_rate = -1; }

    return heart_rate;
}

int32_t MAX30102::measure_spo2()
{
    int32_t red_led_mean = mean_value(red_led_data);
    int32_t ir_led_mean = mean_value(ir_led_data);
    int32_t red_led_rms = rms_value(red_led_data);
    int32_t ir_led_rms = rms_value(ir_led_data);

    if (red_led_mean == 0 || ir_led_mean == 0 || ir_led_rms == 0)
    {
        return -1;
    }
    int32_t spo2 = 110 - 25 * (red_led_rms / red_led_mean) / (ir_led_rms / ir_led_mean);
    return spo2;
}

int32_t MAX30102::measure_temperature()
{ 
    uint8_t int_temp;
    uint8_t dec_temp;

    i2c_read(MAX30102_TEMP_INT, &int_temp, 1);
    i2c_read(MAX30102_TEMP_FRAC, &dec_temp, 1);

    int32_t temp = 100*int_temp + 10*dec_temp; // 2 decimal places
    return temp;
}

MAX30102_data_t MAX30102::measure_data()
{
    read_fifo();

    int32_t peaks_location[MAX30102_BUFFER_SIZE];
    int32_t threshold = measure_threshold(ir_led_data);
    uint8_t peaks = find_peak(ir_led_data, threshold, peaks_location, MAX30102_PEAK_MIN_DIST);

    data.spo2 = measure_spo2();
    data.heart_rate = measure_heart_rate(peaks, peaks_location);
    data.temperature = measure_temperature();
    return data;
}