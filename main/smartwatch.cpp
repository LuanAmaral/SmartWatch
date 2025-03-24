#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "MAX30102.h"
#include "i2c.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" 
{
    void app_main(void);
}
#endif

MAX30102 pulse_oximeter;
i2c i2c_module;
SemaphoreHandle_t i2c_mutex = xSemaphoreCreateMutex();

void measure_spo2_hr_task(void *pvParameter);

void app_main(void)
{   
    // Configure i2c communication
    i2c_module.init(21, 22, 200000, I2C_NUM_0);
    pulse_oximeter.init(&i2c_module, &i2c_mutex);

    xTaskCreate(
        measure_spo2_hr_task, 
        "Measure SPO2 and HR", 
        8192, 
        NULL, 
        1, 
        NULL);
}


void measure_spo2_hr_task(void *pvParameter)
{   
    MAX30102_data_t data;
    esp_err_t err = pulse_oximeter.config();
    if (err != ESP_OK)
    {
        ESP_LOGE("Pulse Oximeter", "Error configuring MAX30102");
        for(;;) vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    pulse_oximeter.wake();
    pulse_oximeter.reset();
    ESP_LOGI("Pulse Oximeter", "Measuring SPO2 and Heart Rate");
    while (1)
    {
        data = pulse_oximeter.measure_data();
        ESP_LOGI("Pulse Oximeter", "SPO2: %ld, Heart Rate: %ld", data.spo2, data.heart_rate);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}