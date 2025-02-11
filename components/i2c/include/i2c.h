/*
 *  i2c.h
 *
 *  Created on: 16 jan 2021
 *      Author: Luan Amaral
 */

#ifndef i2c_H_
#define i2c_H_

#include <stdio.h>
#include "driver/i2c.h"
#include "esp_attr.h"

class i2c
{
    private:
        //int port;
        i2c_config_t conf;

    public:
        size_t rx_buff = 0; /*!<disable */
        size_t tx_buff = 0; /*!<disable */
        i2c_port_t port;

        i2c();
        ~i2c();
        esp_err_t init(int data_pin, int clock_pin, uint32_t freq, i2c_port_t _port);
        esp_err_t write(uint8_t adress, uint8_t regist, uint8_t *data, size_t size);
        esp_err_t read(uint8_t address, uint8_t regist, uint8_t *data, size_t size);
};
#endif