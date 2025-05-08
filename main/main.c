#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"

#include "Fusion.h"
#define SAMPLE_PERIOD (0.05f) 

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;

QueueHandle_t xQueuePos;
typedef struct pos {
    int axis;
    float val;
} pos_t;

static void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {

    uint8_t buffer[6];

    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true); 
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }


    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);  

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }


    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);  

    *temp = buffer[0] << 8 | buffer[1];
}


void mpu6050_task(void *p) {
    // configuracao do I2C
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    mpu6050_reset();
    int16_t acceleration[3], gyro[3], temp;

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    while (1) {
        pos_t pos_data;

        mpu6050_read_raw(acceleration, gyro, &temp);
        FusionVector gyroscope = {
            .axis.x = gyro[0] / 131.0f,
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };

        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f,
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        };

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));


        if (euler.angle.yaw != 0) {
            pos_data.axis = 0;
            pos_data.val = euler.angle.yaw;
            xQueueSend(xQueuePos, &pos_data, 0);
        }

        if (euler.angle.roll != 0) {
            pos_data.axis = 1;
            pos_data.val = euler.angle.roll;
            xQueueSend(xQueuePos, &pos_data, 0);
        }
        if (acceleration[1] < -25000) {
            pos_data.axis = 2;
            pos_data.val = 0.0f;
            xQueueSend(xQueuePos, &pos_data, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void uart_task(void *p) {
    pos_t data;
    while (true) {
        if (xQueueReceive(xQueuePos, &data, 1e6)) {
            uint8_t axis = (uint8_t)data.axis;
            uint16_t val = -1 * (uint16_t)((int)data.val & 0xFFFF);
            uint8_t msb = (val >> 8) & 0xFe;
            uint8_t lsb = val & 0xFe;
            uint8_t end = 0xFF;

            uint8_t pacote[4] = {axis, msb, lsb, end};
            uart_write_blocking(uart0, pacote, 4);
        }
    }
}
int main() {
    stdio_init_all();
    uart_init(uart0, 115200);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);

    xQueuePos = xQueueCreate(32, sizeof(pos_t));

    if (xQueuePos == NULL)
        printf("erro \n");

    xTaskCreate(mpu6050_task, "mpu6050_Task 1", 8192, NULL, 1, NULL);
    xTaskCreate(uart_task, "UART Task", 4095, NULL, 1, NULL);

    vTaskStartScheduler();
    while (true)
        ;
}