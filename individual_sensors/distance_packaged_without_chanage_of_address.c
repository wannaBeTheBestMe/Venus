#include <libpynq.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define VL53L0X_ADDR 0x29

#define SYSRANGE_START       0x00
#define RESULT_RANGE_STATUS  0x14

#define CALIBRATION_OFFSET_MM  -27

int main(void)
{
    printf("VL53L0X Start\n");

    pynq_init();

    switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
    switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);

    iic_init(IIC0);

    /*
     * Reset offset register corrupted by earlier writes
     */
    uint8_t zero = 0x00;
    iic_write_register(IIC0, VL53L0X_ADDR, 0x28, &zero, 1);

    /*
     * Start continuous ranging
     */
    uint8_t start = 0x01;

    if (iic_write_register(IIC0,
                           VL53L0X_ADDR,
                           SYSRANGE_START,
                           &start,
                           1))
    {
        printf("Failed to start sensor\n");

        pynq_destroy();

        return EXIT_FAILURE;
    }

    printf("Sensor running...\n");

    while (1)
    {
        uint8_t start = 0x01;

        iic_write_register(IIC0,
                           VL53L0X_ADDR,
                           0x00,
                           &start,
                           1);

        sleep_msec(50);

        uint8_t data[2];

        if (iic_read_register(IIC0,
                              VL53L0X_ADDR,
                              0x1E,
                              data,
                              2))
        {
            printf("Read error\n");
        }
        else
        {
            int32_t distance =
                (int32_t)((data[0] << 8) | data[1]);

            distance += CALIBRATION_OFFSET_MM;

            if (distance < 0) distance = 0;

            printf("Distance: %d mm\n",
                   (int)distance);
        }

        uint8_t clear = 0x01;

        iic_write_register(IIC0,
                           VL53L0X_ADDR,
                           0x0B,
                           &clear,
                           1);

        sleep_msec(100);
    }

    pynq_destroy();

    return EXIT_SUCCESS;
}
