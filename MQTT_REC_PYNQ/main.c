#include <libpynq.h>
#include <stdio.h>
#include <stdint.h>

int main(void)
{
    pynq_init();

    // UART setup
    switchbox_set_pin(IO_AR0, SWB_UART0_RX);
    switchbox_set_pin(IO_AR1, SWB_UART0_TX);

    uart_init(UART0);
    uart_reset_fifos(UART0);

    sleep_msec(3000); // allow ESP to boot

    while (1)
    {
        if (uart_has_data(UART0))
        {

            uint32_t len = 0;

            for (int i = 0; i < 4; i++)
            {
                while (!uart_has_data(UART0));
                uint8_t byte = uart_recv(UART0);
                len |= (byte << (8 * i));   // little-endian
            }

            fprintf(stderr, "LEN = %u\n", len);

            uint32_t j;

            for (j = 0; j < len; j++) 
            {
                while (!uart_has_data(UART0));
                char c = uart_recv(UART0);
                fputc(c, stderr);
            }

            fputc('\n', stderr);
            fprintf(stderr, "MESSAGE RECIVED!\n");
            fflush(stderr);
        }

        sleep_msec(10);
    }

    pynq_destroy();
    return 0;
}