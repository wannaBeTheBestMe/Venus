#include <libpynq.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define MAX_LEN 200

void send_message(char *msg)
{
    uint32_t len = strlen(msg);

    uart_send(UART0, (len) & 0xFF);
    uart_send(UART0, (len >> 8) & 0xFF);
    uart_send(UART0, (len >> 16) & 0xFF);
    uart_send(UART0, (len >> 24) & 0xFF);

    for (uint32_t i = 0; i < len; i++)
        uart_send(UART0, msg[i]);

    fprintf(stderr, "Sent: %s\n", msg);
}

int main(void)
{
    pynq_init();

    switchbox_set_pin(IO_AR0, SWB_UART0_RX);
    switchbox_set_pin(IO_AR1, SWB_UART0_TX);

    uart_init(UART0);
    uart_reset_fifos(UART0);

    sleep_msec(5000);

    uint32_t len = 0;
    uint32_t bytes_read = 0;
    char buffer[MAX_LEN];

    char input[100];

    while (1)
    {
        // =====================
        //  RECEIVE (non-blocking)
        // =====================
        if (bytes_read < 4)
        {
            // reading length
            while (uart_has_data(UART0) && bytes_read < 4)
            {
                uint8_t byte = uart_recv(UART0);
                len |= (byte << (8 * bytes_read));
                bytes_read++;
            }

            if (bytes_read == 4)
            {
                if (len > MAX_LEN)
                {
                    fprintf(stderr, "Invalid LEN: %u\n", len);
                    len = 0;
                    bytes_read = 0;
                }
            }
        }
        else
        {
            // reading payload
            uint32_t payload_bytes = bytes_read - 4;

            while (uart_has_data(UART0) && payload_bytes < len)
            {
                buffer[payload_bytes++] = uart_recv(UART0);
                bytes_read++;
            }

            if (payload_bytes == len)
            {
                buffer[len] = '\0';
                fprintf(stderr, "Received: %s\n", buffer);

                // reset for next message
                len = 0;
                bytes_read = 0;
            }
        }

        // =====================
        // SEND (non-blocking input)
        // =====================
        if (fgets(input, sizeof(input), stdin) != NULL)
        {
            input[strcspn(input, "\n")] = 0;
            send_message(input);
        }

        sleep_msec(10); // small delay
    }

    pynq_destroy();
    return 0;
}