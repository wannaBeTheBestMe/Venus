#include <libpynq.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <sys/select.h> // Added for non-blocking stdin
#include <unistd.h>     // Added for STDIN_FILENO

#define MAX_LEN 200

void send_message(char *msg)
{
    char buffer[MAX_LEN];
    
    // FIX 1: Add the newline back so the Wi-Fi bridge flushes immediately over MQTT
    snprintf(buffer, sizeof(buffer), "%s\n", msg);
    uint32_t len = strlen(buffer);

    uart_send(UART0, (len) & 0xFF);
    uart_send(UART0, (len >> 8) & 0xFF);
    uart_send(UART0, (len >> 16) & 0xFF);
    uart_send(UART0, (len >> 24) & 0xFF);

    for (uint32_t i = 0; i < len; i++)
        uart_send(UART0, buffer[i]);

    // FIX 2: Force the terminal to draw the text instantly
    fprintf(stderr, "Sent: %s", buffer);
    fflush(stderr); 
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
                    fflush(stderr);
                    len = 0;
                    bytes_read = 0;
                }
            }
        }
        else
        {
            uint32_t payload_bytes = bytes_read - 4;

            while (uart_has_data(UART0) && payload_bytes < len)
            {
                buffer[payload_bytes++] = uart_recv(UART0);
                bytes_read++;
            }

            if (payload_bytes == len)
            {
                buffer[len] = '\0';
                
                // Clean up any stray newlines from the sender for a pretty terminal
                buffer[strcspn(buffer, "\r\n")] = 0; 
                
                fprintf(stderr, "Received: %s\n", buffer);
                fflush(stderr); // Force instant update

                len = 0;
                bytes_read = 0;
            }
        }

        // =====================
        // SEND (TRULY non-blocking input)
        // =====================
        
        // FIX 3: Set up select() to peek at the keyboard without stopping the loop
        fd_set readfds;
        struct timeval tv;
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);

        tv.tv_sec = 0;
        tv.tv_usec = 0; // A 0 timeout means "check instantly and keep moving"

        // select() returns > 0 ONLY if you have pressed Enter
        if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv) > 0)
        {
            if (fgets(input, sizeof(input), stdin) != NULL)
            {
                input[strcspn(input, "\n")] = 0; // Strip it so send_message can safely re-format it
                send_message(input);
            }
        }

        sleep_msec(10); // Prevent CPU maxing
    }

    pynq_destroy();
    return 0;
}
