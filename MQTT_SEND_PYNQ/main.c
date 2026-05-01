#include <libpynq.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

void send_message(char *msg)
{
    // Add +1 to include the string null-terminator (\0) in the payload
    uint32_t msg_size = 0;
    msg_size = strlen(msg)+1; 

    // Send length (4 bytes, LITTLE-endian)
    uart_send(UART0, (msg_size) & 0xFF);
    uart_send(UART0, (msg_size >> 8) & 0xFF);
    uart_send(UART0, (msg_size >> 16) & 0xFF);
    uart_send(UART0, (msg_size >> 24) & 0xFF);

    // Send payload (including the null terminator)
    for (uint32_t i = 0; i < msg_size; i++) {
        uart_send(UART0, msg[i]);
    }

    // Debug output
    fprintf(stderr, "Sent: %s\n", msg);
}

    char INPUT[100];
int main(void)
{
    pynq_init();

    // Correct UART routing
    switchbox_set_pin(IO_AR0, SWB_UART0_RX);
    switchbox_set_pin(IO_AR1, SWB_UART0_TX);

    uart_init(UART0);
    uart_reset_fifos(UART0);
    // Give the Wi-Fi bridge a second to boot
    sleep_msec(5000); 
    
    while(1)
    {
       // Send a command the UI actually understands

      int status_2 = gpio_get_level(IO_AR2);
      printf("status AR2: %d\n",status_2);

      printf("INPUT: ");
      scanf(" %s",INPUT);

       send_message(INPUT);
       sleep_msec(500);
    }

    pynq_destroy();
    return 0;
}