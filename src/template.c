
#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "tkjhat/sdk.h"

// Default stack size for the tasks. It can be reduced to 1024 if task is not using lot of memory.
#define DEFAULT_STACK_SIZE 2048
#define buffer_len 1024

//Add here necessary states
enum state { IDLE=1, RECEIVING, TRANSMITTING };
enum state programState = IDLE;

uint8_t buffer[buffer_len];

// Käsittelijäfunktio
void uartFxn() {
    uart_getc(uart0); // Luetaan vastaanotettu merkki
}

// Taskifunktio
void uartTask (void *pvParams) {
    // Alustetaan UART0
    uart_init(uart0, 9600);

    gpio_set_function(0, GPIO_FUNC_UART); // TX
    gpio_set_function(1, GPIO_FUNC_UART); // RX

    // Asetetaan keskeytyksen käsittelijä uartFxn UART0:n keskeytyksille
    irq_set_exclusive_handler(UART0_IRQ, uartFxn);
    // Vastaanotetaan keskeytyksiä
    irq_set_enabled(UART0_IRQ, true);

    // Kerrotaan UART:lle, että halutaan vastaanottaa keskeytyksiä, kun dataa on luettavissa
    uart_set_irq_enables(uart0, true, false);


    while(1) {
        // Ikuinen loop
        tight_loop_contents();
    }
}

int main() {
    stdio_init_all();
    // Uncomment this lines if you want to wait till the serial monitor is connected
    /*while (!stdio_usb_connected()){
        sleep_ms(10);
    }*/ 
    init_hat_sdk();
    sleep_ms(300); //Wait some time so initialization of USB and hat is done.

    TaskHandle_t myExampleTask = NULL;
    // Create the tasks with xTaskCreate
    BaseType_t result = xTaskCreate(uartTask,       // (en) Task function
                "uartTask",              // (en) Name of the task 
                DEFAULT_STACK_SIZE, // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,               // (en) Arguments of the task 
                2,                  // (en) Priority of this task
                &myExampleTask);    // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("Example Task creation failed\n");
        return 0;
    }

    // Start the scheduler (never returns)
    vTaskStartScheduler();

    // Never reach this line.
    return 0;
}

