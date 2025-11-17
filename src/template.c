
#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <tusb.h>

#include "tkjhat/sdk.h"

// Default stack size for the tasks. It can be reduced to 1024 if task is not using lot of memory.
#define DEFAULT_STACK_SIZE 2048
#define buffer_len 1024
#define UNIT_MS 100
#define BUZZER_FREQUENCY 600

//Add here necessary states
enum state { IDLE=1, RECEIVING, TRANSMITTING };
enum state programState = IDLE;

uint8_t buffer[buffer_len];

static void btn_fxn(uint gpio, uint32_t eventMask) {
    if (gpio == BUTTON1) {
        if (programState == IDLE) {
            programState = TRANSMITTING;
        }
        else {
            programState = IDLE;
        }
    }
    toggle_led(); // Kertoo onko nappia painettu ja onko tilassa TRANSMITTING/IDLE
}

void ledFxn(bool isDot) {
    if (isDot) {
        // Väläytetään lediä lyhyesti YHDEN "unit of timen" verran (100ms)
        toggle_red_led();
        sleep_ms(UNIT_MS);
        toggle_red_led();
        sleep_ms(UNIT_MS);
    }
    else {
        // Väläytetään lediä pitkästi KOLMEN "unit of timen" verran (300ms)
        toggle_red_led();
        sleep_ms(3*UNIT_MS);
        toggle_red_led();
        sleep_ms(UNIT_MS);
    }
}

void buzzerFnx(bool isDot) {
    if (isDot) {
        // Piipataan buzzeria lyhyesti YHDEN "unit of timen" verran (100ms)
        buzzer_play_tone(BUZZER_FREQUENCY, UNIT_MS);
        sleep_ms(UNIT_MS);
    }
    else {
        // Piipataan buzzeria pitkästi KOLMEN "unit of timen" verran (300ms)
        buzzer_play_tone(BUZZER_FREQUENCY, 3*UNIT_MS);
        sleep_ms(UNIT_MS);
    }
}


//
/* static void print_task(void *arg) {
    (void)arg;

    float ax, ay, az, gx, gy, gz, t;
    while(1) {
        if (programState == TRANSMITTING) {
            if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                if (ay < -200) {
                    // lähetetään piste
                    usb_serial_print(".");
                    ledFxn(true);
                    buzzerFnx(true);
                }
                else if (ay > 200) {
                    // lähetetään viiva
                    usb_serial_print("-");
                    ledFxn(true);
                    buzzerFnx(true);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
} */

// ---- Task running USB stack ----
static void usbTask(void *arg) {
    (void)arg;
    while (1) {
        tud_task();              // With FreeRTOS wait for events
                                 // Do not add vTaskDelay. 
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

    init_red_led();
    printf("Initializing red led\r\n");
    init_buzzer();
    printf("Initializing buzzer\r\n");

    /* TaskHandle_t printTask = NULL;
    // Create the tasks with xTaskCreate
    BaseType_t result = xTaskCreate(print_task,       // (en) Task function
                "print_task",              // (en) Name of the task 
                DEFAULT_STACK_SIZE, // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,               // (en) Arguments of the task 
                2,                  // (en) Priority of this task
                &printTask);    // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("Example Task creation failed\n");
        return 0;
    } */

    // Start the scheduler (never returns)
    vTaskStartScheduler();

    // Never reach this line.
    return 0;
}

