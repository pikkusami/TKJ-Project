
#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include <tusb.h>
#include "usbSerialDebug/helper.h"
#include "tkjhat/sdk.h"

//#if CFG_TUSB_OS != OPT_OS_FREERTOS
//#error "This should be using FREERTOS but the CFG_TUSB_OS is not OPT_OS_FREERTOS"
//#endif

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
            printf("Switched to TRANSMITTING");
        }
        else {
            programState = IDLE;
            printf("Switched to IDLE");
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

// Print example task
static void print_task(void *pvParameters) {
    (void)pvParameters;

    float ax, ay, az, gx, gy, gz, t;
    // Setting up the sensor. 
    if (init_ICM42670() == 0) {
        printf("ICM-42670P initialized successfully!\n");
        if (ICM42670_start_with_default_values() != 0){
            printf("ICM-42670P could not initialize accelerometer or gyroscope");
        }
    } else {
        printf("Failed to initialize ICM-42670P.\n");
    }

    bool luettu = false;
    while(1) {
        if (programState == TRANSMITTING) {
            if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                sprintf(buffer,"Accel: X=%.2f, Y=%.2f, Z=%.2f | Gyro: X=%.2f, Y=%.2f, Z=%.2f| Temp: %2.2f°C\n", ax, ay, az, gx, gy, gz, t);
                printf(buffer);
                if (ax < -0.7 && luettu == false) {
                    // lähetetään piste
                    printf(".");
                    ledFxn(true);
                    buzzerFnx(true);
                    luettu = true;
                } else if (ax > 0.7 && luettu == false) {
                    // lähetetään viiva
                    printf("-");
                    ledFxn(true);
                    buzzerFnx(true);
                    luettu = true;
                } else if (ax > 0.7 && luettu == false) {
                    // lähetetään väli
                    printf(" ");
                    ledFxn(true);
                    buzzerFnx(true);
                    luettu = true;
                } else if (ax > -0.7 && ax < 0.7) {
                    luettu = false;
                }
            } else {
                printf("Failed to read data");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

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
    while (!stdio_usb_connected()){
        sleep_ms(10);
    }

    init_hat_sdk();
    sleep_ms(300); //Wait some time so initialization of USB and hat is done.

    init_buzzer();
    printf("Initializing buzzer\r\n");
    init_button1();
    init_button2();
    printf("Initializing buttons\r\n");
    init_led();
    init_red_led();
    printf("Initializing leds\r\n");

    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_RISE, true, btn_fxn);
    gpio_set_irq_enabled(BUTTON2, GPIO_IRQ_EDGE_RISE, true);

    TaskHandle_t printTask, hUsb = NULL;
    // Create the tasks with xTaskCreate
    BaseType_t result = xTaskCreate(print_task,       // (en) Task function
                "print_task",              // (en) Name of the task 
                DEFAULT_STACK_SIZE, // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,               // (en) Arguments of the task 
                2,                  // (en) Priority of this task
                &printTask);    // (en) A handle to control the execution of this task

    xTaskCreate(usbTask, "usb", 1024, NULL, 3, &hUsb);
    #if (configNUMBER_OF_CORES > 1)
        vTaskCoreAffinitySet(hUsb, 1u << 0);
    #endif

    // VERY IMPORTANT, THIS SHOULD GO JUST BEFORE vTaskStartSheduler
    // WITHOUT ANY DELAYS. OTHERWISE, THE TinyUSB stack wont recognize
    // the device.
    // Initialize TinyUSB 
    tusb_init();

    usb_serial_init();

    // Start the scheduler (never returns)
    vTaskStartScheduler();

    // Never reach this line.
    return 0;
}

