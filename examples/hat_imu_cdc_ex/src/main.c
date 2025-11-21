#include <stdio.h>
#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include <tusb.h>
#include "usbSerialDebug/helper.h"
#include <tkjhat/sdk.h>

#if CFG_TUSB_OS != OPT_OS_FREERTOS
#error "This should be using FREERTOS but the CFG_TUSB_OS is not OPT_OS_FREERTOS"
#endif

#define DEFAULT_STACK_SIZE 2048
#define buffer_len 100
#define UNIT_MS 100
#define BUZZER_FREQUENCY_HIGH 600
#define BUZZER_FREQUENCY_LOW 300
#define BUZZER_FREQUENCY_SEND 1000

char buffer[buffer_len];
int currentIndex = 0;

enum state { IDLE=1, WRITING, MSG_READY };
enum state programState = IDLE;

static void btn_fxn(uint gpio, uint32_t eventMask) {
    if (gpio == BUTTON1) {
        if (programState == IDLE) {
            programState = WRITING;
            usb_serial_print("Switched to WRITING\n");
            toggle_red_led();
        }
        else {
            programState = IDLE;
            usb_serial_print("Switched to IDLE\n");
            toggle_red_led();
        }
    }
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

void buzzerFnx(int numero_bzr) {
    if (numero_bzr == 0) {
        // Piipataan buzzeria lyhyesti YHDEN "unit of timen" verran (100ms)
        buzzer_play_tone(BUZZER_FREQUENCY_HIGH, UNIT_MS);
        sleep_ms(UNIT_MS);
    }
    else if (numero_bzr == 1) {
        // Piipataan buzzeria pitkästi KOLMEN "unit of timen" verran (300ms)
        buzzer_play_tone(BUZZER_FREQUENCY_HIGH, 3*UNIT_MS);
        sleep_ms(UNIT_MS);
    }
    else if (numero_bzr == 2) {
        // Piipataan buzzeria lyhyesti KOLMEN "unit of timen" verran (100ms) alemmalla taajuudella
        buzzer_play_tone(BUZZER_FREQUENCY_LOW, 3*UNIT_MS);
        sleep_ms(UNIT_MS);
    }
}

void startup_jingle() {
    // Käynnistys ääni
    ledFxn(true);
    buzzer_play_tone(984, UNIT_MS);
    buzzer_play_tone(890, UNIT_MS);
    buzzer_play_tone(1054, UNIT_MS);
    ledFxn(true);
}

void imu_task(void *pvParameters) {
    (void)pvParameters;

    
    float ax, ay, az, gx, gy, gz, t;
    // Setting up the sensor. 
    if (init_ICM42670() == 0) {
        usb_serial_print("ICM-42670P initialized successfully!\n");
        if (ICM42670_start_with_default_values() != 0){
            usb_serial_print("ICM-42670P could not initialize accelerometer or gyroscope");
        }
        /*int _enablegyro = ICM42670_enable_accel_gyro_ln_mode();
        usb_serial_print ("Enable gyro: %d\n",_enablegyro);
        int _gyro = ICM42670_startGyro(ICM42670_GYRO_ODR_DEFAULT, ICM42670_GYRO_FSR_DEFAULT);
        usb_serial_print ("Gyro return:  %d\n", _gyro);
        int _accel = ICM42670_startAccel(ICM42670_ACCEL_ODR_DEFAULT, ICM42670_ACCEL_FSR_DEFAULT);
        usb_serial_print ("Accel return:  %d\n", _accel);*/
    } else {
        usb_serial_print("Failed to initialize ICM-42670P.\n");
    }
    // Start collection data here. Infinite loop.
    int space_count = 0;
    while (1)
    {
        if (programState == WRITING) {
            if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                //sprintf(buf,"Accel: X=%.2f, Y=%.2f, Z=%.2f | Gyint currentIndex = 0;ro: X=%.2f, Y=%.2f, Z=%.2f| Temp: %2.2f°C\n", ax, ay, az, gx, gy, gz, t);
                //usb_serial_print(buf);
                if (ax < -0.7) {
                    // lähetetään piste
                    usb_serial_print(".");
                    buffer[currentIndex++] = '.';
                    space_count = 0;
                    buzzerFnx(0);
                } else if (ax > 0.7) {
                    // lähetetään viiva
                    usb_serial_print("-");
                    buffer[currentIndex++] = '-';
                    space_count = 0;
                    buzzerFnx(1);
                } else if (ay < -0.7 && currentIndex != 0 && space_count < 2) { // Estetään välilyönnillä aloittaminen
                    // lähetetään väli
                    usb_serial_print(" ");
                    buffer[currentIndex++] = ' ';
                    space_count++;
                    buzzerFnx(2);
                } else if (ay > 0.7) {
                    buffer[currentIndex] = '  \n';
                    programState = MSG_READY;
                }

            } else {
                usb_serial_print("Failed to read imu data\n");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

}

void send_task(void *pvParameters) {
    (void)pvParameters;
    while (1) {
        if (programState == MSG_READY) {
            // Ilmoitetaan käyttäjälle viestin lähetyksestä ja IDLE tilaan siirtymisestä
            usb_serial_print("\nMessage: ");
            buzzer_play_tone(BUZZER_FREQUENCY_SEND, UNIT_MS);
            sleep_ms(50);
            buzzer_play_tone(BUZZER_FREQUENCY_SEND, UNIT_MS);
            usb_serial_print(buffer);
            currentIndex = 0;
            memset(buffer, 0, sizeof buffer);
            programState = IDLE;
            toggle_red_led();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
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
    
    init_hat_sdk();
    sleep_ms(300); //Wait some time so initialization of USB and hat is done.
    //usb_serial_print("Start acceleration test\n");

    init_buzzer();
    printf("Initializing buzzer\r\n");
    init_button1();
    init_button2();
    printf("Initializing buttons\r\n");
    init_led();
    init_red_led();
    printf("Initializing leds\r\n");

    startup_jingle(); // Play startup jingle and confirm successful initialization

    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_RISE, true, btn_fxn);
    gpio_set_irq_enabled(BUTTON2, GPIO_IRQ_EDGE_RISE, true);

    TaskHandle_t hsend_task, hIMUTask, hUsb = NULL;

    xTaskCreate(imu_task, "IMUTask", 1024, NULL, 2, &hIMUTask);
    xTaskCreate(usbTask, "usb", 1024, NULL, 3, &hUsb);
    xTaskCreate(send_task, "sender", 1024, NULL, 2, &hsend_task);
    #if (configNUMBER_OF_CORES > 1)
        vTaskCoreAffinitySet(hUsb, 1u << 0);
    #endif
    
    // VERY IMPORTANT, THIS SHOULD GO JUST BEFORE vTaskStartSheduler
    // WITHOUT ANY DELAYS. OTHERWISE, THE TinyUSB stack wont recognize
    // the device.
    // Initialize TinyUSB 
    tusb_init();
    //Initialize helper library to write in CDC0)
    usb_serial_init();
    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    return 0;
}