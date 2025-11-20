
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
#define BUZZER_FREQUENCY 600

enum state { IDLE=1, RECEIVING, TRANSMITTING };
enum state programState = IDLE;

uint8_t buffer[buffer_len];

static void btn_fxn(uint gpio, uint32_t eventMask) {
    if (gpio == BUTTON1) {
        if (programState == IDLE) {
            programState = TRANSMITTING;
            usb_serial_print("Switched to TRANSMITTING");
        }
        else {
            programState = IDLE;
            usb_serial_print("Switched to IDLE");
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
    uint8_t buf[buffer_len];
    bool luettu = false;
    while (1)
    {
        if (programState == TRANSMITTING) {
            if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                sprintf(buf,"Accel: X=%.2f, Y=%.2f, Z=%.2f | Gyro: X=%.2f, Y=%.2f, Z=%.2f| Temp: %2.2f°C\n", ax, ay, az, gx, gy, gz, t);
                usb_serial_print(buf);

                if (ax < -0.7 && luettu == false) {
                        // lähetetään piste
                        usb_serial_print(".");
                        ledFxn(true);
                        buzzerFnx(true);
                        luettu = true;
                    } else if (ax > 0.7 && luettu == false) {
                        // lähetetään viiva
                        usb_serial_print("-");
                        ledFxn(false);
                        buzzerFnx(false);
                        luettu = true;
                    } else if (ay < -0.7 && luettu == false) {
                        // lähetetään väli
                        usb_serial_print(" ");
                        ledFxn(true);
                        buzzerFnx(true);
                        luettu = true;
                    }
                    
                    luettu = false;

            } else {
                usb_serial_print("Failed to read imu data\n");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
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

    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_RISE, true, btn_fxn);
    gpio_set_irq_enabled(BUTTON2, GPIO_IRQ_EDGE_RISE, true);

    TaskHandle_t hIMUTask, hUsb = NULL;

    xTaskCreate(imu_task, "IMUTask", 1024, NULL, 2, &hIMUTask);
    xTaskCreate(usbTask, "usb", 1024, NULL, 3, &hUsb);
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

