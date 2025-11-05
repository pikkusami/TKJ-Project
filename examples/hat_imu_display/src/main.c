
#include <stdio.h>
#include <math.h>
#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include <tkjhat/sdk.h>


static float a_module, a_azimuth, a_elevation=0;

enum state { WAITING=1, DATA_READY};
enum state programState = WAITING;

void display_task (void *pvParameters) {
    (void)pvParameters;
    
    char buf[5]; //Store a number of maximum 5 figures
    while(1) {
        if (programState == DATA_READY){
            //printf("Print\n");
            clear_display();
            //sprintf(buf,"mod:%f6.2 | θ:%d | φ:%d",a_module, (int)a_azimuth, (int)a_elevation);
            sprintf(buf,"mod:%f6.2",a_module);
            write_text(buf);
            programState = WAITING;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


void imu_task(void *pvParameters) {
    (void)pvParameters;
       
    // Start collection data here. Infinite loop. 
    float ax, ay, az, gx, gy, gz, t;
    while (1)
    {
        if (programState == WAITING){
            //printf("Imu\n");
            if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                a_module = sqrtf(ax*ax + ay*ay + az*az);
                //a_azimuth = atan2f(ay, ax);                        // radians
                //a_elevation = atan2f(az, sqrtf(ax*ax + ay*ay));    // radians
                printf("Accel mod: %f6.2\n", a_module);

            } else {
                a_module = -1.0;
                a_azimuth = -1.0;
                a_elevation = 1-0;

                printf("Failed to read imu data\n");
            }
            programState = DATA_READY;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

}

int main() {
    stdio_init_all();
    // Uncomment this lines if you want to wait till the serial monitor is connected
    while (!stdio_usb_connected()){
        sleep_ms(10);
    }
    init_hat_sdk();
    sleep_ms(500); //Wait some time so initialization of USB and hat is done.
    init_led();
    // Setting up the acceleration. 
    if (init_ICM42670() == 0) {
        printf("ICM-42670P initialized successfully!\n");
        if (ICM42670_start_with_default_values() != 0){
            printf("ICM-42670P could not initialize accelerometer or gyroscope");
        }
        /*int _enablegyro = ICM42670_enable_accel_gyro_ln_mode();
        printf ("Enable gyro: %d\n",_enablegyro);
        int _gyro = ICM42670_startGyro(ICM42670_GYRO_ODR_DEFAULT, ICM42670_GYRO_FSR_DEFAULT);
        printf ("Gyro return:  %d\n", _gyro);
        int _accel = ICM42670_startAccel(ICM42670_ACCEL_ODR_DEFAULT, ICM42670_ACCEL_FSR_DEFAULT);
        printf ("Accel return:  %d\n", _accel);*/
    } else {
        printf("Failed to initialize ICM-42670P.\n");
    }
    init_display();
    printf("Initializing display\n");
   

    printf("Start acceleration test\n");

    TaskHandle_t hIMUTask, hdisplayTask = NULL;

    xTaskCreate(imu_task, "IMUTask", 1024, NULL, 3, &hIMUTask);
    xTaskCreate(display_task, "displayTask", 1024, NULL, 2, &hdisplayTask);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    return 0;
}

