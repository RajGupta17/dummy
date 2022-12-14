#include "mazeblaze2.h"
#include "esp_system.h"
#include "esp_log.h"

#define TAG "app"
#define ROT_ENC_A_GPIO 17
#define ROT_ENC_B_GPIO 5
float degree;


void app_main()
{

    ESP_ERROR_CHECK(enable_motor_driver()) ;

    set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, 75);
    set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD, 75);

    vTaskDelay(1000/portTICK_PERIOD_MS);

    set_motor_speed(MOTOR_A_0, MOTOR_STOP, 75);
    set_motor_speed(MOTOR_A_1, MOTOR_STOP, 75);    

    get_degree();
}