#include "mazeblaze2.h"
//this code will print the readings of LSA on terminal


void app_main(void)
{
    ESP_ERROR_CHECK(enable_lsa());
    ESP_ERROR_CHECK(enable_motor_driver());
    while (1)
    {
        get_raw_lsa();
        vTaskDelay(10/portTICK_PERIOD_MS) ;
    }
}