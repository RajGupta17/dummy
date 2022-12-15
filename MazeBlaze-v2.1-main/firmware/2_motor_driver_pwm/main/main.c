#include "mazeblaze2.h"

void app_main (void)
{

    ESP_ERROR_CHECK(enable_motor_driver()) ;

    //code for turning - clockwise
   set_motor_speed(MOTOR_A_0 , MOTOR_BACKWARD , 70) ;
        set_motor_speed(MOTOR_A_1 , MOTOR_FORWARD , 70 ) ;

}
