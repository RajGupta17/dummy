////////////////////////////////////////////////////////////////////////our latest

// #include "mazeblaze2.h"
// #include "tuning_http_server.h"
// #include "wifi_handler.h"
// #include "driver/gpio.h"

// /*Variables created for path planning testing at 2*/
// int test_path[2000] = {1, 0};
// int pindex = 1;

// // char path[200] = ""; // array for storing path

// // int path_index = 0; // counter thats used for changing the index while storing path

// float error = 0, prev_error = 0, difference, cumulative_error, correction;
// float left_duty_cycle = 0, right_duty_cycle = 0;
// // we dont need a right sensor prev reading since we are using left follow

// float kp = 50, ki = 0, kd = 60;

// const int weights[5] = {3, 1, 0, -1, -3};


// #define BOOT_BUTTON 0

// // MOTOR A0 ---> Right
// // MOTOR A1 ---> Left

// /*This method involves tuning kp , ki ,kd physically*/
// #define GOOD_DUTY_CYCLE 80
// #define MIN_DUTY_CYCLE 50
// #define MAX_DUTY_CYCLE 85
// /*variables which help to decide which turns to take*/

// float bound(float val, float min, float max);
// void calculate_correction();
// void calculate_error();

// void calculate_error()
// {

//     int all_black_flag = 1; // assuming initially all black condition
//     float weighted_sum = 0, sum = 0;
//     float pos = 0;

//     for (int i = 0; i < 5; i++)
//     {
//         if (lsa_reading[i] > BLACK_PATCH)
//         {
//             all_black_flag = 0;
//         }
//         weighted_sum += (float)(weights[i]) * (lsa_reading[i]);
//         sum = sum + lsa_reading[i];
//     }

//     if (sum != 0) // sum can never be 0 but just for safety purposes
//     {
//         pos = weighted_sum / sum; // This will give us the position wrt line. if +ve then bot is facing left and if -ve the bot is facing to right.
//     }

//     if (all_black_flag == 1) // If all black then we check for previous error to assign current error.
//     {
//         if (prev_error > 0)
//         {
//             error = 3;
//         }
//         else
//         {
//             error = -3;
//         }
//     }
//     else
//     {
//         error = pos;
//     }
// }
// // end of function

// void calculate_correction()
// {
//     error = error * 10;              // we need the error correction in range 0-100 so that we can send it directly as duty cycle paramete
//     difference = error - prev_error; // used for calcuating kd
//     cumulative_error += error;       // used for calculating ki

//     cumulative_error = bound(cumulative_error, -30, 30);               // bounding cumulative_error to avoid the issue of cumulative_error being very large
//     correction = kp * error + ki * cumulative_error + kd * difference; // defined in http_server.c

//     prev_error = error; // update error
// }
// // end of function

// float bound(float val, float min, float max) // To bound a certain value in range MAX to MIN
// {
//     if (val > max)
//         val = max;
//     else if (val < min)
//         val = min;
//     return val;
// }
// // end of function

// // all booleans

// bool found_left = false, found_straight = false, found_right = false, black = false;
// bool only_left = false, left = false, right = false, only_right = false, ll = false;


// void line_follow_task(void *arg)
// {

//     while (1)
//     {
//         get_raw_lsa(); // funtion that updates the lsa readings

//         if ((lsa_reading[0] == 1000) && ((lsa_reading[1] == 1000 || lsa_reading[3] == 1000) && lsa_reading[2] == 1000)) // checks left first
//         {
//             printf("Success 00\n");
//             left = true;

//             while (lsa_reading[0] == 1000 && lsa_reading[1] == 1000)
//             {
//                 get_raw_lsa();
//             }

//             // vTaskDelay(50/portTICK_PERIOD_MS);
//             // set_motor_speed(MOTOR_A_0, MOTOR_STOP, 0);
//             // set_motor_speed(MOTOR_A_1, MOTOR_STOP, 0);
//             vTaskDelay(70/portTICK_PERIOD_MS);
//             get_raw_lsa();
//             printf("%d %d %d %d %d", lsa_reading[0], lsa_reading[1], lsa_reading[2], lsa_reading[3], lsa_reading[4]);

//             if (lsa_reading[0] == 0 && lsa_reading[1] == 0 && lsa_reading[3] == 0 && lsa_reading[2] == 0 && lsa_reading[4] == 0 && left == true)
//             {
//                 printf("ONLY LEFT DETECTED");
//                 only_left = true;
//             }
//             else if (lsa_reading[0] == 0 && lsa_reading[4] == 0 && lsa_reading[2] == 1000 && (lsa_reading[1] == 1000 || lsa_reading[3] == 1000) && (left == true))
//             {
//                 printf("STR+LEFT DETECTED");
//                 only_left = false;
//             }
//         }
//         else if (lsa_reading[0] == 0 && lsa_reading[3] == 1000 && lsa_reading[2] == 1000 && lsa_reading[4] == 1000 )
//         {
//             printf("Success 99\n");
//             right = true;

//             while (lsa_reading[4] == 1000 && lsa_reading[3] == 1000)
//             {
//                 get_raw_lsa() ;
//             }

//             // vTaskDelay(50/portTICK_PERIOD_MS);
//             // set_motor_speed(MOTOR_A_0, MOTOR_STOP, 0);
//             // set_motor_speed(MOTOR_A_1, MOTOR_STOP, 0);
//             vTaskDelay(70/portTICK_PERIOD_MS);
//             get_raw_lsa();
//             printf("%d %d %d %d %d", lsa_reading[0], lsa_reading[1], lsa_reading[2], lsa_reading[3], lsa_reading[4]);

//             if ((lsa_reading[0] == 0 && lsa_reading[1] == 0 && lsa_reading[3] == 0 && lsa_reading[2] == 0 && lsa_reading[4] == 0) && (right == true))
//             {
//                 printf("ONLY RIGHT DETECTED");
//                 only_right = true ;
//             }
//             else if (lsa_reading[0] == 0 && lsa_reading[4] == 0 && lsa_reading[2] == 1000 && (lsa_reading[1] == 1000 || lsa_reading[3] == 1000) && (right == true))
//             {
//                 printf("STR+RIGHT DETECTED");
//                 only_right = false ;
//             }
//         }

//         while ((left == true) && (only_left == false)) // STR+LEFT
//         {

//             if (test_path[pindex - 1] == 1)
//             {
//                 test_path[pindex] = 4;
//             }
//             else if (test_path[pindex - 1] == 2)
//             {
//                 test_path[pindex] = 1;
//             }
//             else if (test_path[pindex - 1] == 3)
//             {
//                 test_path[pindex] = 2;
//             }
//             else
//             {
//                 test_path[pindex] = 3;
//             }
//             pindex++;
//             while (1)
//             {
//                 get_raw_lsa();

//                 if (lsa_reading[2] == 0)
//                 {
//                     ll = true;
//                 }

//                 set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, 75);
//                 set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD, 75);
//                 if ((lsa_reading[1] == 1000) && ll)
//                 {
//                     vTaskDelay(100 / portTICK_PERIOD_MS);
//                     set_motor_speed(MOTOR_A_0, MOTOR_STOP, 0);
//                     set_motor_speed(MOTOR_A_1, MOTOR_STOP, 0);
//                     ll = false;
//                     left = false;
//                     only_left = false;

//                     break;
//                 }
//                 vTaskDelay(10 / portTICK_PERIOD_MS);
//             }
//         }

//         if ((lsa_reading[0] == 0 && lsa_reading[1] == 0 && lsa_reading[2] == 0 && lsa_reading[3] == 0 && lsa_reading[4] == 0) && (left == false)) // UTURN
//         {

//             if (test_path[pindex - 1] == 1)
//             {
//                 test_path[pindex] = 3;
//             }
//             else if (test_path[pindex - 1] == 2)
//             {
//                 test_path[pindex] = 4;
//             }
//             else if (test_path[pindex - 1] == 3)
//             {
//                 test_path[pindex] = 1;
//             }
//             else
//             {
//                 test_path[pindex] = 2;
//             }

//             pindex++;

//             while (lsa_reading[2] == 0)
//             {
//                 get_raw_lsa();

//                 set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD, 75);
//                 set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, 75);

//                 if (lsa_reading[2] == 1000)
//                 {
//                     set_motor_speed(MOTOR_A_0, MOTOR_STOP, 0);
//                     set_motor_speed(MOTOR_A_1, MOTOR_STOP, 0);
//                 }

//                 vTaskDelay(10 / portTICK_PERIOD_MS);
//             }
//         }

//         while ((left == true) && (only_left == true)) // ONLY LEFT
//         {
//              while (1)
//             {
//                 get_raw_lsa();

//                 if (lsa_reading[2] == 0)
//                 {
//                     ll = true;
//                 }

//                 set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, 75);
//                 set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD, 75);
//                 if ((lsa_reading[1] == 1000) && ll)
//                 {
//                     vTaskDelay(100 / portTICK_PERIOD_MS);
//                     set_motor_speed(MOTOR_A_0, MOTOR_STOP, 0);
//                     set_motor_speed(MOTOR_A_1, MOTOR_STOP, 0);
//                     ll = false;
//                     left = false;
//                     only_left = false;

//                     break;
//                 }
//                 vTaskDelay(10 / portTICK_PERIOD_MS);
//             }
//         }

//         // Line Following

//         calculate_error();
//         calculate_correction();

//         left_duty_cycle = bound((GOOD_DUTY_CYCLE - correction), MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
//         right_duty_cycle = bound((GOOD_DUTY_CYCLE + correction), MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);

//         set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, left_duty_cycle); /*goes forward in this case*/
//         set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, right_duty_cycle);

//         vTaskDelay(10 / portTICK_PERIOD_MS);

//         // printf("The current readings in ARRAY are : ");

//         // for (int i = 0; i < pindex; i++)
//         // {
//         //     printf("%d ", test_path[i]);
//         // }
//         // printf("\n");
//     }
// }

// // end of task

// void app_main()
// {
//     ESP_ERROR_CHECK(enable_lsa());
//     ESP_ERROR_CHECK(enable_motor_driver());

//     xTaskCreate(&line_follow_task, "line_follow_task", 4096, NULL, 1, NULL);
//     // start_tuning_http_server();
// }

// // end of main

////////////////////////////////////////////////////////////////////////////////////////////////// old

// #include "mazeblaze2.h"
// #include "tuning_http_server.h"
// #include "wifi_handler.h"
// #include "driver/gpio.h"

// char path[200] = ""; // array for storing path

// int path_index = 0; // counter thats used for changing the index while storing path

// float error = 0, prev_error = 0, difference, cumulative_error, correction;
// float left_duty_cycle = 0, right_duty_cycle = 0;
// // we dont need a right sensor prev reading since we are using left follow

// float kp = 35, ki = 0, kd = 45;

// const int weights[5] = {3, 1, 0, -1, -3};

// bool ll = false;

// #define BOOT_BUTTON 0

// // MOTOR A0 ---> Right
// // MOTOR A1 ---> Left

// /*This method involves tuning kp , ki ,kd physically*/
// #define GOOD_DUTY_CYCLE 80
// #define MIN_DUTY_CYCLE 50
// #define MAX_DUTY_CYCLE 85
// /*variables which help to decide which turns to take*/

// float bound(float val, float min, float max);
// void calculate_correction();
// void calculate_error();

// void calculate_error()
// {

//     int all_black_flag = 1; // assuming initially all black condition
//     float weighted_sum = 0, sum = 0;
//     float pos = 0;

//     for (int i = 0; i < 5; i++)
//     {
//         if (lsa_reading[i] > BLACK_PATCH)
//         {
//             all_black_flag = 0;
//         }
//         weighted_sum += (float)(weights[i]) * (lsa_reading[i]);
//         sum = sum + lsa_reading[i];
//     }

//     if (sum != 0) // sum can never be 0 but just for safety purposes
//     {
//         pos = weighted_sum / sum; // This will give us the position wrt line. if +ve then bot is facing left and if -ve the bot is facing to right.
//     }

//     if (all_black_flag == 1) // If all black then we check for previous error to assign current error.
//     {
//         if (prev_error > 0)
//         {
//             error = 3;
//         }
//         else
//         {
//             error = -3;
//         }
//     }
//     else
//     {
//         error = pos;
//     }
// }
// // end of function

// void calculate_correction()
// {
//     error = error * 10;              // we need the error correction in range 0-100 so that we can send it directly as duty cycle paramete
//     difference = error - prev_error; // used for calcuating kd
//     cumulative_error += error;       // used for calculating ki

//     cumulative_error = bound(cumulative_error, -30, 30);               // bounding cumulative_error to avoid the issue of cumulative_error being very large
//     correction = kp * error + ki * cumulative_error + kd * difference; // defined in http_server.c

//     prev_error = error; // update error
// }
// // end of function

// float bound(float val, float min, float max) // To bound a certain value in range MAX to MIN
// {
//     if (val > max)
//         val = max;
//     else if (val < min)
//         val = min;
//     return val;
// }
// // end of function

// // path planning functions

// bool found_left = false, found_straight = false, found_right = false, right = false, black = false;
// bool only_left = false, left = false;
// bool R = false;



// void line_follow_task(void *arg)
// {

//     while (1)
//     {
//         get_raw_lsa(); // funtion that updates the lsa readings

//         // selection();

//         // if ((lsa_reading[0] == 1000) || (lsa_reading[4] == 1000) || (lsa_reading[0] == 0 && lsa_reading[1] == 0 && lsa_reading[2] == 0 && lsa_reading[3] == 0 && lsa_reading[4] == 0))
//         // {
//         //     unsigned char store = select_turn();
//         //     path[path_index] = store;
//         //     path_index++;
//         // }
//         // ESP_LOGI("debug", "%s", path);

//         //     gpio_config_t io_conf;
//         // // bit mask for the pins, each bit maps to a GPIO
//         // uint64_t bit_mask = (1ULL << BOOT_BUTTON);

//         // io_conf.pin_bit_mask = bit_mask;
//         // // set gpio mode to input
//         // io_conf.mode = GPIO_MODE_INPUT;
//         // // enable pull up resistors
//         // io_conf.pull_up_en = 0;
//         // // disable pull down resistors
//         // io_conf.pull_down_en = 1;
//         // // disable gpio interrupts
//         // io_conf.intr_type = GPIO_INTR_DISABLE;
//         // // detailed description can be found at https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html#_CPPv413gpio_config_t

//         // esp_err_t err = gpio_config(&io_conf);

//         //     int boot_button_state = gpio_get_level(0);

//         if (lsa_reading[0] == 1000 && lsa_reading[1] == 1000 && lsa_reading[2] == 1000 && lsa_reading[3] == 1000) // checks left first
//         {
//             left = true;
//         }

//         if ((lsa_reading[0] == 0 && lsa_reading[1] == 0 && lsa_reading[2] == 0 && lsa_reading[3] == 0) && (left == true)) // checks if there is all black after a left (only left condition)
//         {
//             only_left = true;
//         }

//         if ((lsa_reading[2] == 1000) && ((lsa_reading[1] == 1000) || (lsa_reading[3] == 1000)) && (left == true) && (only_left == true)) // if straight line is found after only left condition
//         {
//             left = false;
//             only_left = false;
//         }

//         // Actual Turns Code

//         while ((left == true) && (only_left == false)) // STR+LEFT
//         {

//             while (1)
//             {
//                 get_raw_lsa();

//                 if (lsa_reading[2] == 0)
//                 {
//                     ll = true;
//                 }

//                 set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, 75);
//                 set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD, 75);
//                 if ((lsa_reading[1] == 1000) && ll)
//                 {
//                     vTaskDelay(100 / portTICK_PERIOD_MS);
//                     set_motor_speed(MOTOR_A_0, MOTOR_STOP, 0);
//                     set_motor_speed(MOTOR_A_1, MOTOR_STOP, 0);
//                     ll = false;
//                     left = false;

//                     break;
//                 }
//                 vTaskDelay(10 / portTICK_PERIOD_MS);
//             }
//         }

//         if ((lsa_reading[0] == 0 && lsa_reading[1] == 0 && lsa_reading[2] == 0 && lsa_reading[3] == 0 && lsa_reading[4] == 0) && (only_left == false)) // UTURN
//         {

//             while (lsa_reading[2] == 0)
//             {
//                 get_raw_lsa();

//                 set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD, 75);
//                 set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, 75);

//                 if (lsa_reading[2] == 1000)
//                 {
//                     set_motor_speed(MOTOR_A_0, MOTOR_STOP, 0);
//                     set_motor_speed(MOTOR_A_1, MOTOR_STOP, 0);
//                 }

//                 vTaskDelay(10 / portTICK_PERIOD_MS);
//             }
//         }

//         if ((lsa_reading[0] == 0 && lsa_reading[1] == 0 && lsa_reading[2] == 0 && lsa_reading[3] == 0) && (only_left == true)) // ONLY LEFT
//         {

//             while (lsa_reading[2] == 0)
//             {
//                 get_raw_lsa();

//                 set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, 75);
//                 set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD, 75);

//                 if (lsa_reading[2] == 1000)
//                 {
//                     set_motor_speed(MOTOR_A_0, MOTOR_STOP, 0);
//                     set_motor_speed(MOTOR_A_1, MOTOR_STOP, 0);
//                 }

//                 vTaskDelay(10 / portTICK_PERIOD_MS);
//             }
//         }

//         // Line Following

//         calculate_error();
//         calculate_correction();

//         left_duty_cycle = bound((GOOD_DUTY_CYCLE - correction), MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
//         right_duty_cycle = bound((GOOD_DUTY_CYCLE + correction), MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);

//         set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, left_duty_cycle); /*goes forward in this case*/
//         set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, right_duty_cycle);

//         vTaskDelay(10 / portTICK_PERIOD_MS);
//     }
// }

// // end of task

// void app_main()
// {
//     ESP_ERROR_CHECK(enable_lsa());
//     ESP_ERROR_CHECK(enable_motor_driver());

//     xTaskCreate(&line_follow_task, "line_follow_task", 4096, NULL, 1, NULL);
//     // start_tuning_http_server();
// }

// end of main

//////////////////////////////////////////////////////////////////////////////////// prit latest - 1pm

// #include "mazeblaze2.h"
// #include "tuning_http_server.h"
// #include "driver/gpio.h"

// /*Variables created for path planning testing at 2*/
// int test_path[2000] = {1, 0};
// int pindex = 1;

// // char path[200] = ""; // array for storing path

// // int path_index = 0; // counter thats used for changing the index while storing path

// float error = 0, prev_error = 0, difference, cumulative_error, correction;
// float left_duty_cycle = 0, right_duty_cycle = 0;
// // we dont need a right sensor prev reading since we are using left follow

// float kp = 50, ki = 0, kd = 60;

// const int weights[5] = {3, 1, 0, -1, -3};

// #define BOOT_BUTTON 0

// // MOTOR A0 ---> Right
// // MOTOR A1 ---> Left

// /*This method involves tuning kp , ki ,kd physically*/
// #define GOOD_DUTY_CYCLE 80
// #define MIN_DUTY_CYCLE 50
// #define MAX_DUTY_CYCLE 85
// /*variables which help to decide which turns to take*/

// float bound(float val, float min, float max);
// void calculate_correction();
// void calculate_error();

// void calculate_error()
// {

//     int all_black_flag = 1; // assuming initially all black condition
//     float weighted_sum = 0, sum = 0;
//     float pos = 0;

//     for (int i = 0; i < 5; i++)
//     {
//         if (lsa_reading[i] > BLACK_PATCH)
//         {
//             all_black_flag = 0;
//         }
//         weighted_sum += (float)(weights[i]) * (lsa_reading[i]);
//         sum = sum + lsa_reading[i];
//     }

//     if (sum != 0) // sum can never be 0 but just for safety purposes
//     {
//         pos = weighted_sum / sum; // This will give us the position wrt line. if +ve then bot is facing left and if -ve the bot is facing to right.
//     }

//     if (all_black_flag == 1) // If all black then we check for previous error to assign current error.
//     {
//         if (prev_error > 0)
//         {
//             error = 3;
//         }
//         else
//         {
//             error = -3;
//         }
//     }
//     else
//     {
//         error = pos;
//     }
// }
// // end of function

// void calculate_correction()
// {
//     error = error * 10;              // we need the error correction in range 0-100 so that we can send it directly as duty cycle paramete
//     difference = error - prev_error; // used for calcuating kd
//     cumulative_error += error;       // used for calculating ki

//     cumulative_error = bound(cumulative_error, -30, 30);               // bounding cumulative_error to avoid the issue of cumulative_error being very large
//     correction = kp * error + ki * cumulative_error + kd * difference; // defined in http_server.c

//     prev_error = error; // update error
// }
// // end of function

// float bound(float val, float min, float max) // To bound a certain value in range MAX to MIN
// {
//     if (val > max)
//         val = max;
//     else if (val < min)
//         val = min;
//     return val;
// }
// // end of function

// // all booleans

// bool found_left = false, found_straight = false, found_right = false, black = false;
// bool only_left = false, left = false, right = false, only_right = false, ll = false;

// void line_follow_task(void *arg)
// {

//     while (1)
//     {
//         get_raw_lsa(); // funtion that updates the lsa readings

//         if ((lsa_reading[0] == 1000) && (lsa_reading[1] == 1000) && (lsa_reading[2] == 1000)) // checks left first
//         {
//             printf("dummy left flag\n");
//             left = 1;
//             vTaskDelay(10 / portTICK_PERIOD_MS);
//             get_raw_lsa(); // funtion that updates the lsa readings
//             if (left == 1 || right == 1)
//             {
//                 if ((lsa_reading[0] == 1000) && (lsa_reading[1] == 1000) && (lsa_reading[2] == 1000)) // checks left first
//                 {
//                     printf("left flag confirmed\n");
//                     left = 1;
//                 }
//                 else if (lsa_reading[0] == 0 && lsa_reading[3] == 1000 && lsa_reading[2] == 1000 && lsa_reading[4] == 1000)
//                 {
//                     printf("Right flag confirmed\n");
//                     right = true;
//                 }
//                 else
//                 {
//                     left = 0 ;
//                     right = 0 ;
//                 }
//             }
//         }
//         else if (lsa_reading[0] == 0 && lsa_reading[3] == 1000 && lsa_reading[2] == 1000 && lsa_reading[4] == 1000)
//         {
//             printf("dummy Right flag \n");
//             right = true;
//             vTaskDelay(10 / portTICK_PERIOD_MS);
//             get_raw_lsa(); // funtion that updates the lsa readings
//             if (left == 1 || right == 1)
//             {
//                 if ((lsa_reading[0] == 1000) && (lsa_reading[1] == 1000) && (lsa_reading[2] == 1000)) // checks left first
//                 {
//                     printf("left flag confirmed\n");
//                     left = 1;
//                 }
//                 else if (lsa_reading[0] == 0 && lsa_reading[3] == 1000 && lsa_reading[2] == 1000 && lsa_reading[4] == 1000)
//                 {
//                     printf("Right flag confirmed\n");
//                     right = true;
//                 }
//                 else
//                 {
//                     left = 0 ;
//                     right = 0 ;
//                 }
//             }
//         }

//         if (left == 1) // checks left first
//         {
//             while (lsa_reading[0] == 1000 && lsa_reading[1] == 1000)
//             {
//                 get_raw_lsa();
//             }

//             vTaskDelay(70 / portTICK_PERIOD_MS);

//             get_raw_lsa();

//             printf("%d %d %d %d %d\n", lsa_reading[0], lsa_reading[1], lsa_reading[2], lsa_reading[3], lsa_reading[4]);

//             if (lsa_reading[1] == 0 && lsa_reading[3] == 0 && lsa_reading[2] == 0 )
//             {
//                 printf("ONLY LEFT DETECTED");
//                 only_left = true;
//             }
//             else if (lsa_reading[2] == 1000 && (lsa_reading[1] == 1000 || lsa_reading[3] == 1000) )
//             {
//                 printf("STR+LEFT DETECTED");
//                 only_left = false;
//             }
//         }
//         else if (right == 1)
//         {
//             printf("Success 101\n");

//             while (lsa_reading[4] == 1000 && lsa_reading[3] == 1000)
//             {
//                 get_raw_lsa();
//             }

//             vTaskDelay(70 / portTICK_PERIOD_MS);
//             get_raw_lsa();
//             printf("%d %d %d %d %d\n", lsa_reading[0], lsa_reading[1], lsa_reading[2], lsa_reading[3], lsa_reading[4]);

//             if ((lsa_reading[1] == 0 && lsa_reading[3] == 0 && lsa_reading[2] == 0) )
//             {
//                 printf("ONLY RIGHT DETECTED");
//                 only_right = true;
//             }
//             else if (lsa_reading[2] == 1000 && (lsa_reading[1] == 1000 || lsa_reading[3] == 1000) )
//             {
//                 printf("STR+RIGHT DETECTED");
//                 only_right = false;
//             }
//         }

//         while (left == true) // STR+LEFT
//         {

//             if (test_path[pindex - 1] == 1)
//             {
//                 test_path[pindex] = 4;
//             }
//             else if (test_path[pindex - 1] == 2)
//             {
//                 test_path[pindex] = 1;
//             }
//             else if (test_path[pindex - 1] == 3)
//             {
//                 test_path[pindex] = 2;
//             }
//             else
//             {
//                 test_path[pindex] = 3;
//             }
//             pindex++;
//             while (1)
//             {
//                 get_raw_lsa();

//                 if (lsa_reading[2] == 0)
//                 {
//                     ll = true;
//                 }

//                 set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, 75);
//                 set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD, 75);
//                 if ((lsa_reading[1] == 1000 && lsa_reading[2] == 1000) && ll)
//                 {
//                     // vTaskDelay(100 / portTICK_PERIOD_MS);
//                     set_motor_speed(MOTOR_A_0, MOTOR_STOP, 0);
//                     set_motor_speed(MOTOR_A_1, MOTOR_STOP, 0);
//                     ll = false;
//                     left = false;
//                     right = false;
//                     only_right = false;
//                     only_left = false;

//                     break;
//                 }
//                 vTaskDelay(10 / portTICK_PERIOD_MS);
//             }
//         }

//         if ((lsa_reading[0] == 0 && lsa_reading[1] == 0 && lsa_reading[2] == 0 && lsa_reading[3] == 0 && lsa_reading[4] == 0) && (left == false)) // UTURN
//         {

//             if (test_path[pindex - 1] == 1)
//             {
//                 test_path[pindex] = 3;
//             }
//             else if (test_path[pindex - 1] == 2)
//             {
//                 test_path[pindex] = 4;
//             }
//             else if (test_path[pindex - 1] == 3)
//             {
//                 test_path[pindex] = 1;
//             }
//             else
//             {
//                 test_path[pindex] = 2;
//             }

//             pindex++;

//             while (lsa_reading[2] == 0)
//             {
//                 get_raw_lsa();

//                 set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD, 75);
//                 set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, 75);

//                 if (lsa_reading[2] == 1000)
//                 {
//                     set_motor_speed(MOTOR_A_0, MOTOR_STOP, 0);
//                     set_motor_speed(MOTOR_A_1, MOTOR_STOP, 0);
//                     right = false;
//                     left = false;
//                     only_left = false;
//                     only_right = false;
//                 }

//                 vTaskDelay(10 / portTICK_PERIOD_MS);
//             }
//         }

//         // Line Following

//         calculate_error();
//         calculate_correction();

//         left_duty_cycle = bound((GOOD_DUTY_CYCLE - correction), MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
//         right_duty_cycle = bound((GOOD_DUTY_CYCLE + correction), MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);

//         set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, left_duty_cycle); /*goes forward in this case*/
//         set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, right_duty_cycle);

//         vTaskDelay(10 / portTICK_PERIOD_MS);

//         // printf("The current readings in ARRAY are : ");

//         // for (int i = 0; i < pindex; i++)
//         // {
//         //     printf("%d ", test_path[i]);
//         // }
//         // printf("\n");
//     }
// }

// // end of task

// void app_main()
// {
//     ESP_ERROR_CHECK(enable_lsa());
//     ESP_ERROR_CHECK(enable_motor_driver());

//     xTaskCreate(&line_follow_task, "line_follow_task", 4096, NULL, 1, NULL);
// }

////////////////////////////////////////////////////////////////////////////////////////// prit latest - 2pm

#include "mazeblaze2.h"
#include "tuning_http_server.h"
#include "driver/gpio.h"

/*Variables created for path planning testing at 2*/
int test_path[2000] = {1, 0};
int pindex = 1;

// char path[200] = ""; // array for storing path

// int path_index = 0; // counter thats used for changing the index while storing path

float error = 0, prev_error = 0, difference, cumulative_error, correction;
float left_duty_cycle = 0, right_duty_cycle = 0;
// we dont need a right sensor prev reading since we are using left follow

float kp = 50, ki = 0, kd = 60;

const int weights[5] = {3, 1, 0, -1, -3};

#define BOOT_BUTTON 0

// MOTOR A0 ---> Right
// MOTOR A1 ---> Left

/*This method involves tuning kp , ki ,kd physically*/
#define GOOD_DUTY_CYCLE 80
#define MIN_DUTY_CYCLE 50
#define MAX_DUTY_CYCLE 85
/*variables which help to decide which turns to take*/

float bound(float val, float min, float max);
void calculate_correction();
void calculate_error();

void calculate_error()
{

    int all_black_flag = 1; // assuming initially all black condition
    float weighted_sum = 0, sum = 0;
    float pos = 0;

    for (int i = 0; i < 5; i++)
    {
        if (lsa_reading[i] > BLACK_PATCH)
        {
            all_black_flag = 0;
        }
        weighted_sum += (float)(weights[i]) * (lsa_reading[i]);
        sum = sum + lsa_reading[i];
    }

    if (sum != 0) // sum can never be 0 but just for safety purposes
    {
        pos = weighted_sum / sum; // This will give us the position wrt line. if +ve then bot is facing left and if -ve the bot is facing to right.
    }

    if (all_black_flag == 1) // If all black then we check for previous error to assign current error.
    {
        if (prev_error > 0)
        {
            error = 3;
        }
        else
        {
            error = -3;
        }
    }
    else
    {
        error = pos;
    }
}
// end of function

void calculate_correction()
{
    error = error * 10;              // we need the error correction in range 0-100 so that we can send it directly as duty cycle paramete
    difference = error - prev_error; // used for calcuating kd
    cumulative_error += error;       // used for calculating ki

    cumulative_error = bound(cumulative_error, -30, 30);               // bounding cumulative_error to avoid the issue of cumulative_error being very large
    correction = kp * error + ki * cumulative_error + kd * difference; // defined in http_server.c

    prev_error = error; // update error
}
// end of function

float bound(float val, float min, float max) // To bound a certain value in range MAX to MIN
{
    if (val > max)
        val = max;
    else if (val < min)
        val = min;
    return val;
}
// end of function

// all booleans
bool only_left = false, left = false, right = false, only_right = false, ll = false;

void line_follow_task(void *arg)
{

    while (1)
    {
        get_raw_lsa(); // funtion that updates the lsa readings

        if ((lsa_reading[0] == 1000) && (lsa_reading[1] == 1000) && (lsa_reading[2] == 1000)) // checks left first
        {
            left = 1;
            vTaskDelay(10 / portTICK_PERIOD_MS);
            get_raw_lsa(); // funtion that updates the lsa readings
            if (left == 1 || right == 1)
            {
                if ((lsa_reading[0] == 1000) && (lsa_reading[1] == 1000) && (lsa_reading[2] == 1000)) // checks left first
                {
                    printf("left flag confirmed\n");
                    left = 1;
                }
                else if (lsa_reading[0] == 0 && lsa_reading[3] == 1000 && lsa_reading[2] == 1000 && lsa_reading[4] == 1000)
                {
                    printf("Right flag confirmed\n");
                    right = true;
                }
                else
                {
                    left = 0;
                    right = 0;
                }
            }
        }
        else if (lsa_reading[0] == 0 && lsa_reading[3] == 1000 && lsa_reading[2] == 1000 && lsa_reading[4] == 1000)
        {
            right = true;
            vTaskDelay(10 / portTICK_PERIOD_MS);
            get_raw_lsa(); // funtion that updates the lsa readings
            if (left == 1 || right == 1)
            {
                if ((lsa_reading[0] == 1000) && (lsa_reading[1] == 1000) && (lsa_reading[2] == 1000)) // checks left first
                {
                    printf("left flag confirmed\n");
                    left = 1;
                }
                else if (lsa_reading[0] == 0 && lsa_reading[3] == 1000 && lsa_reading[2] == 1000 && lsa_reading[4] == 1000)
                {
                    printf("Right flag confirmed\n");
                    right = 1;
                }
                else
                {
                    left = 0;
                    right = 0;
                }
            }
        }

        if (left == 1) // checks left first
        {
            while (lsa_reading[0] == 1000 && lsa_reading[1] == 1000)
            {
                get_raw_lsa();
                 vTaskDelay(10 / portTICK_PERIOD_MS);
            }

            vTaskDelay(70 / portTICK_PERIOD_MS);

            get_raw_lsa();

            printf("%d %d %d %d %d\n", lsa_reading[0], lsa_reading[1], lsa_reading[2], lsa_reading[3], lsa_reading[4]);

            if (lsa_reading[1] == 0 && lsa_reading[3] == 0 && lsa_reading[2] == 0)
            {
                printf("ONLY LEFT DETECTED");
                only_left = true;
            }
            else if (lsa_reading[2] == 1000 && (lsa_reading[1] == 1000 || lsa_reading[3] == 1000))
            {
                printf("STR+LEFT DETECTED");
                only_left = false;
            }
        }
        else if (right == 1)
        {
            printf("Success 101\n");

            while (lsa_reading[4] == 1000 && lsa_reading[3] == 1000)
            {
                get_raw_lsa();
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }

            vTaskDelay(70 / portTICK_PERIOD_MS);
            get_raw_lsa();
            printf("%d %d %d %d %d\n", lsa_reading[0], lsa_reading[1], lsa_reading[2], lsa_reading[3], lsa_reading[4]);

            if ((lsa_reading[0] == 0 && lsa_reading[1] == 0 && lsa_reading[3] == 0 && lsa_reading[2] == 0 && lsa_reading[4] == 0))
            {
                printf("ONLY RIGHT DETECTED");
                only_right = true;
            }
            else if (lsa_reading[2] == 1000 && (lsa_reading[1] == 1000 || lsa_reading[3] == 1000))
            {
                printf("STR+RIGHT DETECTED");
                only_right = false;
            }
        }

        get_raw_lsa();

        if (left == 1)
        {

            if (test_path[pindex - 1] == 1)
            {
                test_path[pindex] = 4;
            }
            else if (test_path[pindex - 1] == 2)
            {
                test_path[pindex] = 1;
            }
            else if (test_path[pindex - 1] == 3)
            {
                test_path[pindex] = 2;
            }
            else
            {
                test_path[pindex] = 3;
            }
            pindex++;
            while (1)
            {
                get_raw_lsa();

                if (lsa_reading[1] == 0)
                {
                    ll = true;
                }

                set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, 75);
                set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD, 75);


                if ((lsa_reading[1] == 1000 && lsa_reading[2] == 1000) && ll)
                {
                    // vTaskDelay(80 / portTICK_PERIOD_MS);
                    set_motor_speed(MOTOR_A_0, MOTOR_STOP, 0);
                    set_motor_speed(MOTOR_A_1, MOTOR_STOP, 0);
                    ll = 0;
                    left = 0;
                    only_left = 0;
                    only_right = 0 ;
                    right = 0 ;

                    break;
                }
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        }
        else if (only_right == 1 && right == 1) 
        {

            while (lsa_reading[2] == 0)
            {
                get_raw_lsa();

                set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD, 75);
                set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, 75);

                // vTaskDelay(100 / portTICK_PERIOD_MS);

                if (lsa_reading[2] == 1000)
                {
                    set_motor_speed(MOTOR_A_0, MOTOR_STOP, 0);
                    set_motor_speed(MOTOR_A_1, MOTOR_STOP, 0);

                    ll = 0;
                    left = 0;
                    only_left = 0;
                    only_right = 0 ;
                    right = 0 ;

                    break ;
                }

                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        }
        else if ((lsa_reading[0] == 0 && lsa_reading[1] == 0 && lsa_reading[3] == 0 && lsa_reading[2] == 0 && lsa_reading[4] == 0) &&(right == 0) && (left == 0))
        {
            while (lsa_reading[2] == 0)
            {
                get_raw_lsa();

                set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD, 75);
                set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, 75);

                // vTaskDelay(100 / portTICK_PERIOD_MS);

                if (lsa_reading[2] == 1000)
                {
                    set_motor_speed(MOTOR_A_0, MOTOR_STOP, 0);
                    set_motor_speed(MOTOR_A_1, MOTOR_STOP, 0);

                    ll = 0;
                    left = 0;
                    only_left = 0;
                    only_right = 0 ;
                    right = 0 ;

                    break ;
                }

                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        }

        // Line Following

        calculate_error();
        calculate_correction();

        left_duty_cycle = bound((GOOD_DUTY_CYCLE - correction), MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
        right_duty_cycle = bound((GOOD_DUTY_CYCLE + correction), MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);

        set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, left_duty_cycle); /*goes forward in this case*/
        set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, right_duty_cycle);

        vTaskDelay(10 / portTICK_PERIOD_MS);

        // printf("The current readings in ARRAY are : ");

        // for (int i = 0; i < pindex; i++)
        // {
        //     printf("%d ", test_path[i]);
        // }
        // printf("\n");
    }
}

// end of task

void app_main()
{
    ESP_ERROR_CHECK(enable_lsa());
    ESP_ERROR_CHECK(enable_motor_driver());

    xTaskCreate(&line_follow_task, "line_follow_task", 4096, NULL, 1, NULL);
}