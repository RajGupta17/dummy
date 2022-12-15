#include "mazeblaze2.h"
#include "tuning_http_server.h"
#include "wifi_handler.h"
#include "driver/gpio.h"

/*For Line following*/
float error = 0, prev_error = 0, difference, cumulative_error, correction;
float left_duty_cycle = 0, right_duty_cycle = 0;
// we dont need a right sensor prev reading since we are using left follow

float kp = 15, ki = 0.1, kd = 35;

const int weights[5] = {3, 1, 0, -1, -3};

/*This method involves tuning kp , ki ,kd physically*/
#define GOOD_DUTY_CYCLE 80
#define MIN_DUTY_CYCLE 50
#define MAX_DUTY_CYCLE 85
/*variables which help to decide which turns to take*/

float bound(float val, float min, float max);
void calculate_correction();
void calculate_error();

/*For path planning*/
char path[10000] = ""; // array for storing path
int path_index = 0;

bool left = false, right = false;
bool found_left = false, found_straight = false, found_right = false, black = false;
bool only_left = false bool R = false;
bool ll = false;

// MOTOR A0 ---> Right
// MOTOR A1 ---> Left

void selection()
{
    if (lsa_reading[0] == 1000 && lsa_reading[1] == 1000 &&#include "mazeblaze2.h"
#include "tuning_http_server.h"
#include "wifi_handler.h"
#include "driver/gpio.h"

/*For Line following*/
float error = 0, prev_error = 0, difference, cumulative_error, correction;
float left_duty_cycle = 0, right_duty_cycle = 0;
// we dont need a right sensor prev reading since we are using left follow

float kp = 15, ki = 0.1, kd = 35;

const int weights[5] = {3, 1, 0, -1, -3};

/*This method involves tuning kp , ki ,kd physically*/
#define GOOD_DUTY_CYCLE 80
#define MIN_DUTY_CYCLE 50
#define MAX_DUTY_CYCLE 85
/*variables which help to decide which turns to take*/

float bound(float val, float min, float max);
void calculate_correction();
void calculate_error();

/*For path planning*/
char path[10000] = ""; // array for storing path
int path_index = 0;

bool left = false, right = false;
bool found_left = false, found_straight = false, found_right = false, black = false;
bool only_left = false bool R = false;
bool ll = false;

// MOTOR A0 ---> Right
// MOTOR A1 ---> Left

void selection()
{
    if (lsa_reading[0] == 1000 && lsa_reading[1] == 1000 && lsa_reading[2] == 1000 && lsa_reading[3] == 1000 && lsa_reading[4] == 1000) // checks left first
    {
        left = true;
    }
    else if (lsa_reading[0] == 0 && lsa_reading[1] == 0 && lsa_reading[2] == 0 && lsa_reading[3] == 0 && lsa_reading[4] == 0 && left == true) // checks if there is all black after a left (only left condition)
    {
        only_left = true;
    }
    else if (lsa_reading[0] == 0 lsa_reading[2] == 1000 && lsa_reading[1] == 1000 && lsa_reading[3] == 1000 && left == true) // if straight line is found after only left condition OR after Left + straight condition
    {
        if ((left == true) && (only_left == false)) // STR+LEFT
        {
            found_left = true;
        }
        left = false;
        only_left = false;
    }
    else if (lsa_reading[0] == 0 && lsa_reading[1] == 1000 && lsa_reading[2] == 1000 && lsa_reading[3] == 1000 && lsa_reading[4] == 1000) // str + right
    {
        right = true;

        if ((lsa_reading[2] == 1000) && (lsa_reading[1] == 1000 && lsa_reading[3] == 1000) && (right == true)) // STR+RIGHT
        {
            found_straight = true;
            right = false;
        }
    }
    else if (lsa_reading[1] == 1000 && lsa_reading[2] == 1000 && lsa_reading[3] == 1000 && lsa_reading[4] == 1000) // only right
    {
        right = true;

        if ((lsa_reading[0] == 0 && lsa_reading[1] == 0 && lsa_reading[2] == 0 && lsa_reading[3] == 0 && lsa_reading[4] == 0) && (right == true) && (found_straight == false)) // ONLY RIGHT
        {
            found_right = true;
        }
    }
}

char select_turn()
{

    if (found_left == true)
    {
        found_left = false;
        return 'L';
    }
    else if (found_straight == true)
    {
        found_straight = false;
        return 'S';
    }

    else
    {
        return 'B';
    }
}

void line_follow_task(void *arg)
{

    while (1)
    {
        get_raw_lsa(); // funtion that updates the lsa readings

        if (lsa_reading[0] == 1000 && lsa_reading[1] == 1000 && lsa_reading[2] == 1000) // checks left first
        {
            /*This is a condition for + shaped and T shaped node. This portion means there exists a left for sure and the turn is left*/
            left = true;
        }
        else if (lsa_reading[2] == 1000 && lsa_reading[3] == 1000 && lsa_reading[4] == 1000)
        {
            right = true;
        }

        if (lsa_reading[0] == 0 && lsa_reading[4] == 0 && (right == 1 || left == 1)) // It has come to normal state after reading right and left
        {
            if (lsa_reading[1] == 0 && lsa_reading[2] == 0 && lsa_reading[3] == 0) // checks if there is all black after a left (only left condition)
            /*this is the condition of only left and only right*/
            {
                if (left == 1)
                {
                    only_left = true;
                    set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle*1.4);
                    set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD, right_duty_cycle*1.4);
                }
                else
                {
                    set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD, left_duty_cycle*1.4);
                    set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, right_duty_cycle*1.4);
                }
            }
            else 
            {
                if (left == 1)
                {
                    only_left = false;
                    set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle*1.4);
                    set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD, right_duty_cycle*1.4);
                }
                else
                {
                    set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle);
                    set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, right_duty_cycle);
                }
            }
            left = false ;
            right = false ;
            only_left = false ;
        }

        // Line Following starts here
        calculate_error();
        calculate_correction();

        left_duty_cycle = bound((GOOD_DUTY_CYCLE - correction), MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
        right_duty_cycle = bound((GOOD_DUTY_CYCLE + correction), MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);

        set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, left_duty_cycle); /*goes forward in this case*/
        set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, right_duty_cycle);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    ESP_ERROR_CHECK(enable_lsa());
    ESP_ERROR_CHECK(enable_motor_driver());

    xTaskCreate(&line_follow_task, "line_follow_task", 4096, NULL, 1, NULL);
}

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
// end of function lsa_reading[2] == 1000 && lsa_reading[3] == 1000 && lsa_reading[4] == 1000) // checks left first
    {
        left = true;
    }
    else if (lsa_reading[0] == 0 && lsa_reading[1] == 0 && lsa_reading[2] == 0 && lsa_reading[3] == 0 && lsa_reading[4] == 0 && left == true) // checks if there is all black after a left (only left condition)
    {
        only_left = true;
    }
    else if (lsa_reading[0] == 0 lsa_reading[2] == 1000 && lsa_reading[1] == 1000 && lsa_reading[3] == 1000 && left == true) // if straight line is found after only left condition OR after Left + straight condition
    {
        if ((left == true) && (only_left == false)) // STR+LEFT
        {
            found_left = true;
        }
        left = false;
        only_left = false;
    }
    else if (lsa_reading[0] == 0 && lsa_reading[1] == 1000 && lsa_reading[2] == 1000 && lsa_reading[3] == 1000 && lsa_reading[4] == 1000) // str + right
    {
        right = true;

        if ((lsa_reading[2] == 1000) && (lsa_reading[1] == 1000 && lsa_reading[3] == 1000) && (right == true)) // STR+RIGHT
        {
            found_straight = true;
            right = false;
        }
    }
    else if (lsa_reading[1] == 1000 && lsa_reading[2] == 1000 && lsa_reading[3] == 1000 && lsa_reading[4] == 1000) // only right
    {
        right = true;

        if ((lsa_reading[0] == 0 && lsa_reading[1] == 0 && lsa_reading[2] == 0 && lsa_reading[3] == 0 && lsa_reading[4] == 0) && (right == true) && (found_straight == false)) // ONLY RIGHT
        {
            found_right = true;
        }
    }
}

char select_turn()
{

    if (found_left == true)
    {
        found_left = false;
        return 'L';
    }
    else if (found_straight == true)
    {
        found_straight = false;
        return 'S';
    }

    else
    {
        return 'B';
    }
}

void line_follow_task(void *arg)
{

    while (1)
    {
        get_raw_lsa(); // funtion that updates the lsa readings

        if (lsa_reading[0] == 1000 && lsa_reading[1] == 1000 && lsa_reading[2] == 1000) // checks left first
        {
            /*This is a condition for + shaped and T shaped node. This portion means there exists a left for sure and the turn is left*/
            left = true;
        }
        else if (lsa_reading[2] == 1000 && lsa_reading[3] == 1000 && lsa_reading[4] == 1000)
        {
            right = true;
        }

        if (lsa_reading[0] == 0 && lsa_reading[4] == 0 && (right == 1 || left == 1)) // It has come to normal state after reading right and left
        {
            if (lsa_reading[1] == 0 && lsa_reading[2] == 0 && lsa_reading[3] == 0) // checks if there is all black after a left (only left condition)
            /*this is the condition of only left and only right*/
            {
                if (left == 1)
                {
                    only_left = true;
                    set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle*1.4);
                    set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD, right_duty_cycle*1.4);
                }
                else
                {
                    set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD, left_duty_cycle*1.4);
                    set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, right_duty_cycle*1.4);
                }
            }
            else 
            {
                if (left == 1)
                {
                    only_left = false;
                    set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle*1.4);
                    set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD, right_duty_cycle*1.4);
                }
                else
                {
                    set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle);
                    set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, right_duty_cycle);
                }
            }
            left = false ;
            right = false ;
            only_left = false ;
        }

        // Line Following starts here
        calculate_error();
        calculate_correction();

        left_duty_cycle = bound((GOOD_DUTY_CYCLE - correction), MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
        right_duty_cycle = bound((GOOD_DUTY_CYCLE + correction), MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);

        set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, left_duty_cycle); /*goes forward in this case*/
        set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, right_duty_cycle);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    ESP_ERROR_CHECK(enable_lsa());
    ESP_ERROR_CHECK(enable_motor_driver());

    xTaskCreate(&line_follow_task, "line_follow_task", 4096, NULL, 1, NULL);
}

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