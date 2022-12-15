#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sra_board.h"
#include "tuning_http_server.h"

#define MODE NORMAL_MODE
#define BLACK_MARGIN 400
#define WHITE_MARGIN 2000
#define bound_LSA_LOW 0
#define bound_LSA_HIGH 1000
#define IR_GPIO 18
#define BOOT_BUTTON 0
#define BG_LED_1 32
#define BG_LED_2 33
#define BG_LED_3 25
#define BG_LED_4 26
#define BG_LED_5 27
#define BG_LED_6 14
#define BG_LED_7 12
#define BG_LED_8 13
//static const int pin_out[8] = {BG_LED_1, BG_LED_2, BG_LED_3, BG_LED_4, BG_LED_5, BG_LED_6, BG_LED_7, BG_LED_8}; // Array storing all gpio pin numbers connected to LED
//uint64_t bit_mask = (1ULL << BG_LED_1) | (1ULL << BG_LED_2) | (1ULL << BG_LED_3) | (1ULL << BG_LED_4) | (1ULL << BG_LED_5) | (1ULL << BG_LED_6) | (1ULL << BG_LED_7) | (1ULL << BG_LED_8);
/*
 * weights given to respective line sensor
 */
const int weights[4] = {3, 1, -1, -3};
bool s1;
bool s2;
bool s3;
bool s4;
bool prev_s1;
bool prev_s2;
bool prev_s3;
bool prev_s4;
bool once = false;
bool backtrack=false;
/*
 * Motor value boundts
 */
// pid = 1.5 0 5.5
TaskHandle_t taskhandle1 = NULL;
TaskHandle_t taskhandle2 = NULL;
TaskHandle_t taskhandle3 = NULL;
bool finished = false;
int counter2 = 0;
int direction = 0;
int curr_index = 0;
int is_stop_counter = 0;
const int optimum_duty_cycle = 58;
int optimum_duty_cycle_2 = optimum_duty_cycle + 2;
int optimum_duty_cycle_3 = optimum_duty_cycle + 8;
int lower_duty_cycle = 35;
int higher_duty_cycle = 60;
float left_duty_cycle = 0, right_duty_cycle = 0;
int lfr_data[100];
int counter = 0;
int curr_index2 = 0;
bool path_planning=false;
bool doing_left = false;

int processed_lfr_data[100];
int processed_lfr_data2[100];
int processed_lfr_data3[100];

/*
 * Line Following PID Variables
 */
float error = 0, prev_error = 0, difference, cumulative_error, correction;

/*
 * Union containing line sensor readings
 */
line_sensor_array line_sensor_readings;
esp_err_t enable_all_gpio(){

 // Configures all the gpio pins which are connected to LEDs 

gpio_config_t io_conf;   // Configuration struct of gpio pins 

uint64_t bit_mask = (1ULL << BG_LED_1) | (1ULL << BG_LED_2) | (1ULL << BG_LED_3) | (1ULL << BG_LED_4) | (1ULL << BG_LED_5) | (1ULL << BG_LED_6) | (1ULL << BG_LED_7) | (1ULL << BG_LED_8);

io_conf.pin_bit_mask = bit_mask;    // bit mask gives position of gpio pin number to be on . 

io_conf.mode = GPIO_MODE_OUTPUT;    // We need to output from laptop to gpio pins 

io_conf.pull_up_en = 0;

io_conf.pull_down_en = 1;

io_conf.intr_type = GPIO_INTR_DISABLE;    // No Interrupts

esp_err_t err = gpio_config(&io_conf);   // GIVES ESP OK IF CONFIGURED GPIO PINS 

return err;  // Returning the error 


}

void set_all_led(bool state){
uint8_t var = 0x00;
    bool number[8] = {state, state, state, state, state, state, state, state};
  
        var = bool_to_uint8(number);                                                  // A helper function to convert bool array to unsigned int.
        ESP_ERROR_CHECK(set_bar_graph(var));                                          // Setting bar graph led with unsigned int value.
    


}
void lsa_to_bar()
{
    uint8_t var = 0x00;
    bool number[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 4; i++)
    {
        number[7 - i] = (line_sensor_readings.adc_reading[i] < BLACK_MARGIN) ? 0 : 1; // If adc value is less than black margin, then set that bit to 0 otherwise 1.
        var = bool_to_uint8(number);                                                  // A helper function to convert bool array to unsigned int.
        ESP_ERROR_CHECK(set_bar_graph(var));                                          // Setting bar graph led with unsigned int value.
    }
}

void calculate_correction()
{
    error = error * 10; // we need the error correction in range 0-100 so that we can send it directly as duty cycle paramete
    difference = error - prev_error;
    cumulative_error += error;

    cumulative_error = bound(cumulative_error, -30, 30);

    correction = read_pid_const().kp * error + read_pid_const().ki * cumulative_error + read_pid_const().kd * difference;
    prev_error = error;
}

void calculate_error()
{
    int all_black_flag = 1; // assuming initially all black condition
    float weighted_sum = 0, sum = 0;
    float pos = 0;

    for (int i = 0; i < 4; i++)
    {
        if (line_sensor_readings.adc_reading[i] > BLACK_MARGIN)
        {
            all_black_flag = 0;
        }
        weighted_sum += (float)(weights[i]) * (line_sensor_readings.adc_reading[i]);
        sum = sum + line_sensor_readings.adc_reading[i];
    }

    if (sum != 0) // sum can never be 0 but just for safety purposes
    {
        pos = weighted_sum / sum; // This will give us the position wrt line. if +ve then bot is facing left and if -ve the bot is facing to right.
    }

    if (all_black_flag == 1) // If all black then we check for previous error to assign current error.
    {
        if (prev_error > 0)
        {
            error = 2.5;
        }
        else
        {
            error = -2.5;
        }
    }
    else
    {
        error = pos;
    }
}
bool is_aligned()
{

    line_sensor_readings = read_line_sensor();
    for (int i = 0; i < 4; i++)
    {
        line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
        line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
    }
    s1 = line_sensor_readings.adc_reading[0] > 500;
    s2 = line_sensor_readings.adc_reading[1] > 500;
    s3 = line_sensor_readings.adc_reading[2] > 500;
    s4 = line_sensor_readings.adc_reading[3] > 500;
    return (!s1 && s2 && s3 && !s4);
}

void line_follow_task(void *arg)
{
    ESP_ERROR_CHECK(enable_motor_driver(a, NORMAL_MODE));
    ESP_ERROR_CHECK(enable_line_sensor());
    ESP_ERROR_CHECK(enable_bar_graph());
    
    set_all_led(1);
    bool ir_state=true;
    bool aligned;
    bool info[4];
    //bool ir_state = false;
    int i = 0;
    gpio_config_t io_conf; // Configuration struct of gpio pins

    io_conf.mode = GPIO_MODE_OUTPUT; // We need to output from laptop to gpio pins

    io_conf.pull_up_en = 0;

    io_conf.pull_down_en = 1;

    io_conf.intr_type = GPIO_INTR_DISABLE;

#ifdef CONFIG_ENABLE_OLED
    // Declaring the required OLED struct
    u8g2_t oled_config;

    // Initialising the OLED
    ESP_ERROR_CHECK(init_oled(&oled_config));
#endif

    while (true)
    {
        printf("Inside LFR");

        if (doing_left)
        {
            counter++;
            if (counter == 40)
            {
                counter = 0;
                doing_left = false;
            }
        }
        if (!doing_left)
        {
            while (i < curr_index)
            {
                int curr = lfr_data[i];

                while (i < curr_index && lfr_data[i] == curr && curr != 0)
                {
                    i++;
                }
                processed_lfr_data[curr_index2] = curr;
                curr_index2++;
            }
        }
        ir_state = gpio_get_level((gpio_num_t)IR_GPIO);
        line_sensor_readings = read_line_sensor();
        for (int i = 0; i < 4; i++)
        {
            line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
            line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
        }

        calculate_error();
        calculate_correction();
        // lsa_to_bar();

        s1 = line_sensor_readings.adc_reading[0] > 500;
        s2 = line_sensor_readings.adc_reading[1] > 500;
        s3 = line_sensor_readings.adc_reading[2] > 500;
        s4 = line_sensor_readings.adc_reading[3] > 500;

        // aligned=(!s1 && s2 && s3 && !s4);

        if (!ir_state ){
            once =true;
            while (!(!s1 && !s2 && !s3 && !s4)){

            line_sensor_readings = read_line_sensor();
            for(int i = 0; i < 4; i++)
            {
                line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
                line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
            }
            s1=line_sensor_readings.adc_reading[0]>500;
            s2=line_sensor_readings.adc_reading[1]>500;
            s3=line_sensor_readings.adc_reading[2]>500;
            s4=line_sensor_readings.adc_reading[3]>500;

            calculate_error();
            calculate_correction();
            lsa_to_bar();

        left_duty_cycle = bound((optimum_duty_cycle_3 - correction), lower_duty_cycle, higher_duty_cycle);
        right_duty_cycle = bound((optimum_duty_cycle_3 + correction), lower_duty_cycle, higher_duty_cycle);

        set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle);
        set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, right_duty_cycle);
        vTaskDelay(10/portTICK_PERIOD_MS);

            }
           
        }

        if ((s1 && s2 && s3 && !s4) || (prev_s1 && prev_s2 && prev_s3 && !prev_s4))
        {

            while (s1)
            {
                line_sensor_readings = read_line_sensor();
                for (int i = 0; i < 4; i++)
                {
                    line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
                    line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
                }
                s1 = line_sensor_readings.adc_reading[0] > 500;
                s2 = line_sensor_readings.adc_reading[1] > 500;
                s3 = line_sensor_readings.adc_reading[2] > 500;
                s4 = line_sensor_readings.adc_reading[3] > 500;

                // aligned=(!s1 && s2 && s3 && !s4);
                set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD, 60);
                set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, 60);
                // vTaskDelay(500/portTICK_PERIOD_MS);
            }
        }
        else if (((s1 && s2 && s3 && s4) || (prev_s1 && prev_s2 && prev_s3 && prev_s4)) && backtrack){
            lfr_data[curr_index] = 2;
            curr_index += 1;
            doing_left = true;
                       while (s1)
            {

                line_sensor_readings = read_line_sensor();
                for (int i = 0; i < 4; i++)
                {
                    line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
                    line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
                }
                s1 = line_sensor_readings.adc_reading[0] > 500;
                s2 = line_sensor_readings.adc_reading[1] > 500;
                s3 = line_sensor_readings.adc_reading[2] > 500;
                s4 = line_sensor_readings.adc_reading[3] > 500;
                // aligned=(!s1 && s2 && s3 && !s4);
                set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD, 65);
                set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, 65);
                // vTaskDelay(500/portTICK_PERIOD_MS);
            }
            backtrack=false;
        }
        else if (((s1 && s2 && s3 && s4) || (prev_s1 && prev_s2 && prev_s3 && prev_s4) )&& !backtrack)
        {
                printf("INSIDE LEFT CONDITION \n");
            while (s1)
            {
                is_stop_counter++;
                line_sensor_readings = read_line_sensor();
                for (int i = 0; i < 4; i++)
                {
                    line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
                    line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
                }
                s1 = line_sensor_readings.adc_reading[0] > 400;
                s2 = line_sensor_readings.adc_reading[1] > 500;
                s3 = line_sensor_readings.adc_reading[2] > 500;
                s4 = line_sensor_readings.adc_reading[3] > 500;
                  if (is_stop_counter > 40)
                {
                    if (s1 && s2 && s3 && s4){
                finished = true;
                set_motor_speed(MOTOR_A_0, MOTOR_STOP, 0);
                set_motor_speed(MOTOR_A_1, MOTOR_STOP, 0);
                break;
                    }
                 }
                set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, optimum_duty_cycle);
                set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, optimum_duty_cycle);
                
            }
          
            //printf("stop counter : %i \n ", is_stop_counter);
            is_stop_counter=0;
            
            if (!finished){
                  while (!(s1) )
            {

                line_sensor_readings = read_line_sensor();
                for (int i = 0; i < 4; i++)
                {
                    line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
                    line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
                }
                s1 = line_sensor_readings.adc_reading[0] > 500;
                s2 = line_sensor_readings.adc_reading[1] > 500;
                s3 = line_sensor_readings.adc_reading[2] > 500;
                s4 = line_sensor_readings.adc_reading[3] > 500;
                // aligned=(!s1 && s2 && s3 && !s4);
                set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD,65);
                set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD ,65);
                // vTaskDelay(500/portTICK_PERIOD_MS);
            }
                  while ((s1) )
            {

                line_sensor_readings = read_line_sensor();
                for (int i = 0; i < 4; i++)
                {
                    line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
                    line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
                }
                s1 = line_sensor_readings.adc_reading[0] > 500;
                s2 = line_sensor_readings.adc_reading[1] > 500;
                s3 = line_sensor_readings.adc_reading[2] > 500;
                s4 = line_sensor_readings.adc_reading[3] > 500;
                // aligned=(!s1 && s2 && s3 && !s4);
                set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD,65);
                set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD ,65);
                // vTaskDelay(500/portTICK_PERIOD_MS);
            }
            backtrack=true;
            //    while (s1 )
            // {

            //     line_sensor_readings = read_line_sensor();
            //     for (int i = 0; i < 4; i++)
            //     {
            //         line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
            //         line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
            //     }
            //     s1 = line_sensor_readings.adc_reading[0] > 500;
            //     s2 = line_sensor_readings.adc_reading[1] > 500;
            //     s3 = line_sensor_readings.adc_reading[2] > 500;
            //     s4 = line_sensor_readings.adc_reading[3] > 500;
            //     // aligned=(!s1 && s2 && s3 && !s4);
            //     set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD, 62);
            //     set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, 66);
            //     // vTaskDelay(500/portTICK_PERIOD_MS);
            // }
            }
        }
        if (finished){
           
           vTaskSuspend(NULL);
        }
      
        else if ((!s1 && !s2 && !s3 && !s4) && (prev_s4))
        {

            while (!s4)
            {
                // printf("been in right with %d %d %d %d ",s1,s2,s3,s4);

                line_sensor_readings = read_line_sensor();
                for (int i = 0; i < 4; i++)
                {
                    line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
                    line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
                }
                s1 = line_sensor_readings.adc_reading[0] > 500;
                s2 = line_sensor_readings.adc_reading[1] > 500;
                s3 = line_sensor_readings.adc_reading[2] > 500;
                s4 = line_sensor_readings.adc_reading[3] > 500;
                set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, 60);
                set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD, 60);

                // aligned=(!s1 && s2 && s3 && !s4);
            }
             while (s4)
            {
                // printf("been in right with %d %d %d %d ",s1,s2,s3,s4);

                line_sensor_readings = read_line_sensor();
                for (int i = 0; i < 4; i++)
                {
                    line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
                    line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
                }
                s1 = line_sensor_readings.adc_reading[0] > 500;
                s2 = line_sensor_readings.adc_reading[1] > 500;
                s3 = line_sensor_readings.adc_reading[2] > 500;
                s4 = line_sensor_readings.adc_reading[3] > 500;
                set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, 60);
                set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD, 60);

                // aligned=(!s1 && s2 && s3 && !s4);
            }
        }
        else if ((!s1 && !s2 && !s3 && !s4) && (!prev_s1 && !prev_s2 && !prev_s3 && !prev_s4))
        {
            lfr_data[curr_index] = 4;

            curr_index += 1;

            printf("no hiiii");
            while (!(is_aligned()))
            {
                set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD, 60);
                set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, 60);

                // aligned=(!s1 && s2 && s3 && !s4);
            }
        }
        else if ((!s2 || !s3) && s1 && s4)
        {
            printf("inside inverttt");
            while (1)
            {

                prev_s1 = s1;
                prev_s2 = s2;
                prev_s3 = s3;
                prev_s4 = s4;
                line_sensor_readings = read_line_sensor();
                for (int i = 0; i < 4; i++)
                {
                    line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
                    line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
                }
                s1 = line_sensor_readings.adc_reading[0] > 500;
                s2 = line_sensor_readings.adc_reading[1] > 500;
                s3 = line_sensor_readings.adc_reading[2] > 500;
                s4 = line_sensor_readings.adc_reading[3] > 500;
                printf("inside inverrtt %d %d %d %d ", s1, s2, s3, s4);
                if (((int)s1+(int)s2+(int)s3+(int)s4)<2)
                {
                    break;
                }
                for (int i = 0; i < 4; i++)
                {
                    line_sensor_readings.adc_reading[i] = 1000 - line_sensor_readings.adc_reading[i];
                }
                calculate_error();
                calculate_correction();
                // lsa_to_bar();

                left_duty_cycle = bound((optimum_duty_cycle_2 - correction), lower_duty_cycle, higher_duty_cycle);
                right_duty_cycle = bound((optimum_duty_cycle_2 + correction), lower_duty_cycle, higher_duty_cycle);

                set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle);
                set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, right_duty_cycle);
                //vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        }
        // else if (s1 && !s2 && !s3 && s4){
        //     while (!s2 || !s3 )
        //     {
        //         line_sensor_readings = read_line_sensor();
        // for(int i = 0; i < 4; i++)
        // {
        //     line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
        //     line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
        //     line_sensor_readings.adc_reading[i]=1000-line_sensor_readings.adc_reading[i];
        // }

        // calculate_error();
        // calculate_correction();
        // lsa_to_bar();

        // s1=(1000-line_sensor_readings.adc_reading[0])>600;
        // s2=(1000-line_sensor_readings.adc_reading[1])>600;
        // s3=(1000-line_sensor_readings.adc_reading[2])>600;
        // s4=(1000-line_sensor_readings.adc_reading[3])>600;

        // //aligned=(!s1 && s2 && s3 && !s4);

        // printf(" lsa readings : %i %i %i %i",s1,s2,s3,s4);

        // left_duty_cycle = bound((optimum_duty_cycle - correction), lower_duty_cycle, higher_duty_cycle);
        // right_duty_cycle = bound((optimum_duty_cycle + correction), lower_duty_cycle, higher_duty_cycle);

        // set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle);
        // set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, right_duty_cycle);
        //     }

        // }
        line_sensor_readings = read_line_sensor();
        for (int i = 0; i < 4; i++)
        {
            line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
            line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
        }

        calculate_error();
        calculate_correction();
        // lsa_to_bar();

        prev_s1 = s1;
        prev_s2 = s2;
        prev_s3 = s3;
        prev_s4 = s4;

        //  printf(" lsa readings : %i %i %i %i /n",s1,s2,s3,s4);

        // left_duty_cycle = bound((optimum_duty_cycle - correction), lower_duty_cycle, higher_duty_cycle);
        // right_duty_cycle = bound((optimum_duty_cycle + correction), lower_duty_cycle, higher_duty_cycle);

        // aligned=(!s1 && s2 && s3 && !s4);

        for (int o = 0; o < curr_index2; o++)
        {
            printf("%i ", processed_lfr_data[o]);
        }
        printf("\n");

        //printf("%i %i %i %i",s1,s2,s3,s4);
        //  for (int i=0;i<=curr_index-1;i++){
        //      printf("%i ",lfr_data[i]);
        //  }
        // printf("\n");
        // printf("%i %i %i \n",lfr_data[curr_index-1],lfr_data[curr_index-2],lfr_data[curr_index-3]);

        left_duty_cycle = bound((optimum_duty_cycle - correction), lower_duty_cycle, higher_duty_cycle);
        right_duty_cycle = bound((optimum_duty_cycle + correction), lower_duty_cycle, higher_duty_cycle);

        set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle);
        set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, right_duty_cycle);

        // ESP_LOGI("debug","left_duty_cycle:  %f    ::  right_duty_cycle :  %f  :: error :  %f  correction  :  %f  \n",left_duty_cycle, right_duty_cycle, error, correction);
        // ESP_LOGI("debug", "KP: %f ::  KI: %f  :: KD: %f", read_pid_const().kp, read_pid_const().ki, read_pid_const().kd);
#ifdef CONFIG_ENABLE_OLED
        // Diplaying kp, ki, kd values on OLED
        if (read_pid_const().val_changed)
        {
            display_pid_values(read_pid_const().kp, read_pid_const().ki, read_pid_const().kd, &oled_config);
            reset_val_changed_pid_const();
        }
#endif

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}
void path_planning_task(void *arg)
{
    
    set_all_led(0);
    bool finished = false;
    int counter2 = 0;
    int direction = 0;
    int curr_index = 0;

    const int optimum_duty_cycle = 58;
    int optimum_duty_cycle_2 = optimum_duty_cycle + 2;
    int optimum_duty_cycle_3 = optimum_duty_cycle + 8;
    int lower_duty_cycle = 35;
    int higher_duty_cycle = 60;
    float left_duty_cycle = 0, right_duty_cycle = 0;
    bool ir_state=true;
    int counter = 0;
    
    bool aligned;
    bool info[4];
    //bool ir_state = false;
    gpio_config_t io_conf; // Configuration struct of gpio pins

    int k = 0;
    int i = 0;
    int j = 0;
    io_conf.mode = GPIO_MODE_OUTPUT; // We need to output from laptop to gpio pins

    io_conf.pull_up_en = 0;

    io_conf.pull_down_en = 1;

    io_conf.intr_type = GPIO_INTR_DISABLE;

    while (i < curr_index2)
    {
        if (i <= curr_index2 - 3 && processed_lfr_data[i] == 2 && processed_lfr_data[i + 1] == 4 && processed_lfr_data[i + 2] == 2)
        {
            processed_lfr_data2[j] = -1;
            i += 3;
            j++;
        }
        else if (!(processed_lfr_data[i]==4))
        {
            processed_lfr_data2[j] = processed_lfr_data[i];
            i++;
            j++;
        }
    }
    // i = 0;
    // k = 0;
    // while (i < j)
    // {
    //     if (processed_lfr_data2[i] == 2)
    //     {
    //         processed_lfr_data3[k] = 2;
    //         processed_lfr_data3[k + 1] = 2;
    //         k += 2;
    //     }
    //     else
    //     {
    //         processed_lfr_data3[k] = processed_lfr_data2[i];
    //         k++;
    //     }
    //     i++;
    // }

    // processing block of lfr data

    // while (k<=curr_index2-3){
    //     if (processed_lfr_data[k]==2 && processed_lfr_data[k+1]==4 && processed_lfr_data[k+2]==2){
    //         processed_lfr_data2[k]=0;
    //         k+=3;
    //     }
    //     else{
    //         processed_lfr_data2[k]=processed_lfr_data[k];
    //         k+=1;
    //     }
    // }
    // k=0;

#ifdef CONFIG_ENABLE_OLED
    // Declaring the required OLED struct
    u8g2_t oled_config;

    // Initialising the OLED
    ESP_ERROR_CHECK(init_oled(&oled_config));
#endif

    while (true)

    {   
        printf("inside path planning task !! ");
   
           for (int o = 0; o < j; o++)
        {
            printf("%i ", processed_lfr_data2[o]);
        }
        printf("\n");
        if (doing_left)
        {
            counter++;
            if (counter == 40)
            {
                counter = 0;
                
                counter2++;
                doing_left = false;
            }
        }
       
        ir_state = gpio_get_level((gpio_num_t)IR_GPIO);
        line_sensor_readings = read_line_sensor();
        for (int i = 0; i < 4; i++)
        {
            line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
            line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
        }

        calculate_error();
        calculate_correction();
        // lsa_to_bar();

        s1 = line_sensor_readings.adc_reading[0] > 500;
        s2 = line_sensor_readings.adc_reading[1] > 500;
        s3 = line_sensor_readings.adc_reading[2] > 500;
        s4 = line_sensor_readings.adc_reading[3] > 500;
        if (path_planning){
        prev_s1=s1;prev_s2=s2;prev_s3=s3;prev_s4=s4;
        error = 0, prev_error = 0, difference=0, cumulative_error=0, correction=0;
        set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, optimum_duty_cycle);
        set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, optimum_duty_cycle);
        path_planning=false;
        }
        // aligned=(!s1 && s2 && s3 && !s4);

        if (!ir_state ){
            printf("inside 5");
            once =true;
            while (!(!s1 && !s2 && !s3 && !s4)){

            line_sensor_readings = read_line_sensor();
            for(int i = 0; i < 4; i++)
            {
                line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
                line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
            }
            s1=line_sensor_readings.adc_reading[0]>500;
            s2=line_sensor_readings.adc_reading[1]>500;
            s3=line_sensor_readings.adc_reading[2]>500;
            s4=line_sensor_readings.adc_reading[3]>500;

            calculate_error();
            calculate_correction();
            //lsa_to_bar();

        left_duty_cycle = bound((optimum_duty_cycle_3 - correction), lower_duty_cycle, higher_duty_cycle);
        right_duty_cycle = bound((optimum_duty_cycle_3 + correction), lower_duty_cycle, higher_duty_cycle);

        set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle);
        set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, right_duty_cycle);
        vTaskDelay(10/portTICK_PERIOD_MS);

            }
            
        }
        if ((s1 && s2 && s3 && !s4) || (prev_s1 && prev_s2 && prev_s3 && !prev_s4))
        {printf("inside 3");

            while (s1)
            {
                line_sensor_readings = read_line_sensor();
                for (int i = 0; i < 4; i++)
                {
                    line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
                    line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
                }
                s1 = line_sensor_readings.adc_reading[0] > 500;
                s2 = line_sensor_readings.adc_reading[1] > 500;
                s3 = line_sensor_readings.adc_reading[2] > 500;
                s4 = line_sensor_readings.adc_reading[3] > 500;

                // aligned=(!s1 && s2 && s3 && !s4);
                set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD, 60);
                set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, 60);
                // vTaskDelay(500/portTICK_PERIOD_MS);
            }
        }
                else if (((s1 && s2 && s3 && s4) ) && backtrack){
            printf("inside 3");
            doing_left = true;
            
                       while (s1 )
            {   

                line_sensor_readings = read_line_sensor();
                for (int i = 0; i < 4; i++)
                {
                    line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
                    line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
                }
                s1 = line_sensor_readings.adc_reading[0] > 500;
                s2 = line_sensor_readings.adc_reading[1] > 500;
                s3 = line_sensor_readings.adc_reading[2] > 500;
                s4 = line_sensor_readings.adc_reading[3] > 500;
                // aligned=(!s1 && s2 && s3 && !s4);
                set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD, 65);
                set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, 65);
                // vTaskDelay(500/portTICK_PERIOD_MS);
            }
            
        }
        else if (((s1 && s2 && s3 && s4)  )&& !backtrack)
        {
                printf("INSIDE LEFT CONDITION \n");
            while (s1)
            {
                is_stop_counter++;
                line_sensor_readings = read_line_sensor();
                for (int i = 0; i < 4; i++)
                {
                    line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
                    line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
                }
                s1 = line_sensor_readings.adc_reading[0] > 400;
                s2 = line_sensor_readings.adc_reading[1] > 500;
                s3 = line_sensor_readings.adc_reading[2] > 500;
                s4 = line_sensor_readings.adc_reading[3] > 500;
                  if (is_stop_counter > 40)
                {
                    if (s1 && s2 && s3 && s4){
                finished = true;
                set_motor_speed(MOTOR_A_0, MOTOR_STOP, 0);
                set_motor_speed(MOTOR_A_1, MOTOR_STOP, 0);
                break;
                    }
                 }
                     calculate_error();
                 calculate_correction();
                
        left_duty_cycle = bound((optimum_duty_cycle - correction), lower_duty_cycle, higher_duty_cycle);
        right_duty_cycle = bound((optimum_duty_cycle + correction), lower_duty_cycle, higher_duty_cycle);

        set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle);
        set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, right_duty_cycle);
            }
          
            //printf("stop counter : %i \n ", is_stop_counter);
            is_stop_counter=0;
            printf("%i LFR DATA AT LEFT PLUS TURN ",processed_lfr_data2[counter2]);
            if (!(processed_lfr_data2[counter2]==-1)){
            if (!finished){
                  while (!(s1) )
            {

                line_sensor_readings = read_line_sensor();
                for (int i = 0; i < 4; i++)
                {
                    line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
                    line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
                }
                s1 = line_sensor_readings.adc_reading[0] > 500;
                s2 = line_sensor_readings.adc_reading[1] > 500;
                s3 = line_sensor_readings.adc_reading[2] > 500;
                s4 = line_sensor_readings.adc_reading[3] > 500;
                // aligned=(!s1 && s2 && s3 && !s4);
                set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD,65);
                set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD ,65);
                // vTaskDelay(500/portTICK_PERIOD_MS);
            }
                  while ((s1) )
            {

                line_sensor_readings = read_line_sensor();
                for (int i = 0; i < 4; i++)
                {
                    line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
                    line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
                }
                s1 = line_sensor_readings.adc_reading[0] > 500;
                s2 = line_sensor_readings.adc_reading[1] > 500;
                s3 = line_sensor_readings.adc_reading[2] > 500;
                s4 = line_sensor_readings.adc_reading[3] > 500;
                // aligned=(!s1 && s2 && s3 && !s4);
                set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD,65);
                set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD ,65);
                // vTaskDelay(500/portTICK_PERIOD_MS);
            }
            backtrack=true;
            }
            //    while (s1 )
            // {

            //     line_sensor_readings = read_line_sensor();
            //     for (int i = 0; i < 4; i++)
            //     {
            //         line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
            //         line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
            //     }
            //     s1 = line_sensor_readings.adc_reading[0] > 500;
            //     s2 = line_sensor_readings.adc_reading[1] > 500;
            //     s3 = line_sensor_readings.adc_reading[2] > 500;
            //     s4 = line_sensor_readings.adc_reading[3] > 500;
            //     // aligned=(!s1 && s2 && s3 && !s4);
            //     set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD, 62);
            //     set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, 66);
            //     // vTaskDelay(500/portTICK_PERIOD_MS);
            // }
            }
            if ((processed_lfr_data2[counter2]==-1)){
                counter2++;
            }
        }
        if (finished){
           
           vTaskSuspend(NULL);
        }

        else if ((!s1 && !s2 && !s3 && !s4) && (prev_s4))
        {   printf("inside 2");
            

                      while (!s4)
            {
                // printf("been in right with %d %d %d %d ",s1,s2,s3,s4);

                line_sensor_readings = read_line_sensor();
                for (int i = 0; i < 4; i++)
                {
                    line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
                    line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
                }
                s1 = line_sensor_readings.adc_reading[0] > 500;
                s2 = line_sensor_readings.adc_reading[1] > 500;
                s3 = line_sensor_readings.adc_reading[2] > 500;
                s4 = line_sensor_readings.adc_reading[3] > 500;
                set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, 60);
                set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD, 60);

                // aligned=(!s1 && s2 && s3 && !s4);
            }
             while (s4)
            {
                // printf("been in right with %d %d %d %d ",s1,s2,s3,s4);

                line_sensor_readings = read_line_sensor();
                for (int i = 0; i < 4; i++)
                {
                    line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
                    line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
                }
                s1 = line_sensor_readings.adc_reading[0] > 500;
                s2 = line_sensor_readings.adc_reading[1] > 500;
                s3 = line_sensor_readings.adc_reading[2] > 500;
                s4 = line_sensor_readings.adc_reading[3] > 500;
                set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, 60);
                set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD, 60);

                // aligned=(!s1 && s2 && s3 && !s4);
            }
        }
        else if ((!s1 && !s2 && !s3 && !s4) && (!prev_s1 && !prev_s2 && !prev_s3 && !prev_s4))
        {
           

            
            while (!(is_aligned()))
            {
                set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD, 60);
                set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, 60);

                // aligned=(!s1 && s2 && s3 && !s4);
            }
        }
        else if ((!s2 || !s3) && s1 && s4 )
        {
            printf("inside inverttt");
           
            while (1)
            {

                prev_s1 = s1;
                prev_s2 = s2;
                prev_s3 = s3;
                prev_s4 = s4;
                line_sensor_readings = read_line_sensor();
                for (int i = 0; i < 4; i++)
                {
                    line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
                    line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
                }
                s1 = line_sensor_readings.adc_reading[0] > 500;
                s2 = line_sensor_readings.adc_reading[1] > 500;
                s3 = line_sensor_readings.adc_reading[2] > 500;
                s4 = line_sensor_readings.adc_reading[3] > 500;
                printf("inside inverrtt %d %d %d %d ", s1, s2, s3, s4);
                if (((int)s1+(int)s2+(int)s3+(int)s4)<2)
                {
                    break;
                }
                for (int i = 0; i < 4; i++)
                {
                    line_sensor_readings.adc_reading[i] = 1000 - line_sensor_readings.adc_reading[i];
                }
                calculate_error();
                calculate_correction();
                // lsa_to_bar();

                left_duty_cycle = bound((optimum_duty_cycle_2 - correction), lower_duty_cycle, higher_duty_cycle);
                right_duty_cycle = bound((optimum_duty_cycle_2 + correction), lower_duty_cycle, higher_duty_cycle);

                set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle);
                set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, right_duty_cycle);
                //vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        }
        // else if (s1 && !s2 && !s3 && s4){
        //     while (!s2 || !s3 )
        //     {
        //         line_sensor_readings = read_line_sensor();
        // for(int i = 0; i < 4; i++)
        // {
        //     line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
        //     line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
        //     line_sensor_readings.adc_reading[i]=1000-line_sensor_readings.adc_reading[i];
        // }

        // calculate_error();
        // calculate_correction();
        // lsa_to_bar();

        // s1=(1000-line_sensor_readings.adc_reading[0])>600;
        // s2=(1000-line_sensor_readings.adc_reading[1])>600;
        // s3=(1000-line_sensor_readings.adc_reading[2])>600;
        // s4=(1000-line_sensor_readings.adc_reading[3])>600;

        // //aligned=(!s1 && s2 && s3 && !s4);

        // printf(" lsa readings : %i %i %i %i",s1,s2,s3,s4);

        // left_duty_cycle = bound((optimum_duty_cycle - correction), lower_duty_cycle, higher_duty_cycle);
        // right_duty_cycle = bound((optimum_duty_cycle + correction), lower_duty_cycle, higher_duty_cycle);

        // set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle);
        // set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, right_duty_cycle);
        //     }

        // }
        line_sensor_readings = read_line_sensor();
        for (int i = 0; i < 4; i++)
        {
            line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
            line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
        }

        calculate_error();
        calculate_correction();
        // lsa_to_bar();

        prev_s1 = s1;
        prev_s2 = s2;
        prev_s3 = s3;
        prev_s4 = s4;

        //  printf(" lsa readings : %i %i %i %i",s1,s2,s3,s4);

        // left_duty_cycle = bound((optimum_duty_cycle - correction), lower_duty_cycle, higher_duty_cycle);
        // right_duty_cycle = bound((optimum_duty_cycle + correction), lower_duty_cycle, higher_duty_cycle);

        // aligned=(!s1 && s2 && s3 && !s4);

     

        //printf("%i %i %i %i", s1, s2, s3, s4);
        // for (int i=0;i<=curr_index-1;i++){
        //     printf("%i ",lfr_data[i]);
        // }
        printf("\n");
        // printf("%i %i %i \n",lfr_data[curr_index-1],lfr_data[curr_index-2],lfr_data[curr_index-3]);

        left_duty_cycle = bound((optimum_duty_cycle - correction), lower_duty_cycle, higher_duty_cycle);
        right_duty_cycle = bound((optimum_duty_cycle + correction), lower_duty_cycle, higher_duty_cycle);

        set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle);
        set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, right_duty_cycle);

        // ESP_LOGI("debug","left_duty_cycle:  %f    ::  right_duty_cycle :  %f  :: error :  %f  correction  :  %f  \n",left_duty_cycle, right_duty_cycle, error, correction);
        // ESP_LOGI("debug", "KP: %f ::  KI: %f  :: KD: %f", read_pid_const().kp, read_pid_const().ki, read_pid_const().kd);
#ifdef CONFIG_ENABLE_OLED
        // Diplaying kp, ki, kd values on OLED
        if (read_pid_const().val_changed)
        {
            display_pid_values(read_pid_const().kp, read_pid_const().ki, read_pid_const().kd, &oled_config);
            reset_val_changed_pid_const();
        }
#endif

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}
void suspend_resume_tasks(void *p)
{
    // Task used for suspending task 1 and task 2 when boot button is pressed . Has Higher priority than both of these .

    static bool switcher = 1; // switcher variable used to switch between resume and suspend functions

    while (1)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);

        bool boot_button_state = gpio_get_level((gpio_num_t)BOOT_BUTTON); // Gets the state of the boot button . If it is pressed gpio pin becomes low level.

        if (!boot_button_state)
        { // if boot buttton is pressed :

            if (switcher)
            { // if tasks are to be suspended
                path_planning=true;
                vTaskSuspend(taskhandle1);
                vTaskResume(taskhandle2);
                // Suspend Tasks
                vTaskDelay(3000 / portTICK_PERIOD_MS); // Wait some time . To prevent immediately switching to resume
            }

            else
            { // if tasks are to be resumed

                vTaskSuspend(taskhandle2);
                vTaskResume(taskhandle1);

                vTaskDelay(3000 / portTICK_PERIOD_MS); // Wait some time . To prevent immediately switching to suspend
            }

            switcher = !switcher; // if tasks are suspended , next time boot button will resume them and vice versa .
        }
    }
}
void app_main()
{
    xTaskCreate(&line_follow_task, "line_follow_task", 4096, NULL, 1, &taskhandle1);
    xTaskCreate(&path_planning_task, "Task 2", 4096, NULL, 1, &taskhandle2);
    vTaskSuspend(taskhandle2);
    xTaskCreate(suspend_resume_tasks, "SRTASK", 2048, NULL, 4, &taskhandle3);
    start_tuning_http_server();
}