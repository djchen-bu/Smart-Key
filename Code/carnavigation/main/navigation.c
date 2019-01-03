#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include "driver/uart.h"

#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "soc/rmt_reg.h"
#include "driver/gpio.h"

#include "esp_types.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

#define GPIO_PWM0A_OUT 32   //Set GPIO 15 as PWM0A Left side
#define GPIO_PWM0B_OUT 25   //Set GPIO 16 as PWM0B Right side

#define BUF_SIZE (1024)

#define GPIO_R1 21
#define GPIO_R2 12
#define GPIO_L1 15 
#define GPIO_L2 14

#define GPIO_ULTRA 27

#define GPIO_LEFT_SPEED 36
#define GPIO_RIGHT_SPEED 39

#define ECHO_TEST_TXD   17
#define ECHO_TEST_RXD   16
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)

#define BEACON_TEST_TXD   26 //TX pin
#define BEACON_TEST_RXD   34 //RX pin

#define BUF_SIZE (1024)


//static const char *RMT_TX_TAG = "RMT Tx";

#define RMT_TX_CHANNEL 1
#define RMT_TX_GPIO 33 // had to change 

#define RMT_RX_CHANNEL 0
#define RMT_RX_GPIO_NUM 16
#define rmt_item32_tIMEOUT_US  9500   /*!< RMT receiver timeout value(us) */
#define RMT_CLK_DIV 100
#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)   /*!< RMT counter value for 10 us.(Source clock is APB clock) */

#define SAMPLE_CNT  (10)

#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL_SEC   (1.0) // sample test interval for the first timer
#define PID_INTERVAL_SEC   (0.1) // 100ms PID evaluation interval
#define TEST_WITHOUT_RELOAD   0
#define TEST_WITH_RELOAD 1

#define speed_of_sound_cm 34300.0

#define PI 3.14159
#define WHEEL_DIAMETER 2.5 //in centimeters

// v PID parameters v
#define dt 0.1 //delta t = 100ms
#define Kp 1.0
#define Ki 0.0
#define Kd 1.5

#define LIDAR_THRESH 30.0 // cm
#define turning_time 800

volatile double front_distance = 0;
volatile double speed[2]; //0 for left wheel speed; 1 for right wheel speed
// v Interrupt flag v
volatile bool trigger = false;

volatile int state = 4;
volatile int prev_state = 4;

volatile int baseline;
rmt_item32_t trig[1] = {
{{{ 10, 1 , 0 , 0 }}},
};

static void lidar_initialize()
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
}

static void ir_init()
{
    uart_config_t uart_config = {
        .baud_rate = 1200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, BEACON_TEST_TXD, BEACON_TEST_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_2, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_set_line_inverse(UART_NUM_2, UART_INVERSE_RXD);
}

static void lidar_distance()
{
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    uint16_t value;
    int length;

    while(1) {
        length = 0;
        uart_get_buffered_data_len(UART_NUM_1,(size_t*)&length);
       
        uart_read_bytes(UART_NUM_1,data,length,100);

        if((data[0] == 89) && (data[1] == 89)){
            value = (data[3]<<8)+data[2];
            front_distance = (double)value;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);   
    }
}

static void rmt_tx_init()
{
    rmt_config_t config;
    config.rmt_mode = RMT_MODE_TX;
    config.channel = RMT_TX_CHANNEL;
    config.gpio_num = RMT_TX_GPIO;
    config.mem_block_num = 1;
    config.tx_config.loop_en = 0;

    // enable the carrier to be able to hear the Morse sound
    // if the RMT_TX_GPIO is connected to a speaker
    config.tx_config.carrier_en = 0;
    config.tx_config.idle_output_en = 1;
    config.tx_config.idle_level = 0;
    config.tx_config.carrier_level = 1;

    // set the maximum clock divider to be able to output
    // RMT pulses in range of about one hundred milliseconds

    // CHANGE CLOCK DIVIDER TO 80 TO HAVE EACH UNIT TO BE APPROXIMATELY 1US.
    config.clk_div = 80; //255;

    rmt_config(&config);
    rmt_driver_install(config.channel, 0, 0);
}

static void timer_ultrasonic_init(int timer_idx, 
    bool auto_reload, double timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = auto_reload;
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);
}

void IRAM_ATTR timer_group0_isr(void *para)
{
    // All we want to do is increase the seconds by 1 every trigger
    int timer_idx = (int) para;

    /* Clear the interrupt
       and update the alarm time for the timer with without reload */
    TIMERG0.int_clr_timers.t0 = 1;
    // TIMERG0.int_clr_timers.t1 = 1;

    /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;

    // Set trigger flag as true.
    trigger = true;
}

static void PID_timer_init(int timer_idx, 
    bool auto_reload, double timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = auto_reload;
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);
    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr, 
        (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);
}

static void timer_wheel_init(int timer_idx, 
    bool auto_reload, double timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = auto_reload;
    timer_init(TIMER_GROUP_1, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_1, timer_idx, 0x00000000ULL);
}


static void mcpwm_initialize()
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, GPIO_PWM0B_OUT);

    gpio_pad_select_gpio(GPIO_R1);
    gpio_pad_select_gpio(GPIO_R2);
    gpio_pad_select_gpio(GPIO_L1);
    gpio_pad_select_gpio(GPIO_L2);

    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_R1, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_R2, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_L1, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_L2, GPIO_MODE_OUTPUT);

    gpio_set_level(GPIO_R1, 0);
    gpio_set_level(GPIO_R2, 0);
    gpio_set_level(GPIO_L1, 0);
    gpio_set_level(GPIO_L2, 0);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 500;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);
}

static double us_get_distance_cm(int timer_idx, int echo_pin)
{
    int number_of_items = sizeof(trig) / sizeof(trig[0]);
    rmt_write_items(RMT_TX_CHANNEL, trig, number_of_items, true); // send trigger signal
    double time, distance;
    int timeout = 0;

    while (1) {
        if (!gpio_get_level(echo_pin)){
            
            continue; // dont do anything unless the level changes.

        } else {

            timer_start(TIMER_GROUP_0, timer_idx); // start timer
            while (gpio_get_level(echo_pin)) {
                ; // wait for the level to go down again.
            }
            timer_pause(TIMER_GROUP_0, timer_idx); // stop timer
            timer_get_counter_time_sec(TIMER_GROUP_0, timer_idx, &time);
            timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL); // reset counter value.
            // t = 2d / v;
            distance = (time * speed_of_sound_cm)/ 2;
            //printf("Echo time: %fs, Distance: %fcm\n", time, distance);
            return distance;

        }
    }
}

static void wheel_speed(){
    int lanalog;
    int ranalog;
    double lcount;
    double rcount;
    double loldcount = 0;
    double roldcount = 0;

    gpio_pad_select_gpio(GPIO_LEFT_SPEED);
    gpio_pad_select_gpio(GPIO_RIGHT_SPEED);
    gpio_set_direction(GPIO_LEFT_SPEED, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_RIGHT_SPEED, GPIO_MODE_INPUT);

    timer_start(TIMER_GROUP_1, TIMER_0);

    while(1){
        lanalog = gpio_get_level(GPIO_LEFT_SPEED);
        ranalog = gpio_get_level(GPIO_RIGHT_SPEED);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        if(!lanalog && gpio_get_level(GPIO_LEFT_SPEED)){
            timer_get_counter_time_sec(TIMER_GROUP_1, TIMER_0, &lcount);
            speed[0] = PI * WHEEL_DIAMETER / (lcount - loldcount);
            loldcount = lcount;
        }
        if(!ranalog && gpio_get_level(GPIO_RIGHT_SPEED)){
            timer_get_counter_time_sec(TIMER_GROUP_1, TIMER_0, &rcount);
            speed[1] = PI * WHEEL_DIAMETER / (rcount - roldcount);
            roldcount = rcount;
        }

    }
}

static void motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, float duty_cycle)
{
    gpio_set_level(GPIO_R1, 0);
    gpio_set_level(GPIO_R2, 1);
    gpio_set_level(GPIO_L1, 1);
    gpio_set_level(GPIO_L2, 0);

    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state   
}

static void motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, float duty_cycle)
{
    gpio_set_level(GPIO_R1, 1);
    gpio_set_level(GPIO_R2, 0);
    gpio_set_level(GPIO_L1, 0);
    gpio_set_level(GPIO_L2, 1);

    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

static void motor_turn_left(mcpwm_unit_t mcpwm_num1, mcpwm_unit_t mcpwm_num2, mcpwm_timer_t timer_num1 , mcpwm_timer_t timer_num2, float duty_cycle)
{
    gpio_set_level(GPIO_R1, 0);
    gpio_set_level(GPIO_R2, 1);
    gpio_set_level(GPIO_L1, 0);
    gpio_set_level(GPIO_L2, 0);

    mcpwm_set_duty(mcpwm_num1, timer_num1, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty(mcpwm_num2, timer_num2, MCPWM_OPR_A, duty_cycle);

    mcpwm_set_duty_type(mcpwm_num1, timer_num1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(mcpwm_num2, timer_num2, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
}

static void motor_turn_right(mcpwm_unit_t mcpwm_num1, mcpwm_unit_t mcpwm_num2, mcpwm_timer_t timer_num1 , mcpwm_timer_t timer_num2, float duty_cycle)
{
    gpio_set_level(GPIO_R1, 0);
    gpio_set_level(GPIO_R2, 0);
    gpio_set_level(GPIO_L1, 1);
    gpio_set_level(GPIO_L2, 0);

    mcpwm_set_duty(mcpwm_num1, timer_num1, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty(mcpwm_num2, timer_num2, MCPWM_OPR_A, duty_cycle);

    mcpwm_set_duty_type(mcpwm_num1, timer_num1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(mcpwm_num2, timer_num2, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
}

static void motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
{

    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);

}

static void uart_receive()
{
    int i = 0;
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while(1){
        int length = 0;

        uart_get_buffered_data_len(UART_NUM_2,(size_t*)&length);
        int len = uart_read_bytes(UART_NUM_2, data, BUF_SIZE, 20 / portTICK_RATE_MS);

        for(i = 0; i < 16; i++)
        {
            if(data[i] == 0x0A)
            {
                state = data[i+1];
                //printf("State: %d\n", prev_state);
            }
        }
        vTaskDelay(200/portTICK_RATE_MS);
    }

}

static void pid_compute()
{
    timer_start(TIMER_GROUP_0, TIMER_0);
    double integral, derivative, error, prev_error, output;
    prev_error = 0.0;
    integral = 0.0;
    motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70.0); // L
    motor_forward(MCPWM_UNIT_1, MCPWM_TIMER_1, 70.0); // R
    while(1){
        if (trigger) {
            double value = us_get_distance_cm(TIMER_1, GPIO_ULTRA);
            //printf("US reading: %fcm\n",value);
            error = baseline - value;
            //dt = dt + 1; // what is this?
            integral = integral + error * dt;
            derivative = (error - prev_error) / dt;
            output = (Kp * error) + (Ki * integral) + (Kd * derivative);
            prev_error = error;
            trigger = false;

            // Once evaluated, act upon the speeds of the wheels.
            // base duty: 70.0
            if (error > 2.0 && front_distance >= LIDAR_THRESH) // positive error means we are moving right (towards wall)
            {
                //motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70.0 + ((output/baseline) * -100.0)); // L
                motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                motor_forward(MCPWM_UNIT_1, MCPWM_TIMER_1, 60.0 + ((output/baseline) * 10.0)); // R
                //printf("Correction: %f\n", 70.0 + ((output/baseline) * 10.0));
            } 

            else if (error < -2.0 && front_distance >= LIDAR_THRESH) // negative error means we are moving left (away from wall)
            /*
            NOTE: When the car is too far away from the wall, it starts spinning and freaks out
            */
            {
                motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 60.0 + ((output/baseline) * -10.0)); // L
                //motor_forward(MCPWM_UNIT_1, MCPWM_TIMER_1, 70.0 + ((output/baseline) * 100.0)); // R
                motor_stop(MCPWM_UNIT_1, MCPWM_TIMER_1);
                //printf("Correction: %f\n", 70.0 + ((output/baseline) * -10.0));
            } 

            else if ( front_distance >= LIDAR_THRESH)
            {
                motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 90.0); // L
                motor_forward(MCPWM_UNIT_1, MCPWM_TIMER_1, 90.0); // R
            }

            else if ( front_distance < LIDAR_THRESH - 3.0)
            {
                motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                motor_stop(MCPWM_UNIT_1, MCPWM_TIMER_1);
            }
        }
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

static void main_task()
{
    
    while (1) {
        switch(prev_state){
            case 0:
                if(prev_state != state && front_distance < LIDAR_THRESH){
                    prev_state = state;
                    timer_pause(TIMER_GROUP_0, TIMER_0); // stop the PID evaluation
                    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL); // reset counter value.

                    motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                    motor_forward(MCPWM_UNIT_1, MCPWM_TIMER_1, 70.0); // R
                    vTaskDelay(turning_time/portTICK_PERIOD_MS);

                    motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                    motor_stop(MCPWM_UNIT_1, MCPWM_TIMER_1);

                    baseline = (int) us_get_distance_cm(TIMER_1, GPIO_ULTRA);
                    
                    timer_start(TIMER_GROUP_0, TIMER_0);
                }
                break;
            case 1:
                if(prev_state != state && front_distance < LIDAR_THRESH){
                    prev_state = state;
                    timer_pause(TIMER_GROUP_0, TIMER_0); // stop the PID evaluation
                    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL); // reset counter value.

                    motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                    motor_forward(MCPWM_UNIT_1, MCPWM_TIMER_1, 70.0); // R
                    vTaskDelay(turning_time/portTICK_PERIOD_MS);

                    motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                    motor_stop(MCPWM_UNIT_1, MCPWM_TIMER_1);

                    baseline = (int) us_get_distance_cm(TIMER_1, GPIO_ULTRA);
                    
                    timer_start(TIMER_GROUP_0, TIMER_0);
                }
                break;
            case 2:
                if(prev_state != state && front_distance < LIDAR_THRESH){
                    prev_state = state;
                    timer_pause(TIMER_GROUP_0, TIMER_0); // stop the PID evaluation
                    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL); // reset counter value.

                    motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                    motor_forward(MCPWM_UNIT_1, MCPWM_TIMER_1, 70.0); // R
                    vTaskDelay(turning_time/portTICK_PERIOD_MS);

                    motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                    motor_stop(MCPWM_UNIT_1, MCPWM_TIMER_1);

                    baseline = (int) us_get_distance_cm(TIMER_1, GPIO_ULTRA);
                    
                    timer_start(TIMER_GROUP_0, TIMER_0);
                }
                break;
            case 3:
                timer_pause(TIMER_GROUP_0, TIMER_0); // stop the PID evaluation
                timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL); // reset counter value.
                motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                motor_stop(MCPWM_UNIT_1, MCPWM_TIMER_1);              
                break;
            case 4:
                if(prev_state != state && front_distance < LIDAR_THRESH){
                    prev_state = state;
                    timer_pause(TIMER_GROUP_0, TIMER_0); // stop the PID evaluation
                    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL); // reset counter value.

                    motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                    motor_forward(MCPWM_UNIT_1, MCPWM_TIMER_1, 70.0); // R
                    vTaskDelay(turning_time/portTICK_PERIOD_MS);

                    motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
                    motor_stop(MCPWM_UNIT_1, MCPWM_TIMER_1);

                    baseline = (int) us_get_distance_cm(TIMER_1, GPIO_ULTRA);

                    timer_start(TIMER_GROUP_0, TIMER_0);
                }
                break;
            /*
            default:
                state = prev_state;
                vTaskDelay(10/portTICK_PERIOD_MS);
                break;
            */
        }
    }
}

void app_main()
{
    mcpwm_initialize();
    lidar_initialize();
    rmt_tx_init();
    ir_init();
    timer_ultrasonic_init(TIMER_0, TEST_WITH_RELOAD, TIMER_INTERVAL_SEC);
    timer_ultrasonic_init(TIMER_1, TEST_WITH_RELOAD, PID_INTERVAL_SEC);
    gpio_pad_select_gpio(GPIO_ULTRA);
    gpio_set_direction(GPIO_ULTRA, GPIO_MODE_INPUT);
    timer_wheel_init(TIMER_0, TEST_WITH_RELOAD, TIMER_INTERVAL_SEC);
    PID_timer_init(TIMER_0, TEST_WITH_RELOAD, PID_INTERVAL_SEC);

    baseline = (int) us_get_distance_cm(TIMER_1, GPIO_ULTRA); // The baseline for distance from the wall is set every time the program starts.

    xTaskCreate(main_task, "main_task", 4096, NULL, 5, NULL);
    xTaskCreate(pid_compute, "pid_compute", 4096, NULL, 5, NULL);
    xTaskCreate(lidar_distance, "lidar_distance", 4096, NULL, 4, NULL);
    xTaskCreate(uart_receive, "uart_receive", 4096, NULL, 4, NULL);
    //xTaskCreate(wheel_speed, "wheel_speed", 4096, NULL, 5, NULL);
}
