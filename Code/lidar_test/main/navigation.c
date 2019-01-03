#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/uart.h"

#define GPIO_PWM0A_OUT 32   //Set GPIO 15 as PWM0A
#define GPIO_PWM0B_OUT 25   //Set GPIO 16 as PWM0B

#define BUF_SIZE (1024)

#define GPIO_R1 21
#define GPIO_R2 12
#define GPIO_L1 15 
#define GPIO_L2 14

#define ECHO_TEST_TXD   17
#define ECHO_TEST_RXD   16
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)

volatile int front_distance = 0;

static void echo_task()
{
    /* Configure parameters of an UART driver,
    * communication pins and install the driver */
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

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    uint16_t value; 
    int i=0; 
    while (1) {
        int length =0; 
        uart_get_buffered_data_len(UART_NUM_1,(size_t*)&length);
        if (length == 0)
        {
            vTaskDelay(1000/portTICK_PERIOD_MS);
            continue; 
        }
        int len = uart_read_bytes(UART_NUM_1,data,length,100);
        //finding the header one and header two. Then read the MSB and LSB of the distance
        if((data[i] == 89) && (data[i+1] == 89)) 
        {
            value = (data[i+3]<<8)+data[i+2];
            front_distance = (int)value; 
            printf("distance:%d",front_distance);
        }
        printf("cm\n");
    }

}
static void mcpwm_example_gpio_initialize()
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

}

static void motor_forward(mcpwm_unit_t mcpwm_num1, mcpwm_unit_t mcpwm_num2, mcpwm_timer_t timer_num1 , mcpwm_timer_t timer_num2, float duty_cycle)
{
    gpio_set_level(GPIO_R1, 0);
    gpio_set_level(GPIO_R2, 1);
    gpio_set_level(GPIO_L1, 1);
    gpio_set_level(GPIO_L2, 0);

    mcpwm_set_duty(mcpwm_num1, timer_num1, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty(mcpwm_num2, timer_num2, MCPWM_OPR_A, duty_cycle);

    mcpwm_set_duty_type(mcpwm_num1, timer_num1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
    mcpwm_set_duty_type(mcpwm_num2, timer_num2, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);    
}

static void motor_backward(mcpwm_unit_t mcpwm_num1, mcpwm_unit_t mcpwm_num2, mcpwm_timer_t timer_num1 , mcpwm_timer_t timer_num2, float duty_cycle)
{
    gpio_set_level(GPIO_R1, 1);
    gpio_set_level(GPIO_R2, 0);
    gpio_set_level(GPIO_L1, 0);
    gpio_set_level(GPIO_L2, 1);

    mcpwm_set_duty(mcpwm_num1, timer_num1, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty(mcpwm_num2, timer_num2, MCPWM_OPR_A, duty_cycle);

    mcpwm_set_duty_type(mcpwm_num1, timer_num1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(mcpwm_num2, timer_num2, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);  //call this each time, if operator was previously in low/high state
}

static void motor_right(mcpwm_unit_t mcpwm_num1, mcpwm_unit_t mcpwm_num2, mcpwm_timer_t timer_num1 , mcpwm_timer_t timer_num2, float duty_cycle)
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

static void motor_left(mcpwm_unit_t mcpwm_num1, mcpwm_unit_t mcpwm_num2, mcpwm_timer_t timer_num1 , mcpwm_timer_t timer_num2, float duty_cycle)
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

static void motor_stop(mcpwm_unit_t mcpwm_num1, mcpwm_unit_t mcpwm_num2, mcpwm_timer_t timer_num1, mcpwm_timer_t timer_num2)
{

    mcpwm_set_signal_low(mcpwm_num1, timer_num1, MCPWM_OPR_A);
    mcpwm_set_signal_low(mcpwm_num2, timer_num2, MCPWM_OPR_A);

}

static void motor_control(void *arg)
{
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 500;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);
    while (1) {
        if (front_distance >= 45)
        {
            motor_forward(MCPWM_UNIT_0, MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_TIMER_1, 80.0);
            vTaskDelay(1000/portTICK_PERIOD_MS);
        }
        //motor_left(MCPWM_UNIT_0, MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_TIMER_1, 80.0);
        //vTaskDelay(1000/portTICK_PERIOD_MS);
        if (front_distance <= 45)
        {
            motor_stop(MCPWM_UNIT_0, MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_TIMER_1);
            vTaskDelay(1000/portTICK_PERIOD_MS);
        }
    }
}

void app_main()
{
    //mcpwm_example_gpio_initialize();
    //lidar_initialize();
    printf("Testing LiDAR...\n");
    xTaskCreate(echo_task, "lidar_distance", 4096, NULL, 5, NULL);
    printf("Testing brushed motor...\n");
    xTaskCreate(motor_control, "motor_control", 4096, NULL, 5, NULL);
}