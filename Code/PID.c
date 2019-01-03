/* GPIO Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

/**
 * Brief:
 * This test code shows how to configure gpio and how to use gpio interrupt.
 *
 * GPIO status:
 * GPIO18: output
 * GPIO19: output
 * GPIO4:  input, pulled up, interrupt from rising edge and falling edge
 * GPIO5:  input, pulled up, interrupt from rising edge.
 *
 * Test:
 * Connect GPIO18 with GPIO4
 * Connect GPIO19 with GPIO5
 * Generate pulses on GPIO18/19, that triggers interrupt on GPIO4/5
 *
 */

#define GPIO_OUTPUT_IO_0    33
#define GPIO_OUTPUT_IO_1    15
#define GPIO_OUTPUT_IO_2    32
#define GPIO_OUTPUT_IO_3    14
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1) | (1ULL<<GPIO_OUTPUT_IO_2) | (1ULL<<GPIO_OUTPUT_IO_3))
#define GPIO_INPUT_IO_0     27
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0))
#define ESP_INTR_FLAG_DEFAULT 0

// v Timer interrupt parameters v
#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL_SEC   (0.1) // sample test interval for the first timer
#define TEST_WITHOUT_RELOAD   0
#define TEST_WITH_RELOAD 1

#define speed_of_sound_cm 34300.0

// v PID parameters v
#define dt 0.1 //delta t = 100ms
#define baseline 20 // 20cm
#define Kp 1.0
#define Ki 0.0
#define Kd 0.0

// v UART defines v
#define ECHO_TEST_TXD   17
#define ECHO_TEST_RXD   16
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)

#define BUF_SIZE (1024)

// v RMT stuff v
#define RMT_TX_CHANNEL 1
#define RMT_TX_GPIO 17

#define RMT_RX_CHANNEL 0
#define RMT_RX_GPIO_NUM 16
#define rmt_item32_tIMEOUT_US  9500   /*!< RMT receiver timeout value(us) */
#define RMT_CLK_DIV 100
#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)   /*!< RMT counter value for 10 us.(Source clock is APB clock) */

#define SAMPLE_CNT  (10)

rmt_item32_t trig[1] = {
{{{ 10, 1 , 0 , 0 }}},
};


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

static double us_get_distance_cm(int timer_idx, int echo_pin)
{
    int number_of_items = sizeof(trig) / sizeof(trig[0]);
    rmt_write_items(RMT_TX_CHANNEL, trig, number_of_items, true); // send trigger signal
    double time, distance;
    //int timeout = 0;

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

// v Interrupt flag v
volatile bool trigger = false;

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

    // ADD TO SECONDS.
    trigger = true;
}

static void timer_tg0_init(int timer_idx, 
    bool auto_reload, double timer_interval_sec, bool interrupt)
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
    if (interrupt) {
    	/* Configure the alarm value and the interrupt on alarm. */
	    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
	    timer_enable_intr(TIMER_GROUP_0, timer_idx);
	    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr, 
	        (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);
    }
}



void app_main()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as input mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //enable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    /*
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
    */

    gpio_set_level(GPIO_OUTPUT_IO_0, 0);
	gpio_set_level(GPIO_OUTPUT_IO_1, 0);
	gpio_set_level(GPIO_OUTPUT_IO_2, 0);
	gpio_set_level(GPIO_OUTPUT_IO_3, 0);

	uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
	//uint16_t value;

	double integral, derivative, error, prev_error, output;
	int i=0;
	prev_error = 0.0;
	integral = 0.0;

	rmt_tx_init();

	timer_tg0_init(TIMER_0, TEST_WITH_RELOAD, TIMER_INTERVAL_SEC, 1); //PID timer + interrupt
	timer_start(TIMER_GROUP_0, TIMER_0);
	
	timer_tg0_init(TIMER_1, TEST_WITH_RELOAD, TIMER_INTERVAL_SEC, 0); //ultrasonic timer
	//timer_start(TIMER_GROUP_0, TIMER_1);
	while (1) {
		// get value from LIDAR

		/*
		int length =0; 
        uart_get_buffered_data_len(UART_NUM_1,(size_t*)&length);
        if (length == 0)
        {
            continue; 
        }
        // Read data from the UART
        int len = uart_read_bytes(UART_NUM_1,data,length,100);
        
        //finding the header one and header two. Then read the MSB and LSB of the distance
        if((data[i] == 89) && (data[i+1] == 89)) 
        {
            value = (data[i+3]<<8)+data[i+2]; // cm
        }
        */
		if (trigger) {
			double value = us_get_distance_cm(TIMER_1, 21);
			error = baseline - value;
			//dt = dt + 1;
		    integral = integral + error * dt;
		    derivative = (error - prev_error) / dt;
		    output = Kp * error + Ki * integral + Kd * derivative;
			prev_error = error;
			trigger = false;

	        printf("Error: %fcm\n", error);
			if ((int)output == 0) {
				gpio_set_level(GPIO_OUTPUT_IO_0, 0); // R
				gpio_set_level(GPIO_OUTPUT_IO_1, 1); // G
				gpio_set_level(GPIO_OUTPUT_IO_2, 0); // B
			} else if ((int)output < 0) {
				gpio_set_level(GPIO_OUTPUT_IO_0, 1); // R
				gpio_set_level(GPIO_OUTPUT_IO_1, 0); // G
				gpio_set_level(GPIO_OUTPUT_IO_2, 0); // B
		        writeDisplay(displaybuffer);
			} else if ((int)output > 0) {
				gpio_set_level(GPIO_OUTPUT_IO_0, 0); // R
				gpio_set_level(GPIO_OUTPUT_IO_1, 0); // G
				gpio_set_level(GPIO_OUTPUT_IO_2, 1); // B
			}
		} else {
			vTaskDelay(100/portTICK_PERIOD_MS);
		}
	}
}
