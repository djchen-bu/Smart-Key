/* RMT transmit example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
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

static const char *RMT_TX_TAG = "RMT Tx";

#define RMT_TX_CHANNEL 1
#define RMT_TX_GPIO 17

#define RMT_RX_CHANNEL 0
#define RMT_RX_GPIO_NUM 16
#define rmt_item32_tIMEOUT_US  9500   /*!< RMT receiver timeout value(us) */
#define RMT_CLK_DIV 100
#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)   /*!< RMT counter value for 10 us.(Source clock is APB clock) */

#define SAMPLE_CNT  (10)

#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL_SEC   (1.0) // sample test interval for the first timer
#define TEST_WITHOUT_RELOAD   0
#define TEST_WITH_RELOAD 1

#define speed_of_sound_cm 34300.0

/*
 * Prepare a raw table with a message in the Morse code
 *
 * The message is "ESP" : . ... .--.
 *
 * The table structure represents the RMT item structure:
 * {duration, level, duration, level}
 *
 */
rmt_item32_t items[] = {
    // E : dot
    {{{ 32767, 1, 32767, 0 }}}, // dot
    //
    {{{ 32767, 0, 32767, 0 }}}, // SPACE
    // S : dot, dot, dot
    {{{ 32767, 1, 32767, 0 }}}, // dot
    {{{ 32767, 1, 32767, 0 }}}, // dot
    {{{ 32767, 1, 32767, 0 }}}, // dot
    //
    {{{ 32767, 0, 32767, 0 }}}, // SPACE
    // P : dot, dash, dash, dot
    {{{ 32767, 1, 32767, 0 }}}, // dot
    {{{ 32767, 1, 32767, 1 }}},
    {{{ 32767, 1, 32767, 0 }}}, // dash
    {{{ 32767, 1, 32767, 1 }}},
    {{{ 32767, 1, 32767, 0 }}}, // dash
    {{{ 32767, 1, 32767, 0 }}}, // dot

    // RMT end marker
    {{{ 0, 1, 0, 0 }}}
};

rmt_item32_t trig[1] = {
{{{ 10, 1 , 0 , 0 }}},
};

//Convert uint8_t type of data to rmt format data.
static void IRAM_ATTR u8_to_rmt(const void* src, rmt_item32_t* dest, size_t src_size, 
                         size_t wanted_num, size_t* translated_size, size_t* item_num)
{
    if(src == NULL || dest == NULL) {
        *translated_size = 0;
        *item_num = 0;
        return;
    }
    const rmt_item32_t bit0 = {{{ 32767, 1, 15000, 0 }}}; //Logical 0
    const rmt_item32_t bit1 = {{{ 32767, 1, 32767, 0 }}}; //Logical 1
    size_t size = 0;
    size_t num = 0;
    uint8_t *psrc = (uint8_t *)src;
    rmt_item32_t* pdest = dest;
    while (size < src_size && num < wanted_num) {
        for(int i = 0; i < 8; i++) {
            if(*psrc & (0x1 << i)) {
                pdest->val =  bit1.val; 
            } else {
                pdest->val =  bit0.val;
            }
            num++;
            pdest++;
        }
        size++;
        psrc++;
    }
    *translated_size = size;
    *item_num = num;
}

/*
 * Initialize the RMT Tx channel
 */
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

static void timer_tg0_init(int timer_idx, 
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


static void echo_task()
{
    double time, distance;
    while (1) {
        if (!gpio_get_level(21)){

            continue; // dont do anything unless the level changes.

        } else {
            timer_start(TIMER_GROUP_0, TIMER_0); // start timer
            while (gpio_get_level(21)) {
                ; // wait for the level to go down again.
            }
            timer_pause(TIMER_GROUP_0, TIMER_0); // stop timer
            timer_get_counter_time_sec(TIMER_GROUP_0, TIMER_0, &time);
            timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL); // reset counter value.
            // t = 2d / v;
            distance = (time * speed_of_sound_cm)/ 2;
            printf("Echo time: %fs, Distance: %fcm\n", time, distance);
            vTaskDelay(10/portTICK_PERIOD_MS);
        }
    }
    vTaskDelete(NULL);
}


static void rmt_example_nec_tx_task()
{
    ESP_LOGI(RMT_TX_TAG, "Configuring transmitter");
    // int number_of_items = sizeof(items) / sizeof(items[0]);
    // const uint8_t sample[SAMPLE_CNT] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    int number_of_items = sizeof(trig) / sizeof(trig[0]);

    while (1) {

        //ESP_ERROR_CHECK(rmt_write_items(RMT_TX_CHANNEL, items, number_of_items, true));
        rmt_write_items(RMT_TX_CHANNEL, trig, number_of_items, true);
        //ESP_LOGI(RMT_TX_TAG, "Trigger signal transmitted.");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
    }
    vTaskDelete(NULL);
}

static void us_get_distance_cm(int timer_idx, int echo_pin)
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
            printf("Echo time: %fs, Distance: %fcm\n", time, distance);
            return;

        }
    }
}

static void sensor_task(){
    while(1) {
        us_get_distance_cm(TIMER_0, 21);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void app_main()
{
    rmt_tx_init();
    gpio_pad_select_gpio(21);
    gpio_set_direction(21, GPIO_MODE_INPUT);
    timer_tg0_init(TIMER_0, TEST_WITH_RELOAD, TIMER_INTERVAL_SEC);

    //xTaskCreate(rmt_example_nec_tx_task, "rmt_nec_tx_task", 2048, NULL, 10, NULL);
    //xTaskCreate(echo_task, "echo_task", 2048, NULL, 10, NULL);
    xTaskCreate(sensor_task, "sensor_task", 2048, NULL, 10, NULL);
}
