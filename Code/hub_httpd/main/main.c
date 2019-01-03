/* ESP HTTP Client Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "app_wifi.h"

#include "esp_http_client.h"

//Quest stuff
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/periph_ctrl.h"
#include "driver/rmt.h"
#include "soc/rmt_reg.h"

#define MAX_HTTP_RECV_BUFFER 512
static const char *TAG = "HTTP_CLIENT";

/* Root cert for howsmyssl.com, taken from howsmyssl_com_root_cert.pem

   The PEM file was extracted from the output of this command:
   openssl s_client -showcerts -connect www.howsmyssl.com:443 </dev/null

   The CA root cert is the last cert given in the chain of certs.

   To embed it in the app binary, the PEM file is named
   in the component.mk COMPONENT_EMBED_TXTFILES variable.
*/

// RMT definitions
#define RMT_TX_CHANNEL    1     // RMT channel for transmitter
#define RMT_TX_GPIO_NUM   25    // GPIO number for transmitter signal -- A1
#define RMT_CLK_DIV       100   // RMT counter clock divider
#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)   // RMT counter value for 10 us.(Source clock is APB clock)
#define rmt_item32_tIMEOUT_US   9500     // RMT receiver timeout value(us)

// UART definitions
#define UART_TX_GPIO_NUM 26 // A0
#define UART_RX_GPIO_NUM 34 // A2
#define BUF_SIZE (1024)

// Hardware interrupt definitions
#define GPIO_INPUT_IO_1       4
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_PIN_SEL    1ULL<<GPIO_INPUT_IO_1

// LED Output pins definitions
#define BLUEPIN   14
#define GREENPIN  32
#define REDPIN    15

// Default ID
#define ID 0

// Variables for my ID, minVal and status plus received ID, minVal, and status
char start = 0x0A; 
char myID = (char) ID;
char rxID = 0x00;
int len_out = 3; 
char passcode = 0x69;

//volatiles for interrupts 

volatile bool button_press = false;

// Mutex (for resources), and Queues (for button)
SemaphoreHandle_t mux = NULL;
//static xQueueHandle gpio_evt_queue = NULL;

// Button interrupt handler -- add to queue
static void IRAM_ATTR gpio_isr_handler(void* arg){
  button_press = true;
}

// RMT tx init
static void rmt_tx_init() {
    rmt_config_t rmt_tx;
    rmt_tx.channel = RMT_TX_CHANNEL;
    rmt_tx.gpio_num = RMT_TX_GPIO_NUM;
    rmt_tx.mem_block_num = 1;
    rmt_tx.clk_div = RMT_CLK_DIV;
    rmt_tx.tx_config.loop_en = false;
    rmt_tx.tx_config.carrier_duty_percent = 50;
    // Carrier Frequency of the IR receiver
    rmt_tx.tx_config.carrier_freq_hz = 38000;
    rmt_tx.tx_config.carrier_level = 1;
    rmt_tx.tx_config.carrier_en = 1;
    // Never idle -> aka ontinuous TX of 38kHz pulses
    rmt_tx.tx_config.idle_level = 1;
    rmt_tx.tx_config.idle_output_en = true;
    rmt_tx.rmt_mode = 0;
    rmt_config(&rmt_tx);
    rmt_driver_install(rmt_tx.channel, 0, 0);
}

// Configure UART
static void uart_init() {
  // Basic configs
  uart_config_t uart_config = {
      .baud_rate = 1200, // Slow BAUD rate
      .data_bits = UART_DATA_8_BITS,
      .parity    = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(UART_NUM_1, &uart_config);

  // Set UART pins using UART0 default pins
  uart_set_pin(UART_NUM_1, UART_TX_GPIO_NUM, UART_RX_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // Reverse receive logic line
  uart_set_line_inverse(UART_NUM_1,UART_INVERSE_RXD);

  // Install UART driver
  uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
}

// GPIO init for LEDs
static void led_init() {
    gpio_pad_select_gpio(BLUEPIN);
    gpio_pad_select_gpio(GREENPIN);
    gpio_pad_select_gpio(REDPIN);
    gpio_set_direction(BLUEPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(GREENPIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(REDPIN, GPIO_MODE_OUTPUT);
}

// Button interrupt init
static void hw_int_init() {
    gpio_config_t io_conf;
    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_intr_enable(GPIO_INPUT_IO_1 );
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
    //create a queue to handle gpio event from isr

    //gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
}

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (!esp_http_client_is_chunked_response(evt->client)) {
                // Write out data
                // printf("%.*s", evt->data_len, (char*)evt->data);
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
    }
    return ESP_OK;
}

static void http_rest(char *mystring)
{
    esp_http_client_config_t config = {
        .url = "http://192.168.1.128:8000/hub_post",
        .event_handler = _http_event_handler,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    /*
    We only want to keep the POST request, rest was fluff in the original client code.
    */
    // POST
    const char *post_data = mystring;
    esp_http_client_set_url(client, "http://192.168.1.128:8000/hub_post");
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_post_field(client, post_data, strlen(post_data));
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}

// Send task -- sends payload: start byte | code | ID

/*
The reason we have kept the send task, despite it being a fob-related function, is for debugging any functions
that require a fob communicating with the hub.
*/

void send_task(){
  while(1) {
    if (button_press) {
        char *data_out = (char *) malloc(len_out);
        xSemaphoreTake(mux, portMAX_DELAY);
        data_out[0] = start;
        data_out[1] = passcode;
        data_out[2] = myID;
        uart_write_bytes(UART_NUM_1, data_out, len_out+1);
        xSemaphoreGive(mux);
        //printf("Sent payload.\n");
        button_press = false;

        vTaskDelay(10 / portTICK_PERIOD_MS);
    } else {
        // increased delay to keep watchdog from complaining
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
  }
}

// Receives task -- looks for Start byte then stores received values
void recv_task(){
  // Buffer for input data
  uint8_t *data_in = (uint8_t *) malloc(BUF_SIZE);
  while (1) {
    int len_in = uart_read_bytes(UART_NUM_1, data_in, BUF_SIZE, 20 / portTICK_RATE_MS);
    if (len_in >0) {
      for (int i=0; i < 24; i++) {
        /*
        Payload received: start byte | code | ID
        */
        if (data_in[i] == start && data_in[i+1] == passcode) {

          rxID = data_in[i+2];
          printf("Received comm from device ID 0x%02X, passcode 0x%02X\n", rxID, data_in[i+1]);
           
          // put data into string, comma delimited for parsing on the JS side.
          char* buffer = malloc(100);
          sprintf(buffer, "%d,%d,%d", passcode, rxID, ID); // pass,rxID,ID
           
          // send out the post request back to the JS server. IP is hardcoded.
          http_rest(buffer);

          break;
        }
      }
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    else{
      //printf("Nothing received.\n");
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
  free(data_in);
}

static void http_test_task(void *pvParameters)
{
    app_wifi_wait_connected();
    ESP_LOGI(TAG, "Connected to AP, begin http example");
    http_rest("TEST. Successful request send.");
    ESP_LOGI(TAG, "Finish http example");
    vTaskDelete(NULL);
}

void app_main()
{
    // Mutex for current values when sending and during election
    mux = xSemaphoreCreateMutex();

    // Initialize transmitt and button interrupt
    rmt_tx_init();
    uart_init();
    led_init();
    gpio_set_level(REDPIN, 1); //RED led denotes this is the scary HAL 3000 wannabe hub
    hw_int_init();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    app_wifi_initialise();

    // Create tasks for receive, send, elect, set gpio, and button
    xTaskCreate(recv_task, "uart_rx_task", 1024*4, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(send_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL); // FOR DEBUGGING
    
    xTaskCreate(&http_test_task, "http_test_task", 8192, NULL, 5, NULL);
}
