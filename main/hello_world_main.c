/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

// #define CONFIG_EXAMPLE_CONNECT_WIFI
// #define CONFIG_EXAMPLE_WIFI_SSID "Kleiner 3"
// #define CONFIG_EXAMPLE_WIFI_PASSWORD "m2.inetpw"

#include <stdio.h>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include <math.h>

#include "esp_dsp.h"
#include "driver/adc.h"

#include "soc/sens_reg.h"
#include "soc/sens_struct.h"

#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "esp_eth.h"
#include "protocol_examples_common.h"

#include <esp_http_server.h>

static const char *TAG = "main";
TaskHandle_t taskHandle;

#define N_SAMPLES 1024
#define FFT_SIZE 2048
#define SAMPLE_FREQ 16384
#define MAX_FREQ_CNT 7
#define MIN_FINAL_FREQ_CNT 2
#define MAX_FREQ_IDX 2
#define FREQ_THRESHOLD -35.

int N = N_SAMPLES;
// Input test array
uint16_t data[N_SAMPLES];
uint16_t samples[N_SAMPLES];

uint16_t recording[SAMPLE_FREQ*2];

uint8_t freq_cnt = 0;
uint8_t current_freq_idx = 0;

// Window coefficients
float wind[N_SAMPLES];
// working complex array
float y_cf[N_SAMPLES*2];
// Pointers to result arrays
float* y1_cf = &y_cf[0];


volatile uint16_t count = 0;
bool adc_initialized = false;

#include "esp_adc_cal.h"

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling


#define GPIO_OUTPUT_0 23
#define GPIO_INPUT_0 13


static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    freq_cnt = 0;
    current_freq_idx = 0;
    gpio_set_level(GPIO_OUTPUT_0, 0);
}

uint16_t IRAM_ATTR local_adc1_read(int channel) {
    uint16_t adc_value = 0;
    SENS.sar_meas_start1.sar1_en_pad = (1 << channel); // only one channel is selected
    while (SENS.sar_slave_addr1.meas_status != 0);
    SENS.sar_meas_start1.meas1_start_sar = 0;
    SENS.sar_meas_start1.meas1_start_sar = 1;
    while (SENS.sar_meas_start1.meas1_done_sar == 0);
    adc_value = SENS.sar_meas_start1.meas1_data_sar & 0xFFFC;
    return adc_value;
}


void IRAM_ATTR timer_group0_isr(void *para){
    timer_idx_t timer_idx = (timer_idx_t) para;
    // timer_group_t timer_group = 0;

    uint32_t timer_intr = TIMERG0.int_st_timers.val;
    TIMERG0.hw_timer[timer_idx].update = 1;

    // uint64_t timer_counter_value = ((uint64_t) TIMERG0.hw_timer[timer_idx].cnt_high) << 32
    //    | TIMERG0.hw_timer[timer_idx].cnt_low;

    if ((timer_intr & BIT(timer_idx)) && timer_idx == TIMER_0) {
        if (adc_initialized){
            if (count == N-1){
                for(int i = 0; i < N; i++){
                    data[i] = samples[i];
                }
                count = 0;
                xTaskNotifyFromISR(taskHandle, 0x00, eIncrement, NULL);
            } else {
                count += 1;
                samples[count] = local_adc1_read(ADC1_CHANNEL_6);
            }
        }

        TIMERG0.int_clr_timers.t0 = 1;
    }

    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;
}

static void init_gpio(uint8_t pin){
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1 << pin;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;

    gpio_config(&io_conf);
}

static void init_gpio_input_pullup(uint8_t pin){
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1 << pin;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;

    gpio_config(&io_conf);
}

static void t0_0_init(){
    timer_group_t timer_group = 0;
    timer_idx_t timer_idx = 0;

    timer_config_t config;
    config.divider = 4883;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = true;

    timer_init(timer_group, timer_idx, &config);
    timer_set_counter_value(timer_group, timer_idx, 0x00000000ULL);
    timer_set_alarm_value(timer_group, timer_idx, 1);
    // timer_enable_intr(timer_group, timer_idx, timer_group0_isr, (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);
    timer_enable_intr(timer_group, timer_idx);
    timer_isr_register(timer_group, timer_idx, timer_group0_isr, 
        (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);
    
    timer_start(timer_group, timer_idx);
}

static float find_frequency(uint16_t freq){

    static const uint16_t dc_offset = 2978;
    static const float fsr = 4096.;

    // Convert two input vectors to one complex vector
    for (int i=0 ; i < N ; i++)
    {
        float val = (float) (data[i] - dc_offset) / fsr * wind[i];
        recording[i + freq_cnt * N_SAMPLES + current_freq_idx * MAX_FREQ_CNT * N_SAMPLES] = data[i];
        y_cf[i*2] = val;
        y_cf[i*2 + 1] = y_cf[i*2];
    }

    dsps_fft2r_fc32(y_cf, N);
    dsps_bit_rev_fc32(y_cf, N);
    dsps_cplx2reC_fc32(y_cf, N);

    for (int i = 0 ; i < N/2 ; i++) {
        y1_cf[i] = 10 * log10f((y1_cf[i * 2 + 0] * y1_cf[i * 2 + 0] + y1_cf[i * 2 + 1] * y1_cf[i * 2 + 1])/N);
    }

    uint16_t freq_res = SAMPLE_FREQ / N;
    uint16_t bin = freq / freq_res;

    return y1_cf[bin];
}

static void check_efuse()
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void init_dsp(){
    esp_err_t ret;

    ret = dsps_fft2r_init_fc32(NULL, FFT_SIZE);
    if (ret  != ESP_OK)
    {
        ESP_LOGE(TAG, "Not possible to initialize FFT. Error = %i", ret);
        return;
    }
    dsps_wind_hann_f32(wind, N);
}

static void task(){
    uint32_t ulNotifiedValue = 0;

    uint16_t freqs[] = {918, 758, 605};

    check_efuse();

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_6);

    //Characterize ADC
    esp_adc_cal_characteristics_t  adc_chars;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_12, DEFAULT_VREF, &adc_chars);
    
    uint16_t adc_val = adc1_get_raw(ADC1_CHANNEL_6);
    printf("Initial value: %d\n", adc_val);

    init_dsp();

    adc_initialized = true;
    float accu = 0;

    t0_0_init();

    for ( ;; ){
        xTaskNotifyWait(0x00,
                        0x00,
                        &ulNotifiedValue,
                        portMAX_DELAY);

        uint16_t freq = freqs[current_freq_idx];
        uint16_t next_freq = freqs[current_freq_idx + 1];

        accu += find_frequency(500);

        if ( ulNotifiedValue % 10 == 0 ){
            printf("%f\n", accu / 10);
            accu = 0;
        }

        if ( current_freq_idx == MAX_FREQ_IDX && freq_cnt >= MIN_FINAL_FREQ_CNT ) {
            gpio_set_level(GPIO_OUTPUT_0, 1);
            freq_cnt = 0;
            current_freq_idx = 0;

            for (int k = 0; k < SAMPLE_FREQ * 2; k++){
                printf("%d, ", recording[k]);
            }
        }

        if ( freq_cnt < MAX_FREQ_CNT ){
            if ( find_frequency(freq) >= FREQ_THRESHOLD ){
                freq_cnt += 1;
            } else {
                freq_cnt = 0;
                current_freq_idx = 0;
            }
        } else {
             if ( find_frequency(next_freq) >= FREQ_THRESHOLD ){
                 current_freq_idx += 1;
                 freq_cnt = 1;
             } else if ( find_frequency(freq) >= FREQ_THRESHOLD ){
                 freq_cnt += 1;
             } else {
                freq_cnt = 0;
                current_freq_idx = 0;
             }
        } 
    }
}

void app_main()
{
    init_gpio(GPIO_OUTPUT_0);
    init_gpio_input_pullup(GPIO_INPUT_0);

    //change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_0, GPIO_INTR_ANYEDGE);
    //install gpio isr service
    gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_0, gpio_isr_handler, (void*) GPIO_INPUT_0);

    gpio_set_level(GPIO_OUTPUT_0, 1);

    xTaskCreate(task, "task", 2048, NULL, 5, &taskHandle);
}
