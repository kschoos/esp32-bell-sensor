/* 
    Author: Kai Schoos
    Info:   Bell-Sensor Module for ESP-32.
            The Code samples audio data and checks for a certain frequency pattern in order to find out if the doorbell was rung.
            The bell pattern is hard-coded and only works for a single (my) doorbell.

    Basis: Hello World Example
*/

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
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>


#ifdef CONFIG_EXAMPLE_IPV4
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#else
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#endif

#define PORT CONFIG_EXAMPLE_PORT

// static const char *payload = "Message from ESP32 ";
static void tcp_client_task(void*);

static const char *TAG = "main";
TaskHandle_t th_analyzationTask;

#define N_SAMPLES 1024
#define FFT_SIZE 2048
#define SAMPLE_FREQ 16384
#define DIVIDER 4883
#define MAX_FREQ_CNT 7
#define MIN_FINAL_FREQ_CNT 2
#define MAX_FREQ_IDX 2
#define FREQ_THRESHOLD -35.
#define REC_LENGTH 2*SAMPLE_FREQ

int N = N_SAMPLES;
// Input test array
uint16_t data[N_SAMPLES];
uint16_t samples[N_SAMPLES];

char recording[2 * REC_LENGTH];

uint8_t secs = 0;
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


//--------------------------------------------------------------------------------------------------------
//  ISR Handlers
//--------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------
//  ISR Handler for reset button
//--------------------------------------------------------------------------------------------------------

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    secs = 0;
    current_freq_idx = 0;
    gpio_set_level(GPIO_OUTPUT_0, 0);
}

//--------------------------------------------------------------------------------------------------------
//  ISR Handler for ADC Sampling
//  -- Samples the adc on every call.
//  -- When N Samples are reached, copies the samples to the data array for inspection and notifies the 
//  -- analyzation task.
//--------------------------------------------------------------------------------------------------------

void IRAM_ATTR timer_group0_isr(void *para){
    timer_idx_t timer_idx = (timer_idx_t) para;

    uint32_t timer_intr = TIMERG0.int_st_timers.val;
    TIMERG0.hw_timer[timer_idx].update = 1;

    if ((timer_intr & BIT(timer_idx)) && timer_idx == TIMER_0) {
        if (adc_initialized){
            if (count == N){
                for(int i = 0; i < N; i++){
                    data[i] = samples[i];
                }
                count = 0;
                xTaskNotifyFromISR(th_analyzationTask, 0x00, eIncrement, NULL);
            } else {
                samples[count] = local_adc1_read(ADC1_CHANNEL_6);
                count += 1;
            }
        }

        TIMERG0.int_clr_timers.t0 = 1;
    }

    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;
}

//--------------------------------------------------------------------------------------------------------
//  Barebone implementation of ADC-Sampling
//  This is frankensteined from ESP-IDF code deep down: https://www.toptal.com/embedded/esp32-audio-sampling
//  The reasoning is that ADC-Sampling in ISRs is otherwise not possible / stable.
//--------------------------------------------------------------------------------------------------------

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

//--------------------------------------------------------------------------------------------------------
//  Initializations
//--------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------
//  GPIO Output init
//--------------------------------------------------------------------------------------------------------
static void init_gpio(uint8_t pin){
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1 << pin;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;

    gpio_config(&io_conf);
}

//--------------------------------------------------------------------------------------------------------
//  GPIO Input init
//--------------------------------------------------------------------------------------------------------
static void init_gpio_input_pullup(uint8_t pin){
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1 << pin;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;

    gpio_config(&io_conf);
}

//--------------------------------------------------------------------------------------------------------
//  Timer initialization for audio sampling
//  -- Compare mode
//--------------------------------------------------------------------------------------------------------
static void t0_0_init(){
    timer_group_t timer_group = 0;
    timer_idx_t timer_idx = 0;

    timer_config_t config;
    config.divider = DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = true;

    timer_init(timer_group, timer_idx, &config);
    timer_set_counter_value(timer_group, timer_idx, 0x00000000ULL);
    timer_set_alarm_value(timer_group, timer_idx, 1);
    timer_enable_intr(timer_group, timer_idx);
    timer_isr_register(timer_group, timer_idx, timer_group0_isr, 
        (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);
    
    timer_start(timer_group, timer_idx);
}

//--------------------------------------------------------------------------------------------------------
//  E-Fuse checking for the ADC (Do we have a good VRef?)
//--------------------------------------------------------------------------------------------------------

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

//--------------------------------------------------------------------------------------------------------
//  Initialization code for the fft module.
//--------------------------------------------------------------------------------------------------------

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


//--------------------------------------------------------------------------------------------------------
//  Find a certain frequency in the audio data and return the amplitude of that frequency bin.
//  This currently also stores the data in the appropriate position of the recording-buffer.
//--------------------------------------------------------------------------------------------------------
static float find_frequency(uint16_t freq){

    static const uint16_t dc_offset = 2978;
    static const float fsr = 4096.;

    // Convert two input vectors to one complex vector
    for (int i=0 ; i < N ; i++)
    {
        float val = (float) (data[i] - dc_offset) / fsr * wind[i];
        char lsB = data[i] & 0xFF;
        char msB = (data[i] >> 8) & 0xFF;
        int idx = 2 * (i + secs * N_SAMPLES + current_freq_idx * MAX_FREQ_CNT * N_SAMPLES);
        
        recording[idx] = data[i] & 0xFF;
        recording[idx + 1] = (data[i] & 0xFF00) >> 8;

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


//--------------------------------------------------------------------------------------------------------
//  The main analyzation task.
//  It first does some initialization.
//  The for-loop then is run every time the task is notified (=> The data buffer is full and ready to be analyzed)
//--------------------------------------------------------------------------------------------------------
static void analyzationTask(){
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

        // if ( ulNotifiedValue % 10 == 0 ){
        //     printf("%f\n", accu / 10);
        //     accu = 0;
        // }

        if ( secs < MAX_FREQ_CNT ){
            if ( find_frequency(freq) >= FREQ_THRESHOLD ){
                secs += 1;
            } else {
                secs = 0;
                current_freq_idx = 0;
            }
        } else {
             if ( current_freq_idx == MAX_FREQ_IDX){
                gpio_set_level(GPIO_OUTPUT_0, 1);
                secs = 0;
                current_freq_idx = 0;
                xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);
             }
             else if ( find_frequency(next_freq) >= FREQ_THRESHOLD ){
                 current_freq_idx += 1;
                 secs = 1;
             } else if ( find_frequency(freq) >= FREQ_THRESHOLD ){
                 secs += 1;
             } else {
                secs = 0;
                current_freq_idx = 0;
             }
        } 
    }
}

static void tcp_client_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    do{
    #ifdef CONFIG_EXAMPLE_IPV4
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
    #else // IPV6
        struct sockaddr_in6 dest_addr;
        inet6_aton(HOST_IP_ADDR, &dest_addr.sin6_addr);
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(dest_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
    #endif

        int sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", HOST_IP_ADDR, PORT);

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Successfully connected");

        err = send(sock, recording, 2 * REC_LENGTH, 0);
        if (err < 0) {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
            break;
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    } while(0);

    vTaskDelete(NULL);
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

    xTaskCreate(analyzationTask, "analyzationTask", 2048, NULL, 5, &th_analyzationTask);

    ESP_ERROR_CHECK(nvs_flash_init());
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());
}
