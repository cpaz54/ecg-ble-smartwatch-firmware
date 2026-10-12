#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h> 
#include <zephyr/logging/log.h>  // needs CONFIG_LOG=y in your prj.conf
#include <zephyr/smf.h> // CONFIG_SMF=y
#include <zephyr/drivers/adc.h> // CONFIG_ADC=y
#include <zephyr/drivers/pwm.h> // CONFIG_PWM=y
#include <zephyr/drivers/sensor.h> //CONFIG_SENSOR=y
#include <zephyr/sys/util.h>    


#include "ble-lib.h" // Bluetooth library

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

//MACRO DEFINITIONS ...........................................

#define HEARTBEAT_ON_MS 500 //ms
#define HEARTBEAT_OFF_MS 500 //ms
#define ERROR_LED_ON_MS 500 //ms
#define ERROR_LED_OFF_MS 500 //ms

#define ADC_MAX_MV_MAP 3000 //Battery max is 3V 
#define BATTERY_CHECK_INTERVAL_MS 60000 //60 seconds
#define BUTTON_WAIT_MS 500 //ms debounce wait

//differential sampling parameters
#define DIFF_FS_HZ 150
#define DIFF_INTERVAL_US (1000000U / DIFF_FS_HZ) //interval in microseconds
#define DIFF_CYCLES_MIN 20
#define DIFF_SAMPLES_PER_CYCLE (DIFF_FS_HZ / 10)
#define DIFF_SAMPLE_CYCLES DIFF_CYCLES_MIN
#define DIFF_BUFFER_LEN (DIFF_SAMPLES_PER_CYCLE * DIFF_SAMPLE_CYCLES)
#define SAMPLE_BUFF_MARGIN 1000

//HR Parameters
#define HR_MIN_BPM 40
#define HR_MAX_BPM 200
#define ECG_QRS_THRESHOLD_MV 450 //absolute mv threshold for qrs detection
#define ECG_REFRACTORY_MS (60000U / HR_MAX_BPM) //minimum ms between heart beats
#define HR_UPDATE_CYCLES 3 //updates HR every 3 cycles (2 seconds)

//kernel event bit arrays for buttons
#define BUTTON0_EVENT BIT(0)
#define BUTTON1_EVENT BIT(1)
#define BUTTON2_EVENT BIT(2)
#define BUTTON3_EVENT BIT(3)
#define DIFF_DONE_EVENT BIT(4)
#define BATTERY_CHECK_EVENT BIT(5)
#define ERROR_EVENT BIT(6)

//Meaningful Error Codes
#define ERROR_ADC_INIT BIT(0)
#define ERROR_ADC_READ BIT(1)
#define ERROR_PWM_INIT BIT(2)
#define ERROR_TEMP_INIT BIT(3)
#define ERROR_TEMP_READ BIT(4)
#define ERROR_ECG_READ BIT(5)
#define ERROR_GPIO_INIT BIT(6)


#define ADC_DT_SPEC_GET_BY_ALIAS(adc_alias)                    \
{                                                            \
    .dev = DEVICE_DT_GET(DT_PARENT(DT_ALIAS(adc_alias))),      \
    .channel_id = DT_REG_ADDR(DT_ALIAS(adc_alias)),            \
    ADC_CHANNEL_CFG_FROM_DT_NODE(DT_ALIAS(adc_alias))          \
}                                                            \

//..................................................................

//HARDWARE CONFIGURATIONS...........................................

// GPIOS
static const struct gpio_dt_spec heartbeat_led = GPIO_DT_SPEC_GET(DT_ALIAS(heartbeat), gpios); //LED1 on board
static const struct gpio_dt_spec error_led = GPIO_DT_SPEC_GET(DT_ALIAS(error), gpios); //LED4 on board. All other lights will also be on in ERROR
static const struct gpio_dt_spec ecg_led = GPIO_DT_SPEC_GET(DT_ALIAS(ecgled), gpios); //LED3 on board

//PWM
static const struct pwm_dt_spec battery_led = PWM_DT_SPEC_GET(DT_ALIAS(batteryled)); //LED2 on board

//buttons
static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(DT_ALIAS(idlereturn), gpios); //button0
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(DT_ALIAS(tempread), gpios); //button1
static const struct gpio_dt_spec button2 = GPIO_DT_SPEC_GET(DT_ALIAS(ecgread), gpios); //button2
static const struct gpio_dt_spec button3 = GPIO_DT_SPEC_GET(DT_ALIAS(reset), gpios); //button3
//ADC Channels
static const struct adc_dt_spec adc_vadc = ADC_DT_SPEC_GET_BY_ALIAS(vadc); //P0.02
static const struct adc_dt_spec adc_vadcdiff = ADC_DT_SPEC_GET_BY_ALIAS(vadcdiff); //P0.03 and P0.04 differential

//Temperature Sensor
const struct device *const temp_sensor = DEVICE_DT_GET_ONE(jedec_jc_42_4_temp); //MCP9808 temperature sensor


//..................................................................

//Struct for general varaibles

struct app_ctx {
    // ADC 
    int16_t adc_sample_buf;
    struct adc_sequence adc_sequence;

    // differential ADC buffer 
    int16_t diff_samples[DIFF_BUFFER_LEN];
    struct adc_sequence_options diff_options;

    // Buttons/Callbacks stored inside context so callbacks can find ctx via CONTAINER_OF 
    struct gpio_callback button0_cb;
    struct gpio_callback button1_cb;
    struct gpio_callback button2_cb;
    struct gpio_callback button3_cb;

    // Event & errors (previously K_EVENT_DEFINE macros) 
    struct k_event button_events;
    struct k_event errors;

    // runtime flags and counters 
    bool button0_enabled;
    bool button1_enabled;
    bool button2_enabled;
    bool button3_enabled;
    volatile uint32_t pending_button_event;
    uint16_t previous_state; //for purposes of returning from TEMP

    bool heartbeat_allowed;
    int heartbeat_state;

    int32_t last_battery_mv;
    float last_temp_c;
    int32_t temperature_degC; // Bluetooth-exposed temperature variable
    volatile uint32_t last_heart_rate_bpm;
    uint32_t hr_sum_bpm;      // sum of instantaneous BPM values found 
    uint32_t hr_count;        // number of instantaneous BPMs accumulated since last report
    volatile uint32_t error_flags;
    int64_t last_battery_check;

    volatile bool ecg_active;
    volatile uint32_t diff_samples_processed;
    volatile int32_t diff_peak_abs_mv;
};

// single static instance of app context 
static struct app_ctx app;

//................................................................

//FORWARD DECLARATIONS............................................
void heartbeat_thread(void *p1, void *p2, void *p3);
void hr_blink_thread(void *p1, void *p2, void *p3);

static int button0_set_enabled(struct app_ctx *ctx, bool enabled);
static int button1_set_enabled(struct app_ctx *ctx, bool enabled);
static int button2_set_enabled(struct app_ctx *ctx, bool enabled);
static int button3_set_enabled(struct app_ctx *ctx, bool enabled);

static void button0_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static void button1_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static void button2_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static void button3_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

//ADC helpers
static int adc_setup_and_check(struct app_ctx *ctx);
static int adc_measure_mv(struct app_ctx *ctx, int32_t *val_mv);
static enum adc_action diff_adc_sequence_cb(const struct device *dev, const struct adc_sequence *sequence, uint16_t sampling_index);
static int diff_prepare_sequence(struct app_ctx *ctx, struct adc_sequence *seq);
static int diff_start_async_read(struct app_ctx *ctx, struct adc_sequence *seq);
static int diff_wait_for_completion(struct app_ctx *ctx, uint32_t timeout_ms);
static int diff_compute_peak_from_buffer(struct app_ctx *ctx);
static void diff_cleanup_after_read(struct app_ctx *ctx);
static int diff_adc_buffered_read_and_process(struct app_ctx *ctx);

//Utility helpers
static uint32_t calculate_heart_rate_from_samples(struct app_ctx *ctx);
static int read_temperature_sensor(struct app_ctx *ctx, float *temperature_degC);
static int set_battery_led_brightness_from_mv(struct app_ctx *ctx, int32_t mv);

//ecg state helpers
static int ecg_get_event_action(struct app_ctx *ctx, uint32_t ev);
static void ecg_handle_event_side_effects(struct app_ctx *ctx, uint32_t ev);

//SMF state functions
static void init_entry(void *obj);
static void init_run(void *obj);
static void init_exit(void *obj);

static void idle_entry(void *obj);
static void idle_run(void *obj);
static void idle_exit(void *obj);

static void ecg_entry(void *obj);
static void ecg_run(void *obj);
static void ecg_exit(void *obj);    

static void temp_entry(void *obj);
static void temp_run(void *obj);    
static void temp_exit(void *obj);

static void error_entry(void *obj);
static void error_run(void *obj);
static void error_exit(void *obj);
//................................................................

//THREADs...............................................

K_THREAD_DEFINE(heartbeat_thread_id, 1024, heartbeat_thread, &app, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(hr_blink_thread_id, 1024, hr_blink_thread, &app, NULL, NULL, 5, 0, 0); 


void heartbeat_thread(void *p1, void *p2, void *p3)
{
    struct app_ctx *ctx = (struct app_ctx *)p1;
    while(!ctx->heartbeat_allowed) {
        k_msleep(10);
    }

    gpio_pin_set_dt(&heartbeat_led, 1); //set initial state
    ctx->heartbeat_state = 1;
    while (1) {
        gpio_pin_set_dt(&heartbeat_led, 0); 
        ctx->heartbeat_state = !ctx->heartbeat_state;
        k_msleep(HEARTBEAT_ON_MS);
        gpio_pin_set_dt(&heartbeat_led, 1); 
        ctx->heartbeat_state = !ctx->heartbeat_state;
        k_msleep(HEARTBEAT_OFF_MS);
    }
}
void hr_blink_thread(void *p1, void *p2, void *p3)
{
    struct app_ctx *ctx = (struct app_ctx *)p1;

    // Wait until board init enables GPIOs (heartbeat_allowed set in init_entry)
    while (!ctx->heartbeat_allowed) {
        k_msleep(10);
    }

    // Ensure LED is OFF initially 
    if (device_is_ready(ecg_led.port)) {
        (void)gpio_pin_set_dt(&ecg_led, 0);
    }

    for (;;) {
        if (ctx->ecg_active && ctx->last_heart_rate_bpm > 0) {
            uint32_t period_ms = 60000U / ctx->last_heart_rate_bpm;
            uint32_t on_time_ms = period_ms / 4U;   // 25% ON 
            if (on_time_ms == 0) on_time_ms = 1; 
            uint32_t off_time_ms = period_ms - on_time_ms; //75% off

            if (device_is_ready(ecg_led.port)) {
                // Turn LED ON 
                (void)gpio_pin_set_dt(&ecg_led, 1);
                k_msleep(on_time_ms);

                // Turn LED OFF
                (void)gpio_pin_set_dt(&ecg_led, 0);
                k_msleep(off_time_ms);
            } else {
                // If GPIO unexpectedly not ready, back off a bit
                k_msleep(200);
            }
        } else {
            // Make sure LED is OFF while not in ECG and poll periodically 
            if (device_is_ready(ecg_led.port)) {
                (void)gpio_pin_set_dt(&ecg_led, 0); //off
            }
            k_msleep(100);
        }
    }
}




//BUTTONS........................................................
static int button0_set_enabled(struct app_ctx *ctx, bool enabled) {
    int cfg = enabled ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE;
    if (!device_is_ready(button0.port)) {
        LOG_ERR("button0 GPIO not ready!");
        return -ENODEV;
    }
    int rc = gpio_pin_interrupt_configure_dt(&button0, cfg);

    if(rc < 0) {
        LOG_ERR("gpio_pin_interrupt_configure_dt failed for button0: %d", rc);
        return rc;
    } else {
        ctx->button0_enabled = enabled;
    }
    return 0;
}

static int button1_set_enabled(struct app_ctx *ctx, bool enabled) {
    int cfg = enabled ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE;
    if (!device_is_ready(button1.port)) {
        LOG_ERR("button1 GPIO not ready!");
        return -ENODEV;
    }
    int rc = gpio_pin_interrupt_configure_dt(&button1, cfg);
    if (rc < 0) {
        LOG_ERR("gpio_pin_interrupt_configure_dt failed for button1: %d", rc);
    } else {
        ctx->button1_enabled = enabled;
    }  
    return 0;
}

static int button2_set_enabled(struct app_ctx *ctx, bool enabled) {
    int cfg = enabled ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE;
    
    if (!device_is_ready(button2.port)) {
        LOG_ERR("button2 GPIO not ready!");
        return -ENODEV;
    }
    int rc = gpio_pin_interrupt_configure_dt(&button2, cfg);

    if(rc < 0) {
        LOG_ERR("gpio_pin_interrupt_configure_dt failed for button2: %d", rc);
        return rc;
    } else {
        ctx->button2_enabled = enabled;
    }
    return 0;
}

static int button3_set_enabled(struct app_ctx *ctx, bool enabled) {
    int cfg = enabled ? GPIO_INT_EDGE_TO_ACTIVE : GPIO_INT_DISABLE;
    if (!device_is_ready(button3.port)) {
        LOG_ERR("button3 GPIO not ready!");
        return -ENODEV;
    }
    int rc = gpio_pin_interrupt_configure_dt(&button3, cfg);
    if (rc < 0) {
        LOG_ERR("gpio_pin_interrupt_configure_dt failed for button3: %d", rc);
    } else {
        ctx->button3_enabled = enabled;
    }  
    return 0;
}

//button ISR

static void button0_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    ARG_UNUSED(dev);
    ARG_UNUSED(pins);

    struct app_ctx *ctx = CONTAINER_OF(cb, struct app_ctx, button0_cb);
    k_event_post(&ctx->button_events, BUTTON0_EVENT); // post event in SMF thread to handle button presses
}

static void button1_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    ARG_UNUSED(dev);
    ARG_UNUSED(pins);
    struct app_ctx *ctx = CONTAINER_OF(cb, struct app_ctx, button1_cb);
    k_event_post(&ctx->button_events, BUTTON1_EVENT); // post event in SMF thread to handle button presses
}

static void button2_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    ARG_UNUSED(dev);
    ARG_UNUSED(pins);
    struct app_ctx *ctx = CONTAINER_OF(cb, struct app_ctx, button2_cb);
    k_event_post(&ctx->button_events, BUTTON2_EVENT); // post event in SMF thread to handle button presses
}

static void button3_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    ARG_UNUSED(dev);
    ARG_UNUSED(pins);
    struct app_ctx *ctx = CONTAINER_OF(cb, struct app_ctx, button3_cb);
    k_event_post(&ctx->button_events, BUTTON3_EVENT); // post event in SMF thread to handle button presses
}

//..............................................

//ADC HELPER FUNCTIONS..........................

static int adc_setup_and_check(struct app_ctx *ctx) {
    if(!device_is_ready(adc_vadc.dev)) {
        LOG_ERR("ADC device not ready!");
        return -ENODEV;
    }
    int err = adc_channel_setup_dt(&adc_vadc);
    if (err < 0) {
        LOG_ERR("ADC channel setup failed with error: %d", err);
        return err;
    }
    if(!device_is_ready(adc_vadcdiff.dev)) {
        LOG_ERR("Differential ADC device not ready!");
        return -ENODEV;
    }
    int rc = adc_channel_setup_dt(&adc_vadcdiff);
    if (rc < 0) {
        LOG_ERR("Differential ADC channel setup failed with error: %d", rc);
        return rc;
    }
    return 0;
}

//asynchronous ADC measurement in mV
static int adc_measure_mv(struct app_ctx *ctx, int32_t *val_mv) {
    if (!val_mv)
        return -EINVAL;
    
    adc_sequence_init_dt(&adc_vadc, &ctx->adc_sequence);
    ctx->adc_sequence.buffer = &ctx->adc_sample_buf;
    ctx->adc_sequence.buffer_size = sizeof(ctx->adc_sample_buf);

    int rc = adc_read(adc_vadc.dev, &ctx->adc_sequence);
    if (rc < 0) {
        LOG_ERR("ADC read failed with error: %d", rc);
        return rc;
    }

    //convert to mV
    int32_t mv = (int32_t)ctx->adc_sample_buf;
    rc = adc_raw_to_millivolts_dt(&adc_vadc, &mv);
    if (rc < 0) {
        LOG_ERR("ADC raw to mV conversion failed with error: %d", rc);
        return rc;
    }
    *val_mv = mv;
    return 0;
}

//differential async ADC callback. Must use global app instance because zephyr ADC API doesnt support context passing
static struct app_ctx *g_app_ctx_for_isr = NULL;
static enum adc_action diff_adc_sequence_cb(const struct device *dev, const struct adc_sequence *sequence, uint16_t sampling_index) {

    if (sampling_index >= DIFF_BUFFER_LEN) {
        LOG_ERR("Callback: invalid sampling_index %u (max %u)", sampling_index, DIFF_BUFFER_LEN - 1);
        gpio_pin_set_dt(&ecg_led, 0);
        k_event_post(&g_app_ctx_for_isr->button_events, DIFF_DONE_EVENT);
        return ADC_ACTION_FINISH;
    }
    
     g_app_ctx_for_isr->diff_samples_processed = sampling_index + 1;

    // Finish when last index reached. Post completion from ISR 
    if (sampling_index == (DIFF_BUFFER_LEN - 1)) {
        k_event_post(&g_app_ctx_for_isr->button_events, DIFF_DONE_EVENT);
        return ADC_ACTION_FINISH;
    }

    return ADC_ACTION_CONTINUE;
}

static int diff_prepare_sequence(struct app_ctx *ctx, struct adc_sequence *seq) {
    if (!seq) return -EINVAL;

    // Clear any previous DIFF_DONE_EVENT that could interfere. 
    k_event_wait(&ctx->button_events, DIFF_DONE_EVENT, true, K_NO_WAIT);

    ctx->diff_options.extra_samplings = DIFF_BUFFER_LEN - 1;
    ctx->diff_options.interval_us = DIFF_INTERVAL_US;
    ctx->diff_options.callback = diff_adc_sequence_cb;

    (void)adc_sequence_init_dt(&adc_vadcdiff, seq);

    seq->options = &ctx->diff_options;
    seq->buffer = ctx->diff_samples;
    seq->buffer_size = sizeof(ctx->diff_samples);

    // reset runtime state 
    ctx->diff_samples_processed = 0;
    ctx->diff_peak_abs_mv = 1;

    return 0;
}

static int diff_start_async_read(struct app_ctx *ctx, struct adc_sequence *seq) {
    int rc = adc_read_async(adc_vadcdiff.dev, seq, NULL);
    if (rc < 0) {
        LOG_ERR("adc_read_async() failed with error: %d", rc);
        button1_set_enabled(ctx, true);
        return rc;
    }
    return 0;
}

static int diff_wait_for_completion(struct app_ctx *ctx, uint32_t timeout_ms) {
    uint32_t wait_events = DIFF_DONE_EVENT | BUTTON0_EVENT | BUTTON1_EVENT | BUTTON2_EVENT | BUTTON3_EVENT | ERROR_EVENT;
    uint32_t ev = k_event_wait(&ctx->button_events, wait_events, false, K_MSEC(timeout_ms));

    // If user pressed a button or ERROR occurred while sampling, record and exit
    if (ev & (BUTTON0_EVENT | BUTTON1_EVENT | BUTTON2_EVENT | BUTTON3_EVENT | ERROR_EVENT)) {
        LOG_INF("ADC read interrupted by button/error event: 0x%08X", ev);
        gpio_pin_set_dt(&ecg_led, 0); 
        button1_set_enabled(ctx, true);

        if (ev & DIFF_DONE_EVENT) {
            k_event_wait(&ctx->button_events, DIFF_DONE_EVENT, true, K_NO_WAIT);
        }

        // store which event interrupted us so caller can handle 
        ctx->pending_button_event = ev & (BUTTON0_EVENT | BUTTON1_EVENT | BUTTON2_EVENT | BUTTON3_EVENT | ERROR_EVENT);
        return -EINTR;
    }

    if (!(ev & DIFF_DONE_EVENT)) {
        LOG_ERR("Differential ADC read timeout! Expected %u ms, processed %u/%u samples", timeout_ms, ctx->diff_samples_processed, DIFF_BUFFER_LEN);
        gpio_pin_set_dt(&ecg_led, 0);
        button1_set_enabled(ctx, true);
        return -ETIMEDOUT;
    }
    k_event_wait(&ctx->button_events, DIFF_DONE_EVENT, true, K_NO_WAIT);
    return 0;
}

static int diff_compute_peak_from_buffer(struct app_ctx *ctx) {
    int32_t peak_mv = 0;
    for (uint32_t i = 0; i < ctx->diff_samples_processed; ++i) {
        int32_t raw = (int32_t)ctx->diff_samples[i];
        int32_t mv = raw;
        int rc = adc_raw_to_millivolts_dt(&adc_vadcdiff, &mv);
        if (rc < 0) {
            //skip sample on conversion failure
            continue;
        }
        int32_t abs_mv = (mv >= 0) ? mv : -mv;
        if (abs_mv > peak_mv) peak_mv = abs_mv;
    }

    ctx->diff_peak_abs_mv = (peak_mv > 0) ? peak_mv : 1;
    LOG_INF("Peak absolute voltage: %d mV", ctx->diff_peak_abs_mv);
    return 0;
}

static void diff_cleanup_after_read(struct app_ctx *ctx) {
    gpio_pin_set_dt(&ecg_led, 0);
    button1_set_enabled(ctx,true);
}

static int diff_adc_buffered_read_and_process(struct app_ctx *ctx)
{
    struct adc_sequence seq;
    int rc;

    rc = diff_prepare_sequence(ctx, &seq);
    if (rc < 0) {
        LOG_ERR("diff_prepare_sequence failed: %d", rc);
        return rc;
    }

    rc = diff_start_async_read(ctx, &seq);
    if (rc < 0) {
        // diff_start_async_read already did cleanup on error 
        return rc;
    }

    // compute timeout in ms for the whole requested sample buffer 
    uint32_t timeout_ms = (uint32_t)(((uint64_t)DIFF_BUFFER_LEN * (uint64_t)SAMPLE_BUFF_MARGIN) / DIFF_FS_HZ) + SAMPLE_BUFF_MARGIN;

    rc = diff_wait_for_completion(ctx, timeout_ms);
    if (rc == -EINTR) {
       //interrupted by button press or error
        return -EINTR;
    } else if (rc < 0) {
       //timeout or other error
        diff_cleanup_after_read(ctx);
        return rc;
    }

    // Successful acquisition --> compute peak and finalize 
    (void)diff_compute_peak_from_buffer(ctx);

    diff_cleanup_after_read(ctx);
    return 0;
}


// Temperature reading (MCP9808)
static int read_temperature_sensor(struct app_ctx *ctx, float *temperature_degC) {
    if (!temperature_degC) return -EINVAL;
    if (!device_is_ready(temp_sensor)) {
        LOG_ERR("Temperature sensor %s is not ready", temp_sensor ? temp_sensor->name : "NULL");
        return -ENODEV;
    }
    struct sensor_value sv;
    int err = sensor_sample_fetch(temp_sensor);
    if (err != 0) {
        LOG_ERR("Temperature sensor fetch(): %d", err);
        return err;
    }
    err = sensor_channel_get(temp_sensor, SENSOR_CHAN_AMBIENT_TEMP, &sv);
    if (err != 0) {
        LOG_ERR("Temperature sensor get(): %d", err);
        return err;
    }
    *temperature_degC = sensor_value_to_double(&sv);
    return 0;
}

static uint32_t calculate_heart_rate_from_samples(struct app_ctx *ctx) {
    if (ctx->diff_samples_processed == 0) {
        LOG_INF("No differential samples processed, cannot calculate heart rate");
        return 0;
    }
    
    // refractory expressed in samples (rounded up) i.e minimum samples between accepted peaks
    const uint32_t refractory_samples = (ECG_REFRACTORY_MS * DIFF_FS_HZ + 999) / 1000;

    int last_peak_idx = -((int)refractory_samples + 1); // ensure first peak can be accepted 
    uint32_t last_instant_bpm = 0;

    for (uint32_t i = 0; i < ctx->diff_samples_processed; ++i) {
        int32_t raw = (int32_t)ctx->diff_samples[i];
        int32_t mv = raw;
        int rc = adc_raw_to_millivolts_dt(&adc_vadcdiff, &mv);
        if (rc < 0) {
            continue; // skip sample on conversion failure 
        }

        // use absolute (differential) amplitude 
        uint32_t abs_mv = (mv >= 0) ? (uint32_t)mv : (uint32_t)(-mv);

        // threshold test 
        if (abs_mv >= ECG_QRS_THRESHOLD_MV) {
            // check refractory period
            if ((int)i - last_peak_idx > (int)refractory_samples) {
                if (last_peak_idx >= 0) {
                    // compute RR interval in ms between this peak and previous accepted peak 
                    uint32_t delta_samples = i - (uint32_t)last_peak_idx;
                    uint32_t delta_ms = (uint32_t)((uint64_t)delta_samples * 1000U / (uint64_t)DIFF_FS_HZ);
                    if (delta_ms > 0) {
                        uint32_t inst_bpm = (60000U + delta_ms/2) / delta_ms; // rounded 
                        //clamp values to min/max
                        if (inst_bpm < HR_MIN_BPM) inst_bpm = HR_MIN_BPM;
                        if (inst_bpm > HR_MAX_BPM) inst_bpm = HR_MAX_BPM;

                        // accumulate for averaging at report time 
                        ctx->hr_sum_bpm += inst_bpm;
                        ctx->hr_count += 1;

                        last_instant_bpm = inst_bpm;
                    }
                }
                // accept this as last peak (even if it's the first one) 
                last_peak_idx = (int)i;
            } else {
                // peak ignored: within refractory, prevents double counts 
            }
        }
    }

    return last_instant_bpm; 
}

//.................................................................
//PWM HELPERS......................................................

//set battery_led PWM brightness mapped from 0 to 3 V to 0 - 100% duty cycle
static int set_battery_led_brightness_from_mv(struct app_ctx *ctx, int32_t mv) {
    if (!device_is_ready(battery_led.dev)) {
        LOG_ERR("Battery_led PWM device not ready!");
        return -ENODEV;
    }

    if (mv <= 0) {
        int rc = pwm_set_pulse_dt(&battery_led, 0);
        if (rc) {
            LOG_ERR("Failed to set battery_led PWM to 0%% duty cycle: %d", rc);
            return rc;
        }
        LOG_INF("battery_led PWM updated: 0mV --> 0%% duty cycle");
        return 0;
    }

    if (mv >= ADC_MAX_MV_MAP) {
        int rc = pwm_set_pulse_dt(&battery_led, battery_led.period);
        if (rc) {
            LOG_ERR("Failed to set battery_led PWM to 100%% duty cycle: %d", rc);
            return rc;
        }
        LOG_INF("Battery_led PWM updated: %d mV --> 100%% duty cycle", ADC_MAX_MV_MAP);
        return 0;
    }

    uint32_t pulse_ns = ((uint64_t)battery_led.period * (uint64_t)mv) / (uint64_t)ADC_MAX_MV_MAP;
    if (pulse_ns == 0) 
        pulse_ns = 1;

    int rc = pwm_set_pulse_dt(&battery_led, pulse_ns);
    if (rc) {
        LOG_ERR("Failed to set battery_led PWM to %d mV (%u ns pulse): %d", mv, (unsigned int)pulse_ns, rc);
        return rc;
    } else {
        uint32_t pct = (uint32_t)((pulse_ns * 100ULL) / battery_led.period);
        LOG_INF("Battery_led PWM updated: %d mV -> %u%% (pulse %llu ns, period %u ns)", mv, pct, (unsigned long long)pulse_ns, battery_led.period);
    }
    return 0;
}


//SMF STATE MACHINE SETUP...................................
struct device_sm {
    struct smf_ctx ctx;
    struct app_ctx *app_ctx;
};

enum {ST_INIT, ST_IDLE, ST_ECG, ST_TEMP, ST_ERROR, ST_COUNT};

static const struct smf_state states[ST_COUNT] = {
    [ST_INIT]  = SMF_CREATE_STATE(init_entry,  init_run,  init_exit, NULL, NULL),
    [ST_IDLE] = SMF_CREATE_STATE(idle_entry,  idle_run,  idle_exit, NULL, NULL),
    [ST_ECG] = SMF_CREATE_STATE(ecg_entry, ecg_run, ecg_exit, NULL, NULL),
    [ST_TEMP] = SMF_CREATE_STATE(temp_entry, temp_run, temp_exit, NULL, NULL),
    [ST_ERROR] = SMF_CREATE_STATE(error_entry, error_run, error_exit, NULL, NULL),
};

static inline void set_state(struct smf_ctx *ctx, const struct smf_state *st) {
    smf_set_state(ctx, st);
}

//..............................................................

//ECG STATE HELPERS...........................................
static int ecg_get_event_action(struct app_ctx *ctx, uint32_t ev) {
    if (ev & ERROR_EVENT) {
        return ST_ERROR;
    }
    if (ev & BUTTON0_EVENT) {
        return ST_IDLE;
    }
    if (ev & BUTTON1_EVENT) {
        return ST_TEMP;
    }
    if (ev & BUTTON2_EVENT) {
        return ST_IDLE;
    }
    if (ev & BUTTON3_EVENT) {
        return ST_IDLE;
    }
    return -1; // No action needed
}

static void ecg_handle_event_side_effects(struct app_ctx *ctx, uint32_t ev) {
    if (ev & ERROR_EVENT) {
        LOG_INF("ECG: ERROR_EVENT");
        ctx->ecg_active = false;
    } else if (ev & BUTTON0_EVENT) {
        LOG_INF("ECG: BUTTON0 -> IDLE");
        ctx->ecg_active = false;
    } else if (ev & BUTTON1_EVENT) {
        LOG_INF("ECG: BUTTON1 -> TEMP");
        ctx->previous_state = ST_ECG;
        ctx->ecg_active = false;
    } else if (ev & BUTTON2_EVENT) {
        LOG_INF("ECG: BUTTON2 -> stop ECG (IDLE)");
        ctx->ecg_active = false;
        ctx->last_heart_rate_bpm = 0;
    } else if (ev & BUTTON3_EVENT) {
        LOG_INF("ECG: BUTTON3 -> clear errors and previous bpm + IDLE");
        ctx->error_flags = 0;
        ctx->errors.events = ctx->error_flags;
        bluetooth_notify_error(ctx->error_flags);
        ctx->last_heart_rate_bpm = 0;
        ctx->ecg_active = false;
    }
}

//STATE MACHINE.................................................

//init: configure GPIOS, ADC callbacks, start hertbeat
static void init_entry(void *obj) {
    ARG_UNUSED(obj);
    struct device_sm *sm = obj;
    struct app_ctx *ctx = sm->app_ctx;
    LOG_INF("INIT entry: configuring devices");
   
    //initialize bluetooth
    int bt_rc = bluetooth_init(&bluetooth_callbacks, &remote_service_callbacks);
    if (bt_rc) {
        LOG_ERR("Bluetooth init failed: %d", bt_rc);
        ctx->error_flags |= ERROR_TEMP_INIT;   
        ctx->errors.events = ctx->error_flags;
        k_event_post(&ctx->button_events, ERROR_EVENT);
    } else {
        LOG_INF("Bluetooth initialized");
    }

    //config LEDs

    if (device_is_ready(heartbeat_led.port)) {
        gpio_pin_configure_dt(&heartbeat_led, GPIO_OUTPUT_INACTIVE);
    } else {
        LOG_ERR("Heartbeat LED GPIO not ready!");
        ctx->error_flags |= ERROR_GPIO_INIT;
        k_event_post(&ctx->button_events, ERROR_EVENT);
    }

    //configure button(s)
    if (device_is_ready(button0.port)) {
        int rc = gpio_pin_configure_dt(&button0, GPIO_INPUT | GPIO_PULL_UP);
        if (rc < 0) {
            LOG_ERR("Failed to configure button0 GPIO: %d", rc);
            ctx->error_flags |= ERROR_GPIO_INIT;
            ctx->errors.events = ctx->error_flags;
            k_event_post(&ctx->button_events, ERROR_EVENT);
        }
        gpio_init_callback(&ctx->button0_cb, button0_callback, BIT(button0.pin));
        rc = gpio_add_callback(button0.port, &ctx->button0_cb);
        if (rc < 0) {
            LOG_ERR("Failed to add button0 callback: %d", rc);
        }
        gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_DISABLE); //start disabled
        ctx->button0_enabled = false;
    } else {
        LOG_ERR("Button0 GPIO not ready!");
        ctx->error_flags |= ERROR_GPIO_INIT;
        ctx->errors.events = ctx->error_flags;
        k_event_post(&ctx->button_events, ERROR_EVENT);
    }


    if (device_is_ready(button1.port)) {
        int rc = gpio_pin_configure_dt(&button1, GPIO_INPUT | GPIO_PULL_UP);
        if (rc < 0) {
            LOG_ERR("cfg button1 failed: %d", rc);
            ctx->error_flags |= ERROR_GPIO_INIT;
            ctx->errors.events = ctx->error_flags;
            k_event_post(&ctx->button_events, ERROR_EVENT);
        }
        gpio_init_callback(&ctx->button1_cb, button1_callback, BIT(button1.pin));
        rc = gpio_add_callback(button1.port, &ctx->button1_cb);
        if (rc < 0) 
            LOG_ERR("add cb button1 failed: %d", rc);
        gpio_pin_interrupt_configure_dt(&button1, GPIO_INT_DISABLE);
        ctx->button1_enabled = false;
    } else {
        LOG_ERR("button1 GPIO not ready");
        ctx->error_flags |= ERROR_GPIO_INIT;
        ctx->errors.events = ctx->error_flags;
    }

    if (device_is_ready(button2.port)) {
        int rc = gpio_pin_configure_dt(&button2, GPIO_INPUT | GPIO_PULL_UP);
        if (rc < 0) {
            LOG_ERR("cfg button2 failed: %d", rc);
            ctx->error_flags |= ERROR_GPIO_INIT;
            ctx->errors.events = ctx->error_flags;
            k_event_post(&ctx->button_events, ERROR_EVENT);
        }
        gpio_init_callback(&ctx->button2_cb, button2_callback, BIT(button2.pin));
        rc = gpio_add_callback(button2.port, &ctx->button2_cb);
        if (rc < 0) 
            LOG_ERR("add cb button2 failed: %d", rc);
        gpio_pin_interrupt_configure_dt(&button2, GPIO_INT_DISABLE);
        ctx->button2_enabled = false;
    } else {
        LOG_ERR("button2 GPIO not ready");
        ctx->error_flags |= ERROR_GPIO_INIT;
        ctx->errors.events = ctx->error_flags;
    }

     if (device_is_ready(button3.port)) {
        int rc = gpio_pin_configure_dt(&button3, GPIO_INPUT | GPIO_PULL_UP);
        if (rc < 0) {
            LOG_ERR("cfg button3 failed: %d", rc);
            ctx->error_flags |= ERROR_GPIO_INIT;
            ctx->errors.events = ctx->error_flags;
            k_event_post(&ctx->button_events, ERROR_EVENT);
        }
        gpio_init_callback(&ctx->button3_cb, button3_callback, BIT(button3.pin));
        rc = gpio_add_callback(button3.port, &ctx->button3_cb);
        if (rc < 0) 
            LOG_ERR("add cb button3 failed: %d", rc);
        gpio_pin_interrupt_configure_dt(&button3, GPIO_INT_DISABLE);
        ctx->button3_enabled = false;
    } else {
        LOG_ERR("button3 GPIO not ready");
        ctx->error_flags |= ERROR_GPIO_INIT;
        ctx->errors.events = ctx->error_flags;
    }

    //setup ADC
    int rc = adc_setup_and_check(ctx);
    if (rc == 0) {
        LOG_INF("ADC channel %d configured on %s", adc_vadc.channel_id, adc_vadc.dev->name);
    } else {
        LOG_ERR("ADC not confugred properly");
        ctx->error_flags |= ERROR_ADC_INIT;
        ctx->errors.events = ctx->error_flags;
        k_event_post(&ctx->button_events, ERROR_EVENT);
    }

    //temp sensor
    if (!device_is_ready(temp_sensor)) {
        LOG_ERR("Temperature sensor not ready!");
        ctx->error_flags |= ERROR_TEMP_INIT;
        ctx->errors.events = ctx->error_flags;
        k_event_post(&ctx->button_events, ERROR_EVENT);
    } else {
        LOG_INF("Temperature sensor %s ready", temp_sensor->name);
    }

    //set up LEDs

    if (device_is_ready(error_led.port)) {
        gpio_pin_configure_dt(&error_led, GPIO_OUTPUT_INACTIVE);
    } else {
        LOG_ERR("Error LED GPIO not ready!");
        ctx->error_flags |= ERROR_GPIO_INIT;
        ctx->errors.events = ctx->error_flags;
        k_event_post(&ctx->button_events, ERROR_EVENT);
    }

    if (device_is_ready(ecg_led.port)) {
        gpio_pin_configure_dt(&ecg_led, GPIO_OUTPUT_INACTIVE);
    } else {
        LOG_ERR("ECG LED GPIO not ready!");
        ctx->error_flags |= ERROR_GPIO_INIT;
        ctx->errors.events = ctx->error_flags;
        k_event_post(&ctx->button_events, ERROR_EVENT);
    }
    
    //pwm battery_led
    if (!device_is_ready(battery_led.dev)) {
        LOG_ERR("battery_led PWM device not ready!");
        ctx->error_flags |= ERROR_PWM_INIT;
        ctx->errors.events = ctx->error_flags;
        k_event_post(&ctx->button_events, ERROR_EVENT);
    } else {
        LOG_INF("battery_led PWM configured on %s", battery_led.dev->name);
        /* Ensure battery_led off initially */
        (void)pwm_set_pulse_dt(&battery_led, 0);
    }

    //initialize batttery time stamp
    ctx->last_battery_mv = 0;
    // allow heartbeat thread and heartrate thread to run once GPIO initialized 
    ctx->heartbeat_allowed = true;
    LOG_INF("INIT entry complete");
}

static void init_run(void *obj) {
    struct device_sm *sm = obj;
    struct app_ctx *ctx = sm->app_ctx;

    LOG_INF("INIT run: performing initial battery measurement");
    int32_t mv;
    int rc = adc_measure_mv(ctx, &mv);
    if (rc == 0) {
        ctx->last_battery_mv = mv;
        LOG_INF("Initial battery voltage: %d mV", ctx->last_battery_mv);
        set_battery_led_brightness_from_mv(ctx, ctx->last_battery_mv);
        bluetooth_set_battery_level(ctx->last_battery_mv);
    } else {
        LOG_ERR("Initial battery measurement failed: %d", rc);
        ctx->error_flags |= ERROR_ADC_READ;
        ctx->errors.events = ctx->error_flags;
        bluetooth_notify_error(ctx->error_flags);
        set_state(&sm->ctx, &states[ST_ERROR]);
        return;
    }

    ctx->last_battery_check = k_uptime_get();

    button0_set_enabled(ctx, true); //enable button0 to return to idle 
    button1_set_enabled(ctx, true); //enable button1 to temp read
    button2_set_enabled(ctx, true); //enable button2 to read ecg
    button3_set_enabled(ctx, true); //enable button3 to reset

    ctx->previous_state = ST_INIT; //for purposes of returning from TEMP
    set_state(&sm->ctx, &states[ST_IDLE]);
    return;
}

static void init_exit(void *obj) {
    ARG_UNUSED(obj);
    LOG_INF("Exiting INIT");
}

static void idle_entry(void *obj) {
    ARG_UNUSED(obj);
    struct device_sm *sm = obj;
    struct app_ctx *ctx = sm->app_ctx;
    LOG_INF("IDLE entry: reenabling buttons. ");
    button0_set_enabled(ctx, true); 
    button1_set_enabled(ctx, true); 
    button2_set_enabled(ctx, true); 
    button3_set_enabled(ctx, true);
}

static void idle_run(void *obj) {
    struct device_sm *sm = obj;
    struct app_ctx *ctx = sm->app_ctx;

    if (ctx->last_battery_check == 0) {
        ctx->last_battery_check = k_uptime_get();
    }

    while (1) {
        //battery check only happens while in idle
        int64_t now = k_uptime_get();
        if ((now - ctx->last_battery_check) >= BATTERY_CHECK_INTERVAL_MS) {
            LOG_INF("IDLE: Performing battery measurement");
            int32_t mv = 0;
            int rc = adc_measure_mv(ctx, &mv);
            if (rc != 0) {
                LOG_ERR("Battery measurement failed: %d", rc);
                ctx->error_flags |= ERROR_ADC_READ;
                bluetooth_notify_error(ctx->error_flags);
                k_event_post(&ctx->button_events, ERROR_EVENT);
                set_state(&sm->ctx, &states[ST_ERROR]);
                return;
            }
            ctx->last_battery_mv = mv;
            int rc2 = set_battery_led_brightness_from_mv(ctx, ctx->last_battery_mv);
            if (rc2 != 0) {
                LOG_ERR("Failed to update battery LED brightness: %d", rc2);
                ctx->error_flags |= ERROR_PWM_INIT; //non-fatal error, doesnt go to error state
                ctx->errors.events = ctx->error_flags;
                bluetooth_notify_error(ctx->error_flags);
            }
            LOG_INF("Battery voltage: %d mV", ctx->last_battery_mv);
            bluetooth_set_battery_level(ctx->last_battery_mv);
            ctx->last_battery_check = now;
        }

        //wait for user event
        uint32_t ev = k_event_wait(&ctx->button_events, BUTTON0_EVENT | BUTTON1_EVENT | BUTTON2_EVENT | BUTTON3_EVENT | ERROR_EVENT, true, K_MSEC(BUTTON_WAIT_MS));

        if (ev & ERROR_EVENT) {
            LOG_INF("IDLE: ERROR_EVENT posted, transitioning to ERROR state");
            set_state(&sm->ctx, &states[ST_ERROR]);
            return;
        }

        if (ev & BUTTON0_EVENT) {
            LOG_INF("IDLE: BUTTON0_EVENT detected, staying in IDLE");
            continue; //stay in idle
        }

        if (ev & BUTTON1_EVENT) {
            LOG_INF("IDLE: BUTTON1_EVENT detected, transitioning to TEMP state");
            ctx->previous_state = ST_IDLE;
            set_state(&sm->ctx, &states[ST_TEMP]);
            return;
        }

        if (ev & BUTTON2_EVENT) {
            LOG_INF("IDLE: BUTTON2_EVENT detected, transitioning to ECG state");
            set_state(&sm->ctx, &states[ST_ECG]);
            return;
        }

        if (ev & BUTTON3_EVENT) {
            LOG_INF("IDLE: BUTTON3_EVENT detected, already in IDLE");
            continue; //stay in idle
        }
    }
}

static void idle_exit(void *obj) {
    ARG_UNUSED(obj);
    LOG_INF("IDLE exit");
}


static void ecg_entry(void *obj) {
    ARG_UNUSED(obj);
    struct device_sm *sm = obj;
    struct app_ctx *ctx = sm->app_ctx;
    LOG_INF("ECG entry: Ensuring buttons enabled, clearing stale events");
    (void)button0_set_enabled(ctx, true); // allow return to idle 
    (void)button1_set_enabled(ctx, true); // allow temperature reads during ECG 
    (void)button2_set_enabled(ctx, true); // start/stop ECG 
    (void)button3_set_enabled(ctx, true); // allow reset during ECG

    // Clear stale events that could interfere with starting ECG 
    (void)k_event_wait(&ctx->button_events, BUTTON2_EVENT, true, K_NO_WAIT);
    (void)k_event_wait(&ctx->button_events, DIFF_DONE_EVENT, true, K_NO_WAIT);
    
    // Clear any pending button event from previous state
    ctx->pending_button_event = 0;
}


static void ecg_run(void *obj) {
    struct device_sm *sm = obj;
    struct app_ctx *ctx = sm->app_ctx;
    ctx->ecg_active = true;

    while (ctx->ecg_active) {

        //immediate non-blocking button poll
        uint32_t ev = k_event_wait(&ctx->button_events,BUTTON0_EVENT | BUTTON1_EVENT | BUTTON2_EVENT |BUTTON3_EVENT | ERROR_EVENT,true,K_NO_WAIT); 

        if (ev) {
            int target_state = ecg_get_event_action(ctx, ev);
            if (target_state >= 0) {
                ecg_handle_event_side_effects(ctx, ev);
                set_state(&sm->ctx, &states[target_state]);
                return;
            }
        }

        //* No button event -> perform ONE ECG ADC cycle 
        LOG_INF("ECG run: starting differential ADC read");
        int rc = diff_adc_buffered_read_and_process(ctx);

        if (rc == -EINTR) {
            // ADC cycle aborted by button event - handle it immediately 
            uint32_t ev = ctx->pending_button_event;
            ctx->pending_button_event = 0; // Clear the pending event
            
            // Clear the event from the event system as well
            if (ev) {
                k_event_wait(&ctx->button_events, ev, true, K_NO_WAIT);
            }
            
            LOG_INF("ECG read interrupted by event: 0x%08X", ev);
            
            //handle interupt logic the same as previously
            if (ev) {
                int target_state = ecg_get_event_action(ctx, ev);
                if (target_state >= 0) {
                    ecg_handle_event_side_effects(ctx, ev);
                    set_state(&sm->ctx, &states[target_state]);
                    return;
                }
            }
            
            // Continue the loop to check for events normally
            continue;
        }

        if (rc != 0) {
            LOG_ERR("ECG ADC error: %d", rc);
            ctx->error_flags |= ERROR_ADC_READ;
            ctx->errors.events = ctx->error_flags;
            bluetooth_notify_error(ctx->error_flags);
            ctx->ecg_active = false;
            set_state(&sm->ctx, &states[ST_ERROR]);
            return;
        }

        // ECG cycle completed normally --> compute HR 
        uint32_t inst_hr = calculate_heart_rate_from_samples(ctx);
        ctx->last_heart_rate_bpm = inst_hr;
        uint32_t report_hr = inst_hr;

        if (ctx->hr_count > 0) {
            // compute average HR over the period if available
            report_hr = (ctx->hr_sum_bpm + ctx->hr_count/2) / ctx->hr_count;
            ctx->hr_sum_bpm = 0;
            ctx->hr_count = 0;
        }

        if (report_hr == 0) {
            report_hr = HR_MIN_BPM; //for testing purposes 
        }

        ctx->last_heart_rate_bpm = report_hr;
        LOG_INF("ECG HR (reported average) = %u bpm", report_hr);
        rc = bt_hrs_notify((uint8_t)report_hr);
        if (rc) {
            LOG_WRN("bt_hrs_notify() returned %d", rc);
        }
        // Loop back, checking buttons immediately 
    }
}

static void ecg_exit(void *obj) {
    struct device_sm *sm = obj;
    struct app_ctx *ctx = sm->app_ctx;
    ARG_UNUSED(obj);
    LOG_INF("ECG exit: disabling ECG activity, turning off ECG LED");
    ctx->ecg_active = false;
    gpio_pin_set_dt(&ecg_led, 0); //turn off ECG LED
}

static void temp_entry(void *obj) {
    ARG_UNUSED(obj);
    struct device_sm *sm = obj;
    struct app_ctx *ctx = sm->app_ctx;
    LOG_INF("TEMP entry: previous_state=%d", ctx->previous_state);
    (void)button0_set_enabled(ctx, true); // allow return to idle 
    (void)button3_set_enabled(ctx, true); // allow reset
    (void)button1_set_enabled(ctx, false); // prevent re-press
}

static void temp_run(void *obj) {
    struct device_sm *sm = obj;
    struct app_ctx *ctx = sm->app_ctx;

    float t = 0.0f;
    int rc = read_temperature_sensor(ctx, &t);
    if (rc != 0) {
        LOG_ERR("Temperature read failed: %d", rc);
        ctx->error_flags |= ERROR_TEMP_READ;
        ctx->errors.events = ctx->error_flags;
        bluetooth_notify_error(ctx->error_flags);
        k_event_post(&ctx->button_events, ERROR_EVENT);
        set_state(&sm->ctx, &states[ST_ERROR]);
        return;
    }
    ctx->last_temp_c = t;
    float tmp = ctx->last_temp_c;
    int t_whole = (int)tmp;
    float frac = tmp - (float)t_whole;
    if (frac < 0.0f) {
        frac = -frac;
    }
    int t_frac = (int)(frac * 100.0f + 0.5f); // round to 2 dp 
    LOG_INF("TEMP run: Measured temperature: %d.%02d degC", t_whole, t_frac);

     // Round to nearest integer Celsius (e.g., 23.6 -> 24, 23.4 -> 23) 
    ctx->temperature_degC = (int32_t) ( (t >= 0.0f) ? (int32_t)(t + 0.5f) : (int32_t)(t - 0.5f) );
    bluetooth_notify_temperature(ctx->temperature_degC);

    //give user a small window to press BUTTON0 (to return to IDLE) or BUTTON3 (to reset)
    uint32_t ev = k_event_wait(&ctx->button_events, BUTTON0_EVENT | BUTTON3_EVENT | ERROR_EVENT, true, K_MSEC(SAMPLE_BUFF_MARGIN));
    if (ev & ERROR_EVENT) {
        LOG_INF("TEMP: ERROR_EVENT posted, transitioning to ERROR state");
        set_state(&sm->ctx, &states[ST_ERROR]);
        return;
    }

    if (ev & BUTTON0_EVENT) {
        LOG_INF("TEMP: BUTTON0_EVENT detected, returning to IDLE state");
        set_state(&sm->ctx, &states[ST_IDLE]);
        return;
    }
    if (ev & BUTTON3_EVENT) {
        LOG_INF("TEMP: BUTTON3_EVENT detected, resetting device");
        ctx->error_flags = 0;
        ctx->errors.events = ctx->error_flags;
        bluetooth_notify_error(ctx->error_flags);
        ctx->last_heart_rate_bpm = 0;
        ctx->last_temp_c = 0.0f;
        set_state(&sm->ctx, &states[ST_IDLE]);
        
        return;
    }

    //default return to previous state if no button press.
    if (ctx->previous_state == ST_IDLE) {
        LOG_INF("TEMP: returning to IDLE state");
        set_state(&sm->ctx, &states[ST_IDLE]);
    } else if (ctx->previous_state == ST_ECG) {
        LOG_INF("TEMP: returning to ECG state");
        set_state(&sm->ctx, &states[ST_ECG]);
    } 
}

static void temp_exit(void *obj) {
    struct device_sm *sm = obj;
    struct app_ctx *ctx = sm->app_ctx;
    ARG_UNUSED(obj);
    LOG_INF("TEMP exit: re-enabling button1");
    (void)button1_set_enabled(ctx, true); //re-enable button1
}

static void error_entry(void *obj) {
    struct device_sm *sm = obj;
    struct app_ctx *ctx = sm->app_ctx;
    ARG_UNUSED(obj);
    LOG_ERR("ERROR entry: flags=0x%08X", ctx->error_flags);
    
    button0_set_enabled(ctx, false); //disable normal return to idle 
    button1_set_enabled(ctx, false); //disable temp read
    button2_set_enabled(ctx, false); //disable ECG
    button3_set_enabled(ctx, true); //allow reset

    gpio_pin_set_dt(&error_led, 0); //start off
    gpio_pin_set_dt(&ecg_led, 0); //turn off ECG LED
    pwm_set_pulse_dt(&battery_led, 0); //turn off battery LED

    ctx->errors.events = ctx->error_flags;
    bluetooth_notify_error(ctx->error_flags);

    LOG_INF("ERROR entry complete: LEDs primed");
}

static void error_run(void *obj) {
    struct device_sm *sm = obj;
    struct app_ctx *ctx = sm->app_ctx;

    //mirror heartbeat LED to other LEDs. BUTTON3 resets

     while (1) {
        int hb_level = ctx->heartbeat_state;
        if (hb_level == 0 || hb_level == 1) {
            //normal heartbeat state: mirror to other LEDs
            if (device_is_ready(error_led.port)) gpio_pin_set_dt(&error_led, hb_level);
            if (device_is_ready(ecg_led.port))   gpio_pin_set_dt(&ecg_led, hb_level);
            if (device_is_ready(battery_led.dev)) {
                if (hb_level == 1) {
                    (void)pwm_set_pulse_dt(&battery_led, battery_led.period);
                } else {
                    (void)pwm_set_pulse_dt(&battery_led, 0);
                }
            }
        } else {
            // fallback: toggle other LEDs slowly if heartbeat GPIO read fails 
            if (device_is_ready(error_led.port)) gpio_pin_toggle_dt(&error_led);
            if (device_is_ready(ecg_led.port))   gpio_pin_toggle_dt(&ecg_led);
            if (device_is_ready(battery_led.dev)) {
                (void)pwm_set_pulse_dt(&battery_led, battery_led.period);
                k_msleep(ERROR_LED_ON_MS);
                (void)pwm_set_pulse_dt(&battery_led, 0);
            }
        }
        //wait for BUTTON3 to reset
        uint32_t ev = k_event_wait(&ctx->button_events, BUTTON3_EVENT, true, K_MSEC(50));

        if (ev & BUTTON3_EVENT) {
            LOG_INF("ERROR: BUTTON3_EVENT detected, resetting device");
            ctx->error_flags = 0; //clear errors
            ctx->ecg_active = false;
            ctx->errors.events = ctx->error_flags;
            bluetooth_notify_error(ctx->error_flags);
            gpio_pin_set_dt(&error_led, 0); //turn off error LED
            gpio_pin_set_dt(&ecg_led, 0); //turn off ECG LED
            pwm_set_pulse_dt(&battery_led, 0); //turn off battery LED

            set_state(&sm->ctx, &states[ST_INIT]);
            return;
        }
    }
}

static void error_exit(void *obj) {
    struct device_sm *sm = obj;
    struct app_ctx *ctx = sm->app_ctx;
    ARG_UNUSED(obj);
    LOG_INF("ERROR exit: disabling, reenabling buttons");
    ctx->heartbeat_state = 0; //reset heartbeat state
    //reenable buttons
    button0_set_enabled(ctx, true); 
    button1_set_enabled(ctx, true); 
    button2_set_enabled(ctx, true); 
    button3_set_enabled(ctx, true);
}


//..........................................................

int main(void) {
    static struct device_sm smachine;

    memset(&app, 0, sizeof(app));
    k_event_init(&app.button_events);
    k_event_init(&app.errors);

    smachine.app_ctx = &app; //link context
    g_app_ctx_for_isr = &app;//ADC isr access
    // set initial state
    smf_set_initial(&smachine.ctx, &states[ST_INIT]);

    while (1) {
        int32_t rc = smf_run_state(&smachine.ctx);
        if (rc != 0) {
            if (rc < 0) {
                LOG_ERR("smf_run_state returned error: %d", rc);
            } else {
                LOG_INF("smf terminated with value: %d", rc);
            }
            break;
        }
        k_msleep(1); // avoid busy loop
    }

    return 0;
}