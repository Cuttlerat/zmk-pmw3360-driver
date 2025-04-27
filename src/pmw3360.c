/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pixart_pmw3360

// 12-bit two's complement value to int16_t
// adapted from https://stackoverflow.com/questions/70802306/convert-a-12-bit-signed-number-in-c
#define TOINT16(val, bits) (((struct { int16_t value : bits; }){val}).value)

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zmk/keymap.h>
#include "pmw3360.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pmw3360, CONFIG_INPUT_LOG_LEVEL);

//////// Sensor initialization steps definition //////////
// init is done in non-blocking manner (i.e., async), a //
// delayable work is defined for this purpose           //
enum pmw3360_init_step {
    ASYNC_INIT_STEP_POWER_UP,    // reset cs line and assert power-up reset
    ASYNC_INIT_STEP_SROM_ENABLE, // enable SROM
    ASYNC_INIT_STEP_CONFIGURE,   // set other registers like cpi and downshift time (run, rest1, rest2)
                               // and clear motion registers

    ASYNC_INIT_STEP_COUNT // end flag
};

/* Timings (in ms) needed in between steps to allow each step finishes succussfully. */
// - Since MCU is not involved in the sensor init process, i is allowed to do other tasks.
//   Thus, k_sleep or delayed schedule can be used.
static const int32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {
    [ASYNC_INIT_STEP_POWER_UP] = 50,    // Power-up delay for PMW3360
    [ASYNC_INIT_STEP_SROM_ENABLE] = 10, // SROM enable delay
    [ASYNC_INIT_STEP_CONFIGURE] = 0,    // No delay needed after configuration
};

static int pmw3360_async_init_power_up(const struct device *dev);
static int pmw3360_async_init_srom_enable(const struct device *dev);
static int pmw3360_async_init_configure(const struct device *dev);

static int (*const async_init_fn[ASYNC_INIT_STEP_COUNT])(const struct device *dev) = {
    [ASYNC_INIT_STEP_POWER_UP] = pmw3360_async_init_power_up,
    [ASYNC_INIT_STEP_SROM_ENABLE] = pmw3360_async_init_srom_enable,
    [ASYNC_INIT_STEP_CONFIGURE] = pmw3360_async_init_configure,
};

//////// Function definitions //////////

static int spi_cs_ctrl(const struct device *dev, bool enable) {
    const struct pixart_config *config = dev->config;
    int err;

    if (!enable) {
        k_busy_wait(T_NCS_SCLK);
    }

    err = gpio_pin_set_dt(&config->cs_gpio, (int)enable);
    if (err) {
        LOG_ERR("SPI CS ctrl failed");
    }

    if (enable) {
        k_busy_wait(T_NCS_SCLK);
    }

    return err;
}

static int reg_read(const struct device *dev, uint8_t reg, uint8_t *buf) {
    int err;
    const struct pixart_config *config = dev->config;

    __ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    /* Write register address. */
    const struct spi_buf tx_buf = {.buf = &reg, .len = 1};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Reg read failed on SPI write");
        return err;
    }

    k_busy_wait(T_SRAD);

    /* Read register value. */
    struct spi_buf rx_buf = {
        .buf = buf,
        .len = 1,
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1,
    };

    err = spi_read_dt(&config->bus, &rx);
    if (err) {
        LOG_ERR("Reg read failed on SPI read");
        return err;
    }

    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    k_busy_wait(T_SRX);

    return 0;
}

static int reg_write(const struct device *dev, uint8_t reg, uint8_t val) {
    int err;
    const struct pixart_config *config = dev->config;

    __ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    uint8_t buf[] = {SPI_WRITE_BIT | reg, val};
    const struct spi_buf tx_buf = {.buf = buf, .len = ARRAY_SIZE(buf)};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Reg write failed on SPI write");
        return err;
    }

    k_busy_wait(T_SCLK_NCS_WR);

    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    k_busy_wait(T_SWX);

    return 0;
}

static int motion_burst_read(const struct device *dev, uint8_t *buf, size_t burst_size) {
    int err;
    const struct pixart_config *config = dev->config;

    __ASSERT_NO_MSG(burst_size <= PMW3360_MAX_BURST_SIZE);

    uint8_t reg_buf[] = {PMW3360_REG_MOTION_BURST};

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    const struct spi_buf tx_buf = {.buf = reg_buf, .len = ARRAY_SIZE(reg_buf)};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Motion burst failed on SPI write");
        return err;
    }

    k_busy_wait(T_SRAD_MOTBR);

    const struct spi_buf rx_buf = {
        .buf = buf,
        .len = burst_size,
    };
    const struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

    err = spi_read_dt(&config->bus, &rx);
    if (err) {
        LOG_ERR("Motion burst failed on SPI read");
        return err;
    }

    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    return 0;
}

static int burst_write(const struct device *dev, const uint8_t *addr, const uint8_t *buf,
                       size_t size) {
    int err;
    const struct pixart_config *config = dev->config;

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    const struct spi_buf tx_buf[] = {
        {.buf = (uint8_t *)addr, .len = 1},
        {.buf = (uint8_t *)buf, .len = size}
    };
    const struct spi_buf_set tx = {.buffers = tx_buf, .count = ARRAY_SIZE(tx_buf)};

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Burst write failed on SPI write");
        return err;
    }

    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    return 0;
}

static int check_product_id(const struct device *dev) {
    uint8_t product_id;
    int err = reg_read(dev, PMW3360_REG_PRODUCT_ID, &product_id);
    if (err) {
        LOG_ERR("Cannot read product ID");
        return err;
    }

    LOG_INF("Product ID: 0x%x", product_id);
    if (product_id != PMW3360_PRODUCT_ID) {
        LOG_ERR("Invalid product ID: 0x%x", product_id);
        return -EIO;
    }

    return 0;
}

static int set_cpi(const struct device *dev, uint32_t cpi) {
    /* Resolution is set in counts per inch (CPI).
     * Value is written to resolution register in the device as 
     * resolution = cpi / 100 (5-bit number, 0x00 to 0x77)
     */
    
    if ((cpi > PMW3360_MAX_CPI) || (cpi < PMW3360_MIN_CPI)) {
        LOG_ERR("CPI value %u out of range (%u-%u)", cpi, PMW3360_MIN_CPI, PMW3360_MAX_CPI);
        return -EINVAL;
    }

    /* Convert CPI to register value */
    uint8_t value = (uint8_t)(cpi / 100);
    
    int err = reg_write(dev, PMW3360_REG_RESOLUTION, value);
    if (err) {
        LOG_ERR("Cannot set CPI");
        return err;
    }

    struct pixart_data *dev_data = dev->data;
    dev_data->curr_cpi = cpi;

    return 0;
}

static int set_cpi_if_needed(const struct device *dev, uint32_t cpi) {
    struct pixart_data *data = dev->data;

    if (data->curr_cpi != cpi) {
        return set_cpi(dev, cpi);
    }

    return 0;
}

static int set_downshift_time(const struct device *dev, uint8_t reg_addr, uint32_t time) {
    uint8_t value = 0;

    /* Resolution is set as 10 ms multiplier (except for Run mode).
     * Max allowed value is 2550 ms.
     */
    if ((reg_addr == PMW3360_REG_RUN_DOWNSHIFT) || (reg_addr == PMW3360_REG_REST1_DOWNSHIFT) || 
        (reg_addr == PMW3360_REG_REST2_DOWNSHIFT)) {
        if (time > 2550) {
            LOG_WRN("Downshift time %u out of range. Using 2550 ms", time);
            time = 2550;
        }

        if (reg_addr == PMW3360_REG_RUN_DOWNSHIFT) {
            /* Run downshift time is expressed in 1 ms intervals */
            value = time;
        } else {
            /* Rest downshift time is expressed in 10 ms intervals */
            value = time / 10;
        }
    } else {
        LOG_ERR("Invalid registry for downshift setting: 0x%x", reg_addr);
        return -EINVAL;
    }

    return reg_write(dev, reg_addr, value);
}

static int set_sample_time(const struct device *dev, uint8_t reg_addr, uint32_t sample_time) {
    uint8_t value = 0;

    /* Rest periods are expressed in frames.
     * Frame duration = 4+(ADDITIONAL_X*4)+ADDITIONAL_Y 
     */
    if ((reg_addr == PMW3360_REG_REST1_RATE) || (reg_addr == PMW3360_REG_REST2_RATE) || 
        (reg_addr == PMW3360_REG_REST3_RATE)) {
        if (sample_time > 255) {
            LOG_WRN("Sample time %u out of range. Using 255", sample_time);
            sample_time = 255;
        }
        value = sample_time;
    } else {
        LOG_ERR("Invalid registry for sample time setting: 0x%x", reg_addr);
        return -EINVAL;
    }

    return reg_write(dev, reg_addr, value);
}

static int pmw3360_async_init_power_up(const struct device *dev) {
    int err = 0;

    /* Reset sensor */
    err = reg_write(dev, PMW3360_REG_POWER_UP_RESET, PMW3360_POWERUP_CMD_RESET);
    if (err) {
        LOG_ERR("Cannot reset");
        return err;
    }

    k_busy_wait(T_SRAD_MOTBR);

    /* Read from registers to clear them */
    uint8_t buf[5];
    err = reg_read(dev, PMW3360_REG_MOTION, &buf[0]);
    err = reg_read(dev, PMW3360_REG_DELTA_X_L, &buf[1]);
    err = reg_read(dev, PMW3360_REG_DELTA_Y_L, &buf[2]);
    err = reg_read(dev, PMW3360_REG_DELTA_XY_H, &buf[3]);

    return err;
}

static int pmw3360_async_init_srom_enable(const struct device *dev) {
    int err;

    /* Enable SROM */
    err = reg_write(dev, PMW3360_REG_CONFIG2, 0x00);
    if (err) {
        LOG_ERR("Cannot set CONFIG2 register");
        return err;
    }

    /* Initialize SROM */
    err = reg_write(dev, PMW3360_REG_SROM_ENABLE, 0x1D);
    if (err) {
        LOG_ERR("Cannot initialize SROM");
        return err;
    }

    k_busy_wait(10);
    
    err = reg_write(dev, PMW3360_REG_SROM_ENABLE, 0x18);
    if (err) {
        LOG_ERR("Cannot enable SROM");
        return err;
    }

    return 0;
}

static int pmw3360_async_init_configure(const struct device *dev) {
    int err;
    struct pixart_data *data = dev->data;

    /* Check chip ID */
    err = check_product_id(dev);
    if (err) {
        return err;
    }

    /* Set initial CPI value */
    err = set_cpi(dev, CONFIG_PMW3360_CPI);
    if (err) {
        LOG_ERR("Cannot set CPI");
        return err;
    }

    /* Configure rest modes */
    err = set_downshift_time(dev, PMW3360_REG_RUN_DOWNSHIFT, CONFIG_PMW3360_RUN_DOWNSHIFT_TIME_MS);
    if (err) {
        LOG_ERR("Cannot set run downshift time");
        return err;
    }

    err = set_downshift_time(dev, PMW3360_REG_REST1_DOWNSHIFT, CONFIG_PMW3360_REST1_DOWNSHIFT_TIME_MS);
    if (err) {
        LOG_ERR("Cannot set rest1 downshift time");
        return err;
    }

    err = set_downshift_time(dev, PMW3360_REG_REST2_DOWNSHIFT, CONFIG_PMW3360_REST2_DOWNSHIFT_TIME_MS);
    if (err) {
        LOG_ERR("Cannot set rest2 downshift time");
        return err;
    }

    err = set_sample_time(dev, PMW3360_REG_REST1_RATE, CONFIG_PMW3360_REST1_SAMPLE_TIME);
    if (err) {
        LOG_ERR("Cannot set rest1 sample time");
        return err;
    }

    err = set_sample_time(dev, PMW3360_REG_REST2_RATE, CONFIG_PMW3360_REST2_SAMPLE_TIME);
    if (err) {
        LOG_ERR("Cannot set rest2 sample time");
        return err;
    }

    err = set_sample_time(dev, PMW3360_REG_REST3_RATE, CONFIG_PMW3360_REST3_SAMPLE_TIME);
    if (err) {
        LOG_ERR("Cannot set rest3 sample time");
        return err;
    }

    /* Set lift detection threshold */
    err = reg_write(dev, PMW3360_REG_LIFT_CONFIG, CONFIG_PMW3360_LIFT_DETECTION_THRESHOLD);
    if (err) {
        LOG_ERR("Cannot set lift detection threshold");
        return err;
    }

    /* Ensure clean motion reports */
    uint8_t motion_status;
    err = reg_read(dev, PMW3360_REG_MOTION, &motion_status);
    if (err) {
        LOG_ERR("Cannot read motion register");
        return err;
    }

    /* Set current input mode */
    data->curr_mode = MOVE;
    
    /* Mark sensor as ready */
    data->ready = true;

    LOG_INF("PMW3360 initialized with CPI %u", CONFIG_PMW3360_CPI);

    return 0;
}

static void set_interrupt(const struct device *dev, const bool en) {
    const struct pixart_config *config = dev->config;

    if (en) {
        gpio_pin_interrupt_configure_dt(&config->irq_gpio, GPIO_INT_LEVEL_ACTIVE);
    } else {
        gpio_pin_interrupt_configure_dt(&config->irq_gpio, GPIO_INT_DISABLE);
    }
}

static void pmw3360_async_init(struct k_work *work) {
    struct k_work_delayable *work2 = (struct k_work_delayable *)work;
    struct pixart_data *data = CONTAINER_OF(work2, struct pixart_data, init_work);
    const struct device *dev = data->dev;

    LOG_DBG("PMW3360 async init, step %d", data->async_init_step);

    int err = async_init_fn[data->async_init_step](dev);
    if (err) {
        LOG_ERR("PMW3360 initialization failed at step %d", data->async_init_step);
        data->err = err;
        return;
    }

    data->async_init_step++;

    if (data->async_init_step < ASYNC_INIT_STEP_COUNT) {
        /* Schedule next init step */
        k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step - 1]));
    } else {
        /* Initialization completed */
        set_interrupt(dev, true);
    }
}

struct k_timer automouse_layer_timer;

static void activate_automouse_layer() {
    int layer_state = zmk_keymap_highest_layer_active();
    if (!IS_ENABLED(CONFIG_ZMK_MOUSE_LAYER_AUTO_MOUSE_LAYER) || layer_state == CONFIG_ZMK_MOUSE_LAYER_AUTO_MOUSE_LAYER)
        return;
    zmk_keymap_layer_activate(CONFIG_ZMK_MOUSE_LAYER_AUTO_MOUSE_LAYER);
}

static void deactivate_automouse_layer(struct k_timer *timer) {
    int layer_state = zmk_keymap_highest_layer_active();
    if (!IS_ENABLED(CONFIG_ZMK_MOUSE_LAYER_AUTO_MOUSE_LAYER) || layer_state != CONFIG_ZMK_MOUSE_LAYER_AUTO_MOUSE_LAYER)
        return;
    zmk_keymap_layer_deactivate(CONFIG_ZMK_MOUSE_LAYER_AUTO_MOUSE_LAYER);
}

static enum pixart_input_mode get_input_mode_for_current_layer(const struct device *dev) {
    const struct pixart_config *config = dev->config;
    int32_t layer = zmk_keymap_highest_layer_active();

    for (int i = 0; i < config->scroll_layers_len; i++) {
        if (layer == config->scroll_layers[i]) {
            return SCROLL;
        }
    }

    for (int i = 0; i < config->snipe_layers_len; i++) {
        if (layer == config->snipe_layers[i]) {
            return SNIPE;
        }
    }

    return MOVE;
}

static int pmw3360_report_data(const struct device *dev) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    uint8_t buf[PMW3360_BURST_SIZE];
    int err;

    // determine input mode
    enum pixart_input_mode input_mode = get_input_mode_for_current_layer(dev);
    if (data->curr_mode != input_mode) {
        // device input mode switching, we switch cpi/dpi
        LOG_DBG("Input mode changed from %d to %d", data->curr_mode, input_mode);
        switch (input_mode) {
        case MOVE:
            err = set_cpi_if_needed(dev, CONFIG_PMW3360_CPI);
            break;
        case SCROLL:
            err = set_cpi_if_needed(dev, CONFIG_PMW3360_SCROLL_CPI);
            break;
        case SNIPE:
            err = set_cpi_if_needed(dev, CONFIG_PMW3360_SNIPE_CPI);
            break;
        }
        data->curr_mode = input_mode;
    }

    if (input_mode == SCROLL && !CONFIG_PMW3360_SCROLL_SMART_MODE) {
        input_mode = MOVE;
    }

    /* Get data from the sensor */
    err = motion_burst_read(dev, buf, PMW3360_BURST_SIZE);
    if (err) {
        LOG_ERR("Motion burst failed");
        return err;
    }

    uint8_t motion = buf[0];

    if ((motion & 0x80) != 0) {
        int16_t x = buf[PMW3360_X_L_POS] | ((buf[PMW3360_XY_H_POS] & 0x0F) << 8);
        int16_t y = buf[PMW3360_Y_L_POS] | ((buf[PMW3360_XY_H_POS] & 0xF0) << 4);

        /* Convert 12-bit signed values to 16-bit signed values */
        x = TOINT16(x, 12);
        y = TOINT16(y, 12);

        /* Get only absolute values */
        int16_t abs_x = abs(x);
        int16_t abs_y = abs(y);

        switch (input_mode) {
        case MOVE:
            if (CONFIG_ZMK_MOUSE_LAYER_AUTO_MOUSE_LAYER > 0) {
                activate_automouse_layer();
                k_timer_stop(&automouse_layer_timer);
                k_timer_start(&automouse_layer_timer,
                            K_MSEC(CONFIG_ZMK_MOUSE_LAYER_AUTO_MOUSE_LAYER_TIMEOUT),
                            K_NO_WAIT);
            }

            if (abs_x > 0 || abs_y > 0) {
                // mouse movement
                input_report_abs(dev, INPUT_ABS_X, x, false);
                input_report_abs(dev, INPUT_ABS_Y, -y, true);
            }
            break;

        case SCROLL:
            if (abs_x > 0 || abs_y > 0) {
                data->scroll_delta_x += x;
                data->scroll_delta_y += y;

                int8_t x_scroll = 0;
                int8_t y_scroll = 0;

                if (abs(data->scroll_delta_x) >= CONFIG_PMW3360_SCROLL_ACCUMULATION_X) {
                    if (data->scroll_delta_x < 0) {
                        x_scroll = PMW3360_SCROLL_X_NEGATIVE;
                        data->scroll_delta_x += CONFIG_PMW3360_SCROLL_ACCUMULATION_X;
                    } else {
                        x_scroll = PMW3360_SCROLL_X_POSITIVE;
                        data->scroll_delta_x -= CONFIG_PMW3360_SCROLL_ACCUMULATION_X;
                    }
                }

                if (abs(data->scroll_delta_y) >= CONFIG_PMW3360_SCROLL_ACCUMULATION_Y) {
                    if (data->scroll_delta_y < 0) {
                        y_scroll = PMW3360_SCROLL_Y_NEGATIVE;
                        data->scroll_delta_y += CONFIG_PMW3360_SCROLL_ACCUMULATION_Y;
                    } else {
                        y_scroll = PMW3360_SCROLL_Y_POSITIVE;
                        data->scroll_delta_y -= CONFIG_PMW3360_SCROLL_ACCUMULATION_Y;
                    }
                }

                if (x_scroll != 0 || y_scroll != 0) {
                    input_report_rel(dev, INPUT_REL_HWHEEL, x_scroll, false);
                    input_report_rel(dev, INPUT_REL_WHEEL, y_scroll, true);
                }
            }
            break;

        case SNIPE:
            if (abs_x > 0 || abs_y > 0) {
                // slow mouse movement for snipe
                input_report_abs(dev, INPUT_ABS_X, x, false);
                input_report_abs(dev, INPUT_ABS_Y, -y, true);
            }
            break;
        }
    }

    return err;
}

static void pmw3360_gpio_callback(const struct device *gpiob, struct gpio_callback *cb,
                                  uint32_t pins) {
    struct pixart_data *data = CONTAINER_OF(cb, struct pixart_data, irq_gpio_cb);
    const struct device *dev = data->dev;

    /* Disable interrupt until poll rate expires */
    set_interrupt(dev, false);

    /* Schedule report data */
    k_work_submit(&data->trigger_work);
}

static void pmw3360_work_callback(struct k_work *work) {
    struct pixart_data *data = CONTAINER_OF(work, struct pixart_data, trigger_work);
    const struct device *dev = data->dev;

    pmw3360_report_data(dev);
    set_interrupt(dev, true);
}

static int pmw3360_init_irq(const struct device *dev) {
    int err;
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;

    /* Get the IRQ GPIO device */
    if (!gpio_is_ready_dt(&config->irq_gpio)) {
        LOG_ERR("%s: IRQ GPIO device not ready", config->irq_gpio.port->name);
        return -ENODEV;
    }

    /* Configure IRQ GPIO */
    err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
    if (err) {
        LOG_ERR("Cannot configure IRQ GPIO: %d", err);
        return err;
    }

    gpio_init_callback(&data->irq_gpio_cb, pmw3360_gpio_callback, BIT(config->irq_gpio.pin));

    err = gpio_add_callback(config->irq_gpio.port, &data->irq_gpio_cb);
    if (err) {
        LOG_ERR("Cannot add IRQ GPIO callback");
        return err;
    }

    /* Initialize work */
    k_work_init(&data->trigger_work, pmw3360_work_callback);

    return 0;
}

static int pmw3360_init(const struct device *dev) {
    int err;
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;

    /* Store device data for easy access in callbacks */
    data->dev = dev;

    /* Initialize SPI bus */
    if (!spi_is_ready_dt(&config->bus)) {
        LOG_ERR("SPI bus is not ready");
        return -ENODEV;
    }

    /* Configure CS GPIO */
    if (!gpio_is_ready_dt(&config->cs_gpio)) {
        LOG_ERR("CS GPIO device not ready");
        return -ENODEV;
    }

    err = gpio_pin_configure_dt(&config->cs_gpio, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Cannot configure CS GPIO: %d", err);
        return err;
    }

    /* Initialize IRQ */
    err = pmw3360_init_irq(dev);
    if (err) {
        LOG_ERR("Cannot initialize IRQ: %d", err);
        return err;
    }

    /* Initialize automouse layer timer */
    if (IS_ENABLED(CONFIG_ZMK_MOUSE_LAYER_AUTO_MOUSE_LAYER)) {
        k_timer_init(&automouse_layer_timer, deactivate_automouse_layer, NULL);
    }

    /* Prepare for async initialization */
    k_work_init_delayable(&data->init_work, pmw3360_async_init);
    data->async_init_step = ASYNC_INIT_STEP_POWER_UP;
    data->err = 0;
    data->ready = false;

    /* Start async initialization */
    k_work_schedule(&data->init_work, K_NO_WAIT);

    return 0;
}

/* Make sure data is ready at driver initialization point */
DEVICE_DT_INST_DEFINE(0, pmw3360_init, NULL, &pmw3360_data, &pmw3360_config, POST_KERNEL,
                    CONFIG_INPUT_INIT_PRIORITY, NULL);

static struct pixart_data pmw3360_data;

/* bind to device tree */
static const int32_t scroll_layers[] = {DT_INST_PROP_OR(0, scroll_layers, {-1})};
static const int32_t snipe_layers[] = {DT_INST_PROP_OR(0, snipe_layers, {-1})};

PINCTRL_DT_INST_DEFINE(0);

static const struct pixart_config pmw3360_config = {
    .irq_gpio = GPIO_DT_SPEC_INST_GET(0, irq_gpios),
    .bus = SPI_DT_SPEC_INST_GET(0, SPI_WORD_SET(8), 0),
    .cs_gpio = GPIO_DT_SPEC_INST_GET(0, cs_gpios),
    .scroll_layers = scroll_layers,
    .scroll_layers_len = ARRAY_SIZE(scroll_layers),
    .snipe_layers = snipe_layers,
    .snipe_layers_len = ARRAY_SIZE(snipe_layers),
}; 