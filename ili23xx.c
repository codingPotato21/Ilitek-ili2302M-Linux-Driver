// SPDX-License-Identifier: GPL-2.0-only
/*
 * Simple ILITEK Touch IC driver for older kernels, ex: 5.10.160.
 *
 * Author: codingPotato <https://www.github.com/codingpotato21>
 *
 * Based on: ili210x.c & ilitek_ts_i2c.c
 * Authors:
 * Olivier Sobrie <olivier@sobrie.be>
 * Copyright (C) 2011 ILI Technology Corporation.
 * Copyright (C) 2020 Luca Hsu <luca_hsu@ilitek.com>
 * Copyright (C) 2021 Joe Hung <joe_hung@ilitek.com>
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <asm/unaligned.h>

/* General Values */
#define ILI_POLL_PERIOD     20
#define ILI_DATA_SIZE       64
#define ILI_SCRN_RES_SIZE   8
#define MAX_TOUCHES         5

/* Touchscreen Registers */
#define REG_TOUCHDATA       0x10
#define REG_SCRN_RES        0x21

static const struct ilitek_ts_data {
    struct i2c_client *client;
    struct input_dev *input;
    struct gpio_desc *reset_gpio;
    struct touchscreen_properties prop;
    bool stop;
    int min_x;
    int max_x;
    int min_y;
    int max_y;
} ilitek_ts_data;

static int ilitek_read_reg(struct i2c_client *client, u8 reg, void *buf, size_t len)
{
    char buffer_str[256]; // A bit extra space added
    int buffer_str_index = 0, i = 0;

    struct i2c_msg msg[] = {
        {
            .addr   = client->addr,
            .flags  = 0,
            .len    = 1,
            .buf    = &reg,
        },
        {
            .addr   = client->addr,
            .flags  = I2C_M_RD,
            .len    = len,
            .buf    = buf,
        }
    };
    int error, ret;

    ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
    if (ret != ARRAY_SIZE(msg)) {
        error = ret < 0 ? ret : -EIO;
        dev_err(&client->dev, "%s failed: %d\n", __func__, error);
        return error;
    }

    // Print the reply buffer
    dev_dbg(&client->dev, "Received Touchscreen Buffer:");

    for (i = 0; i < len; i++) {
        buffer_str_index += snprintf(buffer_str + buffer_str_index, sizeof(buffer_str) - buffer_str_index, "%02x ", *((u8 *)buf + i));
    }
    dev_dbg(&client->dev, "%s\n", buffer_str);

    return 0;
}

static bool ilitek_report_events(struct ilitek_ts_data *priv, u8 *touchdata)
{
    struct i2c_client *client = priv->client;
    struct input_dev *input = priv->input;
    unsigned int x = 0, y = 0, status = 0;
    bool contact = false, touch = false;
    int packet_len = 5, i = 0;

    // Loop through the data for each finger aka multi touch
    for (i = 0; i < MAX_TOUCHES; i++) {
        status = get_unaligned_be16(touchdata + (i * packet_len) + 1);
        if (!(status & BIT(15))) {
            touch = false;
        } else {
            touch = true;
            x = status & 0x3fff;
            y = get_unaligned_be16(touchdata + (i * packet_len) + 3);

            // Print touch data for debugging
            dev_dbg(&client->dev, "Finger %d, X: %d, Y: %d\n", i, x, y);
        }

        input_mt_slot(input, i);
        if (input_mt_report_slot_state(input, MT_TOOL_FINGER, touch)) {
            touchscreen_report_pos(input, &priv->prop, x, y, true);
            contact = true;
        }
    }

    input_mt_report_pointer_emulation(input, false);
    input_sync(input);

    return contact;
}

static irqreturn_t ilitek_irq(int irq, void *irq_data)
{
    struct ilitek_ts_data *priv = irq_data;
    struct i2c_client *client = priv->client;
    u8 touchdata[ILI_DATA_SIZE];

    int error = 0;
    bool keep_polling = false;

    do {
        // Read the first size of the data
        error = ilitek_read_reg(client, REG_TOUCHDATA, touchdata, ILI_DATA_SIZE);
        if (error) {
            dev_err(&client->dev, "Unable to get touch data: %d\n", error);
            break;
        }
        
        // Update touch
        keep_polling = ilitek_report_events(priv, touchdata);
        if (keep_polling) msleep(ILI_POLL_PERIOD);
    } while (!priv->stop && keep_polling);

    return IRQ_HANDLED;
}

static int ilitek_get_scrn_res(struct ilitek_ts_data *priv)
{
    int error;
    u8 outbuf[ILI_SCRN_RES_SIZE];
    struct i2c_client *client = priv->client;
    struct device *dev = &client->dev;

    error = ilitek_read_reg(client, REG_SCRN_RES, outbuf, ILI_SCRN_RES_SIZE);
    if (error) return error;

    priv->min_x = get_unaligned_le16(outbuf);
    priv->min_y = get_unaligned_le16(outbuf + 2);
    priv->max_x = get_unaligned_le16(outbuf + 4);
    priv->max_y = get_unaligned_le16(outbuf + 6);

    // Print parsed panel props
    dev_dbg(dev, "Determined Touchscreen Resolution\n");
    dev_dbg(dev, "min_x: %d\n", priv->min_x);
    dev_dbg(dev, "min_y: %d\n", priv->min_y);
    dev_dbg(dev, "max_x: %d\n", priv->max_x);
    dev_dbg(dev, "max_y: %d\n", priv->max_y);

    return 0;
}

static void ilitek_power_down(void *data)
{
    struct gpio_desc *reset_gpio = data;

    gpiod_set_value_cansleep(reset_gpio, 1);
}

static void ilitek_stop(void *data)
{
    struct ilitek_ts_data *priv = data;

    /* Tell ISR to quit even if there is a contact. */
    priv->stop = true;
}

static int ilitek_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

    struct ilitek_ts_data *priv;
    struct device *dev = &client->dev;
    struct gpio_desc *reset_gpio;
    struct input_dev *input;
    int error;

    dev_dbg(dev, "Probing for ILITEK I2C Touschreen driver");

    priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) return -ENOMEM;

    if (client->irq <= 0) {
        dev_err(dev, "No IRQ!\n");
        return -EINVAL;
    }

    reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
    if (IS_ERR(reset_gpio))
        return PTR_ERR(reset_gpio);

    if (reset_gpio) {
        error = devm_add_action_or_reset(dev, ilitek_power_down, reset_gpio);
        if (error) return error;

        usleep_range(12000, 15000);
        gpiod_set_value_cansleep(reset_gpio, 0);
        msleep(160);
    }

    input = devm_input_allocate_device(dev);
    if (!input) return -ENOMEM;

    priv->client = client;
    priv->input = input;
    priv->reset_gpio = reset_gpio;
    i2c_set_clientdata(client, priv);

    /* Setup input device */
    input->name = "ILITEK Touchscreen";
    input->id.bustype = BUS_I2C;

    /* Touchscreen resolution */
    error = ilitek_get_scrn_res(priv);
    if (error) {
        dev_err(dev, "Unable to get touchscreen resolution, err: %d\n", error);
        dev_info(dev, "Using resolution of SZ_64K. Touch may be uncalibrated now!");
        dev_info(dev, "Manual override of the resolution in the DT is required! Read the documentation for information.");

        priv->min_x = 0;
        priv->min_y = 0;
        priv->max_x = SZ_64K;
        priv->max_y = SZ_64K;
    }

    /* Multi touch */
    input_set_abs_params(input, ABS_MT_POSITION_X, priv->min_x, priv->max_x, 0, 0);
    input_set_abs_params(input, ABS_MT_POSITION_Y, priv->min_y, priv->max_y, 0, 0);
    touchscreen_parse_properties(input, true, &priv->prop);

    error = input_mt_init_slots(input, MAX_TOUCHES, INPUT_MT_DIRECT);
    if (error) {
        dev_err(dev, "Unable to set up slots, err: %d\n", error);
        return error;
    }

    error = devm_request_threaded_irq(dev, client->irq, NULL, ilitek_irq, IRQF_ONESHOT, client->name, priv);
    if (error) {
        dev_err(dev, "Unable to request touchscreen IRQ, err: %d\n",
            error);
        return error;
    }

    error = devm_add_action_or_reset(dev, ilitek_stop, priv);
    if (error) return error;

    error = input_register_device(priv->input);
    if (error) {
        dev_err(dev, "Cannot register input device, err: %d\n", error);
        return error;
    }

    return 0;
}

static const struct i2c_device_id ilitek_i2c_id[] = {
    { "ili2302", (long)&ilitek_ts_data },
    { }
};
MODULE_DEVICE_TABLE(i2c, ilitek_i2c_id);

static const struct of_device_id ilitek_dt_ids[] = {
    { .compatible = "ilitek,ili2302", .data = &ilitek_ts_data },
    { }
};
MODULE_DEVICE_TABLE(of, ilitek_dt_ids);

static struct i2c_driver ilitek_ts_driver = {
    .driver = {
        .name = "ilitek_i2c",
        .of_match_table = ilitek_dt_ids,
    },
    .id_table = ilitek_i2c_id,
    .probe = ilitek_i2c_probe,
};

module_i2c_driver(ilitek_ts_driver);

MODULE_AUTHOR("codingPotato <https://www.github.com/codingpotato21>");
MODULE_DESCRIPTION("ILITEK I2C Touchscreen Driver");
MODULE_LICENSE("GPL");

