/*
 * PAC1934 I2C ADC driver
 */

#include <asm/ioctl.h>
#include <linux/cdev.h>
#include <linux/completion.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/mutex.h>

#define REFRESH_V 0x1f

#define VBUS1 0x07
#define VBUS2 0x08
#define VBUS3 0x09
#define VBUS4 0x0a

#define VSENSE1 0x0b
#define VSENSE2 0x0c
#define VSENSE3 0x0d
#define VSENSE4 0x0e

#define VPOWER1 0x17
#define VPOWER2 0x18
#define VPOWER3 0x19
#define VPOWER4 0x1a

typedef void (*data_processor)(u32, int *, int *);

struct pac1934 {
	struct i2c_client *client;
	u32 vbus[4];
	u32 vsense[4];
	u32 vpower[4];
	u16 sample_rate;
	struct mutex lock;
	struct task_struct *kthread;
};

/* Getting data by i2c_smbus* in renode takes very long time */
/* Additionally only one thread is running, so running another thread periodically */
/* kills the system, so on testing on renode please keep FETCH_DATA_IN_THREADS undefined */

// #define FETCH_DATA_IN_THREADS

static void update_data(struct pac1934 *state)
{
	u16 val;
	int i, j;
	u8 bits[16];
	i2c_smbus_write_byte(state->client, REFRESH_V);
	/* wait for data to be ready for reading */
	usleep_range(1000, 1100);
	for (i = 0; i < 4; i++) {
		val = i2c_smbus_read_word_data(state->client, VBUS1 + i);
		mutex_lock(&state->lock);
		state->vbus[i] = val;
		mutex_unlock(&state->lock);
	}
	for (i = 0; i < 4; i++) {
		val = i2c_smbus_read_word_data(state->client, VSENSE1 + i);
		mutex_lock(&state->lock);
		state->vsense[i] = val;
		mutex_unlock(&state->lock);
	}
	for (i = 0; i < 4; i++) {
		val = i2c_smbus_read_i2c_block_data(state->client, VPOWER1 + i,
						    4, bits);
		mutex_lock(&state->lock);
		state->vpower[i] = 0;
		for (j = 0; j < 4; j++) {
			state->vpower[i] |= (u32)bits[3 - j] << (24 - 8 * j);
		}
		mutex_unlock(&state->lock);
	}

}

static u32 *chann_type_to_buff(long chan_type, struct pac1934 *state)
{
	switch (chan_type) {
	case IIO_VOLTAGE:
		return state->vbus;
	case IIO_CURRENT:
		return state->vsense;
	case IIO_POWER:
		return state->vpower;
	default:
		return NULL;
	}
}

static void process_voltage(u32 raw_val, int *val, int *val2)
{
	*val = 32 * raw_val;
	*val2 = 16;
}

static void process_current(u32 raw_val, int *val, int *val2)
{
	*val = 2500 * raw_val;
	*val2 = 16;
}

static void process_power(u32 raw_val, int *val, int *val2)
{
	*val = 800 * raw_val;
	*val2 = 28;
}

static data_processor chann_type_to_processor(long chan_type)
{
	switch (chan_type) {
	case IIO_VOLTAGE:
		return process_voltage;
	case IIO_CURRENT:
		return process_current;
	case IIO_POWER:
		return process_power;
	default:
		return NULL;
	}
}

static int set_data_raw(struct iio_chan_spec const *chan, struct pac1934 *state,
			int *val)
{
	u32 *buf = chann_type_to_buff(chan->type, state);
	if (buf == NULL) {
	        return -EINVAL;
	}

	mutex_lock(&state->lock);
	*val = buf[chan->channel];
	mutex_unlock(&state->lock);
	return IIO_VAL_INT;
}

static int set_data_processed(struct iio_chan_spec const *chan,
			      struct pac1934 *state, int *val, int *val2)
{
	u16 tmp;
	data_processor proc;
	u32 *buf = chann_type_to_buff(chan->type, state);
	mutex_lock(&state->lock);
	tmp = buf[chan->channel];
	mutex_unlock(&state->lock);
	proc = chann_type_to_processor(chan->type);
	proc(tmp, val, val2);
	return IIO_VAL_FRACTIONAL_LOG2;
}

int read_i2c(void *data)
{
	struct pac1934 *state = data;
	while (!kthread_should_stop()) {
		update_data(state);
	}

	return 0;
}

static int pac1934_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int *val,
			    int *val2, long mask)
{
	struct pac1934 *state = iio_priv(indio_dev);

#ifndef FETCH_DATA_IN_THREADS
	update_data(state);
#endif

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		return set_data_raw(chan, state, val);
	case IIO_CHAN_INFO_PROCESSED:
		return set_data_processed(chan, state, val, val2);
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = state->sample_rate;
		return IIO_VAL_INT;
	}
	return -EINVAL;
}

int is_sample_rate_aviable(u16 val)
{
	return (val == 1024) || (val == 256) || (val == 8);
}

static int pac1934_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, int val,
			     int val2, long mask)
{
	struct pac1934 *state = iio_priv(indio_dev);
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!is_sample_rate_aviable((u16)val))
			return -EINVAL;
		state->sample_rate = val;
		return 0;
	}
	return -EINVAL;
}

#define PAC1934_VBUS_CHANNEL(chan_type, chan_num)                              \
	{                                                                      \
		.type = (chan_type), .indexed = 1, .channel = (chan_num),      \
		.info_mask_separate =                                          \
			BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_PROCESSED), \
		.info_mask_shared_by_dir = BIT(IIO_CHAN_INFO_SAMP_FREQ),       \
	}

#define PAC1934_CHANNEL_SET(chan_type)                                         \
	PAC1934_VBUS_CHANNEL((chan_type), 0),                                  \
		PAC1934_VBUS_CHANNEL((chan_type), 1),                          \
		PAC1934_VBUS_CHANNEL((chan_type), 2),                          \
		PAC1934_VBUS_CHANNEL((chan_type), 3)

static const struct iio_chan_spec pac1934_channels[] = {
	PAC1934_CHANNEL_SET(IIO_VOLTAGE),
	PAC1934_CHANNEL_SET(IIO_CURRENT),
	PAC1934_CHANNEL_SET(IIO_POWER),
};

static const struct iio_info pac1934_info = {
	.read_raw = pac1934_read_raw,
	.write_raw = pac1934_write_raw,
};

static struct task_struct *kthread;

static int pac1934_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct pac1934 *data;
	struct iio_dev *indio_dev;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->client = client;
	data->sample_rate = 1024;
	mutex_init(&data->lock);
	kthread = data->kthread;
	i2c_set_clientdata(client, indio_dev);

	indio_dev->name = dev_name(&client->dev);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &pac1934_info;
	indio_dev->channels = pac1934_channels;
	indio_dev->num_channels = ARRAY_SIZE(pac1934_channels);

#ifdef FETCH_DATA_IN_THREADS
	data->kthread = kthread_run(read_i2c, data, "read_i2c_thread");
#endif

	return iio_device_register(indio_dev);
}

static int pac1934_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct pac1934 *state = iio_priv(indio_dev);

#ifdef FETCH_DATA_IN_THREADS
	kthread_stop(state->kthread);
#endif
	iio_device_unregister(indio_dev);

	return 0;
}

static const struct of_device_id pac1934_dt_ids[] = {
	{ .compatible = "microchip,pac1934" },
	{}
};

static struct i2c_driver pac1934_driver = {
    .driver =
        {
            .name = "pac1934",
            .of_match_table = pac1934_dt_ids,
        },
    .probe = pac1934_probe,
    .remove = pac1934_remove,
};

module_i2c_driver(pac1934_driver);

MODULE_LICENSE("GPL");
