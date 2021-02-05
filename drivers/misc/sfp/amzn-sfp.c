/*
 * Copyright Amazon.com, Inc. and its affiliates. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/gpl-2.0.html>.
 *
 * Driver for SFP+, QSFP+, QSFP28 and QSFP-DD modules
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/delay.h>

#ifdef CONFIG_SYSCTL
#include <linux/sysctl.h>
#endif

#define	AMZN_SFP_TYPE_SFP_PLUS	1	/* See SFF-8431 & SFF-8472 */
#define	AMZN_SFP_TYPE_QSFP_PLUS	2	/* See SFF-8679 */
#define	AMZN_SFP_TYPE_QSFP28	3	/* See SFF-8661 */
#define	AMZN_SFP_TYPE_QSFP_DD	4	/* http://www.qsfp-dd.com/ */

/*
 * Two key EEPROM sizes:
 * FULL: the maximum given an 8-bit register address.
 * HALF: the division into a lower and an upper half.
 */
#define	AMZN_SFP_FULL_SIZE	256
#define	AMZN_SFP_HALF_SIZE	(AMZN_SFP_FULL_SIZE >> 1)

/* Page select register for QSFP+, QSFP28 and QSFP-DD modules. */
#define	AMZN_QSFP_PAGE_SELECT	127


struct amzn_sfp_softc {
	struct bin_attribute	attr;
	struct i2c_client	*client;
	struct rt_mutex		lock;
	int			sfp_type;
	int			cur_page;
	unsigned long		cur_page_ts;
};

/*
 * The default retention time in seconds of the cur_page variable.
 * By default this is 1 second.
 */
static int amzn_sfp_page_retention = 1;

/*
 * Writing upper page select and reading downstream page causes modules to hang
 */
static int amzn_sfp_page_load_wait_ms = 4;

#ifdef CONFIG_SYSCTL
static struct ctl_table amzn_sfp_sysctls[] = {
    {
	.procname = "amzn-sfp-page-retention",
	.data = &amzn_sfp_page_retention,
	.maxlen = sizeof(amzn_sfp_page_retention),
	.mode = 0644,
	.proc_handler = proc_dointvec,
    },
    {
	.procname = "amzn-sfp-page-load-wait-ms",
	.data = &amzn_sfp_page_load_wait_ms,
	.maxlen = sizeof(amzn_sfp_page_load_wait_ms),
	.mode = 0644,
	.proc_handler = proc_dointvec,
    },
    {
    }
};
#endif /* CONFIG_SYSCTL */


static ssize_t amzn_sfp_rw(struct bin_attribute *ba, char *buf, loff_t ofs,
    size_t len, u16 flags)
{
	struct amzn_sfp_softc *sc = ba->private;
	struct i2c_client *client = sc->client;
	char iobuf[AMZN_SFP_HALF_SIZE + 1];
	struct i2c_msg msg[2];
	unsigned long ts;
	int error, nmsgs;
	u16 addr;
	u8 reg;

	/* Make sure the offset and length are valid. */
	if (ofs < 0 || ofs >= ba->size)
		return -ESPIPE;
	if (len == 0)
		return -EINVAL;
	if (ofs + len > ba->size)
		return -ENOSPC;

	addr = client->addr;

	rt_mutex_lock(&sc->lock);

	switch (sc->sfp_type) {
	case AMZN_SFP_TYPE_SFP_PLUS:
		/*
		 * Handle I2C address auto-increment for DOM access.
		 * Never cross-over between different I2C addresses!
		 */
		addr += ofs / AMZN_SFP_FULL_SIZE;
		reg = ofs % AMZN_SFP_FULL_SIZE;
		if (len + reg > AMZN_SFP_FULL_SIZE)
			len = AMZN_SFP_FULL_SIZE - reg;
		break;
	case AMZN_SFP_TYPE_QSFP_PLUS:
	case AMZN_SFP_TYPE_QSFP28:
	case AMZN_SFP_TYPE_QSFP_DD:
		/*
		 * Handle paging of the upper half.  Never cross-over
		 * into different pages or from lower into upper half.
		 * The lower half starts at offset 0. Page 0 of the
		 * upper half starts at offset 128. Upper half page 1
		 * starts at offset 256.  Upper half page X starts at
		 * offset 128 + X * 128.
		 */
		if (ofs < AMZN_SFP_HALF_SIZE) {
			/* Lower half: don't cross-over into upper half. */
			reg = ofs;
			if (len + reg > AMZN_SFP_HALF_SIZE)
				len = AMZN_SFP_HALF_SIZE - reg;
			break;
		}

		/* Upper half */

		/* Prepare the buffer for writing page select. */
		iobuf[0] = AMZN_QSFP_PAGE_SELECT;
		iobuf[1] = (ofs / AMZN_SFP_HALF_SIZE) - 1;

		/* Calculate the offset on the page. */
		reg = (ofs % AMZN_SFP_HALF_SIZE) + AMZN_SFP_HALF_SIZE;

		/* Stay within the page size (which starts at 128). */
		if (len + reg > AMZN_SFP_FULL_SIZE)
			len = AMZN_SFP_FULL_SIZE - reg;

		/*
		 * Ok, so now we have to be careful.  We can't just
		 * blindly write the page select register.  DACs in
		 * particular barf on that (NAK the data write).
		 * This even applies to writing a value 0 to select
		 * page 0.  As such, we should only write to the
		 * page select register when necessary.
		 * So, what we need to do is read the current value
		 * of the page select register and only select the
		 * page if the current page is different from the
		 * desired page.  But this would add overhead for
		 * each and every upper page access.
		 * To prevent the extra overhead, we cache the page
		 * in memory and give it a 1 second retention.  As
		 * such, we only read the page select register once
		 * a second to determine the active upper page.
		 * The retention is needed because modules are hot-
		 * pluggable.  We don't have the knowledge in this
		 * driver to know if the module we're talking to
		 * hasn't been replaced.
		 */
		ts = sc->cur_page_ts + amzn_sfp_page_retention * HZ;
		if (sc->cur_page == -1 ||
		    !time_in_range(jiffies, sc->cur_page_ts, ts)) {
			/*
			 * Read the page select register and update our
			 * notion of the current page.  Read the value
			 * in iobuf[2] to preserve the desired page in
			 * iobuf[1].
			 */
			nmsgs = 0;
			msg[nmsgs].addr = addr;
			msg[nmsgs].flags = 0;
			msg[nmsgs].len = 1;
			msg[nmsgs].buf = &iobuf[0];
			nmsgs++;
			msg[nmsgs].addr = addr;
			msg[nmsgs].flags = I2C_M_RD;
			msg[nmsgs].len = 1;
			msg[nmsgs].buf = &iobuf[2];
			nmsgs++;

			error = i2c_transfer(client->adapter, msg, nmsgs);
			if (error < 0) {
				/* Don't trust our state. */
				sc->cur_page = -1;
				rt_mutex_unlock(&sc->lock);
				return error;
			}

			if (iobuf[2] != sc->cur_page) {
				if (sc->cur_page != -1)
					dev_notice(&sc->client->dev,
					    "resetting current page to %u"
					    " (was %u)\n", iobuf[2],
					    sc->cur_page);
				sc->cur_page = iobuf[2];
			}
			sc->cur_page_ts = jiffies;
		}

		/*
		 * Don't write the page select register if the desired
		 * page is the same as the current page.
		 */
		if (iobuf[1] == sc->cur_page)
			break;

		/*
		 * Write the page select register.  We have a lock on
		 * the device, so we know the page can not be changed
		 * between now and when we access the page.
		 */
		nmsgs = 0;
		msg[nmsgs].addr = addr;
		msg[nmsgs].flags = 0;
		msg[nmsgs].len = 2;
		msg[nmsgs].buf = iobuf;
		nmsgs++;

		error = i2c_transfer(client->adapter, msg, nmsgs);
		if (error < 0) {
			/* Don't trust our state. */
			sc->cur_page = -1;
			rt_mutex_unlock(&sc->lock);
			return error;
		}

		sc->cur_page = iobuf[1];
		sc->cur_page_ts = jiffies;
		break;
	default:
		/*
		 * Unknown SFP type, we simply give access to the full
		 * EEPROM, without any knowledge about.  You get what
		 * you get...
		 */
		reg = ofs;
		break;
	}

	/*
	 * The SPI-I2C controller has a limited buffer
	 * size (96 bytes) and the driver simply returns EOPNOTSUPP when
	 * a larger transfer is requested.  Bad driver!
	 */
	if (len > 64)
		len = 64;

	nmsgs = 0;
	if (flags == I2C_M_RD) {
		msg[nmsgs].addr = addr;
		msg[nmsgs].flags = 0;
		msg[nmsgs].len = 1;
		msg[nmsgs].buf = &reg;
		nmsgs++;
		msg[nmsgs].addr = addr;
		msg[nmsgs].flags = flags;
		msg[nmsgs].len = len;
		msg[nmsgs].buf = buf;
		nmsgs++;
	} else {
		iobuf[0] = reg;
		memcpy(iobuf + 1, buf, len);
		msg[nmsgs].addr = addr;
		msg[nmsgs].flags = 0;
		msg[nmsgs].len = len + 1;
		msg[nmsgs].buf = iobuf;
		nmsgs++;
	}
	/* Writing to page select byte and immediate read to that same page 
	 * cause certain modules to hang. Wait 4 - 5ms for modules to load 
	 * upper page eeprom
	 */
	ts = sc->cur_page_ts + msecs_to_jiffies(amzn_sfp_page_load_wait_ms);
	if (time_in_range(jiffies, sc->cur_page_ts, ts)) {
		ts = amzn_sfp_page_load_wait_ms * 1000;
		usleep_range(ts, ts + 1000);
	}

	error = i2c_transfer(client->adapter, msg, nmsgs);

	rt_mutex_unlock(&sc->lock);

	if (error < 0)
		return error;
	if (error != nmsgs)
		return -EPIPE;
	return (ssize_t)len;
}

static ssize_t amzn_sfp_read(struct file *fp, struct kobject *kobj,
    struct bin_attribute *ba, char *buf, loff_t ofs, size_t len)
{
	ssize_t result;

	result = amzn_sfp_rw(ba, buf, ofs, len, I2C_M_RD);
	return result;
}

static ssize_t amzn_sfp_write(struct file *fp, struct kobject *kobj,
    struct bin_attribute *ba, char *buf, loff_t ofs, size_t len)
{
	ssize_t result;

	result = amzn_sfp_rw(ba, buf, ofs, len, 0);
	return result;
}

static int amzn_sfp_probe(struct i2c_client *client,
    const struct i2c_device_id *id)
{
	struct amzn_sfp_softc *sc;
	int error;

	/* Paranoia... */
	if (client == NULL || id == NULL)
		return -EINVAL;

	sc = devm_kzalloc(&client->dev, sizeof(*sc), GFP_KERNEL);
	if (sc == NULL)
		return -ENOMEM;

	sc->client = client;
	rt_mutex_init(&sc->lock);
	sc->sfp_type = id->driver_data;
	sc->cur_page = -1;	/* We don't know */
	i2c_set_clientdata(client, sc);

	sysfs_bin_attr_init(&sc->attr);
	sc->attr.attr.name = "eeprom";
	sc->attr.attr.mode = S_IWUSR | S_IRUGO;
	switch (sc->sfp_type) {
	case AMZN_SFP_TYPE_SFP_PLUS:
		/*
		 * SFP+ has DDI.  Which is a separate I2C EEPROM at address
		 * 0x51.  We expose this to user space as the second half
		 * of a double-sized EEPROM.  We use I2C address auto-
		 * increment to select the I2C device.
		 */
		sc->attr.size = 2 * AMZN_SFP_FULL_SIZE;
		break;
	case AMZN_SFP_TYPE_QSFP_PLUS:
	case AMZN_SFP_TYPE_QSFP28:
	case AMZN_SFP_TYPE_QSFP_DD:
		/*
		 * The EEPROM is divided into a lower and an upper half.
		 * The upper half is paged, with the 8-bit page number
		 * set in the lower half.  The paging is abstracted from
		 * user space by providing a single large EEPROM, that is
		 * the concatenation of all halves.  There are 257 halves:
		 * 1 lower half and 256 paged upper halves.
		 * The key objective is to guarantee atomicity of the page
		 * select and the register access on that page.
		 */
		sc->attr.size = 257 * AMZN_SFP_HALF_SIZE;
		break;
	default:
		dev_warn(&client->dev, "unknown SFP type %d; fix driver\n",
		    sc->sfp_type);
		sc->attr.size = AMZN_SFP_FULL_SIZE;
		break;
	}
	sc->attr.private = sc;
	sc->attr.read = amzn_sfp_read;
	sc->attr.write = amzn_sfp_write;
	error = sysfs_create_bin_file(&client->dev.kobj, &sc->attr);
	if (error)
		dev_err(&client->dev,
		    "unable to create 'eeprom' file in sysfs (error %d)\n",
		    error);

	return error;
}

static int amzn_sfp_remove(struct i2c_client *client)
{
	struct amzn_sfp_softc *sc;

	/* Paranoia... */
	if (client == NULL)
		return -EINVAL;
	sc = i2c_get_clientdata(client);
	if (sc == NULL)
		return -ENODEV;

	i2c_set_clientdata(client, NULL);
	sysfs_remove_bin_file(&client->dev.kobj, &sc->attr);
	return 0;
}

static const struct i2c_device_id amzn_sfp_ids[] = {
	{ .name = "sfp+",	.driver_data = AMZN_SFP_TYPE_SFP_PLUS },
	{ .name = "qsfp+",	.driver_data = AMZN_SFP_TYPE_QSFP_PLUS },
	{ .name = "qsfp28",	.driver_data = AMZN_SFP_TYPE_QSFP28 },
	{ .name = "qsfp-dd",	.driver_data = AMZN_SFP_TYPE_QSFP_DD },
	{},
};
MODULE_DEVICE_TABLE(i2c, syspld_of_match);

static struct i2c_driver amzn_sfp_driver = {
	.driver = {
		.name = "amzn-sfp",
		.owner = THIS_MODULE,
	},
	.probe = amzn_sfp_probe,
	.remove = amzn_sfp_remove,
	.id_table = amzn_sfp_ids,
};

static int amzn_sfp_init(struct i2c_driver *drv)
{
	int error;

	error = i2c_add_driver(drv);
#ifdef CONFIG_SYSCTL
	register_sysctl("debug", amzn_sfp_sysctls);
#endif
	return (error);
}

static void amzn_sfp_exit(struct i2c_driver *drv)
{
	i2c_del_driver(drv);
}

module_driver(amzn_sfp_driver, amzn_sfp_init, amzn_sfp_exit);

MODULE_DESCRIPTION("Driver for SFP+, QSFP+, QSFP28 and QSFP-DD modules");
MODULE_AUTHOR("Marcel Moolenaar <mmoolena@amazon.com>");
MODULE_LICENSE("GPL");
