// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for Sherkaner's battery, INA219 connected to TalentCell battery.
 *
 * Copyright (C) 2022 Aaron Griffith <aargri@gmail.com>
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/delay.h>

#include "config.h"

#define INA2XX_CONFIG           0x00
#define INA2XX_SHUNT_VOLTAGE    0x01 /* readonly */
#define INA2XX_BUS_VOLTAGE      0x02 /* readonly */
#define INA2XX_POWER            0x03 /* readonly */
#define INA2XX_CURRENT          0x04 /* readonly */
#define INA2XX_CALIBRATION      0x05

#define INA2XX_MAX_DELAY        69   /* ms */

enum ina2xx_ids { ina219, ina226 };

struct sherkbat_table_entry {
    int capacity;
    int voltage;
    int charge;
    int energy;
};

#include "table.h"

struct ina2xx_config {
    u16 config_default;
    int calibration_value;
    int shunt_div;
    int bus_voltage_shift;
    int bus_voltage_lsb;
    int power_lsb_factor;
};

static const struct ina2xx_config ina2xx_config[] = {
    [ina219] = {
        .config_default = 0x399F, /* PGA=8 */
        .calibration_value = 4096,
        .shunt_div = 100,
        .bus_voltage_shift = 3,
        .bus_voltage_lsb = 4000,
        .power_lsb_factor = 20,
    },
    [ina226] = {
        .config_default = 0x4527, /* averages=16 */
        .calibration_value = 2048,
        .shunt_div = 400,
        .bus_voltage_shift = 0,
        .bus_voltage_lsb = 1250,
        .power_lsb_factor = 25,
    },
};

struct sherkbat_state {
    const struct ina2xx_config *config;
    const struct sherkbat_table_entry *table;
    size_t table_len;
    struct i2c_client *client;
    struct regmap *regmap;
    struct power_supply *psy;

    struct mutex config_lock;
    long rshunt;
    long current_lsb_uA;
    long power_lsb_uW;
};

static int sherkbat_init(struct sherkbat_state *st) {
    int ret = regmap_write(st->regmap, INA2XX_CONFIG, st->config->config_default);
    if (ret < 0)
        return ret;

    return regmap_write(st->regmap, INA2XX_CALIBRATION, st->config->calibration_value);
}

static int sherkbat_read_raw(struct sherkbat_state *st, int reg, unsigned int* regval) {
    int ret, retry;

    for (retry = 5; retry; retry--) {
        ret = regmap_read(st->regmap, reg, regval);
        if (ret < 0)
            return ret;

        if (*regval == 0) {
            unsigned int cal;

            ret = regmap_read(st->regmap, INA2XX_CALIBRATION, &cal);
            if (ret < 0)
                return ret;

            if (cal == 0) {
                dev_warn(&st->client->dev, "Chip not calibrated, reinitializing\n");
                ret = sherkbat_init(st);
                if (ret < 0)
                    return ret;

                msleep(INA2XX_MAX_DELAY);
                continue;
            }
        }

        return 0;
    }

    dev_err(&st->client->dev, "Unable to reinitialize the chip\n");
    return -ENODEV;
}

static int sherkbat_read(struct sherkbat_state *st, u8 reg, int *val) {
    unsigned int regval;
    int ret;

    ret = sherkbat_read_raw(st, reg, &regval);
    if (ret < 0)
        return ret;

    switch (reg) {
    case INA2XX_SHUNT_VOLTAGE:
        *val = DIV_ROUND_CLOSEST((s16)regval, st->config->shunt_div);
        break;
    case INA2XX_BUS_VOLTAGE:
        *val = (regval >> st->config->bus_voltage_shift) * st->config->bus_voltage_lsb;
        *val = DIV_ROUND_CLOSEST(*val, 1000);
        break;
    case INA2XX_POWER:
        *val = regval * st->power_lsb_uW;
        break;
    case INA2XX_CURRENT:
        *val = (s16)regval * st->current_lsb_uA;
        *val = DIV_ROUND_CLOSEST(*val, 1000);
        break;
    case INA2XX_CALIBRATION:
        *val = regval;
        break;
    default:
        return -ENODEV;
    }

    return 0;
}

static ssize_t sherkbat_value_show(struct device *dev, struct device_attribute *da, char *buf) {
    struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
    struct sherkbat_state *st = dev_get_drvdata(dev);
    int val;

    int err = sherkbat_read(st, attr->index, &val);
    if (err < 0)
        return err;
    return sysfs_emit(buf, "%d\n", val);
}

static int sherkbat_set_shunt(struct sherkbat_state *st, long val) {
    // see drivers/hwmon/ina2xx.c
    unsigned int dividend = DIV_ROUND_CLOSEST(1000000000, st->config->shunt_div);
    if (val <= 0 || val > dividend)
        return -EINVAL;

    mutex_lock(&st->config_lock);
    st->rshunt = val;
    st->current_lsb_uA = DIV_ROUND_CLOSEST(dividend, val);
    st->power_lsb_uW = st->config->power_lsb_factor * st->current_lsb_uA;
    mutex_unlock(&st->config_lock);

    return 0;
}

static ssize_t sherkbat_shunt_show(struct device *dev, struct device_attribute *da, char *buf) {
    struct sherkbat_state *st = dev_get_drvdata(dev);
    return sysfs_emit(buf, "%li\n", st->rshunt);
}

static ssize_t sherkbat_shunt_store(struct device *dev, struct device_attribute *da, const char *buf, size_t count) {
    unsigned long val;
    int status;
    struct sherkbat_state *st = dev_get_drvdata(dev);

    status = kstrtoul(buf, 10, &val);
    if (status < 0)
        return status;

    status = sherkbat_set_shunt(st, val);
    if (status < 0)
        return status;

    return count;
}

#define SHERKBAT_READ_MEMBER(ty, s, offset) (*(ty*)((u8*)(s) + offset))
#define sherkbat_bisect(st, member, val) sherkbat_bisect_offset((st), offsetof(struct sherkbat_table_entry, member), (val))

static int sherkbat_bisect_offset(struct sherkbat_state *st, size_t offset, int *val) {
    int ret;
    int bus;
    size_t lo, hi, mid;
    int dX, dV;
    struct sherkbat_table_entry uncalibrated = {
        .capacity = 500,
        .voltage = 0,
        .charge = 0,
        .energy = 0,
    };

    ret = sherkbat_read(st, INA2XX_BUS_VOLTAGE, &bus);
    if (ret < 0)
        return ret;
    bus *= 1000;

    if (st->table_len == 0) {
        /* built without a calibration table, just wing it until we get one */
        *val = SHERKBAT_READ_MEMBER(int, &uncalibrated, offset);
        return 0;
    }

    if (bus >= st->table[0].voltage) {
        *val = SHERKBAT_READ_MEMBER(int, &st->table[0], offset);
        return 0;
    }

    if (bus <= st->table[st->table_len - 1].voltage) {
        *val = SHERKBAT_READ_MEMBER(int, &st->table[st->table_len - 1], offset);
        return 0;
    }

    /* ok, we do actually need to bisect */
    lo = 0;
    hi = st->table_len - 1;
    /* maintain lo.voltage >= bus && bus > hi.voltage
       remember table is in descending order of voltage */
    while (hi - lo > 1) {
        mid = (lo + hi) / 2;
        if (bus > st->table[mid].voltage) {
            hi = mid;
        } else {
            lo = mid;
        }
    }

    /* linear interpolation */
    dX = SHERKBAT_READ_MEMBER(int, &st->table[hi], offset) - SHERKBAT_READ_MEMBER(int, &st->table[lo], offset);
    dV = st->table[hi].voltage - st->table[lo].voltage;

    *val = SHERKBAT_READ_MEMBER(int, &st->table[lo], offset) + DIV_ROUND_CLOSEST((bus - st->table[lo].voltage) * dX, dV);

    return 0;
}

static int sherkbat_get_property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val) {
    int ret;
    struct sherkbat_state *st = power_supply_get_drvdata(psy);

    switch (psp) {
    case POWER_SUPPLY_PROP_STATUS:
        /* FIXME NOT_CHARGING, CHARGING, FULL, DISCHARGING, UNKNOWN */
        val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
        return 0;
    case POWER_SUPPLY_PROP_PRESENT:
        val->intval = 1;
        return 0;
    case POWER_SUPPLY_PROP_ONLINE:
        val->intval = 1;
        return 0;
    case POWER_SUPPLY_PROP_TECHNOLOGY:
        val->intval = SHERKBAT_TECHNOLOGY;
        return 0;
    case POWER_SUPPLY_PROP_VOLTAGE_MIN:
    case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
        val->intval = SHERKBAT_VOLTAGE_MIN;
        return 0;
    case POWER_SUPPLY_PROP_VOLTAGE_MAX:
    case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
        val->intval = SHERKBAT_VOLTAGE_MAX;
        return 0;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        ret = sherkbat_read(st, INA2XX_BUS_VOLTAGE, &val->intval);
        if (ret < 0)
            return ret;
        val->intval *= 1000; // mV to uV
        return 0;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        ret = sherkbat_read(st, INA2XX_CURRENT, &val->intval);
        if (ret < 0)
            return ret;
        val->intval *= 1000; // mA to uA
        return 0;
    case POWER_SUPPLY_PROP_POWER_NOW:
        return sherkbat_read(st, INA2XX_POWER, &val->intval);
    case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
    case POWER_SUPPLY_PROP_CHARGE_FULL:
        val->intval = SHERKBAT_CHARGE_FULL;
        return 0;
    case POWER_SUPPLY_PROP_CHARGE_EMPTY_DESIGN:
    case POWER_SUPPLY_PROP_CHARGE_EMPTY:
        val->intval = 0;
        return 0;
    case POWER_SUPPLY_PROP_CHARGE_NOW:
        return sherkbat_bisect(st, charge, &val->intval);
    case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
    case POWER_SUPPLY_PROP_ENERGY_FULL:
        val->intval = SHERKBAT_ENERGY_FULL;
        return 0;
    case POWER_SUPPLY_PROP_ENERGY_EMPTY_DESIGN:
    case POWER_SUPPLY_PROP_ENERGY_EMPTY:
        val->intval = 0;
        return 0;
    case POWER_SUPPLY_PROP_ENERGY_NOW:
        return sherkbat_bisect(st, energy, &val->intval);
    case POWER_SUPPLY_PROP_CAPACITY:
        ret = sherkbat_bisect(st, capacity, &val->intval);
        if (ret < 0)
            return ret;
        /* careful, capacity is in 0.1% but kernel wants 1% */
        val->intval = DIV_ROUND_CLOSEST(val->intval, 10);
        return 0;
    case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
        /* FIXME set based on level */
        val->intval = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
        return 0;
    case POWER_SUPPLY_PROP_SCOPE:
        val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
        return 0;
    case POWER_SUPPLY_PROP_MODEL_NAME:
        val->strval = SHERKBAT_MODEL_NAME;
        return 0;
    case POWER_SUPPLY_PROP_MANUFACTURER:
        val->strval = SHERKBAT_MANUFACTURER;
        return 0;
    case POWER_SUPPLY_PROP_SERIAL_NUMBER:
        val->strval = SHERKBAT_SERIAL_NUMBER;
        return 0;
    default:
        return -EINVAL;
    }

    return 0;
}

static int sherkbat_set_property(struct power_supply *psy, enum power_supply_property psp, const union power_supply_propval *val) {
    return -EINVAL;
}

static int sherkbat_property_is_writeable(struct power_supply *psy, enum power_supply_property psp) {
    return 0;
}

static enum power_supply_property sherkbat_properties[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_TECHNOLOGY,
    POWER_SUPPLY_PROP_VOLTAGE_MAX,
    POWER_SUPPLY_PROP_VOLTAGE_MIN,
    POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
    POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_POWER_NOW,
    POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
    POWER_SUPPLY_PROP_CHARGE_EMPTY_DESIGN,
    POWER_SUPPLY_PROP_CHARGE_FULL,
    POWER_SUPPLY_PROP_CHARGE_EMPTY,
    POWER_SUPPLY_PROP_CHARGE_NOW,
    POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
    POWER_SUPPLY_PROP_ENERGY_EMPTY_DESIGN,
    POWER_SUPPLY_PROP_ENERGY_FULL,
    POWER_SUPPLY_PROP_ENERGY_EMPTY,
    POWER_SUPPLY_PROP_ENERGY_NOW,
    POWER_SUPPLY_PROP_CAPACITY,
    POWER_SUPPLY_PROP_CAPACITY_LEVEL,
    POWER_SUPPLY_PROP_SCOPE,
    POWER_SUPPLY_PROP_MODEL_NAME,
    POWER_SUPPLY_PROP_MANUFACTURER,
    POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

static const struct power_supply_desc sherkbat_desc = {
    .name = "sherkbat",
    .type = POWER_SUPPLY_TYPE_BATTERY,
    .get_property = sherkbat_get_property,
    .set_property = sherkbat_set_property,
    .property_is_writeable = sherkbat_property_is_writeable,
    .properties = sherkbat_properties,
    .num_properties = ARRAY_SIZE(sherkbat_properties),
};

static const struct regmap_config sherkbat_regmap_config = {
    .reg_bits = 8,
    .val_bits = 16,
};

static SENSOR_DEVICE_ATTR_RO(in0_input, sherkbat_value, INA2XX_SHUNT_VOLTAGE);
static SENSOR_DEVICE_ATTR_RO(in1_input, sherkbat_value, INA2XX_BUS_VOLTAGE);
static SENSOR_DEVICE_ATTR_RO(curr1_input, sherkbat_value, INA2XX_CURRENT);
static SENSOR_DEVICE_ATTR_RO(power1_input, sherkbat_value, INA2XX_POWER);
static SENSOR_DEVICE_ATTR_RW(shunt_resistor, sherkbat_shunt, INA2XX_CALIBRATION);

static struct attribute *sherkbat_attributes[] = {
    &sensor_dev_attr_in0_input.dev_attr.attr,
    &sensor_dev_attr_in1_input.dev_attr.attr,
    &sensor_dev_attr_curr1_input.dev_attr.attr,
    &sensor_dev_attr_power1_input.dev_attr.attr,
    &sensor_dev_attr_shunt_resistor.dev_attr.attr,
    NULL,
};

static const struct attribute_group sherkbat_attr_group = {
    .attrs = sherkbat_attributes,
};

static const struct attribute_group *sherkbat_attr_groups[] = {
    &sherkbat_attr_group,
    NULL,
};

static int sherkbat_probe(struct i2c_client *client) {
    struct power_supply_config cfg = {};
    struct sherkbat_state *st = NULL;
    struct device *hwmon = NULL;
    int ret = 0;

    st = devm_kzalloc(&client->dev, sizeof(*st), GFP_KERNEL);
    if (!st)
        return -ENOMEM;

    st->config = &ina2xx_config[ina219];
    st->table = sherkbat_table;
    st->table_len = ARRAY_SIZE(sherkbat_table);
    mutex_init(&st->config_lock);
    sherkbat_set_shunt(st, SHERKBAT_SHUNT); /* FIXME read config */

    st->client = client;
    st->regmap = devm_regmap_init_i2c(client, &sherkbat_regmap_config);
    if (IS_ERR(st->regmap)) {
        dev_err(&client->dev, "Failed to initialize register map\n");
        return -EINVAL;
    }

    i2c_set_clientdata(client, st);
    cfg.drv_data = st;

    st->psy = devm_power_supply_register(&client->dev, &sherkbat_desc, &cfg);
    if (IS_ERR(st->psy)) {
        dev_err(&client->dev, "Failed to register power supply\n");
        return PTR_ERR(st->psy);
    }

    ret = sherkbat_init(st);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to configure device\n");
        return -ENODEV;
    }

    hwmon = devm_hwmon_device_register_with_groups(&client->dev, client->name, st, sherkbat_attr_groups);
    if (IS_ERR(hwmon)) {
        dev_err(&client->dev, "Failed to initialize hwmon\n");
        return PTR_ERR(hwmon);
    }

    dev_info(&client->dev, "sherkaner battery %s", client->name);

    return 0;
}

static const struct i2c_device_id sherkbat_id[] = {
    {"sherkbat", 0},
    { }
};
MODULE_DEVICE_TABLE(i2c, sherkbat_id);

static struct i2c_driver sherkbat_driver = {
    .driver = {
        .name = KBUILD_MODNAME,
    },
    .probe_new = sherkbat_probe,
    .id_table = sherkbat_id,
};

module_i2c_driver(sherkbat_driver);

MODULE_AUTHOR("Aaron Griffith <aargri@gmail.com>");
MODULE_DESCRIPTION("sherkaner battery driver");
MODULE_LICENSE("GPL");
