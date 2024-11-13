/*
 * Copyright (C) 2021-2022 Stonyx
 * https://www.stonyx.com/
 *
 * This driver is free software. You can redistribute it and/or modify it under the terms of the
 * GNU General Public License Version 3 (or at your option any later version) as published by The
 * Free Software Foundation.
 *
 * This driver is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
 * the GNU General Public License for more details.
 *
 * If you did not received a copy of the GNU General Public License along with this script see
 * http://www.gnu.org/copyleft/gpl.html or write to The Free Software Foundation, 675 Mass Ave,
 * Cambridge, MA 02139, USA.
 */

#include <linux/fs.h>
#include <linux/hwmon.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include "qnap-ec-ioctl.h"

// Define the pr_err prefix
#undef pr_fmt
#define pr_fmt(fmt) "%s @ %s: " fmt, "qnap-ec", __FUNCTION__

// Define module details
MODULE_DESCRIPTION("QNAP EC Driver");
MODULE_VERSION("1.1.2");
MODULE_AUTHOR("Stonyx - https://www.stonyx.com/");
MODULE_LICENSE("GPL");
MODULE_PARM_DESC(val_pwm_channels, "Validate PWM channels");
MODULE_PARM_DESC(sim_pwm_enable, "Simulate pwmX_enable sysfs attributes");
MODULE_PARM_DESC(check_for_chip, "Check for QNAP IT8528 E.C. chip");

// Define maximum number of possible channels
#define QNAP_EC_NUMBER_OF_FAN_CHANNELS 64
#define QNAP_EC_NUMBER_OF_PWM_CHANNELS QNAP_EC_NUMBER_OF_FAN_CHANNELS
#define QNAP_EC_NUMBER_OF_TEMP_CHANNELS 64

// Define the devices structure
struct qnap_ec_devices {
  struct mutex misc_device_mutex;
  bool open_misc_device;
  struct miscdevice misc_device;
  struct platform_device* plat_device;
};

// Define the I/O control data structure
struct qnap_ec_data {
  struct mutex mutex;
  struct qnap_ec_devices* devices;
  struct qnap_ec_ioctl_command ioctl_command;
  uint8_t fan_channel_checked_field[QNAP_EC_NUMBER_OF_FAN_CHANNELS / 8];
  uint8_t fan_channel_valid_field[QNAP_EC_NUMBER_OF_FAN_CHANNELS / 8];
  uint8_t pwm_channel_checked_field[QNAP_EC_NUMBER_OF_PWM_CHANNELS / 8];
  uint8_t pwm_channel_valid_field[QNAP_EC_NUMBER_OF_PWM_CHANNELS / 8];
  uint8_t pwm_enable_value_field[QNAP_EC_NUMBER_OF_PWM_CHANNELS / 8];
  uint8_t temp_channel_checked_field[QNAP_EC_NUMBER_OF_TEMP_CHANNELS / 8];
  uint8_t temp_channel_valid_field[QNAP_EC_NUMBER_OF_TEMP_CHANNELS / 8];
};

// Declare functions
static int __init qnap_ec_init(void);
static int __init qnap_ec_is_chip_present(void);
static int qnap_ec_probe(struct platform_device* platform_dev);
static umode_t qnap_ec_hwmon_is_visible(const void* const_data, enum hwmon_sensor_types type,
                                        u32 attribute, int channel);
static int qnap_ec_hwmon_read(struct device* dev, enum hwmon_sensor_types type, u32 attribute,
                              int channel, long* value);
static int qnap_ec_hwmon_write(struct device* dev, enum hwmon_sensor_types type, u32 attribute,
                               int channel, long value);
static bool qnap_ec_is_fan_channel_valid(struct qnap_ec_data* data, uint8_t channel);
static bool qnap_ec_is_pwm_channel_valid(struct qnap_ec_data* data, uint8_t channel);
static bool qnap_ec_is_temp_channel_valid(struct qnap_ec_data* data, uint8_t channel);
static int qnap_ec_is_pwm_channel_valid_read_fan_pwms(struct qnap_ec_data* data, uint8_t channel,
                                                      uint8_t initial_pwm_values[],
                                                      uint8_t changed_pwm_values[]);
static int qnap_ec_call_lib_function(bool use_mutex, struct qnap_ec_data* data,
                                     enum qnap_ec_ioctl_function_type function_type,
                                     char* function_name, uint8_t argument1_uint8,
                                     uint8_t* argument2_uint8, uint32_t* argument2_uint32,
                                     int64_t* argument2_int64, bool log_return_error);
static int qnap_ec_misc_device_open(struct inode* inode, struct file* file);
static long int qnap_ec_misc_device_ioctl(struct file* file, unsigned int command,
                                          unsigned long argument);
static int qnap_ec_misc_device_release(struct inode* inode, struct file* file);
static void __exit qnap_ec_exit(void);

// Specify the initialization and exit functions
module_init(qnap_ec_init);
module_exit(qnap_ec_exit);

// Define the module parameters
static bool qnap_ec_val_pwm_channels = true;
static bool qnap_ec_sim_pwm_enable = false;
static bool qnap_ec_check_for_chip = true;
module_param_named(val_pwm_channels, qnap_ec_val_pwm_channels, bool, 0);
module_param_named(sim_pwm_enable, qnap_ec_sim_pwm_enable, bool, 0);
module_param_named(check_for_chip, qnap_ec_check_for_chip, bool, 0);

// Declare the platform driver structure pointer
static struct platform_driver* qnap_ec_plat_driver;

// Declare the devices structure pointer
static struct qnap_ec_devices* qnap_ec_devices;

// Function called to initialize the driver
static int __init qnap_ec_init(void)
{
  // Define static constant data consisting of the miscellaneous device file operations structure
  static const struct file_operations misc_device_file_ops = {
    .owner = THIS_MODULE,
    .open = &qnap_ec_misc_device_open,
    .unlocked_ioctl = &qnap_ec_misc_device_ioctl,
    .release = &qnap_ec_misc_device_release
  };

  // Declare needed variables
  int error;

  // Check if the embedded controller chip isn't present
  error = qnap_ec_is_chip_present();
  if (error)
    return error;

  // Allocate memory for the platform driver structure and populate various fields
  qnap_ec_plat_driver = kzalloc(sizeof(struct platform_driver), GFP_KERNEL);
  if (qnap_ec_plat_driver == NULL)
    return -ENOMEM;
  qnap_ec_plat_driver->driver.owner = THIS_MODULE;
  qnap_ec_plat_driver->driver.name = "qnap-ec";
  qnap_ec_plat_driver->probe = &qnap_ec_probe;

  // Register the driver
  error = platform_driver_register(qnap_ec_plat_driver);
  if (error)
  {
    // Free the platform driver structure memory
    kfree(qnap_ec_plat_driver);

    return error;
  }

  // Allocate memory for the devices structure
  qnap_ec_devices = kzalloc(sizeof(struct qnap_ec_devices), GFP_KERNEL);
  if (qnap_ec_devices == NULL)
  {
    // Unregister the driver
    platform_driver_unregister(qnap_ec_plat_driver);

    // Free the platform driver structure memory
    kfree(qnap_ec_plat_driver);

    return -ENOMEM;
  }

  // Initialize the miscellaneous device mutex
  mutex_init(&qnap_ec_devices->misc_device_mutex);

  // Populate various miscellaneous device structure fields
  qnap_ec_devices->misc_device.name = "qnap-ec";
  qnap_ec_devices->misc_device.minor = MISC_DYNAMIC_MINOR;
  qnap_ec_devices->misc_device.fops = &misc_device_file_ops;

  // Register the miscellaneous device
  error = misc_register(&qnap_ec_devices->misc_device);
  if (error)
  {
    // Free the devices structure memory
    kfree(qnap_ec_devices);

    // Unregister the platform driver
    platform_driver_unregister(qnap_ec_plat_driver);

    // Free the platform driver structure memory
    kfree(qnap_ec_plat_driver);

    return error;
  }

  // Allocate memory for the platform device structure and populate various fields
  qnap_ec_devices->plat_device = platform_device_alloc("qnap-ec", 0);
  if (qnap_ec_devices->plat_device == NULL)
  {
    // Unregister the miscellaneous device
    misc_deregister(&qnap_ec_devices->misc_device);

    // Free the devices structure memory
    kfree(qnap_ec_devices);

    // Unregister the driver
    platform_driver_unregister(qnap_ec_plat_driver);

    // Free the platform driver structure memory
    kfree(qnap_ec_plat_driver);

    return -ENOMEM;
  }
  qnap_ec_devices->plat_device->name = "qnap-ec";
  qnap_ec_devices->plat_device->id = PLATFORM_DEVID_NONE;

  // Register the platform device
  error = platform_device_add(qnap_ec_devices->plat_device);
  if (error)
  {
    // Free the platform device structure memory
    platform_device_put(qnap_ec_devices->plat_device);

    // Unregister the miscellaneous device
    misc_deregister(&qnap_ec_devices->misc_device);

    // Free the devices structure memory
    kfree(qnap_ec_devices);

    // Unregister the platform driver
    platform_driver_unregister(qnap_ec_plat_driver);

    // Free the platform driver structure memory
    kfree(qnap_ec_plat_driver);

    return error;
  }

  return 0;
}

// Function called to check if the QNAP embedded controller chip is present
static int __init qnap_ec_is_chip_present(void)
{
  // Declare needed variables
  uint8_t byte1;
  uint8_t byte2;

  // Check if we should not check for the chip
  if (!qnap_ec_check_for_chip)
    return 0;

  // Request access to the input (0x2E) and output (0x2F) ports
  if (request_muxed_region(0x2E, 2, "qnap-ec") == NULL)
    return -EBUSY;

  // Write 0x20 to the input port
  outb(0x20, 0x2E);

  // Read the first identification byte from the output port
  byte1 = inb(0x2F);

  // Write 0x21 to the input port
  outb(0x21, 0x2E);

  // Read the second identification byte from the output port
  byte2 = inb(0x2F);

  // Check if the identification bytes do not match the expected values
  if (byte1 != 0x85 || byte2 != 0x28)
  {
    release_region(0x2E, 2);
    return -ENODEV;
  }

  // Release access to the input and output ports
  release_region(0x2E, 2);

  return 0;
}

// Function called to probe this driver
static int qnap_ec_probe(struct platform_device* platform_dev)
{
  // Define static non-constant and constant data consisting of multiple configuration arrays,
  // multiple hwmon channel info structures, the hwmon channel info structures array, and the
  // hwmon chip information structure
  static u32 fan_config[QNAP_EC_NUMBER_OF_FAN_CHANNELS + 1];
  static u32 pwm_config[QNAP_EC_NUMBER_OF_PWM_CHANNELS + 1];
  static u32 temp_config[QNAP_EC_NUMBER_OF_TEMP_CHANNELS + 1];
  static const struct hwmon_channel_info fan_channel_info = {
    .type = hwmon_fan,
    .config = fan_config
  };
  static const struct hwmon_channel_info pwm_channel_info = {
    .type = hwmon_pwm,
    .config = pwm_config
  };
  static const struct hwmon_channel_info temp_channel_info = {
    .type = hwmon_temp,
    .config = temp_config
  };
  static const struct hwmon_channel_info* hwmon_channel_info[] = {
    &fan_channel_info,
    &pwm_channel_info,
    &temp_channel_info,
    NULL
  };
  static const struct hwmon_ops hwmon_ops = {
    .is_visible = &qnap_ec_hwmon_is_visible,
    .read = &qnap_ec_hwmon_read,
    .write = &qnap_ec_hwmon_write
  };
  static const struct hwmon_chip_info hwmon_chip_info = {
    .info = hwmon_channel_info,
    .ops = &hwmon_ops
  };

  // Declare needed variables
  uint8_t i;
  struct qnap_ec_data* data;
  struct device* device;

  // Allocate device-managed memory for the data structure
  data = devm_kzalloc(&platform_dev->dev, sizeof(struct qnap_ec_data), GFP_KERNEL);
  if (data == NULL)
    return -ENOMEM;

  // Initialize the data mutex, set the devices pointer, and if we are simulating the PWM enable
  // attribute set the PWM enable values
  mutex_init(&data->mutex);
  data->devices = qnap_ec_devices;
  if (qnap_ec_sim_pwm_enable)
    for (i = 0; i < QNAP_EC_NUMBER_OF_PWM_CHANNELS / 8; ++i)
      data->pwm_enable_value_field[i] = 0xFF;

  // Set the custom device data to the data structure
  dev_set_drvdata(&platform_dev->dev, data);

  // Populate the fan configuration array
  for (i = 0; i < QNAP_EC_NUMBER_OF_FAN_CHANNELS; ++i)
    fan_config[i] = HWMON_F_INPUT;
  fan_config[i] = 0;

  // Populate the PWM configuration array
  if (qnap_ec_sim_pwm_enable)
    for (i = 0; i < QNAP_EC_NUMBER_OF_PWM_CHANNELS; ++i)
      pwm_config[i] = HWMON_PWM_INPUT | HWMON_PWM_ENABLE;
  else
    for (i = 0; i < QNAP_EC_NUMBER_OF_PWM_CHANNELS; ++i)
      pwm_config[i] = HWMON_PWM_INPUT;
  pwm_config[i] = 0;

  // Populate the temperature configuration array
  for (i = 0; i < QNAP_EC_NUMBER_OF_TEMP_CHANNELS; ++i)
    temp_config[i] = HWMON_T_INPUT;
  temp_config[i] = 0;

  // Register the hwmon device and pass in the data structure
  device = devm_hwmon_device_register_with_info(&platform_dev->dev, "qnap_ec", data,
                                                &hwmon_chip_info, NULL);
  if (device == NULL)
    return -ENOMEM;

  return 0;
}

// Function called to check if a hwmon attribute is visible
static umode_t qnap_ec_hwmon_is_visible(const void* const_data, enum hwmon_sensor_types type,
                                        u32 attribute, int channel)
{
  // Declare needed variables
  struct qnap_ec_data* data = dev_get_drvdata(&((const struct qnap_ec_data*)const_data)->devices->
    plat_device->dev);

  // Switch based on the sensor type
  switch (type)
  {
    case hwmon_fan:
      switch (attribute)
      {
        case hwmon_fan_input:
          if (qnap_ec_is_fan_channel_valid(data, channel))
            return S_IRUGO;
          break;
      }
      break;
    case hwmon_pwm:
      switch (attribute)
      {
        case hwmon_pwm_enable:
          if (qnap_ec_sim_pwm_enable && qnap_ec_is_pwm_channel_valid(data, channel))
            return S_IRUGO | S_IWUSR;
          break;
        case hwmon_pwm_input:
          if (qnap_ec_is_pwm_channel_valid(data, channel))
            return S_IRUGO | S_IWUSR;
          break;
      }
      break;
    case hwmon_temp:
      switch (attribute)
      {
        case hwmon_temp_input:
          if (qnap_ec_is_temp_channel_valid(data, channel))
            return S_IRUGO;
          break;
      }
      break;
    default:
      break;
  }

  return 0;
}

// Function called to read from a hwmon attribute
static int qnap_ec_hwmon_read(struct device* device, enum hwmon_sensor_types type, u32 attribute,
                              int channel, long* value)
{
  uint32_t fan_speed;
  uint32_t fan_pwm;
  int64_t temperature;
  struct qnap_ec_data* data = dev_get_drvdata(device);

  switch (type)
  {
    case hwmon_fan:
      switch (attribute)
      {
        case hwmon_fan_input:
          if (!qnap_ec_is_fan_channel_valid(data, channel))
            return -EOPNOTSUPP;

          if (qnap_ec_call_lib_function(true, data, int8_func_uint8_uint32pointer,
              "ec_sys_get_fan_speed", channel, NULL, &fan_speed, NULL, true) != 0)
            return -ENODATA;
          *value = fan_speed;
          break;
        default:
          return -EOPNOTSUPP;
      }
      break;
    case hwmon_pwm:
      switch (attribute)
      {
        case hwmon_pwm_enable:
          if (!qnap_ec_sim_pwm_enable || !qnap_ec_is_pwm_channel_valid(data, channel))
            return -EOPNOTSUPP;
          *value = (data->pwm_enable_value_field[channel / 8] >> (channel % 8)) & 0x01;
          break;
        case hwmon_pwm_input:
          if (!qnap_ec_is_pwm_channel_valid(data, channel))
            return -EOPNOTSUPP;

          if (qnap_ec_call_lib_function(true, data, int8_func_uint8_uint32pointer,
              "ec_sys_get_fan_pwm", channel, NULL, &fan_pwm, NULL, true) != 0)
            return -ENODATA;
          *value = fan_pwm;
          break;
        default:
          return -EOPNOTSUPP;
      }
      break;
    case hwmon_temp:
      switch (attribute)
      {
        case hwmon_temp_input:
          if (!qnap_ec_is_temp_channel_valid(data, channel))
            return -EOPNOTSUPP;

          if (qnap_ec_call_lib_function(true, data, int8_func_uint8_doublepointer,
              "ec_sys_get_temperature", channel, NULL, NULL, &temperature, true) != 0)
            return -ENODATA;
          *value = temperature;
          break;
        default:
          return -EOPNOTSUPP;
      }
      break;
    default:
      return -EOPNOTSUPP;
  }

  return 0;
}

// Function called to write to a hwmon attribute
static int qnap_ec_hwmon_write(struct device* device, enum hwmon_sensor_types type, u32 attribute,
                               int channel, long value)
{
  uint8_t fan_pwm = value;
  struct qnap_ec_data* data = dev_get_drvdata(device);

  switch (type)
  {
    case hwmon_pwm:
      switch (attribute)
      {
        case hwmon_pwm_enable:
          if (!qnap_ec_sim_pwm_enable || !qnap_ec_is_pwm_channel_valid(data, channel))
            return -EOPNOTSUPP;

          if (value < 0 || value > 1)
            return -EOPNOTSUPP;

          if (value == 0)
          {
            data->pwm_enable_value_field[channel / 8] &= ~(0x01 << (channel % 8));
            fan_pwm = 255;
            if (qnap_ec_call_lib_function(true, data, int8_func_uint8_uint8,
                "ec_sys_set_fan_speed", channel, &fan_pwm, NULL, NULL, true) != 0)
              return -EOPNOTSUPP;
          }
          else
          {
            data->pwm_enable_value_field[channel / 8] |= (0x01 << (channel % 8));
          }
          break;
        case hwmon_pwm_input:
          if (!qnap_ec_is_pwm_channel_valid(data, channel) || (qnap_ec_sim_pwm_enable &&
             ((data->pwm_enable_value_field[channel / 8] >> (channel % 8)) & 0x01) == 0))
            return -EOPNOTSUPP;

          if (value < 0 || value > 255)
            return -EOVERFLOW;

          if (qnap_ec_call_lib_function(true, data, int8_func_uint8_uint8, "ec_sys_set_fan_speed",
              channel, &fan_pwm, NULL, NULL, true) != 0)
            return -EOPNOTSUPP;
          break;
        default:
          return -EOPNOTSUPP;
      }
      break;
    default:
      return -EOPNOTSUPP;
  }

  return 0;
}

// Function called to check if the fan channel number is valid
static bool qnap_ec_is_fan_channel_valid(struct qnap_ec_data* data, uint8_t channel)
{
  uint32_t fan_status;
  uint32_t fan_speed;
  uint32_t fan_pwm;

  if (((data->fan_channel_checked_field[channel / 8] >> (channel % 8)) & 0x01) == 1)
  {
    if (((data->fan_channel_valid_field[channel / 8] >> (channel % 8)) & 0x01) == 0)
      return false;
    return true;
  }

  mutex_lock(&data->mutex);

  fan_status = 1;
  if (qnap_ec_call_lib_function(false, data, int8_func_uint8_uint32pointer, "ec_sys_get_fan_status",
      channel, NULL, &fan_status, NULL, false) != 0)
  {
    data->fan_channel_checked_field[channel / 8] |= (0x01 << (channel % 8));
    mutex_unlock(&data->mutex);
    return false;
  }

  if (fan_status != 0)
  {
    data->fan_channel_checked_field[channel / 8] |= (0x01 << (channel % 8));
    mutex_unlock(&data->mutex);
    return false;
  }

  fan_speed = 65535;
  if (qnap_ec_call_lib_function(false, data, int8_func_uint8_uint32pointer, "ec_sys_get_fan_speed",
      channel, NULL, &fan_speed, NULL, false) != 0)
  {
    data->fan_channel_checked_field[channel / 8] |= (0x01 << (channel % 8));
    mutex_unlock(&data->mutex);
    return false;
  }

  if (fan_speed == 65535)
  {
    data->fan_channel_checked_field[channel / 8] |= (0x01 << (channel % 8));
    mutex_unlock(&data->mutex);
    return false;
  }

  fan_pwm = 256;
  if (qnap_ec_call_lib_function(false, data, int8_func_uint8_uint32pointer, "ec_sys_get_fan_pwm",
      channel, NULL, &fan_pwm, NULL, false) != 0)
  {
    data->fan_channel_checked_field[channel / 8] |= (0x01 << (channel % 8));
    mutex_unlock(&data->mutex);
    return false;
  }

  if (fan_pwm > 255)
  {
    data->fan_channel_checked_field[channel / 8] |= (0x01 << (channel % 8));
    mutex_unlock(&data->mutex);
    return false;
  }

  data->fan_channel_checked_field[channel / 8] |= (0x01 << (channel % 8));
  data->fan_channel_valid_field[channel / 8] |= (0x01 << (channel % 8));

  mutex_unlock(&data->mutex);

  return true;
}

// Function called to check if the PWM channel number is valid
static bool qnap_ec_is_pwm_channel_valid(struct qnap_ec_data* data, uint8_t channel)
{
  uint8_t i;
  uint8_t fan_pwm;
  uint32_t fan_speed;
  bool valid_channel_marked = false;
  uint8_t initial_fan_pwms[QNAP_EC_NUMBER_OF_PWM_CHANNELS];
  uint8_t changed_fan_pwms[QNAP_EC_NUMBER_OF_PWM_CHANNELS];

  if (!qnap_ec_val_pwm_channels)
    return qnap_ec_is_fan_channel_valid(data, channel);

  if (((data->pwm_channel_checked_field[channel / 8] >> (channel % 8)) & 0x01) == 1)
  {
    if (((data->pwm_channel_valid_field[channel / 8] >> (channel % 8)) & 0x01) == 0)
      return false;
    return true;
  }

  mutex_lock(&data->mutex);

  if (qnap_ec_is_pwm_channel_valid_read_fan_pwms(data, channel, initial_fan_pwms, NULL) != 0)
  {
    data->pwm_channel_checked_field[channel / 8] |= (0x01 << (channel % 8));
    mutex_unlock(&data->mutex);
    return false;
  }

  if (initial_fan_pwms[channel] <= 250)
    fan_pwm = initial_fan_pwms[channel] + 5;
  else
    fan_pwm = initial_fan_pwms[channel] - 5;

  if (qnap_ec_call_lib_function(false, data, int8_func_uint8_uint8, "ec_sys_set_fan_speed", channel,
      &fan_pwm, NULL, NULL, false) != 0)
  {
    data->pwm_channel_checked_field[channel / 8] |= (0x01 << (channel % 8));
    mutex_unlock(&data->mutex);
    return false;
  }

  if (qnap_ec_is_pwm_channel_valid_read_fan_pwms(data, channel, initial_fan_pwms,
      changed_fan_pwms) != 0)
  {
    data->pwm_channel_checked_field[channel / 8] |= (0x01 << (channel % 8));
    mutex_unlock(&data->mutex);
    return false;
  }

  if (initial_fan_pwms[channel] == changed_fan_pwms[channel])
  {
    data->pwm_channel_checked_field[channel / 8] |= (0x01 << (channel % 8));
    mutex_unlock(&data->mutex);
    return false;
  }

  fan_pwm = initial_fan_pwms[channel];
  if (qnap_ec_call_lib_function(false, data, int8_func_uint8_uint8, "ec_sys_set_fan_speed", channel,
      &fan_pwm, NULL, NULL, false) != 0)
  {
    data->pwm_channel_checked_field[channel / 8] |= (0x01 << (channel % 8));
    mutex_unlock(&data->mutex);
    return false;
  }

  for (i = 0; i < QNAP_EC_NUMBER_OF_PWM_CHANNELS; ++i)
  {
    if (((data->pwm_channel_checked_field[i / 8] >> (i % 8)) & 0x01) == 1)
      continue;

    if (initial_fan_pwms[i] != initial_fan_pwms[channel])
      continue;

    if (changed_fan_pwms[i] != changed_fan_pwms[channel])
      continue;

    data->pwm_channel_checked_field[i / 8] |= (0x01 << (i % 8));

    if (!valid_channel_marked)
    {
      fan_speed = 65535;
      if (qnap_ec_call_lib_function(false, data, int8_func_uint8_uint32pointer,
          "ec_sys_get_fan_speed", i, NULL, &fan_speed, NULL, false) != 0)
        continue;

      if (fan_speed == 65535)
        continue;

      data->pwm_channel_valid_field[i / 8] |= (0x01 << (i % 8));
      valid_channel_marked = true;
    }
  }

  mutex_unlock(&data->mutex);

  if (((data->pwm_channel_valid_field[channel / 8] >> (channel % 8)) & 0x01) == 0)
    return false;

  return true;
}

// Function called by the qnap_ec_is_pwm_channel_valid function to read the fan PWMs
static int qnap_ec_is_pwm_channel_valid_read_fan_pwms(struct qnap_ec_data* data, uint8_t channel,
                                                      uint8_t initial_fan_pwms[],
                                                      uint8_t changed_fan_pwms[])
{
  uint8_t i;
  uint8_t j;
  uint32_t fan_pwm;

  for (i = 0, j = channel; i < QNAP_EC_NUMBER_OF_PWM_CHANNELS; ++i, j = (j + 1) %
       QNAP_EC_NUMBER_OF_PWM_CHANNELS)
  {
    if (((data->pwm_channel_checked_field[j / 8] >> (j % 8)) & 0x01) == 1)
      continue;

    if (changed_fan_pwms != NULL && initial_fan_pwms[j] != initial_fan_pwms[channel])
      continue;

    fan_pwm = 256;
    if (qnap_ec_call_lib_function(false, data, int8_func_uint8_uint32pointer, "ec_sys_get_fan_pwm",
        j, NULL, &fan_pwm, NULL, false) != 0)
    {
      if (j == channel)
        return -ENODATA;
      data->pwm_channel_checked_field[j / 8] |= (0x01 << (j % 8));
      continue;
    }

    if (fan_pwm > 255)
    {
      if (j == channel)
        return -ENODATA;
      data->pwm_channel_checked_field[j / 8] |= (0x01 << (j % 8));
      continue;
    }

    if (changed_fan_pwms == NULL)
      initial_fan_pwms[j] = fan_pwm;
    else
      changed_fan_pwms[j] = fan_pwm;
  }

  return 0;
}

// Function called to check if the temperature channel number is valid
static bool qnap_ec_is_temp_channel_valid(struct qnap_ec_data* data, uint8_t channel)
{
  int64_t temperature;

  if (((data->temp_channel_checked_field[channel / 8] >> (channel % 8)) & 0x01) == 1)
  {
    if (((data->temp_channel_valid_field[channel / 8] >> (channel % 8)) & 0x01) == 0)
      return false;
    return true;
  }

  mutex_lock(&data->mutex);

  temperature = -1;
  if (qnap_ec_call_lib_function(false, data, int8_func_uint8_doublepointer,
      "ec_sys_get_temperature", channel, NULL, NULL, &temperature, false) != 0)
  {
    data->temp_channel_checked_field[channel / 8] |= (0x01 << (channel % 8));
    mutex_unlock(&data->mutex);
    return false;
  }

  if (temperature < 0)
  {
    data->temp_channel_checked_field[channel / 8] |= (0x01 << (channel % 8));
    mutex_unlock(&data->mutex);
    return false;
  }

  data->temp_channel_checked_field[channel / 8] |= (0x01 << (channel % 8));
  data->temp_channel_valid_field[channel / 8] |= (0x01 << (channel % 8));

  mutex_unlock(&data->mutex);

  return true;
}

// Function called to call a function in the libuLinux_hal library via the user space helper program
static int qnap_ec_call_lib_function(bool use_mutex, struct qnap_ec_data* data,
                                     enum qnap_ec_ioctl_function_type function_type,
                                     char* function_name, uint8_t argument1_uint8,
                                     uint8_t* argument2_uint8, uint32_t* argument2_uint32,
                                     int64_t* argument2_int64, bool log_return_error)
{
  uint8_t i;
  int return_value;
#ifdef PACKAGE
  char* paths[] = { "/usr/sbin/qnap-ec", "/usr/bin/qnap-ec", "/sbin/qnap-ec", "/bin/qnap-ec" };
#else
  char* paths[] = { "/usr/local/sbin/qnap-ec", "/usr/local/bin/qnap-ec", "/usr/sbin/qnap-ec",
    "/usr/bin/qnap-ec", "/sbin/qnap-ec", "/bin/qnap-ec" };
#endif

  if (use_mutex)
    mutex_lock(&data->mutex);

  data->ioctl_command.function_type = function_type;
  strncpy(data->ioctl_command.function_name, function_name,
    sizeof(((struct qnap_ec_ioctl_command*)0)->function_name) - 1);
  data->ioctl_command.argument1_uint8 = argument1_uint8;
  if (argument2_uint8 != NULL)
    data->ioctl_command.argument2_uint8 = *argument2_uint8;
  if (argument2_uint32 != NULL)
    data->ioctl_command.argument2_uint32 = *argument2_uint32;
  if (argument2_int64 != NULL)
    data->ioctl_command.argument2_int64 = *argument2_int64;

  data->devices->open_misc_device = true;

  i = 0;
  do
    return_value = call_usermodehelper(paths[i], (char*[]){ paths[i], NULL }, NULL, UMH_WAIT_PROC);
  while ((return_value & 0xFF) != 0 && ++i < sizeof(paths) / sizeof(char*));

  if ((return_value & 0xFF) != 0)
  {
#ifdef PACKAGE
    pr_err("qnap-ec helper program not found at the expected path (%s) or any of the fall back "
      "paths (%s, %s, %s)", paths[0], paths[1], paths[2], paths[3]);
#else
    pr_err("qnap-ec helper program not found at the expected path (%s) or any of the fall back "
      "paths (%s, %s, %s, %s, %s)", paths[0], paths[1], paths[2], paths[3], paths[4], paths[5]);
#endif

    data->devices->open_misc_device = false;

    if (use_mutex)
      mutex_unlock(&data->mutex);

    return return_value & 0xFF;
  }

  if (((return_value >> 8) & 0xFF) != 0)
  {
    pr_err("qnap-ec helper program exited with a non zero exit code (+/-%i)",
      ((return_value >> 8) & 0xFF));

    data->devices->open_misc_device = false;

    if (use_mutex)
      mutex_unlock(&data->mutex);

    return (return_value >> 8) & 0xFF;
  }

  data->devices->open_misc_device = false;

  if (data->ioctl_command.return_value_int8 != 0)
  {
    if (log_return_error)
      pr_err("libuLinux_hal library %s function called by qnap-ec helper program returned a non "
        "zero value (%i)", data->ioctl_command.function_name,
        data->ioctl_command.return_value_int8);

    if (use_mutex)
      mutex_unlock(&data->mutex);

    return data->ioctl_command.return_value_int8;
  }

  if (argument2_uint32 != NULL)
    *argument2_uint32 = data->ioctl_command.argument2_uint32;
  if (argument2_int64 != NULL)
    *argument2_int64 = data->ioctl_command.argument2_int64;

  if (use_mutex)
    mutex_unlock(&data->mutex);

  return 0;
}

// Function called when the miscellaneous device is opened
static int qnap_ec_misc_device_open(struct inode* inode, struct file* file)
{
  struct qnap_ec_devices* devices = container_of(file->private_data, struct qnap_ec_devices,
    misc_device);

  if (devices->open_misc_device == false)
    return -EBUSY;

  if (mutex_trylock(&devices->misc_device_mutex) == 0)
    return -EBUSY;

  return 0;
}

// Function called when the miscellaneous device receives an I/O control command
static long int qnap_ec_misc_device_ioctl(struct file* file, unsigned int command,
                                          unsigned long argument)
{
  struct qnap_ec_data* data = dev_get_drvdata(&container_of(file->private_data,
    struct qnap_ec_devices, misc_device)->plat_device->dev);

  switch (command)
  {
    case QNAP_EC_IOCTL_CALL:
      if (!access_ok((const void __user *)argument, sizeof(struct qnap_ec_ioctl_command)))
        return -EFAULT;

      if (copy_to_user((void __user *)argument, &data->ioctl_command,
          sizeof(struct qnap_ec_ioctl_command)) != 0)
        return -EFAULT;
      break;
    case QNAP_EC_IOCTL_RETURN:
      if (!access_ok((const void __user *)argument, sizeof(struct qnap_ec_ioctl_command)))
        return -EFAULT;

      if (copy_from_user(&data->ioctl_command, (const void __user *)argument,
          sizeof(struct qnap_ec_ioctl_command)) != 0)
        return -EFAULT;

      break;
    default:
      return -EINVAL;
  }

  return 0;
}

// Function called when the miscellaneous device is released
static int qnap_ec_misc_device_release(struct inode* inode, struct file* file)
{
  struct qnap_ec_devices* devices = container_of(file->private_data, struct qnap_ec_devices,
    misc_device);

  mutex_unlock(&devices->misc_device_mutex);

  return 0;
}

// Function called to exit the driver
static void __exit qnap_ec_exit(void)
{
  platform_device_unregister(qnap_ec_devices->plat_device);

  misc_deregister(&qnap_ec_devices->misc_device);

  kfree(qnap_ec_devices);

  platform_driver_unregister(qnap_ec_plat_driver);

  kfree(qnap_ec_plat_driver);
}
