#include "accelerometer.h"

#include <stdio.h>

void read_accelerometer(int fd, struct accelerometer_msg *acc_msg) {
  int16_t raw_data[7];

  int ret = read(fd, &raw_data, sizeof(raw_data));
  if (ret != sizeof(raw_data)) {
    printf("Failed to read accelerometer data\n");
  } else {
    acc_msg->acc_x = ((raw_data[0] & REG_HIGH_MASK) << 8) +
                     ((raw_data[0] & REG_LOW_MASK) >> 8);
    acc_msg->acc_y = ((raw_data[1] & REG_HIGH_MASK) << 8) +
                     ((raw_data[1] & REG_LOW_MASK) >> 8);
    acc_msg->acc_z = ((raw_data[2] & REG_HIGH_MASK) << 8) +
                     ((raw_data[2] & REG_LOW_MASK) >> 8);
  }
}
