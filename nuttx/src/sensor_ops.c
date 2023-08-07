#include "sensor_ops.h"

#include <stdio.h>

void read_imu(int fd, struct imu_msg *imu_data) {
  int16_t raw_data[7];

  int ret = read(fd, &raw_data, sizeof(raw_data));
  if (ret != sizeof(raw_data)) {
    printf("Failed to read accelerometer data\n");
  } else {
    imu_data->acc_x = ((raw_data[0] & REG_HIGH_MASK) << 8) +
                      ((raw_data[0] & REG_LOW_MASK) >> 8);
    imu_data->acc_y = ((raw_data[1] & REG_HIGH_MASK) << 8) +
                      ((raw_data[1] & REG_LOW_MASK) >> 8);
    imu_data->acc_z = ((raw_data[2] & REG_HIGH_MASK) << 8) +
                      ((raw_data[2] & REG_LOW_MASK) >> 8);
    imu_data->gyro_x = ((raw_data[4] & REG_HIGH_MASK) << 8) +
                       ((raw_data[4] & REG_LOW_MASK) >> 8);
    imu_data->gyro_y = ((raw_data[5] & REG_HIGH_MASK) << 8) +
                       ((raw_data[5] & REG_LOW_MASK) >> 8);
    imu_data->gyro_z = ((raw_data[6] & REG_HIGH_MASK) << 8) +
                       ((raw_data[6] & REG_LOW_MASK) >> 8);
  }
}

