#ifndef SRC_SENSOR_OPS_H
#define SRC_SENSOR_OPS_H
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REG_LOW_MASK 0xFF00
#define REG_HIGH_MASK 0x00FF
#ifndef CONFIG_APPLICATION_IMU_FUSION_DEMO_AFS_SEL
#define CONFIG_APPLICATION_IMU_FUSION_DEMO_AFS_SEL 4096
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct imu_msg {
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;
  int16_t temp;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
};

struct imu_msg_float {
  float acc_x;
  float acc_y;
  float acc_z;
  float temp;
  float gyro_x;
  float gyro_y;
  float gyro_z;
};


/****************************************************************************
 * Public Functions
 ****************************************************************************/

void read_imu(int fd, struct imu_msg *imu_data);

#endif  // SRC_SENSOR_OPS_H
