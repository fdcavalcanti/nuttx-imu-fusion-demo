#ifndef APPLICATION_IMU_FUSION_DEMO_H
#define APPLICATION_IMU_FUSION_DEMO_H
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

struct accelerometer_msg {
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void read_accelerometer(int fd, struct accelerometer_msg *acc_msg);

#endif  // APPLICATION_IMU_FUSION_DEMO_H
