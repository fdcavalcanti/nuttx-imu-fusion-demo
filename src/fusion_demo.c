/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <nuttx/config.h>
#include <nuttx/mqueue.h>
#include <nuttx/sensors/ioctl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "sensor_ops.h"
#include "Fusion/Fusion.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MQTT_MSG_LEN 100
#define SERIAL_OUTPUT false
#define PORT 5000
#define MAXLINE 1000
#define BYTES_PER_MSG 14

#ifndef CONFIG_APPLICATION_IMU_FUSION_DEMO_SAMPLE_RATE_MS
#define CONFIG_APPLICATION_IMU_FUSION_DEMO_SAMPLE_RATE_MS 10
#endif

#ifndef CONFIG_APPLICATION_IMU_FUSION_DEMO_FS_SEL
#define CONFIG_APPLICATION_IMU_FUSION_DEMO_FS_SEL 65.5f
#endif

#ifndef CONFIG_APPLICATION_IMU_FUSION_DEMO_AFS_SEL
#define CONFIG_APPLICATION_IMU_FUSION_DEMO_AFS_SEL 8192
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool client_listening;
int fd, socket_fd, client_tcp_fd;

/****************************************************************************
 * Public Data
 ****************************************************************************/

static pid_t accelerometer_pid;
static pid_t offload_pid;
static pid_t sensor_ops_pid;

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static int start_network_socket(void);
static int offload_task(int argc, FAR char *argv[]);
static int imu_task(int argc, FAR char *argv[]);
static int sensor_ops_task(int argc, FAR char *argv[]);

/****************************************************************************
 * main
 ****************************************************************************/

int main(int argc, FAR char *argv[]) {
  printf("Starting Fusion Demo\n");
  char *child_argv[4];
  struct mq_attr acc_attr, sensor_ops_attr;
  mqd_t acc_mqd, sensor_ops_mqd;

  printf("Opening message queue channels\n");
  /* IMU Read Task message queue */
  acc_attr.mq_flags = 0;
  acc_attr.mq_maxmsg = 2;
  acc_attr.mq_msgsize = sizeof(struct imu_msg);
  acc_attr.mq_curmsgs = 0;

  acc_mqd = mq_open("IMU Queue", O_RDWR | O_CREAT, 0666, &acc_attr);
  if (acc_mqd == (mqd_t)-1) {
    printf("Failed to start IMU data queue\n");
    return 1;
  }

  /* Sensor Ops Task message queue */
  sensor_ops_attr.mq_flags = 0;
  sensor_ops_attr.mq_maxmsg = 2;
  sensor_ops_attr.mq_msgsize = sizeof(FusionEuler);
  sensor_ops_attr.mq_curmsgs = 0;

  sensor_ops_mqd = mq_open("Sensor Ops Queue", O_RDWR | O_CREAT, 0666, &sensor_ops_attr);
  if (sensor_ops_mqd == (mqd_t)-1) {
    printf("Failed to start Sensor Ops queue\n");
    return 1;
  }

  // int ret = start_network_socket();
  // if (ret < 0) {
  //   printf("Failed to start socket\n");
  //   mq_close(acc_mqd);
  //   return 1;
  // }

  child_argv[0] = (char *)&acc_mqd;
  child_argv[1] = (char *)&sensor_ops_mqd;
  child_argv[2] = (char *)&client_tcp_fd;
  child_argv[3] = NULL;

  accelerometer_pid = task_create(
      "IMU Task", 120, CONFIG_APPLICATION_IMU_FUSION_DEMO_STACKSIZE,
      imu_task, (char *const *)child_argv);
  if (accelerometer_pid < 0) {
    printf("Failed to create IMU task\n");
  }

  sensor_ops_pid = task_create(
      "Sensor Ops Task", 120, CONFIG_APPLICATION_IMU_FUSION_DEMO_STACKSIZE,
      sensor_ops_task, (char *const *)child_argv);
  if (sensor_ops_pid < 0) {
    printf("Failed to create Sensor Ops task\n");
  }

  offload_pid =
      task_create("Offload Task", 100, CONFIG_APPLICATION_IMU_FUSION_DEMO_STACKSIZE,
                  offload_task, (char *const *)child_argv);
  if (offload_pid < 0) {
    printf("Failed to create offload task\n");
  }

  return 0;
}


static int start_network_socket(void) {
  int len;
  struct sockaddr_in server, client;

  /* This condition ensures it is the first connection. */

  if (socket_fd == 0) {
    printf("Starting network\n");
    socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd < 0) {
      printf("Failed to create TCP socket\n");
      goto errout;
    }

    server.sin_family = AF_INET;
    server.sin_addr.s_addr = htonl(INADDR_ANY);
    server.sin_port = htons(PORT);

    if (bind(socket_fd, (struct sockaddr *)&server, sizeof(server)) != 0) {
      printf("TCP socket bind failed\n");
      goto errout;
    }

    if ((listen(socket_fd, 5)) != 0) {
      printf("Listen failed\n");
      goto errout;
    }
  }

  printf("Waiting for client\n");
  len = sizeof(client);
  client_tcp_fd =
      accept(socket_fd, (struct sockaddr *)&client, (socklen_t *)&len);

  if (client_tcp_fd < 0) {
    printf("Failed to accept connection\n");
    goto errout;
  }

  printf("Client connected: %s\n", inet_ntoa(client.sin_addr));
  client_listening = true;
  return OK;

errout:
  close(socket_fd);
  return EXIT_FAILURE;
}

static int offload_task(int argc, FAR char *argv[]) {
  printf("Starting offload task\n");

  int ret;
  int counter = 0;
  static char buffer[MQTT_MSG_LEN];
  mqd_t sensor_ops_mqd = (mqd_t)*argv[2];

  printf("Transmission starts\n");

  while (1) {
    ret = mq_receive(sensor_ops_mqd, (char *) &buffer, sizeof(buffer), 0);
    if (ret > 0) {
      printf("%s", buffer);
      // ret = write(conn_fd, msg_buffer, sizeof(msg_buffer));
      // if (ret <= 0) {
      //   printf("Client disconnected\n");
      //   client_listening = false;
      //   counter = 0;
      //   close(client_tcp_fd);
      //   start_network_socket();
      // }
      memset(buffer, 0, sizeof(buffer));
    }
  }

  return 1;
}

static int imu_task(int argc, FAR char *argv[]) {
  printf("Starting IMU task\n");

  struct imu_msg imu_data;
  mqd_t mqd_imu = (mqd_t)*argv[1];

  fd = open("/dev/imu0", O_RDWR);
  if (fd < 0) {
    printf("ERROR Failed to open IMU\n");
    return EXIT_FAILURE;
  }

  /* AFS_SEL full-scale range select
   * 0 -> ± 2 g -> 16384
   * 1 -> ± 4 g -> 8192
   * 2 -> ± 8 g -> 4096
   * 3 -> ± 16 g -> 2048
   */

  ioctl(fd, SNIOC_SET_AFS_SEL, 1);

  while (1) {
    read_imu(fd, &imu_data);
    int ret = mq_send(mqd_imu, (const void *)&imu_data, sizeof(imu_data), 0);
    if (ret < 0) {
      printf("Failed to send message\n");
    }
    usleep(CONFIG_APPLICATION_IMU_FUSION_DEMO_SAMPLE_RATE_MS*1000);
  }
  return 0;
}

static int sensor_ops_task(int argc, FAR char *argv[]) {
  printf("Starting Sensor Ops task\n");

  /* IMU data and message queue fd */
  struct imu_msg rcv_imu_queue = {0};
  struct imu_msg_float imu_current = {0};
  mqd_t mqd_imu = (mqd_t)*argv[1];

  /* Start FusionAhrs */
  /* Calibration not available for now.*/
  static FusionAhrs ahrs;
  // static FusionEuler euler;
  // static FusionVector gyroscope = {0.0f, 0.0f, 0.0f};
  // static FusionVector accelerometer = {0.0f, 0.0f, 1.0f};
  FusionAhrsInitialise(&ahrs);

  /* Output msg and utilities */
  int ret;
  char msg_buffer[100];
  const float period = CONFIG_APPLICATION_IMU_FUSION_DEMO_SAMPLE_RATE_MS * 1000.0f;
  memset(msg_buffer, 0, sizeof(msg_buffer));
  printf("Sensor Ops ready\n");

  while (1) {
    ret = mq_receive(mqd_imu, (char *) &rcv_imu_queue, sizeof(struct imu_msg), 0);

    if (ret < 0) {
      printf("mq_receive imu_mqd ret: %d\n", ret);
      continue;
    }

    /* Convert accelerometer and gyro values */
    imu_current.acc_x =  (float)rcv_imu_queue.acc_x / CONFIG_APPLICATION_IMU_FUSION_DEMO_AFS_SEL;
    imu_current.acc_y =  (float)rcv_imu_queue.acc_y / CONFIG_APPLICATION_IMU_FUSION_DEMO_AFS_SEL;
    imu_current.acc_z =  (float)rcv_imu_queue.acc_z / CONFIG_APPLICATION_IMU_FUSION_DEMO_AFS_SEL;
    imu_current.gyro_x = rcv_imu_queue.gyro_x / CONFIG_APPLICATION_IMU_FUSION_DEMO_FS_SEL;
    imu_current.gyro_y = rcv_imu_queue.gyro_y / CONFIG_APPLICATION_IMU_FUSION_DEMO_FS_SEL;
    imu_current.gyro_z = rcv_imu_queue.gyro_z / CONFIG_APPLICATION_IMU_FUSION_DEMO_FS_SEL;

    /* Apply received values to Fusion and calculate yaw, pitch and roll */
    const FusionVector gyroscope = {imu_current.gyro_x, imu_current.gyro_y, imu_current.gyro_z};
    const FusionVector accelerometer = {imu_current.acc_x, imu_current.acc_y, imu_current.acc_z};
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, period);
    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
    printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);

    /* Send data to offload task */
    snprintf(msg_buffer, sizeof(msg_buffer), "y%fyp%fpr%fr\n",
             euler.angle.yaw, euler.angle.pitch, euler.angle.roll);
    memset(msg_buffer, 0, sizeof(msg_buffer));
  }

  return 0;
}