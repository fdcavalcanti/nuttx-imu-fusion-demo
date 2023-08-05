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

#define TCP_MSG_LEN 100
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

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static int start_network_socket(void);
static int offload_task(int argc, FAR char *argv[]);
static int imu_task(int argc, FAR char *argv[]);

/****************************************************************************
 * main
 ****************************************************************************/

int main(int argc, FAR char *argv[]) {
  printf("Starting Fusion Demo\n");
  char *child_argv[3];
  struct mq_attr acc_attr;
  mqd_t acc_mqd;

  printf("Opening message queue channels\n");
  acc_attr.mq_flags = 0;
  acc_attr.mq_maxmsg = 2;
  acc_attr.mq_msgsize = sizeof(struct imu_msg);
  acc_attr.mq_curmsgs = 0;

  acc_mqd = mq_open("IMU Queue", O_RDWR | O_CREAT, 0666, &acc_attr);
  if (acc_mqd == (mqd_t)-1) {
    printf("Failed to start IMU data queue\n");
    return 1;
  }

  int ret = start_network_socket();
  if (ret < 0) {
    printf("Failed to start socket\n");
    mq_close(acc_mqd);
    return 1;
  }

  child_argv[0] = (char *)&acc_mqd;
  child_argv[1] = (char *)&client_tcp_fd;
  child_argv[2] = NULL;

  accelerometer_pid = task_create(
      "IMU Task", 120, CONFIG_APPLICATION_IMU_FUSION_DEMO_STACKSIZE,
      imu_task, (char *const *)child_argv);
  if (accelerometer_pid < 0) {
    printf("Failed to create IMU task\n");
  }

  offload_pid =
      task_create("Offload Task", 110, CONFIG_APPLICATION_IMU_FUSION_DEMO_STACKSIZE,
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
  char msg_buffer[TCP_MSG_LEN];
  memset(msg_buffer, 0, sizeof(msg_buffer));
  mqd_t imu_mqd = (mqd_t)*argv[1];
  int conn_fd = (int)*argv[2];
  int counter = 0;
  struct imu_msg rcv_imu_queue = {0};
  struct imu_msg_float imu_current = {0};
  
  FusionAhrs ahrs;
  FusionAhrsInitialise(&ahrs);
  const float period = CONFIG_APPLICATION_IMU_FUSION_DEMO_SAMPLE_RATE_MS / 1000.0;

  printf("Transmission started\n");

  while (1) {
    int ret;
    if (client_listening) {
      ret = mq_receive(imu_mqd, (char *)&rcv_imu_queue,
                       sizeof(struct imu_msg), 0);
      if (ret > 0) {
        counter++;
        imu_current.acc_x =  (float)rcv_imu_queue.acc_x / CONFIG_APPLICATION_IMU_FUSION_DEMO_AFS_SEL;
        imu_current.acc_y =  (float)rcv_imu_queue.acc_y / CONFIG_APPLICATION_IMU_FUSION_DEMO_AFS_SEL;
        imu_current.acc_z =  (float)rcv_imu_queue.acc_z / CONFIG_APPLICATION_IMU_FUSION_DEMO_AFS_SEL;
        imu_current.gyro_x = rcv_imu_queue.gyro_x / CONFIG_APPLICATION_IMU_FUSION_DEMO_FS_SEL;
        imu_current.gyro_y = rcv_imu_queue.gyro_y / CONFIG_APPLICATION_IMU_FUSION_DEMO_FS_SEL;
        imu_current.gyro_z = rcv_imu_queue.gyro_z / CONFIG_APPLICATION_IMU_FUSION_DEMO_FS_SEL;

        const FusionVector gyroscope = {imu_current.gyro_x, imu_current.gyro_y, imu_current.gyro_z};
        const FusionVector accelerometer = {imu_current.acc_x, imu_current.acc_y, imu_current.acc_z};
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, period);
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        // printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
        printf("%f %f %f %f %f %f\n", imu_current.acc_x, imu_current.acc_y, imu_current.acc_z,
               imu_current.gyro_x, imu_current.gyro_y, imu_current.gyro_z);

        // Write raw accelerometer and gyro values
        // snprintf(
        //     msg_buffer, sizeof(msg_buffer), "{%d %.03f %.03f %.03f %.03f %.03f %.03f}\n",
        //     counter, imu_current.acc_x, imu_current.acc_y, imu_current.acc_z, imu_current.gyro_x,
        //     imu_current.gyro_y, imu_current.gyro_z);

        snprintf(msg_buffer, sizeof(msg_buffer), "y%fyp%fpr%fr\n",
                 euler.angle.yaw, euler.angle.pitch, euler.angle.roll);

        ret = write(conn_fd, msg_buffer, sizeof(msg_buffer));

        if (ret <= 0) {
          printf("Client disconnected\n");
          client_listening = false;
          counter = 0;
          close(client_tcp_fd);
          start_network_socket();
        }

        memset(msg_buffer, 0, sizeof(msg_buffer));
      }
    }
  }

  return 1;
}

static int imu_task(int argc, FAR char *argv[]) {
  printf("Starting IMU task\n");
  struct imu_msg imu_data;
  mqd_t mqd = (mqd_t)*argv[1];

  fd = open("/dev/imu0", O_RDWR);
  if (fd < 0) {
    printf("Failed to open IMU\n");
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
    int ret = mq_send(mqd, (const void *)&imu_data, sizeof(imu_data), 0);
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
  mqd_t imu_mqd = (mqd_t)*argv[1];

  /* Start FusionAhrs */
  /* Calibration not available for now.*/
  static FusionAhrs ahrs;
  static FusionEuler euler;
  static FusionVector gyroscope = {0.0f, 0.0f, 0.0f};
  static FusionVector accelerometer = {0.0f, 0.0f, 1.0f};
  FusionAhrsInitialise(&ahrs);

  /* Output msg and utilities */
  int ret;
  char msg_buffer[100];
  memset(msg_buffer, 0, sizeof(msg_buffer));
  printf("Sensor Ops ready\n")

  while (1) {
    ret = mq_receive(imu_mqd, (char *) &rcv_imu_queue, sizeof(struct imu_msg), 0);

    if (ret < 0) {
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
    gyroscope = {imu_current.gyro_x, imu_current.gyro_y, imu_current.gyro_z};
    accelerometer = {imu_current.acc_x, imu_current.acc_y, imu_current.acc_z};
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, period);
    euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

    /* Send data to offload task */
    snprintf(msg_buffer, sizeof(msg_buffer), "y%fyp%fpr%fr\n",
             euler.angle.yaw, euler.angle.pitch, euler.angle.roll);
    memset(msg_buffer, 0, sizeof(msg_buffer));
  }

  return 0;
}