/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <errno.h>
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
#include <mqtt.h>

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
 * Private Types
 ****************************************************************************/

struct mqttc_cfg_s
{
  struct mqtt_client client;
  FAR const char *host;
  FAR const char *port;
  FAR const char *topic;
  FAR const char *msg;
  FAR const char *id;
  uint8_t sendbuf[CONFIG_EXAMPLES_MQTTC_TXSIZE];
  uint8_t recvbuf[CONFIG_EXAMPLES_MQTTC_RXSIZE];
  uint32_t tmo;
  uint8_t flags;
  uint8_t qos;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

static pid_t accelerometer_pid;
static pid_t offload_pid;
static pid_t sensor_ops_pid;
static bool run;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int offload_task(int argc, FAR char *argv[]);
static int imu_task(int argc, FAR char *argv[]);
static int sensor_ops_task(int argc, FAR char *argv[]);
/* MQTT Related */
static FAR void *client_refresher(FAR void *data);
static int initserver(FAR const struct mqttc_cfg_s *cfg);


/****************************************************************************
 * main
 ****************************************************************************/

int main(int argc, FAR char *argv[]) {
  printf("Starting Fusion Demo\n");
  char *child_argv[3];
  struct mq_attr acc_attr, sensor_ops_attr;
  mqd_t acc_mqd, sensor_ops_mqd;
  run = false;

  printf("Opening message queue channels\n");

  /* IMU Read Task message queue */
  acc_attr.mq_flags = 0;
  acc_attr.mq_maxmsg = 2;
  acc_attr.mq_msgsize = sizeof(struct imu_msg);
  acc_attr.mq_curmsgs = 0;

  acc_mqd = mq_open("IMU Queue", O_RDWR | O_CREAT, 0666, &acc_attr);

  if (acc_mqd == (mqd_t)-1)
    {
      printf("Failed to start IMU data queue\n");
      return 1;
    }

  /* Sensor Ops Task message queue */
  sensor_ops_attr.mq_flags = 0;
  sensor_ops_attr.mq_maxmsg = 2;
  sensor_ops_attr.mq_msgsize = sizeof(FusionEuler);
  sensor_ops_attr.mq_curmsgs = 0;

  sensor_ops_mqd = mq_open("Sensor Ops Queue", O_RDWR | O_CREAT, 0666, &sensor_ops_attr);

  if (sensor_ops_mqd == (mqd_t)-1)
    {
      printf("Failed to start Sensor Ops queue\n");
      mq_close(acc_mqd);
      return 1;
    } 

  child_argv[0] = (char *)&acc_mqd;
  child_argv[1] = (char *)&sensor_ops_mqd;
  child_argv[2] = NULL;

  /* Create tasks */
  offload_pid = task_create(
      "Offload Task", 120, CONFIG_APPLICATION_IMU_FUSION_DEMO_STACKSIZE,
      offload_task, (char *const *)child_argv);

  if (offload_pid < 0)
    {
      printf("Failed to create Offload task\n");
    }

  int counter = 0;
  while(!run & (counter < 10))
    {
      usleep(1E6);
      printf("Waiting for connection (%d)\r", counter);
      counter++;
    }

  if (!run)
    {
      printf("\nExiting\n");
      task_delete(offload_pid);
      return -1;
    }

  accelerometer_pid = task_create(
      "IMU Task", 120, CONFIG_APPLICATION_IMU_FUSION_DEMO_STACKSIZE,
      imu_task, (char *const *)child_argv);

  if (accelerometer_pid < 0)
    {
      printf("Failed to create IMU task\n");
    }

  sensor_ops_pid = task_create(
      "Sensor Ops Task", 110, CONFIG_APPLICATION_IMU_FUSION_DEMO_STACKSIZE,
      sensor_ops_task, (char *const *)child_argv);

  if (sensor_ops_pid < 0)
    {
      printf("Failed to create Sensor Ops task\n");
    }

  return 0;
}


static int offload_task(int argc, FAR char *argv[]) {
  printf("Starting offload task.. \n");

  int ret;
  int sockfd;
  int timeout = 100;
  pthread_t thrdid;
  mqd_t mqd_offload = (mqd_t)* argv[2];
  static FusionEuler euler;
  enum MQTTErrors mqtterr;
  char buffer[MQTT_MSG_LEN];
  memset(&buffer, 0, sizeof(buffer));

  /* Start MQTT Publisher */

 struct mqttc_cfg_s mqtt_cfg =
    {
      .host = CONFIG_APPLICATION_IMU_FUSION_DEMO_MQTT_BROKER_IP,
      .port = CONFIG_APPLICATION_IMU_FUSION_DEMO_MQTT_BROKER_PORT,
      .topic = CONFIG_APPLICATION_IMU_FUSION_DEMO_MQTT_TOPIC,
      .msg = "",
      .flags = MQTT_CONNECT_CLEAN_SESSION,
      .tmo = 400,
      .id = NULL,
      .qos = MQTT_PUBLISH_QOS_0,
    };

  sockfd = initserver(&mqtt_cfg);
  if (sockfd < 0)
    {
      printf("Failed to start network\n");
      return -1;
    }

  mqtterr = mqtt_init(&mqtt_cfg.client, sockfd,
                      mqtt_cfg.sendbuf, sizeof(mqtt_cfg.sendbuf),
                      mqtt_cfg.recvbuf, sizeof(mqtt_cfg.recvbuf),
                      NULL);
  if (mqtterr != MQTT_OK)
    {
      printf("ERRPR! mqtt_init() failed.\n");
      close(sockfd);
      return -1;
    }

  mqtterr = mqtt_connect(&mqtt_cfg.client, mqtt_cfg.id,
                         NULL, /* Will topic */
                         NULL, /* Will message */
                         0,    /* Will message size */
                         NULL, /* User name */
                         NULL, /* Password */
                         mqtt_cfg.flags, mqtt_cfg.tmo);

  if (mqtterr != MQTT_OK)
    {
      printf("ERROR! mqtt_connect() failed\n");
      close(sockfd);
      return -1;
    }

  if (mqtt_cfg.client.error != MQTT_OK)
    {
      printf("error: %s\n", mqtt_error_str(mqtt_cfg.client.error));
      close(sockfd);
      return -1;
    }
  else
    {
      printf("Connected to broker\n");
      run = true;
    }

  /* Start a thread to refresh the client (handle egress and ingree client
   * traffic)
   */

  if (pthread_create(&thrdid, NULL, client_refresher, &mqtt_cfg.client))
    {
      printf("ERROR pthread_create() failed.\n");
      close(sockfd);
      return -1;
    }

  while (!mqtt_cfg.client.event_connect && --timeout > 0)
    {
     usleep(10000);
    }

  if (timeout == 0)
    {
      pthread_cancel(thrdid);
      return -1;
    }

  printf("Data offload starts\n");

  while (1)
    {
      ret = mq_receive(mqd_offload, (char *) &euler, sizeof(euler), 0);

      if (ret > 0) 
        {
          snprintf(buffer, sizeof(buffer), "y%fyp%fpr%fr\n", 
                   euler.angle.yaw, euler.angle.pitch, euler.angle.roll);
          mqtterr = mqtt_publish(&mqtt_cfg.client, mqtt_cfg.topic,
                                 buffer, strlen(buffer),
                                 mqtt_cfg.qos);

          if (mqtterr != MQTT_OK)
            {
              printf("ERROR! mqtt_publish() failed\n");
            }
          /* Use for debug on serial */  
          // printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
          memset(&buffer, 0, sizeof(buffer));
        }

  }

  return 1;
}

static int imu_task(int argc, FAR char *argv[]) {
  printf("Starting IMU task.. ");

  struct imu_msg imu_data;
  mqd_t mqd_imu = (mqd_t)*argv[1];

  int fd = open("/dev/imu0", O_RDWR);
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

  printf("done\n");

  while (1) {
    read_imu(fd, &imu_data);
    int ret = mq_send(mqd_imu, (const void *) &imu_data, sizeof(imu_data), 0);
    if (ret < 0) {
      printf("Failed to send message\n");
    }
    usleep(CONFIG_APPLICATION_IMU_FUSION_DEMO_SAMPLE_RATE_MS*1000);
  }
  return 0;
}

static int sensor_ops_task(int argc, FAR char *argv[]) {
  printf("Starting Sensor Ops task.. ");

  /* IMU data and message queue fd */
  struct imu_msg rcv_imu_queue = {0};
  struct imu_msg_float imu_current = {0};
  mqd_t mqd_imu = (mqd_t)*argv[1];
  mqd_t mqd_offload = (mqd_t)*argv[2];

  /* Start FusionAhrs */
  /* Calibration not available for now.*/
  static FusionAhrs ahrs;
  // static FusionEuler euler;
  FusionAhrsInitialise(&ahrs);

  /* Utilities */
  int ret;
  const float period = CONFIG_APPLICATION_IMU_FUSION_DEMO_SAMPLE_RATE_MS / 1000.0f;

  printf("done\n");

  while (1) {
    ret = mq_receive(mqd_imu, (char *) &rcv_imu_queue, sizeof(struct imu_msg), 0);

    if (ret < 0)
      {
        printf("mq_receive imu_mqd ret: %d\n", ret);
        continue;
      }

    /* Convert accelerometer and gyro values */
    imu_current.acc_x =  rcv_imu_queue.acc_x / (float)CONFIG_APPLICATION_IMU_FUSION_DEMO_AFS_SEL;
    imu_current.acc_y =  rcv_imu_queue.acc_y / (float)CONFIG_APPLICATION_IMU_FUSION_DEMO_AFS_SEL;
    imu_current.acc_z =  rcv_imu_queue.acc_z / (float)CONFIG_APPLICATION_IMU_FUSION_DEMO_AFS_SEL;
    imu_current.gyro_x = rcv_imu_queue.gyro_x / (float)CONFIG_APPLICATION_IMU_FUSION_DEMO_FS_SEL;
    imu_current.gyro_y = rcv_imu_queue.gyro_y / (float)CONFIG_APPLICATION_IMU_FUSION_DEMO_FS_SEL;
    imu_current.gyro_z = rcv_imu_queue.gyro_z / (float)CONFIG_APPLICATION_IMU_FUSION_DEMO_FS_SEL;

    /* Apply received values to Fusion and calculate yaw, pitch and roll */
    const FusionVector gyroscope = {{imu_current.gyro_x, imu_current.gyro_y, imu_current.gyro_z}};
    const FusionVector accelerometer = {{imu_current.acc_x, imu_current.acc_y, imu_current.acc_z}};
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, period);
    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

    printf("%f %f %f %f %f %f %f %f %f\n", imu_current.acc_x, imu_current.acc_y, 
    imu_current.acc_z, imu_current.gyro_x, imu_current.gyro_y, imu_current.gyro_z,
    euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
    /* Send data to offload task */
    ret = mq_send(mqd_offload, (const char *) &euler, sizeof(euler), 0);

    if (ret < 0)
      {
        printf("ERROR Failed to send data to offload task!\n");
      }

  }

  return 0;
}

/****************************************************************************
 * Name: client_refresher
 *
 * Description:
 *   The client's refresher. This function triggers back-end routines to
 *   handle ingress/egress traffic to the broker.
 *
 ****************************************************************************/

static FAR void *client_refresher(FAR void *data)
{
  while (1)
    {
      mqtt_sync((FAR struct mqtt_client *)data);
      usleep(100000U);
    }

  return NULL;
}

/****************************************************************************
 * Name: initserver
 *
 * Description:
 *   Resolve server's name and try to establish a connection.
 *
 ****************************************************************************/

static int initserver(FAR const struct mqttc_cfg_s *cfg)
{
  struct addrinfo hints;
  FAR struct addrinfo *servinfo;
  FAR struct addrinfo *itr;
  int fd;
  int ret;

  memset(&hints, 0, sizeof(hints));
  hints.ai_family  = AF_INET;
  hints.ai_socktype = SOCK_STREAM;

  printf("Connecting to %s:%s on topic %s...\n", cfg->host, cfg->port, cfg->topic);

  ret = getaddrinfo(cfg->host, cfg->port, &hints, &servinfo);
  if (ret != OK)
    {
      printf("ERROR! getaddrinfo() failed: %s\n", gai_strerror(ret));
      return -1;
    }

  itr = servinfo;
  do
    {
      fd = socket(itr->ai_family, itr->ai_socktype, itr->ai_protocol);
      if (fd < 0)
        {
          continue;
        }

      ret = connect(fd, itr->ai_addr, itr->ai_addrlen);
      if (ret == 0)
        {
          break;
        }

      close(fd);
      fd = -1;
    }
  while ((itr = itr->ai_next) != NULL);

  freeaddrinfo(servinfo);

  if (fd < 0)
    {
      printf("ERROR! Couldn't create socket\n");
      return -1;
    }

  ret = fcntl(fd, F_GETFL, 0);
  if (ret < 0)
    {
      printf("ERROR! fcntl() F_GETFL failed, errno: %d\n", errno);
      return -1;
    }

  ret = fcntl(fd, F_SETFL, ret | O_NONBLOCK);
  if (ret < 0)
    {
      printf("ERROR! fcntl() F_SETFL failed, errno: %d\n", errno);
      return -1;
    }

  return fd;
}
