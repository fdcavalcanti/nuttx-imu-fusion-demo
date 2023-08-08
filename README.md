
# IMU Fusion Demo

This project uses the NuttX RTOS process accelerometer data using the Attitude And Heading Reference System (AHRS) algorithm [Fusion](https://github.com/xioTechnologies/Fusion) and transmits yaw, pitch and roll information through MQTT. On the other side, a Python scripts allows you to visualize this information using a 3D application, based on [PyTeapot](https://github.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation).


![Functional View](https://github.com/fdcavalcanti/nuttx-imu-fusion-demo/blob/main/files/output_functional.png)

![Implementation](https://github.com/fdcavalcanti/nuttx-imu-fusion-demo/blob/main/files/implementation_overview.png)


## Deployment

To compile on NuttX through menuconfig, add the `nuttx` directory as a symbolic link on `nuttx-apps`.
```bash
  $ cd ~/nuttxspace/nuttx-apps
  $ ln -s ~/imu-nuttx-fusion-demo/nuttx FusionDemo
```
Run `make clean` and it should appear on Application Configuration under Fusion DEMO.

The important configuration options are:

* Accelerometer sensitivity (it was tested using MPU6050)
* Sample rate in milliseconds (defaults to 10 Hz)
* MQTT Broker IP, port and topic

<img src="https://github.com/fdcavalcanti/nuttx-imu-fusion-demo/blob/main/files/menuconfig.png" width="700" />

On the computer side, the best option to run the 3D visualizer or the MQTT dump script is to create a Python virtual environment and install the required packages:
```bash
  $ cd ~/imu-nuttx-fusion-demo
  $ python3 -m venv venv
  $ source venv/bin/activate
  $ pip3 install -r requirements.txt
  $ python3 scripts/view_3d.py
  $ python3 scripts/dump_mqtt.py
```

**_NOTE:_** A MQTT broker is required to run this application. I recommend to use Mosquitto, simple to install and to start running.