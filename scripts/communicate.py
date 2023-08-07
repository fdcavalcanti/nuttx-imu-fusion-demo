import serial
import socket
from abc import ABC, abstractmethod

from paho.mqtt import client as mqtt_client
import paho.mqtt.subscribe as mqtt_subscribe


class BaseDatastream(ABC):
    @abstractmethod
    def connect(self):
        pass

    @abstractmethod
    def read(self):
        pass


class SerialConnection(BaseDatastream):
    def __init__(self, port: str = "/dev/ttyUSB0", baud_rate: int = 115200):
        self.port = port
        self.baud_rate = baud_rate
        self.serial_port = None
    
    def connect(self):
        self.serial_port = serial.Serial(self.port, self.baud_rate)
        self.serial_port.reset_input_buffer()

    def read(self):
        return self.serial_port.readline().decode("UTF-8").replace("\n", "")


class MQTTConnection(BaseDatastream):
    def __init__(self, broker_ip: str = "192.168.0.4", broker_port: int = 5000, broker_topic: str = "imu"):
        self.broker_ip = broker_ip
        self.broker_port = broker_port
        self.broker_topic = broker_topic
        print(f"Subscribing to {self.broker_ip} on topic {self.broker_topic}")
        self.connect()

    def connect(self):
        pass

    def read(self):
        msg = mqtt_subscribe.simple(self.broker_topic, hostname=self.broker_ip, port = self.broker_port)
        return msg.payload.decode("UTF-8").replace("\n", "")


class TCPSocketConnection(BaseDatastream):
    def __init__(self, tcp_ip: str = "192.168.0.3", tcp_port = 5000):
        self.tcp_ip = tcp_ip
        self.tcp_port = tcp_port
        self.sock = None

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.tcp_ip, self.tcp_port))

    def read(self):
        data = self.sock.recv(1024)
        line = data.decode("UTF-8").replace("\n", "")
        return line

