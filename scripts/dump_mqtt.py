from paho.mqtt import client as mqtt_client

BROKER_IP = "192.168.0.4"
BROKER_PORT = 5000
BROKER_TOPIC = "imu"
CLIENT_ID = "test_client"


def connect_hook(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
    else:
        print("Failed to connect, return code %d\n", rc)


def message_hook(client, userdata, msg):
    print(f"Received '{msg.payload.decode()}' from '{msg.topic}' topic")


def subscribe(client: mqtt_client):
    client.subscribe(BROKER_TOPIC)
    client.on_message = message_hook


if __name__ == "__main__":
    client = mqtt_client.Client(CLIENT_ID)
    client.on_connect = connect_hook
    client.connect(BROKER_IP, BROKER_PORT)

    subscribe(client)
    client.loop_forever()
