import paho.mqtt.client as mqtt
import time

MQTT_BROKER = "mqtt.ics.ele.tue.nl"

TOPIC_41_SEND = "/pynqbridge/41/send"
TOPIC_41_RECV = "/pynqbridge/41/recv"

TOPIC_80_SEND = "/pynqbridge/80/send"
TOPIC_80_RECV = "/pynqbridge/80/recv"

BRIDGE_TAG = b"[B]"


def on_connect(client, userdata, flags, reason_code, properties=None):
    print("Connected")
    client.subscribe(TOPIC_41_SEND)
    client.subscribe(TOPIC_80_SEND)


def on_message(client, userdata, msg):
    data = msg.payload

    # prevent infinite loop
    if data.startswith(BRIDGE_TAG):
        return

    new_data = BRIDGE_TAG + data

    if msg.topic == TOPIC_41_SEND:
        client.publish(TOPIC_80_RECV, new_data, qos=1)
        print("41 → 80")

    elif msg.topic == TOPIC_80_SEND:
        client.publish(TOPIC_41_RECV, new_data, qos=1)
        print("80 → 41")


client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.username_pw_set("robot_41_1", "t7gIhbJF")

client.on_connect = on_connect
client.on_message = on_message

client.connect(MQTT_BROKER, 1883, 60)
client.loop_start()

while True:
    time.sleep(1)
