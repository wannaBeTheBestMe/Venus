import paho.mqtt.client as mqtt
import time

# Connection details
MQTT_BROKER = "mqtt.ics.ele.tue.nl"

USERNAME = "robot_41_1"
PASSWORD = "t7gIhbJF"

PUBLISH_TOPIC = "/pynqbridge/41/recv"

# Setup MQTT client
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

client.username_pw_set(USERNAME, PASSWORD)

# Connect
print(f"Connecting to {MQTT_BROKER}...")

client.connect(MQTT_BROKER, 1883, 60)

client.loop_start()

time.sleep(1)

print("CONNECTED!")

# Main loop
while True:

    message_text = input("ENTER COMMAND: ")

    print(f"Sending '{message_text}'")

    client.publish(PUBLISH_TOPIC, message_text)

    print("Message sent!\n")

    time.sleep(0.2)