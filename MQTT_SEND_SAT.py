import paho.mqtt.client as mqtt
import time

# 1. Define connection details (These must match your listener exactly!)
MQTT_BROKER = "mqtt.ics.ele.tue.nl"
USERNAME = "robot_41_1" 
PASSWORD = "t7gIhbJF" # Make sure they put the real password here!

# The exact same topic your laptop is currently listening to
PUBLISH_TOPIC = "/pynqbridge/41/recv"

# 2. Set up the client
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.username_pw_set(USERNAME, PASSWORD)

# 3. Connect to the broker
print(f"Connecting to {MQTT_BROKER}...")
client.connect(MQTT_BROKER, 1883, 60)

# Start a background thread to handle the network traffic safely
client.loop_start()
time.sleep(1) # Give it a quick second to establish the connection

# 4. Send the message!
message_text = str(input("ENTER COMMAND"))
print(f"🚀 Sending message to '{PUBLISH_TOPIC}'...")

# This is the actual line that fires the data into the MQTT group chat
client.publish(PUBLISH_TOPIC, message_text)

# Wait a moment to ensure the message physically leaves the laptop before hanging up
time.sleep(2)
client.loop_stop()
print("✅ Message sent and script closed.")