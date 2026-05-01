import paho.mqtt.client as mqtt

# 1. Define your connection details
MQTT_BROKER = "mqtt.ics.ele.tue.nl"
USERNAME = "robot_41_1" # Your exact username
PASSWORD = "t7gIhbJF" # Your exact password

# THIS IS THE IMPORTANT PART: 
# You and your teammate MUST type the exact same thing here!
# Let's use the robot's actual output channel for testing.
LISTEN_TOPIC = "/pynqbridge/robot_31_1/send" 
#LISTEN_TOPIC = "/pynqbridge/#" 

# 2. What to do when we successfully connect
def on_connect(client, userdata, flags, reason_code, properties=None):
    if reason_code == 0:
        print(f"✅ Connected to TU/e Broker successfully!")
        client.subscribe(LISTEN_TOPIC)
        print(f"🎧 Now listening permanently to: '{LISTEN_TOPIC}'...")
        print(f"⏳ Waiting for your teammate to send something (Press Ctrl+C to stop)...")
    else:
        print(f"❌ Connection failed with code: {reason_code}")

# 3. What to do when a message arrives
def on_message(client, userdata, msg):
    received_text = msg.payload.decode("utf-8")
    # This will print whatever your teammate sends!
    print(f"\n📩 INCOMING MESSAGE from '{msg.topic}':\n   -> {received_text}")

# 4. Set up the client
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.username_pw_set(USERNAME, PASSWORD)
client.on_connect = on_connect
client.on_message = on_message

# 5. Connect and listen forever
print(f"Connecting to {MQTT_BROKER}...")
client.connect(MQTT_BROKER, 1883, 60)

# loop_forever() blocks the program and keeps it listening continuously.
# It will only stop if you press Ctrl+C in the terminal.
client.loop_forever()