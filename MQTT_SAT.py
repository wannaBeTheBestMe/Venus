import paho.mqtt.client as mqtt

MQTT_BROKER = "mqtt.ics.ele.tue.nl"

# Robot 41 (LISTEN)
USERNAME41 = "robot_41_1"
PASSWORD41 = "t7gIhbJF"
LISTEN_TOPIC41 = "/pynqbridge/41/send"

# Robot 80 (SEND)
USERNAME80 = "robot_80_1"
PASSWORD80 = "OQfY8Km2"
PUBLISH_TOPIC80 = "/pynqbridge/80/recv"


def on_connect(client, userdata, flags, reason_code, properties=None):
    if reason_code == 0:
        print("✅ Connected!")
        client.subscribe(LISTEN_TOPIC41)
        print(f"🎧 Listening to {LISTEN_TOPIC41}")
    else:
        print("❌ Connection failed")


def on_message(client, userdata, msg):
    received_text = msg.payload.decode("utf-8")

    print(f"\n📩 Received: {received_text}")

    # 🔥 Forward to robot 80
    sender_client.publish(PUBLISH_TOPIC80, received_text)
    print(f"🚀 Forwarded to {PUBLISH_TOPIC80}")


# Client for listening
listener_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
listener_client.username_pw_set(USERNAME41, PASSWORD41)
listener_client.on_connect = on_connect
listener_client.on_message = on_message

# Separate client for sending
sender_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
sender_client.username_pw_set(USERNAME80, PASSWORD80)
sender_client.connect(MQTT_BROKER, 1883, 60)
sender_client.loop_start()

# Start listener
listener_client.connect(MQTT_BROKER, 1883, 60)
listener_client.loop_forever()