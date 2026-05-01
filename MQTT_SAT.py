import paho.mqtt.client as mqtt

MQTT_BROKER = "mqtt.ics.ele.tue.nl"

# Robot 41
TOPIC_41_SEND = "/pynqbridge/41/send"
TOPIC_41_RECV = "/pynqbridge/41/recv"

# Robot 80
TOPIC_80_SEND = "/pynqbridge/80/send"
TOPIC_80_RECV = "/pynqbridge/80/recv"


def on_connect(client, userdata, flags, reason_code, properties=None):
    if reason_code == 0:
        print("✅ Connected")

        # listen to BOTH robots
        client.subscribe(TOPIC_41_SEND)
        client.subscribe(TOPIC_80_SEND)

        print("🎧 Listening to both robots")
    else:
        print("❌ Connection failed")


def on_message(client, userdata, msg):
    data = msg.payload  # KEEP RAW BYTES

    print(f"\n📩 {msg.topic} ({len(data)} bytes)")

    # 🚀 Forward correctly
    if msg.topic == TOPIC_41_SEND:
        sender.publish(TOPIC_80_RECV, data)
        print("➡️ 41 → 80")

    elif msg.topic == TOPIC_80_SEND:
        sender.publish(TOPIC_41_RECV, data)
        print("⬅️ 80 → 41")


# Listener client
listener = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
listener.username_pw_set("robot_41_1", "t7gIhbJF")
listener.on_connect = on_connect
listener.on_message = on_message

# Sender client
sender = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
sender.username_pw_set("robot_80_1", "OQfY8Km2")
sender.connect(MQTT_BROKER, 1883, 60)
sender.loop_start()

# Start system
listener.connect(MQTT_BROKER, 1883, 60)
listener.loop_forever()
