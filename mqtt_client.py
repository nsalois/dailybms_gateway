import socketpool
import wifi
import adafruit_minimqtt.adafruit_minimqtt as MQTT
from secrets import secrets

class MQTTClient:
    def __init__(self, bms):
        self.bms = bms
        self.mqtt_broker = secrets['mqtt_broker']
        self.mqtt_user = secrets['mqtt_user']
        self.mqtt_password = secrets['mqtt_password']
        self.pool = socketpool.SocketPool(wifi.radio)
        #self.socket_timeout=10  # Set socket timeout to 10 seconds
        self.mqtt_client = MQTT.MQTT(
            broker=self.mqtt_broker,
            username=self.mqtt_user,
            password=self.mqtt_password,
            socket_pool=self.pool,
            client_id="qtpy_bms"
        )
        self.mqtt_client.on_connect = self.connect
        self.mqtt_client.on_disconnect = self.disconnect
        self.mqtt_client.on_subscribe = self.subscribe
        self.mqtt_client.on_publish = self.publish
        self.mqtt_client.on_message = self.message

        self.charge_mos_state = None
        self.discharge_mos_state = None
        self.bms_reset_state = None

    def connect(self, client, userdata, flags, rc):
        print("Connected to MQTT broker!")
        self.mqtt_client.subscribe("bms/command/chargeMOS")
        self.mqtt_client.subscribe("bms/command/dischargeMOS")
        self.mqtt_client.subscribe("bms/command/resetBMS")

    def disconnect(self, client, userdata, rc):
        print("Disconnected from MQTT broker!")

    def subscribe(self, client, userdata, topic, granted_qos):
       return #print(f"Subscribed to {topic} with QoS {granted_qos}")

    def publish(self, client, userdata, topic, pid):
       return #print(f"Published to {topic} with PID {pid}")

    def message(self, client, topic, message):
        print(f"Received message on {topic}: {message}")
        if topic == "bms/command/chargeMOS":
            new_charge_state = message.lower() == "true"
            if new_charge_state != self.charge_mos_state:
                self.bms.setChargeMOS(new_charge_state)
                self.charge_mos_state = new_charge_state
        elif topic == "bms/command/dischargeMOS":
            new_discharge_state = message.lower() == "true"
            if new_discharge_state != self.discharge_mos_state:
                self.bms.setDischargeMOS(new_discharge_state)
                self.discharge_mos_state = new_discharge_state
        elif topic == "bms/command/resetBMS":
            new_reset_state = message.lower() == "true"
            if new_reset_state != self.bms_reset_state:
                if new_reset_state:
                    self.bms.setBmsReset()
                self.bms_reset_state = new_reset_state

    def connect_to_broker(self):
        print("Connecting to MQTT broker...")
        self.mqtt_client.connect()

    def publish_message(self, topic, message):
        if message is None:
            message = "NNNNNNN"  # Provide a default value if message is None
        self.mqtt_client.publish(topic, str(message))