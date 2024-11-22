import time
import json
import board
import busio
import wifi
from daly_bms_uart import Daly_BMS_UART
from mqtt_client import MQTTClient
from secrets import secrets

# Initialize UART
uart = busio.UART(board.TX, board.RX, baudrate=9600)

# Initialize BMS
bms = Daly_BMS_UART(uart)

# Connect to WiFi
print("Connecting to WiFi...")
wifi.radio.connect(secrets['ssid'], secrets['password'])
print("Connected to WiFi!")

# Initialize MQTT Client
mqtt_client = MQTTClient(bms)
mqtt_client.connect_to_broker()

# Initialize the non-blocking timer
last_time = time.monotonic()
interval = 10  # Interval in seconds

# Main loop
while True:
    # Update the MQTT client
    mqtt_client.mqtt_client.loop(1)

    #Non-blocking timer
    current_time = time.monotonic()
    if current_time - last_time >= interval:
        last_time = current_time

        try:
            
            if not bms.getTotalVoltageCurrentSOC():
                print("Failed to get Total Voltage, Current, and SOC")
            if not bms.getPackTemp():
                print("Failed to get Pack Temperature")
            if not bms.getMinMaxCellVoltage():
                print("Failed to get Min/Max Cell Voltage")
            if not bms.getDischargeChargeMosStatus():
                print("Failed to get Discharge/Charge MOS Status")
            if not bms.getStatusInfo():
                print("Failed to get Status Info")
            if not bms.getCellVoltages():
                print("Failed to get Cell Voltages")
            if not bms.getCellBalanceState():
                print("Failed to get Cell Balance State")
            if not bms.getActiveFaults():
                print("Failed to get Active Faults")
            
        except Exception as e:
            print(f"An error occurred3: {e}")

        bms.firstpass = True

        # Print BMS data
        print(f"chargeDischargeStatus:       {bms.get['chargeDischargeStatus']}")
        print(f"Package Temperature (C):     {bms.get['tempAverage']}")
        print(f"Highest Cell Voltage:        #{bms.get['maxCellVNum']} with voltage {bms.get['maxCellmV'] / 1000}")
        print(f"Lowest Cell Voltage:         #{bms.get['minCellVNum']} with voltage {bms.get['minCellmV'] / 1000}")
        print(f"Number of Cells:             {bms.get['numberOfCells']}")
        print(f"Number of Temp Sensors:      {bms.get['numOfTempSensors']}")
        print(f"BMS Chrg / Dischrg Cycles:   {bms.get['bmsCycles']}")
        print(f"BMS Heartbeat:               {bms.get['bmsHeartBeat']}")  # cycle 0-255
        print(f"Discharge MOSFet Status:     {bms.get['disChargeFetState']}")
        print(f"Charge MOSFet Status:        {bms.get['chargeFetState']}")
        print(f"Remaining Capacity mAh:      {bms.get['resCapacitymAh']}")
        print(f"Total Voltage:               {bms.get['packVoltage']}V")
        print(f"Collected Voltage:           {bms.get['collectedVoltage']}V")
        print(f"Current:                     {bms.get['packCurrent']}A")
        print(f"SOC:                         {bms.get['packSOC']}%")
        print(f"Pack Thresholds: Max1: {bms.get['maxPackThreshold1']}, Max2: {bms.get['maxPackThreshold2']},  Min1: {bms.get['minPackThreshold1']}, Min2: {bms.get['minPackThreshold2']}")
        print(f"Cell Thresholds: Max1: {bms.get['maxCellThreshold1']}, Max2: {bms.get['maxCellThreshold2']}, Min1: {bms.get['minCellThreshold1']}, Min2: {bms.get['minCellThreshold2']}")

        # Print cell voltages
        for i in range(bms.get["numberOfCells"]):
            print(f"Cell {i + 1} Voltage:        {bms.get['cellVmV'][i]} mV")

        # Print cell balancing state
        for i in range(bms.get["numberOfCells"]):
            print(f"Cell {i + 1} Balance State:  {bms.get['cellBalanceState'][i]}")
        
        # Print failure codes
        active_failures = [info["description"] for info in bms.failure_codes.values() if info["active"]]
        if active_failures:
            print("Active Failures:")
            for failure in active_failures:
                print(f"  {failure}: {bms.failure_codes[next(key for key, value in bms.failure_codes.items() if value['description'] == failure)]['active']}")
        else:
            print("No active failures.")
        print("")
        print("--------------------------------------------------")
        print("")

        # Collect BMS data
        bms_data = { 
            "chargeDischargeStatus": bms.get['chargeDischargeStatus'],
            "tempAverage": bms.get['tempAverage'],
            "maxCellVNum": bms.get['maxCellVNum'],
            "maxCellmV": bms.get['maxCellmV'] / 1000,
            "minCellVNum": bms.get['minCellVNum'],
            "minCellmV": bms.get['minCellmV'] / 1000,
            "numberOfCells": bms.get['numberOfCells'],
            "numOfTempSensors": bms.get['numOfTempSensors'],
            "bmsCycles": bms.get['bmsCycles'],
            "bmsHeartBeat": bms.get['bmsHeartBeat'],
            "disChargeFetState": bms.get['disChargeFetState'],
            "chargeFetState": bms.get['chargeFetState'],
            "resCapacitymAh": bms.get['resCapacitymAh'],
            "packVoltage": bms.get['packVoltage'],
            "collectedVoltage": bms.get['collectedVoltage'],
            "packCurrent": bms.get['packCurrent'],
            "packSOC": bms.get['packSOC'],
            "maxPackThreshold1": bms.get['maxPackThreshold1'],
            "maxPackThreshold2": bms.get['maxPackThreshold2'],
            "minPackThreshold1": bms.get['minPackThreshold1'],
            "minPackThreshold2": bms.get['minPackThreshold2'],
            "maxCellThreshold1": bms.get['maxCellThreshold1'],
            "maxCellThreshold2": bms.get['maxCellThreshold2'],
            "minCellThreshold1": bms.get['minCellThreshold1'],
            "minCellThreshold2": bms.get['minCellThreshold2'],
            "cellVmV": [bms.get['cellVmV'][i] for i in range(bms.get["numberOfCells"])],
            "cellBalanceState": [bms.get['cellBalanceState'][i] for i in range(bms.get["numberOfCells"])],
            "activeFaults": [info["description"] for info in bms.failure_codes.values() if info["active"]]
        }

        # Convert to JSON string
        bms_data_json = json.dumps(bms_data)

        # Publish JSON string
        mqtt_client.publish_message("bms/data", bms_data_json)