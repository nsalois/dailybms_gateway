"""
Daly BMS UART Interface

This library provides a CircuitPython interface for communicating with a Daly Battery Management System (BMS) over UART.
The interface allows you to read various parameters from the BMS, set configurations, and handle commands via MQTT.

Features:
- Read pack temperature, cell voltages, and cell temperature
- Get min/max cell voltage and discharge/charge MOS status
- Get status information and active faults
- Set State of Charge (SOC)
- Set discharge and charge MOS states
- Reset BMS
- MQTT integration for publishing all BMS data and for remote command handling

Frame Structure:
Each frame consists of the following components:
1. Frame Header: A fixed byte indicating the start of the frame (0xA5).
2. Communication Module Address: The address of the communication module (0x80).
3. Data ID: The identifier for the type of data being sent.
4. Data Length: The length of the data being sent (8 bytes).
5. Data Content: The actual data being sent, which is 8 bytes.
6. Checksum: A checksum byte for error detection.

Example Command Frame:
Byte 0: Frame Header (0xA5)
Byte 1: Communication Module Address (0x80)
Byte 2: Data ID (e.g., 0x90 for VOUT_IOUT_SOC)
Byte 3: Data Length (0x08)
Byte 4-11: Data Content (e.g., [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
Byte 12: Checksum (calculated as the sum of the first 12 bytes, masked to 8 bits)

Example Response Frame:
Byte 0: Frame Header (0xA5)
Byte 1: Communication Module Address (0x80)
Byte 2: Data ID (e.g., 0x90 for VOUT_IOUT_SOC)
Byte 3: Data Length (0x08)
Byte 4-11: Data Content (e.g., [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0])
Byte 12: Checksum (calculated as the sum of the first 12 bytes, masked to 8 bits)

Usage:
1. Initialize the UART and BMS interface:
    import board
    import busio
    from daly_bms_uart import Daly_BMS_UART

    uart = busio.UART(board.TX, board.RX, baudrate=9600)
    bms = Daly_BMS_UART(uart)

2. Use the BMS interface to read data and set configurations:
    if bms.getPackTemp():
        print(f"Pack Temperature: {bms.get['tempAverage']} °C")

3. Handle MQTT commands:
    while True:
        mqtt_client.loop()
        # Add your main loop logic here

License:
This project is licensed under the MIT License. See the LICENSE file for details.
"""

import time
import board
import busio
import re

START_BYTE      = 0xA5
HOST_ADDRESS    = 0x80
DATA_LENGTH     = 0x08
BUFFER_VALUE    = 0x00

NUMBEROFCELLS   = 10
XFER_BUFFER_LENGTH = 13
MIN_NUMBER_CELLS = 1
MAX_NUMBER_CELLS = 48
MIN_NUMBER_TEMP_SENSORS = 1
MAX_NUMBER_TEMP_SENSORS = 16

class Daly_BMS_UART:
    class COMMAND:
        VOUT_IOUT_SOC               = 0x90
        MIN_MAX_CELL_VOLTAGE        = 0x91
        MIN_MAX_TEMPERATURE         = 0x92
        DISCHARGE_CHARGE_MOS_STATUS = 0x93
        STATUS_INFO                 = 0x94
        CELL_VOLTAGES               = 0x95
        CELL_TEMPERATURE            = 0x96
        CELL_BALANCE_STATE          = 0x97
        FAILURE_CODES               = 0x98
        DISCHRG_FET                 = 0xD9
        CHRG_FET                    = 0xDA
        BMS_RESET                   = 0x00
        PACK_THRESHOLDS             = 0x5A
        CELL_THRESHOLDS             = 0x59
        READ_SOC                    = 0x61
        SET_SOC                     = 0x21
        BATTAEY_CODE                = 0x57
        BATTERY_DETAILS             = 0x53
        RATED_PARAMS                = 0x50

    def __init__(self, uart):
        self.uart = uart
        self.rx_done = True  # RX done flag
        self.firstpass = True  # First pass flag
        self.my_txBuffer = bytearray(XFER_BUFFER_LENGTH)
        self.my_rxBuffer = bytearray(XFER_BUFFER_LENGTH)
        self.my_txBuffer[0] = START_BYTE
        self.my_txBuffer[1] = HOST_ADDRESS
        self.my_txBuffer[3] = DATA_LENGTH
        self.get = {
            "packVoltage": 0.0,
            "collectedVoltage": 0.0,
            "packCurrent": 0.0,
            "packSOC": 0.0,
            "productionDate": "",
            "maxCellmV": 0.0,
            "maxCellVNum": 0,
            "minCellmV": 0.0,
            "minCellVNum": 0,
            "cellDiff": 0.0,
            "tempMax": 0,
            "tempMin": 0,
            "tempAverage": 0.0,
            "chargeDischargeStatus": "Idle",
            "chargeFetState": "off",
            "disChargeFetState": "off",
            "bmsHeartBeat": 0,
            "resCapacitymAh": 0,
            "capacity": 0,
            "numberOfCells": 0,
            "numOfTempSensors": 0,
            "chargeState": False,
            "loadState": False,
            "dIO": [False] * 8,
            "bmsCycles": 0,
            "cellVmV": [0.0] * MAX_NUMBER_CELLS,
            "cellTemperature": [0] * MAX_NUMBER_TEMP_SENSORS,
            "cellBalanceState": [False] * MAX_NUMBER_CELLS,
            "cellBalanceActive": False,
            "aDebug": "",
            "maxPackThreshold1": 0.0,
            "maxPackThreshold2": 0.0,
            "minPackThreshold1": 0.0,
            "minPackThreshold2": 0.0,
            "maxCellThreshold1": 0.0,
            "maxCellThreshold2": 0.0,
            "minCellThreshold1": 0.0,
            "minCellThreshold2": 0.0,
            "activeFaults": {}
        }
        self.failure_codes = {
            # Byte 0
            0: {"description": "cellOverVoltageStageOne", "active": False},
            1: {"description": "cellOverVoltageStageTwo", "active": False},
            2: {"description": "cellUnderVoltageStageOne", "active": False},
            3: {"description": "cellUnderVoltageStageTwo", "active": False},
            4: {"description": "packOverVoltageStageOne", "active": False},
            5: {"description": "packOverVoltageStageTwo", "active": False},
            6: {"description": "packUnderVoltageStageOne", "active": False},
            7: {"description": "packUnderVoltageStageTwo", "active": False},
            # Byte 1
            8: {"description": "chargeTempHighStageOne", "active": False},
            9: {"description": "chargeTempHighStageTwo", "active": False},
            10: {"description": "chargeTempLowStageOne", "active": False},
            11: {"description": "chargeTempLowStageTwo", "active": False},
            12: {"description": "dischargeTempHighStageOne", "active": False},
            13: {"description": "dischargeTempHighStageTwo", "active": False},
            14: {"description": "dischargeTempLowStageOne", "active": False},
            15: {"description": "dischargeTempLowStageTwo", "active": False},
            # Byte 2
            16: {"description": "chargeCurrentHighStageOne", "active": False},
            17: {"description": "chargeCurrentHighStageTwo", "active": False},
            18: {"description": "dischargeCurrentHighStageOne", "active": False},
            19: {"description": "dischargeCurrentHighStageTwo", "active": False},
            20: {"description": "socHighStageOne", "active": False},
            21: {"description": "socHighStageTwo", "active": False},
            22: {"description": "socLowStageOne", "active": False},
            23: {"description": "socLowStageTwo", "active": False},
            # Byte 3
            24: {"description": "cellVoltageDiffHighStageOne", "active": False},
            25: {"description": "cellVoltageDiffHighStageTwo", "active": False},
            26: {"description": "tempSensorDiffHighStageOne", "active": False},
            27: {"description": "tempSensorDiffHighStageTwo", "active": False},
            # Byte 4
            28: {"description": "chargeFETTempHigh", "active": False},
            29: {"description": "dischargeFETTempHigh", "active": False},
            30: {"description": "chargeFETTempSensorFail", "active": False},
            31: {"description": "dischargeFETTempSensorFail", "active": False},
            32: {"description": "chargeFETAdhesionFail", "active": False},
            33: {"description": "dischargeFETAdhesionFail", "active": False},
            34: {"description": "chargeFETBreakerFail", "active": False},
            35: {"description": "dischargeFETBreakerFail", "active": False},
            # Byte 5
            36: {"description": "afeAcquisitionFail", "active": False},
            37: {"description": "voltageSensorFail", "active": False},
            38: {"description": "tempSensorFail", "active": False},
            39: {"description": "eepromStorageFail", "active": False},
            40: {"description": "rtcClockFail", "active": False},
            41: {"description": "prechargeFail", "active": False},
            42: {"description": "vehicleCommFail", "active": False},
            43: {"description": "intranetCommFail", "active": False},
            # Byte 6
            44: {"description": "currentModuleFail", "active": False},
            45: {"description": "mainPressureDetectionModuleFail", "active": False},
            46: {"description": "shortCircuitProtectionFail", "active": False},
            47: {"description": "lowVoltageNoChargingFail", "active": False},
        }

    def barfRXBuffer(self):
        """
        Prints the contents of the RX buffer for debugging purposes.
        """
        print("<DALY-BMS DEBUG> RX Buffer: [", end="")
        for i in range(XFER_BUFFER_LENGTH):
            print(f",0x{self.my_rxBuffer[i]:02X}", end="")
        print("]")

    def sendCommand(self, cmdID, data=None):
        """
        Sends a command to the BMS.

        Parameters:
        cmdID (int): The command ID to send.
        data (list, optional): Optional data to send with the command (default is 8 bytes of 0x00).

        Raises:
        ValueError: If the length of data is not exactly 8 bytes.
        """
        if data is None:
            data = [0x00] * 8  # Default data is 8 bytes of 0x00

        # Ensure data is exactly 8 bytes
        if len(data) != 8:
            raise ValueError("Data must be exactly 8 bytes")
        
        # Wait until RX is done before sending a new command, unless it's the first pass
        if not self.firstpass:
            while not self.rx_done:
                time.sleep(0.1)  # Small delay to avoid busy waiting

        # Clear all incoming serial to avoid data collision
        while self.uart.in_waiting > 0:
            self.uart.read(1)

        # Update the TX buffer with the correct values
        self.my_txBuffer[2] = cmdID  # Data ID
        self.my_txBuffer[4:12] = bytearray(data)  # Data content

        # Calculate the checksum
        checksum = sum(self.my_txBuffer[:12]) & 0xFF  # Ensure checksum fits in 1 byte
        # Put it on the frame
        self.my_txBuffer[12] = checksum
        self.uart.write(self.my_txBuffer)
        self.rx_done = False  # Reset RX done flag after sending command
        self.firstpass = False
        
    def receiveBytes(self, timeout=1):
        """
        Receives bytes from the BMS.

        Parameters:
        timeout (int, optional): The timeout period in seconds (default is 1 second).

        Returns:
        bool: True if the expected number of bytes is received, False otherwise.
        """
        # Clear out the input buffer
        self.my_rxBuffer = bytearray(XFER_BUFFER_LENGTH)

        # Wait for the expected number of bytes to become available
        start_time = time.time()
        while self.uart.in_waiting < XFER_BUFFER_LENGTH:
            if time.time() - start_time > timeout:
                print("Receive timeout")
                self.rx_done = False  # Reset RX done flag
                return False
            time.sleep(0.001)  # Small delay to avoid busy waiting

        # Read bytes from the specified serial interface
        rxByteNum = self.uart.readinto(self.my_rxBuffer)

        # Make sure we got the correct number of bytes
        if rxByteNum != XFER_BUFFER_LENGTH:
            print("Incorrect number of bytes received")
            self.rx_done = False  # Reset RX done flag
            return False

        self.rx_done = True  # Set RX done flag
        self.barfRXBuffer()
        return True

    def validateChecksum(self):
        """
        Validates the checksum of the received data.

        Returns:
        bool: True if the checksum is valid, False otherwise.
        """
        # Calculate the checksum of the received data
        calculated_checksum = sum(self.my_rxBuffer[:12]) & 0xFF  # Ensure it is an 8-bit value
        received_checksum = self.my_rxBuffer[12]

        return calculated_checksum == received_checksum

    def setSOC(self, soc):
        """
        Sets the State of Charge (SOC) on the BMS.

        Parameters:
        soc (float): The SOC value to set (0.0 to 100.0).

        Returns:
        bool: True if the SOC is successfully set, False otherwise.
        """
        if soc < 0.0 or soc > 100.0:
            raise ValueError("SOC value must be between 0.0 and 100.0")

        # Convert the SOC to an integer value (0 to 1000)
        soc_int = int(soc * 10)

        # Send the command to set the SOC
        self.sendCommand(self.COMMAND.SET_SOC, data=[(soc_int >> 8) & 0xFF, soc_int & 0xFF] + [0x00] * 6)

        if not self.receiveBytes():
            return False

        # Check if the SOC was set successfully
        return self.validateChecksum()
    
    def getTotalVoltageCurrentSOC(self):
        """
        Retrieves the total voltage, current, and state of charge (SOC) from the BMS.

        Returns:
        bool: True if the data is successfully retrieved, False otherwise.
        """
        self.sendCommand(self.COMMAND.VOUT_IOUT_SOC)
        
        if not self.receiveBytes():
            return False
        
        # Parse the response
        self.get["packVoltage"] = ((self.my_rxBuffer[4] << 8) | self.my_rxBuffer[5]) * 0.1
        self.get["collectedVoltage"] = ((self.my_rxBuffer[6] << 8) | self.my_rxBuffer[7]) * 0.1
        self.get["packCurrent"] = (((self.my_rxBuffer[8] << 8) | self.my_rxBuffer[9]) - 30000) * 0.1
        self.get["packSOC"] = ((self.my_rxBuffer[10] << 8) | self.my_rxBuffer[11]) * 0.1
        
        return True

    def getPackTemp(self):
        """
        Retrieves the pack temperature from the BMS.

        Returns:
        bool: True if the data is successfully retrieved, False otherwise.
        """
        self.sendCommand(self.COMMAND.MIN_MAX_TEMPERATURE)
        if not self.receiveBytes():
            return False
        self.get["tempMax"] = self.my_rxBuffer[4] - 40
        self.get["tempMin"] = self.my_rxBuffer[6] - 40
        self.get["tempAverage"] = (self.get["tempMax"] + self.get["tempMin"]) / 2
        return True

    def getMinMaxCellVoltage(self):
        """
        Returns the highest and lowest individual cell voltage, and which cell is highest/lowest.
        Voltages are returned as floats with milliVolt precision (3 decimal places).

        Returns:
        bool: True on successful acquisition, False otherwise.
        """
        self.sendCommand(self.COMMAND.MIN_MAX_CELL_VOLTAGE)
        if not self.receiveBytes():
            return False
        self.get["maxCellmV"] = (self.my_rxBuffer[4] << 8) | self.my_rxBuffer[5]
        self.get["maxCellVNum"] = self.my_rxBuffer[6]
        self.get["minCellmV"] = (self.my_rxBuffer[7] << 8) | self.my_rxBuffer[8]
        self.get["minCellVNum"] = self.my_rxBuffer[9]
        self.get["cellDiff"] = self.get["maxCellmV"] - self.get["minCellmV"]
        return True

    def getDischargeChargeMosStatus(self):
        """
        Retrieves the discharge and charge MOS status from the BMS.

        Returns:
        bool: True if the data is successfully retrieved, False otherwise.
        """
        self.sendCommand(self.COMMAND.DISCHARGE_CHARGE_MOS_STATUS)
        if not self.receiveBytes():
            return False

        # Parse the response and update the `get` dictionary
        status_byte = self.my_rxBuffer[4]
        if (status_byte == 0x00):
            self.get["chargeDischargeStatus"] = "Idle"
        elif (status_byte == 0x01):
            self.get["chargeDischargeStatus"] = "Charge"
        elif (status_byte == 0x02):
            self.get["chargeDischargeStatus"] = "Discharge"

        self.get["chargeFetState"] = "on" if self.my_rxBuffer[5] == 0x01 else "off"
        self.get["disChargeFetState"] = "on" if self.my_rxBuffer[6] == 0x01 else "off"
        self.get["bmsHeartBeat"] = self.my_rxBuffer[7]
        self.get["resCapacitymAh"] = (
            (self.my_rxBuffer[8] << 24) |
            (self.my_rxBuffer[9] << 16) |
            (self.my_rxBuffer[10] << 8) |
            self.my_rxBuffer[11]
        )  

        return True

    def getStatusInfo(self):
        """
        Retrieves the status information from the BMS.

        Returns:
        bool: True if the data is successfully retrieved, False otherwise.
        """
        self.sendCommand(self.COMMAND.STATUS_INFO)

        if not self.receiveBytes():
            return False

        self.get["numberOfCells"] = self.my_rxBuffer[4]
        self.get["numOfTempSensors"] = self.my_rxBuffer[5]
        self.get["chargeState"] = self.my_rxBuffer[6]
        self.get["loadState"] = self.my_rxBuffer[7]

        # Parse the 8 bits into 8 booleans that represent the states of the Digital IO
        for i in range(8):
            self.get["dIO"][i] = bool(self.my_rxBuffer[8] & (1 << i))

        self.get["bmsCycles"] = (self.my_rxBuffer[9] << 8) | self.my_rxBuffer[10]

        return True

    def getCellVoltages(self):
        """
        Retrieves the cell voltages from the BMS.

        Returns:
        bool: True if the data is successfully retrieved, False otherwise.
        """
        self.sendCommand(self.COMMAND.CELL_VOLTAGES)
        
        # Initialize a buffer to accumulate the received data
        accumulated_data = bytearray()
        frames_needed = 4  # Hardcoded number of frames

        # Receive the required number of frames
        for frame in range(frames_needed):
            if frame < 4:
                self.rx_done = False
            if not self.receiveBytes():  # Increase timeout to 0.3 seconds
                return False
                
            # Check if the frame number is valid
            frame_number = self.my_rxBuffer[0]
            if frame_number == 0xFF:
                continue
            
            # Extract the 6 data bytes for cell voltages (Byte 1 to Byte 6)
            # Frame structure:
            # Byte 0: Frame header
            # Byte 1: Communication module address
            # Byte 2: Data ID
            # Byte 3: Data length
            # Byte 4: Frame number (0 to 15, 0xFF is invalid)
            # Byte 5-10: Cell voltages (each cell voltage is 2 bytes, 1 mV per unit)
            # Byte 11: Reserved
            # Byte 12: Checksum
            data_bytes = self.my_rxBuffer[5:11]
            accumulated_data.extend(data_bytes)

        total_bytes_needed = 20  # Hardcoded total bytes needed for 10 cells (2 bytes per cell)

        if len(accumulated_data) < total_bytes_needed:
            return False

        for i in range(10):  # Hardcoded number of cells
            if i * 2 + 1 < len(accumulated_data):
                # Each cell voltage is 2 bytes
                high_byte = accumulated_data[i * 2]
                low_byte = accumulated_data[i * 2 + 1]
                voltage = (high_byte << 8) | low_byte
                self.get['cellVmV'][i] = voltage
            else:
                break

        return True

    def getCellTemperature(self): 
        """
        Retrieves the cell temperatures from the BMS.

        Returns:
        bool: True if the data is successfully retrieved, False otherwise.
        """
        self.sendCommand(self.COMMAND.CELL_TEMPERATURE)

        if not self.receiveBytes():
            return False

        for i in range(self.get["numOfTempSensors"]):
            self.get["cellTemperature"][i] = self.my_rxBuffer[4 + i] - 40

        return True
    
    def getCellBalanceState(self):
        """
        Retrieves the cell balance state from the BMS.

        Returns:
        bool: True if the data is successfully retrieved, False otherwise.
        """
        self.sendCommand(self.COMMAND.CELL_BALANCE_STATE)

        if not self.receiveBytes():
            return False

        for i in range(self.get["numberOfCells"]):
            self.get["cellBalanceState"][i] = bool(self.my_rxBuffer[4 + (i // 8)] & (1 << (i % 8)))

        self.get["cellBalanceActive"] = self.my_rxBuffer[7]

        return True

    def reverse_bits(byte):
        """
        Reverses the bits in a byte.

        Parameters:
        byte (int): The byte to reverse.

        Returns:
        int: The byte with reversed bits.
        """
        result = 0
        for i in range(8):
            result = (result << 1) | (byte & 1)
            byte >>= 1
        return result

    def getActiveFaults(self):
        """
        Retrieves the active faults from the BMS.

        Returns:
        bool: True if the data is successfully retrieved, False otherwise.
        """
        self.sendCommand(self.COMMAND.FAILURE_CODES)

        if not self.receiveBytes():
            print("<BMS > Receive failed", self.COMMAND.FAILURE_CODES)
            self.barfRXBuffer()
            return False

        # Ensure the received buffer has the expected length
        if len(self.my_rxBuffer) < 13:
            print("Received buffer is too short")
            self.barfRXBuffer()
            return False

        # Extract the relevant data bytes (8 bytes starting from index 4)
        data = self.my_rxBuffer[4:12]

        # Convert data bytes to a binary string
        binary_data = ''.join(f'{byte:08b}' for byte in data)
        print(f"Binary Data: {binary_data}")

        # Update fault statuses based on the binary data
        for bit_number, fault_info in self.failure_codes.items():
            byte_index = bit_number // 8
            bit_index = bit_number % 8
            fault_info["active"] = bool(data[byte_index] & (1 << bit_index))

        # Return only the active faults with their descriptions
        self.activeFaults = {fault_info['description']: fault_info['active'] for fault_info in self.failure_codes.values() if fault_info['active']}
        print(f"Active Faults: {self.activeFaults}")

        return True
    
    def getPackVoltageThreshold(self):
        """
        Retrieves the pack voltage thresholds from the BMS.

        Returns:
        bool: True if the data is successfully retrieved, False otherwise.
        """
        if not self.sendCommand(self.COMMAND.PACK_THRESHOLDS):
            print("<BMS > Receive failed", self.COMMAND.PACK_THRESHOLDS)
            return False

        self.get["maxPackThreshold1"] = float((self.my_rxBuffer[4] << 8) | self.my_rxBuffer[5])
        self.get["maxPackThreshold2"] = float((self.my_rxBuffer[6] << 8) | self.my_rxBuffer[7])
        self.get["minPackThreshold1"] = float((self.my_rxBuffer[8] << 8) | self.my_rxBuffer[9])
        self.get["minPackThreshold2"] = float((self.my_rxBuffer[10] << 8) | self.my_rxBuffer[11])

        return True

    def getVoltageThreshold(self):
        """
        Retrieves the cell voltage thresholds from the BMS.

        Returns:
        bool: True if the data is successfully retrieved, False otherwise.
        """
        if not self.sendCommand(self.COMMAND.CELL_THRESHOLDS):
            print("<BMS > Receive failed", self.COMMAND.CELL_THRESHOLDS)
            return False

        self.get["maxCellThreshold1"] = float((self.my_rxBuffer[4] << 8) | self.my_rxBuffer[5])
        self.get["maxCellThreshold2"] = float((self.my_rxBuffer[6] << 8) | self.my_rxBuffer[7])
        self.get["minCellThreshold1"] = float((self.my_rxBuffer[8] << 8) | self.my_rxBuffer[9])
        self.get["minCellThreshold2"] = float((self.my_rxBuffer[10] << 8) | self.my_rxBuffer[11])

        return True

    def readCapacity(self):
        """
        Reads the capacity from the BMS.

        Returns:
        bool: True if the data is successfully retrieved, False otherwise.
        """
        self.sendCommand(self.COMMAND.RATED_PARAMS)
        
        if not self.receiveBytes():
            print("No data received in readCapacity()")
            return False

        # Unpack the received data
        capacity = (self.my_rxBuffer[4] << 24) | (self.my_rxBuffer[5] << 16) | (self.my_rxBuffer[6] << 8) | self.my_rxBuffer[7]
        cell_volt = (self.my_rxBuffer[8] << 24) | (self.my_rxBuffer[9] << 16) | (self.my_rxBuffer[10] << 8) | self.my_rxBuffer[11]

        if capacity is not None and capacity > 0:
            self.get["capacity"] = capacity / 1000
            return True
        else:
            return False
        
    def readProductionDate(self):
        """
        Reads the production date from the BMS.

        Returns:
        bool: True if the data is successfully retrieved, False otherwise.
        """
        self.sendCommand(self.COMMAND.BATTERY_DETAILS)
        
        if not self.receiveBytes():
            print("No data received in readProductionDate()")
            return False

        # Unpack the received data
        _, _, year, month, day = self.my_rxBuffer[4:9]

        # Format the production date
        self.get["productionDate"] = f"{year + 2000:04d}-{month:02d}-{day:02d}"
        return True

    def readBatteryCode(self):
        """
        Reads the battery code from the BMS.

        Returns:
        bool: True if the data is successfully retrieved, False otherwise.
        """
        self.sendCommand(self.COMMAND.BATTAEY_CODE)
        
        if not self.receiveBytes():
            print("No data received in readBatteryCode()")
            return False

        battery_code = ""
        for i in range(5):
            nr = self.my_rxBuffer[4 + i * 8]
            part = self.my_rxBuffer[5 + i * 8:12 + i * 8].decode("utf-8")
            if nr != i + 1:
                print("Bad battery code index")  # Use string anyhow, just warn
            battery_code += part

        if battery_code:
            self.get["batteryCode"] = re.sub(" +", " ", battery_code.replace("\x00", " ").strip())
        return True

    def setDischargeMOS(self, sw):
        self.sendCommand(self.COMMAND.DISCHRG_FET, data=[0x01 if sw else 0x00] + [0x00] * 7)

        if not self.receiveBytes():
            return False
        
        self.getStatusInfo()

        return True

    def setChargeMOS(self, sw):
        self.sendCommand(self.COMMAND.CHRG_FET, data=[0x01 if sw else 0x00] + [0x00] * 7)

        if not self.receiveBytes():
            return False

        self.getStatusInfo()

        return True

    def setBmsReset(self):
        self.sendCommand(self.COMMAND.BMS_RESET)

        if not self.receiveBytes():
            return False

        return True