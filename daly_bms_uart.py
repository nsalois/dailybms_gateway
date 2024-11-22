import time
import board
import busio

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
            0: {"description": "unitOverVoltageStageOne", "active": False},
            1: {"description": "unitOverVoltageStageTwo", "active": False},
            2: {"description": "unitUnderVoltageStageOne", "active": False},
            3: {"description": "unitUnderVoltageStageTwo", "active": False},
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
        
    def sendCommand(self, cmdID, data=None):
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

        # Print the full TX buffer in hexadecimal format
        print("<DALY-BMS DEBUG> TX Buffer: ", self.my_txBuffer.hex())
        

    def receiveBytes(self, timeout=1):
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
        return True

    def validateChecksum(self):
        # Calculate the checksum of the received data
        calculated_checksum = sum(self.my_rxBuffer[:12]) & 0xFF  # Ensure it is an 8-bit value
        received_checksum = self.my_rxBuffer[12]

        return calculated_checksum == received_checksum

    def barfRXBuffer(self):
        print("<DALY-BMS DEBUG> RX Buffer: [", end="")
        for i in range(XFER_BUFFER_LENGTH):
            print(f",0x{self.my_rxBuffer[i]:02X}", end="")
        print("]")

    def getTotalVoltageCurrentSOC(self):
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
        self.sendCommand(self.COMMAND.MIN_MAX_TEMPERATURE)
        if not self.receiveBytes():
            return False
        self.get["tempMax"] = self.my_rxBuffer[4] - 40
        self.get["tempMin"] = self.my_rxBuffer[6] - 40
        self.get["tempAverage"] = (self.get["tempMax"] + self.get["tempMin"]) / 2
        return True

    def getMinMaxCellVoltage(self):
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
        self.sendCommand(self.COMMAND.DISCHARGE_CHARGE_MOS_STATUS)
        if not self.receiveBytes():
            return False

        # Parse the response and update the `get` dictionary
        status_byte = self.my_rxBuffer[4]
        if status_byte == 0x00:
            self.get["chargeDischargeStatus"] = "Idle"
        elif status_byte == 0x01:
            self.get["chargeDischargeStatus"] = "Charge"
        elif status_byte == 0x02:
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
        self.sendCommand(self.COMMAND.CELL_TEMPERATURE)

        if not self.receiveBytes():
            return False

        for i in range(self.get["numOfTempSensors"]):
            self.get["cellTemperature"][i] = self.my_rxBuffer[4 + i] - 40

        return True
    
    def getCellBalanceState(self):
        self.sendCommand(self.COMMAND.CELL_BALANCE_STATE)

        if not self.receiveBytes():
            return False

        for i in range(self.get["numberOfCells"]):
            self.get["cellBalanceState"][i] = bool(self.my_rxBuffer[4 + (i // 8)] & (1 << (i % 8)))

        self.get["cellBalanceActive"] = self.my_rxBuffer[7]

        return True

    def reverse_bits(byte):
        result = 0
        for i in range(8):
            result = (result << 1) | (byte & 1)
            byte >>= 1
        return result

    def getActiveFaults(self):
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
        binary_data = ''.join(f'{self.reverse_bits(byte):08b}' for byte in data)

        # Update fault statuses based on the binary data
        for bit_number, fault_info in self.failure_codes.items():
            if bit_number < len(binary_data):
                fault_info["active"] = bool(int(binary_data[bit_number]))

        # Return only the active faults with their descriptions
        self.activeFaults = {fault_info['description']: fault_info['active'] for fault_info in self.failure_codes.values() if fault_info['active']}

        return True
    
    def getPackVoltageThreshold(self):
        if not self.sendCommand(self.COMMAND.PACK_THRESHOLDS):
            print("<BMS > Receive failed", self.COMMAND.PACK_THRESHOLDS)
            return False

        self.get["maxPackThreshold1"] = float((self.my_rxBuffer[4] << 8) | self.my_rxBuffer[5])
        self.get["maxPackThreshold2"] = float((self.my_rxBuffer[6] << 8) | self.my_rxBuffer[7])
        self.get["minPackThreshold1"] = float((self.my_rxBuffer[8] << 8) | self.my_rxBuffer[9])
        self.get["minPackThreshold2"] = float((self.my_rxBuffer[10] << 8) | self.my_rxBuffer[11])

        return True

    def getVoltageThreshold(self):
        if not self.sendCommand(self.COMMAND.CELL_THRESHOLDS):
            print("<BMS > Receive failed", self.COMMAND.CELL_THRESHOLDS)
            return False

        self.get["maxCellThreshold1"] = float((self.my_rxBuffer[4] << 8) | self.my_rxBuffer[5])
        self.get["maxCellThreshold2"] = float((self.my_rxBuffer[6] << 8) | self.my_rxBuffer[7])
        self.get["minCellThreshold1"] = float((self.my_rxBuffer[8] << 8) | self.my_rxBuffer[9])
        self.get["minCellThreshold2"] = float((self.my_rxBuffer[10] << 8) | self.my_rxBuffer[11])

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