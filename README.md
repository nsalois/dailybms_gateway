# Daly BMS UART Interface

This repository provides a CircuitPython interface for communicating with a Daly Battery Management System (BMS) over UART. The interface allows you to read various parameters from the BMS, set configurations, and handle commands via MQTT.

## Features

- Read pack temperature, cell voltages, and cell temperature
- Get min/max cell voltage and discharge/charge MOS status
- Get status information and active faults
- Set State of Charge (SOC)
- Set discharge and charge MOS states
- Reset BMS
- MQTT integration for publishing all bms data and for remote command handling


## Credits

This project is inspired by and includes code from the [dbus-serialbattery](https://github.com/Louisvdw/dbus-serialbattery) project, specifically from the `daly.py` file. Special thanks to the contributors of that project for their work.


## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.