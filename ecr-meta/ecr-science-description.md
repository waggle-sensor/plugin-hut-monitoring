# Plugin for monitoring temperature/humidity/orientation
This plugin communicates with AHT20 and MPU6050 sensors that are connected to an Arduino Mega 256 board. Please refer to the [main.ino](../arduino/main.ino) for how the Arduino outputs sensor data via Serial port.

# Science
A container with solar panels mounted on the top is placed on the ground. A solar charger controller and a pack of batteries are inside the container to distribute power to the load. The Arduino board with the sensors is measuring the condition inside the container. We are interested in how temperature and humidity change over seasons which could affect efficiency of charging and discharging betteries. Since the container sits on unpaved surface, we are also interested in whether the container sinks over time. The MPU6050 sits flat on the shelve, measuring roll, pitch, and yaw angles. We should see degree changes from the sensor if the container changes its orientation over time.

# Ontology

Detailed topic names of the measurements can be found in [main.py](../main.py#L13)