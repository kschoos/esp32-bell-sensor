# Bell Sensor

## Description
The Bell-Sensor listens to the sound around it and finds out, whenever the doorbell was rung. 
It then notifies the user.

## How?
It continuously samples the sound around it. It looks for a certain frequency pattern in the data. 
When this pattern is found, it notifies the user via TCP and also sends the data packet that triggered the notification.
This way, the user can check manually if it was a false alarm or not.

## Technology?
This program runs on an ESP-32. The rest of the circuitry is just a condenser microphone, an Op-Amp and some other passive components.


## Microphone-Circuit Schematic:
![Schematic][logo]

[logo]: https://raw.githubusercontent.com/kschoos/esp32-bell-sensor/master/media/bellsensor_schematic.png "Schematic"