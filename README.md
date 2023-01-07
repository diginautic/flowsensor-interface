# flowsensor-interface
Diginautic Flowsensor Interface

N2k address 108

Interface to control (reset + set) and read flowsensor using digital switching.

Listens for 127502 for reset- or setbutton press. Changes button according to received status and reports back binary bank status to N2k using 127501.

Sends fresh water level to N2K using SetN2kFluidLevel.