Firmware Upgrade run Firmware_Tool
Some older hardware used a lower baudrate (38400), select this option if you have it.
Note not all v_1_0_0 hardware uses the low baudrate. 

To put the radio in DFU (ready for firmware update)
Open the tool load the firmware file, click the port icon and select your com port. 
With the radio off and the programming cable connected to the radio, hold down:
SK1+PTT+SK2 at the same time. Give it a few seconds then release
Select write operation on the firmware tool. 

After upgrading the radio from v_1 to v_2 firmware the hardware version changes from v_1_0_0 to v_1_0_4
Newer Firmware Tools will not allow downgrade from V2 to V1 firmware. 
This Firmware Tool *will* allow the downgrade. Follow the same procedure.    

After upgrade from v1 to v2 firmware turn off radio, then open CPS 1.19
Click [R] in CPS and ensure com port is selected. Turn on radio then once booted, connect K1. 
Read the radio config, in Settings go to [Tone Level] and set to 1. 
Write to the radio.

Follow the procedure above for the Font tool. The font tool only works for v1_0_4 as v2 firmware is required.
File to load is font.txt
 