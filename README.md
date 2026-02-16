CAR ALARM/ IGNITION LOCK + WINDSHIELD SYSTEM
--
Group Members:
--
Ellie Nguyen, Colin Vaughan

System Behavior:
--
This project implements an ignition lock/alarm system in addition to a windshield wiper system. 
The system detects driver and passenger seat occupancy, along with seatbelt inputs, to determine whether ignition is allowed.
If the ignition button is pressed while any of the required conditions are not met, an alarm in the form of a buzzer will be triggered, the system will print the failed conditions, and ignition will be prevented.
When the required conditions for ignition are met (i.e., both seats are occupied, and both seatbelts are engaged), a green LED will turn on, indicating ignition is possible. 
Pressing the ignition while the green LED is active will "turn on" the engine, indicated by a blue LED.
While the engine is running, a windshield wiper, driven by a servo motor, operates in one of four user-selected modes: HI, LO, INT, or OFF. The mode is selected via a potentiometer and is displayed on the LCD.
For the intermittent mode (INT), a second potentiometer is used to select the delay, either short, medium, or long. While in this mode, the chosen delay is also displayed on the LCD. The wipers only run when the engine is
is running and returns to 0 degrees when the off mode is selected, or the engine is turned off. 

Design Alternatives
--
We chose a standard position motor for our windshield system. The benefits of using this over a continuous rotation servo are that it allows for direct angle control, which makes it easier to park the wipers back at 0 degrees or to control the max angle. Additionally, the intermittent mode is much cleaner with a position servo as it allows for the motor to move to a certain angle and hold there.    

Summary of testing results:
--
Car alarm/ignition lock

| Specification | Test Process | Result |
|-|-|-|
|When the driver sits down, display the message, “Welcome to enhanced alarm system model 218-W26”. | Engine OFF. Press the driver seat input to “occupied.” |Welcome message is displayed once|
|Indicate ignition is enabled with the green LED only when both seats are occupied, and both seatbelts are fastened.|Try all combinations of seat/belt inputs|Green LED only turns on when both seat inputs and both seatbelt inputs are toggled | 
|If the ignition enabled (green LED) is lit, then normal ignition occurs. Light the blue LED and extinguish the green LED. Display the message, “Engine started.”| Press the ignition button when the Green LED is on | Blue LED is activated, and the green LED is turned off. "Engine on" message is displayed once|
|If the ignition is not enabled (green LED not lit), then ignition is inhibited. Sound the alarm buzzer; display the message, “Ignition inhibited,” and display all the reasons why the ignition was inhibited: “Passenger seat not occupied,” “Driver seatbelt not fastened,” etc. Once the error messages are displayed, the system allows additional start attempts.  | Press the ignition button while various requirements are not met|Alarm is turned on, "ignition inhibited" is displayed, and the corresponding errors are displayed|
|Keep the engine running even if the driver or passenger should remove their seat belts or exit the vehicle.| Engine is on, toggle the seatbelt and occupancy buttons | Engine remains running| 
|When the engine is running, stop the engine when the ignition button is pushed.|Engine is on, ignition button is pressed| Engine light turns off|

| Specification | Test Process | Result |
|---|---|---|
|If the engine is running, and the user selects HI, LO, INT, or OFF, run the wipers in the appropriate mode, with the typical parameter values, as described above. Read the desired delay time for INT mode from the intermittent delay time selector. Do not run the wipers if the engine is not running.|User selects a mode while engine is running and selects a mode while the engine is not running| The servo exhibits the correct behavior for the selected mode|if the engine is running. If the engine is off, the wiper doesn't move|
|If the engine is running, and the user selects HI, LO, INT, or OFF, display the selected mode, including the delay time selection (SHORT, MEDIUM, or LONG) for intermittent mode on the LCD. |User selects a mode, verify that delay is only displayed for INT mode| Selected Mode is displayed on the LCD, and when in the INT mode, selected delay mode is displayed|
|If the wiper mode selector is turned to OFF, or the engine is turned off, then if the wiper is moving, in any mode, complete the current cycle and return the wipers to 0 degrees;|1. Turn the wiper mode off from one of the other modes. 2. Turn off the engine while the wiper is on| Both result in the current cycle being completed and the wipers returning to 0 degrees|
|If the wiper mode selector is turned to OFF, or the engine is turned off, then if the wiper is hesitating in INT mode, remain stationary.|While the wiper is paused, switch to OFF / stop engine.|The wiper remains stationary|



