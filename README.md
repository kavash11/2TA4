# Embedded Systems Projects
A compilation of projects developed using an STM32f429 Discovery board, KEIL uVision and CubeMX

Lab 0: LED 3 on the Discovery board blinks

Lab 1-1: Generate an interrupt to make LED 3 on the Discovery board blink
Lab 1-2: Generate an interrupt to blink LED 3 every second and another interrupt to blink LED 4 every 2 seconds on the Discovery board
Lab 1-3: Led 3 blinks every second and LED 4 blinks every 2 seconds as long as the user button is pressed

Lab 2: When an LED is flashed, the user must press the user button to begin the reaction time game. After a random time interval the LED will blink again and the user must press an external button as fast as possible. The time when this button is pressed is the reaction time. The fastest reaction time is stored in the EEPROM. The LCD will display the most recent and the fastest reaction time.

Lab 3: The exact time and date when the user button is pressed is measured using an RTC and is stored in an 24FC64F EEPROM. The current time is always displayed on the LCD and the current date is displayed when the user button is held. The last two times the user button was pressed is displayed on the LCD when an external button is pressed. Additional external buttons are used to set the time and date.

Lab 4: Upon reset, measure the temperature using an LM35 and display the temperature with the fan setpoint temperature on the LCD after analog to digital conversion. Two external buttons are used to increase/decrease the fan setpoint temperature. If the LM35's temperature is above the setpoint temperature, the fan will turn on and it will turn off once the LM35's temperature is lower than the setpoint temperature. As teh sensor gets warmer, the fan's speed will increase until it reaches max power.

Lab 5: Configure external push buttons to drive a stepper motor in half/full step mode, increase/decrease its speed and change its rotation clockwise/counterclockwise.
