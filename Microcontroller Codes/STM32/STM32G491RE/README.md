📁 1. LED_BUTTON.c
Description:
This example demonstrates basic GPIO functionality. The onboard LED toggles ON/OFF when the user button is pressed.

Peripherals Used: GPIO

Features:

Polls button state continuously.

Controls LED based on button input level.

📁 2. LED_BUTTON_IRQ.c
Description:
This example demonstrates interrupt-driven GPIO control. The onboard LED toggles state in response to a button press event, using external interrupt (EXTI).

Peripherals Used: GPIO, EXTI (Interrupts), NVIC

Features:

Button press triggers an interrupt.

LED toggles inside the interrupt service routine (ISR).

Demonstrates clean edge detection without polling.

📁 3. I2C_Scan_Device.c
Description:
This example scans the I2C bus for connected slave devices and prints their addresses over UART (or console output).

Peripherals Used: I2C, GPIO, (Optional: UART for logging)

Features:

Iterates through all 7-bit I2C addresses (0x00 to 0x7F).

Sends a dummy write to each address.

Detects ACK response to identify active slave devices.

Useful for verifying device connections on the I2C bus.