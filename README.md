# Pumpkin Game

Three microcontrollers: two ESP32s and one Arduino Uno
1. Sender. ESP32 connected to an accelerometer. This sends x, y, z data from the accelerometer using ESP-NOW to the receiver. This device is housed in a 3d printed enclosure with a battery and battery controller. It will be placed inside a pumpkin that player will manipulate to control the game board
2. Receiver. ESP32 connected to a DFPlayer module with UART, to a server motor controler with I2C, and to display Uno with UART. The Sender ESP sends Input data to the receiver using ESP-NOW
3. Display. Arduino UNO connected to a DMD LED display panel. The display Arduino receives data from the Receiver ESP with UART.

The receiver controls two servo motors that are attached to a frame that pivots on two axis. In game play, a metal/conductive ball is rolled around a maze. Additionally, the game board has a start hole and an end hole. Each hole has two copper tape connections which have soldered connections and wires running to the receiver ESP. Additionally, one additional connection with a similar configuration is used as a razz feature. When the ball hits a restricted area, a penalty sound sounds.

The DFPlayer module has an SD card with four files. 1.mp3 (ready to start sound), 2.mp2 (in progress sound), 3.mps (complete sound), and 4.mp3 (razz sound).
