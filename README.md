# webusb_microcontroller
The program for the microcontroller to work with the program from the webusb repository. The STM32F407 microcontroller has three I2C hardware channels. These channels are polled in parallel via interrupts. Sensors within the same channel are polled sequentially. Work with QMC5883L and TLV493D_A1B6 magnet sensors.
