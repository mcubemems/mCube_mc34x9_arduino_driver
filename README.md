Arduino Accelerometer sensor driver for mCube Product MC34X9 series.
==============================================================

The MC34X9 accelerometers features a dedicated motion block that implements the latest algorithms to detect “any motion”, shake, tilt/flip, and tilt 35 position. The product is optimized for high-performance applications by supporting full 16-bit resolution at Output Data Rates (ODR) up to 1KHz. The data samples can either be read directly from the device via a SPI or I2C interface, or queued in an on-board, 32 sample FIFO (per channel). The pinout is designed to match the most popular existing industry standard pinout.

This demo has been verified on Arduino DUE board with mCube's MC34X9 evaluation board. If you want to connect a MC34X9 chip to your own Auduino board, please pay attention to the following const definition in file MC34X9_demo.ino:
1. bSpi
	If you use SPI interface, please set bSpi value to zero.
	Or if you use I2C interface, please set bSpi value to one.
2. SPIChipSelectPin
	If you use SPI interface, please set the real number you're using to connect MC34X9's pin10.
3. I2CAddress
	If you use I2C interface, please set the real number you're using on MC34X9's I2C address.

MC3419
https://mcubemems.com/product/mc3419-3-axis-accelerometer/

MC3479
https://mcubemems.com/product/mc3479-3-axis-accelerometer/