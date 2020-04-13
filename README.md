# SX9324
Linux device driver for the Semtech SX9324, a capacitive Specific Absorption Rate (SAR) controller

**Device tree bindings for sx9324:**

Required properties:
- compatible: must be "semtech,sx9324"
- reg: i2c slave address of the chip
- nirq-gpio: interrupt gpio the chip's NIRQ pin is connected to
- vdd-supply: vdd power supply regulator
- pullup-supply: pull-up power supply regulator for SCL, SDA and NIRQ

**Example:**

	i2c@00000000 {
		/* ... */

		semtech_sx9324@28 {
			compatible = "semtech,sx9324";
			reg = <0x28>;
			nirq-gpio = <&tlmm 77 0>;
			vdd-supply = <&pm660_l13>;
			pullup-supply = <&pm660_l14>;
		};

		/* ... */
	};
