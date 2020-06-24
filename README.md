## Dallas/Maxim 1-Wire support




# Devices supported:
	DS2482-10x	1 channel I2C <> 1-Wire bridge
	DS2482-800	8 channel I2C <> 1-Wire bridge
	DS2484		1 channel I2C <> 1-Wire bridge with programmable timing
	
	DS1990[A]	iButton ROMs
	DS1820		9-bit Thermometers
	DS18S20		9-bit Thermometers
	DS18B20		Fast, programmable 9/10/11/12 bit Thermometers


# Deployment guidelines:
	DS18X20:
		As far as possible, all devices in reasonable proximity should be connected to a single OW bus.
		External power (Gnd + IO + Vcc) should be used to power the devices to ensure read reliability.
		If DS18X20 iButtons are used they cannot be enumerated hence cannot be SENSE/LOG configured. (RULES ??) 

	DS1990x:
		If used for 1:1 (bedroom) access control do NOT install more than 1 iButton reader per OW bus.
		If used for N:1 (informal clock-in) access control >1 iButton readers per OW bus can be used.
	
	
	General:
		Try not to mix DS1990X devices with other types on the same OW bus	