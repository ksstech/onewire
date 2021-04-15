## Dallas/Maxim 1-Wire support

    Single driver for range of Dallas/Maxim I2C <> 1-Wire bridge devices.
    Simultaneous support for a mix of and multiples of each bridge type.
    Dynamically discover DS18x20 thermometers and enumerate using a handler function.
    API to support full functionality of DS18S20 including alarm thresholds, resolution etc.


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
		DS18X20 iButtons cannot be enumerated hence cannot be SENSE/LOG configured.(RULES ??) 
		
		If DS18X20 connected to any of DS2482-800 channels on KSS-AC0x, parasitic power is difficult.
			4	OW	GND		NC		Black
			3	LED	GND		Black	NC		Red+Blk
			2	LED	Vcc		Red		Red
			1	OW	IO		Yellow	Yellow	Yellow
					Result:	OK		FAIL		

	DS1990x:
		If used for 1:1 (bedroom) access control do NOT install more than 1 iButton reader per OW bus.
		If used for N:1 (informal clock-in) access control >1 iButton readers per OW bus can be used.
	
	General:
		Try not to mix DS1990X devices with other types on the same OW bus	
