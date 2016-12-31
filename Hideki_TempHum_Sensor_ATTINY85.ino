/*
*********************************************************************
*  ATMEL ATTINY 25 / 45 / 85 / ARDUINO
*
*                      +-\ / -+
*Reset(D 5) PB5  1 | | 8  Vcc
*     Ain3(D 3) PB3  2 | | 7  PB2(D 2) Ain1
*     Ain2(D 4) PB4  3 | | 6  PB1(D 1) pwm1
*                GND  4 | | 5  PB0(D 0) pwm0 *INT0*
*                      +---- +
*
*  Install: ATtiny - Models in Arduino IDE : http://highlowtech.org/?p=1695
****************************************************************************
* This sketch sends (bogus) thermo / hygro data to a remote weather sensors made by Cresta.
*
* Setup:
*  - connect transmitter input of a 433MHz transmitter to digital pin 11
*  - On the weather station, activate the "scan" function for channel 1.
*/
#include <OneWire.h>
#include <SensorTransmitter.h>
#include "Narcoleptic.h"

//--- ATTiny
#define DALLAS_SENSOR_PIN         2     //   PIN5 = PB0 = D0 
#define TX_433_PIN                0     //   PIN2 = PB3 = D3	[### TODO: PIN_SEND in LaCrosse.cpp ebenfall setzen!!!]
#define SLEEP_MINUTES			  3     //   +++ powerdown mode duration +++
#define OW_ROMCODE_SIZE           8


float                     controller_VCC	= 0.0;
long                      vcc_reading		= 0;
volatile float            batteriespannung	= 0.0;
long					  randomNumber		= 0;


//--- Protypes 
float ReadSingleOneWireSensor(OneWire ds);
long  readVcc(void);

//--- Instances 
//--- initializes a ThermoHygroTransmitter on pin 11, with "random" ID 0, on channel 1.
ThermoHygroTransmitter transmitter(TX_433_PIN, 0, 1);

//--- Temperatursensor-Instanz
OneWire  dallas(DALLAS_SENSOR_PIN);	// on arduino port pin 2 (a 4.7K resistor is necessary, between Vcc and DQ-Pin   1=GND 2=DQ 3=Vcc )


//-------------------------------------------------------------------------
void setup()
{
	pinMode(DALLAS_SENSOR_PIN, OUTPUT);
	digitalWrite(DALLAS_SENSOR_PIN, HIGH);
	pinMode(TX_433_PIN, OUTPUT);
	digitalWrite(TX_433_PIN, LOW);
}
//-------------------------------------------------------------------------
void loop()
{
	//--- set attiny back to normal operation mode
	delay(250); // ms, needed for settling DS18B20 

	//--- Betriebsspannung auslesen  
	vcc_reading = readVcc();

	//float controllerVCC = 1.1 * 1023 / vcc_reading;  
	float controllerVCC = vcc_reading / 100.0;    // ergibt 40 statt 4, also 1 NK-Stelle  0..1000 skaliert
	float theta = ReadSingleOneWireSensor(dallas);
	int iTheta = (int)(theta * 10); 

	//--- send with HIDEKI-protocol
	//--- temperatures are passed at 10 times the real value,
	//--- to avoid using floating point math.
	transmitter.sendTempHumi(iTheta, controllerVCC);

	//--- preserve more power during sleep phase 
	pinMode(DALLAS_SENSOR_PIN, INPUT);
	pinMode(TX_433_PIN,INPUT);

	//--- fall to deep powersave-sleep, see notes in comments and 
	Narcoleptic.delay_minutes(SLEEP_MINUTES);

	//--- set back to normal operation mode
	pinMode(DALLAS_SENSOR_PIN, OUTPUT);
	digitalWrite(DALLAS_SENSOR_PIN, HIGH);

	pinMode(TX_433_PIN, OUTPUT);
	digitalWrite(TX_433_PIN, LOW);

}
//-------------------------------------------------------------------------
float ReadSingleOneWireSensor(OneWire ds)
{
	//--- 18B20 stuff
	byte i;
	byte type_s;
	byte data[12];
	byte addr[8];
	float celsius = 12.3;

	if (!ds.search(addr))
	{
		ds.reset_search();
		delay(250);
		celsius = 99.9;
		return celsius;
	}

	if (OneWire::crc8(addr, 7) != addr[7])
	{
		celsius = 88.8;
		return celsius;
	}

	//--- the first ROM byte indicates which chip
	switch (addr[0])
	{
	case 0x10:
		//Serial.println("  Chip = DS18S20");  // or old DS1820
		type_s = 1;
		break;
	case 0x28:
		// Serial.println("  Chip = DS18B20");
		type_s = 0;
		break;
	case 0x22:
		// Serial.println("  Chip = DS1822");
		type_s = 0;
		break;
	default:
		// Serial.println("Device is not a DS18x20 family device.");
		celsius = 77.7;
		return celsius;
	}

	ds.reset();
	ds.select(addr);
	ds.write(0x44);  // start conversion, use alternatively ds.write(0x44,1) with parasite power on at the end

	delay(1000);     // maybe 750ms is enough, maybe not
					 // we might do a ds.depower() here, but the reset will take care of it.

	ds.reset();		 //--- DS18B20 responds with presence pulse
					 //--- match ROM 0x55, sensor sends ROM-code command ommitted here.
	ds.select(addr);
	ds.write(0xBE);         //--- read scratchpad
	for (i = 0; i < 9; i++)
	{
		//--- we need 9 bytes, 9th byte is CRC, first 8 are data
		data[i] = ds.read();
	}

	//--- Convert the data to actual temperature
	//--- because the result is a 16 bit signed integer, it should
	//--- be stored to an "int16_t" type, which is always 16 bits
	//--- even when compiled on a 32 bit processor.
	int16_t raw = (data[1] << 8) | data[0];
	if (type_s)
	{
		raw = raw << 3;     //--- 9 bit resolution default
		if (data[7] == 0x10)
		{
			//--- "count remain" gives full 12 bit resolution
			raw = (raw & 0xFFF0) + 12 - data[6];
		};
	}
	else
	{
		byte cfg = (data[4] & 0x60);
		// at lower res, the low bits are undefined, so let's zero them
		if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
		else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
		else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
											  //// default is 12 bit resolution, 750 ms conversion time
	};

	celsius = (float)raw / 16.0;		//fahrenheit = celsius * 1.8 + 32.0;

										//---- Check if any reads failed and exit early (to try again).  
	if (isnan(celsius))
	{
		//--- signalize error condition 
		celsius = -99.9;
	};
	return celsius;
}
//---------------------------------------------------------------------
//--- Helpers 
//---------------------------------------------------------------------
long readVcc()
{
	//--- read 1.1V reference against AVcc
	//--- set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
	ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
	ADMUX = _BV(MUX3) | _BV(MUX2);
#else
	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif  

	delay(2); // Wait for Vref to settle

	ADCSRA |= _BV(ADSC); // Start conversion
	while (bit_is_set(ADCSRA, ADSC)); // measuring

	uint8_t low = ADCL; // must read ADCL first - it then locks ADCH  
	uint8_t high = ADCH; // unlocks both

	long result = (high << 8) | low;

	/***************************************************************************************
	*  Berechnung/Skalierung mit manueller Messung der Betriebsspannung:
	*
	*        internal1.1Ref = 1.1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() function)
	*                       = 1.1 * 5126				 / 5258 => 1.09 ==> 1.09*1023*1000 = 1097049
	****************************************************************************************/

	result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000

								//result = 1097049L / result; // korrigierter Wert bei 3V3 müsste fuer jeden Controller bestimmt werden, obiger Wert scheint allgemeiner zu sein.

	return result; // Vcc in millivolts
}
//---------------------------------------------------------------------
//---------------------------------------------------------------------
//---------------------------------------------------------------------
// <eof>
//---------------------------------------------------------------------