/**************************************************************************
	--- Souliss ---
	Controllo Condizionatori Mitsubushi

		'Ciseco Remote Programming
		'Node Address Camera Letto = 04
		'Node Address Cucina = 05
		'Channel Offset = 3
		'BaudRate = 57600

	Libreria IR https://github.com/r45635/HVAC-IR-Control
		
***************************************************************************/
#define USE_DHT
// IMPOSTARE QUA L'INDIRIZZO a seconda del nodo di aggiornare
// 04 Camera - 05 Cucina
#define myvNet_address		0xCE04


#define USARTBAUDRATE_INSKETCH
#define	USART_BAUD57k6			1
#define USART_BAUD115k2			0


#include "bconf/StandardArduino.h"			// Use a standard Arduino
#include "conf/usart.h"

#include "Souliss.h"
#include <SPI.h>
#include "IRremote2.h"
#include <dht.h>


// network addresses
#define myvNet_subnet		0xFF00
#define myvNet_supern		0x0000

#define T_HVAC_MODE_NC	0
#define T_HVAC_MODE		1	//T19 Modo
#define T_HVAC_TEMP_NC  2
#define T_HVAC_TEMP		3	//T19 Temperatura
#define T_HVAC_FAN_NC   4
#define T_HVAC_FAN		5	//T19 Ventola
#define T_HVAC_VANNE_NC 6
#define T_HVAC_VANNE	7	//T19 Pale
#define T_HVAC_POWER	8	//T11 ON Off
#define T_HVAC_SEND		9	//T14 Send command
#define T_TEMP			10
#define T_HUMI			12

#define PIN_DHT22		19

#define DEADBAND      0.05 //Se la variazione è superio del 5% aggiorno
#define DEADBANDNTC   0.01 //Se la variazione è superio del 1% aggiorno
#define DEADBANDLOW	  0.005

// Identify the sensor, in case of more than one used on the same board	
	dht DHT;

IRsend irsend;

int mode = 0;
int temp = 0;
int fan = 0;
int vanne = 0;
boolean pwr_off = false;

void setup()
{	

	Souliss_SetAddress(myvNet_address, myvNet_subnet, myvNet_supern);		

	Souliss_SetT19(memory_map, T_HVAC_MODE_NC);
	Souliss_SetT19(memory_map, T_HVAC_TEMP_NC);
	Souliss_SetT19(memory_map, T_HVAC_FAN_NC);
	Souliss_SetT19(memory_map, T_HVAC_VANNE_NC);
	Souliss_SetT11(memory_map, T_HVAC_POWER);
	Souliss_SetT14(memory_map, T_HVAC_SEND);

#ifdef USE_DHT
	//T52 Temperatur DHT
	Souliss_SetT52(memory_map, T_TEMP);
	//T53 Umidità
	Souliss_SetT53(memory_map, T_HUMI);
#endif

	mOutput(T_HVAC_TEMP) = 53; //22 Gradi
}

void loop()
{
	EXECUTEFAST() {						
		UPDATEFAST();
		FAST_50ms() {	// We process the logic and relevant input and output every 50 milliseconds

		}

		FAST_70ms() {
		}

		FAST_90ms() { 
		}

		FAST_110ms() {
		#ifdef USE_DHT
			Souliss_Logic_T52(memory_map, T_TEMP, DEADBANDLOW, &data_changed);
			Souliss_Logic_T53(memory_map, T_HUMI, DEADBANDLOW, &data_changed);
		#endif
		}

		FAST_510ms() {


		}

		FAST_1110ms() {
			//-------- T19 Controllo Servo
			// Execute the logic that handle the LED
			Souliss_Logic_T19_Bis(memory_map, T_HVAC_MODE_NC, &data_changed);
			Souliss_Logic_T19_Bis(memory_map, T_HVAC_TEMP_NC, &data_changed);
			Souliss_Logic_T19_Bis(memory_map, T_HVAC_FAN_NC, &data_changed);
			Souliss_Logic_T19_Bis(memory_map, T_HVAC_VANNE_NC, &data_changed);
			Souliss_Logic_T11(memory_map, T_HVAC_POWER, &data_changed);
			Souliss_Logic_T11(memory_map, T_HVAC_SEND, &data_changed);

			// La libreria HVAC funziona a logica invertita. Inviando False Accendo, inviando True spengo
			//mode = mOutput(T_HVAC_MODE);
			mode = mOutput(T_HVAC_MODE);
			if (mode == 2) 	mode = 1;
			if (mode == 5) 	mode = 2;
			if (mode == 10) mode = 3;

			temp = round(mOutput(T_HVAC_TEMP) / 2.55);

			fan = mOutput(T_HVAC_FAN);
			if (fan == 2) 	fan = 1;
			if (fan == 5) 	fan = 2;
			if (fan == 7) 	fan = 3;
			if (fan == 10) 	fan = 4;
			if (fan == 12) 	fan = 5;
			if (fan == 15) 	fan = 6;

			vanne = mOutput(T_HVAC_VANNE);
			if (vanne == 2) vanne = 1;
			if (vanne == 5) vanne = 2;
			if (vanne == 7) vanne = 3;
			if (vanne == 10) vanne = 4;
			if (vanne == 12) vanne = 5;
			if (vanne == 15) vanne = 6;

			if (mOutput(T_HVAC_POWER) == Souliss_T1n_OnCoil) {	//Verifico che il T11 con il power sia acceso
				pwr_off = false;
			}
			else {
				pwr_off = true;
			}

			if (mOutput(T_HVAC_SEND)==1) {
				//Resta da verificare il cambiamento di un dato
				irsend.sendHvacMitsubishi((HvacMode)mode, temp, (HvacFanMode)fan, (HvacVanneMode)vanne, pwr_off);
				//Serial.println("IR Sent");
				mOutput(T_HVAC_SEND) = 0;
			}
		}

        FAST_2110ms() {
		}
		FAST_PeerComms();
}
	
	EXECUTESLOW() {
		UPDATESLOW();
		SLOW_10s() {		// We handle the light timer with a 10 seconds base time
		#ifdef USE_DHT
			DHTRead();
		#endif
		}
	}		
}

/**************************************************************************
/*
T19 logic bis, to handle a servo motor
*/
/**************************************************************************/
void Souliss_Logic_T19_Bis(U8 *memory_map, U8 slot, U8 *trigger)
{
	// Look for input value, update output. If the output is not set, trig a data
	// change, otherwise just reset the input

	if (memory_map[MaCaco_IN_s + slot] == Souliss_T1n_ToggleCmd)        // Toogle Command
	{
		// Toogle the actual status of the light
		if (memory_map[MaCaco_OUT_s + slot] == Souliss_T1n_OffCoil)
			memory_map[MaCaco_IN_s + slot] = Souliss_T1n_OnCmd;
		else if (memory_map[MaCaco_OUT_s + slot] == Souliss_T1n_OnCoil)
			memory_map[MaCaco_IN_s + slot] = Souliss_T1n_OffCmd;
		else
			memory_map[MaCaco_IN_s + slot] = Souliss_T1n_RstCmd;
	}
	else if (memory_map[MaCaco_IN_s + slot] == Souliss_T1n_OffCmd)        // Off Command
	{
		// Trigger the change and save the actual color
		if (memory_map[MaCaco_OUT_s + slot] != Souliss_T1n_OffCoil)
		{
			memory_map[MaCaco_OUT_s + slot] = Souliss_T1n_OffCoil;        // Switch off the light state
			*trigger = Souliss_TRIGGED;                                    // Trig the change
		}

		// Once is off, reset
		if ((memory_map[MaCaco_OUT_s + slot + 1] == 0))
			memory_map[MaCaco_IN_s + slot] = Souliss_T1n_RstCmd;        // Reset
	}
	else if (memory_map[MaCaco_IN_s + slot] == Souliss_T1n_OnCmd)
	{
		if (memory_map[MaCaco_OUT_s + slot] != Souliss_T1n_OnCoil)
			*trigger = Souliss_TRIGGED;

		memory_map[MaCaco_OUT_s + slot] = Souliss_T1n_OnCoil;            // Switch on the output

		memory_map[MaCaco_IN_s + slot] = Souliss_T1n_RstCmd;            // Reset
	}
	else if (memory_map[MaCaco_IN_s + slot] == Souliss_T1n_BrightUp)        // Increase the light value 
	{
		// Increase the light value
		if (memory_map[MaCaco_OUT_s + slot + 1] < 255 - Souliss_T1n_BrightValue)
			memory_map[MaCaco_OUT_s + slot + 1] += Souliss_T1n_BrightValue;

		memory_map[MaCaco_IN_s + slot] = Souliss_T1n_RstCmd;            // Reset
	}
	else if (memory_map[MaCaco_IN_s + slot] == Souliss_T1n_BrightDown)                // Decrease the light value
	{
		// Decrease the light value
		if (memory_map[MaCaco_OUT_s + slot + 1] > Souliss_T1n_BrightValue)
			memory_map[MaCaco_OUT_s + slot + 1] -= Souliss_T1n_BrightValue;

		memory_map[MaCaco_IN_s + slot] = Souliss_T1n_RstCmd;            // Reset
	}
	else if (memory_map[MaCaco_IN_s + slot] == Souliss_T1n_Set)
	{
		// Set the new color
		memory_map[MaCaco_OUT_s + slot + 1] = memory_map[MaCaco_IN_s + slot + 1];
		memory_map[MaCaco_AUXIN_s + slot + 1] = memory_map[MaCaco_OUT_s + slot + 1];
		memory_map[MaCaco_IN_s + slot + 1] = Souliss_T1n_RstCmd;

		memory_map[MaCaco_AUXIN_s + slot] = Souliss_T1n_Timed;            // Set a timer for the state notification        
		memory_map[MaCaco_OUT_s + slot] = Souliss_T1n_OnCoil;            // Switch on the output
		memory_map[MaCaco_IN_s + slot] = Souliss_T1n_RstCmd;            // Reset        
	}
	else
	{    // There is no command

		if (memory_map[MaCaco_AUXIN_s + slot] > Souliss_T1n_Set)            // Decrese the timer value
			memory_map[MaCaco_AUXIN_s + slot]--;
		else if (memory_map[MaCaco_AUXIN_s + slot] > 0)                    // If we not getting new commands, the burst        
		{                                                                // is done, send the actual state 
			memory_map[MaCaco_AUXIN_s + slot] = 0;
			*trigger = Souliss_TRIGGED;
		}
	}
}


#ifdef USE_DHT
void DHTRead() {
	int chk = DHT.read22(PIN_DHT22);
	if (chk == DHTLIB_OK) {
		float temperature = DHT.temperature;
		Souliss_ImportAnalog(memory_map, T_TEMP, &temperature);

		float humidity = DHT.humidity;
		Souliss_ImportAnalog(memory_map, T_HUMI, &humidity);
	}
}
#endif





#define Souliss_T1C						0X1C			// 

/**************************************************************************
/*!
	Define the use of Typical 1C
*/
/**************************************************************************/
void Souliss_SetT1C(U8 *memory_map, U8 slot)
{
	memory_map[MaCaco_TYP_s + slot] = Souliss_T1C;
}

U8 Souliss_Logic_T1C(U8 *memory_map, U8 slot, U8 *trigger)
{
	U8 i_trigger = 0;

	if (memory_map[MaCaco_IN_s + slot] == Souliss_T1n_BrightUp)		// Increase the light value
	{
		// Increase the light value
		if (memory_map[MaCaco_OUT_s + slot + 1] < 255 - Souliss_T1n_BrightValue)
			memory_map[MaCaco_OUT_s + slot + 1] += Souliss_T1n_BrightValue;
			memory_map[MaCaco_IN_s + slot] = Souliss_T1n_RstCmd;			// Reset
		}
	else if (memory_map[MaCaco_IN_s + slot] == Souliss_T1n_BrightDown)				// Decrease the light value
	{
		// Decrease the light value
		if (memory_map[MaCaco_OUT_s + slot + 1] > Souliss_T1n_BrightValue)
			memory_map[MaCaco_OUT_s + slot + 1] -= Souliss_T1n_BrightValue;
			memory_map[MaCaco_IN_s + slot] = Souliss_T1n_RstCmd;			// Reset
		}
}