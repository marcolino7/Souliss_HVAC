/**************************************************************************
	--- Souliss ---
	Controllo Condizionatori Mitsubushi

		'Ciseco Remote Programming
		'Node Address = 04
		'Channel Offset = 3
		'BaudRate = 57600

***************************************************************************/

#define	VNET_DEBUG_INSKETCH
#define VNET_DEBUG  		1

#define	MaCaco_DEBUG_INSKETCH
#define MaCaco_DEBUG  		0


#define USARTDRIVER_INSKETCH
#define	USARTDRIVER				Serial1	//Dico al driver vNet di usare la seriale 0 dell'UNO
#define USART_TXENABLE			0


#define USARTBAUDRATE_INSKETCH
#define	USART_BAUD57k6			1
#define USART_BAUD115k2			0

#define USART_DEBUG  			1

#include "bconf/StandardArduino.h"			// Use a standard Arduino
#include "conf/usart.h"

#include "Souliss.h"
#include <SPI.h>
#include "IRremote2.h"


// network addresses
#define myvNet_address		0xCE04
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

#define DEADBAND      0.05 //Se la variazione è superio del 5% aggiorno
#define DEADBANDNTC   0.01 //Se la variazione è superio del 1% aggiorno
#define DEADBANDLOW	  0.005

IRsend irsend;

int mode = 0;
int temp = 0;
int fan = 0;
int vanne = 0;
boolean pwr_off = false;

void setup()
{	
	Serial.begin(115200);
	Serial.println("NodeINIT");

	Souliss_SetAddress(myvNet_address, myvNet_subnet, myvNet_supern);		

	// Tipico T19 per il controllo del Servomotore
	Souliss_SetT19(memory_map, T_HVAC_MODE_NC);
	Souliss_SetT19(memory_map, T_HVAC_TEMP_NC);
	Souliss_SetT19(memory_map, T_HVAC_FAN_NC);
	Souliss_SetT19(memory_map, T_HVAC_VANNE_NC);
	Souliss_SetT11(memory_map, T_HVAC_POWER);

	Serial.println("Joined");

	mOutput(T_HVAC_TEMP) = 53; //22 Gradi
}

void loop()
{
	EXECUTEFAST() {						
		UPDATEFAST();
		FAST_50ms() {	// We process the logic and relevant input and output every 50 milliseconds
			//-------- T19 Controllo Servo
			// Execute the logic that handle the LED
			Souliss_Logic_T19_Bis(memory_map, T_HVAC_MODE_NC, &data_changed);
			Souliss_Logic_T19_Bis(memory_map, T_HVAC_TEMP_NC, &data_changed);
			Souliss_Logic_T19_Bis(memory_map, T_HVAC_FAN_NC, &data_changed);
			Souliss_Logic_T19_Bis(memory_map, T_HVAC_VANNE_NC, &data_changed);
			Souliss_Logic_T11(memory_map, T_HVAC_POWER, &data_changed);

		}

		FAST_70ms() {
		}

		FAST_90ms() { 
		}

		FAST_110ms() {
		}

		FAST_510ms() {
		}

		FAST_1110ms() {
		}

        FAST_2110ms() {

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

			Serial.println("-------------");
			Serial.print("mode:");
			Serial.println(mode);

			Serial.print("temp:");
			Serial.println(temp);

			Serial.print("fan:");
			Serial.println(fan);

			Serial.print("vanne:");
			Serial.println(vanne);



			if (mOutput(T_HVAC_POWER) == Souliss_T1n_OnCoil) {	//Verifico che il T11 con il power sia acceso
				pwr_off = false;
			}
			else {
				//irsend.sendHvacMitsubishi(mode, temp, FAN_SPEED_AUTO, VANNE_AUTO_MOVE, true);
				pwr_off = true;
			}

			Serial.print("pwr_off:");
			Serial.println(pwr_off);
			
			if (isTrigger()) {
				//Resta da verificare il cambiamento di un dato
				irsend.sendHvacMitsubishi((HvacMode)mode, temp, (HvacFanMode)fan, (HvacVanneMode)vanne, pwr_off);
				Serial.println("IR Sent");
			}
		}
		FAST_PeerComms();
}
	
	EXECUTESLOW() {
		UPDATESLOW();
		SLOW_10s() {		// We handle the light timer with a 10 seconds base time
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
        if(memory_map[MaCaco_OUT_s + slot] == Souliss_T1n_OffCoil)        
            memory_map[MaCaco_IN_s + slot] = Souliss_T1n_OnCmd;            
        else if(memory_map[MaCaco_OUT_s + slot] == Souliss_T1n_OnCoil)
            memory_map[MaCaco_IN_s + slot] = Souliss_T1n_OffCmd;
        else
            memory_map[MaCaco_IN_s + slot] = Souliss_T1n_RstCmd;
    }
    else if (memory_map[MaCaco_IN_s + slot] == Souliss_T1n_OffCmd)        // Off Command
    {
        // Trigger the change and save the actual color
        if(memory_map[MaCaco_OUT_s + slot] != Souliss_T1n_OffCoil)  
        {                
            memory_map[MaCaco_OUT_s + slot] = Souliss_T1n_OffCoil;        // Switch off the light state
            *trigger = Souliss_TRIGGED;                                    // Trig the change
        }
 
        // Once is off, reset
        if((memory_map[MaCaco_OUT_s + slot + 1] == 0))
            memory_map[MaCaco_IN_s + slot] = Souliss_T1n_RstCmd;        // Reset
    }
    else if (memory_map[MaCaco_IN_s + slot] == Souliss_T1n_OnCmd)
    {
        if(memory_map[MaCaco_OUT_s + slot] != Souliss_T1n_OnCoil)  
            *trigger = Souliss_TRIGGED;    
    
        memory_map[MaCaco_OUT_s + slot] = Souliss_T1n_OnCoil;            // Switch on the output
        
            memory_map[MaCaco_IN_s + slot] = Souliss_T1n_RstCmd;            // Reset
    }
    else if (memory_map[MaCaco_IN_s + slot] == Souliss_T1n_BrightUp)        // Increase the light value 
    {
        // Increase the light value
        if(memory_map[MaCaco_OUT_s + slot + 1] < 255 - Souliss_T1n_BrightValue) 
            memory_map[MaCaco_OUT_s + slot + 1] += Souliss_T1n_BrightValue;
        
        memory_map[MaCaco_IN_s + slot] = Souliss_T1n_RstCmd;            // Reset
    }
    else if (memory_map[MaCaco_IN_s + slot] == Souliss_T1n_BrightDown)                // Decrease the light value
    {
        // Decrease the light value
        if(memory_map[MaCaco_OUT_s + slot + 1] > Souliss_T1n_BrightValue) 
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
        
        if(memory_map[MaCaco_AUXIN_s + slot] > Souliss_T1n_Set)            // Decrese the timer value
            memory_map[MaCaco_AUXIN_s + slot]--;
        else if(memory_map[MaCaco_AUXIN_s + slot] > 0)                    // If we not getting new commands, the burst        
        {                                                                // is done, send the actual state 
            memory_map[MaCaco_AUXIN_s + slot] = 0;
            *trigger = Souliss_TRIGGED;                                    
        }    
    }    
}
