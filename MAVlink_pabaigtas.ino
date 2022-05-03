/*
 Name:		MAVlink_pabaigtas.ino
 Created:	5/2/2022 3:58:09 PM
 Author:	Pavel Duniak
*/

// Galimos MSG_ID 
//GLOBAL_POSITION_INT ( #33 ) (LAT, LON, ALT, VZ,VX, VY, YAW)
//HIGH_LATENCY2 ( #235 )

/*
Name:		MAV_test.ino
Created:	4/18/2022 9:30:07 PM
Author:	Pavel Duniak
//https://mavlink.io/en/messages/common.html#messages
*/



// include the SD library:
#include <SPI.h>
#include <SD.h>
#include "mavlink.h"
#include <rtc_clock.h>



File dataFile;

// change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
// MKRZero SD: SDCARD_SS_PIN
// SPI
// MISO - 50
// SCK - 52
// MOSI - 51

const int PIN_chipSelect = 10;  // RNN:  UNO, (???????? ?? Mega 2022.03.22) 


// In case we need a second serial port for debugging
#define SOFT_SERIAL_DEBUGGING   // Comment this line if no serial debugging is needed
#ifdef SOFT_SERIAL_DEBUGGING

// Library to use serial debugging with a second board

#define RxPin0 9               //PIN RX FOR UNO
#define TxPin0 10              //PIN TX FOR UNO
const int PIN_CAMERA = 5;      //PIN FOR CAMERA PULSE
const int PIN_RC_RX_CH = 14;   //SERVO TESTER PWM SIGNAL INPUT PIN
int   ch9_value;
int   i = 0;
int   counter = 1;
bool  flag_active_on = false;
bool  logData = false;         // peremennaja logit dannije ili net ( if true to zalogila vsio)
byte  RCdeadband = 35;         // RCsignal can vary this much from 1500uS without controller responding
RTC_clock rtc_clock(XTAL);
int hh, mm, ss;

#define  ARDUINO_DUE_MEGA // ??? UNO


#ifdef ARDUINO_DUE_MEGA 

#define pxSerial Serial2

#else

#include <SoftwareSerial.h>


//int request = 0;
//int receive = 0;

SoftwareSerial pxSerial = SoftwareSerial(RxPin0, TxPin0);  // RX, TX || UNO - PX4

#endif // ARDUINO_DUE_MEGA 

#endif

														   //#include "common/mavlink_msg_request_data_stream.h"


											 // Mavlink variables
unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 1000;  // next interval to count
const int num_hbs = 60;                      // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int num_hbs_pasados = num_hbs;



//-----------------------------------------------------------------------------
bool init_sdcard()
{
	//setup SD card
	Serial.print("Initializing SD card...");

	// see if the SD card is present and can be initialized:
	if (!SD.begin(PIN_chipSelect))
	{
		Serial.println("Card failed, or not present");
		// don't do anything more:
		return false;
	}
	Serial.println("card initialized.");

	//write down the date (year / month / day         prints only the start, so if the logger runs for sevenal days you only findt the start back at the begin.


	dataFile = SD.open("gps_xyz.txt", FILE_WRITE);
	if (dataFile)
	{
		Serial.println("Start logging:");
		dataFile.println(" ");
		dataFile.println("Start logging:");
		dataFile.println("Nr| Pitch| Roll  | Yaw   | Alt  |Lattitude|Lontittude|");

		dataFile.close();
	}
	else        	// if the file isn't open, pop up an error:
	{
		Serial.println("error opening gps_xyz.txt");
		return false;
	}


	return true;
}


//-----------------------------------------------------------------------------


void write_GPS_on_SDCard(int32_t lat, int32_t lng, int32_t alt)
{
	//dataFile.println( " 1.123456, 2.123456, 12312 ");

	//dataFile = SD.open("gps_xyz.txt", FILE_WRITE);
	if (dataFile && logData)
	{

		//Serial.print(counter);
		//Serial.print(' ');
		//dataFile.print(counter);
		dataFile.print(' ');
		dataFile.print(lat);
		dataFile.print(' ');
		dataFile.print(lng);
		dataFile.print(' ');
		dataFile.println(alt);
		dataFile.print(' ');
		//dataFile.println(std::chrono::system_clock::now());

		logData = false;    
		//dataFile.close();

		//counter++;
	}

	else        	// if the file isn't open, pop up an error:
		Serial.println("error opening gps_xyz.txt");

}

//-----------------------------------------------------------------------------
void write_PRY_on_SDCard(float pitch, float roll, float yaw)
{
	//dataFile = SD.open("gps_xyz.txt", FILE_WRITE);
	if (dataFile)
	{
		Serial.print(counter);
		Serial.print(' ');
		dataFile.print(counter);
		dataFile.print(' ');
		dataFile.print(pitch, 4);  // dataFile.print(pitch, 2);  ?? ??????? ?????? ??? ????? ????? ??????? ?? ????? ??????? ??????
		dataFile.print(' ');
		dataFile.print(roll, 4);
		dataFile.print(' ');
		dataFile.print(yaw, 4);

		//dataFile.close();
		logData = true;   // Jeigu mes pataikem i PRY ir nesusilauzeme pusiaukelyje, tada mes turime TRUE ir tada galime i kita keisa eit
		counter++;
	}

	else        	// if the file isn't open, pop up an error:
		Serial.println("error opening gps_xyz.txt");

}


//-----------------------------------------------------------------------------
inline void do_shut()    //  Timer mozhet sdelat
{
	Serial.println("_shut_");
	//PIN_CAMERA
	digitalWrite(PIN_CAMERA, 1);
	delay(5);                         //pulso trukme
	digitalWrite(PIN_CAMERA, 0);
}





//-----------------------------------------------------------------------------
int RC_get_channel_value(int ch)
{
	int t = (int)pulseIn(ch, HIGH, 500000); //  , 50000);      // read throttle/left stick  (25000 us = 25 ms)  - timeout (1000000 us = 1000 ms - Defualt)
	return t;
}



//=============================================================================
void setup()
{

#ifndef ARDUINO_DUE_MEGA
	pinMode(RxPin0, INPUT);
	pinMode(TxPin0, OUTPUT);
#endif

	pinMode(PIN_RC_RX_CH, INPUT);
	pinMode(PIN_CAMERA, OUTPUT);



	// MAVLink interface start
	// Serial.begin(57600);

#ifdef SOFT_SERIAL_DEBUGGING
	// [DEB] Soft serial port start
	Serial.begin(57600);
	pxSerial.begin(57600);
	Serial.println("MAVLink starting.");
#endif

	if (!init_sdcard())
	{
		Serial.write("ERROR!!! ");

		// sd karta test
		/*
		while (1)
		{
			digitalWrite(PIN_CAMERA, 1);
			delay(50);
			digitalWrite(PIN_CAMERA, 0);
			delay(50);
		}
		*/
	}



	for (int i = 0; i < 5; i++)
	{
		digitalWrite(PIN_CAMERA, 1);
		delay(100);
		digitalWrite(PIN_CAMERA, 0);
		delay(100);
	}

}


///////////////////////////////////////////////////////////////////////////////
void loop()
{



	//Serial.print('.');
	ch9_value = RC_get_channel_value(PIN_RC_RX_CH);



	// MAVLink
	/* The default UART header for your MCU */
	int sysid = 1;                   ///< ID 20 for this airplane. 1 PX, 255 ground station
	int compid = 158;                ///< The component sending the message
	int type = MAV_TYPE_FIXED_WING;   ///< This system is an airplane / fixed wing
    // Define the system type, in this case an airplane -> on-board controller
    //uint8_t system_type = MAV_TYPE_FIXED_WING;
	//uint8_t autopilot_type = MAV_AUTOPILOT_PIXHAWK;

	//uint8_t system_type = MAV_TYPE_GENERIC;
	//uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;MAV_AUTOPILOT_PIXHAWK

	uint8_t system_type = MAV_TYPE_FIXED_WING;
	uint8_t autopilot_type = MAV_AUTOPILOT_PIXHAWK;


	uint8_t system_mode = MAV_MODE_PREFLIGHT;///< Booting up
	uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
	uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
											  // Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	// Pack the message
	//mavlink_msg_heartbeat_pack(sysid,compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
	mavlink_msg_heartbeat_pack(1, 0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);

	// Copy the message to the send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

	// Send the message with the standard UART send function
	// uart0_send might be named differently depending on
	// the individual microcontroller / library in use.
	unsigned long currentMillisMAVLink = millis();
	if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) {
		previousMillisMAVLink = currentMillisMAVLink;

#ifdef SOFT_SERIAL_DEBUGGING
		pxSerial.write(buf, len);
		//Serial.println("Ardu HB");
#else
		Serial.write(buf, len);
#endif

		//Mav_Request_Data();
		num_hbs_pasados++;
		if (num_hbs_pasados >= num_hbs)
		{
			// Request streams from Pixhawk
#ifdef SOFT_SERIAL_DEBUGGING
			Serial.println("Streams requested!");
#endif
			Mav_Request_Data();
			num_hbs_pasados = 0;
		}

	}

	// Check reception buffer
	comm_receive();

} // LOOP




void Mav_Request_Data()
{
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];


	// STREAMS that can be requested
	/*
	Definitions are in common.h: enum MAV_DATA_STREAM
	MAV_DATA_STREAM_ALL=0, // Enable all data streams
	MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
	MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
	MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
	MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
	MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
	MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
	MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
	MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
	MAV_DATA_STREAM_ENUM_END=13,
	Data in PixHawk available in:
	- Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
	- Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
	*/

	// To be setup according to the needed information to be requested from the Pixhawk
	//const int  maxStreams = 4;
	//const uint8_t MAVStreams[maxStreams] = { MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_POSITION, MAV_DATA_STREAM_RAW_SENSORS, MAV_DATA_STREAM_RAW_CONTROLLER };
	//const uint16_t MAVRates[maxStreams] = { 0x01, 0x01, 0x01, 0x01 };   // 0x01 - ??????? ??? ???????

	const int  maxStreams = 2;
	const uint8_t MAVStreams[maxStreams] = { MAVLINK_MSG_ID_ATTITUDE, MAVLINK_MSG_ID_GPS_RAW_INT};
	const uint16_t MAVRates[maxStreams] = { 0x05, 0x05};


	for (int i = 0; i < maxStreams; i++)
	{
		mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
		uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

#ifdef SOFT_SERIAL_DEBUGGING

		pxSerial.write(buf, len);
#else
		Serial.write(buf, len);
#endif
	}

}



void comm_receive()
{

	//mavlink_attitude_t attitude;
	mavlink_message_t msg;
	mavlink_status_t status;

	// Echo for manual debugging
	// Serial.println("---Start---");

#ifdef SOFT_SERIAL_DEBUGGING
	while (pxSerial.available() > 0)
	{
		uint8_t c = pxSerial.read();
#else
	while (Serial.available() > 0) {
		uint8_t c = Serial.read();
#endif

		// Try to get a new message
		if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
		{
			dataFile = SD.open("gps_xyz.txt", FILE_WRITE);
			if (ch9_value > 1500)
				flag_active_on = true; 					// ON

			else 			///if (ch9_value > 1800)  //if ( ch9_value > 1800  && ch9_value < 2200)
				flag_active_on = false;
			
			//Serial.println(msg.msgid);
			// Handle message
			switch (msg.msgid)
			{
				
//------------------------------------------YAW,PITCH,ROLL (Atrodo to ko mums reikia)-------------------------------------------------------------------------------------------------
						//Rezultatas:
						//Yaw:0.52 -- Roll angle (-pi..+pi)
						//Pitch:-0.21 -- Pitch angle (-pi..+pi)
						//Roll:0.02 -- Yaw angle (-pi..+pi)
						///*
			case MAVLINK_MSG_ID_ATTITUDE:  // #30
			{
				mavlink_attitude_t attitude;
				mavlink_msg_attitude_decode(&msg, &attitude);
#ifdef SOFT_SERIAL_DEBUGGING
				Serial.print("Pitch: ");
				Serial.println(attitude.pitch, 4);
				Serial.print("Roll: ");
				Serial.println(attitude.roll, 3);
				Serial.print("Yaw: ");
				Serial.println(attitude.yaw, 5);
#endif
		
				if (flag_active_on)
				{
					//do_shut();          // 
					                    //Laiko tarpas tarp nuotrauku sudarymo.


					write_PRY_on_SDCard(
						attitude.pitch,
						attitude.roll,
						attitude.yaw
					);  // ????????? ?????? GPS ? ????

					Serial.println();
				}

			}
			break;


//--------------------------------------Kitas b?das gauti GPS (Veikia)----------------------------------------
						//Rezultatas:
						//Alt2:246370
						//Lat2:54.7014308
						//Lon2:25.2162280


			case MAVLINK_MSG_ID_GPS_RAW_INT:  // #24
			{

				mavlink_gps_raw_int_t coordinate;
				mavlink_msg_gps_raw_int_decode(&msg, &coordinate);
#ifdef SOFT_SERIAL_DEBUGGING
				Serial.print("Alt2: ");
				Serial.println(coordinate.alt);
				Serial.print("Lat2: ");
				Serial.println(coordinate.lat);
				Serial.print("Lon2: ");
				Serial.println(coordinate.lon);
#endif
		
				if (flag_active_on && logData)
				{
					write_GPS_on_SDCard
					(
						coordinate.alt,
						coordinate.lat,
						coordinate.lon
					);  //GPS
					do_shut();
					delay(3000);
					Serial.println();
				}

				Serial.println();
				Serial.print("ch9: ");
				Serial.println(ch9_value);
				Serial.println();
			}
			break;
			
			}

			dataFile.close();
			
		}
	}
	}



