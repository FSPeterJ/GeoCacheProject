#include <math.h>
/******************************************************************************

GeoCache Hunt Project (GeoCache.cpp)

This is skeleton code provided as a project development guideline only.  You
are not required to follow this coding structure.  You are free to implement
your project however you wish.

Team Number:

Team Members:

1. Gustavo Lattari
2. Peter Jefferys
3.
4.

NOTES:

You only have 32k of program space and 2k of data space.  You must
use your program and data space wisely and sparingly.  You must also be
very conscious to properly configure the digital pin usage of the boards,
else weird things will happen.

The Arduino GCC sprintf() does not support printing floats or doubles.  You should
consider using sprintf(), dtostrf(), strtok() and strtod() for message string
parsing and converting between floats and strings.

The GPS provides latitude and longitude in degrees minutes format (DDDMM.MMMM).
You will need convert it to Decimal Degrees format (DDD.DDDD).  The switch on the
GPS Shield must be set to the "Soft Serial" position, else you will not receive
any GPS messages.

*******************************************************************************

Following is the GPS Shield "GPRMC" Message Structure.  This message is received
once a second.  You must parse the message to obtain the parameters required for
the GeoCache project.  GPS provides coordinates in Degrees Minutes (DDDMM.MMMM).
The coordinates in the following GPRMC sample message, after converting to Decimal
Degrees format(DDD.DDDDDD) is latitude(23.118757) and longitude(120.274060).  By
the way, this coordinate is GlobalTop Technology in Taiwan, who designed and
manufactured the GPS Chip.

"$GPRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,123.48,260406,3.05,W,A*2C/r/n"

$GPRMC,         // GPRMC Message
064951.000,     // utc time hhmmss.sss
A,              // status A=data valid or V=data not valid
2307.1256,      // Latitude 2307.1256 (degrees minutes format dddmm.mmmm)
N,              // N/S Indicator N=north or S=south
12016.4438,     // Longitude 12016.4438 (degrees minutes format dddmm.mmmm)
E,              // E/W Indicator E=east or W=west
0.03,           // Speed over ground knots
165.48,         // Course over ground (decimal degrees format ddd.dd)
260406,         // date ddmmyy
3.05,           // Magnetic variation (decimal degrees format ddd.dd)
W,              // E=east or W=west
A               // Mode A=Autonomous D=differential E=Estimated
*2C             // checksum
/r/n            // return and newline

Following are approximate results calculated from above GPS GPRMC message
(when GPS_ON == 0) to the GEOLAT0/GEOLON0 tree location:

degMin2DecDeg() LAT 2307.1256 N = 23.118757 decimal degrees
degMin2DecDeg() LON 12016.4438 E = 120.274060 decimal degrees
calcDistance() to GEOLAT0/GEOLON0 target = 45335760 feet
calcBearing() to GEOLAT0/GEOLON0 target = 22.999655 degrees

The resulting relative target bearing to the tree is 217.519650 degrees

******************************************************************************/

/*
Configuration settings.

These defines makes it easy for you to enable/disable certain
code during the development and debugging cycle of this project.
There may not be sufficient room in the PROGRAM or DATA memory to
enable all these libraries at the same time.  You must have have
NEO_ON, GPS_ON and SDC_ON during the actual GeoCache Flag Hunt on
Finals Day
*/


#define DEBUG 1


// NOTE: You must not use digital pins 0, 1, 6, 7, 8, 10, 11, 12, 13 for implementing your button.  
// These digital pins are being used by GPS, SecureDigital and NeoPixel.

#define NEO_ON 1		// NeoPixelShield
#define NEO_DEBUG_ON 0		// NeoPixelShield
#define TRM_ON 1		// SerialTerminal
#define SDC_ON 1		// SecureDigital
#define GPS_ON 0	// Live GPS Message (off = simulated)

// define pin usage
#define NEO_TX	6		// NEO transmit
#define GPS_TX	7		// GPS transmit
#define GPS_RX	8		// GPS receive


#define DISTANCE_LONG_FACTOR 25
#define DISTANCE_MED_FACTOR 10
#define DISTANCE_SHORT_FACTOR 1

#define COLORSTAGING 3
#define LED_BAR_LENGTH 5.0

#define BUFFER_SIZE 5  //number of history records to store

// GPS message buffer was at 128 default, but GPRMC spec does not exceed 82 characters per message - PBJ
#define GPS_RX_BUFSIZ	83
char cstr[GPS_RX_BUFSIZ];

// global variables
uint8_t locdataCurrent = 0;// The latest location index
uint8_t target = 0;		// Selected target number 
float heading = 0.0;	// current heading in angle
float distance = 0.0;	// target distance in feet
float bearing = 0.0;	// target bearing in angle

unsigned long rendertime;



struct loc {
	float lat;
	float lon;
	char NS;
	char EW;
};

struct locdata : loc {
	float time;
	float heading;
};



loc targets[1] = {
	//Tree out front
	loc{
		28.594532,
		-81.304437,
		'N',
		'W'
	}
};


//Short-term History - Per second
locdata locdataBuffer[BUFFER_SIZE];

#if GPS_ON
#include <SoftwareSerial.h>
SoftwareSerial gps(GPS_RX, GPS_TX);
#endif

#if NEO_ON
#include "Adafruit_NeoPixel.h"
Adafruit_NeoPixel strip = Adafruit_NeoPixel(40, NEO_TX, NEO_GRB + NEO_KHZ800);
#endif

#if SDC_ON
#include <SD.h>
#endif

/*
Following is a Decimal Degrees formatted waypoint for the large tree
in the parking lot just outside the front entrance of FS3B-116.
*/
#define GEOLAT0 28.594532
#define GEOLON0 -81.304437

#if GPS_ON
/*
These are GPS command messages (only a few are used).
*/
#define PMTK_AWAKE "$PMTK010,002*2D"
#define PMTK_STANDBY "$PMTK161,0*28"
#define PMTK_Q_RELEASE "$PMTK605*31"
#define PMTK_ENABLE_WAAS "$PMTK301,2*2E"
#define PMTK_ENABLE_SBAS "$PMTK313,1*2E"
#define PMTK_CMD_HOT_START "$PMTK101*32"
#define PMTK_CMD_WARM_START "$PMTK102*31"
#define PMTK_CMD_COLD_START "$PMTK103*30"
#define PMTK_CMD_FULL_COLD_START "$PMTK104*37"
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17"
#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_API_SET_FIX_CTL_1HZ  "$PMTK300,1000,0,0,0,0*1C"
#define PMTK_API_SET_FIX_CTL_5HZ  "$PMTK300,200,0,0,0,0*2F"
#define PMTK_SET_NMEA_OUTPUT_RMC "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_GGA "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

#endif // GPS_ON

/*************************************************
**** GEO FUNCTIONS - BEGIN ***********************
*************************************************/

/**************************************************
Convert Degrees Minutes (DDDMM.MMMM) into Decimal Degrees (DDD.DDDD)

float degMin2DecDeg(char *cind, char *ccor)

Input:
cind = string char pointer containing the GPRMC latitude(N/S) or longitude (E/W) indicator
ccor = string char pointer containing the GPRMC latitude or longitude DDDMM.MMMM coordinate

Return:
Decimal degrees coordinate.

**************************************************/
float degMin2DecDeg(char *cind, char *ccor)
{
	//Source for reference : http://stackoverflow.com/questions/18442158/latitude-longitude-in-wrong-format-dddmm-mmmm-2832-3396n -G

	float degrees = 0.0;
	double a = strtod(ccor, 0);
	double d = (int)a / 100;
	a -= d * 100;
	degrees = (float)(d + (a / 60));

	if (cind[0] == 'N' || cind[0] == 'E')
	{
		//degrees positive if north or east
	}
	//else degrees negative if south or west
	else if (cind[0] = 'S' || cind[0] == 'W')
		degrees *= -1;

	return(degrees);
}


loc Midpoint(loc location[], int length) {
	loc temp;
	for (int i = 0; i < length; i++)
	{
		temp.lat += location[i].lat;
		temp.lon += location[i].lon;
	}
	temp.lat /= length;
	temp.lon /= length;
	temp.NS = location[0].NS;
	temp.EW = location[0].EW;
	return tmp;
}

/**************************************************
Calculate Great Circle Distance between to coordinates using
Haversine formula.

float  (float flat1, float flon1, float flat2, float flon2)

EARTH_RADIUS_FEET = 3959.00 radius miles * 5280 feet per mile

Input:
flat1, flon1 = first latitude and longitude coordinate in decimal degrees
flat2, flon2 = second latitude and longitude coordinate in decimal degrees

Return:
distance in feet (3959 earth radius in miles * 5280 feet per mile)
**************************************************/
float calcDistance(float flat1, float flon1, float flat2, float flon2)
{

	//Code extracted from https://rosettacode.org/wiki/Haversine_formula - G

	float distance = 0.0;

	float dx, dy, dz;
	flon1 -= flon2;

	flon1 *= (3.1415926536 / 180), flat1 *= (3.1415926536 / 180), flat2 *= (3.1415926536 / 180);

	dz = sin(flat1) - sin(flat2);
	dx = cos(flon1) * cos(flat1) - cos(flat2);
	dy = sin(flon1) * cos(flat1);

	distance = (asin(sqrt(dx * dx + dy * dy + dz * dz) / 2) * 2 * 6371);

	// This will return distance in KM -G
	//Conversion from KM to feet : distance *= 3280.8 -G

	distance *= 3280.839895;
	return(distance);
}



/**************************************************
Calculate Great Circle Bearing between two coordinates

float calcBearing(float flat1, float flon1, float flat2, float flon2)

Input:
flat1, flon1 = first latitude and longitude coordinate in decimal degrees
flat2, flon2 = second latitude and longitude coordinate in decimal degrees

Return:
angle in decimal degrees from magnetic north (normalize to a range of 0 to 360)
**************************************************/
float calcBearing(float flat1, float flon1, float flat2, float flon2)
{

	//Source: http://mathforum.org/library/drmath/view/55417.html

	float bearing = 0.0;
	float distance = calcDistance(flat1, flon1, flat2, flon2);
	float temp = (sin(flat2) - sin(flat1)*cos(distance)) / (sin(distance) * cos(flat1));
	bearing = acos(temp);

	return(bearing);
}

/*************************************************
**** GEO FUNCTIONS - END**************************
*************************************************/



/*
Sets target number, heading and distance on NeoPixel Display

NOTE: Target number, bearing and distance parameters used
by this function do not need to be passed in, since these
parameters are in global data space.

*/
#if NEO_ON




uint8_t dColors[] = {
	0,
	127,//3 type COLORSTAGING
	255//2 type COLORSTAGING
};



uint32_t StagedColor(int number) {
	int r = (number % COLORSTAGING);
	int g = (int)((float)number * (1.0f / 3.0f)) % COLORSTAGING;
	int b = (int)((float)number * (1.0f / 9.0f)) % COLORSTAGING;
	return strip.Color(dColors[r], dColors[g], dColors[b]);
}


void DistanceBarRender(float dist = distance, int factor = 25) {
	if (dist > 1000)
	{
		factor = DISTANCE_LONG_FACTOR;
	}
	else if (dist > 100)
	{
		factor = DISTANCE_MED_FACTOR;
	}
	else
	{
		factor = DISTANCE_SHORT_FACTOR;
	}
	for (int i = 0; i < 5; i++)
	{
		//This math mess makes me sad :( - Probably can be optimized better PBJ
		strip.setPixelColor(i * 8, StagedColor((dist + ((LED_BAR_LENGTH * factor) - i*factor + factor)) / (LED_BAR_LENGTH * factor)));
		strip.show();
	}
};



void MapRender() {
	for (uint8_t i = 0; i< 25, i++) {
		strip.setPixelColor(i + ((i / 5) * 3), 0);
	};
	uint8_t x;
	uint8_t y;
	x = radians;

}


void setNeoPixel(void)
{
	// Update min. every 250ms
	if ((rendertime - millis()) > 250) {
		rendertime = millis();
		DistanceBarRender(distance);
	}

}


#endif	// NEO_ON
/*

*/
void ProcessWeightedAverage() {


	int largestDistance = 0;
	locdata * distanceClosest;
	float largestDistanceSum = 0;
	//Cycle points
	for (int i = 0; i < BUFFER_SIZE; i++)
	{
		locdata * closest;
		float closestDist;
		float distanceSum;
		//Cycle distances to other points from point
		for (int c = 0; c < BUFFER_SIZE; c++)
		{
			if (c != i)
			{
				float pointdist = calcDistance(locdataBuffer[i].lat, locdataBuffer[i].lon, locdataBuffer[c].lat, locdataBuffer[c].lon);
				distanceSum += pointdist;
				if (pointdist < closestDist) {
					closest = &locdataBuffer[c];
					closestDist = pointdist;
				}
			}
		}
		if (distanceSum > largestDistanceSum) {
			largestDistance = i;
			distanceClosest = closest;
			largestDistanceSum = distanceSum;
		}
	}
	locdataBuffer[largestDistance] = Midpoint({ locdataBuffer[largestDistance], *distanceClosest }, 2);


}






/*

Following is the GPS Shield "GPRMC" Message Structure.This message is received
once a second.You must parse the message to obtain the parameters required for
the GeoCache project.GPS provides coordinates in Degrees Minutes(DDDMM.MMMM).
The coordinates in the following GPRMC sample message, after converting to Decimal
Degrees format(DDD.DDDDDD) is latitude(23.118757) and longitude(120.274060).By
the way, this coordinate is GlobalTop Technology in Taiwan, who designed and
manufactured the GPS Chip.

"$GPRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,123.48,260406,3.05,W,A*2C/r/n"

$GPRMC,         // GPRMC Message
064951.000,     // utc time hhmmss.sss
A,              // status A=data valid or V=data not valid
2307.1256,      // Latitude 2307.1256 (degrees minutes format dddmm.mmmm)
N,              // N/S Indicator N=north or S=south
12016.4438,     // Longitude 12016.4438 (degrees minutes format dddmm.mmmm)
E,              // E/W Indicator E=east or W=west
0.03,           // Speed over ground knots
165.48,         // Course over ground (decimal degrees format ddd.dd)
260406,         // date ddmmyy
3.05,           // Magnetic variation (decimal degrees format ddd.dd)
W,              // E=east or W=west
A               // Mode A=Autonomous D=differential E=Estimated
* 2C             // checksum
/ r / n            // return and newline

*/
void ProcessGPSMessage() {
	//Check if valid RMC
	if (cstr[18] == 'V') {
		Serial.println(cstr);
		locdataCurrent = ++locdataCurrent % BUFFER_SIZE;
		char substrbuffer[16];

		memcpy(substrbuffer, cstr + 7, sizeof(substrbuffer));
		//substrbuffer[13] = '\0';
		locdataBuffer[locdataCurrent].time = atof(substrbuffer);
		int i = 20;
		int s = i;
		//Latitude


		while (cstr[i] != ',' && i < sizeof(cstr))
		{
			i++;
		}
		if (i != s)
		{
			memset(substrbuffer, 0, sizeof(substrbuffer)); 	//Clear buffer
			memcpy(substrbuffer, &cstr[s], i - s);

			////substrbuffer[11] = substrbuffer[3];
			//// substrbuffer[12] = substrbuffer[3];
			//Serial.println("String in Memory");
			//
			////sscanf(substrbuffer, "%f",&( locdataBuffer[locdataCurrent].lat));
			//locdataBuffer[locdataCurrent].lat = atof(substrbuffer);
			//Serial.println("atof");
			//Serial.println(locdataBuffer[locdataCurrent].lat, 6);
			//locdataBuffer[locdataCurrent].lat = strtod(&substrbuffer[0], NULL);
			//Serial.println("strtod");
			//Serial.println(locdataBuffer[locdataCurrent].lat, 6);
			//String tempLat2(locdataBuffer[locdataCurrent].lat, 6);
			//Serial.println("Reconvert cstr");
			//Serial.println(tempLat2);

		}

		i++;
		s = i;
		while (cstr[i] != ',' && i < sizeof(cstr))
		{

			i++;
		}
		if (i != s)
		{
			locdataBuffer[locdataCurrent].lat = degMin2DecDeg(&cstr[s], &substrbuffer[0]);
			Serial.println(locdataBuffer[locdataCurrent].lat);

			//use sscanf instead;
			//memset(substrbuffer, 0, sizeof(substrbuffer));
			//memcpy(&substrbuffer, &cstr[s], i - s);
			locdataBuffer[locdataCurrent].NS = cstr[s];
		}


		i++;
		s = i;
		//Longitude
		while (cstr[i] != ',' && i < sizeof(cstr))
		{
			i++;
		}
		if (i != s)
		{
			memset(substrbuffer, 0, sizeof(substrbuffer)); 	//Clear buffer
			memcpy(&substrbuffer, &cstr[s], i - s);
		}
		i++;
		s = i;
		while (cstr[i] != ',' && i < sizeof(cstr))
		{
			i++;
		}
		if (i != s)
		{
			locdataBuffer[locdataCurrent].lat = degMin2DecDeg(&cstr[s], &substrbuffer[0]);
			locdataBuffer[locdataCurrent].EW = cstr[s];
		}
		//memset(substrbuffer, 0, sizeof(substrbuffer)); 	//Clear buffer
		cstr[18] = 'D';  //prevent reduntant

		for (int i = 0; i < BUFFER_SIZE; i++)
		{
			char tempBuffer[200];
			String tempBearing(locdataBuffer[i].heading, 8);
			String tempTime(locdataBuffer[i].time, 8);
			String tempLat(locdataBuffer[i].lat, 8);
			String tempLon(locdataBuffer[i].lon, 8);
			sprintf(tempBuffer, "Local data %i: Heading: %s Lat: %s Long: %s Time: %s ", i, tempBearing.c_str(), tempLat.c_str(), tempLon.c_str(), tempTime.c_str());
			Serial.println(tempBuffer);

		}
	};
}

#if GPS_ON
/*
Get valid GPS message. This function returns ONLY once a second.

NOTE: DO NOT CHANGE THIS CODE !!!

void getGPSMessage(void)

Side affects:
Message is placed in global "cstr" string buffer.

Input:
none

Return:
none

*/
void getGPSMessage(void)
{


	uint8_t x = 0, y = 0, isum = 0;

	memset(cstr, 0, sizeof(cstr));

	// get nmea string
	while (true)
	{
		if (gps.peek() != -1)
		{
			cstr[x] = gps.read();

			// if multiple inline messages, then restart
			if ((x != 0) && (cstr[x] == '$'))
			{
#if DEBUG
				Serial.println(cstr);
#endif // DEBUG
				x = 0;
				cstr[x] = '$';
			}

			// if complete message
			if ((cstr[0] == '$') && (cstr[x++] == '\n'))
			{
				// nul terminate string before /r/n
				cstr[x - 2] = 0;

				// if checksum not found
				if (cstr[x - 5] != '*')
				{
					x = 0;
					continue;
				}

				// convert hex checksum to binary
				isum = strtol(&cstr[x - 4], NULL, 16);

				// reverse checksum
				for (y = 1; y < (x - 5); y++) isum ^= cstr[y];

				// if invalid checksum
				if (isum != 0)
				{
					x = 0;
					continue;
				}
				// else valid message
				break;
			}
		}
	}
#if DEBUG
	Serial.println(cstr);
#endif // DEBUG


}

#else
/*
Get simulated GPS message once a second.

This is the same message and coordinates as described at the top of this
file.  You could edit these coordinates to point to the tree out front (GEOLAT0,
GEOLON0) to test your distance and direction calculations.  Just note that the
tree coordinates are in Decimal Degrees format, and the message coordinates are
in Degrees Minutes format.

NOTE: DO NOT CHANGE THIS CODE !!!

void getGPSMessage(void)

Side affects:
Static GPRMC message is placed in global "cstr" null terminated char string buffer.

Input:
none

Return:
none

*/

void getGPSMessage(void)
{
	static unsigned long gpsTime = 0;

	// simulate waiting for message
	while (gpsTime > millis()) delay(100);

	// do this once a second
	gpsTime = millis() + 1000;
	distance += 1;
	memcpy(cstr, "$GPRMC,064951.000,V,2307.125654,N,12016.443854,E,0.03,165.48,260406,3.05,W,A*2C", sizeof(cstr));


	return;
}

#endif	// GPS_ON

void SecureDigWrite() {
	// if GPRMC message (3rd letter = R)
	while (cstr[3] == 'R')
	{
		// parse message parameters
		// calculated destination heading
		// calculated destination distance
		break;
	}
}

void TargetChange() {

	//target += 1;
}


void CalculateDistanceBearing() {
	loc temp = Midpoint(locdataBuffer, BUFFER_SIZE);
	heading = calcBearing(temp.lat, temp.lon, locdataBuffer[locdataCurrent].lat, locdataBuffer[locdataCurrent].lon);
	float bearingNORTH = calcBearing(locdataBuffer[locdataCurrent].lat, locdataBuffer[locdataCurrent].lon, targets[target].lat, targets[target].lon);
	bearing = bearingNORTH - heading;
	distance = calcDistance(locdataBuffer[locdataCurrent].lat, locdataBuffer[locdataCurrent].lon, targets[target].lat, targets[target].lon);
}


//==============
// Setup
//=====================================================
// Only the main loop should be below this function
//=====================================================
void setup(void)
{
#if TRM_ON
	// init serial interface
	Serial.begin(115200);
#endif	

	//Referenced https://github.com/adafruit/Adafruit_NeoPixel/blob/master/examples/strandtest/strandtest.ino
#if NEO_ON
	// init NeoPixel Shield
	pinMode(NEO_ON, OUTPUT);
	strip.begin();
	strip.setBrightness(16);
	strip.show(); // Initialize all pixels to 'off'
#endif	

#if SDC_ON
	/*
	Initialize the SecureDigitalCard and open a numbered sequenced file
	name "MyMapNN.txt" for storing your coordinates, where NN is the
	sequential number of the file.  The filename can not be more than 8
	chars in length (excluding the ".txt").
	*/
#endif

#if GPS_ON
	// enable GPS sending GPRMC message
	gps.begin(9600);
	gps.println(PMTK_SET_NMEA_UPDATE_1HZ);
	gps.println(PMTK_API_SET_FIX_CTL_1HZ);
	gps.println(PMTK_SET_NMEA_OUTPUT_RMC);
#endif		

	// init target button here
}




//==============
// MAIN LOOP
//=====================================================
// Nothing should be after this function
// Do not write logic in main loop.
//=====================================================
void loop(void)
{
	// max 1 second blocking call till GPS message received

	getGPSMessage();

	ProcessGPSMessage();
	//ProcessWeightedAverage();
	CalculateDistanceBearing();

	// if button pressed, set new target
	TargetChange();

#if SDC_ON
	// write current position to SecureDigital then flush
	SecureDigWrite();
#endif 

#if NEO_ON
	// set NeoPixel target display target, heading, distance
	setNeoPixel();
#endif		

#if TRM_ON
	// print debug information to Serial Terminal
#endif		
}

