
#include <Wire.h>
#include <SPI.h>
#include <WiFi101.h>

// Local Network Settings
char ssid[] = "YOUR_SSID";			// your network SSID (name)
char pass[] = "YOUR_PWD";			// your network password

int wifiConnectionAttemptCounter = 0;
int status = WL_IDLE_STATUS;

// Initialize Arduino Ethernet Client
WiFiClient client;
//WiFiServer server(80);

/* timer variables 
unsigned long currentMillis;        // store the current value from millis()
unsigned long previousMillis;       // for comparison with currentMillis
unsigned int samplingInterval = 19; // how often to sample analog inputs (in ms)
*/
#define devAddress		0x29				// I2C adddress to access BNO055 
#define dimBuf			6					// 
#define	led				6					// 
#define reg1			0xff				//


// ****** BNO055 CONFIG REGISTER	*****************
#define regPageID		0x07
#define regSysStatus	0x39
#define regPowerMode	0x3E
#define regOprMode		0x3D
#define regSysErrorCode	0x3A
#define regSysStatCode	0x39
#define	regAccelConfig	0x08


// *****	BNO055 ACCELERATION OUTPUT DATA REGISTERS	******* 
#define regAcelDat_X_L	0x08
#define regAcelDat_X_H	0x09
#define regAcelDat_Y_L	0x0A
#define regAcelDat_Y_H	0x0B
#define regAcelDat_Z_L	0x0C
#define	regAcelDat_Z_H	0x0D
// ******************************************

#define configMode			0x00			// To config BNO055
#define accelOnlyMode		0x01			// BNO055 as accelerometer only
#define accelAndGyroMode	0x05			// BNO055 as accelerometer and gyroscope
#define	accelRange_2G		0x00			// 2G	
#define	accelRange_4G		0x01			// 4G
#define accelBW_7			0x00			// 7.81Hz
#define accelBW_15			0x04			// 15.63Hz
#define	accelOM_Normal		0x00			// Operational Mode Normal
#define accelOM_Suspended	0x20			// Operational Mode Suspended
#define accelUnitSel_ms		0x00			// Unit selected m/s²
#define accelUnitSel_mg		0x01			// Unit selected mg
#define pwrMode_Normal		0x00			//

String message;
byte buff[dimBuf];							// 


void printWifiStatus() {
	// print the SSID of the network you're attached to:
	Serial.print("SSID: ");
	Serial.println(WiFi.SSID());

	// print your WiFi shield's IP address:
	IPAddress ip = WiFi.localIP();
	Serial.print("IP Address: ");
	Serial.println(ip);

	// print the received signal strength:
	long rssi = WiFi.RSSI();
	Serial.print("signal strength (RSSI):");
	Serial.print(rssi);
	Serial.println(" dBm");
}

void errCode(void) {						// reads status and error code
	Serial.println("");
	readReg(devAddress, regSysStatCode, buff, 2);
	Serial.print("STAT= ");
	Serial.println(buff[0]);
	Serial.print("ERR= ");
	Serial.println(buff[1]);
	Serial.println("");
}

unsigned int magnitude(short xd, short yd, short zd) {
	unsigned int res =(unsigned int) sq(xd) + (unsigned int)sq(yd) + (unsigned int)sq(zd);
	res = sqrt(res);
	return res;
}

void writeReg(byte adr, byte reg, byte dat) {
  Wire.beginTransmission(adr);
  Wire.write(byte(reg));
  Wire.write(byte(dat));
  Wire.endTransmission();
}

void readReg(byte adr, byte reg, byte *buf, byte nDat) {
  Wire.beginTransmission(adr);
  Wire.write(byte(reg));
  Wire.endTransmission();
  Wire.requestFrom(adr, nDat);
  byte rcvd = Wire.available();
  if (nDat <= rcvd) {
    for (int i = 0; i < rcvd; i++) {
      buf[i] = Wire.read();
    }
  }
}

void configAccel(void) {								// configure BNO055 to use the accelerometer only

	writeReg(devAddress, regPageID, 0);					// set page 0 for read data
	readReg(devAddress, regPageID, buff, 1);
	
//	Serial.print("Pag ");
//	Serial.println(buff[0]);	


	writeReg(devAddress, regOprMode, configMode);		// set config mode
	readReg(devAddress, regOprMode, buff, 1);	
//	if (buff[0] == configMode) Serial.println("Modo Configuracion");



	writeReg(devAddress, regPowerMode, pwrMode_Normal);
	readReg(devAddress, regPowerMode, buff, 1);
//	if (buff[0] == pwrMode_Normal) Serial.println("PWR MODE NORMAL");



	writeReg(devAddress, regPageID, 1);					// set page 1 for config reg
	readReg(devAddress, regPageID, buff, 1);
//	if (buff[0] == 1)Serial.println("Pag 1");

	writeReg(devAddress, regAccelConfig, (accelRange_2G | accelBW_7 | accelOM_Normal));	// config accelerometer
	readReg(devAddress, regAccelConfig, buff, 1);
//	if (buff[0] == (accelRange_2G | accelBW_7 | accelOM_Normal)) Serial.println("Accel configurado");

	writeReg(devAddress, regPageID, 0);					// set page 0 for read data
	readReg(devAddress, regPageID, buff, 1);

	writeReg(devAddress, regOprMode, accelOnlyMode);	// set accelerometer only
	readReg(devAddress, regOprMode, buff, 1);
//	if (buff[0] == accelOnlyMode) Serial.println("Modo Accelerometro");

}

unsigned int accelData(void) {									// reads x, y, z component of accel and returns resulting magnitude
	short xd = 0;
	short yd = 0;
	short zd = 0;
	unsigned int kd;
	readReg(devAddress, regAcelDat_X_L, buff, 6);		// 

	xd = buff[1] << 8;
	xd = xd | buff[0];
	
	yd = buff[3]<<8;
	yd = yd | buff[2];
	
	zd = buff[5] << 8;
	zd = zd | buff[4];

	kd = magnitude(xd, yd, zd);
//	Serial.print("Mag= ");
//	Serial.println(r);
	return kd;
}

String messageCreator(unsigned int dat) {
	String sensor = "\"Sensor\":\"Sensor_1\"";
	String magAc = "\"Magnitude Accel\":" + String (dat);
	String comp = "(" + sensor + "," + magAc + ")";
	return comp;
}

void sendData(void) {
	Serial.println("Connecting...");
	if (client.connect("YourAzure URL ServiceAddress", 80)) {
		client.println("POST /tables/yourRegister HTTP/1.1");
		client.println("Host: YourAzure URL ServiceAddress");
		client.println("X-ZUMO-APPLICATION: yourAzureAccessKey");
		client.println("Content-Type: application/json");
		client.println("Content-Length: xx");		// xx: your data (message) length
		client.println();
		client.println(message);
		client.println();
		Serial.println("Data sent");
	}
	else {
		Serial.println("Failed connection...");
	}
}

void waitAnswer(void) {
	while (!client.available()) {
		if (!client.connected()) {
			return;
		}
	}
}

void readAnswer(void) {
	bool print = true;
	while (client.available()) {
		char c = client.read();
		if (c == '\n') print = false;
		if (print) Serial.print(c);
	}
}

void endOp(void) {
	client.stop();
}



void setup() {
	Serial.begin(115200);
	while (!Serial) {		// wait for serial port to connect. 
		; 
	}

	Wire.begin();

	if (WiFi.status() == WL_NO_SHIELD) {		// check for the presence of the shield:
		Serial.println("WiFi shield not present");
		
		while (true);							// don't continue
	}
	while (status != WL_CONNECTED) {			// attempt to connect to Wifi network
		Serial.print("Attempting to connect to SSID: ");
		Serial.println(ssid);
		status = WiFi.begin(ssid, pass);		// Connect to WPA/WPA2 network. 
		delay(10000);							// wait 10 seconds for connection
	}
	printWifiStatus();							//  print out the status

	pinMode(led, OUTPUT);
	configAccel();
	Serial.println("SetUp Ready");
}

byte cnt=0;
#define dimData		18					//	data buffer for last 3 min	
unsigned int data[dimData];
unsigned int r;

void loop() {
	unsigned int med = 0;
	Serial.print("DAT: ");
	Serial.println(cnt);

	med = accelData();		// reads actual accel data and average
	med += accelData();
	med >>= 1;
	data[cnt] = med;		// 
	Serial.print("Mag= ");
	Serial.println(med);
	Serial.println();
	cnt++;
	if (cnt >= dimData) {
		unsigned int t = 0;
		for (cnt = 0; cnt < dimData; cnt++) {
			t += data[cnt];
		}
		r = t/dimData;
		cnt = 0;
	
		message = messageCreator(r);
		sendData();
		waitAnswer();
		readAnswer();
		endOp();
	}
	delay(10000);			// waits 10 secs.
 }
