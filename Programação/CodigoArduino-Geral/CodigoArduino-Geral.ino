#include <SPI.h> 
// #include <LoRa.h>
#include "Adafruit_CCS811.h"// expensaor de porta
#include <Arduino.h>
#include <MPU9250_WE.h> //mpu Giro - Acl - MaG
#include <Wire.h> //i2C
#include <MCP23017.h> // Co2
#include <BMx280I2C.h> //Barometro e temp
#include <WiFi.h>
#include "uFire_SHT20.h"


#define MCP23017_ADDR 0x20
#define bmp280_ADDR 0x76
#define MPU_ADDR 0x68

const uint8_t ligado = 1;
const uint8_t desligado = 0;
const uint8_t OFF = 0;
const uint8_t RED = 1;
const uint8_t GREEN = 2;
const uint8_t BLUE = 3;
const uint8_t WHITE = 4;
const uint8_t PURPLE = 5;
const char* ssid     = "Nokia 5.4";
const char* password = "Arkhe151299";
uint16_t mcpState = 0;

WiFiServer server(80);

MCP23017 mcp = MCP23017(MCP23017_ADDR);
Adafruit_CCS811 ccs;
BMx280I2C bmx280(bmp280_ADDR);
MPU9250_WE myMPU9250 = MPU9250_WE(MPU_ADDR);
uFire_SHT20 sht20;

void setup() {
  Serial.begin(115200);

  while (!Serial);

	Wire.begin();

  mcp.init();
  mcp.portMode(MCP23017Port::A, 0);  //Port A as output
  mcp.portMode(MCP23017Port::B, 0);  //Port B as output
  mcp.writeRegister(MCP23017Register::GPIO_A, 0x00);  //Reset port A 
  mcp.writeRegister(MCP23017Register::GPIO_B, 0x00);  //Reset port B
  mcp.write(0);

  sht20.begin();

  Serial.println("CCS811 test");

  if(!ccs.begin() || !bmx280.begin()){
    Serial.println("Failed to start sensor! Please check your wiring.");
    while(1);
  }

  if(!myMPU9250.init())
    Serial.println("MPU9250 does not respond");
  else
    Serial.println("MPU9250 is connected");
  

  if (bmx280.isBME280())
		Serial.println("sensor is a BME280");
	else
		Serial.println("sensor is a BMP280");

	bmx280.resetToDefaults();
  bmx280.writeOversamplingPressure(BMx280MI::OSRS_P_x16);
	bmx280.writeOversamplingTemperature(BMx280MI::OSRS_T_x16);
  
  while(!ccs.available());

  Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
  delay(1000);
  myMPU9250.autoOffsets();
  Serial.println("Done!");

  // Serial.println("LoRa Receiver");

  // LoRa.setPins(2,13,14); // Setano Pinos SS, RST e DI00

  // if (!LoRa.begin(433E6)) { //Se o módulo não iniciar na frequencia 433Mhz faça:
  //   Serial.println("Starting LoRa failed!");
  //   while (1);
  // } else{
  //   Serial.println("Lora Iniciado");
  // }
  
  myMPU9250.enableGyrDLPF();
  myMPU9250.setGyrDLPF(MPU9250_DLPF_6);
  myMPU9250.setSampleRateDivider(99);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);

  // We start by connecting to a WiFi network

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  server.begin();

}

void loop() {
/*********************************************************************/
/* Medição Luz */
  int sensorValue = analogRead(34);  // Lê o valor do sensor
  float percent = map(sensorValue, 0, 4095, 0, 100);


  Serial.print("Dados Cru da Luz:");
  Serial.println(sensorValue);
  Serial.print("Porcentagem luz:");
  Serial.println(percent);

  // // try to parse packet
  // int packetSize = LoRa.parsePacket();

  // if (packetSize) {
  //   // received a packet
  //   Serial.print("Received packet '");

  //   // read packet
  //   while (LoRa.available()) {
  //     Serial.print((char)LoRa.read());
  //   }

  //   // print RSSI of packet
  //   Serial.print("' with RSSI ");
  //   Serial.println(LoRa.packetRssi());
  // }

/**************************************************************************************/
/* RGBS */

  if (percent < 50){
    setCorRGB(RED);
  } else if ( percent >=50 && percent < 75){
    setCorRGB(PURPLE);
  } else if (percent >= 75){
    setCorRGB (BLUE);
  }

/**************************************************************************************/
/* CO2 */

  if(ccs.available()){
    if(!ccs.readData()){
      Serial.print("CO2: ");
      Serial.print(ccs.geteCO2());
      Serial.print("ppm, TVOC: ");
      Serial.println(ccs.getTVOC());
    }
  }

/**************************************************************************************/
/* Humidade */
    Serial.println();
    sht20.measure_all();
//    Serial.println((String)sht20.tempC + "°C"); Juntar com as outras temp

    Serial.println((String)sht20.RH + " %RH");
    Serial.println((String)sht20.vpd() + " kPa VPD");
    Serial.println();

/**************************************************************************************/
/* Barometro */
  float temp = bmx280.getTemperature();

  if (!bmx280.measure())
	{
		Serial.println("could not start measurement, is a measurement already running?");
		return;
	}

  do
	{
		delay(100);
	} while (!bmx280.hasValue());

	Serial.print("Pressure: "); Serial.println(bmx280.getPressure());
	Serial.print("Temperature: "); Serial.println(temp);


/**************************************************************************************/
/* Giroscopio */
  xyzFloat gyrRaw = myMPU9250.getGyrRawValues();
  xyzFloat corrGyrRaw = myMPU9250.getCorrectedGyrRawValues();
  xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat gyr = myMPU9250.getGyrValues();
  xyzFloat angle = myMPU9250.getAngles();

  float resultantG = myMPU9250.getResultantG(gValue);

  Serial.println("Acceleration in g (x,y,z):");
  Serial.print(gValue.x);
  Serial.print("   ");
  Serial.print(gValue.y);
  Serial.print("   ");
  Serial.println(gValue.z);
  Serial.print("Resultant g: ");
  Serial.println(resultantG);

  Serial.println("Gyroscope Data in degrees/s (x,y,z):");
  Serial.print(gyr.x);
  Serial.print("   ");
  Serial.print(gyr.y);
  Serial.print("   ");
  Serial.println(gyr.z);
  Serial.println(WiFi.localIP());

  /* Angles are also based on the corrected raws. Angles are simply calculated by
   angle = arcsin(g Value) */
  Serial.print("Angle x  = ");
  Serial.print(angle.x);
  Serial.print("  |  Angle y  = ");
  Serial.print(angle.y);
  Serial.print("  |  Angle z  = ");
  Serial.println(angle.z);

  Serial.print("Orientation of the module: ");
  Serial.println(myMPU9250.getOrientationAsString());

  Serial.println();

  Serial.println("*********************************");

  delay(1000);

/********************* Wifi ***********************************************************/

WiFiClient client = server.available();   // listen for incoming clients

if (client) {                             // if you get a client,
  Serial.println("New Client.");
  String currentLine = "";

  while (client.connected()) {
    if (client.available()) {
      char c = client.read();
      Serial.write(c);
      
      if (c == '\n') {
        if (currentLine.length() == 0) {
          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:text/html");
          client.println();
          client.println("<html><body>");
          
          // Style the buttons with CSS
          client.println("<style>");
          client.println("button { background-color: #4CAF50; color: white; padding: 15px 20px; font-size: 16px; }");
          client.println("</style>");
          
          // Create buttons for controlling the LED
          client.println("<button><a href=\"/H\">Turn On LED</a></button>");
          client.println("<button><a href=\"/L\">Turn Off LED</a></button>");

          client.println("<a>Gyroscope Data in degrees/s (x,y,z):</a>");
          client.print("<a >" + String(gyr.x) + " "+ String(gyr.y) + " "+ String(gyr.z) + "</a>");
          client.println("</body></html>");
          client.println();
          break;
        } else {
          currentLine = "";
        }
      } else if (c != '\r') {
        currentLine += c;
      }

      if (currentLine.endsWith("GET /H")) {
        setLed(ligado);
      }
      
      if (currentLine.endsWith("GET /L")) {
        setLed(desligado);
      }
    }
  }

  client.stop();
  Serial.println("Client Disconnected.");
}

/**************************************************************************************************************************/

}

void setLed(uint8_t estado){
  if(estado == ligado){
    mcp.digitalWrite(0, 1);
    mcp.digitalWrite(1, 1);
    mcp.digitalWrite(14, 1);
    mcp.digitalWrite(15, 1);
  } else if (estado == desligado){
    mcp.digitalWrite(0, 0);
    mcp.digitalWrite(1, 0);
    mcp.digitalWrite(14, 0);
    mcp.digitalWrite(15, 0);
  }
}

//hiperspectral*****!!!!!!!!! (Olhar depois)!
void setCorRGB(uint8_t cor){
  switch (cor){
      case OFF: //All Off
        mcpState = mcp.read();
        mcpState &= ~(_BV(2) | _BV(3) | _BV(4) | _BV(5) | _BV(6) | _BV(7) | _BV(8) | _BV(9) | _BV(10) | _BV(11) | _BV(12) | _BV(13)); 
        mcp.write(mcpState);
        break;
      case RED: //All Red
        mcpState = mcp.read();
        mcpState &= ~(_BV(2) | _BV(3) | _BV(4) | _BV(5) | _BV(6) | _BV(7) | _BV(8) | _BV(9) | _BV(10) | _BV(11) | _BV(12) | _BV(13));
        mcpState |= (_BV(4) | _BV(7) | _BV(8) | _BV(11));
        mcp.write(mcpState);
        break;
      case GREEN: //All Green
        mcpState = mcp.read();
        mcpState &= ~(_BV(2) | _BV(3) | _BV(4) | _BV(5) | _BV(6) | _BV(7) | _BV(8) | _BV(9) | _BV(10) | _BV(11) | _BV(12) | _BV(13));
        mcpState |= (_BV(3) | _BV(6) | _BV(9) | _BV(12));
        mcp.write(mcpState);
        break;
      case BLUE: //All Blues
        mcpState = mcp.read();
        mcpState &= ~(_BV(2) | _BV(3) | _BV(4) | _BV(5) | _BV(6) | _BV(7) | _BV(8) | _BV(9) | _BV(10) | _BV(11) | _BV(12) | _BV(13));
        mcpState |= (_BV(2) | _BV(5) | _BV(10) | _BV(13));
        mcp.write(mcpState);
        break;
      case WHITE: //All White
        mcpState = mcp.read();
        mcpState &= ~(_BV(2) | _BV(3) | _BV(4) | _BV(5) | _BV(6) | _BV(7) | _BV(8) | _BV(9) | _BV(10) | _BV(11) | _BV(12) | _BV(13));
        mcpState |= (_BV(2) | _BV(3) | _BV(4) | _BV(5) | _BV(6) | _BV(7) | _BV(8) | _BV(9) | _BV(10) | _BV(11) | _BV(12) | _BV(13));
        mcp.write(mcpState);
        break;
      case PURPLE: //All Purple
        mcpState = mcp.read();
        mcpState &= ~(_BV(2) | _BV(3) | _BV(4) | _BV(5) | _BV(6) | _BV(7) | _BV(8) | _BV(9) | _BV(10) | _BV(11) | _BV(12) | _BV(13));
        mcpState |= (_BV(2) | _BV(4) | _BV(5) | _BV(7) | _BV(8) | _BV(10) | _BV(11) | _BV(13));
        mcp.write(mcpState);
        break;
      default:
        mcpState = mcp.read();
        mcp.write(mcpState);
        break;
    }
}






