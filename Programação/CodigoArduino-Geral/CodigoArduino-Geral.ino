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
#include <WebServer.h>


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
const char* ssid     = "COVID_20";
const char* password = "1c75a33b210";
uint16_t mcpState = 0;
#include <WebServer.h>

WebServer sv(80);
MCP23017 mcp = MCP23017(MCP23017_ADDR);
Adafruit_CCS811 ccs;
BMx280I2C bmx280(bmp280_ADDR);
MPU9250_WE myMPU9250 = MPU9250_WE(MPU_ADDR);
uFire_SHT20 sht20;

/******************************************************************************************************/
/* Subrotinas */

  void conectado() {      //Sub-rotina para caso o servidor fique online

    xyzFloat gValue = myMPU9250.getGValues();
    xyzFloat gyr = myMPU9250.getGyrValues();
    xyzFloat angle = myMPU9250.getAngles();
    int sensorValue = analogRead(34);  // Lê o valor do sensor
    float percentluz = map(sensorValue, 0, 4095, 0, 100);

    sv.send(200, "text/html", html(sht20.tempC, bmx280.getPressure(), sht20.RH, sht20.dew_point(), ccs.geteCO2(), percentluz, gValue.x, gValue.y, gValue.z, angle.x, angle.y, angle.z, gyr.x, gyr.y, gyr.z));     //Envia ao servidor, em formato HTML, o nosso script, com os parâmetros de pressão e temperatura
  }
  void nao_encontrado() {                                                           //Sub-rotina para caso seja retornado um erro
    sv.send(404, "text/plain", "Não encontrado");                                   //Retorna a mensagem de erro em caso de um retorno 404
  }

  String html(float temperatura, float pressao, float humidade, float pOrv, float co2, float lum, float ax, float ay, float az, float angx, float angy, float angz, float gx, float gy, float gz) {                                   //Variável que armazenará o script HTML
    String cd = "<!DOCTYPE html>\n";
      cd += "<html lang=\"pt-br\">\n";
      cd += "<head>\n";
      cd += "<meta charset=\"UTF-8\">\n";
      cd += "<meta http-equiv=\"X-UA-Compatible\" content=\"IE=edge\">\n";
      cd += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no;\">\n";
      cd += "<title>Ambiente de Telemetria</title>\n";
      cd += "<style>\n";
      cd += "html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
      cd += "body{margin-top: 50px;} \n";
      cd += "h1 {color: #444444; margin: 50px auto 30px;}\n";
      cd += "p {font-size: 24px; color: #444444; margin-bottom: 10px;}\n";
      cd += "</style>\n";
      cd += "</head>\n";
      cd += "<div>\n";
      cd += "<tables style=\"border-collapse: collapse; width: 100%; height: 20px;\">\n";
      cd += "<tbody>\n";
      cd += "<tr style=\"height: 150px; display: flex; flex-direction: row; justify-content: center;\">\n";
      cd += "<td style=\"width: 100%; height: 150px; text-align: center;\">\n";
      cd += "<h1 style=\"text-align: center;\"><strong> Ambiente de Telemetria Cefast Aerospace </strong></h1>\n";
      cd += "</td>\n";
      cd += "</tr>\n";
      cd += "</tbody>\n";
      cd += "</table>\n";
      cd += "</div>\n";
      cd += "<div>\n";
      cd += "<table style=\"border-collapse: collapse; width: 100%; height: 20px;\">\n";
      cd += "<tbody>\n";
      cd += "<tr style=\"height: 20px;\">\n";
      cd += "<td style=\"width: 100%; height: 20px; text-align: center;\">\n";
      cd += "<h3><strong> &nbsp;Unidade de Medidas Inercial (IMU) </strong></h3>\n";
      cd += "</td>\n";
      cd += "</tr>\n";
      cd += "</tbody>\n";
      cd += "</table>\n";
      cd += "</div>\n";
      cd += "<div style=\" width: 100%; display: flex; justify-content: center;\">\n";
      cd += "<table border=\"1\" style=\"border-collapse: collapse; width: 80%; height: 80px;\">\n";
      cd += "<tbody>\n";
      cd += "<tr style=\"height: 20px;\">\n";
      cd += "<td style=\"width: 33.3333%; text-align: center;\">Acelerometro</td>\n";
      cd += "<td style=\"width: 33.3333%; text-align: center;\">Giroscópio</td>\n";
      cd += "<td style=\"width: 33.3333%; text-align: center;\">Angulo dos Eixos</td>\n";
      cd += "</tr>\n";
      cd += "<tr style=\"height: 60px;\">\n";
      cd += "<td style=\"width: 33.3333%; text-align: center;\">\n";
      cd += "<table border=\"1\" style=\"border-collapse: collapse; width: 100%;\">\n";
      cd += "<tbody>\n";
      cd += "<tr style=\"height: 20px;\">\n";
      cd += "<td style=\"width: 33.3333%;\">x</td>\n";
      cd += "<td style=\"width: 33.3333%;\">y</td>\n";
      cd += "<td style=\"width: 33.3333%;\">z</td>\n";
      cd += "</tr>\n";
      cd += "<tr style=\"height: 40px;\">\n";
      cd += "<td style=\"width: 33.3333%;\"><a> " + String(ax) + " </a></td>\n";
      cd += "<td style=\"width: 33.3333%;\"><a> " + String(ay) + " </a></td>\n";
      cd += "<td style=\"width: 33.3333%;\"><a> " + String(az) + " </a></td>\n";
      cd += "</tr>\n";
      cd += "</tbody>\n";
      cd += "</table>\n";
      cd += "</td>\n";
      cd += "<td style=\"width: 33.3333%; text-align: center;\">\n";
      cd += "<table border=\"1\" style=\"border-collapse: collapse; width: 100%;\">\n";
      cd += "<tbody>\n";
      cd += "<tr style=\"height: 20px;\">\n";
      cd += "<td style=\"width: 33.3333%;\">x</td>\n";
      cd += "<td style=\"width: 33.3333%;\">y</td>\n";
      cd += "<td style=\"width: 33.3333%;\">z</td>\n";
      cd += "</tr>\n";
      cd += "<tr style=\"height: 40px;\">\n";
      cd += "<td style=\"width: 33.3333%;\"><a> " + String(gx) + " </a></td>\n";
      cd += "<td style=\"width: 33.3333%;\"><a> " + String(gy) + " </a></td>\n";
      cd += "<td style=\"width: 33.3333%;\"><a> " + String(gz) + " </a></td>\n";
      cd += "</tr>\n";
      cd += "</tbody>\n";
      cd += "</table>\n";
      cd += "</td>\n";
      cd += "<td style=\"width: 33.3333%; text-align: center;\">\n";
      cd += "<table border=\"1\" style=\"border-collapse: collapse; width: 100%;\">\n";
      cd += "<tbody>\n";
      cd += "<tr style=\"height: 20 px;\">\n";
      cd += "<td style=\"width: 33.3333%;\">x</td>\n";
      cd += "<td style=\"width: 33.3333%;\">y</td>\n";
      cd += "<td style=\"width: 33.3333%;\">z</td>\n";
      cd += "</tr>\n";
      cd += "<tr style=\"height: 40px;\">\n";
      cd += "<td style=\"width: 33.3333%;\"><a> " + String(angx) + " </a></td>\n";
      cd += "<td style=\"width: 33.3333%;\"><a> " + String(angy) + " </a></td>\n";
      cd += "<td style=\"width: 33.3333%;\"><a> " + String(angz) + " </a></td>\n";
      cd += "</tr>\n";
      cd += "</tbody>\n";
      cd += "</table>\n";
      cd += "</td>\n";
      cd += "</tr>\n";
      cd += "</tbody>\n";
      cd += "</table>\n";
      cd += "</div> \n"; 
      cd += "<div> \n"; 
      cd += "<table style=\"border-collapse: collapse; width: 100%; height: 52px;\">\n";
      cd += "<tbody>\n";
      cd += "<tr style=\"height: 52px;\">\n";
      cd += "<td style=\"height: 52px;\">\n";
      cd += "<h3 style=\"text-align: center;\"><strong> Medição dos Sensores </strong></h3>\n";
      cd += "</td>\n";
      cd += "</tr>\n";
      cd += "</tbody>\n";
      cd += "</table>\n";
      cd += "</div>  \n";  
      cd += "<div style=\" width: 100%; display: flex; justify-content: center;\">\n";
      cd += "<table border=\"1\" style=\"border-collapse: collapse; width: 80%; height: 40px;\">\n";
      cd += "<tbody>\n";
      cd += "<tr style=\"height: 20px;\">\n";
      cd += "<td style=\"width: 16.6667%; height: 20px;\">Temperatura(&deg;C):</td>\n";
      cd += "<td style=\"width: 12.0188%; height: 20px;\"><a> " + String(temperatura) + " </a></td>\n";
      cd += "<td style=\"width: 23.1456%; height: 20px;\">Humidade (%HR):</td>\n";
      cd += "<td style=\"width: 13.9906%; height: 20px;\"><a> " + String(humidade) + " </a></td>\n";
      cd += "<td style=\"width: 20.0469%; height: 20px;\">CO2 (ppm):</td>\n";
      cd += "<td style=\"width: 14.1316%; height: 20px;\"><a> " + String(co2) + " </a></td>\n";
      cd += "</tr>\n";
      cd += "<tr style=\"height: 20px;\">\n";
      cd += "<td style=\"width: 16.6667%; height: 20px;\">Pressão(hPA):</td>\n";
      cd += "<td style=\"width: 12.0188%; height: 20px;\"><a> " + String(pressao) + " </a></td>\n";
      cd += "<td style=\"width: 23.1456%; height: 20px;\">Ponto de orvalho(ºC):</td>\n";
      cd += "<td style=\"width: 13.9906%; height: 20px;\"><a> " + String(pOrv) + " </a></td>\n";
      cd += "<td style=\"width: 20.0469%; height: 20px;\">Luminosidade (%):</td>\n";
      cd += "<td style=\"width: 14.1316%; height: 20px;\"><a> " + String(lum) + " </a></td>\n";
      cd += "</tr>\n";
      cd += "</tbody>\n";
      cd += "</table>\n";
      cd += "</div>\n";
      cd += "<div style=\" width: 100%; display: flex; justify-content: center; margin-top: 30px; \">\n";
      cd += "<table  style=\"border-collapse: collapse; width: 80%; height: 40px; \">\n";
      cd += "<tbody>\n";
      cd += "<tr style=\"height: 20px;\">\n";
      cd += "<td style=\"width: 16.6667%; height: 20px; padding: 20px;\"></td>\n";
      cd += "<td style=\"width: 12.0188%; height: 20px; padding: 20px;\"><button> <a href=/T> RGB Temperatura </a> </button></td></td>\n";
      cd += "<td style=\"width: 23.1456%; height: 20px; padding: 20px;\"><button> <a href=/H> Ligar as LED </a> </button></td>\n";
      cd += "<td style=\"width: 13.9906%; height: 20px; padding: 20px;\"></td>\n";
      cd += "</tr>\n";
      cd += "<tr style=\"height: 20px;\">\n";
      cd += "<td style=\"width: 16.6667%; height: 20px; padding: 20px;\"></td>\n";
      cd += "<td style=\"width: 12.0188%; height: 20px; padding: 20px;\"><button> <a href=/S> RGB Luminosidade </a> </button></td></td>\n";
      cd += "<td style=\"width: 23.1456%; height: 20px; padding: 20px;\"><button> <a href=/L> Desligar as LED </a> </button></td>\n";
      cd += "<td style=\"width: 13.9906%; height: 20px; padding: 20px;\"></td>\n";
      cd += "</tr>\n";
      cd += "</tbody>\n";
      cd += "</table>\n";
      cd += "</div>\n";
      cd += "<style>\n";
      cd += "button { background-color: #5081a9; color: white; padding: 15px 20px; font-size: 16px; border-radius: 10px; cursor: pointer;}\n";
      cd += "a { text-decoration: none }\n";
      cd += "</style>\n";
      cd += "</html>\n";

    return cd;                                                                      //Retorna o script                                             
  } 



/***************************************************************************************************/
/* Setup */
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
  
  sv.on("/", conectado);
  sv.onNotFound(nao_encontrado);
  sv.begin();                   

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
/* RGBS */ // Muda de cor de acordo com a luminosidade captada pelo sensor

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
      float co2 = ccs.geteCO2();
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

    float temph = sht20.tempC;
    float hum = sht20.RH;
    float pOrv = sht20.dew_point();

    Serial.println((String)sht20.RH + " %RH");
    Serial.println((String)sht20.vpd() + " kPa VPD");
    Serial.println();

/**************************************************************************************/
/* Barometro */

  if (!bmx280.measure())
	{
		Serial.println("could not start measurement, is a measurement already running?");
		return;
	}

  do
	{
		delay(100);
	} while (!bmx280.hasValue());

  float tempb = bmx280.getTemperature();
  float press = bmx280.getPressure();

	Serial.print("Pressure: "); Serial.println(press);
	Serial.print("Temperature: "); Serial.println(temph);


/**************************************************************************************/
/* Giroscopio */
  xyzFloat gyrRaw = myMPU9250.getGyrRawValues();
  xyzFloat corrGyrRaw = myMPU9250.getCorrectedGyrRawValues();
  xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat gyr = myMPU9250.getGyrValues();
  xyzFloat angle = myMPU9250.getAngles();

  float resultantG = myMPU9250.getResultantG(gValue);

  float tempG = myMPU9250.getTemperature();
  float acelx = gValue.x;
  float acely = gValue.y;
  float acelz = gValue.z;
  float gyrx = gyr.x;
  float gyry = gyr.y;
  float gyrz = gyr.z;
  float anglex = angle.x;
  float angley = angle.y;
  float anglez = angle.z; 

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

  float medtemp = (temph + tempb + tempG)/3;  // Média das temperaturas

/********************* Wifi ***********************************************************/

    sv.handleClient();  


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






