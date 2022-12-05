#include <Arduino.h>
#include <SoftwareSerial.h>  
#include "DFRobot_EC.h"
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define EC_PIN A2     //EC接在A2
#define ONE_WIRE_BUS 8    // 告訴 OneWire library DQ 接在那隻腳上 
#define DO_PIN A3     //DO接在A3
#define VREF 5000    //VREF (mv)
#define ADC_RES 1024 //ADC Resolution
#define TWO_POINT_CALIBRATION 0  //DO校正方式 單點校正:0 , 雙點校正:1
#define CAL1_V (730) //單點校正mv
#define CAL1_T (25)   //單點校正℃
#define CAL2_V (1300) //雙點校正mv
#define CAL2_T (15)   //雙點校正℃

#define DEBUG false
#define CYCLETIME 1000 //600000
DFRobot_EC ec;
OneWire oneWire(ONE_WIRE_BUS); // 建立 OneWire 物件
DallasTemperature DS18B20(&oneWire); // 建立 DS18B20 物件

const int PH_SensorPin = A1;  //This is the pin number connected to P1
const int PH_LMPin = A0;      // 讀取PH感測器模組板載 LM35 溫度, 1度c 約 0.033v
float pH_Value, PH_voltage, PH_LM, EC_voltage, ec_Value, temperature, DO_Value;
uint8_t Temperaturet;
uint16_t ADC_Raw, ADC_Voltage, DO;
const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};
String sPH,sEC,sT,sDO;

void setup() {
  Serial.begin(115200);  
  while (!Serial);
  // Sensor - initialize
  DS18B20.begin();
  ec.begin();
}

void loop() {
  ///============================================================Get Temperature
  DS18B20.requestTemperatures();  //Get started to Request Temperatures
  Serial.print("Temperature : ");
  temperature = DS18B20.getTempCByIndex(0);  //Read Temperature from DS18B20
  if (temperature == -127)
    Serial.print("temperature ERROR");
  else  
    Serial.print(temperature);
  Serial.print(".C");
  ///============================================================Get pH Value
  PH_LM = (analogRead(PH_LMPin) / 1024 * 5) / 0.033;
  PH_voltage = analogRead(PH_SensorPin)*5.0/1024;
  pH_Value = 7 + ((3-PH_voltage) / 0.18);
  Serial.print("   PH : ");
  Serial.print(pH_Value,2);
  ///============================================================Get EC Value
  EC_voltage = analogRead(EC_PIN)*5000.0/1024.0;  // read the mV voltage
  ec_Value =  ec.readEC(EC_voltage,temperature);  // convert voltage to EC with temperature compensation
  Serial.print("   EC : ");
  Serial.print(ec_Value,2);
  Serial.print("ms/mm");
  ///============================================================Get DO Value
  Temperaturet = (uint8_t)temperature;
  ADC_Raw = analogRead(DO_PIN);
  ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;
  DO_Value = readDO(ADC_Voltage, Temperaturet)/1000.0;
#if DEBUG
  Serial.println("   DO : " + String(DO_Value) + "mg/L");
#endif
  ///=================================================================Collecting data and to assembling string
  String dataString;
  // Temperature
  if(int(temperature)!=-127){
    if(int(temperature)<10)
      dataString += "0";      
    dataString += String(int(temperature*100));
  }
  else
    dataString += "eeee";
  
  // pH
  if(pH_Value>0 && pH_Value<18){
    if(pH_Value<0.01)dataString += "0000"; //Join pH Value Data
      else if(pH_Value<0.10) {
      dataString += "000"; //Join pH Value Data
      dataString += String(int(pH_Value*100));
    } else if (pH_Value>=0.10 && pH_Value<1.00) {
      dataString += "00"; //Join pH Value Data
      dataString += String(int(pH_Value*100));
    } else if (pH_Value>=1.00 && pH_Value<10.00) {
      dataString += "0"; //Join pH Value Data
      dataString += String(int(pH_Value*100));
    } else if (pH_Value>=10.00) {
      dataString += String(int(pH_Value*100));
    }
  }
  else
    dataString += "eeee";
  // EC
  if(ec_Value!=0){
    if(ec_Value<0.10) {
      dataString += "000"; //Join EC Value Data
      dataString += String(int(ec_Value*100));
    } else if (ec_Value>=0.10 && ec_Value<1.00) {
      dataString += "00"; //Join EC Value Data
      dataString += String(int(ec_Value*100));
    }
    else if (ec_Value>=1.00 && ec_Value<10.00)
    {
      dataString += "0"; //Join EC Value Data
      dataString += String(int(ec_Value*100));
    }
    else if (ec_Value>=10.00) dataString += String(int(ec_Value*100));
  }
  else
    dataString += "eeee";
  // DO
  if(DO_Value!=0){
    if(DO_Value<0.10) {
      dataString += "000"; //Join DO Value Data
      dataString += String(int(DO_Value*100));
    } else if (DO_Value>=0.10 && DO_Value<1.00) {
      dataString += "00"; //Join DO Value Data
      dataString += String(int(DO_Value*100));
    } else if (DO_Value>=1.00 && DO_Value<10.00) {
      dataString += "0"; //Join DO Value Data
      dataString += String(int(DO_Value*100));
    } else if (DO_Value>=10.00) dataString += String(int(DO_Value*100));
  }    
  else
    dataString +="eeee";

  // Send data to D1 mini
  Serial.println(dataString);
  delay(CYCLETIME);

}

///=================================================================Calculate Dissolved Oxygen
int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c)
{
#if TWO_POINT_CALIBRATION == 0
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#else
  uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#endif
}
