//MCP47xxx_Testing.ino
#include <Wire.h>
#define ADR 0x60
#define SYS_STATUS_REG 0x0A
#define VREF_REG 0x08
#define POWERDOWN_REG 0x09

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(38400);
}

void loop() {
  // put your main code here, to run repeatedly:
  // SetGain(0, 0, false);
  // SetRef(0b1, 0, false);
  Wire.beginTransmission(ADR);  //Set REF
  Wire.write(0x40);
  // Wire.endTransmission();

  // Wire.beginTransmission(ADR);
  // Wire.write(0x20);
  // Wire.beginTransmission(ADR);
  Wire.write(0x00);
  Wire.write(0b10);
  Wire.endTransmission();


  Serial.println(SetVoltageRaw(255, 0));
  delay(1000);
  Serial.println(SetVoltageRaw(1024, 0));
  delay(1000);
}

void SetVoltage(float Val, bool Module)
{

}

int SetVoltageRaw(uint16_t Val, bool Module)
{
	Wire.beginTransmission(ADR);
	Wire.write(0x00); //DEBUG!
	Wire.write(Val >> 8); //Write LSB
	Wire.write(Val & 0xFF); //Write MSB
	return Wire.endTransmission();
}

void SetRef(uint8_t Mode, bool Module, bool Temp)
{
	uint16_t TempReg = 0; //Temp val for storing register value
	Wire.beginTransmission(ADR); //Point to System status reg
	Wire.write(VREF_REG);
	Wire.endTransmission();
	Wire.requestFrom(ADR, 2); //Read in reg value
	TempReg = Wire.read(); //Read in LSB
	TempReg = Wire.read() << 8;  //Read in MSB
	TempReg = TempReg & (0x0000 | (0x03 << (Module*2)));
	TempReg = TempReg | (Mode << (Module*2));

	Serial.println(TempReg, HEX); //DEBUG!
	Wire.beginTransmission(ADR);
	Wire.write(SYS_STATUS_REG);
	Wire.write(TempReg >> 8);
	Wire.write(TempReg & 0xFF);
	return Wire.endTransmission();

}

void SetPowerMode(uint8_t Mode)
{

}

int SetGain(uint8_t Mode, bool Module, bool Temp)  //Set gain mode for a given module, set in non-volatile with Temp = TRUE
{
	uint16_t TempReg = 0; //Temp val for storing register value
	Wire.beginTransmission(ADR); //Point to System status reg
	Wire.write(SYS_STATUS_REG);
	Wire.endTransmission();
	Wire.requestFrom(ADR, 2); //Read in reg value
	TempReg = Wire.read(); //Read in LSB
	TempReg = Wire.read() << 8;  //Read in MSB
	TempReg = TempReg & (0xFC3F | 1 << ( + Module));
	TempReg = TempReg | (Mode << (7 + Module));

	Serial.println(TempReg, HEX); //DEBUG!
	Wire.beginTransmission(ADR);
	Wire.write(SYS_STATUS_REG);
	Wire.write(TempReg >> 8);
	Wire.write(TempReg & 0xFF);
	return Wire.endTransmission();
}
