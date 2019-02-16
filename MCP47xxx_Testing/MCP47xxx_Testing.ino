//MCP47xxx_Testing.ino
#include <Wire.h>
#define ADR 0x60
#define SYS_STATUS_REG 0x0A
#define VREF_REG 0x08
#define POWERDOWN_REG 0x09

#define WRITE 0b000 //Wrtie command bits 
#define READ 0b110 //Read command bits

#define VREF_NO_BUF 0b10 //Define voltage ref control value for unbuffered external referance 
#define VREF_BUF 0b11 //Define  voltage ref control value for buffered external referance 
#define INTERNAL_BAND_GAP 0b01 //Define voltage ref control value for using internal band gap (1.22V)
#define VDD 0b00 //Define voltage ref control value for using Vdd as voltage referance 

#define DAC0 0x00 //Define control value for DAC0
#define DAC1 0x01 //Define control value for DAC1

#define GAIN_1 0x00 //Define control sequence for unity gain
#define GAIN_2 0x01 //Define control sequence for x2 gain

#define BIT_NUM 4096 //FIX! Find better way to assign bit number?

const float VddVoltage = 5.0; //FIX! Read this value in using a constructor/begin
const float VRefVoltage = 2.5; //FIX! read this value in using a constructor/begin
uint8_t DAC_Ref[2] = {0}; //Used to keep track internally of the status of DACx referance connection
bool DAC_Gain[2] = {0}; //Used to keep track internally of the status of DACx gain setting
// uint8_t DAC0_Ref = 0; //Used to keep track internally of the status of DAC0 referance connection
// uint8_t DAC1_Ref = 0; //Used to keep track internally of the status of DAC1 referance connection
// bool DAC0_Gain = 0; //Used to keep track internally of the status of DAC0 gain setting
// bool DAC1_Gain = 0; //Used to keep track internally of the status of DAC1 gain setting


void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(38400);
}

void loop() {
  SetGain(GAIN_2, DAC0, false);
  SetRef(VDD, DAC0, false);
  SetVoltage(1.0, DAC0);
  delay(1000);
  // SetRef(VREF_BUF, DAC0, false);
  // SetVoltage(1.0, DAC0);
  // delay(1000);

  SetRef(INTERNAL_BAND_GAP, DAC0, false);
  SetVoltage(1.0, DAC0);
  delay(1000);

  // Wire.beginTransmission(ADR);  //Set REF
  // Wire.write(0x40);

  // Wire.write(0x00);
  // Wire.write(0b10);
  // Wire.endTransmission();


  // Serial.println(SetVoltageRaw(255, 0));
  // delay(1000);
  // SetRef(VREF_BUF, DAC0, false);
  // // SetGain(GAIN_2, DAC0, false);
  // Serial.println(SetVoltageRaw(255, 0));
  // delay(1000);
}

void SetVoltage(float Val, bool Module)
{
	uint16_t RawVal = 0; //Used to corelate float with bit representation

	switch(DAC_Ref[Module]){
		case VREF_NO_BUF:
			RawVal = (Val*BIT_NUM)/(VRefVoltage * (1 + DAC_Gain[Module])); //Convert the desired value into a bit representation using the selected voltage ref and gain
			break;
		case VREF_BUF: //Same as unbuffered, does not effect voltage value
			RawVal = (Val*BIT_NUM)/(VRefVoltage * (1 + DAC_Gain[Module])); //Convert the desired value into a bit representation using the selected voltage ref and gain
			break;
		case INTERNAL_BAND_GAP: //Use fixed 1.22v as ref, additional multiple of 2 added in band-gap path
			RawVal = (Val*BIT_NUM)/(1.22 * (2 + DAC_Gain[Module])); //Convert the desired value into a bit representation using the selected voltage ref and gain
			break;
		case VDD: //Omit gain, since this configuration is not allowed with Vdd as ref
			RawVal = (Val*BIT_NUM)/(VddVoltage); //Convert the desired value into a bit representation using the selected voltage ref and gain
			break;
			//Add default case??
	}
	SetVoltageRaw(RawVal, Module); //Use calulated raw value and pass module through 

}

int SetVoltageRaw(uint16_t Val, bool Module)
{
	Wire.beginTransmission(ADR);
	Wire.write(0x00); //DEBUG!
	Wire.write(Val >> 8); //Write LSB
	Wire.write(Val & 0xFF); //Write MSB
	return Wire.endTransmission();
}

int SetRef(uint8_t Mode, bool Module, bool Temp)
{
	if(Mode == VDD) SetGain(GAIN_1, Module, Temp); //If VDD is used as ref, set gain to 1
	else SetGain(DAC_Gain[Module], Module, Temp); //Otherwise set the gain to stored value
	uint16_t TempReg = 0; //Temp val for storing register value
	Wire.beginTransmission(ADR); //Point to System status reg
	Wire.write((VREF_REG << 3) | READ); //Write register location, along with appended command 
	Wire.endTransmission();
	Wire.requestFrom(ADR, 2); //Read in reg value
	TempReg = Wire.read(); //Read in LSB
	TempReg = Wire.read() << 8;  //Read in MSB
	TempReg = TempReg & (0x0000 | (0x03 << (Module*2)));
	TempReg = TempReg | (Mode << (Module*2));

	// Serial.println(TempReg, HEX); //DEBUG!
	Wire.beginTransmission(ADR);
	Wire.write((VREF_REG << 3) | WRITE); //Write register location, along with appended command 
	Wire.write(TempReg >> 8);  //Write back moddified temp reg
	Wire.write(TempReg & 0xFF);
	int Error = Wire.endTransmission();
	if(Error == 0) DAC_Ref[Module] = Mode; //Only copy value if write was successful 
	return Error; //Return result of I2C transfer
}

void SetPowerMode(uint8_t Mode)
{

}

int SetGain(uint8_t Mode, bool Module, bool Temp)  //Set gain mode for a given module, set in non-volatile with Temp = TRUE
{
	uint16_t TempReg = 0; //Temp val for storing register value
	Wire.beginTransmission(ADR); //Point to System status reg
	Wire.write((SYS_STATUS_REG << 3) | READ); //Write register location, along with appended command 
	Wire.endTransmission();
	Wire.requestFrom(ADR, 2); //Read in reg value
	TempReg = Wire.read(); //Read in LSB
	TempReg = Wire.read() << 8;  //Read in MSB
	TempReg = TempReg & (0xFCFF | 1 << (8 + Module)); //Clear appropriate bit of register (8 or 9)
	TempReg = TempReg | (Mode << (8 + Module)); //Write mode value to appropriate bit location 

	// if(Mode == GAIN_2) TempReg = 0x100; //DEBUG!
	// else TempReg = 0x000; //DEBUG!

	// Serial.println(TempReg, HEX); //DEBUG!
	Wire.beginTransmission(ADR);
	Wire.write((SYS_STATUS_REG << 3) | WRITE); //Write register location, along with appended command 
	Wire.write(TempReg >> 8); //Write back moddified temp reg
	Wire.write(TempReg & 0xFF);
	int Error = Wire.endTransmission();
	if(Error == 0) DAC_Gain[Module] = Mode; //Only copy value if write was successful 
	return Error; //Return result of I2C transfer
}
