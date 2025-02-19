#include "JY901.h"

void setup()
{
	Serial.begin(115200);
	JY901.StartIIC();
  Serial.println("IIC started");
  // reset angle to 0
  JY901.WriteWord(0x76, 0x00);
} 

void loop()
{
	//print received data. Data was received in serialEvent;
  Serial.print("Angle: ");
  JY901.GetAngle();
  double z_angle = (double)JY901.stcAngle.Angle[2] / 32768.0 * 180;
  Serial.println(z_angle);
	delay(500);
}