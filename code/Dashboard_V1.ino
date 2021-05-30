#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include <Servo.h>

#include <buffer.h>
#include <crc.h>
#include <datatypes.h>
#include <VescUart.h>

#include <EEPROM.h>
int mah_addr = 0;
int volt_addr = 1;

VescUart UART;

Servo VESC;

LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2); //0x27 for PCF8574T chip;0x3F for PCF8574AT chip

int throttleInput = A0;
int throttleVal = 0;
int pwmValue = 0;

int prev_bat_remaining = 0;

int time_1 = 0;
//int time_2 = 0;

float spd = 0;
float range_remaining = 0;

float bat_remaining = 0;
int bat_max_discharge_mah = 16000; //80% of rated capacity

void setup() {
  Serial.begin(115200);

  VESC.attach(3, 1000, 2000);
  VESC.writeMicroseconds(1500);

  lcd.init();
  lcd.backlight();
  lcd.home();
  lcd.clear();
  
  lcd.print("Connecting...");
  
  while (!Serial) {;}

  UART.setSerialPort(&Serial);
  
  lcd.clear();
  lcd.home();

  time_1 = millis();

  int prev_bat_remaining = EEPROM.read(mah_addr);

  int prev_bat_voltage = EEPROM.read(volt_addr);

  if (prev_bat_remaining >= 1)
  {
    prev_bat_remaining = 1;
  }

  if (UART.getVescValues())
  {
    if (prev_bat_voltage > (round(UART.data.inpVoltage)) + 2)
    {
      prev_bat_remaining = 1;
    }
  }
}

void loop() {
  throttleVal = analogRead(throttleInput);
  //Serial.print("Throttle Input Value:");
  //Serial.print(throttleVal);
  //Serial.print(" ");
  //Serial.print("PWM Output Value:");
  pwmValue = map(throttleVal, 185, 865, 1000, 2000);
  //Serial.println(pwmValue);
  
  VESC.writeMicroseconds(pwmValue);

  if (UART.getVescValues())
  {
    
    //int spd = round((UART.data.tachometer) * 0.06942684 * 0.000991650884 * 60); //RPM * Gear ratio = wheel RPM; wheel RPM * wheel circ in miles = miles/min; miles/min*60 = mph
/*
    if ((millis() - time_1) > 500)
    {      
      time_1 = millis();
      spd = round((UART.data.rpm) * 0.06942684 * 0.000991650884 * 60);
      lcd.setCursor(0,0);
      lcd.print("SPD: ");
      lcd.setCursor(4,0);
      if (spd < 10)
      {
        lcd.print(0);
        lcd.setCursor(5,0);
      }
      lcd.print(spd);
      lcd.setCursor(6,0);
      lcd.print("mph");
    }
*/  if ((millis() - time_1) > 2000)
    {
      time_1 = millis();
      
      int bat_remaining = round(((1-(UART.data.ampHours/(bat_max_discharge_mah)))*100) - (1-prev_bat_remaining));
      int bat_volts = round(UART.data.inpVoltage);

      lcd.setCursor(0,0);
      lcd.print("BAT: ");
      lcd.setCursor(4,0);
      lcd.print(bat_remaining);
      lcd.setCursor(8,0);
      lcd.print("%");
  
  
      EEPROM.write(mah_addr, bat_remaining);
      EEPROM.write(volt_addr, bat_volts);
    }
    
    //int range_remaining = round(((UART.data.tachometerAbs * 0.06942684 * 0.000991650884)/(1-bat_remaining)) * bat_remaining); //Total distance in miles traveled divided by battery used, times battery remaining (miles/Ah * Ah left in battery)
  
    //lcd.setCursor(12,1);
    //lcd.print(range_remaining);
    //lcd.setCursor(14,1);
    //lcd.print("mi");

  }
  else
  {
    Serial.println('Failed to get VESC data');
  }

  

  
}
