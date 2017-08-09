#include <Wire.h>
#include <EEPROM.h>
#include <RotaryEncoder.h>
#include <LiquidCrystal_I2C.h>

float tensaoA0;

#define AMOSTRAS 100

LiquidCrystal_I2C lcd(0x3F,16,2);

float vin = 0.0;
float R1 = 30000.0;
float R2 = 7640.0;
int mVperAmp = 132; 
int ACSoffset = 2500;
double Voltage = 0;
double Amps = 0;
int lcdr = 5;

RotaryEncoder encoder(A2, A3);

int address = 0;
int valor = 0;
int newPos = 0;
const int sw = 8;
const int pwm = 5;

void setup()
{
  pinMode(sw, INPUT);
  pinMode(pwm, OUTPUT);
  TCCR0B = TCCR0B & B11111000 | B00000001;    // set timer 0 divisor to 1 for PWM frequency of 62500.00 Hz
  lcd.init();
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("CassioKF  V1.0");
  lcd.setCursor(0, 1);
  lcd.print("Iniciando...");
  delay(1000);
  lcd.clear();
  encoder.setPosition(EEPROM.read(address));

  // You may have to modify the next 2 lines if using other pins than A2 and A3
  PCICR |= (1 << PCIE1);    // This enables Pin Change Interrupt 1 that covers the Analog input pins or Port C.
  PCMSK1 |= (1 << PCINT10) | (1 << PCINT11);  // This enables the interrupt for pin 2 and 3 of Port C.
} // setup()

// The Interrupt Service Routine for Pin Change Interrupt 1
// This routine will only be called on any signal change on A2 and A3: exactly where we need to check.
ISR(PCINT1_vect) {
  encoder.tick(); // just call tick() to check the state.
}

float lePorta(uint8_t portaAnalogica) {
  float total=0;  
  for (int i=0; i<AMOSTRAS; i++) {
    total += 1.0 * analogRead(portaAnalogica);
    delay(5);
  }
  return total / (float)AMOSTRAS;
}

void loop()
{
  static int pos = 0;

  int newPos = encoder.getPosition();
  float set = (newPos / 10.0);
  if (pos != newPos) {
    pos = newPos;
    analogWrite(pwm, newPos);
  } // if

  if (newPos > 255){
      newPos = 255;
      encoder.setPosition(255);
    }
  if (newPos < 0){
      newPos = 0;
      encoder.setPosition(0);
    }

  valor = digitalRead(sw);

  tensaoA0 = (lePorta(A0) * 5.0) / 1024.0;
  vin = tensaoA0 / (R2/(R1+R2)); 

  Voltage = (lePorta(A1) * 5000.0) / 1024.0;
  Amps = ((Voltage - ACSoffset) / mVperAmp);

  if (Amps <= 0){
    Amps =0;
  }

  float potencia = (vin * Amps);

  if (potencia <= 0){
    potencia =0;
  }

  if (valor != 1)
  {
    EEPROM.update(address, encoder.getPosition());

    while (digitalRead(sw) == 0)
      delay(10);
  }

  if(lcdr < 5){
    lcdr++;
  }
  if(lcdr >= 5){
    
    if (EEPROM.read(address) == newPos){
      lcd.setCursor(6, 1);
      lcd.print("s");
    }
    if (EEPROM.read(address) != newPos){
      lcd.setCursor(6, 1);
      lcd.print(" ");
    }
    lcd.setCursor(0, 0);
    lcd.print("T=");
    lcd.setCursor(2, 0);
    lcd.print(vin,2);
    //lcd.print("V");
    lcd.setCursor(8, 0);
    lcd.print("C=");
    lcd.setCursor(10, 0);
    lcd.print(Amps,2);
    lcd.setCursor(15, 0);
    lcd.print("A");
    lcd.setCursor(0, 1);
    lcd.print("S=");
    lcd.setCursor(2, 1);
    lcd.print(set);

    lcd.setCursor(8, 1);
    lcd.print("P=");
    lcd.setCursor(10, 1);
    lcd.print(potencia,1);
    lcd.setCursor(15, 1);
    lcd.print("W");
    lcdr = 0;
  }
} // loop ()

// The End

