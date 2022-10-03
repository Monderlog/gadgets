//Спрраведливо для датчика XIBIBEI SENSOR XDB303 0,5-4,5В, 0-1МПа

// for LCD w/ GPIO MODIFIED for the ATtiny85
#include <LiquidCrystal_I2C.h>
// I2C Master lib for ATTinys which use USI - comment this out to use with standard arduinos
#include <TinyWireM.h>
// дельта для калибровки датчика в милливольтах
#define DELTA_mV_Press 0
// напряжение питания датчика в милливольтах
//требуется измерить текущее и указать значение/2, требуется использовать делитель напряжения на резисторах R1=R2 (аналоговый пин имеет ограничение измеряемого напряжения 2500мВ)
//!от значения напряжения и его стабильности сильно зависит точность измерений
#define Input_mV 4980
#define pin_Press_sensor 1
#define SERIAL_BAUD 115200
// одно из стандартных значений, определить можно через Digispark_Examples/i2cScanner
#define GPIO_ADDR 0x27

unsigned long timing=0;
  unsigned int Press_read = 0;
  unsigned int Press_bar = 0;

float Press_sensor=1;
// адрес 16 символьныого дисплея на 2 строки
LiquidCrystal_I2C lcd(GPIO_ADDR,16,2);

void setup() {
Serial.begin(SERIAL_BAUD);

   pinMode(pin_Press_sensor, INPUT);

  TinyWireM.begin();                    // initialize I2C lib - comment this out to use with standard arduinos
  lcd.init();                           // initialize the lcd
  lcd.backlight();                      // Print a message to the LCD.
  delay(100);
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Pressure sensor test");
delay(2000);
   lcd.clear();
}

void loop() {
  if(millis()-timing > 1000)
{
Pressure_Data_read();
LCD_print();
timing = millis();
}
}

void Pressure_Data_read() 
  {
// количество считываний для усреднения
  const byte count_read = 10;
// массив для хранения данных
  int Analog_data_read[count_read];
      for (byte i = 0; i < count_read; i++)
      {
// считываем вход и помещаем величину в ячейки массива
      Analog_data_read[i] = analogRead(pin_Press_sensor);
      delay(2);
// сортируем массив по возрастанию значений в ячейках
      }
// временная переменная
        int New_data = 0;
        for (byte i = 0; i < count_read; i++)
          {
          for (byte j = 0; j < count_read - 1; j++)
            {
// упорядочиваем массив в порядке возрастания
              if (Analog_data_read[j] > Analog_data_read[j + 1])
              {
              New_data = Analog_data_read[j];
              Analog_data_read[j] = Analog_data_read[j + 1];
              Analog_data_read[j + 1] = New_data;
              }
            }
          }
// берем среднее значение из полученного массива
      Press_read = Analog_data_read[count_read/2];
// масштабируем полученную величину в милливольты
      Press_read = map(Press_read, 0, 4980, 0, Input_mV)-DELTA_mV_Press;
  Serial.print(Press_read);
  Serial.print(" -> ");
// берем значения из рабочего диапазона датчика давления
  if ((Press_read >= 500)&&(Press_read <= 4980))
  {
// масштабируем мВ в значения МПа (500/2(делитель на резисторах R1=R2=10кОМ) - верхний диапазон датчика давления для датчика 1 МПа)
  Press_bar = (map(Press_read, 500, 4980, 0, 1000));
// забрасываем значение в float
  Press_sensor = Press_bar*0.01;
  }
// присваиваем граничные значения при выходе за пределы рабочего диапазона датчика давления
  else if (Press_read < 500)
  {  Press_sensor = 0.00;  }
  else if (Press_read > 4980)
  {  Press_sensor = 10.0;  }
Serial.println(Press_sensor);
}

void LCD_print()
{
    lcd.setCursor(0, 0);
  lcd.print("Voltage: ");
  lcd.print(Press_read);
  lcd.print(" mV  ");
    lcd.setCursor(0, 1);
  lcd.print("Press.: ");
  lcd.print(Press_sensor);
  lcd.print(" bar ");
}