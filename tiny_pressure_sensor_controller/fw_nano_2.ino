#include <microDS18B20.h>
#include <GyverOLED.h>

#define DELTA_mV_Press 0
// напряжение питания датчика в милливольтах
//!от значения напряжения и его стабильности сильно зависит точность измерений
#define Input_mV 4920
#define pin_Press_sensor A0
#define SERIAL_BAUD 9600
// Цифровые датчики температуры, безадресно 18b20 на D3, D5 и D8
MicroDS18B20<3> T_system_sensor; // на плате
MicroDS18B20<5> T_cold_water_sensor; // ХВС
MicroDS18B20<8> T_hot_water_sensor; // ГВС

GyverOLED<SSD1306_128x64, OLED_NO_BUFFER> oled;

unsigned long last_time;
unsigned int Press_read = 0;
unsigned int Press_bar = 0;
unsigned int T_system = 0;
unsigned int T_cold_water = 0;
unsigned int T_hot_water = 0;

float Press_sensor=1;

void setup(){
  Serial.begin(SERIAL_BAUD);
    pinMode(pin_Press_sensor, INPUT);
  oled.init();        // инициализация
  oled.clear();       // очистка
  oled.setContrast(230);
  oled.setScale(2);   // масштаб текста (1..4)
  oled.home();        // курсор в 0,0
  oled.print("Измерение");
  oled.setCursor(0, 4);
  oled.print("температуры");
  oled.setCursor(0, 6);
  oled.print("и давления");
    delay(2000);
  oled.clear();
}

void loop(){
  // ансихронный таймер
  if(millis()-last_time > 1000){
  // запрос давления
  Pressure_Data_read();
  // запрос температуры
  Temperature();
  // вывод показаний на экран
  Oled_print();
  }
}
void Temperature(){
  T_system_sensor.requestTemp();
  T_cold_water_sensor.requestTemp();
  T_hot_water_sensor.requestTemp();
  // Т на плате
  if (T_system_sensor.readTemp()) T_system = T_system_sensor.getTemp();
  else T_system = "error";
  Serial.print("T_system: ");
  Serial.print(T_system);
  // Т ХВС
  if (T_cold_water_sensor.readTemp()) T_cold_water = T_cold_water_sensor.getTemp();
  else T_cold_water = "error";
  Serial.print(" T_cold_water: ");
  Serial.print(T_cold_water);
  // Т ГВС
  if (T_hot_water_sensor.readTemp()) T_hot_water = T_hot_water_sensor.getTemp();
  else T_hot_water = "error";
  Serial.print(" T_hot_water: ");
  Serial.print(T_hot_water);
}

void Pressure_Data_read(){
  const byte count_read = 10;
  int Analog_data_read[count_read];                              // массив для хранения данных
      for (byte i = 0; i < count_read; i++){
      Analog_data_read[i] = analogRead(pin_Press_sensor);     // считываем вход и помещаем величину в ячейки массива
      delay(2);
      }                                                       // сортируем массив по возрастанию значений в ячейках
        int New_data = 0;                                     // временная переменная
        for (byte i = 0; i < count_read; i++){
          for (byte j = 0; j < count_read - 1; j++){
              if (Analog_data_read[j] > Analog_data_read[j + 1]){    // упорядочиваем массив в порядке возрастания

              New_data = Analog_data_read[j];
              Analog_data_read[j] = Analog_data_read[j + 1];
              Analog_data_read[j + 1] = New_data;
              }
            }
          }
      Press_read = Analog_data_read[count_read/2];             // берем среднее значение из полученного массива
      Press_read = map(Press_read, 0, 1023, 0, Input_mV)-DELTA_mV_Press;            // масштабируем полученную величину в милливольты
  Serial.print(" Press mV: ");
  Serial.print(Press_read);
  Serial.print(" -> ");

  if ((Press_read >= 500)&&(Press_read <= 4500)){                // берем значения из рабочего диапазона датчика давления
  Press_bar = (map(Press_read, 500, 4500, 0, 1000));    // масштабируем милливольты в значения давления (500 - верхний диапазон датчика давления для датчика 0,5 МПа)
  Press_sensor = Press_bar*0.01;                         // переводим значение в float
  }
  else if (Press_read < 500){                             // присваиваем граничные значения при выходе за пределы рабочего диапазона датчика давления
    Press_sensor = 0.00;  }
  else if (Press_read > 4500){
    Press_sensor = 10.0;  }
Serial.print(" Press BAR: ");
Serial.println(Press_sensor);
}

void Oled_print(){
  oled.autoPrintln(true);
  oled.setContrast(100);
  oled.setScale(1);
  oled.home();
  oled.print("Напряжение: ");
  oled.print(Press_read);
  oled.print(" mV ");
  oled.setCursor(0, 1);
  oled.print("Давление:   ");
  oled.print(Press_sensor);
  oled.print(" bar");
  oled.setCursor(0, 2);
  oled.setScale(1);
  oled.print("На плате:      ");
  oled.setScale(2);
  oled.print(T_system);
  oled.print("C");
  oled.setCursor(0, 4);
  oled.setScale(1);
  oled.print("Холодная вода: ");
  oled.setScale(2);
  oled.print(T_cold_water);
  oled.print("C");
  oled.setCursor(0, 6);
  oled.setScale(1);
  oled.print("Горячая вода:  ");
  oled.setScale(2);
  oled.print(T_hot_water);
  oled.print("C");
}