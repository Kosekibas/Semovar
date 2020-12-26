#include <Arduino.h>
#include "avr/wdt.h" // подключаем watch dog таймер
#include <OneWire.h>
#include <SPI.h>
#include "SdFat.h"

enum StateWork //перечисление рабочих состояний
{
  kGetInput, //получение входных данных
  kStart,    //начало работы
  kMode,     //режим
  kPwm,      //шим
  kDataLog,  //запись данных
  kWait,     //ожидание
  kStop,     //окончание работы
  kBlocking  //блокировка
};
StateWork current_state;  // переменная рабочих состояний
StateWork previous_state; // переменная предыдущего состояния
byte entry;               //флаг=1 при входе автомата в новое состояние
//--------------------------------переменные для таймеров
#define MAX_TIMERS 4    //количество таймеров
#define TEMPER_TIMER 0  //таймер опроса датчика температуры
#define LOGGING_TIMER 1 //таймер записи данных на карту
#define TRIAC_TIMER 2 //таймер выдачи импульса симистору
//----проверка времен 
long start_time=0;  
long time_work=0;
// состояние таймеров
#define TIMER_STOPPED 0
#define TIMER_RUNNING 1
#define TIMER_PAUSED 2
char timers_states[MAX_TIMERS];    //массив текущих состояний таймера
long timers[MAX_TIMERS];           //переменные количества интервалов таймеров
long process_timer_1 = 0;          //предыдущее число таймера
long process_timer_2 = 0;          //текущее число таймера
long process_timer_interval = 10; //интервал присвоения нового числа таймеру
#define time_interval_triac 1  //интервал для выдачи упраляющих импульсов на симистор
#define time_interval_temper 100  //интервал для опроса температуры
#define time_interval_log 99    //интервал логгирования 
long time_before_start = 0; //интервал присвоения нового числа таймеру

// назначение кнопок
#define BUTTON_START 2 //назначение пина кнопки старт
#define BUTTON_LOG 3   //назначение пина кнопки логгирования
bool is_work = false;  // перешли в режим работы
bool log_on = false;   //логгирование вклчено
// ШИМ
//-#define PWM_OUT 5 //назначение пина шим
int pwm;              // вход переменного резистора
// реле
#define RELAY 4   //назначение пина реле
//----------------------------- сервопривод
int out_servo=0; // установка положения сервопривода в градусах
int servoPin = 9;            // порт подключения сервы
int pulseWidth;              // длительность импульса
//----------------------------- управление симистором
int setpoint=100;	// установка мощности
int modulator=0;	// рабочая переменная
int er=50;        // ошибка
		
#define TRIAC_ON (PORTC |= (1<<PC0)); // на си включение тиристора, 
#define TRIAC_OFF (PORTC &= ~(1<<PC0)); //отключение тиристора
//-------------------------------переменные датчика температуры
OneWire ds(8);         // Создаем объект OneWire для шины 1-Wire, с помощью которого будет осуществляться работа с датчиком
enum TempCommunication //состояния общения с датчиком
{
  kSendRecuest, //передать запрос
  kGetAnsver    //получить ответ
};
TempCommunication temp_state; //переменаая состояния датчика температуры
float temperature;            // переменная хранящая температуру
//------------------------------- переменные SD карты
#define FILE_BASE_NAME "Data"           // общее название логов
const uint8_t chipSelect = SS;          // пин выбора чипа SD.  !!!!!Отключить все остальное с шины SPI!!!!!.
SdFat sd;                               // создаем объект SD карты.
SdFile file;                            // Создаем объект лога.
#define error(msg) sd.errorHalt(F(msg)) //Сообщения об ошибках, хранящиеся во флэш-памяти.

void setup()
{//си настройка АЦП
  ADCSRA |= (1 << ADPS2);                     //Биту ADPS2 присваиваем единицу - коэффициент деления 16
  ADCSRA &= ~ ((1 << ADPS1) | (1 << ADPS0));  //Битам ADPS1 и ADPS0 присваиваем нули
//
  pinMode(BUTTON_START, INPUT);
  pinMode(BUTTON_LOG, INPUT);
  pinMode(RELAY, OUTPUT);  
  digitalWrite(RELAY, LOW);    // выключает реле
  //-pinMode(PWM_OUT, OUTPUT); 
  //-analogWrite(PWM_OUT, 0); //откл шим  
  pinMode(servoPin, OUTPUT);          // пин сервы, как выход
  pinMode(RELAY, OUTPUT); 
  DDRC = (1 << DDC0);// выставляем А0 как цифровой выход
  TRIAC_OFF;
  Serial.begin(9600);
  current_state = kGetInput; // первое состояние- опрос входов
  temp_state = kSendRecuest;
}
//---------------------------функции таймера
void ProcessTimers(void) // прибавление к таймерам через интервал
{
  process_timer_2 = millis();
    if ((process_timer_2 - process_timer_1) >= process_timer_interval)
  {
     process_timer_1 = process_timer_2;
    for (byte i = 0; i < MAX_TIMERS; i++)
      if (timers_states[i] == TIMER_RUNNING)
        timers[i]++;
    wdt_reset(); // сброс сторожевого таймера
  }
}
void StartTimer(byte Timer) //запуск таймера
{
  if (timers_states[Timer] == TIMER_STOPPED)
  {
    timers_states[Timer] = TIMER_RUNNING;
    timers[Timer] = 0;
  }
}
void StopTimer(byte Timer) //Стоп таймера
{
  if (timers_states[Timer] == TIMER_RUNNING)
  {
    timers_states[Timer] = TIMER_STOPPED;
    timers[Timer] = 0;
  }
}
unsigned int GetTimer(byte Timer) // получить время таймера
{
  return timers[Timer];
}
void ResetTimer(byte Timer) // сбросить таймер
{
  if (timers_states[Timer] == TIMER_RUNNING)
    timers[Timer] = 0;
}
void writeHeader()
{ //записывем заголовок файла
  file.print(F("second"));
  file.print(F(",input"));
  file.print(F(",output"));
  file.println();
}

void servoPulse(int servoPin, int myAngle) 
{//поворот сервоприводом
  pulseWidth = (myAngle * 11) + 500;  // конвертируем угол в микросекунды
  digitalWrite(servoPin, HIGH);       // устанавливаем серве высокий уровень
  delayMicroseconds(pulseWidth);      // ждём
  digitalWrite(servoPin, LOW);        // устанавливаем низкий уровень
}

void brasenham(void){
    modulator = setpoint + er; 
     if (modulator < 50)
       {
         TRIAC_OFF;
         er = modulator ; 
       }
          else 
        {
          TRIAC_ON;
          er=modulator-100;
        }

     

    /*
* есть две глобальные переменные:
regValue - то, что хотим получить, пусть будет в интервале [0..99];
regError - накопление ошибки, это придумал Брезенхем. 

* когда выяснилось, какую мощность хотим передать в нагрузку (пусть будет 20 из интервала [0..99]):
regValue = 20;
regError = 50; 
Устанавливаем regError = 99/2, середина интервала. Это нужно делать каждый раз при изменении regValue;

* в начале каждого полного периода сетевого напряжения делаем: 

regError = regError - regValue;
if (regError < 0) {
regError = regError + 99; // максимальное значение 
_симистор_открыть_; // на весь период
} else
_симистор_не_открывать_; // весь период
    */
}

//---------------------------//основной цикл
void loop()
{
  ProcessTimers(); //прибавление таймеров
  if (previous_state != current_state)
    entry = 1;
  else
    entry = 0;           // если предыдущее состояние отличается от текущего-работаем по первому вхождению
 
  switch (current_state) //<><><><><><><> ------------бегаем по состояниям
  {
  //-----------------------------------------------сон
  case kGetInput:
    if (entry) // первое вхождение
    {
      previous_state = current_state;
    }

    // Определяем температуру от датчика DS18b20
    byte data[2];       // Место для значения температуры
    switch (temp_state) // работаем с датчиком температуры
    {
    case kSendRecuest:          // отправляем запрос подготовить значение температуры
    //start_time=micros();//**
      ds.reset();               // Начинаем взаимодействие со сброса всех предыдущих команд и параметров
            //   time_work=micros()-start_time;//**
       // Serial.println(time_work);//**
      ds.write(0xCC);           // Даем датчику DS18b20 команду пропустить поиск по адресу. В нашем случае только одно устрйоство
      ds.write(0x44);           // Даем датчику DS18b20 команду измерить температуру. Само значение температуры мы еще не получаем - датчик его положит во внутреннюю память

      temp_state = kGetAnsver;  // переходим в состояние получить ответ
      StartTimer(TEMPER_TIMER); //запускаем таймер
      break;
    case kGetAnsver:                                   // получаем значение температуры
      if (timers[TEMPER_TIMER] > time_interval_temper) //если прошло время на подготовку ответа
      {
        StopTimer(TEMPER_TIMER);
        ds.reset();
        ds.write(0xCC);
        ds.write(0xBE);      // Просим передать нам значение регистров со значением температуры
        data[0] = ds.read(); // Читаем младший байт значения температуры
        data[1] = ds.read(); // А теперь старший
        // Формируем итоговое значение:
        //    - сперва "склеиваем" значение,
        //    - затем умножаем его на коэффициент, соответсвующий разрешающей способности (для 12 бит по умолчанию - это 0,0625)
        temperature = ((data[1] << 8) | data[0]) * 0.0625;
        // Выводим полученное значение температуры в монитор порта
        
        Serial.println(temperature);

        temp_state = kSendRecuest; // переходим в состояние отправки запроса температуры
      }

      break;
    }

    // опрос переменника для для шим
    pwm=analogRead(6); 
    setpoint=map(pwm, 0, 1023, 0, 100);
    //setpoint=50;
    //опрос кнопки логгирования
    log_on = digitalRead(BUTTON_LOG);
    //опрос кнопки старт
    if (digitalRead(BUTTON_START))
    {
      if (is_work)
      {
        current_state = kMode; // переходим в состояние-режима работы
      }
      else
      {
        current_state = kStart; // переходим в состояние-запуска работы
        is_work = true;
      }
    }
    else
    {
      if (is_work)
      {
        current_state = kStop; // переходим в состояние-остановки работы работы
        is_work = false;
      }
    }
        
    break;
  //------------------------------------------------начинаем работать, создаем лог файл
  case kStart:
    if (entry)
    { // первое вхождение
      previous_state = current_state;
      servoPulse(servoPin, 90);
    }
    
    if (log_on) //если логгирование включено
    {
      const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1; // размер под общее имя лога
      char fileName[13] = FILE_BASE_NAME "00.csv";

      // Инициализировать на самой высокой скорости, поддерживаемой платой, которая
      // не более 50 МГц. Попробуйте снизить скорость, если возникают ошибки SPI.
      if (!sd.begin(chipSelect, SD_SCK_MHZ(16)))
      {
        sd.initErrorHalt();
      }

      // Поиск незанятого имени файла.
      if (BASE_NAME_SIZE > 6)
      {
        error("Long filename");
      }
      while (sd.exists(fileName))
      { //пока имя файла уже существует
        if (fileName[BASE_NAME_SIZE + 1] != '9')
        {                                 //если второе число номера не максимум
          fileName[BASE_NAME_SIZE + 1]++; //увеличиваем его
        }
        else if (fileName[BASE_NAME_SIZE] != '9')
        {                                     //иначе если первое число номера не максимум
          fileName[BASE_NAME_SIZE + 1] = '0'; // обнуляем второе число номера(от 09 переходим к 10)
          fileName[BASE_NAME_SIZE]++;         // прибавляем первое число
        }
        else
        {
          error("Can't create file name");
          current_state = kBlocking; //уходим в блокировку
        }
      }
      if (!file.open(fileName, O_WRONLY | O_CREAT | O_EXCL))
      {
        error("file.open");
        current_state = kBlocking; //уходим в блокировку
      }
      // записываем заголовок файла.
      writeHeader();
      // записывать с кратного интервалу выборки.
      /////////////////////logTime = millis()/(1000UL*SAMPLE_INTERVAL_MS) + 1;!!!!!!!!!!!!!!!!!!!!!!!!!
      //logTime *= 1000UL*SAMPLE_INTERVAL_MS;
    }
    digitalWrite(RELAY, HIGH);    // включить реле
    current_state = kMode; //уходим в работу по режиму
    time_before_start=millis(); // запомнить время от которого пойдет отсчет в логе
    break;
  //------------------------------------------------работа по режиму
  case kMode:     // пид или ручник
    if (entry)
    { // первое вхождение
      previous_state = current_state;
     // Serial.println(current_state);
    }
    StartTimer(TRIAC_TIMER);
    current_state = kPwm; //уходим в шим
    break;
  //------------------------------------------------вывод на шим
  case kPwm:
    if (entry)
    { // первое вхождение
      previous_state = current_state;
     // Serial.println(current_state);
    }
    if (timers[TRIAC_TIMER] > time_interval_triac) //очередной расчет для симистора
      {
        ResetTimer(TRIAC_TIMER);
        brasenham();
      }
    //-analogWrite(PWM_OUT, pwm / 4); //вывод на шим
    if (log_on)  { current_state = kDataLog; }//уходим в логгирование
    else  { current_state = kGetInput; } //уходим в опрос входов
    break;
  //------------------------------------------------запись лога
  case kDataLog:
    if (entry)
    { // первое вхождение
      previous_state = current_state;
     // Serial.println(current_state);
    }
    StartTimer(LOGGING_TIMER);                     //запускаем таймер записи новой строчки
    if (timers[LOGGING_TIMER] > time_interval_log) //если прошло время на подготовку ответа
    {
     long log_time=(process_timer_2-time_before_start)/1000000UL;
     //long log_time=(process_timer_2-time_before_start);
      file.print(log_time);
      file.write(',');
      file.print(timers[LOGGING_TIMER]);
      //file.print(pwm);
      file.write(',');
      file.print(temperature);
      file.println();
     ///1 Serial.println("Log");
     StopTimer(LOGGING_TIMER);
    }
    current_state = kGetInput; //уходим в опрос входов
    break;
    //-------------------------------------------------остановкара боты
  case kStop:
    if (entry)
    { // первое вхождение
      //Serial.println(current_state);
    }
    if (log_on)
    {
         file.close();   // закрываем файл лога
    }
    StopTimer(TRIAC_TIMER);
    TRIAC_OFF;
    //-analogWrite(PWM_OUT, 0); //откл шим
    servoPulse(servoPin, 135);
    digitalWrite(RELAY, LOW);    // выключает реле
   ///1 Serial.println(F("Done"));
    current_state = kGetInput; //уходим в опрос входов
    //-------------------------------------------------блокировка
    break;
  case kBlocking:
    if (entry)
    { // первое вхождение
   ///1   Serial.println(current_state);
    }
    digitalWrite(RELAY, LOW);    // выключает реле
     TRIAC_OFF;
    break;
  default:
    break;
  }

}