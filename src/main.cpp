
// ! внимание мать
    //? как так
    // TODO 
// TODO проверка разрешающей способности датчика температуры. по умолчанию 12 бит. если уменьшать, то необходимо считывать этот параметр и при необходимости записывать в датчик при каждом включении
// TODO оставить в датчике температуры только целую часть и уставки пересчитать с учетом коэффициента-долой плавающие точки на 8 битах
// TODO 1 лог если сначала небл включен то и файл не создается, если потом включить то некуда писать строчик-ошибка
// TODO 2 разобраться с 85.00 от датчика температуры-баг который может заменить режим
// TODO 3 управление реле - обращаться на с++ напрямую
#include <Arduino.h>
#include "avr/wdt.h" // подключаем watch dog таймер
#include <OneWire.h>
#include <SPI.h>
#include "SdFat.h"

enum StateWork //перечисление рабочих состояний
{
  kSleep,   //сон или ожидание
  kStart,    //начало работы
  //kMode,     //режим   //! пока исключен за ненадобностью
  kMainHeat, //главный нагрев
  kMaintenanceTemp, //поддержание температуры
  //kPwm,      //шим //! пока исключен за ненадобностью
  kWait,     //ожидание
  kStop,     //окончание работы
  kBlocking  //блокировка
};
StateWork current_state;  // переменная рабочих состояний
StateWork previous_state; // переменная предыдущего состояния
byte entry;               //флаг=1 при входе автомата в новое состояние
//--------------------------------перечисление событий блокировки
enum StateBlock
{
  blockCrc, //несовпадает CRC код датчика температуры
  blockDs18b20, //датчик температуры не той системы
  blockSdInit, //не могу достучаться до sd карты
  blockLogCreate, // не могу создать файл на сд карте
  blockLogOpen,   // не могу открыть файл на sd карте
  blockHeatMax, // перегрев
  blockNone   // нет блокировки
};
StateBlock block_event;

//--------------------------------переменные для таймеров
#define MAX_TIMERS 4    //количество таймеров
#define TEMPER_TIMER 0  //таймер опроса датчика температуры
#define LOGGING_TIMER 1 //таймер записи данных на карту
#define TRIAC_TIMER 2 //таймер выдачи импульса симистору
#define BLOCK_TIMER 3 //таймер выдачи импульса симистору
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
#define time_interval_block 50  // интервал проверки блокировки
long time_before_start = 0;     //обнуление время с которого начался режим работы

// назначение кнопок
#define BUTTON_START 2 //назначение пина кнопки старт //! (PIND&(1 << PD2) ==1 если нажата
#define BUTTON_LOG 5   //назначение пина кнопки логгирования //! (PIND&(1 << PD3) ==1 если нажата!! проверить возможно pd5
bool is_work = false;  // перешли в режим работы
bool log_on = false;   //логгирование вклчено
bool but_start=false;       // кнопка старт нажата
// ШИМ
#define PWM_OUT 3 //назначение пина шим 
int pwm;              // вход переменного резистора
//----------------------------- Главный нагреватель
#define MAIN_HEATER 6   //назначение пина реле главного нагревателя
#define MAIN_HEATER_ON (PORTD |= (1<<PD6)); // на си включение реле нагрева, 
#define MAIN_HEATER_OFF (PORTD &= ~(1<<PD6)); //отключение реле нагрева

//----------------------------- сервопривод
int out_servo=0; // установка положения сервопривода в градусах
int servoPin = 9;            // порт подключения сервы
int pulseWidth;              // длительность импульса
//----------------------------- управление симистором
int setpoint_support=100;	// уставка поддержания температуры
int setpoint_heat=98; // уставка температуры основного нагрева 
int setpoint_cooler=0x30; //уставка охладителя
int modulator=0;	// рабочая переменная
int er=50;        // ошибка
int dataPID=0;
#define RELAY 4   //назначение пина реле защиты симистра
#define TRIAC_RELAY_ON (PORTD |= (1<<PD4)); // на си включение реле защиты симистра, 
#define TRIAC_RELAY_OFF (PORTD &= ~(1<<PD4)); //отключение реле защиты симистра
#define TRIAC_ON (PORTC |= (1<<PC0)); // на си включение тиристора, 
#define TRIAC_OFF (PORTC &= ~(1<<PC0)); //отключение тиристора
//-------------------------------переменные датчика температуры
OneWire sensor_water(8);         // Создаем объект OneWire для шины 1-Wire, с помощью которого будет осуществляться работа с датчиком
OneWire sensor_vapor(7);         // датчик пара
enum TempCommunication //состояния общения с датчиком
{
  kSendRecuest, //передать запрос
  kGetAnsver    //получить ответ
};
TempCommunication temp_state; //переменаая состояния датчика температуры
float temperature_water;            //  температура воды
float temperature_vapor;            // температура пара
byte  temperature_error_vapor = 0;           //  счетчик количества ошибок в температуре
byte  temperature_error_water = 0;           //
//------------------------------- переменные SD карты
#define FILE_BASE_NAME "Data"           // общее название логов
const uint8_t chipSelect = SS;          // пин выбора чипа SD.  !!!!!Отключить все остальное с шины SPI!!!!!.
SdFat sd;                               // создаем объект SD карты.
SdFile file;                            // Создаем объект лога.
#define error(msg) sd.errorHalt(F(msg)) //Сообщения об ошибках, хранящиеся во флэш-памяти.

StateBlock TempControlData (OneWire& ds) 
{
  byte temp_code[8]; //массив информации датчика температуры
  ds.reset();
  ds.write(0x33); //считать код подчненного устройства
  for ( byte i = 0; i < 8; i++) {           // нам нужно 8 байтов
    temp_code[i] = ds.read();
  }
  if ( ds.crc8( temp_code, 7) != temp_code[7] || temp_code[0]==0) return  blockCrc;  
  if ( (temp_code[0]=!0x28))  return  blockDs18b20; 
  return blockNone;
}
void TempSendRecuest (OneWire& ds)
{
    ds.reset();               // Начинаем взаимодействие со сброса всех предыдущих команд и параметров
    ds.write(0xCC);           // обращение ко всем устройствам на шине
    ds.write(0x44);           // Даем датчику DS18b20 команду измерить температуру. Само значение температуры мы еще не получаем - датчик его положит во внутреннюю память
}

byte  TempGetAnsver (OneWire& ds, float &temperature, byte error)
{
  byte data[9];       // Место для значения температуры
  ds.reset();
  ds.write(0xCC);
  ds.write(0xBE);      // Просим передать нам значение регистров со значением температуры
  for ( byte i = 0; i < 9; i++) {           // нам нужно 9 байтов
    data[i] = ds.read();
  }
  if ( OneWire::crc8( data, 8) != data[8] || data[7]==0)  // контроль CRC и если примем нули
  {
    error +=1; 
    return error;
  }
  error=0;
  // Формируем итоговое значение:
  //    - сперва "склеиваем" значение,
  //    - затем умножаем его на коэффициент, соответсвующий разрешающей способности (для 12 бит по умолчанию - это 0,0625)
  temperature = ((data[1] << 8) | data[0]) * 0.0625;
  return error;
}
void setup()
{
  noInterrupts();//отключить прерывания
  //си настройка АЦП
  // |= -записать единицу, не стирая другие
  // &=~ -записать ноль, в бит n
  ADCSRA |= (1 << ADPS2);                     //Биту ADPS2 присваиваем единицу - коэффициент деления 16
  ADCSRA &= ~ ((1 << ADPS1) | (1 << ADPS0));  //Битам ADPS1 и ADPS0 присваиваем нули

  // си настройка таймера 2 на шим с фазовой коррекцией вывод на порт B (нога 3 ардуино) без предделител-частота 31.4 кГц
  TCCR2B |= (1<<CS20);  // делитель на 0
  TCCR2B &= ~ ((1 << CS21) | (1 << CS22)); //делитель на 0
  TCCR2A |= (1<<COM2B1) | (1<<WGM20); // задействуем выход B TCNT2 это PD3, 3 нога ардуины / шим с коррекцией фазы 
  DDRD |=(1<<DDD3); // PD3 на выход
  OCR2B= 0; //начальное значение на шим
  
  DDRC |= (1 << DDC0);// выставляем А0 как цифровой выход
  TRIAC_OFF;
  interrupts();//включить прерывания

  pinMode(BUTTON_START, INPUT);
  pinMode(BUTTON_LOG, INPUT);
  pinMode(RELAY, OUTPUT);  
  digitalWrite(RELAY, LOW);    // выключает реле
  pinMode(MAIN_HEATER, OUTPUT);
  digitalWrite(MAIN_HEATER, LOW);    // выключает реле
  pinMode(servoPin, OUTPUT);          // пин сервы, как выход

  Serial.begin(9600);
  current_state = kSleep; // первое состояние- сон
  temp_state = kSendRecuest;
  //TODO бумкнуть задержку
  block_event= TempControlData(sensor_water);// проверка подключенного датчика температуры воды
  block_event= TempControlData(sensor_vapor);// проверка подключенного датчика температуры пара
  if (block_event != blockNone) {current_state=kBlocking;}// 
}

int computePID(float input, float sp_support, float kp, float ki, float kd, float dt, int minOut, int maxOut) {
  float err = sp_support - input;
  static float integral = 0, prevErr = 0;
  integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
  float D = (err - prevErr) / dt;
  prevErr = err;
  return constrain(err * kp + integral + D * kd, minOut, maxOut);
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
  file.print(F(",water"));
  file.print(F(",heater"));
  file.print(F(",vapor"));
  file.print(F(",cooler"));
  file.println();
}

void servoPulse(int servoPin, int myAngle) 
{//поворот сервоприводом
  pulseWidth = (myAngle * 11) + 500;  // конвертируем угол в микросекунды
  digitalWrite(servoPin, HIGH);       // устанавливаем серве высокий уровень
  delayMicroseconds(pulseWidth);      // ждём
  digitalWrite(servoPin, LOW);        // устанавливаем низкий уровень
}

void brasenham(int data){
    modulator = data + er; 
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
    
    //* 
}
void ResetWork (void)   //функция сброса переменных выходов и таймеров для остановки и блокировки
{
  is_work=false;
  temp_state = kSendRecuest;
  StopTimer(TRIAC_TIMER);
  StopTimer(TEMPER_TIMER);
  StopTimer(LOGGING_TIMER);
  TRIAC_OFF;
  MAIN_HEATER_OFF;// отключаем нагрев
  servoPulse(servoPin, 135);
  TRIAC_RELAY_OFF;   // выключает реле защиты
  OCR2B=0;//отключаем шим
  ///1 Serial.println(F("Done"));
}

void DataLog (void) //TODO передавать все переменные
{
  StartTimer(LOGGING_TIMER);                     //запускаем таймер записи новой строчки
  if (timers[LOGGING_TIMER] > time_interval_log) //если прошло время на подготовку ответа
  {
    //long log_time=(process_timer_2-time_before_start)/1000000UL;
    long log_time=(process_timer_2-time_before_start)/1000UL;
    file.print(log_time);
    file.write(',');
    file.print(temperature_water);
    file.write(',');
    file.print(dataPID);
    file.write(',');
    file.print(temperature_vapor);
    file.write(',');
    file.print(pwm);
    file.println();

    StopTimer(LOGGING_TIMER);
  }
}
// производит опрос входов
//возвращает 0 при успешном выполнени, 1 при ошибке
bool  GetInput(void)      
{
  switch (temp_state) // работаем с датчиком температуры
  {
  case kSendRecuest:          // отправляем запрос подготовить значение температуры
    TempSendRecuest (sensor_vapor);
    TempSendRecuest (sensor_water);
    temp_state = kGetAnsver;  // переходим в состояние получить ответ
    
    StartTimer(TEMPER_TIMER); //запускаем таймер
    break;
  case kGetAnsver:                                   // получаем значение температуры
    if (timers[TEMPER_TIMER] > time_interval_temper) //если прошло время на подготовку ответа
    {
      StopTimer(TEMPER_TIMER);
      if (TempGetAnsver(sensor_vapor, temperature_vapor,temperature_error_vapor) == 3)  block_event =blockCrc; //если ошибка возвращается 3 раза подряд, то в блок
      if (TempGetAnsver(sensor_water, temperature_water,temperature_error_water) == 3)  block_event =blockCrc; //если ошибка возвращается 3 раза подряд, то в блок
      temp_state = kSendRecuest; // переходим в состояние отправки запроса температуры
      // Выводим полученное значение температуры в монитор порта
      Serial.println(temperature_water);
      Serial.println(temperature_vapor);
      OCR2B = setpoint_cooler; //!!!вывод на шим
      //---------------расчет пид регулятора
      //dataPID=computePID (temperature_water,setpoint_support,0.89,0.37,0.37,1,0,100);
      //dataPID=computePID (temperature_water,setpoint_support,2.595,0.0002036,1.297,1,0,100); // для маленького тэна на панасонике
      //!!!!!!!!!!!!!!!!!!!!dataPID=computePID (temperature_water,setpoint_support,6.656,0.001201,943.8,1,0,100); // для маленького тэна на панасонике 2
      dataPID=computePID (temperature_water,setpoint_support,35,0.1,1,1,0,100); // для маленького тэна на панасонике вручную правлю D/i/p  !!!Оставляем
    }
    break;
  }
  
  // опрос переменника для для шим
  // TODO медианный фильтр 
  pwm=analogRead(6); 
  setpoint_cooler=map(pwm, 0, 1023, 10, 255); //масштабирование переменника в диапазон шим кулера
  log_on = digitalRead(BUTTON_LOG);        //опрос кнопки логгирования // TODO
  but_start = digitalRead(BUTTON_START);    //опрос кнопки старт // TODO
  //? если опрос температуры выкинуть вниз то условие выхода по ошибке можно упростить убрав строку ниже
  if (block_event ==blockCrc) return true; //если по датчику температуры получили блокировку, то дальше не идем
  return false;// выходим без ошибок
}

// создает файл лога
//возвращает 0 если файл создан успешно, 1 если произошла ошибка
bool CreateLogFile (void)
{
  const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1; // размер под общее имя лога
  char fileName[13] = FILE_BASE_NAME "00.csv";
  // Инициализировать на самой высокой скорости, поддерживаемой платой, которая
  // не более 50 МГц. Попробуйте снизить скорость, если возникают ошибки SPI.
  if (!sd.begin(chipSelect, SD_SCK_MHZ(16)))
  {
    //sd.initErrorHalt(); //какие то внутренние проверки, которые нам в пень пока не впились
    block_event=blockSdInit; //карта не определена
    return true;
  }
  // Поиск незанятого имени файла. //! исключил - имя для нас константа
 /* if (BASE_NAME_SIZE > 6)
  {
    Serial.print("Long filename\n");    
  }
  */
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
      block_event=blockLogCreate;
      return true;
    }
  }
  if (!file.open(fileName, O_WRONLY | O_CREAT | O_EXCL))
  {
    block_event=blockLogOpen;
    return true;
  }
  // записываем заголовок файла.
  writeHeader();
  // TODO какую то константу добавить что файл открыт норм
  // записывать с кратного интервалу выборки.
  /////////////////////logTime = millis()/(1000UL*SAMPLE_INTERVAL_MS) + 1;!!!!!!!!!!!!!!!!!!!!!!!!!
  //logTime *= 1000UL*SAMPLE_INTERVAL_MS;
  return false;
}

//---------------------------//основной цикл
void loop()
{
  ProcessTimers(); //прибавление таймеров
  if (GetInput()) current_state = kBlocking; // опрос входов если вернул истину то переходим в состояние блокировки
  if (previous_state != current_state)
    entry = 1;
  else
    entry = 0;           // если предыдущее состояние отличается от текущего-работаем по первому вхождению
  switch (current_state) //<><><><><><><> ------------бегаем по состояниям
  {
  //-----------------------------------------------сон
  case kSleep:
      if (entry)
      { // первое вхождение
        previous_state = current_state;
        Serial.print("Sleep\n");
      }
      if (but_start) current_state = kStart; // переходим в состояние-запуска работы
  break;
  //------------------------------------------------начинаем работать, создаем лог файл
  case kStart:
    if (entry)
    { // первое вхождение
      previous_state = current_state;
      Serial.print("Start\n");
      servoPulse(servoPin, 90);
    }
    if (log_on) //если логгирование включено
    {
      if (CreateLogFile()) current_state = kBlocking; //если файл лога не создан, уходим по ошибке
    }
    time_before_start=millis();   // запомнить время от которого пойдет отсчет в логе
    current_state = kMainHeat;        //уходим в первичный нагрев
    
    break;

  //------------------------------------------------первичный нагрев
  case kMainHeat:
  // TODO алгоритм: сначала включаем реле, если температура больше уставки, то выключаем реле и уходим в поддержание температуры
  if (entry)
    { // первое вхождение
      previous_state = current_state;
      Serial.print("Heat");
      if (digitalRead(MAIN_HEATER) == 0)    MAIN_HEATER_ON; /// если нагреватель выключен-включаем //? нужна ли проверка?
    }
  if (temperature_water>setpoint_heat) //если температура выше уставки
  {
    MAIN_HEATER_OFF;// отключаем нагрев
    current_state = kMaintenanceTemp; //уходим в поддержание температуры
    break;
  }
   // TODO если во время нагрева прирост температуры отрицательный , то блокировка по неисправности тэна  
  if (log_on)  
  {
   //! if (file) DataLog();    //пишем лог ! проверка по константе создан ли файл
  
  } 
  if (but_start== false)  current_state = kStop; //уходим в остановку
  break;
  //------------------------------------------------поддержание температуры
  case kMaintenanceTemp:
  // TODO алгоритм: либо держим температуру по пид либо крутилкой вручную
  //? необходимо разобраться, возможно ли что поддержание небудет справляться и необходимо сваливаться в нагрев?
  if (entry)
    { // первое вхождение
      previous_state = current_state;
      Serial.print("Maintenance");
      TRIAC_RELAY_ON;    // включить реле защиты симистора
      StartTimer(TRIAC_TIMER);      // запуск таймера симстора
    }
  if (timers[TRIAC_TIMER] > time_interval_triac) //очередной расчет для симистора
  {
    ResetTimer(TRIAC_TIMER);
    brasenham(dataPID);        // поддержание температуры через пид
    //!brasenham(setpoint_support); // вывод на симистор уставки с крутилки
  }
  
  if (temperature_water>110) block_event=blockHeatMax;   // перегрев уходим в блокировку
  if (temperature_water<90) current_state=kMainHeat;    // если недогрев то уходим на главный подогрев
  //-analogWrite(PWM_OUT, pwm / 4);               //вывод на шим
  if (log_on)  DataLog();                         //уходим в логгирование
  if (block_event!=blockNone) current_state=kBlocking;
  if (but_start == false) current_state=kStop;
  break;

  case kStop:
    if (entry)// TODO засунуть в первое вхождение это состояние без многозаходности?
    { // первое вхождение
      previous_state = current_state;
      //Serial.println("Stop");
      ResetWork ();   //сбрасываем выходы и таймеры
      if (log_on)  file.close();   // закрываем файл лога
    }
    current_state=kSleep; //уходим в состояние сна 
    break;
    //-------------------------------------------------блокировка
  case kBlocking:
    if (entry)
    { // первое вхождение
      previous_state = current_state;
      Serial.print("BLOCK ");
      ResetWork (); 
      
      //если ведется лог, то сделать запись об ошибке 
      if (log_on) {
      file.print(F("e="));
      file.print(block_event);
      file.println();
      file.close(); 
      }
    }
    switch (block_event) // по какой причине блокировка
    {
    case blockCrc:  // блокировка по контрольной сумме датчика температуры 
      // ? помимо этого опроса будет происходить еще и опрос в функции получения входов
      StartTimer(BLOCK_TIMER); // опрос проблемных мест делать с задержкой , пусть пол секунды
      if (timers[BLOCK_TIMER] > time_interval_block)
      {
        StopTimer(BLOCK_TIMER);
        Serial.print("CRC is not valid!\n");  //  CRC не корректен //? уйти в полную блокировку или сбросить программу и восстановить если приблема исчезнет.
        block_event=TempControlData(sensor_water); // повторить проверку контрольной суммы
        if( block_event==blockNone)  block_event=TempControlData(sensor_vapor);// если исправен датчик пара, то проверяем датчик воды
      }
      break;
    case blockDs18b20:
      Serial.print("not DS18B20\n");// тип датчика не 18b20
      break;
    case blockSdInit:
      Serial.print("NO SD card"); //не смог создать файл лога
      break;
    case blockLogCreate:
      Serial.print("Can't create file name"); //не смог создать файл лога
      break;
    case blockLogOpen:
      Serial.print("file.open"); // не смог открыть файл лога
      break;
    case   blockHeatMax: // перегрев
      Serial.print("blockHeatMax\n");    
      break;
    case blockNone:
      current_state = kSleep; //! тут будет не все так просто мать твою! нужно выходить туда откуда пиршли) или нихуя! Скорее нужно начать отрабатывать заново-новый лог, новый нагрев и выход на режим
      break;
    default:
      break;
    }
    break;
  default:
    break;
  }

}

