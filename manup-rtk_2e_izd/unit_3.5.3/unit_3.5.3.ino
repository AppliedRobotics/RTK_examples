#include <DynamixelWorkbench.h>//подключение библиотеки
#define DEVICE_NAME "3" //номер физической шины для Open CM
#define BAUDRATE 1000000//скорость передачи данных сервоприводов
DynamixelWorkbench dxl_wb;//создание экземпляра класса DynamixelWorkbench

void setup() 
{
 Serial.begin(57600);//скорость передачи данных с компьютером
 while(!Serial); // дождитесь открытия последовательного монитора
 const char *log = NULL;
 bool result = false;
 uint8_t scanned_id[16];//массив id
 uint8_t dxl_cnt = 0;
 uint8_t range = 100;
 result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);//инициализация сервоприводов
 if (result == false)//инициализация не прошла
  {
    Serial.println(log);
    Serial.println("Failed to init");
  }
 else//инициализация прошла
  {
    Serial.print("Succeeded to init : ");
    Serial.println(BAUDRATE); 
  }
 Serial.println("Wait for scan...");
 result = dxl_wb.scan(scanned_id, &dxl_cnt, range, &log);//сканирование сервоприводов
 if (result == false)//сканирование не прошло
  {
    Serial.println(log);
    Serial.println("Failed to scan");
  }
 else//сканирование прошло
  {
    Serial.print("Find ");
    Serial.print(dxl_cnt);
    Serial.println(" Dynamixels");
    //в цикле проходим по 5 сервоприводам
    for (int i=1; i<=5; i++)
    {
      const uint8_t handler_index_speed = 0;//переменная с индексом обработчика синхронной записи скорости
      int32_t goal_speed[5] = {16, 12, 12, 16, 60};//массив скоростей
      result = dxl_wb.addSyncWriteHandler(1, "Moving_Speed", &log);//инициализация обработчика синхронной записи скорости
      if (result == false)//инициализация не прошла
      {
        Serial.println(log);
        Serial.println("Failed to add sync write handler");
      }
      else Serial.println(log);//инициализаця прошла
      result = dxl_wb.syncWrite(handler_index_speed, goal_speed, &log);//синхронная запись скоростей
      if (result == false)//запись не прошла
      {
        Serial.println(log);
        Serial.println("Failed to sync goal_speed");
      }
      else Serial.println(log);//запись прошла
    }
  } 
}

void loop()
{
}