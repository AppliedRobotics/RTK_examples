#include <DynamixelWorkbench.h>//подключение библиотеки
#define DEVICE_NAME "3" //номер физической шины для Open CM
#define BAUDRATE 1000000//скорость передачи данных с сервоприводами
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
      uint8_t dxl_id = i;
      result = dxl_wb.jointMode(i, 0, 0, &log);//подключение режима joint
      if (result == false)//подключение не прошло
      {
        Serial.println(log);
        Serial.println("Failed to change joint mode");
      }
      else//подключение прошло
      {
        Serial.println("Succeed to change joint mode");
      }
    }
  } 
}

void loop()
{
}