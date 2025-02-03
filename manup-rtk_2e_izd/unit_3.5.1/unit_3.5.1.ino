#include <DynamixelWorkbench.h>//подключение библиотеки
#define DEVICE_NAME "3" //номер физической шины для Open CM
#define BAUDRATE_NUM 7 //количество возможных скоростей передачи данных для сервоприводов
DynamixelWorkbench dxl_wb;//создание экземпляра класса DynamixelWorkbench

void setup() 
{
 Serial.begin(57600);//скорость передачи данных с компьютером
 while(!Serial); // Ожидание открытия монитора порта
 const char *log;
 bool result = false;
 uint8_t scanned_id[100];//массив id
 uint8_t dxl_cnt = 0;
 uint32_t baudrate[BAUDRATE_NUM] = {9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000};// массив скоростей
 uint8_t range = 253;
 uint8_t index = 0;
 // в цикле проходим по всем указанным скоростям 
 while (index < BAUDRATE_NUM)
 {
  result = dxl_wb.init(DEVICE_NAME, baudrate[index], &log);//инициализация сервоприводов
  if (result == false)//инициализация не прошла
  {
    Serial.println(log);
    Serial.println("Failed to init");
  }
  else//инициализация прошла
  {
    Serial.print("Succeed to init : ");
    Serial.println(baudrate[index]); 
  }
  dxl_cnt = 0;
  //в цикле проходим по всем указанным id
  for (uint8_t num = 0; num < 100; num++) scanned_id[num] = 0;
  result = dxl_wb.scan(scanned_id, &dxl_cnt, range, &log);//сканирование сервоприводов
  if (result == false)//сканирование не прошло
  {
    Serial.println(log);
    Serial.println("Failed to scan");
  }
  else//сканирование прошло
  {
    Serial.print("Find");
    Serial.print(dxl_cnt);
    Serial.println("Dynamixels");
    //в цикле проходим по всем сервоприводам
    for (int cnt = 0; cnt < dxl_cnt; cnt++)
    {
      Serial.print("id : ");
      Serial.print(scanned_id[cnt]);//указание id
      Serial.print("model name : ");
      Serial.println(dxl_wb.getModelName(scanned_id[cnt]));//указание модели сервопривода
    }
  } 
  index++;
 }
}

void loop() 
{
}