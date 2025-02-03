#include <DynamixelWorkbench.h>//подключение библиотеки
#define DEVICE_NAME "3" //номер физической шины для Open CM
#define BAUDRATE 1000000//скорость передачи данных сервопривода
DynamixelWorkbench dxl_wb;//создание экземпляра класса DynamixelWorkbench

const uint8_t handler_index_speed = 0;//переменная с индексом обработчика синхронной записи скорости
int32_t goal_speed[5] = {16, 12, 12, 16, 60};//массив скоростей
const uint8_t handler_index_pos = 1;//переменная с индексом обработчика синхронной записи положения
int32_t goal_position[5] = {2048, 2048, 2048, 2048, 512};//массив положений

//функция установки позиции
void Set_Pos(){
 const char *log;
 bool result = false;
 result = dxl_wb.syncWrite(handler_index_pos, goal_position, &log);//синхронная запись положений
 if (result == false)//запись позиций не прошла
 {
 Serial.println(log);
 Serial.println("Failed to set goal_position");
 }
}

void setup() 
{
 Serial.begin(57600);//скорость передачи данных с компьютером
 //while(!Serial); // ожидание открытия монитора порта
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
 for (int i=1; i<=5; i++){
 uint8_t dxl_id = i;
 result = dxl_wb.jointMode(i, 0, 0, &log);//включение режима joint сервоприводов
 if (result == false)//режим joint не включен
 {
 Serial.println(log);
 Serial.println("Failed to change joint mode");
 }
 else//режим joint включен
 {
 Serial.println("Succeed to change joint mode");
 }
 delay(250);
 }
 } 
 result = dxl_wb.addSyncWriteHandler(1, "Moving_Speed", &log);//инициализация обработчика синхронной записи скорости
 if (result == false)//инициализация не прошла
 {
 Serial.println(log);
 Serial.println("Failed to add sync write handler");
 }
 else Serial.println(log);//инициализация прошла
 result = dxl_wb.syncWrite(handler_index_speed, goal_speed, &log);//синхронная запись скоростей
 if (result == false)//запись не прошла
 {
 Serial.println(log);
 Serial.println("Failed to sync goal_speed");
 }
 else Serial.println(log);//запись прошла
 result = dxl_wb.addSyncWriteHandler(1, "Goal_Position", &log);//инициализация обработчика синхронной записи положения
 if (result == false)//инициализация не прошла
 {
 Serial.println(log);
 Serial.println("Failed to add sync write handler");
 }
 else Serial.println(log);//инициализация прошла
 result = dxl_wb.syncWrite(handler_index_pos, goal_position, &log);//синхронная запись положений
 if (result == false)//запись не прошла
 {
 Serial.println(log);
 Serial.println("Failed to set goal_position");
 }
Set_Pos();//функция установки позиции
}

void loop()//функция движения манипулятора в диапазоне +-100 тиков
{
 for (int i=0; i<100; i++){
 goal_position[0] = 2048-i;//пересчет положения 1-го сервопривода
 goal_position[1] = 2048-i;//пересчет положения 2-го сервопривода
 goal_position[2] = 2048-i;//пересчет положения 3-го сервопривода
 goal_position[3] = 2048-i;//пересчет положения 4-го сервопривода
 goal_position[4] = 512-i;//пересчет положения 5-го сервопривода
 Set_Pos();//установка позиции
 delay(20);
 }
 for (int i=0; i<100; i++){
 goal_position[0] = 2048+i;//пересчет положения 1-го сервопривода
 goal_position[1] = 2048+i;//пересчет положения 2-го сервопривода
 goal_position[2] = 2048+i;//пересчет положения 3-го сервопривода
 goal_position[3] = 2048+i;//пересчет положения 4-го сервопривода
 goal_position[4] = 512+i;//пересчет положения 5-го сервопривода
 Set_Pos();//установка позиции
 delay(20);
 }
 is_moving();//функция для определения движения сервоприводов
}

void is_moving()//функция для определения движения сервоприводов
{
 int32_t moving1;
 dxl_wb.itemRead(1, "Moving", &moving1);//считывание движения сервопривода
 delay(10);
 if (moving1)//если сервопривод движется, выводим в монитор порта его id
 {
 Serial.print("Moving");
 Serial.println(1);
 }

 int32_t moving2;
 dxl_wb.itemRead(2, "Moving", &moving2);//считывание движения сервопривода
 delay(10);
 if (moving2)//если сервопривод движется, выводим в монитор порта его id
 {
 Serial.print("Moving");
 Serial.println(2);
 }

 int32_t moving3;
 dxl_wb.itemRead(3, "Moving", &moving3);//считывание движения сервопривода
 delay(10);
 if (moving3)//если сервопривод движется, выводим в монитор порта его id
 {
 Serial.print("Moving");
 Serial.println(3);
 }

  int32_t moving4;
 dxl_wb.itemRead(4, "Moving", &moving4);//считывание движения сервопривода
 delay(10);
 if (moving4)//если сервопривод движется, выводим в монитор порта его id
 {
 Serial.print("Moving");
 Serial.println(4);
 }

  int32_t moving5;
 dxl_wb.itemRead(5, "Moving", &moving5);//считывание движения сервопривода
 delay(10);
 if (moving5)//если сервопривод движется, выводим в монитор порта его id
 {
 Serial.print("Moving");
 Serial.println(5);
 }
 
}
