#include <DynamixelWorkbench.h> // подключение библиотеки
#define BAUDRATE 1000000 // скорость передачи данных для сервоприводов
#define DEVICE_NAME "3" // подключение OpenCM
// инициализация ID сервоприводов и модуля TrackingCam
#define CAM_ID 51
#define DXL_ID1 1
#define DXL_ID2 2
#define DXL_ID3 3
#define DXL_ID4 4
#define jointN 4 // инициализация ID сервоприводов
// инициализация пинов для работы с пневмосистемой
#define vacuum_pump 8//пин, к которому подключен насос
#define vacuum_key 9//пин, к которому подключен пневмораспределитель
#define DEBUG_SERIAL Serial // последовательный порт, подключаемый к компьютеру
// объявление параметров и переменных
DynamixelWorkbench dxl_wb;
int val = 0;
//массивы положений
int32_t new_alphas[4];
int32_t old_alphas[4];
int32_t diff_alphas[4];
int max_el = 0;//переменная для записи максимального пути
int vel[5];//массив скоростей
float acc[5];//массив ускорений
uint8_t dxl_id[5] = {DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4};//массив id
const uint8_t handler_index = 0;//переменная с индексом обработчика синхронной записи положения
// переменные для решения ОЗК
float pi = 3.14159265; // задание константы Пи
double rad2ticks = 2*pi / 4096; // перевод из радиан (0...360 градусов) в условные угловые единицы (0...16383)
float degree_coeff = 4096 / 360; // количество условных угловых единиц на 1 градус поворота
bool input_coords = true; // флаг для печати строки ввода координат в последовательный порт
int start_pos[jointN+1]; // инициализация одномерного массива для положений, размер его задается 5+1=6,
int i = 0; // счетчик цикла
int alphas[jointN+1]; // массив целевых угловых позиций 1-5 сервоприводов (в условных единицах)
float X; // желаемое положение схвата по оси X (в метрах)
float Y; // желаемое положение схвата по оси Y (в метрах)
float Z; // желаемое положение схвата по оси Z (в метрах)
float x; // положение по оси X относительно 2 и 3 звеньев (в метрах)
float y; // положение по оси Y относительно 2 и 3 звеньев (в метрах)
float z; // положение по оси Z относительно 2 и 3 звеньев (в метрах)
float xf; // расчетное положение по оси X относительно 2 и 3 звеньев (в метрах)
float yf; // расчетное положение по оси Y относительно 2 и 3 звеньев (в метрах)
float zf; // расчетное положение по оси Z относительно 2 и 3 звеньев (в метрах)
float d; // вспомогательная расчетная величина - расстояние от основания манипулятора до схвата
float betta_d; // вспомогательная расчетная величина - угол между осью X и d
float gamma_d; // вспомогательная расчетная величина - угол между звеном 2 и d
float alpha1; // целевая угловая позиция 1 сервопривода (в радианах)
float alpha2; // целевая угловая позиция 2 сервопривода (в радианах)
float alpha3; // целевая угловая позиция 3 сервопривода (в радианах)
float alpha4; // целевая угловая позиция 4 сервопривода (в радианах)
float alpha5; // целевая угловая позиция 5 сервопривода (в радианах)
// параметры звеньев манипулятора
float l1 = 0.124;
float l2 = 0.02004;
float l3 = 0.15;
float l4 = 0.15;
float l5 = 0.0399;
float l6 = 0.1345;
// параметры модуля TrackingCam
struct TrackingCamBlobInfo_t
{
 uint8_t type;
 uint8_t dummy;
 uint16_t cx;
 uint16_t cy;
 uint32_t area;
 uint16_t left;
 uint16_t right;
 uint16_t top;
 uint16_t bottom;
};
int cx;
int type;
int cy;
// координаты шариков на паллете в системе координат манипулятора
float coord_balls[3][2] = {
 { 0.185, 0.012},
 { 0.237, 0.012},
 { 0.29, 0.013}
};
// координаты шариков на паллете правой стороны в системе координат манипулятора
float coord_pravo[3][2] = {
 { 0.185, -0.05},
 { 0.237, -0.05},
 { 0.29, -0.047}
};
// координаты шариков на паллете левой стороны в системе координат манипулятора
float coord_levo[3][2] = {
 { 0.185, 0.065},
 { 0.237, 0.067},
 { 0.295, 0.067}
};
// координаты домашней позиции для манипулятора
float X_home = 0.15;
float Y_home = -0.15;
  
void setup() {
 // объявление работы пинов пневмосистеме
pinMode(vacuum_pump, OUTPUT);
pinMode(vacuum_key, OUTPUT);
DEBUG_SERIAL.begin(57600);// установка скорости обмена данными по последовательному порту компьютер
  dxl_wb.init(DEVICE_NAME, BAUDRATE); // инициализация устройств
 // установка режима работы для сервоприводов
 uint16_t model_number = 0;
 for (int cnt = 0; cnt < 4; cnt++)
 {
 dxl_wb.ping(dxl_id[cnt], &model_number);
 dxl_wb.jointMode(dxl_id[cnt], 50, 0);//режим joint
 }
 //задание и переход в стартовую позицию для манипулятора
 start_position();
 IK(0.15, -0.15, 0.1);
 delay(4000);
 // пинг камеры
 dxl_wb.ping(CAM_ID);
}

// разделяет строку по пробелам
void parse_input(String str) {
 int separator = str.indexOf(' '); // ищем первый пробел в строке:
 X = str.substring(0, separator).toInt()/100.0; 
// переводим строку из сантиметров в метры
 int separator2 = str.indexOf(' ', separator+1); // ищем второй пробел
 Y = str.substring(separator+1, separator2).toInt()/100.0;
 Z = str.substring(separator2).toInt()/100.0;
}
// функция задания базовых координат для сервоприводов
void start_position()
{
 for (i=0; i <= jointN; i++)
 {
 start_pos[i] = 2048;
 old_alphas[i] = start_pos[i];
 }
}
// функция по решению ОЗК
void IK(float X1, float Y1, float Z1)
{
 Z1 = correct_Z(X1, Z1, Y1);
 alpha1 = atan(Y1/X1);
 float l = sqrt(X1*X1+Y1*Y1);
 z = Z1 - l1 + l6;
 x = l-l5-l2;
 d = sqrt(x*x + z*z);
 gamma_d = acos((l3*l3 + d*d - l4*l4)/(2*l3*d));
 betta_d = gamma_d+atan(z / x);
 alpha2 = pi/2 - betta_d; // вычисление угла альфа 2
 alpha3 = pi - acos((l3*l3 + l4*l4 - d*d)/(2*l3*l4)); // вычисление угла альфа3
 alpha3 = alpha3 - (pi/2 - alpha2); // пересчет в целевое положение 3 сервопривода (учитывается альфа 2)
 alpha4 = 0; // углы альфа4 и альфа5 принимаются нулевыми. альфа - это угол отклонения следующего звена от предыдущего
 alpha5 = 0;
 alphas[0] = start_pos[0] + int(alpha1/rad2ticks); // пересчет в целевое положение 1 сервопривода
 alphas[1] = start_pos[1] - int(alpha2/rad2ticks); // пересчет в целевое положение 2 сервопривода
 alphas[2] = start_pos[2] + int(alpha3/rad2ticks); // пересчет в целевое положение 3 сервопривода
 alphas[3] = start_pos[3] + int(alpha4/rad2ticks); // пересчет в целевое положение 4 сервопривода
 // цикл для поиска разницы пути между старым и новым положением для каждого сервопривода
 for (int i = 0; i < 4; i++)
 {
 new_alphas[i] = alphas[i];
 diff_alphas[i] = abs(new_alphas[i] - old_alphas[i]);
 }
 // поиск наибольшего пути сервопривода
for (int i = 0; i < 4; i++)
 {
 if(diff_alphas[i] > max_el)
 max_el = diff_alphas[i];
 }
 // присвоение старых координат
 for (int i = 0; i < 4; i++)
 old_alphas[i] = new_alphas[i];
 Handler();
 // обнуление максимального пути
 max_el = 0;
}
// функция запуска синхронного движения сервоприводов
void Handler()
{
 Vel();
 for (int cnt = 0; cnt < 4; cnt++)
 {
 int l = 40;//регистр скорости
 int b = 2;//длина регистра скорости
 uint8_t v = vel[cnt];
 uint8_t a = acc[cnt];
 dxl_wb.writeRegister(dxl_id[cnt], (uint16_t)32, (uint16_t)2, &v); // запись скорости вращения для каждого привода
 dxl_wb.writeRegister(dxl_id[cnt], (uint16_t)73, (uint16_t)1, &a);
 } 
 dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Position");
 dxl_wb.syncWrite(handler_index, dxl_id, 4, new_alphas, 1);
}
// расчет скорости для приводов
void Vel()
{
 float t_maxx = max_el/50;
 float t_R_T = 2*(50/1);
 float T = t_R_T+t_maxx;
 
 for(int i = 0; i < 4; i++)
 {
 vel[i] = (diff_alphas[i])/(t_maxx);
 acc[i] = vel[i]/T/2;
 }
}
// функция по корректировке положения конечного звена по оси Z
float correct_Z(float X2, float Z2, float Y2)
{
 X2 = sqrt(X2*X2+Y2*Y2);
 if (X2 <= 0.171)
 {
 float zr = (0.057*(0.17-X2));
Z2 = Z2 - zr;
 }
 if (X2 > 0.171)
 {
 float zr = (0.057*(X2-0.17));
 Z2 = Z2 + zr;
 }
return Z2;
}
// чтение данных с камеры
void read_TC()
{
 int ax_n = 1;
 int n = 0;
 TrackingCamBlobInfo_t blob[1];
 for (int i = 0; i < ax_n; i++)
 {
 uint32_t resp[16];
 if (!dxl_wb.readRegister(CAM_ID, 16 + i * 16, 16, resp))
 break;
 int idx = 0;
 blob[i].type = resp[idx++];
 type = blob[i].type;
 if (blob[i].type == 0xFF)
 break;
 blob[i].dummy = resp[idx++];
 blob[i].cx = resp[idx] + (resp[idx + 1] << 8);
 idx += 2;
 cx = blob[i].cx;
 blob[i].cy = resp[idx] + (resp[idx + 1] << 8);
 idx += 2;
 cy = blob[i].cy;
 blob[i].area = (resp[idx] + (resp[idx + 1] << 8)) * 4;
 idx += 2;
 blob[i].left = resp[idx] + (resp[idx + 1] << 8);
 idx += 2;
 blob[i].right = resp[idx] + (resp[idx + 1] << 8);
 idx += 2;
 blob[i].top = resp[idx] + (resp[idx + 1] << 8);
 idx += 2;
 blob[i].bottom = resp[idx] + (resp[idx + 1] << 8);
 idx += 2;
 n++;
 }
 Serial.println("n = " + String(n));
 for (int i = 0; i < n; i++)
 {
  Serial.print( String(blob[i].type) + " " + String(blob[i].cx) + " " + String(blob[i].cy) + "\n");
 }
}
// функция по определению цвета шарика и его положения
void detect_ball()
{
 read_TC();
 if(cy < 95 && cy != 0)
 {
 if(type == 0)
 {
 move_manip(1, false);
 }
 else
 {
 move_manip(1, true);
 }
 }
 if((cy >= 95) && (cy < 120))
 {
 if(type == 0)
 {
 move_manip(2, false);
 }
 else
 {
 move_manip(2, true);
 }
 }
 if(cy >= 120)
 {
 if(type == 0)
 {
 move_manip(3, false);
 }
 else
 {
 move_manip(3, true);
 }
 }
 cy = 0;
}
// функция алгоритма движения манипулятора
void move_manip(int N, bool i)//N-nomer yacheyki, i - pravo/levo
{
 if(N == 1) //1-yacheyka
 {
 IK(coord_balls[0][0], coord_balls[0][1], 0.10);//над ячейкой
 delay(3000);
 digitalWrite(vacuum_pump, HIGH);
 digitalWrite(vacuum_key, HIGH);
 IK(coord_balls[0][0], coord_balls[0][1], 0.05);//вплотную к шарику
 delay(1600);
 IK(coord_balls[0][0], coord_balls[0][1], 0.10);//вверх над ячейкой
 Serial.println("1 yacheyka");
 delay(1000);
 if(i)//pravo
 {
 IK(coord_pravo[0][0], coord_pravo[0][1], 0.10);//вправо сверху
 delay(2000);
 IK(coord_pravo[0][0], coord_pravo[0][1], 0.05);//вплотную к ячейке
 delay(1600);
 digitalWrite(vacuum_pump, LOW);
 digitalWrite(vacuum_key, LOW);
 IK(coord_pravo[0][0], coord_pravo[0][1], 0.10);//вверх над ячейкой
 Serial.println("Vpravo");
 delay(1000);
 }
 else//levo
 {
 IK(coord_levo[0][0], coord_levo[0][1], 0.10);//влево сверху
 delay(2000);
 IK(coord_levo[0][0], coord_levo[0][1], 0.05);//вплотную к ячейке
 delay(1600);
 digitalWrite(vacuum_pump, LOW);
 digitalWrite(vacuum_key, LOW);
 IK(coord_levo[0][0], coord_levo[0][1], 0.10);//вверх над ячейкой
 Serial.println("Vlevo");
delay(1000);
 }
 }
 if(N == 2) //2-yacheyka
 {
 IK(coord_balls[1][0], coord_balls[1][1], 0.10);
 delay(3000);
 digitalWrite(vacuum_pump, HIGH);
 digitalWrite(vacuum_key, HIGH);
 IK(coord_balls[1][0], coord_balls[1][1], 0.04);
 delay(1600);
 IK(coord_balls[1][0], coord_balls[1][1], 0.10);
 Serial.println("2 yacheyka");
 delay(1000);
 if(i)//pravo
 {
 IK(coord_pravo[1][0], coord_pravo[1][1], 0.10);
 delay(2000);
 IK(coord_pravo[1][0], coord_pravo[1][1], 0.04);
 delay(1600);
 digitalWrite(vacuum_pump, LOW);
 digitalWrite(vacuum_key, LOW);
 IK(coord_pravo[1][0], coord_pravo[1][1], 0.10);
 Serial.println("Vpravo");
 delay(1000);
 }
 else//levo
 {
 IK(coord_levo[1][0], coord_levo[1][1], 0.10);
 delay(2000);
 IK(coord_levo[1][0], coord_levo[1][1], 0.04);
 delay(1600);
 digitalWrite(vacuum_pump, LOW);
 digitalWrite(vacuum_key, LOW);
 IK(coord_levo[1][0], coord_levo[1][1], 0.10);
 Serial.println("Vlevo");
 delay(1000);
 }
 }
  if(N == 3) //3-yacheyka
 {
 IK(coord_balls[2][0], coord_balls[2][1], 0.10);
 delay(3000);
 digitalWrite(vacuum_pump, HIGH);
 digitalWrite(vacuum_key, HIGH);
 IK(coord_balls[2][0], coord_balls[2][1], 0.04);
 delay(1600);
 IK(coord_balls[2][0], coord_balls[2][1], 0.10);
 Serial.println("3 yacheyka");
 delay(1000);
 if(i) //pravo
 {
 IK(coord_pravo[2][0], coord_pravo[2][1], 0.10);
 delay(2000);
 IK(coord_pravo[2][0], coord_pravo[2][1], 0.04);
 delay(1600);
 digitalWrite(vacuum_pump, LOW);
 digitalWrite(vacuum_key, LOW);
 delay(1000);
 IK(coord_pravo[2][0], coord_pravo[2][1], 0.10);
 Serial.println("Vpravo");
 delay(1000);
 }
 else//levo
 {
 IK(coord_levo[2][0], coord_levo[2][1], 0.10);
 delay(2000);
 IK(coord_levo[2][0], coord_levo[2][1], 0.04);
 delay(1600);
 digitalWrite(vacuum_pump, LOW);
 digitalWrite(vacuum_key, LOW);
 delay(1000);
 IK(coord_levo[2][0], coord_levo[2][1], 0.10);
 Serial.println("Vlevo");
 delay(1000);
 }
 }
 IK(X_home, Y_home, 0.10);
 delay(3000);
}

void loop() {
 detect_ball(); // основная функция сортировки
 delay(100);
 
}