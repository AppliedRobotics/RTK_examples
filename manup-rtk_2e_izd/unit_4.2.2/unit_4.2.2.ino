#include <DynamixelWorkbench.h>// подключение библиотеки
#define BAUDRATE 1000000 // скорость передачи данных для сервоприводов
#define DEVICE_NAME "3" // подключение OpenCM
// Инициализация ID 
#define DXL_ID1 1
#define DXL_ID2 2
#define DXL_ID3 3
#define DXL_ID4 4
#define jointN 4 // количество сервоприводов
#define DEBUG_SERIAL Serial 
// последовательный порт, подключаемый к компьютеру
// объявление параметров и переменных 
DynamixelWorkbench dxl_wb;
int val = 0;
int id_max_el = 0;
int32_t new_alphas[4]; 
int32_t old_alphas[4]; 
int32_t diff_alphas[4]; 
int max_el = 0;
int id_maxel = 0;
float vel[5];
float acc[5];
uint8_t dxl_id[5] = {DXL_ID1, DXL_ID2, DXL_ID3, DXL_ID4}; 
const uint8_t handler_index = 0;
// переменные для решения ОЗК
float pi = 3.14159265; // задание константы Пи
double rad2ticks = 2*pi / 4096; 
// перевод из радиан (0...360 градусов) в условные угловые единицы (0...4096)
float degree_coeff = 4096 / 360; 
// количество условных угловых единиц на 1 градус поворота
bool input_coords = true; 
// флаг для печати строки ввода координат в последовательный порт
int start_pos[jointN+1]; 
// инициализация одномерного массива для по- ложений, размер его задается 4+1=5
int i = 0; // счетчик цикла
int alphas[jointN+1]; 
// массив целевых угловых позиций 4+1 сервоприводов (в условных единицах)
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
float l1 = 0.124; float l2 = 0.02004; float l3 = 0.15; float l4 = 0.15; float l5 = 0.0399; float l6 = 0.1345;

//функция, которая разделяет строку по пробелам 
void parse_input(String str) {
int separator = str.indexOf(' '); // ищем первый пробел в строке: 
X = str.substring(0, separator).toInt()/100.0; 
// переводим строку из сантиметров в метры
int separator2 = str.indexOf(' ', separator+1); // ищем второй пробел 
Y = str.substring(separator+1, separator2).toInt()/100.0;
Z = str.substring(separator2).toInt()/100.0;
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
// функция задания базовых координат для сервоприводов
 void start_position()
{
//проходим по всем сервоприводам
for (i=0; i <= jointN; i++)
{
start_pos[i] = 2048; 
old_alphas[i] = start_pos[i];
}
}
// функция по решению ОЗК 
void IK(float X1, float Y1, float Z1)
{
Z1 = correct_Z(X1, Z1, Y1); alpha1 = atan(Y1/X1);
float l = sqrt(X1*X1+Y1*Y1); z = Z1 - l1 + l6;
x = l-l5-l2;
d = sqrt(x*x + z*z);
gamma_d = acos((l3*l3 + d*d - l4*l4)/(2*l3*d)); 
betta_d = gamma_d+atan(z / x);
alpha2 = pi/2 - betta_d;// вычисление угла альфа 2
alpha3 = pi - acos((l3*l3 + l4*l4 - d*d)/(2*l3*l4));// вычисление угла альфа 3
alpha3 = alpha3 - (pi/2 - alpha2); 
// пересчет в целевое положение 3 сервопривода (учитывается альфа 2)
alpha4 = 0;
// углы альфа4 и альфа5 принимаются нулевыми. альфа - это угол отклонения следующего звена от предыдущего
alpha5 = 0;
alphas[0] = start_pos[0] + int(alpha1/rad2ticks); // пересчет в целевое положение 1 сервопривода
alphas[1] = start_pos[1] - int(alpha2/rad2ticks); // пересчет в целевое положение 2 сервопривода
alphas[2] = start_pos[2] + int(alpha3/rad2ticks); // пересчет в целевое положение 3 сервопривода
alphas[3] = start_pos[3] + int(alpha4/rad2ticks); // пересчет в целевое положение 4 сервопривода
//цикл для поиска разницы пути между старым и новым положением для каждого сервопривода
for (int i = 0; i < 4; i++)
{
new_alphas[i] = alphas[i];
diff_alphas[i] = abs(new_alphas[i] - old_alphas[i]);
Serial.print("diff_alphas:");
Serial.println(diff_alphas[i]);
}
// поиск наибольшего пути сервопривода 
for (int i = 0; i < 4; i++)
{
if(diff_alphas[i] > max_el) {
max_el = diff_alphas[i];
id_maxel = i;
}
}
// присвоение старых координат
 for (int i = 0; i < 4; i++){
 old_alphas[i] = new_alphas[i];}
 Handler();
 max_el = 0; //обнуление максимального пути 
 }
 
 // функция запуска синхронного движения сервоприводов 
 void Handler()
 {
 Vel();//расчет скорости для сервоприводов
 
 for (int cnt = 0; cnt < 4; cnt++)
 {
 uint32_t get_dataV = 0;
 int l = 40;//регистр скорости
 int b = 2;//длина регистра скорости
 uint8_t v = vel[cnt];
 uint8_t a = acc[cnt];
 dxl_wb.writeRegister(dxl_id[cnt], (uint16_t)32, (uint16_t)2, &v); 
 // запись скорости вращения для каждого привода
 dxl_wb.writeRegister(dxl_id[cnt], (uint16_t)73, (uint16_t)1, &a);
 dxl_wb.readRegister(dxl_id[cnt], (uint16_t)l, (uint16_t)b, &get_dataV); 
 // чтение скорости каждого сервопривода 
 Serial.print("vel:");
 Serial.println(get_dataV);
 // выставление параметров ПИД регулятора
 // dxl_wb.itemWrite(dxl_id[cnt], "Position_P_Gain", P_param);
 // dxl_wb.itemWrite(dxl_id[cnt], "Position_I_Gain", I_param);
 // dxl_wb.itemWrite(dxl_id[cnt], "Position_D_Gain", D_param);
 }
 dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Position");//инициализация обработчика синхронной записи положения
 dxl_wb.syncWrite(handler_index, dxl_id, 4, new_alphas, 1);//синхронная запись
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
 
 void setup() {
 DEBUG_SERIAL.begin(57600); // установка скорости обмена данными по последовательному порту с компьютером 
 dxl_wb.init(DEVICE_NAME, BAUDRATE); // инициализация устройств
 // установка режима работы для сервоприводов 
 uint16_t model_number = 0;
 for (int cnt = 0; cnt < 4; cnt++)
 {
 dxl_wb.ping(dxl_id[cnt], &model_number); 
 dxl_wb.jointMode(dxl_id[cnt], 20, 0);
 }
 // задание и переход в стартовую позицию для манипулятора 
 start_position();
 IK(0.15, 0, 0.05);
 delay(3000);
 }
 
 void loop() {
 // для того, чтобы управлять манипулятором в ручном режиме, открываем монитор последовательного порта и вводим координаты целевой позиции в сантиметрах. Например: «15 0 10»
 if (input_coords) {
 DEBUG_SERIAL.println("Введите координаты цели x,y,z в сантиметрах через пробел:");
 input_coords = false;
 }
 
 if (DEBUG_SERIAL.available()) {
 parse_input(DEBUG_SERIAL.readString());//чтение координат
 DEBUG_SERIAL.println(X);
 DEBUG_SERIAL.println(Y);
 DEBUG_SERIAL.println(Z);
 IK(X,Y,Z);//функция по решению ОЗК
 delay(3000);
 input_coords = true;
 }
 }