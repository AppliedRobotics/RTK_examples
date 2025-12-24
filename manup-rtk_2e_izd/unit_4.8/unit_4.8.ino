//Библиотеки для работы с протоколом UDP и получения данных
#include <Ethernet.h> 
#include <EthernetUdp.h>
#include <Regexp.h> 
//МАС-адрес
byte mac[] = { 
 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED 
};
//IP-адрес
IPAddress ip(192, 168, 42, 3);
IPAddress ip_pal(192, 168, 42, 12);
IPAddress ip_ugl(192, 168, 42, 11);
IPAddress ip_lamp_ugl(192, 168, 42, 13);
IPAddress ip_lamp_pp(192, 168, 42, 14);
//Порт
unsigned int localPort = 8888; 
unsigned int UDPremotePort = 8888; 
//Буфер для хранения данных
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];
//Пример команды для паллетайзера
char manip_home[] = "p:200:0:70:0#";
//Объявление переменных
EthernetUDP Udp;
MatchState ms;
int Xangle_m, Yangle_m, Xangle_m1, Yangle_m1;
int Xm, Ym, Xm1, Ym1;
//координаты палеты для паллетайзера
int arr_mpal_s_pallet[12][3] = {
 {1, 75, 280},//левая центральная верхняя
 {2, 75, 235},//левая правая верхняя 
 {3, 33, 235},//левая правая вторая сверху
 {4, 35, 280},//левая центральная, вторая сверху
 {5, 33, 202},//правая левая, вторая сверху
 {6, -7, 235},//левая правая, третья сверху 
 {7, -7, 195},//правая левая, третья сверху 
 {8, -60, 280},//левая центральная, вторая снизу
 {9, -60, 200},//правая левая, вторая снизу 
 {10, -100, 240},//правая левая, вторая снизу 
 {11, -100, 200},//правая левая, вторая снизу 
 {12, -60, 240}//левая правая вторая снизу 
};

//координаты палеты для углового манипулятора
int arr_mugl_s_pallet[12][3] = {
 {1, 150, 60},//левая центральная верхняя
 {2, 192, 60},//левая правая верхняя
 {3, 192, 20},//левая правая вторая сверху
 {4, 150, 20},//левая центральная, вторая сверху
 {5, 230, 20},//правая левая, вторая сверху
 {6, 190, -20},// левая правая, третья сверху
 {7, 232, -20},//правая левая, третья сверху 
 {8, 145, -60}, // левая центральная, вторая снизу
 {9, 230, -60}, //правая левая, вторая снизу 
 {10, 190, -100}, //правая левая, вторая снизу 
 {11, 233, -100}, //правая левая, вторая снизу 
 {12, 190, -60}, //левая правая вторая снизу
};
//домашие координаты для паллетайзера
int arr_mpal_b_pallet[10][3] = {
 {1, 200, 0},//
 {2, 30, -215},
 {3, -14, -212},
 {4, -58, -210},
 {5, -102, -205},
 {6, 80, -165},
 {7, 40, -165},
 {8, -8, -165},
 {9, -50, -160},
 {10, -95, -160}
};
//домашние координаты для углового манипулятора
int arr_mugl_b_pallet[10][3] = {
 {1, 90, 170},//
 {2, 51, 186},
 {3, 10, 188},
 {4, -27, 190},
 {5, -70, 193},
 {6, 86, 135},
 {7, 40, -165},
 {8, -8, -165},
 {9, -50, -160},
 {10, -95, -160}
};
//Объявление статусов манипуляторов
int manipulatorStatusM, manipulatorCmdM;
int manipulatorStatusP, manipulatorCmdP;
//Функция отвечает за обработку данных с манипуляторов, получаемых по пакету UDP

void processUdp() {
 packetBuffer[UDP_TX_PACKET_MAX_SIZE]=0;
 int packetSize = Udp.parsePacket();
 if (packetSize) {
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    Serial.println(packetBuffer);
    if (packetBuffer[3] == '1') {
      Serial.print("M ");
      ms.Target(packetBuffer);
      char result = ms.Match("M:%d+:(%d+):(%d+):", 0);
      if (result == REGEXP_MATCHED) {
        char m[10];
        ms.GetMatch(m);
        if (ms.level == 2) {
          String mv = ms.GetCapture(m, 0), cmd = ms.GetCapture(m, 1);
          manipulatorStatusM = mv.toInt();
          manipulatorCmdM = cmd.toInt();
          Serial.print("MVm: ");
          Serial.print(manipulatorStatusM);
          Serial.print(" CMDm: ");
          Serial.print(manipulatorCmdM);
          Serial.println();
        }
      } else if (result == REGEXP_NOMATCH) {
          Serial.println("No luck");
        } else {
          Serial.println("error");
        }
    }
    else if (packetBuffer[3] == '2') {
      Serial.print("P ");
      MatchState ms;
      ms.Target(packetBuffer);
      char result = ms.Match("M:%d+:(%d+):(%d+):", 0);
      if (result == REGEXP_MATCHED) {
        char m[10];
        ms.GetMatch(m);
        if (ms.level == 2) {
          String mv = ms.GetCapture(m, 0), cmd = ms.GetCapture(m, 1);
          manipulatorStatusP = mv.toInt();
          manipulatorCmdP = cmd.toInt();
          Serial.print("MVp: ");
          Serial.print(manipulatorStatusP);
          Serial.print(" CMDp: ");
          Serial.print(manipulatorCmdP);
          Serial.println();
        }
      } else if (result == REGEXP_NOMATCH) {
        Serial.println("No luck");
      } else {
        Serial.println("error");
      }
    }
 }
}

void sendUDP(String str, IPAddress ip) {
  char out[str.length() + 1];
  str.toCharArray(out, str.length() + 1);
  Udp.beginPacket(ip, 8888);
  Udp.write(out);
  Udp.endPacket();
}

void lampColorPP(int color) {
  switch (color) {
    case 4:  //blue
      sendUDP(makeUdpToLamp(0, 1, 0, 0), ip_lamp_pp);
      break;
    case 1:  //red
      sendUDP(makeUdpToLamp(1, 0, 0, 0), ip_lamp_pp);
      break;
    case 3:  //green
      sendUDP(makeUdpToLamp(0, 0, 1, 0), ip_lamp_pp);
      break;
    case 2:  //yellow
      sendUDP(makeUdpToLamp(0, 0, 0, 1), ip_lamp_pp);
      break;
  }
}
void lampColorUGL(int color) {
  switch (color) {
    case 4:  //blue
      sendUDP(makeUdpToLamp(0, 1, 0, 0), ip_lamp_ugl);
      break;
    case 1:  //red
      sendUDP(makeUdpToLamp(1, 0, 0, 0), ip_lamp_ugl);
      break;
    case 3:  //green
      sendUDP(makeUdpToLamp(0, 0, 1, 0), ip_lamp_ugl);
      break;
    case 2:  //yellow
      sendUDP(makeUdpToLamp(0, 0, 0, 1), ip_lamp_ugl);
      break;
  }
}
//Формирование строки для лампы
String makeUdpToLamp(int a, int b, int c, int v) {
  //l:1:1:1:1#
  //red a, yellow b, green c, blue v
  String str = "l:" + String(a) + ":" + String(b) + ":" + String(c) + ":" + String(v) + "#";
  return str;
}
//Функция по формированию и отправке команды движения на паллетайзер
void move_mP(int x = 180, int y = 0, int z = 150, int g = 0) 
{
 processUdp();
 String o = "p:" + (String)x + ":" + (String)y + ":" + (String)z + ":" + (String)g + "#"; 
 char out[o.length() + 1];
 o.toCharArray(out, o.length() + 1);
 int cmdMemo = manipulatorCmdP;
 do {
 processUdp();
 if (manipulatorStatusP == 0 ) break;
 } while (manipulatorStatusP == 1);
 Udp.beginPacket(ip_pal, 8888);
 Udp.write(out);
 Udp.endPacket();
 manipulatorStatusP = 1;
 do {
 processUdp();
 if (manipulatorStatusP == 0 && manipulatorCmdP > cmdMemo ) break;
 } while (manipulatorStatusP == 1 || manipulatorCmdP <= cmdMemo );
}
//Функция по формированию и отправке команды движения на угловой манипулятор
void move_mUgl(int x = 180, int y = 0, int z = 150, int g = 0) 
{
  processUdp();
  String o = "g:" + (String)x + ":" + (String)y + ":0:" + (String)z + ":" + (String)g + "#";
  char out[o.length() + 1];
  o.toCharArray(out, o.length() + 1);
  int cmdMemo = manipulatorCmdM;
  do {
    processUdp();
    if (manipulatorStatusM == 0) break;
  } while (manipulatorStatusM == 1);
  Udp.beginPacket(ip_ugl, 8888);
  Udp.write(out);
  Udp.endPacket();
  manipulatorStatusM = 1;
  do {
    processUdp();
    if (manipulatorStatusM == 0 && manipulatorCmdM > cmdMemo) break;
  } while (manipulatorStatusM == 1 || manipulatorCmdM <= cmdMemo);
}
//Полный алгоритм работы системы
void auto_mode()
{ 
 random_coord();
 lampColorPP(4);
 lampColorUGL(3);
 move_mUgl(arr_mugl_b_pallet[0][1], arr_mugl_b_pallet[0][2], 150, 0); //домашняя позиция
 lampColorPP(3);
 lampColorUGL(4);
 move_mP(135, 185, 100, 1);
 move_mP(135, 185, 40, 1);
 move_mP(135, 185, 100, 1);//захват с горки
 move_mP(Xm, Ym, 100, 1);
 move_mP(Xm, Ym, 50, 0);
 move_mP(Xm, Ym, 60, 0);
 delay(50);
 move_mP(Xm, Ym, 100, 0);
 //vtoroy shar
 move_mP(135, 185, 100, 1);
 move_mP(135, 185, 40, 1);
 move_mP(135, 185, 100, 1);
 move_mP(Xm1, Ym1, 100, 1);
 move_mP(Xm1, Ym1, 50, 0);
 move_mP(Xm1, Ym1, 60, 0);
 delay(50);
 move_mP(Xm1, Ym1, 100, 0);
 move_mP(arr_mpal_b_pallet[0][1], arr_mpal_b_pallet[0][2], 150, 0);//домашняя позиция
 delay(1000);
 lampColorPP(4);
 lampColorUGL(3);
 move_mUgl(Xangle_m, Yangle_m, 100, 1);
 move_mUgl(Xangle_m, Yangle_m, 40, 1);
 move_mUgl(Xangle_m, Yangle_m, 100, 1);
 move_mUgl(145, 235, 100, 1);//кидает на горку
 move_mUgl(145, 235, 80, 0);
 delay(50);
 move_mUgl(145, 235, 100, 0);
 //vtoroy shar
 move_mUgl(Xangle_m1, Yangle_m1, 100, 1);
 move_mUgl(Xangle_m1, Yangle_m1, 40, 1);
 move_mUgl(Xangle_m1, Yangle_m1, 100, 1);
 move_mUgl(145, 235, 100, 1);//кидает на горку
 move_mUgl(145, 235, 80, 0);
 delay(50);
 move_mUgl(145, 235, 100, 0);
 
}
//Реализация алгоритма случайного перемещения объекта по заранее известным координатам
void random_coord()
{
  //первый шарик
 int a = random(11);
 Xm = arr_mpal_s_pallet[a][1];
 Ym = arr_mpal_s_pallet[a][2];
 Xangle_m = arr_mugl_s_pallet[a][1];
 Yangle_m = arr_mugl_s_pallet[a][2];
 Serial.println("X: "+String(Xm) + " Y: "+String(Ym));
 Serial.println("X: "+String(Xangle_m) + " Y: "+String(Yangle_m));
 int b;
 //второй шарик
 do{
  b = random(11);
 }while(a == b);
 Xm1 = arr_mpal_s_pallet[b][1];
 Ym1 = arr_mpal_s_pallet[b][2];
 Xangle_m1 = arr_mugl_s_pallet[b][1];
 Yangle_m1 = arr_mugl_s_pallet[b][2];
}
void setup() {
 Serial.begin(115200); 
 Ethernet.init(10); 
 Ethernet.begin(mac, ip); 
 Udp.begin(localPort);
 //Перемещение манипуляторов в базовую позицию 
 processUdp();
  move_mUgl(arr_mugl_b_pallet[0][1], arr_mugl_b_pallet[0][2], 150, 0);
  move_mP(arr_mpal_b_pallet[0][1], arr_mpal_b_pallet[0][2], 75, 0);
}
void loop() {
 auto_mode();
}
