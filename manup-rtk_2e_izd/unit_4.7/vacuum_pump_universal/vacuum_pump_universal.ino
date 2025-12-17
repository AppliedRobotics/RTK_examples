#if defined(__OPENCM904__) 
  #define DEVICE_NAME "3" //подключенние 
  #define vacuum_pump 8 //пин, к которому подключен насос
  #define vacuum_key 9 //пин, к которому подключен пневмораспределитель
  #define RED_BUTTON 10 //пин, к которому подключена кнопка
  #define NanoPiSerial Serial1
#elif defined(__OPENCR__)
  #define vacuum_pump 51
  #define vacuum_key 57
  #define RED_BUTTON 63
  #define NanoPiSerial Serial2
  #define DEVICE_NAME ""
#endif

void setup() {
  pinMode(RED_BUTTON, INPUT_PULLUP);//установка пина на вход, подтянутый к питанию
  pinMode(vacuum_pump, OUTPUT);//установка пина на выход
  pinMode(vacuum_key, OUTPUT);//установка пина на выход
  digitalWrite(vacuum_pump, LOW);//подача нулевого значения
  digitalWrite(vacuum_key, LOW);//подача нулевого значения
  NanoPiSerial.begin(115200);// установка скорости обмена данными по последовательному порту с компьютером 
}

void loop() {
  if(!digitalRead(RED_BUTTON))//чтение состояния кнопки
  { 
    NanoPiSerial.println("push up");//кнопка не нажата
  }
  else
  {
    NanoPiSerial.println("push down"); //кнопка нажата
  }
  //чтение значения переменной c из NanoPi
  int c = NanoPiSerial.parseInt(); 
  if(c == 2){//подача нулевого значения 
    digitalWrite(vacuum_pump, LOW);
    digitalWrite(vacuum_key, LOW);
    }
  if(c == 1){//подача напряжения
    digitalWrite(vacuum_pump, HIGH);
    digitalWrite(vacuum_key, HIGH);
    }
  delay(100);
}
