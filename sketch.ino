// https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
// https://emanual.robotis.com/docs/en/parts/controller/opencm904/
// https://bestprogrammer.ru/programmirovanie-i-razrabotka/vozvrat-massiva-iz-funktsii-c
// https://ru.stackoverflow.com/questions/526433/
#include <Dynamixel2Arduino.h>  // Подключение библиотеки Dynamixel
#include <math.h>
#include "GyverTimer.h" //Использования этого таймера, чтобы избежать задержек
using namespace ControlTableItem; // Пространство имён для обращения к параметрам диномикселей
#define CONVERT_DEG_TO_GOAL_POS_FUNC_DEBUG true // Отладка функции конвертирования градусов в Goal Position
#define MOVE_SERV_TO_GOAL_POS_FUNC_DEBUG true // Отладка функции управления перемещения до Goal Position
#define WAIT_SERV_POS_PERF_FUNC_DEBUG true // Отладка функции ожидания занятия позиций серваками
#define IK_FUNC_DEBUG true // Отладка функции обратной кинематики
#define MAX_INPUT_VAL_AT_MANUAL_CONTROL 6 // Максимальное количество значений в строку монитора порта при ручном управлении
#define DEBUG_SERIAL Serial // Установка константы, отвечающей за последовательный порт, подключаемый к компьютеру
#define DXL_SERIAL Serial3 // OpenCM9.04 EXP Board's DXL port Serial. (To use the DXL port on the OpenCM 9.04 board, you must use Serial1 for Serial. And because of the OpenCM 9.04 driver code, you must call Serial1.setDxlMode(true); before dxl.begin();.)
#define DXL_DIR_PIN 22 // Инициализация переменной, отвечащей за номер пина, подключенного к информационному пину приводов манипулятора
#define DXL_PROTOCOL_VERSION 1.0 // Инициализация переменной, отвечащей за протокол передачи данных от OpenCM9.04 к приводам
#define JOINT_N 2 // Количество сервоприводов
#define DYNAMIXEL_DEG_OFFSET 30 // Оступ начала 0 позиции в градусах от разворота диномикселя
#define MAX_TIME_PERFORMED_POS 7000 // Максимальное время в мм для занятии позиции, время для защиты от невозможности занять позицию 
#define EXP_BOARD_BUTTON1_PIN 16 // Пин кнопки 1 на плате расширения
#define EXP_BOARD_BUTTON2_PIN 17 // Пин кнопки 2 на плате расширения
#define EXP_BOARD_LED1_PIN 18 // Пин светодиода 1 на плате расширения
#define EXP_BOARD_LED2_PIN 19 // Пин светодиода 2 на плате расширения
#define EXP_BOARD_LED3_PIN 20 // Пин светодиода 3 на плате расширения
// Для светодиодов на плате расширения, которые от земли
#define LED_HIGH LOW
#define LED_LOW HIGH

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
GTimer servosWorksMaxTimeTimer(MS);
GTimer serialPrintTimer(MS);
GTimer TIMER(MS);

struct Servo_D {
  Dynamixel2Arduino* d_dxl;   //Указатель на экземпляр динамикселя
  int* mode; //режим динамикселя
  int* position; //начальная позиция динамикселя
  bool* endsession; //хз
  int* id_s; //id каждого подключенного сервопривода
  int JOINT_S; //Кол-во испульзуемых динамикселей
  //смена направления вращения каждого динамикселя
  void next_mode(int id) {
    if (mode[id] == 0) {
      d_dxl->setOperatingMode(id, OP_VELOCITY);
      mode[id]++;
    } else {
      d_dxl->setOperatingMode(id, OP_POSITION);
      mode[id] = 0;
    }
  }
  //перевод из градусов в другую единицу измерения для динамикселя
  long double todeg(long double value) {
    return value * 1024.0 / 300.0;
  }
  //конструктор класса динамикселя
  Servo_D(Dynamixel2Arduino& dx, int J = 0, int* pos = nullptr, int* id = nullptr, int sz = 100) { //передается ссылка на экземпляр класса Dynamixel2Arduino, кол-во динамикселей, информация о режиме вращения для каждого динамикселя, массив id динамикселй
    JOINT_S = J;
    mode = new int[256];
    endsession = new bool[sz];
    position = new int[J];
    id_s = new int[J];
    d_dxl = &dx;   
    for (int i = 0; i < JOINT_S; i++) {
      position[i] = pos[i];
      id_s[i] = id[i];
    }
    for (int i = 0; i < 256; i++) {
      mode[i] = 0;
    }
  }
  //бесконечное вращение динамикселя. Передается скорость, id, направление вращения
  void inf_rotation(float speed, int id, bool dir = 0) {
    if (mode[id] == 0) next_mode(id);
    if (dir == 1) speed += 1023;
    d_dxl->setGoalVelocity(id, speed);
  }
  //проверка на подключение сервоприводов
  void check_serv() {
    for (int i = 0; i < JOINT_S; i++) {
      int id = id_s[i];
      if (d_dxl->ping(id) == true) {
        DEBUG_SERIAL.println("Dynamixel with ID " + String(id) + " found, model " + String(d_dxl->getModelNumber(id)) + ".");
      } else if (serialPrintTimer.isReady()) {
        DEBUG_SERIAL.println("Dynamixel with ID " + String(id) + " not found! Wait...");
      }
    }
  }
  //инициализация 
  void init() {
    for (int i = 0; i < JOINT_S; i++) {
      int id = id_s[i];

      d_dxl->torqueOff(id);
      bool setDinamixelOperationMode = d_dxl->setOperatingMode(id, OP_POSITION);
      if (!setDinamixelOperationMode && serialPrintTimer.isReady()) {
        DEBUG_SERIAL.println("Dynamixel with ID " + String(id) + " mode not set!");
      }
      d_dxl->torqueOn(id);

      pinMode(EXP_BOARD_BUTTON1_PIN, INPUT_PULLDOWN);
      pinMode(EXP_BOARD_BUTTON2_PIN, INPUT_PULLDOWN);
    }
    Serial.printf("succeced init!");
  }
  //Вращения сервоприводов. Передается массив нужных позиций в градусах для каждого сервопривода, погрешность, скорость
  void rotate(long double* target, long double eps, int speed, int session) {
    for (int i = 0; i < JOINT_S; i++) {
      int id = id_s[i];
      if (target[i] == -1) continue;
      long double targ = todeg(target[i]);
      long double cur = d_dxl->getPresentPosition(id);
      if (fabsl(targ - cur) <= eps) {
        target[i] = -1;
        d_dxl->setGoalVelocity(id, 0);
        //used[session][id] = true;
        continue;
      }
      if (mode[id] != 0) next_mode(id);   
      d_dxl->setGoalVelocity(id, speed);          
      d_dxl->setGoalPosition(id, (int)targ);      
    }
  }
  //понятия не имею как работает. Вряд ли работает. Не использовать:)
    void unsafe_rotate(long double* target, long double eps,  int speed, int session){
    for(int i = 0; i < JOINT_S; i++){
      int id = id_s[i]; 
      if(target[i] == -1) continue; 
      if(abs(target[i] - dxl.getPresentPosition(id)) <= eps){
        target[i] = -1;
        d_dxl->setGoalVelocity(id, 0);
        endsession[session] = true;
        Serial.println(id);
        continue;
      }
      if(endsession[session]) continue;
      long double start = dxl.getPresentPosition(id); 
      if(mode[id] == 0) next_mode(id); 
      long double targ = todeg(target[i]); 
      if(start <= targ){
        if(abs(start - targ) <= 1024 - abs(start - targ)) d_dxl->setGoalVelocity(id, speed);
        else d_dxl->setGoalVelocity(id, speed+1023);
      }
     else{
        if(abs(start - targ) <= 1024 - abs(start - targ)) d_dxl->setGoalVelocity(id, 1023 + speed);
        else d_dxl->setGoalVelocity(id, speed);
      }
    }
  }
};
// БЫЛО: Servo_D serv();
//Пример setup
Servo_D* serv = nullptr;
void setup() {
  DEBUG_SERIAL.begin(115200);
  DEBUG_SERIAL.println("Wait press btn1 (auto mode) or btn (manual control)...");
  DEBUG_SERIAL.println("Setup...");
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  serialPrintTimer.setInterval(500);
  serialPrintTimer.reset();
  TIMER.setInterval(5000); 
  TIMER.reset();
  int cnt = 2;
  int id[2] = {1, 2};
  int pos[2] = {0, 0};
  serv = new Servo_D(dxl, cnt, pos, id);
  serv->check_serv();
  serv->init();
}
bool Stop_Program = false;
void loop() {
  //!!!задержку использовать через GyverTimer
  if(Stop_Program == true) return;
  serv->inf_rotation(300, 1, 1); //пример использования inf_rotation 
  vector<ll> target = {180б 180}; //позиция, на которую нужно повернуть сервопривода. Дли i-го элемента массива target должен соответствовать id[i].
  //rotate(target, 3, 300, 0); пример использование rotate
  
}
