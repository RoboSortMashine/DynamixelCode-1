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
GTimer D_TIMER(MS); //Таймер для динамикселей
#include "Servo_D.h"
//Пример setup
Servo_D* serv = nullptr; //создание указателя на сервопривод 
//Пример создания setup:
void setup() {
  DEBUG_SERIAL.begin(115200);
  DEBUG_SERIAL.println("Wait press btn1 (auto mode) or btn (manual control)...");
  DEBUG_SERIAL.println("Setup...");
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  serialPrintTimer.setInterval(500);
  serialPrintTimer.reset();
  TIMER.setInterval(5000);  //запуск таймера 
  TIMER.reset();
  int cnt = 2; //кол-во сервоприводов 
  int id[2] = {1, 2}; //id i-го сервопривода
  int pos[2] = {0, 0}; //начальные позиции сервоприводов
  serv = new Servo_D(dxl, cnt, pos, id); //сохдание экземпляра класса 
  serv->check_serv(); //проверка сервоприводов 
  serv->init(); //инициализация сервопривода 
  D_TIMER.setInterval(5000); //Установка интервала таймера
}
bool Stop_Program = false;
void loop() { 
  if(Stop_Program) return;
  //Бесконечное вращение сервопривода с id 1
  serv->inf_rotation(300, 1, 0); //пример использования inf_rotation 
  if (D_TIMER.isReady()){
    serv->inf_rotation(0, 1, 0); //останавливаем бесконечное вращение
    Stop_Program = true;
  }
}







