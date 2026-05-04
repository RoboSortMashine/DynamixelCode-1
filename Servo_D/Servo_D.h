#ifndef SERVO_D_H
#define SERVO_D_H

struct Servo_D {
  Dynamixel2Arduino* d_dxl;   //Указатель на экземпляр динамикселя
  int* mode; //режим динамикселя
  long double* position; //начальная позиция динамикселя
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
  void update_position(){
    for(int i = 0; i < id_s.size(); i++){
      position[id_s[i]] = d_dxl->getPresentPosition(id_s[i]);
    }
  }
  //конструктор класса динамикселя
  Servo_D(Dynamixel2Arduino& dx, int J = 0, long double* pos = nullptr, int* id = nullptr, int sz = 100) { //передается ссылка на экземпляр класса Dynamixel2Arduino, кол-во динамикселей, информация о режиме вращения для каждого динамикселя, массив id динамикселй
    JOINT_S = J;
    mode = new int[256];
    endsession = new bool[sz];
    position = new long double[300];
    id_s = new int[J];
    d_dxl = &dx;   
    for (int i = 0; i < JOINT_S; i++) {
      //position[i] = pos[i];
      id_s[i] = id[i];
      position[id_s[i]] = pos[i];
    }
    for (int i = 0; i < 256; i++) {
      mode[i] = 0;
    }
  }
  //бесконечное вращение динамикселя. Передается скорость, id, направление вращения
  void inf_rotation(float speed, int id, bool dir = 0) {
    if (mode[id] == 0) next_mode(id);
    if (speed == 0) {
        d_dxl->setGoalVelocity(id, 0);
        position[id] = d_dxl->getPresentPosition(id);
        return;
    }
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
       // DEBUG_SERIAL.println("Dynamixel with ID " + String(id) + " mode not set!");
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
      d_dxl->setGoalPosition(id, targ);      
      position[id] = d_dxl->getPresentPosition(id);
    }
  }
  //понятия не имею как работает. Вряд ли работает. Не использовать:)
  /*
  void unsafe_rotate(long double* target, long double eps,  int speed, int session){
    for(int i = 0; i < JOINT_S; i++){
      int id = id_s[i]; 
      if(target[i] == -1) continue; 
      if(abs(target[i] - d_dxl->getPresentPosition(id)) <= eps){
        target[i] = -1;
        d_dxl->setGoalVelocity(id, 0);
        endsession[session] = true;
        Serial.println(id);
        continue;
      }
      if(endsession[session]) continue;
      long double start = d_dxl->getPresentPosition(id); 
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
  */
    
};
#endif 
