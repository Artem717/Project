//....................................................................инициализация библиотек
#include "HMC5883L.h"
#include <SoftwareSerial.h>
#include "iarduino_GPS_NMEA.h"
#include <Servo.h> 
#include <math.h>
#include <Wire.h>
SoftwareSerial GPS(PB7, PB6);
//....................................................................задача переменных
//....................................................................переменные GPS
iarduino_GPS_NMEA gps;   
unsigned long start;
long out_lat, out_lon, lat_home, lon_home; //lat широта , lon долгота
int out_spe;//скорость
float lat1, lon1, lat2, lon2;
int out_quan_sat; //кол-во спутников
//...................................................................режимы
int in_mode=0;//выбор режима
bool starts=1;//флаг старта
bool enab=0;//достижение точки
bool ret_home=false;//авария
bool hand=false;//ручной режим
float az_home=0;//азимут домашней точки
bool f_sos_time=false;//флаг отсчёта времени сигнала SOS
unsigned long sos_time=0;//время сигнала SOS
bool sig=false;
unsigned long timer;
bool f_in1=1;
bool f_in2=1;
bool f_in3=1;
//...................................................................переменные дальномера
long duration, cm;
//...................................................................переменные компаса
HMC5883L compass;
float xv, yv, zv;
float heading;
float az_ship=0;//азимут
float az_ship_now;//азимут в данный момент
float az_fin;//азимут требующийся
bool f_az_ship_now;//флагзапоминания азимута в данный момент
float headingDegrees=0;
float scaler=0;
boolean scaler_flag = false;
float normal_vector_length=0;
float calibrated_values[3]; 
//...................................................................переменные двигатель правый
Servo motor_right;
bool f_reverseL=0;
//...................................................................переменные двигатель левый
Servo motor_left;
bool f_reverseR=0;
//...................................................................переменные двигатели
int js_position = 1500;  //Начальная позиция, всегда 1.5 мс для регуляторов бесколлекторных двигателей
int max_position = 2300; //Максимальное значение ШИМ 2.3 мс
int min_position = 800;  //Минимальное значени ШИМ 0.8 мс
int del_start = 0;  //Флаг задержки запуска
bool f_reverse=0; //флаг реверса
bool f_turn_left;//флаг поворота налево
bool f_turn_right;//флаг поворота налево
//...................................................................переменные двигатель средний
//...................................................................переменные работы с мусором
int in_garb=0;//выталкивание мусора
int down_sw1=0; //концевик задний пресс
int up_sw2=0;//концевик передний выталкиватель
int up_sw1=0;//концевик передний пресс
int down_sw2=0;//концевик задний выталкиватель
bool f_tap1=false;//флаг нажатия пресс
bool f_tap2=false;//флаг нажатия выталкиватель
bool f_open=false; //флаг открытия дверц
unsigned long garb_time=0; //время выхода мусора из кадра
unsigned long servo_time=0; //время закрытия левой дверцы
unsigned long servo_end_time=0; //время окончания закрытия дверц
unsigned long pres_start=0;//время начало пресования
unsigned long pres_stop=0;//время окончания пресования
unsigned long servo_time_press=0;//время закрытия правой дверцы
bool f_servo_time=false;//флаг начатия записи времени закрытия левой дверцы
bool f_pres_start=false;//флаг начатия записи времени начала пресования
bool f_servo_time_press=false;//флаг начатия записи времени закрытия правой дверцы
bool f_pres=false;//флаг прессования
bool f_pressed=false;//флаг спресованности
//...................................................................переменные сервопривод дверцы правой
Servo servo_right;
//...................................................................переменные сервопривод дверцы левый
Servo servo_left;
int t_servo=0;
//...................................................................переменные зрения
//...................................................................переменные получения данных
String Y0, Y1, Y2, Y3, Y4, Y5, Y6, Y7, Y8, Y9, Y10, Y11, Y12, Y13, Y14, Y15, Y16, Y17;
//...................................................................переменные отправки данных
unsigned long last_time1;//таймер отправки
int out_tra1=0;//концевик1
int out_tra2=0;//концевик2
int out_tra3=0;//концевик3
int out_tra4=0;//концевик4
int out_garb=0;//наличие мусора
int out_ret_home=0;//возврат домой
int out_lev_garb=0;//наполненность мусором
int out_bat=0;//напряжение
int out_dist=0;//растояние впереди
int out_enab=0;
int out_az=0;
bool out_unl_garb=false;
int out_war=0; //аварийные режимы
//...................................................................переменные ручного управления
int in_arr_up=0;//стрелка вверх
int in_arr_down=0;//стрелка ввниз
int in_arr_right=0;//стрелка вправо
int in_arr_left=0;//стрелка влево
bool f_down_hand;
//...................................................................переменные расчёта движения
float in_lat=0;//широта
float in_lon=0;//долгота
int in_val_up=1700;
int in_val_down=1300;
float ang_tr=0;//угол треугольника
float az_mot=0;//азимут направления
//...................................................................переменные управления светом
int in_light=0;//включить свет или нет
bool f_light_on=false;
//...................................................................переменные управления работы
int in_work=0;//работать или нет
int in_garb_cb=0;
//...................................................................конец переменных
//................................................функция преобразований для работы компаса
void transformation(float uncalibrated_values[3])    
{
  double calibration_matrix[3][3] = 
  {
    { 1.286, 0,  0.041},
    { 0.025, 1.197,  0.244},
    {0.036, 0.06,   1.719}
  };
  double bias[3] = 
  {
    -17.391,
    -480.952,
    -432.93
  };  
  //считаем
  for (int i=0; i<3; ++i) uncalibrated_values[i] = uncalibrated_values[i] - bias[i];
  float result[3] = {0, 0, 0};
  for (int i=0; i<3; ++i)
    for (int j=0; j<3; ++j)
      result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
  for (int i=0; i<3; ++i) calibrated_values[i] = result[i];
}
//функция стабилизации длины вектора магнитометра (стабилизация радиуса сферы)
void vector_length_stabilasation(){ 
  //рассчитать нормальную длину вектора
  if (scaler_flag == false)
  {
    getHeading();
    normal_vector_length = sqrt(calibrated_values[0]*calibrated_values[0] + calibrated_values[1]*calibrated_values[1] + calibrated_values[2]*calibrated_values[2]);
    scaler_flag = true;
  } 
  //рассчитать текущий скейлер
  scaler = normal_vector_length/sqrt(calibrated_values[0]*calibrated_values[0] + calibrated_values[1]*calibrated_values[1] + calibrated_values[2]*calibrated_values[2]);
  //применить текущий скаляр к калиброванным координатам (глобальный массив калиброванных значений)
  calibrated_values[0] = calibrated_values[0]*scaler;
  calibrated_values[1] = calibrated_values[1]*scaler;
  calibrated_values[2] = calibrated_values[2]*scaler;
}
void setup() {
  Serial.begin(19200);
  //Serial12.begin(9600); // com port и hc-12
  GPS.begin(57600);// gps
  gps.begin(GPS);
  //................................................................инициализация двигателей
  motor_left.attach(PB10, min_position, max_position);    //Инициальзация левого мотора (порт, начальная позиция, максимальная позиция)       !!!
  delay(400); 
  motor_right.attach(PB4, min_position, max_position);  //Инициальзация правого мотора (порт, начальная позиция, максимальная позиция)      !!!
  delay(400);
  //................................................................инициализация сервоприводов 
  servo_left.attach(PB11, 0, 180);
  servo_right.attach(PB5, 0, 180);
  servo_left.write(70);
  servo_right.write(110);
  t_servo=millis();
  //................................................................инициализация компаса
  Wire.setSCL(PB8);
  Wire.setSDA(PB9);
  Wire.begin(); 
  compass = HMC5883L();
  compass.setRange(HMC5883L_RANGE_1_3GA);
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  //................................................................инициализация дальномера
  pinMode(PA15, OUTPUT);
  pinMode(PB3, INPUT);
  //................................................................инициализация Raspberry
  pinMode(PA12, INPUT_PULLUP);
  //................................................................инициализация освещения
  pinMode(PA1, OUTPUT);//белый свет
  pinMode(PA6, OUTPUT);//красный свет
  digitalWrite(PA1, 1);
  digitalWrite(PA6, 1);
  delay(200);
  digitalWrite(PA1, 0);
  digitalWrite(PA6, 0);
  //................................................................инициализация звука
  pinMode(PA0, OUTPUT);//пищалка
  digitalWrite(PA0, 1);
  delay(200);
  digitalWrite(PA0, 0);
  //................................................................инициализация концевиков
  pinMode(PB0, INPUT);//задний выталкиватель
  pinMode(PA3, INPUT);//передний выталкиватель
  pinMode(PB1, INPUT);//задний пресс
  pinMode(PA2, INPUT);//передний пресс
  //................................................................инициализация пресса
  pinMode(PB15, OUTPUT);
  pinMode(PB14, OUTPUT);
  pinMode(PA8, OUTPUT);
  pinMode(PA7, INPUT_PULLUP);
  //................................................................инициализация выталкивателя
  pinMode(PB12, OUTPUT);
  pinMode(PB13, OUTPUT);

}
void loop() {
  getdata();
//......................................................................работа освещения
if(in_light==1 && f_light_on==0){
  f_light_on=1;
  digitalWrite(PA1, 1);//белый свет
  digitalWrite(PA6, 1);//красный свет
} else if(analogRead(PA5)<67 && f_light_on==0){
  f_light_on=1;
  digitalWrite(PA1, 1);//белый свет
  digitalWrite(PA6, 1);//красный свет
} else if(in_light==0 && analogRead(PA5)>=67 &&f_light_on==1) {
  f_light_on=0;
  digitalWrite(PA1, 0);//белый свет
  digitalWrite(PA6, 0);//красный свет
} 
//...................................................................... проверка потери связи

if(millis()-timer>1000){
  hand=true;
}else{
  hand=false;
}
if(millis()-timer>60000){
  hand=false;
  if(ret_home==false){
  f_sos_time=1;
  }
  ret_home=true;
  out_ret_home=1;
}else{
  ret_home=false;
  out_ret_home=0;
  f_sos_time=0;
}
//......................................................................работа постоянных узлов
//....................................................вольтметр
out_bat=analogRead(PA4)*0.013*100+10;
//....................................................работа дальномера
  digitalWrite(PA15, LOW);
  delayMicroseconds(5);
  digitalWrite(PA15, HIGH);
  delayMicroseconds(10);
  digitalWrite(PA15, LOW);
  duration = pulseIn(PB3, HIGH);
  out_dist = (duration/2) / 29.1; 
//....................................................получение состояния концевиков 
  up_sw1=analogRead(PA2);
  down_sw1=analogRead(PB1);
  up_sw2=analogRead(PA3);
  down_sw2=analogRead(PB0); 
//....................................................выгрузка мусора 
  if(in_garb==1 &&out_unl_garb==0){
      out_unl_garb=0;
      servo_left.write(26);
      servo_right.write(154);
      if(up_sw2<=1000 && f_tap2==0){
        analogWrite(PA8,250);
        digitalWrite(PB12,1);
        digitalWrite(PB13,0);
      }
      if(up_sw2>1000){
        f_tap2=1;
        analogWrite(PA8,240);
        digitalWrite(PB12,0);
        digitalWrite(PB13,1);
      } 
      if(down_sw2>1000 && f_tap2==1){
        out_unl_garb=1;
        f_tap2=0;
        analogWrite(PA8,0);
        digitalWrite(PB12,0);
        digitalWrite(PB13,0);
        servo_right.write(110);
        servo_left.write(70);
      }
    } else if(in_garb==0){
      out_unl_garb=0;
    } 
  //проверка окончания запуска микрокомпьютера
if(digitalRead(PA12)==1 && f_pres==0 && millis()-t_servo>60000 && in_garb==0 && in_mode!=3){
  out_garb=digitalRead(PA12);
 }else if(f_pres==0 && in_mode!=3){
   out_garb=in_garb_cb;
 }
 if(out_garb==1){
  f_open=1;
  servo_left.write(26);
  servo_right.write(154);
  garb_time=millis();
  f_servo_time=1;
 }else if(f_open==1 && in_garb==0){
   if(millis()-garb_time>1000){
    servo_left.write(155);
    if(f_servo_time==1){
    servo_time=millis();
    f_servo_time=0;
    }
    if(millis()-servo_time>3000){
    servo_right.write(25);
    f_pres=1;
    f_open=0;
    f_pres_start=1;
    servo_end_time=millis();
    }
   }
 }
//...............................................................пресcование
if(f_pres==1){
  if(millis()-servo_end_time>3000){
  if(down_sw1<=1000&&f_pressed==false){
     digitalWrite(PB14,0);//прессуем
     digitalWrite(PB15,1);
     if(f_pres_start==1){
     pres_start=millis();
     f_pres_start=0;
     }
  }
  if((digitalRead(PA7)==0||down_sw1>1000)&&f_pressed==false){
   //возвращаемся
   digitalWrite(PB14,1);
   digitalWrite(PB15,0);
   f_pressed=true;
   pres_stop=millis();
   if(down_sw1>1000){
    out_lev_garb=50;
   }else{
   out_lev_garb=100-(20*((pres_stop-pres_start)/1000)/8);
   }
   f_servo_time_press=1;
   }
  if(up_sw1>1000 && f_pressed==true){
    digitalWrite(PB14,0);//открываем
    digitalWrite(PB15,0);
    servo_left.write(26);
    delay(100);
    f_pressed=false;
    f_pres=0;
    servo_right.write(110);
    servo_left.write(70);
    //}
   }
  }
} 
out_bat=(out_bat+(analogRead(PA4)*0.013*100+10))/2;

//...............................................................GPS 
    if(millis()-start>1000){
    start=millis();
    gps.read();
    out_lat=gps.latitude*1000000;
    out_lon=gps.longitude*1000000;
    out_spe=gps.speed;
    out_quan_sat=gps.satellites[GPS_ACTIVE ];
    }
out_bat=(out_bat+(analogRead(PA4)*0.013*100+10))/2;
if(out_bat<1050){
  hand=false;
  ret_home=true;
  out_ret_home=1;
  f_sos_time=1;
}
//-----------------расчёт азимута направления
float lat1=out_lat/1000000;//
float lon1=out_lon/1000000;//
if(lat1!=0 || lon1!=0){
if(starts==true){//присвоение домашних координат(первый старт)
  lat_home=lat1;
  lon_home=lon1;
  starts=false;
}

if(lat2!=in_lat/1000000 || lon2!=in_lon/1000000){//присвоение координат цели
lat2=in_lat/1000000;// преобразуем переменные для далнейшего использования
lon2=in_lon/1000000;//
enab=0;
}
if(in_mode==3 || ret_home==true){ //присвоение домашних координат для возврата
  lat2=lat_home;
  lat2=lat_home;
}
double h = abs(lat1 - lat2) * (2 * PI * 6371210  / 360);
double sh= abs(lon1 - lon2)*(2 * PI * (6371210 * sin(lat2)/360));
double g = hypot(h, sh);
g = g - g / 10;
//азимут
ang_tr  = abs(sin(sh/g)*57.3);
            
double az=0;
if(lat1 > lat2 && lon1<lon2){
az_mot=180-ang_tr;
}
else if(lat1==lat2 && lon1<lon2)
{
az_mot=90;
}
else if (lon1 == lon2 && lat1 > lat2)
{
az_mot = 180;
}
else if (lon1 == lon2 && lat1 < lat2)
{
az_mot = 0;
}
else if (lon1 < lon2 && lat1 < lat2)
{
az_mot = ang_tr;
}
else if (lon1 > lon2 && lat1 < lat2)
{
az_mot = 360-ang_tr;
}
else if (lon1 > lon2 && lat1 == lat2)
{
az_mot = 270;
}
else if (lon1 > lon2 && lat1 > lat2)
{
az_mot = 180+ang_tr;
}
}
out_bat=(out_bat+(analogRead(PA4)*0.013*100+10))/2;
if(out_bat<1050){
  hand=false;
  ret_home=true;
  out_ret_home=1;
  f_sos_time=1;
}
senddata();//отправка данных
//...................................................................расчёт азимута движения     
float values_from_magnetometer[3];
getHeading();                             //
values_from_magnetometer[0] = xv;
values_from_magnetometer[1] = yv;
values_from_magnetometer[2] = zv;
transformation(values_from_magnetometer);
vector_length_stabilasation();
heading = atan2(calibrated_values[0], -calibrated_values[1]);
float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
heading += declinationAngle;
if (heading < 0){
    heading += 2 * PI;
}
if (heading > 2 * PI){
    heading -= 2 * PI;
}
// Перевод в градусы
az_ship = heading * 180/PI;
getdata();//получение данных
//..............................................................................................................выбор режима работы
if(in_mode==0 && ret_home==false){
  hand=false;
  motor_left.write(1500);
  delay(50);
  motor_right.write(1500);
  delay(90);
  f_reverseL=0;
  f_reverseR=0;
  f_in1=1;
  f_in2=1;
  f_in3=1;
}else if(in_mode==1&& ret_home==false){
  hand=false;
  //........................................................автоматический режим управления
  if(in_work==1){
   if(f_in1==1){
     motor_right.write(1500);
     delay(50);
     motor_left.write(1500);
     delay(90);
     f_in1=0;
     f_in2=1;
     f_in3=1;
     digitalWrite(PA0, 0);
     }
    if(enab==0){
    if(out_dist<200){
      if(f_az_ship_now==0){
      az_ship_now=az_ship;
      f_az_ship_now=1;
      }
      if(f_turn_left==0){//смотрим налево
        if(az_ship_now>=90){
          az_fin=az_ship_now-90;
        }else{
          az_fin=360-90+az_ship_now;
        }
      if(az_ship != az_fin){//крутимся против часовой
        motor_right.write(in_val_up);
        f_reverseL=0;
        if(f_reverseR==0){
        motor_left.write(800);
        delay(100);
        motor_left.write(1500);
        delay(100);
        f_reverseR=1;
       }
       motor_left.write(800);
       } else{
        f_turn_left=1;
       }
      }else if (f_turn_left==1 && f_turn_right==0){//смотрим направо
        if(az_ship_now<180){
          az_fin=az_ship_now+180;
        }else{
          az_fin=360-180-az_ship_now;
        }
        if(az_ship != az_fin){//крутимся по часовой
        motor_right.write(in_val_up);
        f_reverseL=0;
        if(f_reverseR==0){
        motor_left.write(800);
        delay(100);
        motor_left.write(1500);
        delay(100);
        f_reverseR=1;
       }
       motor_left.write(800);
       }
      }else{
        out_war=1;
      }
    }else{
      f_az_ship_now=0;
      f_turn_left=0;
    if(lat1==lat2 && lon1==lon2){
      enab=1;
      motor_right.write(1500);
      f_reverseR=0;
      motor_left.write(1500);
      f_reverseL=0;
    } else if(az_ship==az_mot){
      motor_left.write(in_val_up);
      motor_right.write(in_val_up);
      f_reverseL=0;
      f_reverseR=0;
      }else if(az_ship>az_mot){
      //крутимся против часовой
      motor_left.write(in_val_up);
      f_reverseR=0;
      if(f_reverseL==0){
       motor_right.write(800);
       delay(50);
       motor_right.write(1500);
       delay(90);
       f_reverseL=1;
      }
      motor_right.write(800);
    } else if(az_ship<az_mot){
      //крутимся по часовой
      motor_right.write(in_val_up);
      f_reverseL=0;
      if(f_reverseR==0){
       motor_left.write(800);
       delay(50);
       motor_left.write(1500);
       delay(90);
       f_reverseR=1;
      }
      motor_left.write(800);
      }
    }
    }
    }else{
      motor_right.write(1500);
      delay(50);
      motor_left.write(1500);
      delay(90);
    }
  }else if((in_mode==2 || hand==true) && ret_home==false){ //........................................................ручной режим управления
  if(f_in2==1){
  motor_right.write(1500);
  delay(50);
  motor_left.write(1500);
  delay(90);
  f_in1=1;
  f_in2=0;
  f_in3=1;
  }
  digitalWrite(PA0, 0);
  if(out_dist<=20){
      if(f_down_hand==0){
      motor_right.write(1500);
      delay(50);
      motor_left.write(1500);
      delay(90);
      f_down_hand=1;
      }
      if(f_reverseL==0){
       motor_left.write(800);
       delay(100);
       motor_left.write(1500);
       delay(100);
       f_reverseL=1;
       }
      motor_left.write(800);
      if(f_reverseR==0){
       motor_right.write(800);
       delay(100);
       motor_right.write(1500);
       delay(100);
       f_reverseR=1;
      }
      motor_right.write(800);
  }else{
  f_down_hand=0;
  if(in_arr_up==1){//вперёд
    motor_left.write(in_val_up);
    delay(80);
    motor_right.write(in_val_up);
    delay(90);
    f_reverseL=0;
    f_reverseR=0;
  
  }else if(in_arr_down==1){//назад
    if(f_reverseL==0){
       motor_left.write(800);
       delay(100);
       motor_left.write(1500);
       delay(100);
       f_reverseL=1;
      }
      motor_left.write(800);
    if(f_reverseR==0){
       motor_right.write(800);
       delay(100);
       motor_right.write(1500);
       delay(100);
       f_reverseR=1;
      }
      motor_right.write(800);
    
  }else if(in_arr_right==1){//вправо
    motor_left.write(in_val_up);
      f_reverseR=0;
      if(f_reverseL==0){
       motor_right.write(800);
       delay(100);
       motor_right.write(1500);
       delay(100);
       f_reverseL=1;
      }
      motor_right.write(800);
  }else if(in_arr_left==1){//влево
    motor_right.write(in_val_up);
      f_reverseL=0;
      if(f_reverseR==0){
       motor_left.write(800);
       delay(100);
       motor_left.write(1500);
       delay(100);
       f_reverseR=1;
      }
      motor_left.write(800);
    
  }else{
    motor_left.write(1500);
    delay(50);
    motor_right.write(1500);
    delay(30);
    f_reverseL=0;
    f_reverseR=0;
  }
  }
}else if(in_mode==3 || ret_home==true){//........................................................режим возврата
  hand=false;
  if(f_in3==1){
  motor_right.write(1500);
  delay(50);
  motor_left.write(1500);
  delay(50);
  f_in3=0;
  }
  switch (ret_home){
    case 1:
    if(f_sos_time==1){
     digitalWrite(PA0, 1);
     sos_time=millis();
     f_sos_time=0;
   } else if(millis()-sos_time>500 ){
     digitalWrite(PA0, sig);
     sig = !sig;
     sos_time=millis();
   }
   break;
   }
   if(out_dist<200){
      if(f_az_ship_now==0){
      az_ship_now=az_ship;
      f_az_ship_now=1;
      }
      if(f_turn_left==0){//смотрим налево
        if(az_ship_now>=90){
          az_fin=az_ship_now-90;
        }else{
          az_fin=360-90+az_ship_now;
        }
      if(az_ship != az_fin){//крутимся против часовой
        motor_right.write(in_val_up);
        f_reverseL=0;
        if(f_reverseR==0){
        motor_left.write(800);
        delay(100);
        motor_left.write(1500);
        delay(100);
        f_reverseR=1;
       }
       motor_left.write(800);
       } else{
        f_turn_left=1;
       }
      }else if (f_turn_left==1 && f_turn_right==0){//смотрим направо
        if(az_ship_now<180){
          az_fin=az_ship_now+180;
        }else{
          az_fin=360-180-az_ship_now;
        }
        if(az_ship != az_fin){//крутимся по часовой
        motor_right.write(in_val_up);
        f_reverseL=0;
        if(f_reverseR==0){
        motor_left.write(800);
        delay(100);
        motor_left.write(1500);
        delay(100);
        f_reverseR=1;
       }
       motor_left.write(800);
       }
      }else{
        out_war=1;
      }
    }else{
      f_az_ship_now=0;
      f_turn_left=0;
   if(lat1==lat2 && lon1==lon2){
      enab=1;
      motor_right.write(1500);
      f_reverseR=0;
      motor_left.write(1500);
      f_reverseL=0;
    }else if(az_ship>az_mot){
      //крутимся против часовой
      motor_left.write(in_val_up);
      f_reverseR=0;
      if(f_reverseL==0){
       motor_right.write(800);
       delay(50);
       motor_right.write(1500);
       delay(90);
       f_reverseL=1;
      }
      motor_right.write(800);
    } else if(az_ship<az_mot){
      //крутимся по часовой
      motor_right.write(in_val_up);
      f_reverseL=0;
      if(f_reverseR==0){
       motor_left.write(800);
       delay(50);
       motor_left.write(1500);
       delay(90);
       f_reverseR=1;
      }
      motor_left.write(800);
      }
    else if(az_ship==az_mot){
      //двигаемся вперёд
      motor_left.write(in_val_up);
      delay(50);
      motor_right.write(in_val_up);
      delay(50);
      f_reverseL=0;
      f_reverseR=0;
      }
}
}
//..........................................................................отправка данных
senddata();
}

//.......................................................................получение показаний компаса
void getHeading()
{ 
    Vector raw = compass.readRaw();    
    xv = (float)raw.XAxis;
    yv = (float)raw.YAxis;
    zv = (float)raw.ZAxis;
    
}
void getdata(){
  //.................................................................получение данных с управляющего устройства 
  if(Serial.available()){    
String Var="";  
timer=millis();
while (!Serial.available()) delay(20); 
delay(35); 
while (Serial.available())  
Var += (char)Serial.read();
//Serial1.println(Var);
 //разбиваю
Y0=Var.substring(Var.indexOf("YA")+2,Var.indexOf("AY")); 
Y1=Var.substring(Var.indexOf("YB")+2,Var.indexOf("BY")); 
Y2=Var.substring(Var.indexOf("YC")+2,Var.indexOf("CY")); 
Y3=Var.substring(Var.indexOf("YD")+2,Var.indexOf("DY"));
Y4=Var.substring(Var.indexOf("YE")+2,Var.indexOf("EY")); 
Y5=Var.substring(Var.indexOf("YF")+2,Var.indexOf("FY")); 
Y6=Var.substring(Var.indexOf("YG")+2,Var.indexOf("GY")); 
Y7=Var.substring(Var.indexOf("YH")+2,Var.indexOf("HY"));
Y8=Var.substring(Var.indexOf("YI")+2,Var.indexOf("IY")); 
Y9=Var.substring(Var.indexOf("YJ")+2,Var.indexOf("JY")); 
Y10=Var.substring(Var.indexOf("YK")+2,Var.indexOf("KY")); 
Y11=Var.substring(Var.indexOf("YL")+2,Var.indexOf("LY")); 
Y12=Var.substring(Var.indexOf("YM")+2,Var.indexOf("MY")); 
//меняю
in_lat=(Y0.toInt());
in_lon=(Y1.toInt());
in_mode=(Y2.toInt());  
in_arr_up=(Y3.toInt());
in_arr_down=(Y4.toInt());  
in_arr_right=(Y5.toInt());
in_arr_left=(Y6.toInt());
in_light=(Y7.toInt());
in_garb=(Y8.toInt());
in_work=(Y9.toInt());
in_garb_cb=(Y10.toInt());
in_val_up=(Y11.toInt()); 
}
}
void senddata(){
  if(millis()-last_time1>800){
last_time1=millis();
out_az=az_ship*100;
String test = "";
test += out_spe;//
       test += ";"; //
       test += out_az; //
       test += ";";//           Собираем переменную
       test += out_lon;//
       test += ";";//
       test += out_lat;//
       test += ";";//
       test += up_sw1;//
       test += ";";
       test += down_sw1;//
       test += ";";//
       test += up_sw2;//
       test += ";";
       test += down_sw2;//
       test += ";";//
       test += out_enab;//
       test += ";";//
       test += out_ret_home;//
       test += ";";//
       test += out_garb;//
       test += ";";//
       test += out_lev_garb;//
       test += ";";
       test += out_bat;//
       test += ";";
       test += out_dist;//
       test += ";";
       test += out_unl_garb;//
       test += ";";
       test += out_quan_sat;//
       test += ";";
       test += out_war;//
       test += ";";
       Serial.println(test);

}
}
