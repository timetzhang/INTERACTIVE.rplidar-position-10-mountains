#include <Keyboard.h>
#include <RPLidar.h>

#define ENABLE 13 //Grounded this pin to enable the RPLIDAR
#define RPLIDAR_MOTOR 3 // this PWM pin for control the speed of RPLIDAR's motor.

RPLidar lidar;

uint8_t keys[10] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j'};
long lastKeyTime;

//确定x为负还是正
int operation;

//探测画幅, 单位为 0.1mm, 1000为1米
int left_width = 2800;
int right_width = 2800;
int height = 2400;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(ENABLE, INPUT_PULLUP);

  Keyboard.begin();
  lidar.begin(Serial1);

  // set pin modes
  pinMode(RPLIDAR_MOTOR, OUTPUT);
}

void loop() {
  if (digitalRead(ENABLE) == LOW) {
    if (IS_OK(lidar.waitPoint())) {
      float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
      float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
      bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
      byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement

      //如果检测到距离，开始工作
      if (distance > 0) {
        // 左边扇形与右边扇形度数处理
        if (angle >= 270) {
          angle = 360 - angle;
          operation = 1; //左边为 +x
        }
        else {
          operation = -1; //左边为 -x
        }

        //通过sin和cos得出向量(x,y)
        int x = operation * sin(angle * (PI / 180)) * distance;
        int y = cos(angle * (PI / 180)) * distance;

        //RPLIDAR CABLE往上, 所以处理 0 - 90 与 270 - 360
        if (x > -left_width && x < right_width && y < height && ((angle >= 0 && angle <= 90) || (angle >= 270 && angle <= 360))) {
          Serial.print(x);
          Serial.print(",");
          Serial.println(y);

          //X-Y位置对应的keys
          //Mountain 3
          if (x < -1584 && x > -1837 && y > 1216 && y < 1400) {
            Keyboard.write(keys[2]);
            delay(150);
          }
          //Mountain 4
          if (x < -863 && x > -1164 && y > 1363 && y < 1489) {
            Keyboard.write(keys[3]);
            delay(150);
          }
          //Mountain 5
          if (x < -315 && x > -638 && y > 1230 && y < 1680) {
            Keyboard.write(keys[4]);
            delay(150);
          }
          //Mountain 6
          if (x > 43 && x < 311 && y > 1358 && y < 1590) {
            Keyboard.write(keys[5]);
            delay(150);
          }
          //Mountain 7
          if (x > 759 && x < 1069 && y > 1228 && y < 1448) {
            Keyboard.write(keys[6]);
            delay(150);
          }
          //Mountain 8
          if (x > 1978 && x < 1873 && y > 2112 && y < 1558) {
            Keyboard.write(keys[7]);
            delay(150);
          }
          //Mountain 9
          if (x > 2255 && x < 2509 && y > 1406 && y < 1489) {
            Keyboard.write(keys[8]);
            delay(150);
          }
        }
      }
    } else {
      analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor

      // try to detect RPLIDAR...
      rplidar_response_device_info_t info;
      if (IS_OK(lidar.getDeviceInfo(info, 100))) {
        // detected...
        lidar.startScan();
        // start motor rotating at max allowed speed
        analogWrite(RPLIDAR_MOTOR, 190);
        delay(1000);
      }
    }
  }
  if (digitalRead(ENABLE) == HIGH) {
    analogWrite(RPLIDAR_MOTOR, 0);
    delay(2000);
  }
}
