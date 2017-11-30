/* Receive the controlling message, turning on/off and pwm, and
   than set the corresponding pin.
*/
#include <WiFiClient.h>
#include <LWiFi.h>
#include <task.h>
#include <math.h>

#define SSID "1scream2.4G"
#define PASSWD "2017scream"
#define TCP_IP "192.168.0.38"
#define TCP_IP_PHONE "192.168.0.34"
#define TCP_PORT 5000
#define ENB 13
#define LEFT_CENTER_TIME 340 //it takes *** second to turn 90 degree
#define LEFT_WHEEL_TIME 460
#define RIGHT_CENTER__TIME 440
#define RIGHT_WHEEL_TIME 480
#define BACK_RIGHT_TIME 550
#define BACK_LEFT_TIME 600
#define BLOCK 710 //it takes *** second to walk 26 cm
#define MIDWAY 192 //3 * 64

static char buf[48], bufPh[48], buf_send[32], buf_phsend[32];
static char client_ID[] = "BBLYAT!", Team[] = "DWLT";
static char *recv_ID, *recv_buf;
static int messageLen, phmessageLen;
static int MyPosX, MyPosY, DstPosX, DstPosY;
int ENB_VAL = 255;
bool timeToGo = false;
enum motorPinID {
  L_F = 0,
  L_B,
  R_F,
  R_B,
  NUM_OF_MOTOR_PIN
};
enum ultrasonicPinID {
  U_F = 0,
  U_L,
  U_R,
  NUM_OF_ULTRASONIC_PIN
};
enum colorSensorPinID {
  s0 = 0,
  s1,
  s2,
  s3,
  SENSOR_OUT,
  NUM_OF_COLOR_SENSOR_PIN
};
static const uint8_t USTrigPins[NUM_OF_ULTRASONIC_PIN] = {2, 4, 11 };  // F, L, R
static const uint8_t USEchoPins[NUM_OF_ULTRASONIC_PIN] = {3, 5, 12 };  // F, L, R
static const uint8_t motorPins[NUM_OF_MOTOR_PIN] = {14, 15, 16, 17};  //  L_F, L_B, R_F, R_B
static const uint8_t CSPins[NUM_OF_COLOR_SENSOR_PIN] = {6, 7, 8, 9, 10};
WiFiClient wifiClientPh;
WiFiClient wifiClient;
IPAddress ip;

void setup()
{
  for (int motorpins = 0; motorpins < NUM_OF_MOTOR_PIN; motorpins++) {
    pinMode(motorPins[motorpins], OUTPUT);
  }
  analogWrite(ENB, ENB_VAL);

  int status = WL_IDLE_STATUS;
  Serial.begin(115200);
  while (!Serial)
    ;

  // set WiFi
  // WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWD);
  while (status != WL_CONNECTED) {
    // Connect failed, blink 0.5 second to indicate
    // the board is retrying.
    delay(500);
    WiFi.begin(SSID, PASSWD);
    status =  WiFi.begin(SSID, PASSWD);
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(SSID);
    Serial.println(status);
  }

  // Conenct to AP successfully
  // wifiClient.connect(TCP_IP, TCP_PORT);
  while (!wifiClient.connect(TCP_IP, TCP_PORT)) {
    //delay(300);
    Serial.print("Attempting to connect to SERVER: ");
    Serial.println(TCP_IP);
  }
/*
  //wifiClient.connectPh(TCP_IP_PHONE, TCP_PORT);
  while (!wifiClientPh.connect(TCP_IP_PHONE, TCP_PORT)) {
    delay(300);
    Serial.print("Attempting to connect to PHONE SERVER: ");
    Serial.println(TCP_IP_PHONE);
  }*/

  reg_ID();

  //delay(1000);
  xTaskCreate(
    askPos,          /* Task function. */
    "askPos",        /* String with name of task. */
    10000,            /* Stack size in words. */
    NULL,             /* Parameter passed as input of the task */
    1,                /* Priority of the task. */
    NULL);            /* Task handle. */
}

void reg_ID()
{
  strcpy(buf, "Register|");
  strcat(buf, client_ID);
  wifiClient.write(buf, strlen(buf));
  wifiClient.flush();
}

void send_mes(char ID[], char mes[])
{
  sprintf(buf, "%s|%s", ID, mes);
  wifiClient.write(buf, strlen(buf));
  wifiClient.flush();
}

void askPos( void * parameter )
{
  while (1)
  {
    if ((messageLen = wifiClient.available()) > 0) {
      int i = 0;
      do
      {
        buf[i++] = wifiClient.read();
      } while (i < 32 && buf[i - 1] != '\r');

      buf[i - 1] = '\0';
      recv_ID = strtok(buf, "|\0");
      Serial.print(recv_ID);
      Serial.print(":");
      recv_buf = strtok(NULL, "|\0");
      Serial.println(recv_buf);
      if (strcmp(recv_buf, "Start")==0) {
        timeToGo = true;
      }
      if(strcmp(recv_buf, "Done")==0){
        timeToGo = false;
      }
      sscanf(recv_buf, "POS:(%d,%d)(%d,%d)", &MyPosX, &MyPosY, &DstPosX, &DstPosY);
//    send_phone(MyPosX, MyPosY);
      send_mes("Position", "");
    }
    //delay(100);
  }
  
//  Serial.println("Ending task 1");
  vTaskDelete( NULL );
}
/*
void send_phone(int x, int y)
{
  sprintf(buf_phsend, "(%d,%d)", x, y);
  wifiClientPh.write(buf_phsend, strlen(buf_phsend));
  wifiClientPh.flush();
}*/

void stand(int t) { //set it zero if just want to clear the status of pins
  digitalWrite(motorPins[L_F], LOW);
  digitalWrite(motorPins[L_B], LOW);
  digitalWrite(motorPins[R_F], LOW);
  digitalWrite(motorPins[R_B], LOW);
  delay(t);
}
struct LEFT {
  void center(int t) {
    digitalWrite(motorPins[L_F], LOW);
    digitalWrite(motorPins[L_B], HIGH);
    digitalWrite(motorPins[R_F], HIGH);
    digitalWrite(motorPins[R_B], LOW);
    delay(t);
  }
  void wheel(int t) {
    digitalWrite(motorPins[L_F], LOW);
    digitalWrite(motorPins[L_B], LOW);
    digitalWrite(motorPins[R_F], HIGH);
    digitalWrite(motorPins[R_B], LOW);
    delay(t);
  }
};
struct RIGHT {
  void center(int t) {
    digitalWrite(motorPins[L_F], HIGH);
    digitalWrite(motorPins[L_B], LOW);
    digitalWrite(motorPins[R_F], LOW);
    digitalWrite(motorPins[R_B], HIGH);
    delay(t);
  }
  void wheel(int t) {
    digitalWrite(motorPins[L_F], HIGH);
    digitalWrite(motorPins[L_B], LOW);
    digitalWrite(motorPins[R_F], LOW);
    digitalWrite(motorPins[R_B], LOW);
    delay(t);
  }
};
struct MOVE {
  LEFT left;
  RIGHT right;
  void forward(int t) {
    digitalWrite(motorPins[L_F], HIGH);
    digitalWrite(motorPins[L_B], LOW);
    digitalWrite(motorPins[R_F], HIGH);
    digitalWrite(motorPins[R_B], LOW);
    delay(t);
  }
  void back(int t) {
    digitalWrite(motorPins[L_F], LOW);
    digitalWrite(motorPins[L_B], HIGH);
    digitalWrite(motorPins[R_F], LOW);
    digitalWrite(motorPins[R_B], HIGH);
    delay(t);
  }
};
MOVE moving;

long ultrasonicGetDistance(uint8_t trig, uint8_t echo)
{
    long duration;

    pinMode(trig, OUTPUT);
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(5);
    digitalWrite(trig, LOW);

    pinMode(echo, INPUT);
    duration = pulseIn(echo, HIGH, 5000000L);
    return duration / 29 / 2;
}
/*
void handleCommand()
{
  if (!timeToGo) {
    return;
  }
  // Stop moving
  if (bufPh[1] == 'E') {
    stand(100);
    return;
  }
  Serial.print(bufPh[0]);
  Serial.println(bufPh[1]);

  switch (bufPh[0]) {
    case 'F':   // Forward
      moving.forward(0);
      break;
    case 'B':   // Backward
      moving.back(0);
      break;
    case 'L':   // Turn left
      moving.left.wheel(0);
      break;
    case 'R':   // Turn right
      moving.right.wheel(0);
      break;
    case 'Z':   // Report ultrasonic distance and color
      moving.left.center(BACK_LEFT_TIME);
      stand(0);
      //    reportUltrasonic();
      //reportColorSensor();
      break;
  }
}*/
bool doIt = true;

void loop()
{
  if(!timeToGo){
    stand(0);
    return;
  }
  
  static const int InitPosX = MyPosX;
  static const int InitPosY = MyPosY;
  if(abs(MyPosX - InitPosX) < MIDWAY && abs(MyPosY - InitPosY) < MIDWAY){
    moving.forward(100);
    return;
  }
  if(doIt){
    int VectorX0 = MyPosX - InitPosX;
    int VectorY0 = MyPosY - InitPosY;
    int EndVectorX0 = DstPosX - InitPosX;
    int EndVectorY0 = DstPosY - InitPosY;
    int thetaEnd0 = atan2(EndVectorY0, EndVectorX0) * 180 / PI;
    int thetaPos0 = atan2(VectorY0, VectorX0) * 180 / PI;
    int theta0 = thetaEnd0 - thetaPos0;
    if(theta0 < -180)
      theta0 += 360;
    stand(150);
    if(theta0 < 0){
      moving.left.wheel(LEFT_WHEEL_TIME * 60 / 90);
    }else if(theta0 > 0){
      moving.right.wheel(RIGHT_WHEEL_TIME * 60 / 90);
    }
    doIt = false;
    return;
  }

  static const int StartPosX = MyPosX;
  static const int StartPosY = MyPosY;
  static const int EndVectorX = DstPosX - StartPosX;
  static const int EndVectorY = DstPosY - StartPosY;
  static const float thetaEnd = atan2(EndVectorY, EndVectorX) * 180 / PI;
  float thetaPos, theta, thetaNow, theta2;
  int VectorX;
  int VectorY;
  int LastPosX, LastPosY;
  int VectorNowX, VectorNowY;
  int adjustmentRate;
  
  LastPosX = MyPosX;
  LastPosY = MyPosY;
  moving.forward(100);
  VectorX = MyPosX - StartPosX;
  VectorY = MyPosY - StartPosY;
  VectorNowX = MyPosX - LastPosX;
  VectorNowY = MyPosY - LastPosY;
  
  thetaNow = atan2(VectorNowY, VectorNowX) * 180 / PI;
  thetaPos = atan2(VectorY, VectorX) * 180 / PI;
  theta2 = thetaEnd - thetaNow;
  theta = thetaEnd - thetaPos;
  if(theta2 < -180)
    theta2 += 360;
  if(theta < -180)
    theta += 360;
  adjustmentRate = asin(32 / sqrt(pow(VectorNowX, 2) + pow(VectorNowY, 2))) * 180 / PI;

  if(theta < -adjustmentRate){
    moving.left.wheel(35);
  }else if(theta > adjustmentRate){
    moving.right.wheel(35);
  }else{
    if(theta2 < -2){
      moving.left.wheel(8);
    }else if(theta2 > 2){
      moving.right.wheel(8);
    }
  }
}
