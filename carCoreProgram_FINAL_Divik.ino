#include <WiFiClient.h>
#include <LWiFi.h>
#include <task.h>
#include <math.h>
#include <FreeRTOS.h>


#define SSID "1scream2.4G"
#define PASSWD "2017scream"
#define TCP_IP "192.168.0.71"
#define TCP_PORT 5000
#define FORWARD_TIME 100

static bool timetogo = false; //false for waiting for start, true for can remote

//wificlient connect between car and wifiserver
WiFiClient wifiClient;
static char buf[128], buf_send[128];
static char client_ID[] = "Divik", Team[] = "A", BaseA = 'C', BaseB = 'C';
static int treasure[4][2] = {0};
static int index = 0, hp = 0;
bool falsetrue = false;

//step for Dst1 to Dst2, check for things in step
static char *recv_ID, *recv_buf, *recv_mod, name[32];

enum MotorPinID
{
  L_F = 0,
  L_B,
  R_F,
  R_B,
  NUM_OF_MOTOR_PIN
};

class point
{
  public:
    point(int x = -1, int y = -1);
    int x, y;
};

point::point(int x, int y)
{
  this->x = x;
  this->y = y;
}

//Dst1 for first destination(may not be my treasure)
point DstPos, MyPos, lighthouse[3];
const point PosA(96, 96), PosB(480, 480);


// Pin assignment
static const uint8_t motorPins[NUM_OF_MOTOR_PIN] = {16, 17, 14, 15};  //  L_F, L_B, R_F, R_B

void reg_ID() {
  strcpy(buf, "Register");
  strcat(buf, Team);
  strcat(buf, "|");
  strcat(buf, client_ID);
  wifiClient.write(buf, strlen(buf));
  wifiClient.flush();
}

void send_mes(char ID[], char mes[]) {
  sprintf(buf, "%s|%s", ID, mes);
  wifiClient.write(buf, strlen(buf));
  wifiClient.flush();
}

void setup() {
  int motorpins = 0;
  int status = WL_IDLE_STATUS;
  while (motorpins < NUM_OF_MOTOR_PIN) {
    pinMode(motorPins[motorpins], OUTPUT);
    motorpins++;
  }
  Serial.begin(115200);
  while (!Serial);
  // set WiFi
  WiFi.begin(SSID, PASSWD);
  while (status != WL_CONNECTED) {
    // Connect failed, blink 0.5 second to indicate
    // the board is retrying.
    delay(500);
    status = WiFi.begin(SSID, PASSWD);
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(SSID);
  }

  // Conenct to AP successfully
  while (!wifiClient.connect(TCP_IP, TCP_PORT)) {
    Serial.print("Attempting to connect to SERVER: ");
    Serial.println(TCP_IP);
  }
  reg_ID();
  xTaskCreate(
    askPos,          /* Task function. */
    "askPos",        /* String with name of task. */
    10000,           /* Stack size in words. */
    NULL,            /* Parameter passed as input of the task */
    1,               /* Priority of the task. */
    NULL);           /* Task handle. */
}


void askPos( void * parameter ) {
  //get message from server
  //format: Master|Words
  //or like: Master|(1,2)(3,4)
  //so we need to escape | and some characters
  while (1) {
    //read message from server
    if (wifiClient.available())
    {
      for (int i = 0; i < 128 && buf[i - 1] != '\r'; i++)
      {
        buf[i] = wifiClient.read();
      }
      recv_ID = strtok(buf, "|");
      Serial.println(recv_ID);
      Serial.println("####################");
      recv_buf = strtok(NULL, "|");
      Serial.println(recv_buf);
      if (!strcmp(recv_ID, "Master")) {   //From Master
        if (!strncmp(recv_buf, "Start", 5))
        { //Start
          Serial.println("Start!!");
          timetogo = true;
        }
        else if (!strncmp(recv_buf, "Done", 4))
        { //End
          timetogo = false;
        }
        else { //Something else
          recv_mod = strtok(recv_buf, ":");
          if (!strncmp(recv_mod, "POS", 3)) {
            recv_mod = strtok(NULL, "\n");
            //            Serial.println(recv_mod);
            sscanf(recv_mod, "(%d, %d)BaseA:%cBaseB:%cTowers:(%d, %d)(%d, %d)(%d, %d)Blood:%d", &MyPos.x, &MyPos.y, &BaseA, &BaseB, &lighthouse[0].x, &lighthouse[0].y, &lighthouse[1].x, &lighthouse[1].y, &lighthouse[2].x, &lighthouse[2].y, &hp);
          }
          //          Serial.println("========================");
          //          Serial.println(lighthouse[index].x);
          //          Serial.println(lighthouse[index].y);
          //          Serial.println(hp);
          //          Serial.println("========================");
          DstPos.x = lighthouse[index].x;
          DstPos.y = lighthouse[index].y;
        }
      }
      if (timetogo) {
        send_mes("Position", "");
      }
    }
  }
  vTaskDelete(NULL);
}

void freeze(int t) {
  analogWrite(motorPins[L_F], 0);
  analogWrite(motorPins[L_B], 0);
  analogWrite(motorPins[R_F], 0);
  analogWrite(motorPins[R_B], 0);
  delay(t);
}
void forward(int t, int power) {
  analogWrite(motorPins[L_F], power);
  analogWrite(motorPins[L_B], 0);
  analogWrite(motorPins[R_F], power);
  analogWrite(motorPins[R_B], 0);
  delay(t);
}
void backward(int t, int power) {
  analogWrite(motorPins[L_F], 0);
  analogWrite(motorPins[L_B], power);
  analogWrite(motorPins[R_F], 0);
  analogWrite(motorPins[R_B], power);
  delay(t);
}
void left(int t) {
  analogWrite(motorPins[L_F], 0);
  analogWrite(motorPins[L_B], 255);
  analogWrite(motorPins[R_F], 255);
  analogWrite(motorPins[R_B], 0);
  delay(t);
}
void right(int t) {
  analogWrite(motorPins[L_F], 255);
  analogWrite(motorPins[L_B], 0);
  analogWrite(motorPins[R_F], 0);
  analogWrite(motorPins[R_B], 255);
  delay(t);
}
void slightly_left(int t, int angle) {
  analogWrite(motorPins[L_F], 255 - angle);
  analogWrite(motorPins[L_B], 0);
  analogWrite(motorPins[R_F], 255);
  analogWrite(motorPins[R_B], 0);
  delay(t);
}
void slightly_right(int t, int angle) {
  analogWrite(motorPins[L_F], 255);
  analogWrite(motorPins[L_B], 0);
  analogWrite(motorPins[R_F], 255 - angle);
  analogWrite(motorPins[R_B], 0);
  delay(t);
}

void loop()
{
  //for self-moving
  double DstDir;
  double MyDir;
  double Degree;
  if (timetogo) { //for go to lighthouse
    point PrevPos(MyPos.x, MyPos.y);

    //go to lighthouse
    if (BaseA != 'O' && BaseB != 'O') {
      forward(FORWARD_TIME, 255);
      DstDir = atan2(DstPos.y - PrevPos.y, DstPos.x - PrevPos.x);
    } else if (BaseA == 'O') {
      if (abs(MyPos.x - PosA.x) <= 50 && abs(MyPos.y - PosA.y) <= 50) { //if get to the Dst1
        freeze(100);
        return;
      }
      forward(FORWARD_TIME, 255);
      DstDir = atan2(PosA.y - PrevPos.y, PosA.x - PrevPos.x);
    } else if (BaseB == 'O') {
      if (abs(MyPos.x - PosB.x) <= 50 && abs(MyPos.y - PosB.y) <= 50) { //if get to the Dst1
        freeze(100);
        return;
      }
      forward(FORWARD_TIME, 255);
      DstDir = atan2(PosB.y - PrevPos.y, PosB.x - PrevPos.x);
    }

    MyDir = atan2(MyPos.y - PrevPos.y, MyPos.x - PrevPos.x);
    Degree = MyDir - DstDir;

    if (Degree < -0.1 || Degree > PI + 0.1) {
      if (Degree < -PI / 2 || Degree > -PI * 3 / 2){
        right(50);
        PrevPos = MyPos;
        forward(100, 255);
      }
      else
//        slightly_right(75, (400 * Degree / PI > 255) ? 255 : 400 * Degree / PI);
        slightly_right(75, 128);
    } else if (Degree > 0.1) {
      if (Degree > PI / 2){
        left(50);
        PrevPos = MyPos;
        forward(100, 255);
      }
      else
//        slightly_left(75, (400 * Degree / PI > 255) ? 255 : 400 * Degree / PI);
        slightly_left(75, 128);
    }
  } else {
    freeze(100);
  }
}
