#include <WiFiClient.h>
#include <LWiFi.h>
#include <task.h>
#include <math.h>
#include <FreeRTOS.h>


#define SSID "1scream2.4G"
#define PASSWD "2017scream"
#define TCP_IP "192.168.0.50"
#define TCP_PORT 5000

static bool timetogo = false; //false for waiting for start, true for can remote

//wificlient connect between car and wifiserver
WiFiClient wifiClient;
static char buf[128], buf_send[128];
static char client_ID[] = "BLAYT!", Team[] = "A", BaseA = 'C', BaseB = 'C';
static int treasure[4][2] = {0};
static int index = 0, bigTurn = 1, hp = 0;
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


// Pin assignment
static const uint8_t motorPins[NUM_OF_MOTOR_PIN] = {14, 15, 16, 17};  //  L_F, L_B, R_F, R_B

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
    WiFi.begin(SSID, PASSWD);
    status = WiFi.begin(SSID, PASSWD);
    status =  WiFi.begin(SSID, PASSWD);
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
        if (!strcmp(recv_buf, "Start"))
        { //Start
          Serial.println("Start!!");
          timetogo = true;
        }
        else if (!strcmp(recv_buf, "Done"))
        { //End
          timetogo = false;
        }
        else { //Something else
          recv_mod = strtok(recv_buf, ":");
          if (!strcmp(recv_mod, "POS")) {
            recv_mod = strtok(NULL, ":");
            sscanf(recv_mod, "(%d, %d)BaseA:%cBaseB:%cTowers:(%d, %d)(%d, %d)(%d, %d)Blood:%d", &MyPos.x, &MyPos.y, &BaseA, &BaseB, &lighthouse[0].x, &lighthouse[0].y, &lighthouse[1].x, &lighthouse[1].y, &lighthouse[2].x, &lighthouse[2].y, &hp);
          }
        }
      }
      send_mes("Position", "");
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
void forward(int t) {
  analogWrite(motorPins[L_F], 150);
  analogWrite(motorPins[L_B], 0);
  analogWrite(motorPins[R_F], 150);
  analogWrite(motorPins[R_B], 0);
  delay(t);
}
void backward(int t) {
  analogWrite(motorPins[L_F], 0);
  analogWrite(motorPins[L_B], 150);
  analogWrite(motorPins[R_F], 0);
  analogWrite(motorPins[R_B], 150);
  delay(t);
}
void left(int t) {
  analogWrite(motorPins[L_F], 0);
  analogWrite(motorPins[L_B], 150);
  analogWrite(motorPins[R_F], 150);
  analogWrite(motorPins[R_B], 0);
  delay(t);
}
void right(int t) {
  analogWrite(motorPins[L_F], 150);
  analogWrite(motorPins[L_B], 0);
  analogWrite(motorPins[R_F], 0);
  analogWrite(motorPins[R_B], 150);
  delay(t);
}
void slightly_left(int t) {
  analogWrite(motorPins[L_F], 0);
  analogWrite(motorPins[L_B], 0);
  analogWrite(motorPins[R_F], 150);
  analogWrite(motorPins[R_B], 0);
  delay(t);
}
void slightly_right(int t) {
  analogWrite(motorPins[L_F], 150);
  analogWrite(motorPins[L_B], 0);
  analogWrite(motorPins[R_F], 0);
  analogWrite(motorPins[R_B], 0);
  delay(t);
}

void loop()
{
  //for self-moving
  point PrevPos;
  double DstDir;
  double MyDir;
  switch (index) {// lighthouse positon will automatically turn -1 when something gets there
    case 0:
      DstPos.x = lighthouse[0].x;
      DstPos.y = lighthouse[0].y;
      break;
    case 1:
      DstPos.x = lighthouse[1].x;
      DstPos.y = lighthouse[1].y;
      break;
    case 2:
      DstPos.x = lighthouse[2].x;
      DstPos.y = lighthouse[2].y;
      break;
  }
  if (DstPos.x != -1) { //for go to lighthouse
    PrevPos.x = MyPos.x;
    PrevPos.y = MyPos.y;
    forward(50);
    DstDir = atan2(DstPos.y - PrevPos.y, DstPos.x - PrevPos.x);
    MyDir = atan2(MyPos.y - PrevPos.y, MyPos.x - PrevPos.x);

    if (bigTurn) { // reduce error toward Dst
      if (MyDir - DstDir < 0 || MyDir - DstDir > PI) {
        double temp = (MyDir - DstDir < 0) ? abs(MyDir - DstDir) : (MyDir - DstDir - PI); // I want to figure out how much should I turn
        slightly_right(300 * temp / 90); // 300 for 90 degree
      }else if (MyDir - DstDir > 0){
        slightly_left(300 * (MyDir - DstDir) / 90); // 300 for 90 degree
      }
      bigTurn = 0;
    } else {
      if (abs(MyPos.x - DstPos.x) <= 50 && abs(MyPos.y - DstPos.y) <= 50) { //if get to the Dst
        DstPos.x = -1;
        DstPos.y = -1;
        freeze(0);
        bigTurn = 1;
      }

      if (MyDir - DstDir < 0 || MyDir - DstDir > PI)
        slightly_right(75);
      else if (MyDir - DstDir > 0)
        slightly_left(75);
    }
  } else { // if DstPos euqals -1, that mean some one before me reaches it, maybe I should stop?
    freeze(0);
  }
  //for game end
  if (timetogo == false)
  {
    DstPos.x = -1;
    DstPos.y = -1;
    freeze(0);
  }
}
