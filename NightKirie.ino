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
static int index, step = 0, check = 0, hp = 0;
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
point Dst1Pos, MyPos, lighthouse[3];


// Pin assignment
static const uint8_t motorPins[NUM_OF_MOTOR_PIN] = {14, 15, 16, 17};  //  L_F, L_B, R_F, R_B
static const uint8_t buttonPins = 2;

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
  pinMode(buttonPins, INPUT);
  Serial.begin(115200);
  while (!Serial)
    ;
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
      for (int i = 0; i < 128 && buf[i-1] != '\r'; i++)
      {
        buf[i] = wifiClient.read();
      }
      recv_ID = strtok(buf, "|");
      Serial.println(recv_ID);
      Serial.println("####################");
      recv_buf= strtok(NULL,"|");
      Serial.println(recv_buf);
      if (!strcmp(recv_ID, "Master")) {   //From Master
        if (!strcmp(recv_buf, "Start"))
        { //Start
          Serial.println("Start!!");
          timetogo = true;
          step = 0;
          index = setIndex(MyPos.x, MyPos.y);

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
          Serial.println(recv_buf);
          Serial.println(MyPos.x);
          Serial.println(MyPos.y);
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
  if (step != 2) { //for game start
    pushButton();
    if (step == 0 && Dst1Pos.x != -1) { //for go to first point(may not be the treasure)
      if (check == 0) {
        double degree;
        point NewPos;
        switch (index) {
          case 0:
            NewPos.x = -(Dst1Pos.y - MyPos.y);
            NewPos.y = Dst1Pos.x - MyPos.x;
            break;
          case 1:
            NewPos.x = Dst1Pos.x - MyPos.x;
            NewPos.y = Dst1Pos.y - MyPos.y;
            break;
          case 2:
            NewPos.x = Dst1Pos.y - MyPos.y;
            NewPos.y = -(Dst1Pos.x - MyPos.x);
            break;
          case 3:
            NewPos.x = -(Dst1Pos.x - MyPos.x);
            NewPos.y = -(Dst1Pos.y - MyPos.y);
            break;
        }
        degree = atan2(NewPos.y, NewPos.x);
        if (degree >= 0) //turn 90 degree
          right(300);
        else
          left(300);
        check = 1;
      }
      else if (check == 1) {
        if (abs(MyPos.x - Dst1Pos.x) <= 50 && abs(MyPos.y - Dst1Pos.y) <= 50 && (falsetrue || timetogo == false)) { //if get to the Dst1
          step = 1;
          check = 0;
          Dst1Pos.x = -1;
          Dst1Pos.y = -1;
          freeze(0);
        }
        point PrevPos;

        forward(50);
        double Dst1Dir = atan2(Dst1Pos.y - PrevPos.y, Dst1Pos.x - PrevPos.x);
        double MyDir = atan2(MyPos.y - PrevPos.y, MyPos.x - PrevPos.x);
        if (MyDir - Dst1Dir < 0 || MyDir - Dst1Dir > PI)
          slightly_right(75);
        else if (MyDir - Dst1Dir > 0)
          slightly_left(75);
      }
    }
    else if (step == 1 && Dst1Pos.x != -1) { //for go to the real treasure
      if (check == 0) {
        int PrevPosX = MyPos.x;
        int PrevPosY = MyPos.y;
        forward(50);
        freeze(0);
        double Dst2Dir = atan2(Dst1Pos.y - PrevPosY, Dst1Pos.x - PrevPosX);
        double MyDir = atan2(MyPos.y - PrevPosY, MyPos.x - PrevPosX);
        if (MyDir - Dst2Dir < 0 || MyDir - Dst2Dir > PI)
          right(300);
        else if (MyDir - Dst2Dir > 0)
          left(300);
        check = 1;
      }
      else if (check == 1) {
        int PrevPosX = MyPos.x;
        int PrevPosY = MyPos.y;
        forward(50);
        freeze(0);
        double Dst2Dir = atan2(Dst1Pos.y - PrevPosY, Dst1Pos.x - PrevPosX);
        double MyDir = atan2(MyPos.y - PrevPosY, MyPos.x - PrevPosX);
        if (MyDir - Dst2Dir < 0 || MyDir - Dst2Dir > PI)
          slightly_right(75);
        else if (MyDir - Dst2Dir > 0)
          slightly_left(75);
      }
    }
    else {  //all other condition
      freeze(0);
    }
  }
  //for game end
  if (timetogo == false && step != 0)
  {
    step = 2;
    check = 0;
    Dst1Pos.x = -1;
    Dst1Pos.y = -1;
    freeze(0);
    falsetrue = false;
  }
}

//detect button is triggered or not
void pushButton()
{
  if (digitalRead(buttonPins) == HIGH)
  {
    backward(100);
    left(250);
    forward(100);
  }
}
//setup treasure index
int setIndex(int x, int y)
{
  //position is at up or down
  if (x >= 192 && x <= 256)
  {
    //position at up
    if (y <= 192) index = 0;
    //position at down
    else if (y >= 256) index = 2;
  }
  //position is at left or right
  else if (y >= 192 && y <= 256)
  {
    //position at left
    if (x <= 192) index = 3;
    //position at right
    else if (x >= 256) index = 1;
  }
}
