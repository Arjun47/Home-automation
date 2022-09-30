#include <DS3231.h>

// Init the DS3231 using the hardware interface
DS3231 rtc(SDA, SCL);
Time t;
int BULB_ON_LIMIT = 50;                                   // time till BULB_RELAY remains on after PIR trigger
int RETRY_WAIT_TIME = 480;                                // retry wait time of water
int waterCheckTime = 120;                                 // seconds till water flow needs to be monitored
int WATER_FILL_RETRY_LIMIT = 5;                           // water retry attempts limit
int retryWaterAttempts = 0;                               // track retrying of waterStatus test
const int TANK_STATUS_INPUT_PIN = 9;                      // tank switch input
const int PIR_SENSOR_INPUT_PIN = 4;                       // PIR sensor input pin
const int LED_INFO_OUTPUT_PIN = 13;                       // used for some indicator
const int BULB_RELAY_OUTPUT_PIN = 7;                      // Bulb relay
const int MOTOR_RELAY_OUTPUT_PIN = 8;                     // Motor relay
unsigned char FLOW_SENSOR_INPUT_PIN = 2;                  // Sensor Input
const int BUZZER_PIN = 10;                                // buzzer
volatile int flow_frequency;                              // Measures flow sensor pulses
unsigned int litrePerHour;                                // Calculated litres/hour
unsigned long currentTime;                                // temporary current time in milli secs
unsigned long cloopTime;                                  // for checking 1 sec
int BULB_COUNTER = 0;                                     // count each second till BULB_ON_LIMIT
bool waterFillingStart = false;                           // FLAG for water filling process
bool retryPending = false;                                // FLAG for is waiting for retry time
bool isTankFull = false;                                  // FLAG for tank full status
bool isMotorOn = false;                                   //FLAG for motor on/off status
bool MOTION_FLAG;                                         // true if motion detected
int waterStatusAvg = 0;                                   // Average above the threshold
long int waterStatusCounter = 0;                          // incrementing counter for checking waterCheckTime
bool waterStatusFlag = false;                             // FLAG for water flow
bool setToDefaultFlag = false;                            // resetting all variable to default
bool timeInRange = false;                                 // time in range flag
int buzzerCounter = 0;                                    // incrementing for checking buzzer on time
unsigned long int retryTimeMinute = 10999999999999999999; // retry time i.e. current failed + 10 minutes initialized to infinity
#define MOTOR_OFF HIGH
#define MOTOR_ON LOW

void flow() // Interrupt function
{
  flow_frequency++;
}

void setup()
{
  Serial.begin(9600);
  rtc.begin();
  pinMode(PIR_SENSOR_INPUT_PIN, INPUT);
  pinMode(TANK_STATUS_INPUT_PIN, INPUT);
  pinMode(LED_INFO_OUTPUT_PIN, OUTPUT);
  pinMode(MOTOR_RELAY_OUTPUT_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BULB_RELAY_OUTPUT_PIN, OUTPUT);
  pinMode(FLOW_SENSOR_INPUT_PIN, INPUT);
  digitalWrite(FLOW_SENSOR_INPUT_PIN, HIGH); // Optional Internal Pull-Up
  digitalWrite(BULB_RELAY_OUTPUT_PIN, LOW);
  digitalWrite(MOTOR_RELAY_OUTPUT_PIN, HIGH);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_INFO_OUTPUT_PIN, LOW);
  MOTION_FLAG = false;
  attachInterrupt(0, flow, RISING); // Setup Interrupt
  sei();                            // Enable interrupts
  currentTime = millis();
  cloopTime = currentTime;
}

void loop()
{
  t = rtc.getTime();
  int PIR_Sensor;
  int tankFullStatusSensor;
  //read sensors
  PIR_Sensor = digitalRead(PIR_SENSOR_INPUT_PIN);
  tankFullStatusSensor = digitalRead(TANK_STATUS_INPUT_PIN);
  //Serial.print("tankFullStatusSensor");
  //Serial.print(tankFullStatusSensor);
  if (tankFullStatusSensor == HIGH)
  {
    isTankFull = true;
    //Serial.println("Tank is full");
  }
  else
    isTankFull = false;

  if (PIR_Sensor == LOW)
  {
    if (BULB_COUNTER > BULB_ON_LIMIT and MOTION_FLAG)
    {
      digitalWrite(BULB_RELAY_OUTPUT_PIN, LOW);
      BULB_COUNTER = 0;
      MOTION_FLAG = false;
    }
    else if (!MOTION_FLAG)
    {
      //Serial.print("No object in sight");
      digitalWrite(BULB_RELAY_OUTPUT_PIN, LOW);
    }
  }
  else if ((PIR_Sensor == HIGH) and BULB_COUNTER < BULB_ON_LIMIT)
  {
    digitalWrite(BULB_RELAY_OUTPUT_PIN, HIGH);
    //Serial.print("Object detected");
    MOTION_FLAG = true;
  }
  if (MOTION_FLAG)
  {
    BULB_COUNTER++;
  }
  //print_flags();
  //Serial.print(t.hour);
  //Serial.print("-");
  //Serial.println(t.min);
  // motor logic start
  check_correct_time(t.hour, t.min);
  if (timeInRange and retryWaterAttempts < WATER_FILL_RETRY_LIMIT and isTankFull == false)
  {
    //Serial.println("started motor logic");
    if (waterFillingStart == false)
    {
      waterFillingStart = true;
      setMotor(MOTOR_ON);
      isMotorOn = true;
      //Serial.println("motor turned on");
      waterStatusFlag = true; // initial assumption
      setToDefaultFlag = false;
      buzzerCounter = 0;
    }
    if (waterStatusLastSec())
      waterStatusAvg++;
    if (waterStatusCounter > waterCheckTime)
    {
      if (waterStatusAvg > (waterCheckTime * 0.6))
      {
        waterStatusFlag = true;
        digitalWrite(LED_INFO_OUTPUT_PIN, HIGH);
        //Serial.println("water is coming");
      }
      else
      {
        waterStatusFlag = false;
        digitalWrite(LED_INFO_OUTPUT_PIN, LOW);
        //Serial.println("water is NOT coming");
        if (!retryPending)
        {
          retryPending = true;
          retryTimeMinute = minutePlusTen(t);
          //Serial.println(retryTimeMinute);
        }
        //retryPending = true;
      }
      //Serial.print(rtc.getUnixTime(t));
      //Serial.print("---");
      //Serial.println(retryTimeMinute);
      if (retryPending == true and rtc.getUnixTime(t) > retryTimeMinute)
      {
        retryWaterAttempts++;
        setMotor(MOTOR_ON); // motor ON
        isMotorOn = true;
        //Serial.println("Motor turned ON after retry wait");
        //Serial.println(retryWaterAttempts);
        waterStatusFlag = true; // assumption after retry
        retryPending = false;
      }
      //reset counters
      waterStatusCounter = 0;
      waterStatusAvg = 0;
    }
  }
  else if ((!timeInRange or isTankFull == true) and setToDefaultFlag == false)
  {
    //Serial.println("entered resetting");
    resetToDefault();
    setToDefaultFlag = true;
    digitalWrite(LED_INFO_OUTPUT_PIN, LOW);
  }
  if (isMotorOn and (waterStatusFlag == false or isTankFull == true or retryWaterAttempts == WATER_FILL_RETRY_LIMIT))
  {
    //Serial.println("Motor turned OFF");
    setMotor(MOTOR_OFF); // motor OFF
    isMotorOn = false;
  }
  if (isTankFull)
  {
    if (buzzerCounter <= 25)
    {
      //Serial.println("Buzzer is ON");
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerCounter++;
    }
    else
    {
      //Serial.println("Buzzer is OFF");
      digitalWrite(BUZZER_PIN, LOW);
    }
  }
  delay(1000);
  waterStatusCounter++;
  //Serial.println(waterStatusCounter);
  print_time();
}

void resetToDefault()
{
  retryWaterAttempts = 0;
  waterFillingStart = false;
  retryPending = false;
  waterStatusAvg = 0;
  waterStatusCounter = 0;
  waterStatusFlag = false;
}

long int minutePlusTen(Time t_)
{
  long int currentTimeUnix = rtc.getUnixTime(t_);
  return currentTimeUnix + 600;
}

void setMotor(bool motorStatus)
{
  digitalWrite(MOTOR_RELAY_OUTPUT_PIN, motorStatus);
}

bool waterStatusLastSec()
{
  //Serial.println("water check");
  bool water_coming = false;
  currentTime = millis();
  if (currentTime >= (cloopTime + 1000))
  {
    cloopTime = currentTime; // Updates cloopTime
    litrePerHour = (flow_frequency * 60 / 7.5);
    flow_frequency = 0; // Reset Counter
    Serial.println(litrePerHour);
    if (litrePerHour > 350)
      water_coming = true;
  }
  return water_coming;
}

void check_correct_time(int hr, int mi)
{
  if (hr == 3 and mi >= 40)
    timeInRange = true;
  else if (hr == 4)
    timeInRange = true;
  else if (hr == 5)
    timeInRange = true;
  else if (hr == 6 and mi < 20)
    timeInRange = true;
  else
    timeInRange = false;
}

void print_time()
{
  Serial.print(rtc.getDOWStr());
  Serial.print(" ");

  // Send date
  Serial.print(rtc.getDateStr());
  Serial.print(" -- ");

  // Send time
  Serial.println(rtc.getTimeStr());
}

void print_flags()
{
  Serial.println("------------------------");
  Serial.print("retryWaterAttempts");
  Serial.println(retryWaterAttempts);
  Serial.print("waterFillingStart");
  Serial.println(waterFillingStart);
  Serial.print("retryPending");
  Serial.println(retryPending);
  Serial.print("waterStatusAvg");
  Serial.println(waterStatusAvg);
  Serial.print("waterStatusCounter");
  Serial.println(waterStatusCounter);
  Serial.print("waterStatusFlag");
  Serial.println(waterStatusFlag);
  Serial.println("timeInRange");
  Serial.println(timeInRange);
}
