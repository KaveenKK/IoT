
#include <Arduino.h>
#include <BluetoothSerial.h>  // library for bluetooth connection
#include <ThreeWire.h>
#include <Wire.h>
#include <MPU6050.h>
#include "SleepStage_model.h" // model data header file
#include <RtcDS1302.h> //library for clock module
#include <U8g2lib.h>// U8g2 library for display control
#include <tflm_esp32.h>
#include <eloquent_tinyml.h>   //ml model
#define ARENA_SIZE 5000 //  size of  the model

Eloquent::TF::Sequential<TF_NUM_OPS, ARENA_SIZE> tf;


BluetoothSerial ESP_BT;
const int ACCELEROMETER_POWER_PIN = 18; // Pin to control power to accelerometer

const int GROUND_PIN = 19; // Ground pin connected to the accelerometer

//accerometer
//SDA=d21
//scl =d22
MPU6050 mpu;
//clock module
ThreeWire myWire(25,27,26); // DAT, CLK, RST
RtcDS1302<ThreeWire> Rtc(myWire);

const int MPU_addr = 0x68; // I2C address of the MPU6050
int BluetReceive;



// Define GPIO pins for vibrating motor
const int VIBRATING_MOTOR_PLUS_PIN = 2;
const int VIBRATING_MOTOR_MINUS_PIN = 4;



// OLED Display
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// Variables to store alarm time received from Bluetooth
int receivedAlarmHour = 0;
int receivedAlarmMinute = 0;
int receivedAlarmSecond = 0;

// Global variables to store the previous accelerometer readings
float prev_xAcc = 0;
float prev_yAcc = 0; 
float prev_zAcc = 0;


bool alarmTriggered = false;


//to represent time with 2 digits
String formatDigits(int digits) {
  if (digits < 10) {
    return "0" + String(digits);
  } else {
    return String(digits);
  }
}


const static char* WeekDays[] =
{
    "Monday",
    "Tuesday",
    "Wednesday",
    "Thursday",
    "Friday",
    "Saturday",
    "Sunday"
};

void setup() 
{
    Serial.begin(9600);
    Wire.begin();
    pinMode(ACCELEROMETER_POWER_PIN, OUTPUT);
    u8g2.begin();
    mpu.initialize();
    ESP_BT.begin("Smart Alarm");

    tf.setNumInputs(3); // Set the number of inputs 
    tf.setNumOutputs(2);

    // operations used in the model
    tf.resolver.AddFullyConnected();
    tf.resolver.AddSoftmax();  
    while (!tf.begin(SleepStage_model).isOk()){
        Serial.println(tf.exception.toString());
    }
    
    Serial.print("compiled: ");
    Serial.print(__DATE__);
    Serial.println(__TIME__);

    pinMode(VIBRATING_MOTOR_PLUS_PIN, OUTPUT);
    pinMode(VIBRATING_MOTOR_MINUS_PIN, OUTPUT);

    Rtc.Begin();

    RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
    printDateTime(compiled);
    Serial.println();

    
}

//activity prediction
int predictSleepStage(float delta_xAcc, float delta_yAcc, float delta_zAcc) {
    float input[3] = {delta_xAcc, delta_yAcc, delta_zAcc};

    if (!tf.predict(input).isOk()) {
       Serial.println(tf.exception.toString());
       return -1;
    }
    
    
    Serial.print("predicted class: ");
    Serial.println(tf.classification); 
    return tf.classification;
    //returning the predicted class value
   
}
char datestring[26]; //defining it globally outside the funcyion

void loop()
{
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az); //obtaining accelerometer values  

  // Convert raw accelerometer values to G-force values
    float xAcc = (float)ax / 16384.0;
    float yAcc = (float)ay / 16384.0;
    float zAcc = (float)az / 16384.0;

    // Calculate the difference between the current and previous readings
    float delta_xAcc = xAcc - prev_xAcc;
    float delta_yAcc = yAcc - prev_yAcc;
    float delta_zAcc = zAcc - prev_zAcc;

    // Update the previous readings 
    prev_xAcc = xAcc;
    prev_yAcc = yAcc;
    prev_zAcc = zAcc;

    int SleepStage = predictSleepStage(delta_xAcc, delta_yAcc, delta_zAcc); 

    if (ESP_BT.available())
    {
     BluetReceive = ESP_BT.read(); //reading the received data
     String message = ESP_BT.readString();
     Serial.print("recieved data: ");
     Serial.println(message);
     // Extract values directly and assign to variables
     receivedAlarmHour = message.substring(0, message.indexOf(',')).toInt();
     receivedAlarmMinute = message.substring(message.indexOf(',') + 1, message.indexOf(',', message.indexOf(',') + 1)).toInt();
    }
    
    
    // Get current time
    RtcDateTime now = Rtc.GetDateTime();
  
    printDateTime(now);
  
    // Print accelerometer data, sleep stages, and real-time to the OLED display
    u8g2.clearBuffer(); // Clear the display buffer
    u8g2.setFont(u8g2_font_ncenB08_tr); // Choosing a smaller font size 
    u8g2.drawStr(0, 24, "Smart Alarm");
    u8g2.setCursor(10, 45); // Set the cursor position for real-time
    u8g2.print(datestring);
    u8g2.sendBuffer();//sending to the display


  
  // Check if it's current time is equal or greater than received alarm time(keep the alarm on for 10 mins from the alarm time) 
  if (now.Hour() == receivedAlarmHour && now.Minute() >= receivedAlarmMinute && now.Minute() < receivedAlarmMinute + 10) {
     if (SleepStage == 1){
    
         // Power on the motor
         digitalWrite(VIBRATING_MOTOR_PLUS_PIN, HIGH);
         digitalWrite(VIBRATING_MOTOR_MINUS_PIN, LOW);
         // Wait for  (10 seconds)
         delay(10000);

         // Deactivate vibrating motor
         digitalWrite(VIBRATING_MOTOR_PLUS_PIN, LOW);
         digitalWrite(VIBRATING_MOTOR_MINUS_PIN, LOW);
      }  else {
          // keep the vibrating motor deactivated
           digitalWrite(VIBRATING_MOTOR_PLUS_PIN, LOW);
           digitalWrite(VIBRATING_MOTOR_MINUS_PIN, LOW);
    }

  }

    Serial.println("Predicted Stage: " + String(getSleepStage(SleepStage)));
    delay(1000);
}


// convert sleep stage index to name
const char* getSleepStage(int SleepStage) {
    switch(SleepStage) {
        case 0: return "Awake";
        case 1: return "Sleeping";
        default: return "Unknown";
    }
}



#define countof(a) (sizeof(a) / sizeof(a[0]))

void printDateTime(const RtcDateTime& dt)
{
    
    snprintf_P(datestring, 
            countof(datestring),
            PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
            dt.Month(),
            dt.Day(),
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    Serial.print(datestring);
}


void activateVibratingMotor() {
    // Activate vibrating motor
    digitalWrite(VIBRATING_MOTOR_PLUS_PIN, HIGH);
    digitalWrite(VIBRATING_MOTOR_MINUS_PIN, LOW);

    // Wait for 1 minute (10 seconds)
    delay(10000);

    // Deactivate vibrating motor
    digitalWrite(VIBRATING_MOTOR_PLUS_PIN, LOW);
    digitalWrite(VIBRATING_MOTOR_MINUS_PIN, LOW);
}
