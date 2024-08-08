#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>//for the accelerometer
#include <U8g2lib.h> //for the esp32 display
#include "mynewmodel_data.h" // model data header file
#include <tflm_esp32.h>
#include <eloquent_tinyml.h>
//#define TF_NUM_OPS 2
#define ARENA_SIZE 3000 // Size is adjusted based on the size of  the model

Eloquent::TF::Sequential<TF_NUM_OPS, ARENA_SIZE> tf;
MPU6050 mpu;
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

void setup() {
    Serial.begin(9600);
    Wire.begin();
    mpu.initialize();
    u8g2.begin();
    
    tf.setNumInputs(3); // Set the number of inputs 
    tf.setNumOutputs(5); // set the output as 5 for the five activities

    
    // operations used in your model
    tf.resolver.AddFullyConnected();
    tf.resolver.AddSoftmax();


    while (!tf.begin(Accelerometer_model).isOk()){
        Serial.println(tf.exception.toString());
    }
}
//activity prediction
int predictActivity(float xAcc, float yAcc, float zAcc) {
    float input[3] = {xAcc, yAcc, zAcc};

    if (!tf.predict(input).isOk()) {
       Serial.println(tf.exception.toString());
       return -1;
    }
    
    Serial.print("predicted class: ");
    Serial.println(tf.classification); 
    return tf.classification;
    //returning the predicted class value

    
}

void loop() {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az); //obtaining accelerometer values

    float xAcc = (float)ax / 16384.0;
    float yAcc = (float)ay / 16384.0;
    float zAcc = (float)az / 16384.0;

    int activity = predictActivity(xAcc, yAcc, zAcc);


    // Display the result on the OLED
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 24, "Activity:");
    u8g2.drawStr(0, 36, getActivityName(activity)); 
    u8g2.sendBuffer();
   
   //for serial monitor display
    Serial.print("X-Acc: ");
    Serial.print(xAcc);
    Serial.print(" g, Y-Acc: ");
    Serial.print(yAcc);
    Serial.print(" g, Z-Acc: ");
    Serial.print(zAcc);
    Serial.println(" g");
    Serial.println("Predicted Activity: " + String(getActivityName(activity)));

    delay(1000);
}

// function to convert activity index to name
const char* getActivityName(int activity) {
    switch(activity) {
        case 0: return "Sleeping";
        case 1: return "Standing";
        case 2: return "Walking";
        case 3: return "Action without moving";
        case 4: return "walking down the stairs";
        default: return "Unknown";
    }
}
