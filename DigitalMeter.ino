#include "DFRobot_GDL.h"
#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <OneButton.h>

// Display PINS
#define TFT_DC 4
#define TFT_RST 3
#define TFT_CS 2
#define BUTTON_PIN 6

// Ultrasonic
#define MAX_RANGE 520
#define ADC_SOLUTION 1023.0

// Display COLORS
#define BACKGRD COLOR_RGB565_BLACK
#define FOREGRD COLOR_RGB565_WHITE
#define SUCCEC COLOR_RGB565_GREEN

// Display INIT
DFRobot_ST7789_240x320_HW_SPI screen(TFT_DC, TFT_CS, TFT_RST);
int text_size = 5;

// Accel Sensor
MPU6050 mpu;
bool sensorInit = false;
int16_t ax, ay, az;
double roll = 0.00, pitch = 0.00;

// Ultrasonic Sensor
int sonicPin = A3;

// Button
OneButton btn = OneButton(BUTTON_PIN, false);

enum MODES {
  VERTICAL = 0,
  HORIZONTAL = 1,
  DISTANCE = 2,
};

int mode = VERTICAL;

unsigned long lastTime = 0;
int updateTimer = 100;

void calibrateSensor(){
  int text_size = 3;
  screen.fillScreen(BACKGRD);
  screen.setTextSize(text_size);
  screen.setCursor((screen.width()/2) - ((5 * text_size) * 6), (screen.height()/2) - (8 * 2));
  screen.println("Calibrating.");
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  screen.fillScreen(BACKGRD);
}

void changeMode(){
  Serial.println("BUTTON CLICKED");
    if(mode == 2){
      mode = 0;
    }else{
      mode++;
    }
}

void setup(){
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(115200);
  btn.attachLongPressStop(calibrateSensor);
  btn.attachClick(changeMode);
  btn.setDebounceTicks(0);
  btn.setClickMs(10);
  btn.setLongPressIntervalMs(1000);
  screen.begin();
  screen.setRotation(3);
  mpu.initialize();
  sensorInit = mpu.testConnection();
  screen.fillScreen(BACKGRD);
}

void displayLevel(int angle, int x, int y, String unit, String mode){
  int text_x = 5 * text_size;
  int text_y = 8 * text_size;
  int spacing = 4;
  screen.setTextSize(text_size);
  if((angle >= 89 && angle <= 91) || angle <= 1 || angle >= 179){
    screen.setTextColor(SUCCEC, BACKGRD);
    screen.fillRect(x - (text_x + (text_x/2)), y + (text_y/2), (text_x + spacing) * 3, 5, SUCCEC);
  }else{
    screen.setTextColor(FOREGRD, BACKGRD);
    screen.fillRect(x - (text_x + (text_x/2)), y + (text_y/2), (text_x + spacing) * 3, 5, FOREGRD);
  }
  if(angle < 10){
    screen.fillRect(x - (text_x + (text_x/2)) + (text_x + spacing), y - (text_y/2), (text_x * 2) + 8, text_y, BACKGRD);
    screen.setCursor(x - (text_x + (text_x/2)), y - (text_y/2));
    screen.print(angle);
  }else if(angle < 100){
    screen.fillRect(x - (text_x + (text_x/2)) + (text_x + spacing) * 2, y - (text_y/2), text_x + (spacing * 3), text_y, BACKGRD);
    screen.setCursor(x - (text_x + (text_x/2)), y - (text_y/2));
    screen.print(angle);
  }
  screen.setCursor(x - (text_x + (text_x/2)), y - (text_y/2));
  screen.print(angle);
  screen.setTextSize(2);
  screen.setCursor(x - (text_x + (text_x/2)) + ((text_x + spacing) * 3) +  10, y + (text_y/2));
  screen.print(unit);
  screen.setCursor(20, 20);
  screen.print("Mode: ");
  screen.print(mode);
}

float sensity_t, dist_t;
void loop(){  if(!sensorInit) return;
  Serial.println(digitalRead(BUTTON_PIN));
  lastTime = millis();

  // MPU6050 Processing
  mpu.getAcceleration(&ax, &ay, &az);
  pitch = ax/182.04;
  roll = az/182.04;
  
  // Ultrasonic Sensor
  sensity_t = analogRead(sonicPin);
  dist_t = sensity_t * MAX_RANGE / ADC_SOLUTION;
  int distance = dist_t;

  // Update Data
  if(millis() < lastTime + updateTimer ){
    if(mode == VERTICAL){
      int angle = floor(abs(pitch));
      displayLevel(angle, (screen.width()/2), (screen.height()/2), "DEG", "VER");
    }
    if(mode == HORIZONTAL){
      int angle = floor(abs(roll - 90));
      displayLevel(angle, (screen.width()/2), (screen.height()/2), "DEG", "HOR");
    }
    if(mode == DISTANCE){
      displayLevel(distance, (screen.width()/2), (screen.height()/2), "CEN", "DIS");
    }
  }

  btn.tick();
}