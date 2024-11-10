/*
  Project: UGV ROS Node for Position, Distance, and Motor Control
  Description: This code controls a UGV equipped with an MPU6050 IMU, HC-SR04 ultrasonic sensors, 
               and motor driver. It uses the HC-05 Bluetooth module for ROS communication.

  HC-05 Serial Bluetooth Module:
    - Baud rate: 9600
    - Connected via Serial2 to Arduino

  ROS Topics:
    Publishers:
      - "front_distance" : Publishes the front ultrasonic sensor distance (std_msgs/Float32)
      - "back_distance" : Publishes the back ultrasonic sensor distance (std_msgs/Float32)
      - "position" : Publishes the UGV position as a Vector3 (geometry_msgs/Vector3) with x = posX, y = posY, z = yaw

    Subscribers:
      - "motor_instructions_left" : Receives speed control data for the left motor (std_msgs/Int32)
      - "motor_instructions_right" : Receives speed control data for the right motor (std_msgs/Int32)
*/

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include <Wire.h>
#include <MPU6050.h>

// Motor driver pins
const int ENA = 9;
const int IN1 = 8;
const int IN2 = 7;
const int ENB = 3;
const int IN3 = 5;
const int IN4 = 4;

// HC-SR04 pins
const int trigPinFront = 12;
const int echoPinFront = 11;
const int trigPinBack = 10;
const int echoPinBack = 6;

// IMU sensor
MPU6050 mpu;

// Bluetooth module
HardwareSerial &btSerial = Serial2;

ros::NodeHandle nh;

// ROS messages
std_msgs::Float32 front_distance_msg;
std_msgs::Float32 back_distance_msg;
geometry_msgs::Vector3 position_msg;

// ROS publishers
ros::Publisher front_distance_pub("front_distance", &front_distance_msg);
ros::Publisher back_distance_pub("back_distance", &back_distance_msg);
ros::Publisher position_pub("position", &position_msg);

void setMotorSpeed(int motor, int speed) {
	if (motor == 0) { // Left motor
		analogWrite(ENA, abs(speed));
		if (speed > 0) {
			digitalWrite(IN1, HIGH);
			digitalWrite(IN2, LOW);
		} else {
			digitalWrite(IN1, LOW);
			digitalWrite(IN2, HIGH);
		}
	} else if (motor == 1) { // Right motor
		analogWrite(ENB, abs(speed));
		if (speed > 0) {
			digitalWrite(IN3, HIGH);
			digitalWrite(IN4, LOW);
		} else {
			digitalWrite(IN3, LOW);
			digitalWrite(IN4, HIGH);
		}
	}
}

void motorInstructionsLeftCallback(const std_msgs::Int32& msg) {
	setMotorSpeed(0, msg.data);
}

void motorInstructionsRightCallback(const std_msgs::Int32& msg) {
	setMotorSpeed(1, msg.data);
}

ros::Subscriber<std_msgs::Int32> sub_left("motor_instructions_left", motorInstructionsLeftCallback);
ros::Subscriber<std_msgs::Int32> sub_right("motor_instructions_right", motorInstructionsRightCallback);

void setup() {
	// Initialize Bluetooth serial communication
	btSerial.begin(9600);
	nh.getHardware()->setHardwareSerial(&btSerial);
	nh.initNode();
	nh.subscribe(sub_left);
	nh.subscribe(sub_right);
	nh.advertise(front_distance_pub);
	nh.advertise(back_distance_pub);
	nh.advertise(position_pub);

	pinMode(ENA, OUTPUT);
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(ENB, OUTPUT);
	pinMode(IN3, OUTPUT);
	pinMode(IN4, OUTPUT);

	pinMode(trigPinFront, OUTPUT);
	pinMode(echoPinFront, INPUT);
	pinMode(trigPinBack, OUTPUT);
	pinMode(echoPinBack, INPUT);

	Wire.begin();
	mpu.initialize();
}

// Variables to store position data
float posX = 0.0;
float posY = 0.0;
float yaw = 0.0;

// Variables for IMU data
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Function to update position based on IMU data
void updatePosition() {
	// Retrieve raw IMU data
	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

	// Example integration to update position and yaw (placeholder logic)
	posX += ax * 0.0001; // Integration step for X position
	posY += ay * 0.0001; // Integration step for Y position
	yaw += gz * 0.0001; // Integration step for yaw
}

void loop() {
	// Publish front distance
	digitalWrite(trigPinFront, LOW);
	delayMicroseconds(2);
	digitalWrite(trigPinFront, HIGH);
	delayMicroseconds(10);
	digitalWrite(trigPinFront, LOW);
	float durationFront = pulseIn(echoPinFront, HIGH);
	front_distance_msg.data = (durationFront * 0.034) / 2;
	front_distance_pub.publish(&front_distance_msg);

	// Publish back distance
	digitalWrite(trigPinBack, LOW);
	delayMicroseconds(2);
	digitalWrite(trigPinBack, HIGH);
	delayMicroseconds(10);
	digitalWrite(trigPinBack, LOW);
	float durationBack = pulseIn(echoPinBack, HIGH);
	back_distance_msg.data = (durationBack * 0.034) / 2;
	back_distance_pub.publish(&back_distance_msg);

	// Update and publish position data
	updatePosition();
	position_msg.x = posX;
	position_msg.y = posY;
	position_msg.z = yaw; // Use z for yaw
	position_pub.publish(&position_msg);

	nh.spinOnce();
	delay(100);
}
