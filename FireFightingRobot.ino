#include <Servo.h>

// Pin definitions
#define enA 10 //Enable1 L298 Pin enA
#define in1 9  //Motor1 L298 Pin in1
#define in2 8  //Motor1 L298 Pin in2
#define in3 7  //Motor2 L298 Pin in3
#define in4 6  //Motor2 L298 Pin in4
#define enB 5  //Enable2 L298 Pin enB
#define ir_R A0
#define ir_F A1
#define ir_L A2
#define pump A5
int Speed = 160; // Write the duty cycle (0 to 255) for motor speed

// Servo and sensor settings
Servo myServo;
int s1, s2, s3;
int baselineR, baselineF, baselineL;

void setup() {
    // Start serial communication for debugging
    Serial.begin(9600);

    // Set pin modes
    pinMode(ir_R, INPUT);
    pinMode(ir_F, INPUT);
    pinMode(ir_L, INPUT);
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(pump, OUTPUT);
    
    // Attach servo to pin A4
    myServo.attach(A4);
    
    // Initialize motor speed
    analogWrite(enA, Speed); // Motor 1 speed
    analogWrite(enB, Speed); // Motor 2 speed
    
    // Initial servo sweep for calibration
    for (int angle = 90; angle <= 140; angle += 5) {
        myServo.write(angle);
        delay(50);
    }
    for (int angle = 140; angle >= 40; angle -= 5) {
        myServo.write(angle);
        delay(50);
    }
    for (int angle = 40; angle <= 95; angle += 5) {
        myServo.write(angle);
        delay(50);
    }

    // Set sensor baseline values
    baselineR = analogRead(ir_R);
    baselineF = analogRead(ir_F);
    baselineL = analogRead(ir_L);
}

void loop() {
    // Read sensor values
    s1 = analogRead(ir_R);
    s2 = analogRead(ir_F);
    s3 = analogRead(ir_L);

    // Print sensor values for debugging
    Serial.print(s1);
    Serial.print("\t");
    Serial.print(s2);
    Serial.print("\t");
    Serial.println(s3);
    
    // Control logic
    if (isFireDetected(s1, baselineR)) {
        stopMotors();
        activatePump();
        aimServo(40); // Right side
    } 
    else if (isFireDetected(s2, baselineF)) {
        stopMotors();
        activatePump();
        aimServo(90); // Front
    } 
    else if (isFireDetected(s3, baselineL)) {
        stopMotors();
        activatePump();
        aimServo(140); // Left side
    } 
    else if (s1 >= baselineR + 50 && s1 <= 700) {
        digitalWrite(pump, LOW);
        moveBackward();
        delay(100);
        turnRight();
        delay(200);
    } 
    else if (s2 >= baselineF + 50 && s2 <= 800) {
        digitalWrite(pump, LOW);
        moveForward();
    } 
    else if (s3 >= baselineL + 50 && s3 <= 700) {
        digitalWrite(pump, LOW);
        moveBackward();
        delay(100);
        turnLeft();
        delay(200);
    } 
    else {
        digitalWrite(pump, LOW);
        stopMotors();
    }

    delay(50); // Small delay for loop stability
}

bool isFireDetected(int sensorValue, int baseline) {
    return (sensorValue < baseline - 50);
}

void activatePump() {
    digitalWrite(pump, HIGH);
    delay(3000); // Run pump for 3 seconds
    digitalWrite(pump, LOW);
}

void aimServo(int angle) {
    myServo.write(angle); // Move servo to the specified angle
    delay(500); // Wait for the servo to reach the position
}

void moveForward() {
    digitalWrite(in1, HIGH); // Right motor forward
    digitalWrite(in2, LOW);  // Right motor backward
    digitalWrite(in3, LOW);  // Left motor backward
    digitalWrite(in4, HIGH); // Left motor forward
    delay(10);
}

void moveBackward() {
    digitalWrite(in1, LOW);  // Right motor forward
    digitalWrite(in2, HIGH); // Right motor backward
    digitalWrite(in3, HIGH); // Left motor backward
    digitalWrite(in4, LOW);  // Left motor forward
    delay(10);
}

void turnRight() {
    digitalWrite(in1, LOW);  // Right motor forward
    digitalWrite(in2, HIGH); // Right motor backward
    digitalWrite(in3, LOW);  // Left motor backward
    digitalWrite(in4, HIGH); // Left motor forward
    delay(10);
}

void turnLeft() {
    digitalWrite(in1, HIGH); // Right motor forward
    digitalWrite(in2, LOW);  // Right motor backward
    digitalWrite(in3, HIGH); // Left motor backward
    digitalWrite(in4, LOW);  // Left motor forward
    delay(10);
}

void stopMotors() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    delay(10);
}


