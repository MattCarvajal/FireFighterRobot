#include <Servo.h>

#define enFR 13 // Front-right wheel enable
#define inFR1 11  // Front-right wheel forward
#define inFR2 12 // Front-right wheel backward

#define enFL 8 // Front-left wheel enable
#define inFL1 9 // Front-left wheel forward
#define inFL2 10  // Front-left wheel backward

#define enRL 2 // Rear-left wheel enable
#define inRL1 3  // Rear-left wheel forward
#define inRL2 4  // Rear-left wheel backward

#define enRR 5 // Rear-right wheel enable
#define inRR1 6  // Rear-right wheel forward
#define inRR2 7  // Rear-right wheel backward

#define ir_R A0
#define ir_F A1
#define ir_L A3
int thresholdL = 15; // Custom threshold for A3 as it is giving a lower value
#define pump A5
#define servoPin A4 // Servo control pin

int Speed = 160;

//servo and sensor settings
Servo myServo;
int s1, s2, s3; //sensor values
int baselineR, baselineF, baselineL;

void setup() {
    Serial.begin(9600);

    //set pin modes for motors
    pinMode(enFL, OUTPUT);
    pinMode(inFL1, OUTPUT);
    pinMode(inFL2, OUTPUT);
    pinMode(enFR, OUTPUT);
    pinMode(inFR1, OUTPUT);
    pinMode(inFR2, OUTPUT);
    pinMode(enRL, OUTPUT);
    pinMode(inRL1, OUTPUT);
    pinMode(inRL2, OUTPUT);
    pinMode(enRR, OUTPUT);
    pinMode(inRR1, OUTPUT);
    pinMode(inRR2, OUTPUT);

    //set pin modes for sensors
    pinMode(ir_R, INPUT);
    pinMode(ir_F, INPUT);
    pinMode(ir_L, INPUT);
    pinMode(pump, OUTPUT);

    //attach servo
    myServo.attach(servoPin);

    //set initial motor speeds
    analogWrite(enFL, Speed);
    analogWrite(enFR, Speed);
    analogWrite(enRL, Speed);
    analogWrite(enRR, Speed);

    //set sensor baselines
    baselineR = analogRead(ir_R);
    baselineF = analogRead(ir_F);
    baselineL = analogRead(ir_L);

    //print baseline values for debugging if val less than 1000 ~ problem
    Serial.println("Baselines:");
    Serial.print("Right: ");
    Serial.println(baselineR);
    Serial.print("Front: ");
    Serial.println(baselineF);
    Serial.print("Left: ");
    Serial.println(baselineL);
}

void loop() {
    //read sensor values
    s1 = analogRead(ir_R); //right sensor
    s2 = analogRead(ir_F); //front sensor
    s3 = analogRead(ir_L); //left sensor

    //debugging output for flame
    Serial.print("Sensor Values - Right: ");
    Serial.print(s1);
    Serial.print("\tFront: ");
    Serial.print(s2);
    Serial.print("\tLeft: ");
    Serial.println(s3);

    //flame detection and movement logic
    if (isFireDetected(s2, baselineF)) { // Fire detected in front
        stopMotors();
        aimServo(90);       //aim forward
        approachFire();     //move closer dynamically
        activatePump(90);     //extinguish the fire
    } else if (isFireDetected(s1, baselineR)) { //fire detected on the right
        stopMotors();
        int dynamicAngle = calculateDynamicAngle(s1, baselineR, 90); //calculate the right angle dynamically
        aimServo(dynamicAngle);
        approachFire();     //move closer dynamically
        activatePump(dynamicAngle);     //extinguish the fire
    } else if (isFireDetected(s3, baselineL)) { //fire detected on the left
        stopMotors();
        int dynamicAngle = calculateDynamicAngle(s3, baselineL, 90); //calculate the left angle dynamically
        aimServo(dynamicAngle);
        approachFire();     //move closer dynamically
        activatePump(dynamicAngle);     //extinguish the fire
    } else {
        Serial.println("No fire detected. Stopping.");
        stopMotors(); //no fire detected
        resetServo();
    }
    delay(50);
}


//dynamically approach the fire
void approachFire() {
    Serial.println("Approaching fire dynamically.");
    while (true) {
        // continuously read sensor values
        s1 = analogRead(ir_R); //right sensor
        s2 = analogRead(ir_F); //front sensor
        s3 = analogRead(ir_L); //left sensor

        // Stop if fire is very close
        if (isCloseToFire(s2, baselineF)) {
            stopMotors();
            Serial.println("Fire is close. Stopping.");
            break;
        }

        //calculate proportional adjustments
        int deviationR = baselineR - s1;
        int deviationL = baselineL - s3;

        if (isFireDetected(s2, baselineF)) { 
            //move forward if fire is directly ahead
            moveForward();
        } else if (isFireDetected(s1, baselineR) && deviationR > deviationL) { 
            //adjust right dynamically based on sensor deviation
            int rightTurnIntensity = calculateTurnIntensity(deviationR, baselineR);
            turnRightDynamic(rightTurnIntensity);
        } else if (isFireDetected(s3, baselineL) && deviationL > deviationR) { 
            //adjust left dynamically based on sensor deviation
            int leftTurnIntensity = calculateTurnIntensity(deviationL, baselineL);
            turnLeftDynamic(leftTurnIntensity);
        } else { 
            //stop if no clear direction
            stopMotors();
            break;
        }

        //short delay for smoother movement
        delay(100);
    }
}


//check if the fire is close
bool isCloseToFire(int sensorValue, int baseline) {
    return sensorValue < (baseline - 900); //adjust threshold as needed for proximity
}


//omnidirectional movement functions
void moveForward() {
    Serial.println("Moving forward.");
    digitalWrite(inFL1, HIGH); digitalWrite(inFL2, LOW);
    digitalWrite(inFR1, HIGH); digitalWrite(inFR2, LOW);
    digitalWrite(inRL1, HIGH); digitalWrite(inRL2, LOW);
    digitalWrite(inRR1, HIGH); digitalWrite(inRR2, LOW);
}

void moveBackward() {
    Serial.println("Moving backward.");
    digitalWrite(inFL1, LOW); digitalWrite(inFL2, HIGH);
    digitalWrite(inFR1, LOW); digitalWrite(inFR2, HIGH);
    digitalWrite(inRL1, LOW); digitalWrite(inRL2, HIGH);
    digitalWrite(inRR1, LOW); digitalWrite(inRR2, HIGH);
}

void moveLeft() {
    Serial.println("Moving left.");
    digitalWrite(inFL1, LOW); digitalWrite(inFL2, HIGH);
    digitalWrite(inFR1, HIGH); digitalWrite(inFR2, LOW);
    digitalWrite(inRL1, HIGH); digitalWrite(inRL2, LOW);
    digitalWrite(inRR1, LOW); digitalWrite(inRR2, HIGH);
}

void moveRight() {
    Serial.println("Moving right.");
    digitalWrite(inFL1, HIGH); digitalWrite(inFL2, LOW);
    digitalWrite(inFR1, LOW); digitalWrite(inFR2, HIGH);
    digitalWrite(inRL1, LOW); digitalWrite(inRL2, HIGH);
    digitalWrite(inRR1, HIGH); digitalWrite(inRR2, LOW);
}

void stopMotors() {
    Serial.println("Stopping motors.");
    digitalWrite(inFL1, LOW); digitalWrite(inFL2, LOW);
    digitalWrite(inFR1, LOW); digitalWrite(inFR2, LOW);
    digitalWrite(inRL1, LOW); digitalWrite(inRL2, LOW);
    digitalWrite(inRR1, LOW); digitalWrite(inRR2, LOW);
}

void moveForwardRight() {
    Serial.println("Moving diagonally forward-right.");
    //activate only the appropriate motors for forward-right diagonal movement
    digitalWrite(inFL1, HIGH); digitalWrite(inFL2, LOW); //front-left forward
    digitalWrite(inRR1, HIGH); digitalWrite(inRR2, LOW); //rear-right forward
    digitalWrite(inFR1, LOW); digitalWrite(inFR2, LOW); //front-right stop
    digitalWrite(inRL1, LOW); digitalWrite(inRL2, LOW); //rear-left stop
}

void moveForwardLeft() {
    Serial.println("Moving diagonally forward-left.");
    digitalWrite(inFR1, HIGH); digitalWrite(inFR2, LOW); 
    digitalWrite(inRL1, HIGH); digitalWrite(inRL2, LOW); 
    digitalWrite(inFL1, LOW); digitalWrite(inFL2, LOW); 
    digitalWrite(inRR1, LOW); digitalWrite(inRR2, LOW); 
}


bool isFireDetected(int sensorValue, int baseline) {
    int threshold = (baseline == baselineL) ? thresholdL : 500; //custom threshold for left sensor
    bool detected = sensorValue < (baseline - threshold);

    //debugging output for fire detection
    Serial.print("Sensor Value: ");
    Serial.print(sensorValue);
    Serial.print(", Baseline: ");
    Serial.print(baseline);
    Serial.print(", Threshold: ");
    Serial.print(threshold);
    Serial.print(", Fire Detected: ");
    Serial.println(detected ? "YES" : "NO");

    return detected;
}


void activatePump(int currentAngle) {
    Serial.println("Activating pump.");
    digitalWrite(pump, HIGH);
    sprinkleMotion(currentAngle); 
    digitalWrite(pump, LOW);
}

void aimServo(int angle) {
    Serial.print("Aiming servo at ");
    Serial.print(angle);
    Serial.println(" degrees.");
    myServo.write(angle);
    delay(500);
}
void sprinkleMotion(int baseAngle) {
    int leftAngle = baseAngle - 15;   // Calculate left-most angle dynamically
    int rightAngle = baseAngle + 15; // Calculate right-most angle dynamically
    int delayTime = 100;             // Delay in milliseconds for smooth motion

    Serial.print("Starting sprinkler motion around angle: ");
    Serial.println(baseAngle);

    //perform one full sweep (left-right-left)
    for (int i = 0; i < 1; i++) { 
        //sweep from baseAngle - 15 to baseAngle + 15
        for (int angle = leftAngle; angle <= rightAngle; angle++) {
            myServo.write(angle);
            delay(delayTime);
        }
        //sweep back from baseAngle + 15 to baseAngle - 15
        for (int angle = rightAngle; angle >= leftAngle; angle--) {
            myServo.write(angle);
            delay(delayTime);
        }
    }

    Serial.println("Sprinkler motion complete.");
}
void resetServo() {
    Serial.println("Resetting servo to 90 degrees.");
    myServo.write(90); 
    delay(500);
}
int calculateDynamicAngle(int sensorValue, int baseline, int baseAngle) {
    //calculate the deviation from the baseline
    int deviation = baseline - sensorValue;
    int maxDeviation = (baseline == baselineL) ? 50 : 800; //adjust for the left sensor
    //map deviation to an angle adjustment (e.g., 0-45 degrees)
    //adjust the multiplier and maximum angle as needed
    int maxAdjustment = 45; //maximum angle adjustment
    int adjustment = map(deviation, 0, maxDeviation, 0, maxAdjustment); //map range dynamically

    //clamp the angle adjustment to avoid excessive turning
    adjustment = constrain(adjustment, 0, maxAdjustment);

    //calculate the final angle
    int dynamicAngle = baseAngle + adjustment;

    //debugging output
    Serial.print("Calculated Dynamic Angle: ");
    Serial.println(dynamicAngle);

    return dynamicAngle;
}
int calculateTurnIntensity(int deviation, int baseline) {
    //map deviation to a proportional motor adjustment range
    int maxIntensity = 255; //max motor speed
    int minIntensity = 100; //min motor speed for small adjustments
    int intensity = map(deviation, 0, 800, minIntensity, maxIntensity); //adjust range as needed
    intensity = constrain(intensity, minIntensity, maxIntensity); //constrain intensity

    //debugging output
    Serial.print("Calculated Turn Intensity: ");
    Serial.println(intensity);

    return intensity;
}
void turnRightDynamic(int intensity) {
    Serial.println("Turning right dynamically.");
    //set motor speeds proportionally for a smooth turn
    analogWrite(enFL, intensity);
    analogWrite(enRL, intensity);
    digitalWrite(inFL1, HIGH); digitalWrite(inFL2, LOW);
    digitalWrite(inRL1, HIGH); digitalWrite(inRL2, LOW);

    //reduce speed on the other side for turning
    analogWrite(enFR, intensity / 2);
    analogWrite(enRR, intensity / 2);
    digitalWrite(inFR1, LOW); digitalWrite(inFR2, HIGH);
    digitalWrite(inRR1, LOW); digitalWrite(inRR2, HIGH);
}
void turnLeftDynamic(int intensity) {
    Serial.println("Turning left dynamically.");
    //set motor speeds proportionally for a smooth turn
    analogWrite(enFR, intensity);
    analogWrite(enRR, intensity);
    digitalWrite(inFR1, HIGH); digitalWrite(inFR2, LOW);
    digitalWrite(inRR1, HIGH); digitalWrite(inRR2, LOW);

    //reduce speed on the other side for turning
    analogWrite(enFL, intensity / 2);
    analogWrite(enRL, intensity / 2);
    digitalWrite(inFL1, LOW); digitalWrite(inFL2, HIGH);
    digitalWrite(inRL1, LOW); digitalWrite(inRL2, HIGH);
}
