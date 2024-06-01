/*
    Final code

    Ultrasonic
    Lcm: 16
    Fcm: 14
    Rcm: 16

    P: 3
    I: 0
    D: 1

    Ultrasonic Sampling Rate: 20 Hz

*/



// ===== PINS ===== //
int R1 = 2;
int R2 = 3;
int L1 = 4;
int L2 = 5;
int RE = 9;
int LE = 10;

int Ftrig = 7;
int Fecho = A2;
int Ltrig = 8; 
int Lecho = A0;
int Rtrig = 6;
int Recho = A3;

int S0 = 11;
int S1 = 12;
int S2 = A1;
int S3 = A5;
int sensorOut = A4;
//==============================//



// ===== CLASSES, OBJECTS, VARS ===== //
int what;
int wall = 0;
char whatArray[7][22] = {
    "DEFAULT",
    "FRONT WALL LEFT OPEN",
    "FRONT WALL RIGHT OPEN",
    "FRONT OPEN LEFT FAR",
    "FRONT OPEN LEFT NEAR",
    "FRONT OPEN LEFT IN",
    "FRONT OPEN LEFT OPEN",
};
char wallArray[3][6] = {
    "None",
    "Left",
    "Right",
};
char colorArray[6][6] = {
    "None",
    "RED",
    "GREEN",
    "BLUE",
    "WHITE",
    "BLACK",
};
//==============================//



// ===== MEASUREMENTS VARS ===== //
int Fcm = 0;
int Lcm = 0;
int Rcm = 0;
int LcurrentSpeed, RcurrentSpeed;

bool onLeftWall = false;
bool onRightWall = false;
bool initLoop = false;
int rightTurnCounter = 0;
int leftTurnCounter = 0;

unsigned long currentTimeSensor;
unsigned long ledTime;

int redRead, greenRead, blueRead;
int color = 0;
bool stopOnRed = true;
//==============================//



// ===== UTILS ===== //
int speed(int val){
    return map(val, 0, 100, 0, 255);
}

void printAll(){
    Serial.print(Lcm);
    Serial.print("\t");

    Serial.print(Fcm);
    Serial.print("\t");

    Serial.print(Rcm);
    Serial.print("\t");

    Serial.print(LcurrentSpeed);
    Serial.print("\t");

    Serial.print(RcurrentSpeed);
    Serial.print("\t");

    Serial.print(whatArray[what]);
    Serial.print("\t");

    Serial.print(wallArray[wall]);
    Serial.print("\t");

    Serial.print(rightTurnCounter);
    Serial.print("\t");

    Serial.print(leftTurnCounter);
    Serial.print("\t");

    Serial.print(colorArray[color]);
    Serial.print("\t");
    
    Serial.print("\n");
}
//==============================//



// ===== SENSOR & MOTOR ===== //
float sensorRun(int trigPin, int echoPin){
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);   

    long duration = pulseIn(echoPin, HIGH/*, 5000*/);
    return (duration / 2) / 29.1;
}

int colorSensor(int opt){
    int freq, component;
    switch (opt){
        case 1:
            // Filter for red and read
            digitalWrite(S2, LOW);
            digitalWrite(S3, LOW);
            freq = pulseIn(sensorOut, LOW);
            component = map(freq, 50, 720, 255, 0);
            break;
        case 2:
            // Filter for green and read
            digitalWrite(S2, HIGH);
            digitalWrite(S3, HIGH);
            freq = pulseIn(sensorOut, LOW);
            component = map(freq, 60, 670, 255, 0);
            break;
        case 3:
            // Filter for blue and read
            digitalWrite(S2, LOW);
            digitalWrite(S3, HIGH);
            freq = pulseIn(sensorOut, LOW);
            component = map(freq, 40, 720, 255, 0);
            break;
        default:
            break;
    }
    return component;
}

void asyncSensor(){
    currentTimeSensor = millis();

    Fcm = sensorRun(Ftrig, Fecho);
    Lcm = sensorRun(Ltrig, Lecho);
    Rcm = sensorRun(Rtrig, Recho);

}

void motorRun(int Lnum, int Rnum){
    analogWrite(LE, speed(abs(Lnum)));
    LcurrentSpeed = Lnum;

    if(Lnum > 0){
        digitalWrite(L1, HIGH);
        digitalWrite(L2, LOW);
    } else if(Lnum < 0){
        digitalWrite(L1, LOW);
        digitalWrite(L2, HIGH);
    } else {
        digitalWrite(L1, LOW);
        digitalWrite(L1, LOW);
    }

    analogWrite(RE, speed(abs(Rnum)));
    RcurrentSpeed = Rnum;

    if(Rnum > 0){
        digitalWrite(R1, HIGH);
        digitalWrite(R2, LOW);
    } else if(Rnum < 0){
        digitalWrite(R1, LOW);
        digitalWrite(R2, HIGH);
    } else {
        digitalWrite(R1, LOW);
        digitalWrite(R2, LOW);
    }
}
//==============================//



// ===== PID ===== //
int Kp = 3;
int Ki = 0;
int Kd = 1;

int Lsetval = 16;
int Rsetval = 16;
int Lerror;
int Rerror;
int LPrevError = 0;
int RPrevError = 0;

int i_error = 0;
int d_error = 0;

int pidRun(int error, int prevError){
    i_error += error;
    d_error = error - prevError;
    int out = Kp*error + (Ki*i_error) + (Kd*d_error);

    if(out > 25){
        out = 25;
    } else if(out < -25){
        out = -25;
    }
    return out;
}
//==============================//



// ===== LEFT RIGHT WALL FOLLOWING ===== //
void leftWallFollowing(){
    leftTurnCounter = 0;

    onRightWall = false;
    onLeftWall =  true;
    wall = 1;

    if(color == 5){
        rightTurnCounter = 0;
    }

    if(Fcm > 14){
        if(Lcm <= 25){
            what = 5;
            motorRun(50-pidRun(Lerror, LPrevError), 50+pidRun(Lerror, LPrevError));
        } else if(Lcm > 25){ 
            what = 6;
            motorRun(25, 100);
        }
    } else if(Fcm <= 14){
        if(Rcm > 25){
            what = 2;
            motorRun(25, -25);
            delay(1000);
            rightTurnCounter++;
        } else if(Lcm > 25){
            what = 1;
            motorRun(-25, 25);
            delay(1000);
        } else{
            motorRun(0, 0);
        }
    }
}

void rightWallFollowing(){
    rightTurnCounter = 0;

    onRightWall = true;
    onLeftWall = false;
    wall = 2;

    if(color == 5){
        leftTurnCounter = 0;
    }

    if(Fcm > 14){
        if(Rcm <= 25){
            motorRun(50+pidRun(Rerror, RPrevError), 50-pidRun(Rerror, RPrevError));
        } else if(Rcm > 25){
            motorRun(100, 25);
        }
    } else if(Fcm <= 14){
        if(Lcm > 25){
            motorRun(-25, 25);
            delay(1000);
            leftTurnCounter++;
        } else if(Rcm > 25){
            motorRun(25, -25);
            delay(1000);
        } else{
            motorRun(0, 0);
        }
    }
}
//==============================//



// ===== SETUP & LOOP ===== //
void setup(){
    Serial.begin(9600);
    
    pinMode(R1, OUTPUT);
    pinMode(R2, OUTPUT);
    pinMode(L1, OUTPUT);
    pinMode(L2, OUTPUT);

    pinMode(Ftrig, OUTPUT);
    pinMode(Ltrig, OUTPUT);
    pinMode(Rtrig, OUTPUT);

    pinMode(Fecho, INPUT);
    pinMode(Lecho, INPUT);
    pinMode(Lecho, INPUT);

    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
    pinMode(sensorOut, INPUT);

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    delay(500);
    digitalWrite(13, LOW);
    delay(50);
    digitalWrite(13, HIGH);
    delay(50);
    digitalWrite(13, LOW);
    delay(50);
    digitalWrite(13, HIGH);
    delay(50);
    digitalWrite(13, LOW);
    delay(50);
    digitalWrite(13, HIGH);
    delay(50);
    digitalWrite(13, LOW);

    digitalWrite(S0, HIGH);
    digitalWrite(S1, LOW);
}

void loop(){
    if((rightTurnCounter > 0) || (leftTurnCounter > 0)){
        digitalWrite(13, HIGH);
    } else{
        digitalWrite(13, LOW);
    }

    if(!initLoop){
        Lcm = sensorRun(Ltrig, Lecho);
        Fcm = sensorRun(Ftrig, Fecho);
        Rcm = sensorRun(Rtrig, Recho);

        redRead = colorSensor(1);
        greenRead = colorSensor(2);
        blueRead = colorSensor(3);

        initLoop = true;
    }
    else {
        if(millis() > currentTimeSensor + 50){
            asyncSensor();
        }

        redRead = colorSensor(1);
        greenRead = colorSensor(2);
        blueRead = colorSensor(3);

        if((redRead >= 210) && (greenRead >= 210) && (blueRead >= 210)){
            color = 4;
        } else if(( ((redRead - greenRead) >= 40) && ((redRead - blueRead) >= 40) )){
            color = 1;
        } else if((redRead < 180) && (greenRead < 180) && (blueRead < 180)){
            color = 5;
        } else{
            color = 0;
        }

        if((color == 1) && stopOnRed){
            motorRun(0, 0);
            delay(10000);
            stopOnRed = false;
        } else if((color == 4) && !stopOnRed){
            stopOnRed = true;
        }

        LPrevError = Lerror;\]
        Lerror = Lcm - Lsetval;
        RPrevError = Rerror;
        Rerror = Rcm - Rsetval;

        if(!onLeftWall && !onRightWall){
            motorRun(50, 50); 
            if(Fcm <= 14){
                motorRun(25, -25);
                delay(1000);
                onLeftWall = true;
            } else if((Lcm <= 20) && leftTurnCounter < 3){
                leftWallFollowing();
            } else if((Rcm <= 25) && leftTurnCounter < 3){
                motorRun(25, -25);
                delay(1000);
            }
        }
        else {
            if((rightTurnCounter >= 2) && Rcm < 20){
                rightWallFollowing(); 
            } else {
                if(onRightWall){
                    if(leftTurnCounter >= 3){
                        onRightWall = false;
                        onLeftWall = false;
                    } else {
                        rightWallFollowing(); 
                    }
                } else if(onLeftWall) {
                    leftWallFollowing();
                }
            }
        }
    }

    printAll();
}
//==============================//