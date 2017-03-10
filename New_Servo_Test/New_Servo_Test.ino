int triggerPin = 10;
int selectPinA = 12;
int selectPinB = 11;
int powerPin = 6;

int buttonPin = 9;
int buttonPin2 = 5;

bool pushedFlag = false;

bool outA = false;
bool outB = false;

int counter = 0;

int wait = 7;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
  while(!Serial)

  pinMode(buttonPin, INPUT);
  pinMode(buttonPin2, OUTPUT);
  digitalWrite(buttonPin2, HIGH);

  pinMode(triggerPin, OUTPUT);
  pinMode(selectPinA, OUTPUT);
  pinMode(selectPinB, OUTPUT);

  pinMode(powerPin, OUTPUT);
  digitalWrite(powerPin, HIGH);

  digitalWrite(triggerPin, HIGH);
  
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  bool buttonIn = digitalRead(buttonPin);

 // if (!pushedFlag) {
    //pushedFlag = true;
/*
    if (counter < 4)
      counter ++;
    else
      counter = 0;

    if (counter < 2)
      outB = false;
    else
      outB = true;

    if (counter == 0 || counter == 2)
      outA = false;
    else
      outA = true;*/

    digitalWrite(selectPinA, LOW);
    digitalWrite(selectPinB, LOW);
    delay(100);
    digitalWrite(triggerPin, LOW); //LOW
    Serial.println("Pulsing servo");
    delay(wait);
    digitalWrite(triggerPin, HIGH);
    delay(1000);

    digitalWrite(selectPinA, LOW);
    digitalWrite(selectPinB, HIGH);
    delay(100);
    digitalWrite(triggerPin, LOW); //LOW
    Serial.println("Pulsing servo");
    delay(wait);
    digitalWrite(triggerPin, HIGH);
    delay(1000);

    digitalWrite(selectPinA, HIGH);
    digitalWrite(selectPinB, LOW);
    delay(100);
    digitalWrite(triggerPin, LOW); //LOW
    Serial.println("Pulsing servo");
    delay(wait);
    digitalWrite(triggerPin, HIGH);
    delay(1000);

    digitalWrite(selectPinA, HIGH);
    digitalWrite(selectPinB, HIGH);
    delay(100);
    digitalWrite(triggerPin, LOW); //LOW
    Serial.println("Pulsing servo");
    delay(wait);
    digitalWrite(triggerPin, HIGH);
    delay(1000);
    /*
    digitalWrite(selectPinA, HIGH);
    digitalWrite(selectPinB, LOW);
    delay(10000);
    digitalWrite(selectPinB, HIGH);
    digitalWrite(selectPinA, LOW);
    delay(10000);
    digitalWrite(selectPinA, HIGH);
    digitalWrite(selectPinB, HIGH);
    delay(10000);
    */
    
    
  }
 // }
/*
  if (!buttonIn)
    pushedFlag = false;

}*/
