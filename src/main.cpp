#include <Arduino.h>
#include <Zumo32U4.h>

Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motros;
Zumo32U4OLED display;
Zumo32U4Encoders encoder;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;

int mode = 0;

//
// linjefølger:
//

//millis for rundeteller
unsigned long currentmillis_round;  
unsigned long previusmilllis_round;
const int time_round = 100;

// millis for restart sving
unsigned long currentmillis_restart;  
unsigned long previusmilllis_restart;
const int time_restart = 200;


// array for position
unsigned int lineSensorValues[5];


int16_t position; // hovedposition fra 0 - 4000
float left_speed; // fart til venstrebelte
float right_speed; // fart til høyre belte
int16_t left_speed1; // verdi av fart med D
int16_t right_speed1; // verdi av fart med D

int16_t lastError; 
int16_t error; // lagre hvor langt unna man er streken

float integral; 

int rounds_N; // telle runder

float speed = 100.0; // standarfart

// 
// knapp modus:
// 

// case verdi i firkant modus.
int sq_mode = 0;

//case verdi snu 180 grader
int turning;

//telle verdier
float count_right;
float count_left;
float count_distance;

int reset; // setter en restart verdi

int buttonpress; // case verdi modus

// hindre debaunce
int prevButtonState = LOW;
int buttonState; 


// startfunksjon til bilen
void sensorStartupp(){
  display.setLayout11x4(); // skriftstørelse på skjerm
  delay(1000);
  for (int i = 0; i <= 140; i++){ //kalibrere sensorene til bilen
    lineSensors.calibrate();
    motros.setSpeeds(-200, 200);
  }
  motros.setSpeeds(0,0); // sette fart lik 0 så motor ikke blir ødelagt
  delay(150);
}


void setup() {
  lineSensors.initFiveSensors();
  sensorStartupp();
}

//
//linje følging:
//

// lese av positisonsverdi
void readValue(){
  //lineSensors.readCalibrated(lineSensorValues);
  position = lineSensors.readLine(lineSensorValues);
}

// styring med pid
void steeringPID(){

  left_speed = (map(position, 0, 4000, 0, 100))/100.0; // faktor til å gange fart
  right_speed = (map(position, 0, 4000, 100, 0))/100.0; 

  error = position - 2000; // hvor feil er bilen
  
  int16_t speedDifference = error * 0.25  + (error - lastError)* 3; // deriverte

  lastError = error; // husk hvor bilen var
  
  left_speed1 = (int16_t)speed*left_speed  + speedDifference*left_speed; //legger inn den deriverte
  right_speed1 = (int16_t)speed*right_speed - speedDifference*right_speed;
  
  left_speed1 = constrain(left_speed1, 0, (int16_t)speed); // hva skal bilen gjøre
  right_speed1 = constrain(right_speed1, 0, (int16_t)speed);

  motros.setSpeeds(left_speed1,right_speed1); // setter fart på motor
  //motros.setSpeeds(0,0);

}

// dårlig versjiion av pid men kan kanskje forbedres
void badPID(){

  error = position - 2000;
  
  int16_t speedDifference = error * 0.25  + (error - lastError)* 3 + integral * 0.001;

  integral += error;
  lastError = error;
  
  left_speed1 = (int16_t)speed + speedDifference;
  right_speed1 = (int16_t)speed - speedDifference;

  left_speed1 = constrain(left_speed1, 0, (int16_t)speed);
  right_speed1 = constrain(right_speed1, 0, (int16_t)speed);

  motros.setSpeeds(left_speed1,right_speed1);
}

// styring uten PID
void steeringStandar(){
  left_speed = (map(position, 0, 4000, 0, 100))/100.0;
  right_speed = (map(position, 0, 4000, 100, 0))/100.0;

    motros.setSpeeds(left_speed,right_speed);
}

//funksjon for rundetelling (teipet som kryss)
void rounds(){
  if (lineSensorValues[0] >= 1000 && lineSensorValues[4] >= 1000){
    currentmillis_round = millis();
    if (currentmillis_round - previusmilllis_round >= time_round){
      previusmilllis_round = currentmillis_round;
      rounds_N = rounds_N + 1;
    }
  }
}

//
// knapp modus;
//


// funksjon som teller hvor langt bilen har kjørt
void distance(){
  count_left = encoder.getCountsLeft();
  count_right = encoder.getCountsRight();
  count_distance = ((count_left + count_right)/2)/7625;  // 7625 er ca 1 m, man litt under da
}

// funksjon som restarter tellingen.
void resetdistance(){
   if (reset == 1){
    encoder.getCountsAndResetLeft();
    encoder.getCountsAndResetRight();

    count_left = encoder.getCountsLeft();
    count_right = encoder.getCountsRight();
    count_distance = ((count_left + count_right)/2)/7625; //legger inn count distance for å kunne starte på sq_mode 0.

    motros.setSpeeds(0,0);

    
    currentmillis_restart = millis(); // bruker millis istede for delay()
    if (currentmillis_restart - previusmilllis_restart >= time_restart){
      reset = 0;
    }
  }
}


// funskjon for bilen til å kjøre i firkanter
void square(){

  distance();

  switch (sq_mode)
  {

  case 0:
    resetdistance(); // henter inn resetdistance funksjonen

    if (reset == 0){
      motros.setSpeeds(speed,speed); // setter farten rett frem
    }

    if (count_distance >= 0.3){ // når bilen har kjørt en hvis lengde bytter den modus.
      sq_mode = 1;
      reset = 1;
      previusmilllis_restart = millis();
    }
    break;


case 1:

  resetdistance();  // henter inn resetdistance funksjonen
  if (reset == 0){
    motros.setSpeeds(speed*(-1), speed*(1)); // får bilen til å snu
  }

  if (count_right >= 600){ // når bilen har snudd 90 grader byttes modus, når farten er 100 snus den på 600
    sq_mode = 0;
    reset = 1;
    previusmilllis_restart = millis();
  }

  break;
}
}

// funksjon for sirkel
void circle(){
  resetdistance();
  motros.setSpeeds(speed*1, speed*2);
}

void turningopperation(){
    distance();

  switch (turning)
  {

  case 0:
    resetdistance(); // henter inn resetdistance funksjonen

    motros.setSpeeds(speed,speed); // setter farten rett frem

    if (count_distance >= 0.3){ // når bilen har kjørt en hvis lengde bytter den modus.
      turning = 1;
      reset = 1;
    }
    break;


case 1:

  resetdistance();  // henter inn resetdistance funksjonen

  motros.setSpeeds(speed*(-1), speed*(1)); // får bilen til å snu

  if (count_right >= 1200){ // når bilen har snudd 180 grader byttes modus, når farten er 100 snus den på 600
    turning = 0;
    reset = 1;
  }
  break;
}
}

// funksjon for hvilket modus bilen skal være i avhengig av knappetrykk.
void button_mode(){

  if (buttonA.isPressed()){ // kjøre i firkant, knapp A
    buttonpress = 1;
    reset = 1;
    sq_mode = 0;
  }
  else if (buttonB.isPressed()) // kjøre i sirkel, knapp B
  {
    buttonpress = 2;
    reset = 1;
  }
  else if (buttonC.isPressed()) // kjøre frem og tilbake, knapp C
  {
    buttonpress = 3;
    reset = 1;
    turning = 0;
  }
  
  
switch (buttonpress) // hvilken case avhenger av knappetrykk.
{
case 1:
  square();
  break;
case 2:
  circle();
  break;
case 3:
  turningopperation();
break;

default:
  break;
}
}

//
// skjerm;
//

// skjermviser
void show(){
  //display.gotoXY(0,0);
  //display.println(rounds_N);
  display.gotoXY(3,0);
  display.println(lineSensorValues[2]);
  display.gotoXY(5,1);  
  display.println(lineSensorValues[4]);
  display.gotoXY(0,1);
  display.println(lineSensorValues[0]);
  display.gotoXY(5,2);
  display.println(lineSensorValues[3]);
  display.gotoXY(0,2);
  display.println(lineSensorValues[1]);
  display.gotoXY(0,3);
  display.println(mode);

// knapp modus:

  //display.gotoXY(0,0);
  //display.println(count_distance);
  //display.gotoXY(0,1);
  //display.println(count_right);
  //display.gotoXY(0,2);
  //display.println(sq_mode);

}

//
// hovedmodus:
//



//  bytte hovedmodus
void change_mode(){

// hindre debaunce
buttonState = buttonA.isPressed();
  if (buttonState !=  prevButtonState){
  prevButtonState = buttonState;
    if (buttonState == HIGH) {

       if (buttonA.isPressed() and buttonB.isPressed()){ // hvis A og B trykkes samtidig skifter modus.
         if (mode == 0){
           mode = 1;
         }
         else if (mode ==1)
         {
           mode = 0;
         }
       }

    }
  }

}

void main_mode(){
  switch (mode)
  {
  case 0:
    show();
    readValue();
    steeringPID();
    rounds();
    change_mode();
    break;

  case 1:
    show();
    button_mode();
    change_mode();
    break;
  
  default:
    break;
}

}

void loop() {
 main_mode();
 }
