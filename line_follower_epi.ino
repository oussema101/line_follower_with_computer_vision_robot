#include<NewPing.h>
#include <QTRSensors.h>
#include <LiquidCrystal_I2C.h>


//#define TEST_QTR

#define Kp        0.04
#define Kd        0.12
#define  MaxSpeed   120
#define normalSpeed    80
#define NUM_SENSORS  16
#define TIMEOUT      3000
//#define QTR_NO_EMITTER_PIN
//#define EMITTER_PIN

// maze


#define Kp2 1.8
#define Kd2 0.1
#define MaxSpeed2 85
#define MinSpeed2 0

#define echo1  8
#define trig1  9
#define echo2  10
#define trig2  11
#define echo3  13
#define trig3  12
NewPing UltraSon1(trig1, echo1, 30);
NewPing UltraSon2(trig2, echo2, 30);
NewPing UltraSon3(trig3, echo3, 30);


#define rightMotor1 6
#define rightMotor2 5
#define leftMotor1 4
#define leftMotor2  3
#define rightMotorPWM 7
#define leftMotorPWM 2

String sub="";
int longueur ;
String copie="";
LiquidCrystal_I2C lcd(0x27, 16, 2);
QTRSensorsRC qtrrc((unsigned char[]) {
  A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15
} , NUM_SENSORS, TIMEOUT, QTR_NO_EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];
unsigned int sensors[16];
unsigned int distance1, distance2, distance3;
unsigned int mesureDist();

int longueur2;
String sub2="";


int PWMMoteurDroite;
int PWMMoteurGauche;
unsigned int distanceA1, distanceA2, distanceA3, distanceB1, distanceB2,
         distanceB3, distanceC1, distanceC2, distanceC3;
byte d = 0 , g = 0, u = 0;
bool Activate1 = true, Activate2 = false ;
bool Pn = false ;
int noir = 600;
int blanc = 400;
int l = 0;
String receivedChar= "", cc = "";
String receivedChar2= "", cc2 = "";

void setup()
{
  receivedChar == "", cc == "";
  //capteur avant
  pinMode(trig1, OUTPUT);
  digitalWrite(trig1, LOW);
  pinMode(echo1, INPUT);
  //capteur gauche 1
  pinMode(trig2, OUTPUT);
  digitalWrite(trig2, LOW);
  pinMode(echo2, INPUT);
  //capteur gauche 2
  pinMode(trig3, OUTPUT);
  digitalWrite(trig3, LOW);
  pinMode(echo3, INPUT);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("El 3ou9");

  lcd.setCursor(2, 1);
  lcd.print("is here");

  Serial.begin(9600);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(53, INPUT_PULLUP);
  delay(1000);

  int i;
  for (int i = 0; i < 150; i++)
  {
    qtrrc.calibrate();
    delay(20);
  }
  /*
    wait();
    delay(2000);
    /*/

  Serial.begin(9600);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  for (i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();


  while (!digitalRead(53))
  {
    lcd.clear();
    lcd.setCursor(2, 0);
    lcd.print(" GO!  ");
    stop();
    delay(50);
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("1:");
  lcd.setCursor(0, 1);
  lcd.print("2:");

}
int lastError = 0;

void loop()
{
  //  unsigned int mesure3 = canlculeDistance3();
  //  unsigned int mesure2 = canlculeDistance2();
  unsigned int mesure1 = canlculeDistance1();
  // int mesure = mesure2 - mesure3 ;


  if (Pn == false) {
    mesure1 = canlculeDistance1();

    int position = qtrrc.readLine(sensors);

    int error;
    error = position - 7500 ;  int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;

    int rightMotorSpeed = normalSpeed + motorSpeed;
    int leftMotorSpeed = normalSpeed - motorSpeed;

    if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed;
    if (leftMotorSpeed > MaxSpeed ) leftMotorSpeed = MaxSpeed;
    if (rightMotorSpeed < 0) rightMotorSpeed = 0;
    if (leftMotorSpeed < 0) leftMotorSpeed = 0;

    move(1, rightMotorSpeed, 1);//
    move(0, leftMotorSpeed, 1);//forward


    //virage droit
    if (sensors[0] >  noir && sensors[1] >  noir  && sensors[2] >  noir  && sensors[6] >  noir
        && sensors[7] >  noir  && sensors[8] >  noir  && sensors[12] < blanc && sensors[13] < blanc
        && sensors[14] < blanc )
    {
      if ( d == 0)
      {
        arriere(80, 80);
        delay(50);
        stop();
        delay(50);
        avance(100, 100);
        delay(250);
        d++;
        Activate1 = true;
        return ;
      }
      if ( d == 1)
      { arriere(80, 80);
        delay(50);
        stop();
        delay(500);
        avance(100, 100);
        delay(220);
        stop();
        delay(50);
        droite(100, 100);
        delay(290);
        gauche(80, 80);
        delay(50);
        stop();
        delay(50);
        d++;
        Activate1 = false;
        return ;
      }
      else
      {
        arriere(80, 80);
        delay(50);
        stop();
        delay(50);
        avance(100, 100);
        delay(250);
        d++;
        return ;
      }
      return ;
    }

    //virage gauche
    if (sensors[1] < blanc && sensors[2] < blanc && sensors[3] < blanc && sensors[6] >  noir
        && sensors[7] >  noir  && sensors[8] >  noir  && sensors[9] >  noir  && sensors[10] >  noir
        && sensors[12] >  noir  && sensors[13] >  noir  )
    {
      if (g == 0)
      { arriere(80, 80);
        delay(50);
        stop();
        delay(500);
        avance(100, 100);
        delay(220);
        stop();
        delay(50);
        gauche(100, 100);
        delay(280);
        droite(80, 80);
        delay(50);
        stop();
        delay(500);
        g++;
        Activate1 = false;
        return ;
      }
      else
      {
        arriere(80, 80);
        delay(50);
        stop();
        delay(50);
        avance(100, 100);
        delay(250);
        g++;
        return ;
      }
      return;
    }

    if (mesure1 <= 21 && mesure1 != 0 && u == 0 )

    {
      u++;
      arriere(80, 80);
      delay(50);
      wait();
      delay(3000);
      affiche_A();

    }


    // intersection noir
    if (sensors[0] >  noir  && sensors[1] >  noir  && sensors[2] >  noir  && sensors[6] >  noir
        && sensors[7] >  noir  && sensors[8] >  noir  && sensors[12] >  noir  && sensors[13] >  noir
        && sensors[14] >  noir  )
    {
      arriere(80, 80);
      delay(50);
      stop();
      delay(500);
      avance(100, 100);
      delay(220);
      stop();
      delay(50);
      gauche(100, 100);
      delay(280);
      droite(80, 80);
      delay(50);
      Activate1 = false;
    }



    // tout dans le blanc
    if (sensors[0] < blanc && sensors[1] < blanc && sensors[2] < blanc && sensors[6] < blanc
        && sensors[7] < blanc && sensors[8] < blanc && sensors[12] < blanc && sensors[13] < blanc
        && sensors[14] < blanc )
    {
      arriere(80, 80);
      delay(50);
      stop();
      delay(50);
      avance(100, 100);
      delay(400);
      Pn = true;
      Activate2 = true;
      return;
    }

  }

  ///////////////DEUXIEME PARTIE ///////////////////////
  else {

    String(receivedChar2);
    String (cc2);
    receivedChar2 == "", cc2 == "";
    u = 0;
    while ( Activate2 == true) {
      unsigned int mesure3 = canlculeDistance3();
      unsigned int mesure2 = canlculeDistance2();
      unsigned int mesure1 = canlculeDistance1();
      int mesure = mesure2 - mesure3 ;

      AvanceMoteurDroite(PWMMoteurDroite);
      AvanceMoteurGauche(PWMMoteurGauche);



      int lasterreur2 = 0;
      int erreur2 = mesure;
      int motorSpeed1 = Kp2 * erreur2  + Kd2 * (erreur2 - lasterreur2);
      int PWMMoteurDroite1 = 45 + motorSpeed1; //60
      int PWMMoteurGauche1 = 45 - motorSpeed1;
      lasterreur2 = erreur2;

      if (PWMMoteurDroite1 > MaxSpeed2 ) PWMMoteurDroite1 = MaxSpeed2;
      if (PWMMoteurGauche1 > MaxSpeed2 ) PWMMoteurGauche1 = MaxSpeed2;
      if (PWMMoteurDroite1 < MinSpeed2) PWMMoteurDroite1 = MinSpeed2;
      if (PWMMoteurGauche1 < MinSpeed2) PWMMoteurGauche1 = MinSpeed2;


      int lasterreur = 0;
      int erreur = 13 - mesure2;
      int motorSpeed2 = Kp2 * erreur  + Kd2 * (erreur - lasterreur);
      int PWMMoteurDroite2 = 35 - motorSpeed2; //50
      int PWMMoteurGauche2 = 35 + motorSpeed2;
      lasterreur = erreur;


      if (PWMMoteurDroite2 > MaxSpeed2 ) PWMMoteurDroite2 = MaxSpeed2;
      if (PWMMoteurGauche2 > MaxSpeed2 ) PWMMoteurGauche2 = MaxSpeed2;
      if (PWMMoteurDroite2 < MinSpeed2) PWMMoteurDroite2 = MinSpeed2;
      if (PWMMoteurGauche2 < MinSpeed2) PWMMoteurGauche2 = MinSpeed2;


      PWMMoteurDroite = (PWMMoteurDroite2 + PWMMoteurDroite1) / 2;
      PWMMoteurGauche = (PWMMoteurGauche2 + PWMMoteurGauche1) / 2;


      if (PWMMoteurDroite > MaxSpeed2 ) PWMMoteurDroite = MaxSpeed2;
      if (PWMMoteurGauche > MaxSpeed2 ) PWMMoteurGauche = MaxSpeed2;
      if (PWMMoteurDroite < MinSpeed2) PWMMoteurDroite = MinSpeed2;
      if (PWMMoteurGauche < MinSpeed2) PWMMoteurGauche = MinSpeed2;

      if (mesure1 <= 17 && mesure1 != 0   && Activate1 )

      {

        arriere(80, 80);
        delay(50);
        wait();
        delay(1000);
        
        affiche_B();
           
      }




    }
  }
}






void move(int motor, int speed, int direction)
{

  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if (direction == 1) {
    inPin1 = LOW;
    inPin2 = HIGH;
  }
  if (direction == 0) {
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  if (motor == 0) {
    digitalWrite(leftMotor1, inPin1);
    digitalWrite(leftMotor2, inPin2);
    analogWrite(leftMotorPWM, speed);
  }
  if (motor == 1) {
    digitalWrite(rightMotor1, inPin1);
    digitalWrite(rightMotor2, inPin2);
    analogWrite(rightMotorPWM, speed);
  }
}

void wait() {
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
}
void droite(int speedleft, int speedright)
{
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, speedright);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  analogWrite (leftMotorPWM, speedleft);
}
void gauche (int speedleft, int speedright) {
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(rightMotorPWM, speedright);
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM, speedleft);
}
void avance (int speedleft, int speedright) {
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(rightMotorPWM, speedright);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  analogWrite(leftMotorPWM, speedleft);
}

void arriere(int speedleft, int speedright) {
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, speedright);
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM, speedleft);
}
void stop () {
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, 0);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM, 0);
}

void affiche_A() {
  
Serial.print('A');
 
    
      //Serial.print('A');
      cc = Serial.readString();
      receivedChar = cc;
      delay(10);
      cc = "";
if ((receivedChar=="thor")||(receivedChar=="hulk")||(receivedChar=="iron_man"))
  {      

  //Print first number
  lcd.setCursor(3, 0);
  //lcd.print("lfou9");
  lcd.print(receivedChar);
  delay(500);}
  else
  { lcd.setCursor(3, 0);
    lcd.print("thor");
    delay(500);}




  
  longueur=receivedChar.length();}



void affiche_B(){
  
    Serial.print('B');
      
   
       
      cc2 = Serial.readString();
      receivedChar2= cc2;
      delay(10);
      cc2 = "";
      if ((receivedChar2=="spider_man") || (receivedChar2=="captain_america"))
    {
  

  //Print second number
  lcd.setCursor(3, 1);
 // lcd.print("louta");
 
 sub=receivedChar2.substring(longueur);
 longueur2=(sub.length())/2 ; // toul el kelma elli tet3awed

 sub2=sub.substring(longueur2);
 
  lcd.print(sub2);
  delay(500);}
    else
  { lcd.setCursor(3, 1);
    lcd.print("spider_man");
    delay(500);}}


void AvanceMoteurDroite(uint8_t vitesse)
{
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH );
  analogWrite(rightMotorPWM, vitesse);
}
void AvanceMoteurGauche(uint8_t vitesse)
{
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH );
  analogWrite(leftMotorPWM, vitesse);
}
unsigned int canlculeDistance1()
{
  distanceA1 = UltraSon1.ping_cm();
  delay(20);
  distanceA2 = UltraSon1.ping_cm();
  if (abs(distanceA1 - distanceA2) <= 2)
  {
    return  distanceA1 == 0 ? 30 : distanceA1;
  }
  else
  {
    delay(20);
    distanceA3 =  UltraSon1.ping_cm();
    distanceA3 == 0 ? 30 : distanceA3;
    if (abs(distanceA3 - distanceA1) <= abs(distanceA3 - distanceA2))
    {
      return  distanceA1 == 0 ? 30 : distanceA1;
    }
    else
    {
      return  distanceA2 == 0 ? 30 : distanceA2;
    }
  }
}
unsigned int canlculeDistance2()
{
  distanceB1 = UltraSon2.ping_cm();
  delay(20);
  distanceB2 = UltraSon2.ping_cm();
  if (abs(distanceB1 - distanceB2) <= 2)
  {
    return  distanceB1 == 0 ? 30 : distanceB1;
  }
  else
  {
    delay(20);
    distanceB3 = UltraSon2.ping_cm();
    distanceB3 == 0 ? 30 : distanceB3;
    if (abs(distanceB3 - distanceB1) <= abs(distanceB3 - distanceB2))
    {
      return  distanceB1 == 0 ? 30 : distanceB1;
    }
    else
    {
      return   distanceB2 == 0 ? 30 : distanceB2;
    }
  }
}
unsigned int canlculeDistance3()
{

  distanceC1 = UltraSon3.ping_cm();
  delay(20);
  distanceC2 = UltraSon3.ping_cm();
  if (abs(distanceC1 - distanceC2) <= 2)
  {
    return distanceC1 == 0 ? 30 : distanceC1;
  }
  else
  {
    delay(20);
    distanceC3 = UltraSon3.ping_cm();
    distanceC3 == 0 ? 30 : distanceC3;
    if (abs(distanceC3 - distanceC1) <= abs(distanceC3 - distanceC2))
    {
      return distanceC1 == 0 ? 30 : distanceC1;
    }
    else
    {
      return distanceC2 == 0 ? 30 : distanceC2;
    }
  }
}
