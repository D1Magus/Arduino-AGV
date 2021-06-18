// SENAI/SC - Serviço Nacional de Aprendizagem Industrial
// Data: 18/06/2021  |  Joinville
// Integrantes: Eduardo Expedito Menezes, Fernando Henrique Vaz Mello e Gabriel Vinicius Tomasi
// Professores: Leandro Yoshio Morita e Rafael Raul Pereira
// Projeto AGV

#include <Ultrasonic.h> //Biblioteca para o HR-SR04
#include <AFMotor.h> //Biblioteca para controle do motor


#define ledPinGreen 47 //Definição de pinos
#define ledPinYellow 51
#define triggerPin 22
#define echoPin 24
#define buzzerPin 51
#define obsenPin 45 

//Iniciando e criando os motores
AF_DCMotor leftMotor(2, MOTOR12_8KHZ);
AF_DCMotor rightMotor(3, MOTOR34_8KHZ);
AF_DCMotor conveyorMotor(4, MOTOR34_8KHZ);

//Iniciando e criando o sensor ultrassônico
Ultrasonic ultraSensor(triggerPin, echoPin);

//Variaveis
int rightSpeed, leftSpeed, sensorsSum, location, distance;
long proportional, integral, derivative, lastProportional, errorValue, sensorsAverage;
boolean sensors[] = {0, 0, 0, 0}; //Array boolean para os sensores 
int maxSpeed = 55; //Velocidade máxima do AGV
boolean ended = 0, inConveyor, timer = 0;
int mode = 0;

//Variaveis constantes 
const int setPoint = 1500; //Posição a ser alcançada pelo AGV
const int setDistance = 10;
const int fastTurnRating = 1;


//Variaveis para timing
unsigned long currentMillis = 0; 
unsigned long previousLedMillis = 0, previousCalcMillis = 0, previousBuzzerMillis = 0, previousConveyorMillis = 0, previousTurnMillis = 0;
unsigned const int ledInterval = 500;
unsigned const int calcInterval = 0;
unsigned const int buzzerInterval1 = 100;
unsigned const int buzzerInterval2 = 200;
unsigned const int conveyorInterval = 15000;
int testState = 0;
boolean turningState = 0, withPart = 0, brake;


//Variaveis controle PID(proporcional, integral e derivativo)
float Kp = 0.9;
const float Ki = 0;
float Kd = 4.5;

//Função para calcular o erro usando o controle PID
void pidCalculation(){
  proportional = location - setPoint;
  integral = integral + proportional;
  derivative = proportional - lastProportional;
  lastProportional = proportional;
  errorValue = int(proportional*Kp + integral*Ki + derivative*Kd);
}

//Função para calcular e mapear a velocidade
void speedCalculation(){
  errorValue = (errorValue > 255) ? 255 : errorValue; //Operação ternária
  errorValue = (errorValue < -255) ? -255 : errorValue; //Operação ternária

  if(errorValue < 0){
    if((maxSpeed != 255)) errorValue = map(errorValue, 0, -255, 0, -(maxSpeed)); //Mapeamento do valor da velocidade com a velocidade máxima
    rightSpeed = maxSpeed + errorValue; //Velocidade motor 
    leftSpeed = maxSpeed - errorValue; //Velocidade motor 
  }
  else{
    if((maxSpeed != 255)) errorValue = map(errorValue, 0, 255, 0, (maxSpeed)); //Mapeamento do valor da velocidade com a velocidade máxima
    rightSpeed = maxSpeed + errorValue; //Velocidade motor 
    leftSpeed = maxSpeed - errorValue; //Velocidade motor 
  }
}

void setup() {
  pinMode(28, INPUT);
  pinMode(26, INPUT);
  pinMode(30, INPUT);
  pinMode(32, INPUT);
  pinMode(ledPinGreen, OUTPUT);
  pinMode(ledPinYellow, OUTPUT);
  digitalWrite(ledPinGreen, HIGH);
  digitalWrite(ledPinYellow, HIGH);
  withPart = 0;
  timer = 0;
  mode = 0;
  rightMotor.run(RELEASE);
  leftMotor.run(RELEASE);
  conveyorMotor.run(RELEASE);

}

void loop() {
  sensorsAverage = 0;
  sensorsSum = 0;
  currentMillis = millis(); //Valor do tempo apos o sistema ligar em milisegundos
  distance = ultraSensor.read(); //Input da distancia do SR-04
  if(mode != 2 && mode != 0 && turningState == 0){
    sensors[0] = (digitalRead(28)); //Leitura dos valores dos sensores de linha infravermelhos
    sensors[1] = (digitalRead(26));
    sensors[2] = (digitalRead(30));
    sensors[3] = (digitalRead(32));
  }
 
  if((sensors[0] == 1)&&(sensors[1]==1)&&(sensors[2]==1)&&(sensors[3]==1)){ //AGV chegou no destino
              timer++;
              if(withPart == 1) mode = 2;
              if(withPart == 0) mode = 0;
  }
  
  if(distance > setDistance && turningState == 0){   //Condição para parar quando algo está proximo ao sensor SR-04
    if (currentMillis - previousCalcMillis > calcInterval) { //Condição para timing baseado no calcInterval
      if(mode == 0){ //Modo Standy(Esperando a peça ser colocada na esteira)
        digitalWrite(ledPinYellow, HIGH);
        if(timer != 0 && brake == 0){ //Freio
          maxSpeed = 105; 
          rightMotor.setSpeed(maxSpeed);
          leftMotor.setSpeed(maxSpeed);
          rightMotor.run(BACKWARD);
          leftMotor.run(BACKWARD);
          delay(200);
          brake = 1;
        }
         inConveyor = !(digitalRead(obsenPin));
         leftMotor.setSpeed(0);
         rightMotor.setSpeed(0);
         leftMotor.run(RELEASE);
         rightMotor.run(RELEASE);
         if(inConveyor == 1){
          //Parte do codigo que diz quando é preciso dar um 180 para continuar o codigo
          if(timer != 0){
           maxSpeed = 58;
           rightSpeed = maxSpeed*0.8;
           leftSpeed = maxSpeed*1.3;
           rightMotor.setSpeed(rightSpeed);
           leftMotor.setSpeed(leftSpeed);
           rightMotor.run(FORWARD);
           leftMotor.run(BACKWARD);
           delay(1500);
           sensors[0] = 0;
           sensors[1] = 0;
           sensors[2] = 0;
           sensors[3] = 0;
           while(!(sensors[2] == 1)){
             rightSpeed = maxSpeed*0.8;
             leftSpeed = maxSpeed*1.3;
             rightMotor.setSpeed(rightSpeed);
             leftMotor.setSpeed(leftSpeed);
             sensors[2] = (digitalRead(30));
             rightMotor.run(FORWARD);
             leftMotor.run(BACKWARD);
           }
          turningState = 0;
         }
         mode = 1;
         withPart = 1;
         //Terminou de fazer a curva e está com a peça 
       }
      }
      if(mode == 1){ //Modo onde o AGV está em movimento
          digitalWrite(ledPinYellow, LOW);
          brake = 0;
          previousCalcMillis = millis(); 
          if(sensors[1]==1||sensors[2]==1){
            Kp = 0.083; //Valores do PID e velocidade quando o AGV está no meio da linha
            Kd = 0.8; 
            maxSpeed = 60;
          }
          else{
            Kp = 0.3; //Valores do PID e velocidade quando o AGV não está na linha ou está fazendo curva
            Kd = 3;
            maxSpeed = 40;
          }
  
          
          for(int i=0; i<4; i++){ //Loop para pegar dados do sensor e calular media e soma.
            sensorsAverage += sensors[i] * i * 1000;
            sensorsSum += int(sensors[i]);
          }
          
            location = int(sensorsAverage/sensorsSum); //Calcula a localização

            if((!((sensors[0] == 0)&&(sensors[1]==0)&&(sensors[2]==0)&&(sensors[3]==0)))) //Parte do codigo para situações diferentes dos sensores e como reagir a elas(Ex: curva 90º)
          { 
             pidCalculation();
             speedCalculation();
            if((sensors[1]==1)||(sensors[2]==1)){
              leftMotor.setSpeed(leftSpeed);
            }
            else{
              leftMotor.setSpeed(leftSpeed*fastTurnRating);
            }
            if((leftSpeed == 0 && rightSpeed != 0)&&(sensors[0]==1||sensors[3]==1)){
              //leftMotor.setSpeed(maxSpeed);
             // leftMotor.run(BACKWARD);
             rightMotor.run(RELEASE);
            }
            else{
              if(leftSpeed == 0)leftMotor.run(RELEASE);
              else leftMotor.run(FORWARD);
           }
          
            if((sensors[1]==1)||(sensors[2]==1)){
              rightMotor.setSpeed(rightSpeed);
            }
            else{
              rightMotor.setSpeed(rightSpeed*fastTurnRating);
            }
            if((rightSpeed == 0 && leftSpeed != 0)&&(sensors[0]==1||sensors[3]==1)){
              //rightMotor.setSpeed(maxSpeed);
              //rightMotor.run(BACKWARD);
              rightMotor.run(RELEASE);
            }
            else{
              if(rightSpeed == 0) rightMotor.run(RELEASE);
              else rightMotor.run(FORWARD);
           }
          }
          else{ //Parte do codigo que implementa um freio caso a roda esteja em uma velocidade muito baixa durante uma curva
            if(currentMillis - previousTurnMillis > 200){
              previousTurnMillis = millis();
              if(rightSpeed == 0 && leftSpeed != 0){
              rightMotor.setSpeed(maxSpeed/1.7);
              rightMotor.run(BACKWARD);
            }
            if(leftSpeed == 0 && rightSpeed != 0){
              leftMotor.setSpeed(maxSpeed/1.7);
              leftMotor.run(BACKWARD);
            }
            }
          }
      
      }
      if(mode == 2){ //Modo para entregar a peça
        maxSpeed = 105;

        rightMotor.setSpeed(maxSpeed);
        leftMotor.setSpeed(maxSpeed);
        rightMotor.run(BACKWARD);
        leftMotor.run(BACKWARD);
        delay(200);
        //Freio
        previousConveyorMillis = millis();
        rightMotor.run(RELEASE);
        leftMotor.run(RELEASE);
        conveyorMotor.setSpeed(110);
        conveyorMotor.run(BACKWARD);
        delay(2500);
        //Entrega a peça com a esteira
        conveyorMotor.run(RELEASE);
        turningState = 1;
        //Curva 180º
        maxSpeed = 58;
        rightSpeed = maxSpeed*0.8;
        leftSpeed = maxSpeed*1.3;
        rightMotor.setSpeed(rightSpeed);
        leftMotor.setSpeed(leftSpeed);
        rightMotor.run(FORWARD);
        leftMotor.run(BACKWARD);
        delay(1500);
        sensors[0] = 0;
        sensors[1] = 0;
        sensors[2] = 0;
        sensors[3] = 0;
       
       while(!(sensors[2] == 1)){
         rightSpeed = maxSpeed*0.8;
         leftSpeed = maxSpeed*1.3;
         rightMotor.setSpeed(rightSpeed);
         leftMotor.setSpeed(leftSpeed);
         sensors[2] = (digitalRead(30));
         rightMotor.run(FORWARD);
         leftMotor.run(BACKWARD);
       }
       
       turningState = 0;
       withPart = 0;
       mode = 1;
       //Termina sem a peça e com uma curva 180º feia
      }
    }
    
  }
     else{//Para caso algo esteja muito proximo do sensor SR-04
      rightMotor.run(RELEASE);
      leftMotor.run(RELEASE);
     }  
}
    
      
    
    
    
    
