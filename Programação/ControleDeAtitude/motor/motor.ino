#define DIRECTION 2
#define BREAK 3
#define START 4
#define PWM 5

void setup() {
  pinMode(PWM, OUTPUT);
  pinMode(START,OUTPUT);
  pinMode(BREAK,OUTPUT);
  pinMode(DIRECTION,OUTPUT); 
 
  digitalWrite(START,HIGH);
  digitalWrite(BREAK,HIGH);
  //tone(PWM,26000);  //Velocidade do motor: 26KHz = 3900rpm
}

void loop() {
  freia(1); //Freia
  delay(1000);

  digitalWrite(DIRECTION,LOW);  //Muda a direção para HORÁRIO 

  freia(0);   //Libera o freio
  acelera();
  delay(2000);
  desacelera();

  freia(1);  //Freia
  delay(1000);
  
  digitalWrite(DIRECTION,HIGH); //Muda a direção para ANTI-HORÁRIO

  freia(0);   //Libera o freio
  acelera();
  delay(2000);
  desacelera();
}

void freia(int x){
  if(x)
    digitalWrite(BREAK,LOW);  //Freia
  else
    digitalWrite(BREAK,HIGH);  //Libera freio
}

void acelera(){
  tone(PWM,18000);
}

void desacelera(){
    tone(PWM,0);
}

