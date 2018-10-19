//Driver 1
int EN11  = 22;
int EN12  = 23; 
int EN13  = 24;    
int EN14  = 25;    

//Driver 2
int EN21  = 26;
int EN22  = 27; 
int EN23  = 28;    
int EN24  = 29;  

//Buttons
int up = 30;
int down = 31;
int right = 32;
int left = 33;

void setup(){
 //Driver 1
 pinMode (EN11, OUTPUT); 
 pinMode (EN12, OUTPUT);
 pinMode (EN13, OUTPUT);
 pinMode (EN14, OUTPUT);

 //Driver 2
 pinMode (EN21, OUTPUT); 
 pinMode (EN22, OUTPUT);
 pinMode (EN23, OUTPUT);
 pinMode (EN24, OUTPUT);

 //Buttons
 pinMode (up,    INPUT_PULLUP);
 pinMode (down,  INPUT_PULLUP);
 pinMode (right, INPUT_PULLUP);
 pinMode (left,  INPUT_PULLUP);
}

void forward(){
    digitalWrite(EN11, HIGH);
    digitalWrite(EN12, LOW);
    digitalWrite(EN13, HIGH);
    digitalWrite(EN14, LOW);
    digitalWrite(EN21, HIGH);
    digitalWrite(EN22, LOW);
    digitalWrite(EN23, HIGH);
    digitalWrite(EN24, LOW);
}

void reverse(){
  digitalWrite(EN11, LOW);
  digitalWrite(EN12, HIGH);
  digitalWrite(EN13, LOW);
  digitalWrite(EN14, HIGH);
  digitalWrite(EN21, LOW);
  digitalWrite(EN22, HIGH);
  digitalWrite(EN23, LOW);
  digitalWrite(EN24, HIGH);
}

void turnright(){
  digitalWrite(EN11, LOW);
  digitalWrite(EN12, HIGH);
  digitalWrite(EN13, HIGH);
  digitalWrite(EN14, LOW);
  digitalWrite(EN21, LOW);
  digitalWrite(EN22, HIGH);
  digitalWrite(EN23, HIGH);
  digitalWrite(EN24, LOW);
}

void turnleft(){
  digitalWrite(EN11, HIGH);
  digitalWrite(EN12, LOW);
  digitalWrite(EN13, LOW);
  digitalWrite(EN14, HIGH);
  digitalWrite(EN21, HIGH);
  digitalWrite(EN22, LOW);
  digitalWrite(EN23, LOW);
  digitalWrite(EN24, HIGH);
}

void stop(){
  digitalWrite(EN11, LOW);
  digitalWrite(EN12, LOW);
  digitalWrite(EN13, LOW);
  digitalWrite(EN14, LOW);
  digitalWrite(EN21, LOW);
  digitalWrite(EN22, LOW);
  digitalWrite(EN23, LOW);
  digitalWrite(EN24, LOW); 
}

void loop(){
  if(!digitalRead(up)){
    forward();
  }else if(!digitalRead(down)){
    reverse();
  }else if(!digitalRead(right)){
    turnright();
  }else if(!digitalRead(left)){
    turnleft();
  }else{
    stop();
  }
  
  /*TEST 1
  digitalWrite(EN11, HIGH);
  digitalWrite(EN12, LOW);
  digitalWrite(EN13, HIGH);
  digitalWrite(EN14, LOW);
  digitalWrite(EN21, HIGH);
  digitalWrite(EN22, LOW);
  digitalWrite(EN23, HIGH);
  digitalWrite(EN24, LOW);*/
}
