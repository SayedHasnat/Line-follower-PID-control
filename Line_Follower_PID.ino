int const sens_numb= 5;
int Lmot[]= {5,3};
int Rmot[]= {10,11};
int sens[]= {14,15,16,17,18};
int threshold=200;

int min_speed=150,avg_speed=200,new_speed;

void  val_read();
void ser_print();
void weight_gen();
void straight();
void left();
void right();
void stp();
void pid_line_follow();;
void direct();


int j,k;
int analog_val[sens_numb], digital_val[sens_numb], weight[sens_numb];
int P=0, I=0, D=0, correction, error=0, previous_error=0, kp=1, kd=1, ki=1;
int weighted_val[sens_numb], total_weight;
void setup()
{
  for(j=0;j<sens_numb;j++)
      pinMode(sens[j],INPUT);
  
  for(j=0;j<2;j++)
     {
       pinMode(Lmot[j],OUTPUT);
      pinMode(Rmot[j],OUTPUT);
    }
   Serial.begin(9600);
   
}

void  val_read()
{
  for(j=0;j<sens_numb;j++)
    {
       analog_val[j]=analogRead(sens[j]);
      if (analog_val[j]<=threshold)
        digital_val[j]=0;    //for black value< threashold
      else
        digital_val[j]=1;
    }
}


void ser_print()
{
    Serial.println("##################");
    Serial.println("analog values");
  for(j=0;j<sens_numb;j++)
    Serial.println("Sensor"+String(j)+"="+String(analog_val[j]));
    Serial.println(" ");
    
    Serial.println("##################");
    Serial.println("digital values");
  for(j=0;j<sens_numb;j++)
    Serial.print("  " + String(digital_val[j]));
    Serial.println(" ");
    
    Serial.println("##################");
    Serial.println("Generated weight");
  for(j=0;j<sens_numb;j++)
    Serial.print("  "+ String(weight[j]));
    Serial.println(" ");
    
    Serial.println("##################");
    Serial.println("Weighted Value");
  for(j=0;j<sens_numb;j++)
    Serial.print("  "+ String(weighted_val[j]));
    Serial.println(" ");
    Serial.println("##################");
}

void weight_gen()
{
     k=sens_numb/2;
   if(sens_numb%2!=0){
       k= -k;
     for(j=0;j<sens_numb;j++){
       weight[j]=k;
       k=k+1;
       
     }
   }  
   else{
     k= -k;
     for(j=0;j<sens_numb;j++){
       if(k==0)
         k=1;
       weight[j]=k;
       k=k+1;
     }
   }
}

void straight()
{
  analogWrite(Lmot[0],avg_speed);
  analogWrite(Lmot[1],0);
  analogWrite(Rmot[1],avg_speed);
  analogWrite(Rmot[2],0);
}

void left()
{
  analogWrite(Lmot[0],0);
  analogWrite(Lmot[1],0);
  analogWrite(Rmot[1],new_speed);
  analogWrite(Rmot[2],0);
}

void right()
{
  analogWrite(Lmot[0],new_speed);
  analogWrite(Lmot[1],0);
  analogWrite(Rmot[1],0);
  analogWrite(Rmot[2],0);
}

void stp()
{
  analogWrite(Lmot[0],0);
  analogWrite(Lmot[1],0);
  analogWrite(Rmot[1],0);
  analogWrite(Rmot[2],0);
}

void pid_line_follow()
{
  
  for(j=0;j<sens_numb;j++){
    weighted_val[j] =weight[j] * digital_val[j];
    total_weight = total_weight + weighted_val[j];
  }
  int current_weight=abs(total_weight);
  int target_weight=0;
  error =  current_weight - target_weight;
  P=error*kp;
  I=I+error;
  I=I*ki;
  D=kd * (error - previous_error);
  correction= P+I+D;
  new_speed=min_speed + correction;
  previous_error=error; 
}

void direct()
{
  int mid = (sens_numb/2)+1;
   if(total_weight==0 && sens[mid]==1)
    stp();
   else if(total_weight==0&&sens[mid]==0)
    straight();
   else if(total_weight<0)
     left();
   else if(total_weight>0)
     right();
}

void loop()
{
  val_read();
  weight_gen();
  ser_print();
  pid_line_follow();
  direct();
  //delay(3000);
}
