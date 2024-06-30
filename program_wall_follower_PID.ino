
#define Sensor_maksimum
#define DIST_konversi 0.034/2 
#define INPUT_jarak_target 15 
#define error_jarak 5  
#define kecepatan 80
#define MAX_nilai_sensor 10000
#define MAX_SENSOR 3000
#define CLIBRATION 3

float integral, derivative, output, currentDistance;
int distance, check_wilayah_sensor;
float baca1, baca2, baca3;

float current_distance;
//sensor kanan
#define echokanan A1
#define trigkanan A0

//sensor tengah
#define echotengah A3
#define trigtengah A2

//sensor kiri
#define echokiri A5
#define trigkiri A4

//motor kanan
const int IN1 = 2;
const int IN2 = 3;
const int enable12 = 11;

//motor kiri
const int IN3 = 5;
const int IN4 = 6;
const int enable34=10;

//tuning parameter PID
float kp = 2;
float kd = 0;
float ki = 0;

//distance
float errorD;
float Last_ErrorD = 0;
float dt = 0.1;

//membaca_sensor
float sensor_output(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

   float duration = pulseIn(echoPin, HIGH);

    if(duration==0.0||duration>=MAX_nilai_sensor){
    duration = MAX_SENSOR;
    }
    
//konversi jarak ke cm
  float distance = duration*DIST_konversi;
   
  return distance; 
  }

  //mendapatkan nilai rata-rata dari ketiga sensor

float rata_sensor(float baca1, float baca2, float baca3){
  float distance = (baca1+baca2+baca3)/3;
  return distance;

}
//pembaca wilaya sensor untuk mekondisikan putaran motor
 int check_wilayah_sensor(float baca1, float baca2, float baca3){
  distance = current_distance(baca1, baca2, baca3);
  if(abs(distance-INPUT_jarak_target)>ERROR_jarak){
    if(distance > INPUT_jarak_target){
        return -1; //untuk motor kiri
      }else{
        return 1; //untuk motor kanan
        }
    }else{
      return 0; //inside region
      }
  }

//perhitungan PID
  
   float pid(baca1, baca2, baca3){
   currentDistance = current_distance(baca1, baca2, baca3);
   errorD = currentDistance - INPUT_jarak_target;
   integral = integral + errorD*dt;
   derivative = (errorD - Last_ErrorD/dt);
   output = kp*errorD + ki*integral + kd*derivative;
   Last_ErrorD  = errorD;

  return output;
  }


void setup() {
pinMode(trigkanan,OUTPUT);
pinMode(echokanan,INPUT);
pinMode(trigtengah,OUTPUT);
pinMode(echotengah,INPUT);
pinMode(trigkiri,OUTPUT);
pinMode(echokiri,INPUT);

pinMode(IN1, OUTPUT);
pinMode(IN2, OUTPUT);
pinMode(IN3, OUTPUT);
pinMode(IN4, OUTPUT);
pinMode(enable12, OUTPUT);
pinMode(enable34, OUTPUT);
digitalWrite(enable12, LOW);
digitalWrite(enable34, LOW);

}

void loop() {
  float distance1 = sensor_output(trigkanan, echokanan);
  float distance2 = sensor_output(trigtengah, echotengah);
  float distance3 = sensor_output(trigkiri, echokiri);

   region = check_region(distance1, distance2, distance3);
   pid = ceil(pid_distance(distance1, distance2, distance3));
   speedL = SPEED - pid;
  speedR = SPEED + pid;

    if(speedR>255){
    speedR = 255;
  }
  if(speedR<0){
    speedR = 0; 
   }
  if(speedL>200){
    speedL = 200;
  }
  if(speedL<50){
    speedL = 50; 
   }

   if(region == 0){
    analogWrite(en34, SPEED+4);
    analogWrite(en12, SPEED);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    }
   else{
    analogWrite(en34, speedL+ CALIBRATION);
    analogWrite(en12, speedR);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(iN4, LOW);
    }

}
