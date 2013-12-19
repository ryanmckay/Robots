#pragma config(Sensor, S1, HTEOPD, sensorAnalogActive)
#pragma config(Sensor, S2, soundSensor, sensorSoundDB)
#pragma config(Sensor, S3, lightSensor, sensorLightActive)
#pragma config(Sensor, S4, touchSensor, sensorTouch)
#include "drivers/hitechnic-eopd.h";

bool go = true;


void checkSensors(){
int time = nPgmTime/1000;
nxtDisplayString(0, "%d", time);

/*if button is hit or light sensor finds something*/
if(SensorRaw[lightSensor] < 450 || SensorValue(touchSensor) == 1){
//back up
motor[motorB] = -50;
motor[motorA] = -50;
wait1Msec(300);

//turn
motor[motorB] = 35;
motor[motorA] = -35;
wait1Msec(600);  //625 when 25

//stop
motor[motorA] = 0;
motor[motorB] =0;
wait1Msec(75);
}
}

task main(){

nxtDisplayString(0, "%d", SensorValue[soundSensor]);
wait1Msec(250);//gets rid of sound bug
while(1){
if (SensorValue[soundSensor] > 90){
go = !go;
motor[motorA] = 0;
motor[motorB] = 0;
wait1Msec(500);
}
if(go){

eraseDisplay();

/*while it if following the wall*/
if(HTEOPDreadProcessed(HTEOPD) > 30 && SensorRaw[lightSensor] > 450 && SensorValue(touchSensor) == 0){
motor[motorA] = 75;
motor[motorB] = 85;
checkSensors();
}
/*if it loses the wall*/
else if(HTEOPDreadProcessed(HTEOPD) < 30){
while(HTEOPDreadProcessed(HTEOPD) < 30){
if(HTEOPDreadProcessed(HTEOPD) < 12){
for(int i = 0; i < 10; i++){
motor[motorB] = -10;
motor[motorA] = 50;
wait1Msec(10);
checkSensors();
}
for(int i = 0; i < 30; i++){
motor[motorB] = 80;
motor[motorA] = 85;
wait1Msec(10);
checkSensors();
}
}
else if(HTEOPDreadProcessed(HTEOPD) > 12){
for(int i = 0; i < 10; i++){
motor[motorA] = 60;
motor[motorB] = 55;
wait1Msec(10);
checkSensors();
}
}
checkSensors();
if (SensorValue[soundSensor] > 90){
go = !go;
motor[motorA] = 0;
motor[motorB] = 0;
wait1Msec(500);
break;
}
}//end of while
}//end of else if

checkSensors();
}//end of while
}
}//end of main
robot3.c

1 of 1
