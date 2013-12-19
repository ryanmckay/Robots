//Wait(300) 15degree
//Ryan McKay and George Jaray
task main(){
     //int t0 = CurrentTick();
     long t0 = CurrentTick();
     SetSensorLight(IN_3);
     bool working = true;
     int timer = 0;
     while(working){
                 OnFwd(OUT_AB, 100);
                 until(SensorValue(IN_3) >= 55);
                 int limit =8;
                 int i = 4;
                 while( i <= limit){
                        while(SensorValue(IN_3) >= 55 && timer< i){
                        OnRev(OUT_A, 45);
                        OnFwd(OUT_B, 45);
                        Wait(90);
                        timer++;
                        }
                        timer = 0;
                        Off(OUT_AB);
                        while(SensorValue(IN_3) >= 55 && timer< i*2){
                        OnFwd(OUT_A, 45);
                        OnRev(OUT_B, 45);
                        Wait(90);
                        timer++;
                        }
                        timer = 0;
                        Off(OUT_AB);
                        if(i != limit){
                        while(SensorValue(IN_3) >= 55 && timer< i){
                        OnRev(OUT_A, 45);
                        OnFwd(OUT_B, 45);
                        Wait(90);
                        timer++;
                        }}
                        timer = 0;
                        i = i+i;
                 }
                 timer = 0;
                 if(SensorValue(IN_3) > 55){
                     working = false;
                 }


     }
     int final = (CurrentTick()- t0)/1000;
     //NumOut(0, LCD_LINE1, t0);
     NumOut(0, LCD_LINE1, final);
     Off(OUT_AB);
     TextOut(0, LCD_LINE2, "We're done");
     PlayTone(TONE_A3, 1000);
     PlayTone(TONE_D4, 1000);
     PlayTone(TONE_A3, 1000);
     PlayTone(TONE_D4, 2000);
     Wait(10000);
}

