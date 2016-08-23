    String inData;
    void setup()
    {
     Serial.begin(9600);
    }
    void loop(){
    int satazimuth = 0;
    int satelevation = 0;
    int antazimuth = 0;
    int antelevation =0;
    int antstatus =0;
    int antmovement =0;
    
    if (Serial.find("Z")){
      satazimuth = Serial.parseInt();
      satelevation = Serial.parseInt();
      antazimuth = Serial.parseInt();
      antelevation = Serial.parseInt();
      antstatus = Serial.parseInt ();
      antmovement = Serial.parseInt();
      
      Serial.print(satazimuth);
      Serial.print("   ");
      Serial.print(satelevation);
      Serial.print("   ");
      Serial.print(antazimuth);
      Serial.print("   ");
      Serial.print(antelevation);
      Serial.print("   ");
      Serial.print(antstatus);
      Serial.print("   ");
      Serial.print(antmovement);
      Serial.println("   ");
      
      
      }
    //inData="";
        // if (Serial.available() > 0) {
           //  int h=Serial.available();   
            // if you are getting escape -characters try h--; here
        
    //for (int i=0;i<h;i++){
                // inData += (char)Serial.read();
             //}
        // if you are getting escape -characters try Serial.read(); here
        //}
        //print it out
       // Serial.println(inData);
        delay (200);
    }
