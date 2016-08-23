// WebServer Interface Arduino Orbitron Nanikana-Aerospace 2015
#include <SPI.h>
#include <Ethernet.h>

// MAC address from Ethernet shield sticker under board
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192,168,100,177); // IP address, may need to change depending on network
EthernetServer server(8001);  // create a server at port 8001

String readString; // We will be using strings to keep track of things.
    int satazimuth = 0;
    int satelevation = 0;
    int antazimuth = 0;
    int antelevation =0;
    int antstatus =0;
    int antmovement =0;

void setup()
{
  pinMode(2, OUTPUT); //Lights ON Sw
  pinMode(3, OUTPUT); // To Arduino-Orbitron MANUAL/AUTOMATIC Mode Sw
  pinMode(4, OUTPUT); // AUX 1
  pinMode(5, OUTPUT); // AUX 2
  Serial.begin(9600);  
  Ethernet.begin(mac, ip);  // initialize Ethernet device
    server.begin();           // start to listen for clients
    
}

void loop()
{
  
    
    
    
    if (Serial.find("Z")){
      satazimuth = Serial.parseInt();
      satelevation = Serial.parseInt();
      antazimuth = Serial.parseInt();
      antelevation = Serial.parseInt();
      antstatus = Serial.parseInt ();
      antmovement = Serial.parseInt();}
      
    EthernetClient client = server.available();  // try to get client

    if (client) {  // got client?
        boolean currentLineIsBlank = true;
        while (client.connected()) {
            if (client.available()) {   // client data available to read
                char c = client.read(); // read 1 byte (character) from client
                // last line of client request is blank and ends with \n
                // respond to client only after last line received
                
                if (readString.length() < 100) {
          readString += c;
        } 
                
                if (c == '\n' && currentLineIsBlank) {
                    // send a standard http response header
                    Serial.println(readString);                      // And here we begin including the HTML
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-Type: text/html");
                    client.println("Connection: close");
                    client.println();
                    // send web page
                    client.println("<!DOCTYPE html>");
                    client.println("<html>");
                    client.println("<head>");
                    client.println("<title>Nanikana-Aerospace AAT</title>");
                    //client.println("<h1>Nanikana-Aerospace Antenna Track</h1>");
                    client.println("</head>");
                    client.println("<meta http-equiv=\"refresh\" content=\"30\">"); // This is used to refresh the page so
                    client.println("<body>");
                    
      client.print("<BR>   SAT AZ : ");
      client.println(satazimuth);
      client.print("  SAT EL: ");
      client.println(satelevation);
      client.print("</BR><BR>  ANT AZ : ");
      client.println(antazimuth);
      client.print("   ANT EL : ");
      client.println(antelevation);
      client.print("</BR><BR>   STATUS : ");
       
      client.println(antstatus);
      client.print("   INFO  : ");
      client.println(antmovement);
      client.println("</BR>   ");
      
      
     // client.println("<p>");
//client.println("<FORM>");
//client.println("<INPUT type=button value=LIGHTS-ON onClick=window.location='/?lighton1\'>");
//client.println("<INPUT type=button value=LIGHTS-OFF onClick=window.location='/?lightoff1\'>");
//client.println("</FORM>");   // Above and below you'll see that when we click on a button, it adds a little snippet
//client.println("<FORM>");    // to the end of the URL. The Arduino watches for this and acts accordingly.
//client.println("<INPUT type=button value=MANUAL onClick=window.location='/?lighton2\'>");
//client.println("<INPUT type=button value=AUTOM. onClick=window.location='/?lightoff2\'>");

//client.println("</FORM>");

                    client.println("</body>");
                    client.println("</html>");
                    
                    //Serial.print(satazimuth); //DEBUG SERIAL OUT
                   // Serial.print(satelevation);
                    //Serial.print(antazimuth);
                    //Serial.print(antelevation);
                    //Serial.print(antstatus);
                    //Serial.println(antmovement);
                    
                    break;
                }
                // every line of text received from the client ends with \r\n
                if (c == '\n') {
                    // last character on line of received text
                    // starting new line with next character read
                    currentLineIsBlank = true;
                } 
                else if (c != '\r') {
                    // a text character was received from client
                    currentLineIsBlank = false;
                }
            } // end if (client.available())
        } // end while (client.connected())
        delay(1);      // give the web browser time to receive the data
       // if(readString.indexOf("?lighton1") >0)     // these are the snippets the Arduino is watching for.
        //  {
        //    digitalWrite(2, HIGH);
        //  }
        //  else{
        //  if(readString.indexOf("?lightoff1") >0)
         // {
          //  digitalWrite(2, LOW);
         // }
          
          //else{
          //  if(readString.indexOf("?lighton2") >0)
          //  {
          //    digitalWrite(3, HIGH);
          //  }
          
          //else{
          //  if(readString.indexOf("?lightoff2") >0)
          //  {
           //   digitalWrite(3, LOW);
           // }
          
         // else{
          //  if(readString.indexOf("?serv90") >0)
          //  {
          //    myservo.write(90);
          //  }
         // }
         // }
        //  }
        //  }
         // readString = "";
        client.stop(); // close the connection
    } // end if (client)
}
