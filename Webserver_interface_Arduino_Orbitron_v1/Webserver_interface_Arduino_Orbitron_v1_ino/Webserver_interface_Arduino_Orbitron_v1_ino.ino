
#include <SPI.h>
#include <UIPEthernet.h>

// MAC address from Ethernet shield sticker under board
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192,168,100,177); // IP address, may need to change depending on network
EthernetServer server(80);  // create a server at port 80

    int satazimuth = 0;
    int satelevation = 0;
    int antazimuth = 0;
    int antelevation =0;
    int antstatus =0;
    int antmovement =0;

void setup()
{
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
                if (c == '\n' && currentLineIsBlank) {
                    // send a standard http response header
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
      client.print("   MOVE  : ");
      client.println(antmovement);
      client.println("</BR>   ");
                    client.println("</body>");
                    client.println("</html>");
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
        client.stop(); // close the connection
    } // end if (client)
}
