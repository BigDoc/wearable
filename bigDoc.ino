/**
 * BigDoc : wearable implementataion to collect data from sensors and send encryted health data to server
 * @author parth_shel
 * @version Nov. 5th, 2016 
 */ 
#include <SPI.h>
#include <WiFi.h>
#include<Wire.h>
//#include<Servo.h>
//#include <TinyGPS++.h>
//#include <SoftwareSerial.h>
//#include <Adafruit_GPS.h>

//Adafruit_GPS GPS(&Serial1);
#define GPSECHO  false

class Vigenere
{
    public:
       String key;
 
        Vigenere(String key)
        {
            for (int i = 0; i < key.length(); ++i)
            {
                if (key[i] >= 'A' && key[i] <= 'Z')
                    this->key += key[i];
                else if (key[i] >= 'a' && key[i] <= 'z')
                    this->key += key[i] + 'A' - 'a';
            }
        }
 
        String encrypt(String text)
        {
            String out;
 
            for (int i = 0, j = 0; i < text.length(); ++i)
            {
                char c = text[i];
 
                if (c >= 'a' && c <= 'z')
                    c += 'A' - 'a';
                else if (c < 'A' || c > 'Z')
                    continue;
 
                out += (c + key[j] - 2 * 'A') % 26 + 'A';
                j = (j + 1) % key.length();
            }
 
            return out;
        }
 
        String decrypt(String text)
        {
            String out;
 
            for (int i = 0, j = 0; i < text.length(); ++i)
            {
                char c = text[i];
 
                if (c >= 'a' && c <= 'z')
                    c += 'A' - 'a';
                else if (c < 'A' || c > 'Z')
                    continue;
 
                out += (c - key[j] + 26) % 26 + 'A';
                j = (j + 1) % key.length();
            }
 
            return out;
        }
};

const String deviceID = "bigdoc00"; //device ID of the wearable

const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

char ssid[] = "ArchHacks16"; //  your network SSID (name)
char pass[] = "";    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)

//GPS inittialization
//SoftwareSerial serial_connection(10, 11); //RX=pin 10, TX=pin 11
  //Serial1.begin(9600);
//TinyGPSPlus gps;

int status = WL_IDLE_STATUS;
// if you don't want to use DNS (and reduce your sketch size)
// use the numeric IP instead of the name for the server:
IPAddress server(128,211,192,177);  // numeric IP for Google (no DNS)
//char server[] = "www.bigdoc.me";    // name address for Google (using DNS)

// Initialize the Ethernet client library
// with the IP address and port of the server
// that you want to connect to (port 80 is default for HTTP):
WiFiClient client;

//object for Vigenere cipher encryption
Vigenere cipher("MUSTARDSNACK");


void setup() {
  load();
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  //serial_connection.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv != "1.1.0") {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid);

    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to wifi");
  printWifiStatus();

  Serial.println("\nStarting connection to server...");
}

void loop() {
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
  //data packet to send to the aerver     
    String dataPacket = "{ \"deviceID\" : " + deviceID + /*" , \"latitude\" : " + (String)gps.location.lat() + " , \"longitude\" : " + (String)gps.location.lng() + " , \"speed\" : " + (String)gps.speed.mps() + */" , \"temp\" : " + (String)(Tmp) + " , \"gyroX\" : " + (String)GyX + " \"gyroY\" : " + (String)GyY + " , \"gyroZ\" : " + (String)GyZ + " , \"accelX\" : " + (String)AcX + " , \"accelY\" : " + (String)AcY + " , \"accelZ\" : " + (String)AcZ + " }";
  //encrypt the data packet using Vigenere cipher
  dataPacket = cipher.encrypt(dataPacket);
                    
  // if you get a connection, report back via serial:
  if (client.connect(server, 8008)) {
    Serial.println("connected to server");
                   
    // Make a HTTP request:
    client.println("GET :8008/senddata HTTP/1.1");
    client.println("Host: 128.211.192.177");
    client.println("Connection: close");
    client.print("Content-Length: ");
    client.println(dataPacket.length());
    client.println();
    client.println(dataPacket);
     // read GPS data
   /*while(serial_connection.available())
  {
    gps.encode(serial_connection.read());
  }*/

  // if there are incoming bytes available
  // from the server, read them and print them:
  while (client.available()) {
    char c = client.read();
    Serial.write(c);
  }

  // if the server's disconnected, stop the client:
  if (!client.connected()) {
    Serial.println();
    Serial.println("disconnecting from server.");
    client.stop();
  }
    digitalWrite(13,HIGH);
    delay(500);
    digitalWrite(13,LOW);
        delay(3);
}
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

//LED blink load sequence
void load() {
  for(int i = 0;i < 10;i++) {
    digitalWrite(13,HIGH);
    delay(30);
    digitalWrite(13,LOW);
  }
  digitalWrite(13,HIGH);
  delay(700);
  digitalWrite(13,LOW);
  delay(1000);
  digitalWrite(13,HIGH);
  delay(200);
  digitalWrite(13,LOW);
}






