
//Antenna dolgai - RadioRecive_0005_irg vagy hasonló nevű teszt progi az alap
//A 8-as nagyon faszán működik, itt némi átiratt, hogy a i2c kommunikáció jobban menjen
//A 9-es is nagyon jó, ez már inkább stilisztika
#include <RF24.h>
//********************************************************
//GPS dolgai V08-as alap progi
#include <SoftwareSerial.h>
//********************************************************
//MPU6050 ...test_modded_szep_plotter_V1
#include <Wire.h>
//********************************************************

//****************************************************************************************
//**********************************Konstansok******************************************
//****************************************************************************************

//========================================================
//Antenna változók/konstansok
RF24 radio(9, 10); // CE, CSN
const byte address[6] = "00001"; //rádió címe
struct Data_Package //Data "formátum", amiben a távirányító küldi a dolgokat
{
  byte passKEY;
  byte j1_x;
  byte j1_y;
  byte j1_b;
  byte j2_x;
  byte j2_y;
  byte j2_b;
  byte empty;
};
//Megmondjuk a programnak, hogy ezek változó dolgok, ne akarjon okoskodni a fordító
volatile Data_Package data;
volatile bool newData = false; //Interrupthoz, hogy új adat-e
const byte MAXRADIOIDLETIME = 60; //kb 10 ~= 1 sec, ha 6 másodpercig nem kap jelet cisnáljon valamit
byte radioIdleCounter = 0;
bool emergencyStopRad; //megszűnt rádió vétel
//********************************************************
//GPS változók, konstansok
static const byte RXPin = 3, TXPin = 4;
static const uint16_t Baud = 9600;
SoftwareSerial ss(RXPin, TXPin); //komunikációs vonal a gps-szel
byte serialLength = 0;
//********************************************************
//Accel/Gyro konstansok, változók
//nem kell
//********************************************************
//Voltage measuring
const uint16_t CRITICALL_CELL_VOLTAGE_1 = 836; //~ 3,73 V, ha Aref 4,57 !!!!!!!!!!!!!!!!!!!!!!!!!
const uint16_t CRITICALL_CELL_VOLTAGE_PER_DIVIDED_CELL = 232; //~leosztott feszültség, ami megfelel a 3,73 V-nak
//ha Aref = 4,57 V !!!!!!!!!!!!!!!!!!!!!!!!!
//végső projektnél majd teszteld, hogy annyi-e (valszeg felkúszik 5-re, akkor excelben ezeket
//ÁT KELL SZÁMOLNI!
//********************************************************

//Továbbítandó adatok i2C-n
byte memsCHUNK[12];
bool emergencyStopBat; //alacsony aksi feszültség
byte highB;
byte lowB; 

//****************************************************************************************
//**********************************SETUP*************************************************
//****************************************************************************************

void setup() 
{
  Serial.begin(Baud); 
  // Rádió alap beállításai:
TRYAGAIN:
  if (!radio.begin()) 
  {
    Serial.println(F("Radio hardware not responding!"));
    delay(125);
    goto TRYAGAIN;
  }
  radio.openReadingPipe(1, address);   //Setting the address at which we will receive the data
  radio.setPALevel(RF24_PA_MIN); 
  radio.setDataRate(RF24_1MBPS); //250 kbs bőven elég lenne, de azt csak a + tudja elvileg (hardware)
  radio.setChannel(77);
  radio.setPayloadSize(8); // 7 db van, de legyen szép szám
  radio.startListening();              //This sets the module as receiver
  Serial.println(F("Attaching interrupt!"));
  attachInterrupt(digitalPinToInterrupt(2), ISR_icomingSignal, FALLING);
  Serial.println(F("Ready!"));
  delay(250);
  //********************************************************
  //GPS alap beállításai
  //Rahedli GPS chip beállítás, hogy ne szemetelje a buffert felesleges dolgokkal
  const char* nmeaData1 = "$PUBX,40,RMC,0,0,0,0,0,0";
  const char* nmeaData2 = "$PUBX,40,GLL,0,0,0,0,0,0";
  const char* nmeaData3 = "$PUBX,40,GSV,0,0,0,0,0,0";
  const char* nmeaData4 = "$PUBX,40,GSA,0,0,0,0,0,0";
  const char* nmeaData5 = "$PUBX,40,VTG,0,0,0,0,0,0";
  const char* nmeaData6 = "$PUBX,40,ZDA,0,0,0,0,0,0"; //"PUBX,40,GGA,0,1,0,0,0,0"
  const char* nmeaData7 = "$PUBX,40,GGA,0,1,0,0,0,0"; //ENABLE
  Serial.println(F("The following command is going to be sent to GPS:"));
  ss.begin(Baud);
  delay(100);
  nmeaSentenceCreator(nmeaData1, ss);
  nmeaSentenceCreator(nmeaData2, ss);
  nmeaSentenceCreator(nmeaData3, ss);
  nmeaSentenceCreator(nmeaData4, ss);
  nmeaSentenceCreator(nmeaData5, ss);
  nmeaSentenceCreator(nmeaData6, ss);
  nmeaSentenceCreator(nmeaData7, ss);
  Serial.println(F("Messages Sent."));
  //********************************************************
  //MPU6050
  Wire.begin(); 
  MPU6050_Write(0x6B, 0b10000000); //Reset-eli a MEMS chipet
  delay(100);
  Awaken(); //Felébreszti a MEMS chipet, ha kell, bár elvileg a reset ezt kiüti, biztos, ami sicher....
  //Regiszterekbe beállítások átirkálása
  MPU6050_Write(0x1B, 0b00011000); //± 2000 °/s - max felbontás
  MPU6050_Write(0x1C, 0b00011000); //± 16g      - max felbontás
  MPU6050_Write(0x19, 0b00000000); //mintavételezés gyakorisága (legsűrűbb)
  MPU6050_Write(0x1A, 0b00000000); //8 kHz helyett 1 kHz-re lehet állítani, a 0 a 8 kHz-nek felel meg
  //********************************************************
  //A feszültség méréshez nem kell ide semmi különöset tenni.
  //********************************************************
  //Kommunikáció a másik NANO-val
  //Ahhoz se kell ide semmi

}

//****************************************************************************************
//**********************************LOOP**************************************************
//****************************************************************************************
void loop()
{
  //PRIO 1: Rádióból jövő adatok
  ELEJE:
  if (newData)
    {
          //Reading the data
      if (data.passKEY != 88)
        {
          Serial.println(F("Not good pass key. Someoneelse is probably using same channel."));
          newData = false;
        }
      Serial.print(F("passKey: "));
      Serial.print(data.passKEY);
      Serial.print(F(" | j1_x: "));
      Serial.print(data.j1_x);
      Serial.print(F(" | j1_y: "));
      Serial.print(data.j1_y);
      Serial.print(F(" | j1_b: "));
      Serial.print(data.j1_b);
    
      Serial.print(F(" | j2_x: "));
      Serial.print(data.j2_x);
      Serial.print(F(" | j2_y: "));
      Serial.print(data.j2_y);
      Serial.print(F(" | j2_b: "));
      Serial.println(data.j2_b);
      radioIdleCounter = 0; //nullázuk a számlálót
      emergencyStopRad = false; //ha visszajön a jel
      newData = false;     
    }
    else //ha nincs új adat
    {  
      radioIdleCounter++;
      Serial.println(F("NO NEW RADIO.")); //only while testing
      if (radioIdleCounter > MAXRADIOIDLETIME)
      {
        emergencyStopRad = true;
      }
    }

    //Végeztünk az antenna kód részével
    //****************************************************************************************************************************
    //****************************************************************************************************************************
    //Gyorsulás és Gyro adatok beolvasása
    Serial.write("MEMS: ");

    //A memsCHUNK nevű tömb feltöltése a gyorsulási adatokkal:
    
    MPU6050_Read(0x3B, &memsCHUNK[0]); //Ax High
    MPU6050_Read(0x3C, &memsCHUNK[1]); //Ax Low
    MPU6050_Read(0x3D, &memsCHUNK[2]); //Ay High
    MPU6050_Read(0x3E, &memsCHUNK[3]); //Ay Low
    MPU6050_Read(0x3F, &memsCHUNK[4]); //Az High
    MPU6050_Read(0x40, &memsCHUNK[5]); //Az Low
    MPU6050_Read(0x43, &memsCHUNK[6]); //Gx High
    MPU6050_Read(0x44, &memsCHUNK[7]); //Gx Low
    MPU6050_Read(0x45, &memsCHUNK[8]); //Gy High
    MPU6050_Read(0x46, &memsCHUNK[9]); //Gy Low
    MPU6050_Read(0x47, &memsCHUNK[10]); //Gz High
    MPU6050_Read(0x48, &memsCHUNK[11]); //Gz Low

    //test
    for(byte i = 0; i<12; i++)
    {
      Serial.print(memsCHUNK[i]);
      Serial.print("\t");
    }

    Serial.write("\r\n");

    //****************************************************************************************************************************
    //****************************************************************************************************************************
    //Adatok kiírása i2c-re
      Wire.beginTransmission(0xF2); //mems chip mindig lezárásra kerül
    
      Wire.write(0); //vészleállítás?
      Wire.write(emergencyStopRad);
      Wire.write(emergencyStopBat);
      Wire.endTransmission(true); //max 32 bájtot bír átküldeni egy adagban //ink legyenek szépen elkülönítve
      delay(2);

      Wire.beginTransmission(0xF2);
      Wire.write(128); //Rádio adatok
      Wire.write(data.j1_x);
      Wire.write(data.j1_y);
      Wire.write(data.j1_b);
      Wire.write(data.j2_x);
      Wire.write(data.j2_y);
      Wire.write(data.j2_b);
      Wire.endTransmission(true); //max 32 bájtot bír átküldeni egy adagban
      delay(2);
      
      Wire.beginTransmission(0xF2);
      Wire.write(255);//mems adatok
      Wire.write(memsCHUNK, 12);
      Wire.endTransmission(true);
//****************************************************************************************************************************
//****************************************************************************************************************************
    //GPS adatok beszedése
    serialLength = ss.available(); //Megnézzük mennyi adat van az rx pin (GPS) bufferen
    if (serialLength > 0)
    {
      //ha több, mint 0 akkor van adat azt beírjuk BYTE-onként egy byte tömbbe.
      byte gpsHolder[serialLength]; //char-ral nem akart rendesen működni, maradnak a számok
      for(byte i = 0; i<serialLength; i++)
      {
        gpsHolder[i] = ss.read();
      }
      //ellenörző kiiratás WRITE-tal
      for(byte i = 0; i<serialLength; i++)
      {
        Serial.write(gpsHolder[i]);
      }
      Serial.write("\r\n");
    }
    
    //**********************************************************************************************************************************
    //**********************************************************************************************************************************
    //Feszültség mérés, ha kell vészleállítás a következő körben
    
    if (analogRead(A0) < CRITICALL_CELL_VOLTAGE_1) //836 ~3,73 V
      EmergencyStop(1);
    if (analogRead(A1)/2 < CRITICALL_CELL_VOLTAGE_PER_DIVIDED_CELL)
      EmergencyStop(2); //Reménykedünk, hogy a cella_1 és cella_2 nagyjából egyszerre merül
    //Egyébként, ha nagyon komolyan kéne venni, ide muszáj lenne a float használata a hardweres kialakítás miatt
    if (analogRead(A2)-analogRead(A1) < CRITICALL_CELL_VOLTAGE_PER_DIVIDED_CELL)
      EmergencyStop(3);
    if (analogRead(A3)-analogRead(A2) < CRITICALL_CELL_VOLTAGE_PER_DIVIDED_CELL)
      EmergencyStop(4);
    //Ezért a két darabért nem éri meg for loopot írni szerintem

    //Serialra kiírás
    Serial.print(F("Voltage: \t"));
    for (byte i = 0; i<4; i++)
    {
      Serial.print(analogRead(i)); //Ha csak az analogRead()-et használod, akkor nem kell kiírni, hogy A0, A1.. elég a: 0,1,... 
      Serial.print("\t");
    }
    Serial.print("\n\r");
//******************************************** END OF LOOP ***********************************************
//********************************************************************************************************
//********************************************************************************************************
//********************************************************************************************************
//********************************************************************************************************
}

//****************************************************************************************
//**********************************Szub rutinok******************************************
//****************************************************************************************

//Rádióhoz
void ISR_icomingSignal() //Interrupt szubrutin
{
  radio.read(&data, radio.getPayloadSize());
  newData = true; //Megmondjuk, hogy új adatok vannak eltárolva az antennáról a data nevű változóban
}

//****************************************************************************************
//GPS-hez

void nmeaSentenceCreator(char *nmea_data, SoftwareSerial ss) //NMEA mondat generáló, hogy tudjunk parancsolni a GPS-nek
{
  char buf[32]; //annyi biztos elég...
  char buf2[3];
  
  String(nmea0183_checksum(nmea_data), HEX).toCharArray(buf2,3);
  
  strcpy(buf, nmea_data);
  strcat(buf, "*"); //ez fontos h ne a mondatban legyen kül beleszámolná a checksumba
  strcat(buf, buf2);
  strcat(buf, "\r\n");
  
  Serial.print(buf);
  ss.print(buf);
  delay(100);
}

int nmea0183_checksum(char *nmea_data) //Ellenörző bit kalkulátor
{
    int crc = 0;
    int i;
    // ignore the first $ sign,  no checksum in sentence
    for (i = 1; i < strlen(nmea_data); i ++) 
    {
      crc ^= nmea_data[i];
    }

    return crc;
}
//****************************************************************************************
//MPU6050-hez
int16_t PrintRegData(int Hadress, int Ladress) //ezt átírva eléggé
{
  byte H;
  byte L;
  MPU6050_Read(Hadress, &H);  // Get data
  MPU6050_Read(Ladress, &L);  // Get data
  int16_t data = H << 8 | L; //azért nem unsigned, mert -/+ módban üzemel!
  
  Serial.print(data);
  Serial.print("\t");
  return data;
}

void Awaken()
{
  uint8_t awakeByte;                                 // Data will go here
  MPU6050_Read(0x6B, &awakeByte);      // Get data
  boolean sleepModeOn = bitRead(awakeByte,6);        // Check if sleep mode on
  while(sleepModeOn)
  {                                // If on...
    delay(500);                                      // delay
    bitClear(awakeByte,6);                           // Clear bit of sleep
    MPU6050_Write(0x6B, 0b00000000);   // Send data back
    delay(500);                                      // delay
    MPU6050_Read(0x6B, &awakeByte);    // Get data again
    sleepModeOn = bitRead(awakeByte,6);              // If sleep mode off, exit loop
  }                                                  //
  delay(500);                                        // delay
}

void MPU6050_Read(int address,uint8_t *data)
{            // Read from MPU6050. Needs register address, data array
  int size = sizeof(*data);                              //
  Wire.beginTransmission(0x68);           // Begin talking to MPU6050
  Wire.write(address);                                   // Set register address
  Wire.endTransmission(false);                           // Hold the I2C-bus
  Wire.requestFrom(0x68, size, true);     // Request bytes, release I2C-bus after data read
  int i = 0;                                             //
  while(Wire.available())
  {                               //
    data[i++]=Wire.read();                               // Add data to array
  }
  Wire.endTransmission();
}

void MPU6050_Write(int address,byte dbytes) // Write to MPU6050. Needs register address, data array
{  
  Wire.beginTransmission(0x68);    // Begin talking to MPU6050
  Wire.write(address);                            // Set register address
  Wire.write(dbytes);                        // akkor csak byte-onként küldünk adatot
  Wire.endTransmission();                         // Release I2C-bus
}

//****************************************************************************************
//Feszültségméréshez
void EmergencyStop(byte cellNUM) //ezt kiirattam serail monitoron tesztelésnél
{
  emergencyStopBat = true;
}

//****************************************************************************************
//Kommunikáció másik i2c-vel

//Semmi
//