#include <RH_RF24.h>

/****************************************************************
Client-Programm - Slave
Sende- und Empfangsfunktion

30.06.2015
Vers. 1
*****************************************************************/

#include <SPI.h>
#include <TimerOne.h>
#include <util/crc16.h>


#define RFM24_SS 6
#define RFM24_MOSI 11
#define RFM24_MISO 12 
#define RFM24_IRQ 2
#define RFM24_SDN 5
RH_RF24 rf24 = RH_RF24(RFM24_SS, RFM24_IRQ, RFM24_SDN); //Initialize a new instance of the RadioHead RFM24W driver as rf24

unsigned char Master_address = 0x01;
unsigned char slave_a_address = 0xBE;  //Funktionen fuer diesen Slave
unsigned char slave_b_address = 0xEF;

uint8_t AddressTemp, Data1Temp, Data2Temp, crc8Temp, Address, Data1, Data2, crc8;
uint8_t Package[4]={};

uint8_t calc_crc8(char * dataIn, uint8_t len)// uses the ccitt generator polynom 0x07
{
  uint8_t i = 0, last_crc = 0;
  for (i = 0; i<len; i++){
    last_crc = _crc8_ccitt_update(last_crc, dataIn[i]);
  }
  return last_crc;
}

int SendSuccess = 0;
//Funktion zur Uebergabe von true oder false an das Gesamtprogramm
bool SendSuccessFunc(int SendSuccess){
   if (SendSuccess = 1){
       return true;
       SendSuccess = 0;
   }
   else return false;
}  

void setup(){
  rf24.setModemConfig(RH_RF24::GFSK_Rb5Fd10);
  rf24.setTxPower(0x4f);
        
}
void rfm24_on() {
  rf24.setModeIdle();
  rf24.setFrequency(433.000);
}
//Beginn der Dauerschleife mit staendigem Abhören des Datenkanals, ob Daten zum Empfangen vorhanden sind
void loop (){
//Empfangen einer Nachricht
  if (rf24.available()){  //Bedingung erfuellt, wenn Daten ueber den Datenkanal empfangen werden
    uint8_t buf[RH_RF24_MAX_MESSAGE_LEN]; //Speichern aller empfangenen Daten in einem Array
    uint8_t len = sizeof(buf);    //Speichern der Länge des Arrayfelds

    if (rf24.recv(buf, &len)) //wenn alle Daten empfangen sind
    {
      AddressTemp = buf[0];
      Data1Temp = buf[1];
      Data2Temp = buf[2];
      crc8Temp = buf[3];
      //Serial.print("Empfangene Daten: ");
      //Serial.println((char*)buf);
      Serial.print("Empfangene Adresse: ");
      Serial.println(AddressTemp);
      Serial.print("Empfangene Daten1: ");
      Serial.println(Data1Temp);
      Serial.print("Empfangene Daten2: ");
      Serial.println(Data2Temp);
      Serial.print("Empfangene Prüfsumme: ");
      Serial.println(crc8Temp);

      //Vergleichen der empfangenen Adresse mit der eigenen
      if (AddressTemp == slave_a_address){
        Address = Master_address;
        Package [0] = Address;
        Package [1] = Data1;
        Package [2] = Data2;
        Package [3] = crc8;
        rf24.send(Package, sizeof(Package)); // groesse des Pakets ist immer 4 Byte
        rf24.waitPacketSent();               // wartet auf Rueckmeldung des Moduls ueber die gesendeten Daten
        Serial.println("Slave A sent Package to Master.");
        SendSuccess = 1;
        SendSuccessFunc(SendSuccess);
        }
    }
  }
  else{
    Serial.print("Master wartet auf Daten. ");
    }
} //End void loop
