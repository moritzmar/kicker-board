#include <RadioHead.h>
#include <radio_config_Si4460.h>
#include <RHCRC.h>
#include <RHDatagram.h>
#include <RHGenericDriver.h>
#include <RHGenericSPI.h>
#include <RHHardwareSPI.h>
#include <RHMesh.h>
#include <RHNRFSPIDriver.h>
#include <RHReliableDatagram.h>
#include <RHRouter.h>
#include <RHSoftwareSPI.h>
#include <RHSPIDriver.h>
#include <RHTcpProtocol.h>
#include <RH_ASK.h>
#include <RH_RF24.h>
#include <RH_Serial.h>
#include <RH_TCP.h>
/****************************************************************
Serverprogramm - Master
Sende- und Empfangsfunktion

30.06.2015
Vers. 1

FS
*****************************************************************/
#include <SoftwareSerial.h>
#include <SPI.h>
#include <TimerOne.h>


#define RFM24_SS 6
#define RFM24_MOSI 11
#define RFM24_MISO 12 
#define RFM24_IRQ 2
#define RFM24_SDN 5
RH_RF24 rf24 = RH_RF24(RFM24_SS, RFM24_IRQ, RFM24_SDN); //Initialize a new instance of the RadioHead RFM24W driver as rf24

unsigned char Master_address = 0x01;
unsigned char slave_a_address = 0xBE;
unsigned char slave_b_address = 0xEF;
int CallSlave_a = 1;
int CallSlave_b = 0;
uint8_t AddressTemp, Data1Temp, Data2Temp, crc8Temp, Address, Data1, Data2, crc8;
uint8_t Package[4]={};

void setup(void){
  Timer1.initialize(2500000); //5000000 entspricht 2.5 sec
  Timer1.attachInterrupt(SendCall); // SendCall-Funktion zum Senden der Anfrage an die Slaves; soll alle 5 Sekunden gestartet werden
  rf24.setModemConfig(RH_RF24::GFSK_Rb5Fd10);
  rf24.setTxPower(0x4f);
  Serial.begin(9600);
  if (!rf24.init()){ //Initialisierung des Funkmoduls fehlgeschlagen?
    Serial.println("Init failed");
  }  
}

void rfm24_on() {
  rf24.setModeIdle();
  rf24.setFrequency(433.000);
}

//volatile unsigned long IrgendEineVariableDieManAusInterruptHerausHabenWill = 0; // volatile wird genutzt, um Daten aus dem Interrupt in Dauerschleife übergeben zu können
volatile int SendSuccess = 0;
//Interrupt-Funktion zum Aufruf der Master-Sendefunktion
void SendCall()    //void SendCall(void)
{
int SendToSlave_aOK = 0;
int SendToSlave_bOK = 0;

  if (CallSlave_a == 1) {        //Ansprechen von Slave_a durch Senden des Pakets mit der entsprechenden Adresse 0xBE
    Address = slave_a_address;
    Package [0] = Address;
    Package [1] = Data1;
    Package [2] = Data2;
    Package [3] = crc8;
    rf24.send(Package, sizeof(Package)); // groesse des Pakets ist immer 4 Byte
    rf24.waitPacketSent();               // wartet auf Rueckmeldung des Moduls ueber die gesendeten Daten
    Serial.println("Master sent Package to Slave_a.");
    SendToSlave_aOK = 1;
    SendSuccess = 1;              //Variable fuer Funktion, die true als Rueckgabewert geben kann
    SendSuccessFunc(SendSuccess); //Aufruf der Funktion, um true fuer das erfolgreiche Senden zu uebergeben     
 } 
  else if (CallSlave_b == 1) {    //Ansprechen von Slave_b durch Senden des Pakets mit der entsprechenden Adresse 0xEF
    Address = slave_b_address;
    Package [0] = Address;
    Package [1] = Data1;
    Package [2] = Data2;
    Package [3] = crc8;
    rf24.send(Package, sizeof(Package)); // groesse des Pakets ist immer 4 Byte
    rf24.waitPacketSent();               // wartet auf Rueckmeldung des Moduls ueber die gesendeten Daten
    Serial.println("Master sent Package to Slave_b.");
    SendToSlave_bOK = 1;
    SendSuccess = 1;              //Variable fuer Funktion, die true als Rueckgabewert geben kann
    SendSuccessFunc(SendSuccess); //Aufruf der Funktion, um true fuer das erfolgreiche Senden zu uebergeben  
 } 
   else{
   Serial.println("No Slave to call selected.");
   SendSuccessFunc(SendSuccess);  //Aufruf der Funktion, um false fuer das nicht erfolgreiche Senden zu uebergeben
   }

//Tauschen des anzusprechenden Slaves   
  if (SendToSlave_aOK == 1) { 
    CallSlave_a = 0;
    CallSlave_b = 1;
  }
  else if (SendToSlave_bOK == 1) { 
    CallSlave_a = 1;
    CallSlave_b = 0;
  }
  else{
  Serial.println("No Message successfully sent.");
  }
}  // Ende der Sendefunktion SendCall()

//Funktion zur Uebergabe von true oder false an das Gesamtprogramm
bool SendSuccessFunc(int SendSuccess){
   if (SendSuccess = 1){
       return true;
       SendSuccess = 0;
   }
   else return false;
}  

/*****************************************************************************
Ende Sendefunktion
Beginn Dauerschleife mit Empfangsfunktion
*****************************************************************************/
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
    }
  }
  else{
    Serial.print("Master wartet auf Daten. ");
    }
} //End void loop
