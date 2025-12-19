#include <Arduino.h>
#include <Ethernet.h>
#include <WiFi.h>
#include <ArduinoModbus.h>
#include <ArduinoRS485.h>
#include <SPI.h>
#include "FlashIAPLimits.h"
#include <FlashIAP.h>
#include <FlashIAPBlockDevice.h>
#include "webpage.h"

//pin collegati
#define LED_BUILTIN PI_0 //led integrato sulla scheda
#define LED_BUILTIN_2 PI_1 //secondo led integrato sulla scheda
#define SENSOR_PIN A0 //input dal sensore POSTERIORE I1
#define SENSOR_PIN_2 A1// input dal sensore ANTERIORE I2

//variabili globali
unsigned long contatore = 0; //variabile per il conteggio degli impulsi del sensore
int front_sensorstate = LOW;
int front_precsensorstate = LOW;
int back_sensorstate = LOW;
int back_precsensorstate = LOW;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 200; // tempo di debouncing in ms
bool front_state = false; //variabile di toggle
bool back_state = false; //variabile di toggle
int status = WL_IDLE_STATUS; // variabile per lo stato della connessione. WL_IDLE_STATUS significa che non è connesso
bool connectionType; //true = WIFI, false = ETHERNET

void rilevamento_anteriore();
void rilevamento_posteriore();
void backToggle();
void frontToggle();
void WriteDebounce(unsigned long debounceDelay);
bool ethConnectionTest(byte mac[], IPAddress ip);
bool wifiConnectionTest(char ssid[], char pass[]);

//ETHERNET
//Definisco l'indirizzo IP del server Modbus
IPAddress ip(192, 168, 200, 170); // Indirizzo IP statico del server Modbus
byte mac[] = {0xA0, 0xCD, 0xF3, 0xB1, 0xEC, 0x1E};
EthernetServer ethServer(502); // Porta standard Modbus TCP è 502
EthernetServer WebServer(80);  // Server Web (porta 80)
EthernetClient WebClient; //client Web Ethernet

//WIFI
char ssid[] = "Galaxy S25 711A";
char pass[] = "HakunaMatata";
WiFiServer wifiServer(502); // Porta standard Modbus TCP è 502
WiFiServer wifiWebServer(80);  // Server Web (porta 80)
WiFiClient wifiWebClient; //client Web WiFi
static WiFiClient modbusClient;

ModbusTCPServer modbusTCPServer; // Creazione del server Modbus TCP (ethServer invece è il server Ethernet di base)

void setup() {
  // Inizializzo la comunicazionne seriale a 9600 baud
  Serial.begin(9600);
  // while(!Serial); //attendo che la seriale sia pronta
  delay(500);
  Serial.print("Ciao! Inizializzo la connessione...");
  connectionType = true;
  if(connectionType) { // arduino OPTA (STM32H7 + mbed OS)
  /* non controlla il cavo o il link, ma solo se il driver Ethernet è disponibile a livello hardware 
    (cioè se il microcontrollore STM32 ha un controller MAC attivo e il PHY risponde via MDIO). 
    Su OPTA il controllore LAN8742A è sempre presente anche se il cavo non è collegato, motivo per cui 
    Ethernet.hardwareStatus() ritorna sempre. If entra sempre nel blocco.
  */
    if(wifiConnectionTest(ssid, pass)){
      Serial.println("Connessione WiFi riuscita!");
      connectionType = true;
    } else {
      Serial.println("Connessione WiFi fallita, provo con Ethernet...");
      connectionType = false;
    }
  }
  if(!connectionType){
    Serial.println("Connessione con Ethernet");
    Serial.println("Assicurarsi che il cavo Ethernet sia collegato...");
    delay(5000);
    if(ethConnectionTest(mac, ip)){
      Serial.println("Connessione Ethernet riuscita!");
      connectionType = false;
    } else {
      Serial.println("Connessione Ethernet fallita, non posso continuare.");
      while(1);
    }
  }
  //}
  Serial.print("Connesso alla rete!");

  if(!modbusTCPServer.begin()){ // Avvio il server Modbus TCP
    Serial.println("Impossibile avviare il server Modbus TCP!");
    Serial.println("Server Ethernet avviato!");
    while(1);
  }

  //configuro 2 registri di tipo Coils all'indirizzo 0
  if(!modbusTCPServer.configureCoils(0x00, 2)){
    Serial.println("Impossibile configurare i registri di tipo COILS!");
    while(1);
  } 

  pinMode(LED_BUILTIN, OUTPUT); //imposto il pin del led come output
  pinMode(SENSOR_PIN, INPUT); //imposto il pin del sensore come input
  pinMode(LED_BUILTIN_2, OUTPUT); //imposto il pin del secondo led come output
  pinMode(SENSOR_PIN_2, INPUT); //imposto il pin del secondo sensore come input
  Serial.println("Stato connectionType (0=ETH, 1=WIFI): ");
  Serial.println(connectionType);
}

void loop() {
  if(connectionType == 0){ //ETHERNET
    EthernetClient client = ethServer.available();

    if(client){
      modbusTCPServer.accept(client); // "passo" il server Ethernet al server Modbus TCP
      while(client.connected()){
        //polling per le richieste Modbus mentre il client è connesso
        modbusTCPServer.poll();
        //aggiorno il registro coil frontale con lo stato del toggle
        modbusTCPServer.coilWrite(0x00, front_state);
        //aggiorno il registro coil posteriore con lo stato del toggle
        modbusTCPServer.coilWrite(0x01, back_state);
        //controllo i sensori
        rilevamento_anteriore();
        rilevamento_posteriore();

        WebClient = WebServer.accept();

        // Qui gestisco il client WEB

        if(WebClient){
          Serial.println("Nuovo Client connesso");
          String request = "";
          boolean currentLineIsBlank = true; // Variabile per tenere traccia delle linee vuote
          while(WebClient.connected()){
              if(WebClient.available()){
                  char c = WebClient.read(); // Leggo un carattere dal client
                  // Fine dell'header HTTP è indicata da una linea vuota
                  request += c;
                  if (c == '\n' && currentLineIsBlank) {
                      // Invio della risposta HTTP
                      WebClient.println("HTTP/1.1 200 OK");
                      WebClient.println("Content-Type: text/html");
                      WebClient.println("Connection: close");
                      WebClient.println();
                      WebClient.println(generateHTML(debounceDelay));
                      break;
                  }
                  if (c == '\n') {
                      currentLineIsBlank = true; // Inizio di una nuova linea
                  } else if (c != '\r') {
                      currentLineIsBlank = false; // Carattere diverso da '\r', quindi la linea non è vuota
                  }
                  if(request.indexOf("GET /?debounce=") >= 0){
                    auto precdebounceDelay = debounceDelay;
                    int startIndex = request.indexOf("debounce=") + 9;
                    int endIndex = request.indexOf(" ", startIndex); //trovo lo spazio dopo il valore
                    String debounceValueStr = request.substring(startIndex, endIndex);
                    debounceDelay = debounceValueStr.toInt();
                    Serial.print("Nuovo valore di debounceDelay: ");
                    Serial.println(debounceDelay);
                    
                    if(debounceDelay != precdebounceDelay) WriteDebounce(debounceDelay);
                  }
              }
            }
          WebClient.stop();
        }
      }
      client.stop(); // Chiudo la connessione con il client Modbus
    }
  }

  if(connectionType == 1){ //WIFI
    if(!modbusClient || !modbusClient.connected()) {
      //Se non c'è un client modbus attivo, ne accetto uno nuovo
      WiFiClient newClient = wifiServer.accept();
      if(newClient){
        modbusClient.stop(); //chiudo eventuale client precedente
        modbusClient = newClient;
        modbusTCPServer.accept(modbusClient); // "passo" il client WiFi al server Modbus TCP
      }
    }
    modbusTCPServer.poll();
    modbusTCPServer.coilWrite(0x00, front_state);
    modbusTCPServer.coilWrite(0x01, back_state);
    rilevamento_anteriore();
    rilevamento_posteriore();

    wifiWebClient = wifiWebServer.available();

    if(wifiWebClient){
      Serial.println("Nuovo Client connesso");
        String request = "";
        boolean currentLineIsBlank = true; // Variabile per tenere traccia delle linee vuote
        while(wifiWebClient.connected()){
            if(wifiWebClient.available()){
                char c = wifiWebClient.read(); // Leggo un carattere dal client
                // Fine dell'header HTTP è indicata da una linea vuota
                request += c;
                if (c == '\n' && currentLineIsBlank) {
                    // Invio della risposta HTTP
                    wifiWebClient.println("HTTP/1.1 200 OK");
                    wifiWebClient.println("Content-Type: text/html");
                    wifiWebClient.println("Connection: close");
                    wifiWebClient.println();
                    wifiWebClient.println(generateHTML(debounceDelay));
                    break;
                }
                if (c == '\n') {
                    currentLineIsBlank = true; // Inizio di una nuova linea
                } else if (c != '\r') {
                    currentLineIsBlank = false; // Carattere diverso da '\r', quindi la linea non è vuota
                }
                if(request.indexOf("GET /?debounce=") >= 0){
                  auto precdebounceDelay = debounceDelay;
                  int startIndex = request.indexOf("debounce=") + 9;
                  int endIndex = request.indexOf(" ", startIndex); //trovo lo spazio dopo il valore
                  String debounceValueStr = request.substring(startIndex, endIndex);
                  debounceDelay = debounceValueStr.toInt();
                  Serial.print("Nuovo valore di debounceDelay: ");
                  Serial.println(debounceDelay);
                  
                  if(debounceDelay != precdebounceDelay) WriteDebounce(debounceDelay);
                }
            }
        }
    }
  }
  yield(); //funzione di sistema che cede il controllo del processore al altri task interni o gestori di sistema
  //dovrebbe mantenere vivo il wifi stack
}

void rilevamento_anteriore(){
  int lettura = digitalRead(SENSOR_PIN_2); //leggo lo stato del sensore
  if (lettura != front_precsensorstate){ //se lo stato corrente del sensore è diverso rispetto all'ultimo stato registrato
    lastDebounceTime = millis();   //resetto il timer
  }
  
  //se è passato abbastanza tempo senza cambiamenti
  if ((millis() - lastDebounceTime) > debounceDelay){
    //aggiorno lo stato solo se è cambiato davvero
    if(lettura != front_sensorstate){
      front_sensorstate = lettura;
        if (front_sensorstate == HIGH) { //rilevo solo i fronti di salita
        contatore ++;
        //Serial.print("Impulso rilevato, conteggio: ");
        //Serial.print(contatore);
        digitalWrite(LED_BUILTIN_2, HIGH); //accende il led
        } else {
          digitalWrite(LED_BUILTIN_2, LOW); //spegne il led
          frontToggle();
        }
    }
  }
  front_precsensorstate = lettura; 
}

void rilevamento_posteriore(){
  int lettura = digitalRead(SENSOR_PIN); //leggo lo stato del sensore
  if (lettura != back_precsensorstate){ //se lo stato corrente del sensore è diverso rispetto all'ultimo stato registrato
    lastDebounceTime = millis();   //resetto il timer
  }
  
  //se è passato abbastanza tempo senza cambiamenti
  if ((millis() - lastDebounceTime) > debounceDelay){
    //aggiorno lo stato solo se è cambiato davvero
    if(lettura != back_sensorstate){
      back_sensorstate = lettura;
        if (back_sensorstate == HIGH) { //rilevo solo i fronti di salita
        contatore ++;
        //Serial.print("Impulso rilevato, conteggio: ");
        //Serial.print(contatore);
        digitalWrite(LED_BUILTIN, HIGH); //accende il led
        } else {
          digitalWrite(LED_BUILTIN, LOW); //spegne il led
          backToggle();
        }
    }
  }
  back_precsensorstate = lettura; 
}

void frontToggle(){
  front_state =! front_state;
}

void backToggle(){
  back_state =! back_state;
}

void WriteDebounce(unsigned long debounceDelay){
    auto [flashSize, startAddress, iapSize] = getFlashIAPLimits();
    //Creo un block device nella memoria flash disponibile
    FlashIAPBlockDevice blockDevice(startAddress, iapSize);
    //Prima di poterlo usare si deve inizializzare 
    blockDevice.init();

    const auto eraseBlockSize = blockDevice.get_erase_size(); // dimensione del blocco di cancellazione
    const auto programBlockSize = blockDevice.get_program_size(); // dimensione del blocco di programmazione

    //Calcolo il numero di bytes necessari per salvare il debounceDelay
    const auto messageSize = sizeof(debounceDelay);
    const unsigned int requiredEraseBlocks = ceil(messageSize / (float)eraseBlockSize); //numero di blocchi di cancellazione necessari
    const unsigned int requiredProgramBlocks = ceil(messageSize / (float) programBlockSize); //numero di blocchi di programmazione necessari
    const auto dataSize = requiredProgramBlocks * programBlockSize; //dimensione totale dei dati da scrivere
    unsigned long buffer[dataSize]; //buffer per i dati

    Serial.println("Lettura del valore precedente di debounceDelay dalla memoria flash...");
    blockDevice.read(buffer, 0, dataSize);
    Serial.println(*(unsigned long*)buffer);

    //Cancello un blocco iniziando iniziando dall'indirizzo 0 relativamente all'inizio del block device
    blockDevice.erase(0, requiredEraseBlocks * eraseBlockSize);
    Serial.println("Cancellazione del blocco di memoria completata.");

    //Scrivo il nuovo valore di debounceDelay nella memoria flash
    Serial.println("Scrittura del nuovo valore di debounceDelay nella memoria flash...");
    blockDevice.program((char*)&debounceDelay, 0, dataSize);
    Serial.println("Scrittura completata.");

    // Leggo di nuovo per verificare
    blockDevice.read(buffer, 0, dataSize);
    Serial.println("Valore letto dalla memoria flash:");
    Serial.println(*(unsigned long*)buffer);

    // Deinizializzo il block device
    blockDevice.deinit();
    Serial.println("FATTO!");
}

bool ethConnectionTest(byte mac[], IPAddress ip){
  Serial.println(" INSERIRE CAVO DI RETE E ATTENDERE...");

  // Inizializzo la connessione Ethernet con l'IP statico
  Ethernet.begin(mac, ip);

  Serial.println("Cavo inserito!");
  unsigned long start = millis();
  //Provo a stabilire la connessione per 3 secondi
  while(millis() - start < 3000){
    if(Ethernet.localIP() != IPAddress(0,0,0,0)){
      // Se otteniamo un IP valido, la connessione Ethernet è attiva
      Serial.print("Ethernet connessa, IP: ");
      Serial.println(Ethernet.localIP());
      //avvio il server ethrenet
      ethServer.begin();
      //avvio il server Web ethernet
      WebServer.begin();
      return true;
    }
    delay(200);
  }
  //Se dopo 3 secondi non abbiamo ottenuto un IP valido, la connessione fallisce
  return false;
}

bool wifiConnectionTest(char ssid[], char pass[]){
  Serial.println("Connessione con WiFi");
  contatore = 5;
    while(status != WL_CONNECTED){ //finchè non sono connesso alla rete
      Serial.print("Tentativo di connessione alla rete: ");
      Serial.println(ssid);
      status = WiFi.begin(ssid, pass); //avvio la connessione WiFi (con DHCP abilitato)
      contatore--;
      if(contatore == 0){
        return false;
      }
      delay(1000);
    }
    wifiServer.begin(); //Avvio il server WiFi
    wifiWebServer.begin(); //Avvio il Web Server WiFi
    return true;
}