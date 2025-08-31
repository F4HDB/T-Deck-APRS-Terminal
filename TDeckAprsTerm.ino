// TDeck APRS Term v1.00
// Programme d'expérimentation du module Lilygo T-Deck.
// Par Frank (F4HDB).
// Date création : 10/06/2025.
// Dernière mise à jour : 24/07/2025.


/**** INCLUSION DES FICHIERS SUPPORTS ****/
#include <Arduino.h>
#include <RadioLib.h>
#include <esp_task_wdt.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <Preferences.h>


/**** DEFINITION DES PINS ****/
#define BOARD_POWERON       10

#define BOARD_I2S_WS        5
#define BOARD_I2S_BCK       7
#define BOARD_I2S_DOUT      6

#define BOARD_I2C_SDA       18
#define BOARD_I2C_SCL       8

#define BOARD_BAT_ADC       4

#define BOARD_TOUCH_INT     16
#define BOARD_KEYBOARD_INT  46

#define BOARD_SDCARD_CS     39
#define BOARD_TFT_CS        12
#define RADIO_CS_PIN        9

#define BOARD_TFT_DC        11
#define BOARD_TFT_BACKLIGHT 42

#define BOARD_SPI_MOSI      41
#define BOARD_SPI_MISO      38
#define BOARD_SPI_SCK       40

#define BOARD_TBOX_G02      2
#define BOARD_TBOX_G01      3
#define BOARD_TBOX_G04      1
#define BOARD_TBOX_G03      15

#define BOARD_ES7210_MCLK   48
#define BOARD_ES7210_LRCK   21
#define BOARD_ES7210_SCK    47
#define BOARD_ES7210_DIN    14

#define RADIO_BUSY_PIN      13
#define RADIO_RST_PIN       17
#define RADIO_DIO1_PIN      45

#define BOARD_BOOT_PIN      0

#define BOARD_BL_PIN        42


/**** DEFINITION DES CONSTANTES DU CLAVIER ****/
#define LILYGO_KB_SLAVE_ADDRESS             0x55
#define LILYGO_KB_BRIGHTNESS_CMD            0x01
#define LILYGO_KB_ALT_B_BRIGHTNESS_CMD      0x02

#define KEY_DELETE  8
#define KEY_RETURN  13


/**** DEFINITION DES PARAMETRES LORA ****/
#define LORA_RF_FREQUENCY                           433.775   // MHz
#define LORA_BANDWIDTH                              125.0     // kHz
#define LORA_SPREADING_FACTOR                       12        // [SF7..SF12]
#define LORA_CODINGRATE                             5
#define LORA_SYNC_WORD                              0x12
#define LORA_TX_OUTPUT_POWER                        3         // dBm
#define LORA_PREAMBLE_LENGTH                        8         // Idem pour Tx et Rx
#define LORA_TCXO_VOLTAGE                           1.6
#define LORA_USE_REGULATOR_LDO                      false

#define LORA_START_BYTE_1 '<'
#define LORA_START_BYTE_2 0xFF
#define LORA_START_BYTE_3 0x01


/**** DEFINITION DES PARAMETRES APRS ****/
#define APRS_PERIOD     0                         // Période d'émission en minutes.                        
#define APRS_CALLSIGN   "NOCALL"                  // Indicatif de l'émetteur.                
#define APRS_PATH       "APLG01,WIDE1-1,WIDE2-1"  // Indicatif du destinataire avec le chemin.
#define APRS_OVERLAY    "L"                       // Overlay APRS.    
#define APRS_SYMBOL     "&"                       // Symbole APRS.
#define APRS_LAT        "0000.00N"                // Latitude de la trame APRS.
#define APRS_LNG        "00000.00E"               // Longitude de la trame APRS.
#define APRS_COMMENT    "433.775 MHz 3 dBm"       // Commentaire de la trame APRS.
#define APRS_TO         "NOCALL"                  // Destinataire des messages APRS.


/**** DEFINITION DES COMMANDES ****/
#define CMD_REBOOT    "reboot"
#define CMD_CLS       "cls"
#define CMD_SEND      "send"
#define CMD_BEACON    "beacon"

#define CMD_PERIOD    "period"
#define CMD_CALLSIGN  "callsign"
#define CMD_PATH      "path"
#define CMD_LAT       "lat"
#define CMD_LNG       "lng"
#define CMD_COMMENT   "comment"
#define CMD_TO        "to"

#define CMD_MESSAGE   "msg"


/**** DEFINITION DES PARAMETRES SYSTEME ****/
#define WDT_TIMEOUT 25

#define NB_DISPLAY_FRAMES 5


/**** DECLARATION DES VARIABLES GLOBALES ****/
String TrameTNC2Tx;   // Trame APRS au format TNC2 pour l'émission.
String TrameTNC2Rx;   // Trame APRS au format TNC2 pour la réception.

int InstantTx;        // Ancien instant d'émission.

SX1262 Radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN); // Objet SX1262 pour l'interface radio.
int State = RADIOLIB_ERR_NONE;  // Etat renvoyé par les méthodes de la bibliothèque Radiolib.

volatile bool ReceivedFlag = false; // Flag pour indiquer la réception radio.
volatile bool Interrupt = true;     // Flag pour activer l'interruption de la réception radio.

TFT_eSPI Tft; // Objet pour utiliser l'écran.

esp_err_t Esp32Error; // Variable du watchdog.

String DisplayFrames[NB_DISPLAY_FRAMES]; // Tampon des lignes de l'écran.

String Command = "";  // Commande saisie au clavier.
String Argument = ""; // Argument de la commande saisie au clavier.

Preferences Prefs;  // Préférences pour la sauvegarde dans la mémoire flash.

int Period = APRS_PERIOD;         // Période d'émission en minutes.                        
String Callsign = APRS_CALLSIGN;  // Indicatif de l'émetteur.                
String Path = APRS_PATH;          // Indicatif du destinataire avec le chemin.
String Overlay = APRS_OVERLAY;    // Overlay APRS.    
String Symbol = APRS_SYMBOL;      // Symbole APRS.
String Lat = APRS_LAT;            // Latitude de la trame APRS.
String Lng = APRS_LNG;            // Longitude de la trame APRS.
String Comment = APRS_COMMENT;    // Commentaire de la trame APRS.
String To = APRS_TO;              // Destinataire des messages APRS.

bool Beacon = false;  // Demande d'émission de la balise APRS.


/**** DEFINITION DE LA FONCTION D'INTERRUPTION D'EMISSION ****/
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif

void setRxFlag(void)
{
  if(Interrupt) ReceivedFlag = true;
}


/**** DEFINITION DE LA FONCTION DE MISE A JOUR DE L'ECRAN ****/
void Display(void)
{
  if(Command.length() > 0)
  {
    if(Command.length() == 1)
    {
      Tft.fillScreen(TFT_BLACK); 
      Tft.setRotation(1);
      Tft.setTextColor(TFT_YELLOW,TFT_BLACK);
      Tft.setTextSize(2);
    }

    // Affichage de la commande.
    Tft.setCursor(0, 0, 2);
    Tft.println(">" + Command);
  }
  else
  {
    // Initialisation de l'écran.
    Tft.begin();

    Tft.fillScreen(TFT_BLACK);
  
    Tft.setRotation(1);
    Tft.setCursor(0, 0, 2);
    Tft.setTextColor(TFT_WHITE,TFT_BLACK);
    Tft.setTextSize(1);

    // Affichage du trafic.
    for(int Index = 0; Index < NB_DISPLAY_FRAMES; Index ++)
    Tft.println(DisplayFrames[Index]);
  }
}


/**** DEFINITION DE LA FONCTION D'EXECUTION D'UNE COMMANDE ****/
void Run(String Cmd)
{
  // Redémarrage.
  if(Cmd == CMD_REBOOT) delay(30000);

  // Effacement de l'écran.
  else if(Cmd == CMD_CLS)
  {
    for(int Index = 0; Index < NB_DISPLAY_FRAMES; Index ++)
    DisplayFrames[Index] = "";
  }

  // Emission de texte brut.
  else if(Cmd.startsWith(CMD_SEND))
  {
    Argument = Cmd.substring(Cmd.indexOf(" ") + 1, Cmd.length());
    
    TrameTNC2Tx = LORA_START_BYTE_1;
    TrameTNC2Tx += char(LORA_START_BYTE_2);
    TrameTNC2Tx += char(LORA_START_BYTE_3);
    TrameTNC2Tx += Argument;

    // Désactivation de l'interruption.
    Interrupt = false;
    
    // Emission de la trame APRS.
    Radio.transmit(TrameTNC2Tx);
    
    Serial.println(F("[SX1262] Transmit"));
    Serial.println(Argument);

    // Activation de l'interruption.
    Interrupt = true;

    // Démarrage de la réception radio.
    Serial.print(F("[SX1262] Starting to listen : "));
    State = Radio.startReceive();
    
    if(State == RADIOLIB_ERR_NONE)
    Serial.println(F("success"));
    else
    {
      Serial.print(F("failed, code "));
      Serial.println(State);
      while(true);
    }
    
    // Mise à jour du tampon de l'écran.
    for(int Index = NB_DISPLAY_FRAMES - 2; Index >= 0; Index --) DisplayFrames[Index + 1] = DisplayFrames[Index];
    DisplayFrames[0] = "[TX] " + Argument;
  }

  // Demande d'émission d'une balise APRS.
  else if(Cmd == CMD_BEACON && !Beacon) Beacon = true;

  // Emission d'un message.
  else if(Cmd.startsWith(CMD_PERIOD))
  {
    Argument = Cmd.substring(Cmd.indexOf(" ") + 1, Cmd.length());

    if((Argument.toInt() > 0) && (Argument.toInt() <= 600))
    {
      Period = Argument.toInt();
    
      // Mise à jour du tampon de l'écran.
      for(int Index = NB_DISPLAY_FRAMES - 2; Index >= 0; Index --) DisplayFrames[Index + 1] = DisplayFrames[Index];
      DisplayFrames[0] = "#Beacon ON / Period = ";
      DisplayFrames[0] += Period;
      DisplayFrames[0] += " min";
    }
    else
    {
      Period = 0;
    
      // Mise à jour du tampon de l'écran.
      for(int Index = NB_DISPLAY_FRAMES - 2; Index >= 0; Index --) DisplayFrames[Index + 1] = DisplayFrames[Index];
      DisplayFrames[0] = "#Beacon OFF";    
    }

    Prefs.putInt("Period", Period);
  }
 
  // Changement de l'indicatif APRS.
  else if(Cmd.startsWith(CMD_CALLSIGN))
  {
    Argument = "";
    
    if(Cmd != CMD_CALLSIGN)
    {
      Argument = Cmd.substring(Cmd.indexOf(" ") + 1, Cmd.length());
      Argument.trim();
    }

    // Le nombre de caractères doit être correct.
    if((Argument.length() > 0) && (Argument.length() <= 9))
    {
      Callsign = Argument;
      Prefs.putString("Callsign", Callsign);
    }

    // Mise à jour du tampon de l'écran.
    for(int Index = NB_DISPLAY_FRAMES - 2; Index >= 0; Index --) DisplayFrames[Index + 1] = DisplayFrames[Index];
    DisplayFrames[0] = "#Callsign : " + Callsign; 
  }

  // Changement du chemin APRS.
  else if(Cmd.startsWith(CMD_PATH))
  {
    Argument = "";
    
    if(Cmd != CMD_PATH)
    {
      Argument = Cmd.substring(Cmd.indexOf(" ") + 1, Cmd.length());
      Argument.trim();
    }

    // Le nombre de caractères doit être correct.
    if((Argument.length() > 0) && (Argument.length() < 100))
    {
      Path = Argument;
      Prefs.putString("Path", Path);
    }

    // Mise à jour du tampon de l'écran.
    for(int Index = NB_DISPLAY_FRAMES - 2; Index >= 0; Index --) DisplayFrames[Index + 1] = DisplayFrames[Index];
    DisplayFrames[0] = "#Path : " + Path; 
  }
  
  // Changement de la latitude APRS.
  else if(Cmd.startsWith(CMD_LAT))
  {
    Argument = "";
    
    if(Cmd != CMD_LAT)
    {
      Argument = Cmd.substring(Cmd.indexOf(" ") + 1, Cmd.length());
      Argument.trim();
    }

    // Le nombre de caractères doit être correct.
    if(Argument.length() == 8)
    {
      Lat = Argument;
      Prefs.putString("Lat", Lat);
    }

    // Mise à jour du tampon de l'écran.
    for(int Index = NB_DISPLAY_FRAMES - 2; Index >= 0; Index --) DisplayFrames[Index + 1] = DisplayFrames[Index];
    DisplayFrames[0] = "#Latitude : " + Lat; 
  }

  // Changement de la longitude APRS.
  else if(Cmd.startsWith(CMD_LNG))
  {
    Argument = "";
    
    if(Cmd != CMD_LNG)
    {
      Argument = Cmd.substring(Cmd.indexOf(" ") + 1, Cmd.length());
      Argument.trim();
    }

    // Le nombre de caractères doit être correct.
    if(Argument.length() == 9)
    {
      Lng = Argument;
      Prefs.putString("Lng", Lng);
    }

    // Mise à jour du tampon de l'écran.
    for(int Index = NB_DISPLAY_FRAMES - 2; Index >= 0; Index --) DisplayFrames[Index + 1] = DisplayFrames[Index];
    DisplayFrames[0] = "#Longitude : " + Lng; 
  }

  // Changement du commentaire APRS.
  else if(Cmd.startsWith(CMD_COMMENT))
  {
    Argument = "";
    
    if(Cmd != CMD_COMMENT)
    {
      Argument = Cmd.substring(Cmd.indexOf(" ") + 1, Cmd.length());
      Argument.trim();
    }

    // Le nombre de caractères doit être correct.
    if((Argument.length() > 0) && (Argument.length() < 256))
    {
      Comment = Argument;
      Prefs.putString("Comment", Comment);
    }

    // Mise à jour du tampon de l'écran.
    for(int Index = NB_DISPLAY_FRAMES - 2; Index >= 0; Index --) DisplayFrames[Index + 1] = DisplayFrames[Index];
    DisplayFrames[0] = "#Comment : " + Comment; 
  }

  // Changement du destinataire des messages APRS.
  else if(Cmd.startsWith(CMD_TO))
  {
    Argument = "";
    
    if(Cmd != CMD_TO)
    {
      Argument = Cmd.substring(Cmd.indexOf(" ") + 1, Cmd.length());
      Argument.trim();
    }

    // Le nombre de caractères doit être correct.
    if((Argument.length() > 0) && (Argument.length() <= 9))
    {
      To = Argument;
      Prefs.putString("To", To);
    }

    // Mise à jour du tampon de l'écran.
    for(int Index = NB_DISPLAY_FRAMES - 2; Index >= 0; Index --) DisplayFrames[Index + 1] = DisplayFrames[Index];
    DisplayFrames[0] = "#To : " + To; 
  }


  // Emission d'un message APRS.
  else if(Cmd.startsWith(CMD_MESSAGE))
  {
    Argument = Cmd.substring(Cmd.indexOf(" ") + 1, Cmd.length());
    
    TrameTNC2Tx = LORA_START_BYTE_1;
    TrameTNC2Tx += char(LORA_START_BYTE_2);
    TrameTNC2Tx += char(LORA_START_BYTE_3);

    TrameTNC2Tx += Callsign;
    TrameTNC2Tx += '>';
    TrameTNC2Tx += Path;
    TrameTNC2Tx += "::";
    TrameTNC2Tx += To;

    int NbSpaces = 9 - To.length();
    if(NbSpaces > 0)
    for(int Cpt = 0; Cpt < NbSpaces; Cpt ++) 
    TrameTNC2Tx += " ";
    
    TrameTNC2Tx += ":";
    TrameTNC2Tx += Argument;

    // Désactivation de l'interruption.
    Interrupt = false;
    
    // Emission de la trame APRS.
    Radio.transmit(TrameTNC2Tx);
    
    Serial.println(F("[SX1262] Transmit"));
    Serial.println(TrameTNC2Tx.substring(3));

    // Activation de l'interruption.
    Interrupt = true;

    // Démarrage de la réception radio.
    Serial.print(F("[SX1262] Starting to listen : "));
    State = Radio.startReceive();
    
    if(State == RADIOLIB_ERR_NONE)
    Serial.println(F("success"));
    else
    {
      Serial.print(F("failed, code "));
      Serial.println(State);
      while(true);
    }
    
    // Mise à jour du tampon de l'écran.
    for(int Index = NB_DISPLAY_FRAMES - 2; Index >= 0; Index --) DisplayFrames[Index + 1] = DisplayFrames[Index];
    DisplayFrames[0] = "[TX] " + TrameTNC2Tx.substring(3);
  }

  // La commande est inconnue.
  else
  {
    // Mise à jour du tampon de l'écran.
    for(int Index = NB_DISPLAY_FRAMES - 2; Index >= 0; Index --) DisplayFrames[Index + 1] = DisplayFrames[Index];
    DisplayFrames[0] = "#Unknow command";
  }
}


/**** DEFINITION DE LA FONCTION D'INITIALISATION ****/
void setup()
{
  // Initialisation du watchdog.
  Esp32Error = esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);

  // Initialisation des préférences.
  Prefs.begin("TDECK", false);

  Period = Prefs.getInt("Period");
  Callsign = Prefs.getString("Callsign");
  Path = Prefs.getString("Path");
  Lat = Prefs.getString("Lat");
  Lng = Prefs.getString("Lng");
  Comment = Prefs.getString("Comment");
  To = Prefs.getString("To");

  if(Callsign == "") Callsign = APRS_CALLSIGN;
  if(Path == "") Path = APRS_PATH;
  if(Lat == "") Lat = APRS_LAT;
  if(Lng == "") Lng = APRS_LNG;
  if(Comment == "") Comment = APRS_COMMENT;
  if(To == "") To = APRS_TO;

  // Initialisation des entrées et sorties.
  pinMode(BOARD_POWERON, OUTPUT);
  digitalWrite(BOARD_POWERON, HIGH);

  pinMode(BOARD_SDCARD_CS, OUTPUT);
  pinMode(RADIO_CS_PIN, OUTPUT);
  pinMode(BOARD_TFT_CS, OUTPUT);
  digitalWrite(BOARD_SDCARD_CS, HIGH);
  digitalWrite(RADIO_CS_PIN, HIGH);
  digitalWrite(BOARD_TFT_CS, HIGH);

  pinMode(BOARD_BL_PIN, OUTPUT);
  digitalWrite(BOARD_BL_PIN, HIGH);

  pinMode(BOARD_SPI_MISO, INPUT_PULLUP);
  SPI.begin(BOARD_SPI_SCK, BOARD_SPI_MISO, BOARD_SPI_MOSI);
    
  // Initialisation du port série.
  Serial.begin(115200);
  //while(!Serial) delay(100);

  // Initilisation de l'interface radio.
  State = Radio.begin(LORA_RF_FREQUENCY, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE, LORA_SYNC_WORD, LORA_TX_OUTPUT_POWER,
  LORA_PREAMBLE_LENGTH, LORA_TCXO_VOLTAGE, LORA_USE_REGULATOR_LDO);
  
  // Affichage de l'état d'initialisation de l'interface radio.
  Serial.print(F("[SX1262] Initializing : "));
  
  if(State == RADIOLIB_ERR_NONE)
  Serial.println(F("success"));

  else
  {
    Serial.print(F("failed, code "));
    Serial.println(State);
    while(true);
  }

  // Affectation de la fonction pour l'interruption de la réception radio. 
  Radio.setPacketReceivedAction(setRxFlag);

  // Démarrage de la réception radio.
  Serial.print(F("[SX1262] Starting to listen : "));
  State = Radio.startReceive();
  
  if(State == RADIOLIB_ERR_NONE)
  Serial.println(F("success"));
  else
  {
    Serial.print(F("failed, code "));
    Serial.println(State);
    while(true);
  }

  // Initialisation du clavier.
  Wire.begin(BOARD_I2C_SDA, BOARD_I2C_SCL);
  delay(500);
  Wire.requestFrom(LILYGO_KB_SLAVE_ADDRESS, 1);
  if(Wire.read() == -1)
  {
    Serial.println("[KEYBRD] Not present");
    while(true);
  }

  // Initialisation du tampon de l'écran.
  for(int Index = 0; Index < NB_DISPLAY_FRAMES; Index ++) DisplayFrames[Index] = "";
  DisplayFrames[0] = "#Hello !";

  // Affichage de l'écran.
  Display();
  
  // Initialisation de l'instant d'émission.
  InstantTx = millis();

  // Reset du watchdog
  esp_task_wdt_reset();
  
  // Très important : laisser au minimum un délai de 1 ms après un reset du watchdog.
  delay(100);
}


/**** DEFINITION DE LA FONCTION PRINCIPALE ****/
void loop()
{
  // Lecture du clavier.
  char KeyValue = 0;
  
  Wire.requestFrom(LILYGO_KB_SLAVE_ADDRESS, 1);

  while (Wire.available() > 0)
  {
    KeyValue = Wire.read();
    
    if(KeyValue != (char)0x00)
    {
      Serial.print("[KEYBRD] Input ");
      Serial.print(KeyValue);
      Serial.print(" : ");
      Serial.println(int(KeyValue));

      switch(int(KeyValue))
      {
        case KEY_DELETE:
        Command = Command.substring(0, Command.length() - 1);
        Tft.fillScreen(TFT_BLACK);
        break;

        case KEY_RETURN:
        
        // Mise à jour du tampon de l'écran.
        for(int Index = NB_DISPLAY_FRAMES - 2; Index >= 0; Index --) DisplayFrames[Index + 1] = DisplayFrames[Index];
        DisplayFrames[0] = ">" + Command;

        Run(Command);
        
        Command = "";
        Tft.fillScreen(TFT_BLACK);
        break;
        
        default:
        Command += KeyValue;
      }

      Display();
    }
  }

  // On demande l'émission une balise APRS au bout du délai écoulé.
  if(((millis() - InstantTx) > (Period * 60000)) && (Period > 0) && !Beacon) Beacon = true;

  // On émet une balise APRS.
  if(Beacon)
  {
    // Désactivation de la demande d'émission d'une balise APRS.
    Beacon = false;
    
    // Affectation de l'instant d'émission.
    InstantTx = millis();

    // Désactivation de l'interruption.
    Interrupt = false;
    
    // Constitution de la trame APRS.
    TrameTNC2Tx = LORA_START_BYTE_1;
    TrameTNC2Tx += char(LORA_START_BYTE_2);
    TrameTNC2Tx += char(LORA_START_BYTE_3);
    TrameTNC2Tx += Callsign;
    TrameTNC2Tx += '>';
    TrameTNC2Tx += Path;
    TrameTNC2Tx += ":=";
    TrameTNC2Tx += Lat;
    TrameTNC2Tx += Overlay;
    TrameTNC2Tx += Lng;
    TrameTNC2Tx += Symbol;
    TrameTNC2Tx += Comment;

    // Emission de la trame APRS.
    Radio.transmit(TrameTNC2Tx);
    
    Serial.println(F("[SX1262] Transmit"));
    Serial.println(TrameTNC2Tx.substring(3));

    // Activation de l'interruption.
    Interrupt = true;

    // Démarrage de la réception radio.
    Serial.print(F("[SX1262] Starting to listen : "));
    State = Radio.startReceive();
    
    if(State == RADIOLIB_ERR_NONE)
    Serial.println(F("success"));
    else
    {
      Serial.print(F("failed, code "));
      Serial.println(State);
      while(true);
    }
    
    // Mise à jour du tampon de l'écran.
    for(int Index = NB_DISPLAY_FRAMES - 2; Index >= 0; Index --) DisplayFrames[Index + 1] = DisplayFrames[Index];
    DisplayFrames[0] = "[TX] " + TrameTNC2Tx.substring(3);

    // Affichage de l'écran.
    Display();
  }


  // Réception d'un signal radio.
  if(ReceivedFlag)
  {
    // Initialisation du flag de réception.
    ReceivedFlag = false;

    // Désactivation de l'interruption.
    Interrupt = false;

    // Lecture de la trame APRS.
    State = Radio.readData(TrameTNC2Rx);

    // Affichage de la trame si elle est correcte, et de ses informations à l'écran.
    if(State == RADIOLIB_ERR_NONE && TrameTNC2Rx.charAt(0) == LORA_START_BYTE_1
    && TrameTNC2Rx.charAt(1) == LORA_START_BYTE_2 && TrameTNC2Rx.charAt(2) == LORA_START_BYTE_3)
    {
      // packet was successfully received
      Serial.println(F("[SX1262] Received packet"));
      Serial.println(TrameTNC2Rx.substring(3));
 
      // print RSSI (Received Signal Strength Indicator)
      Serial.print(F("[SX1262] RSSI : "));
      Serial.print(Radio.getRSSI());
      Serial.println(F(" dBm"));

      // print SNR (Signal-to-Noise Ratio)
      Serial.print(F("[SX1262] SNR : "));
      Serial.print(Radio.getSNR());
      Serial.println(F(" dB"));

      // print frequency error
      Serial.print(F("[SX1262] Frequency error : "));
      Serial.print(Radio.getFrequencyError());
      Serial.println(F(" Hz"));

      // Mise à jour du tampon de l'écran.
      for(int Index = NB_DISPLAY_FRAMES - 2; Index >= 0; Index --) DisplayFrames[Index + 1] = DisplayFrames[Index];
      DisplayFrames[0] = "[RX] " + TrameTNC2Rx.substring(3);
    }
    
    // Gestion des erreurs de réception.
    else if (State == RADIOLIB_ERR_CRC_MISMATCH) Serial.println(F("CRC error!"));

    else
    {
      Serial.print(F("failed, code "));
      Serial.println(State);
    }

    // Activation de l'interruption.
    Interrupt = true;

    // Démarrage de la réception radio.
    Serial.print(F("[SX1262] Starting to listen : "));
    State = Radio.startReceive();
    
    if(State == RADIOLIB_ERR_NONE)
    Serial.println(F("success"));
    else
    {
      Serial.print(F("failed, code "));
      Serial.println(State);
      while(true);
    }
 
    // Affichage de l'écran.
    Display();
  }

  // Reset du watchdog
  esp_task_wdt_reset();

  // Attente de 20 milli secondes.
  delay(20);
}
