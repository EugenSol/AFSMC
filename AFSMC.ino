//
// Steuerung der Tauchzelle
//
// Alle Variablen mit direktem physikalischen Beziehungen werden als solche
// kenntlich gemacht.
// Hierbei wird stehts die Einheit angegeben, z.B. m (Meter), ms (Millisekunden)
// oder mbar (Millibar).
// Sind keine derartigen Bezeichnungen vorhanden, so handelt es sich es sich entweder
// um keine im direkten Zusammenhang mit der physikalischen Beschreibung der Tauchzelle
// stehende Variable oder diese Variable ist einheitenlos (1).

//ALLGEMEIN
//Includes
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <SPI.h>
#include <String.h>
#include <EEPROM.h>

//Deklaration
///Zeit
elapsedMillis looptime = 0; //Zeit die der Teensy fuer einen Durchlauf benötigt in ms
elapsedMillis overalltime = 0; //Gesamte Durchlaufzeit in ms
const int iterationtime = 25; //Zeit die der Teensy fuer einen Durchlauf hat in ms
const int maxoveralltime = 64000; //Maximale Tauchzeit, bedingt durch den EEPROM in ms
///Koordinaten
double z = 0; //Tauchtiefe der Tauchzelle in m
double zpunkt = 0; //Geschwindigkeit der Tauchzelle in m/s
const double z_ziel = 0.5; //Zieltiefe der Tauchzelle in m
double p0 = 0; //Nulldruck für z = 0 in mbar
double p_ziel = 0; //Zieldruck der Tauchzelle in mbar


//DRUCKSENSOR
// Declaration of Copyright
// Copyright (c) for pressure sensor 2009 MEAS Switzerland
// Edited 2015 Johann Lange
// @brief This C code is for starter reference only. It was based on the
// MEAS Switzerland MS56xx pressure sensor modules and Atmel Atmega644p
// microcontroller code and has been by translated Johann Lange
// to work with Teensy 3.1 microcontroller.
//Macros
#define TRUE 1
#define FALSE 0
#define CMD_RESET 0x1E //ADC reset command (ADC = Analgo-Digital-Converter)
#define CMD_ADC_READ 0x00 //ADC read command
#define CMD_ADC_CONV 0x40 //ADC conversion command
#define CMD_ADC_D1 0x00 //ADC D1 conversion
#define CMD_ADC_D2 0x10 //ADC D2 conversion
#define CMD_ADC_256 0x00 //ADC OSR=256 (OSR = Oversamplingrate)
#define CMD_ADC_512 0x02 //ADC OSR=512
#define CMD_ADC_1024 0x04 //ADC OSR=1024
#define CMD_ADC_2048 0x06 //ADC OSR=2056
#define CMD_ADC_4096 0x08 //ADC OSR=4096
#define CMD_PROM_RD 0xA0 //Prom read command

//Deklarationen
unsigned long D1; //ADC value of the pressure conversion
unsigned long D2; //ADC value of the temperature conversion
unsigned int C[8]; //calibration coefficients
double P; //compensated pressure value
double T; //compensated temperature value
double dT; //difference between actual and measured temperature
double OFF; //offset at actual temperature
double SENS; //sensitivity at actual temperature
double T2; //compensated pressure value, 2nd order
double OFF2; //compensated pressure value, 2nd order
double SENS2; //compensated pressure value, 2nd order
unsigned char n_crc;
// int i; //counting variable


//SPI
const int MISO_Pin = 12;
const int MOSI_Pin = 11;
const int SCK_Pin = 13;
const int CS_Pin_PS = 10; //Chip Select Pressure Sensor
SPISettings settings_PS(4000000, MSBFIRST, SPI_MODE0); //Grundeinstellungen fuer das SPI-Protokoll, Pressure Sensor


//SLIDING-MODE-OBSERVER (SMO)
//Deklaration
///Modelparameter
const double rho = 10; //Observerparameter
const double tau = 0.2; //Observerparameter
///Koordinaten
double xhat1 = 0; //Durch den SMO geschätzte Tiefe der Tauchzelle in m
double xhat2 = 0; //Durch den SMO geschätzte Geschwindigkeit der Tauchzelle in m/s
double xhat1_prev = 0; //Durch den SMO vorangegangene geschätzte Tiefe der Tauchzelle in m
double xhat2_prev = 0; //Durch den SMO vorangegangene geschätzte Geschwindigkeit der Tauchzelle in m/s


//PID-REGLER
//Deklaration
///Drucke des Regelkreises
double pdiff = 0; //Differenzdruck zwischen P und p_ziel, fuer P-Anteil
double pdiff_sum = 0; //aufsummierter Differenzdruck, fuer I-Anteil
double palpha = 0; //Stellgroesse des PID-Reglers zur Berechnung des Winkel alpha
double palpha_max = 50; //Maximalwert der Stellgroesse fuer sinnvolle Auswertung
///Reglergroessen
const double Kp = 1; //10; //Proportionale Verstaerkung des PID-Reglers
const double Ki = 0; //0.01; //Integrale Verstaerkung des PID-Reglers
const double Kd = 1000; //1000; //Differentiale Verstaerkung des PID-Reglers


//MOTOR
//Deklaration
///Pins
const int DIR_Pin = 21;
const int STEP_Pin = 22;
const int SLEEP_Pin = 23;
const int Endlagen_Pin = 17; //HIGH: nicht gedrückt, LOW: gedrückt
///Variablen
const int iplanet = 256; //Uerbersetzung des Planetengetriebes
const double schrittwinkel = 18; //Vollschrittwinkel des Motors in grd
double winkel = 0; //Tatsächlicher Winkel des Zahnrades in grd
double alpha = 0; //Angestrebter Winkel des Zahnrades grd
const double alpha_max = 57.5; //Maximaler Winkel des Zahnrades grd
const double Schrittfrequenz = 2500; //Maximale Schrittfrequen bei welcher das maximal benötigte Drehmoment noch aufgebracht werden kann in steps/s (max 3540)
const double Schrittzeit = 1000000 / Schrittfrequenz; //Die Zeit die der Motor fuer einen Schritt bekommt in µs
double Schrittzeit_halb = 0; //Die Zeit die der Motor fuer einen Puls bekommt in µs, Definition in setup
int Schrittzahl = 0; //Schrittzaehler ausgehend von 0 fuer Winkel = 0

//EEPROM
//Deklarations
int nsd = 0; //Zaehler fuer das Abspeichern der Daten in das Array
const int zeilenArrayMax = 128; //Maximale Anzahl der Zeilen des Arrays
int iArray = 0; //Anzahl der bereits belegten Zeilen des Arrays
long datenArray[128][4]; //Array um die Daten zu speichern
bool aufEEPROMgeschrieben = false;


void setup() {

  //MOTOR
  Schrittzeit_halb = round(Schrittzeit / 2);

  //SPI
  pinMode(CS_Pin_PS, OUTPUT);
  pinMode(MISO_Pin, OUTPUT);
  pinMode(SCK_Pin, OUTPUT);
  pinMode(MOSI_Pin, INPUT);
  SPI.setMOSI(MOSI_Pin);
  SPI.setMISO(MISO_Pin);
  SPI.setSCK(SCK_Pin);
  SPI.begin();

  //Drucksensor
  SPI.beginTransaction(settings_PS);
  cmd_reset(); // reset the module after powerup
  for (int i = 0; i < 8; ++i) {
    C[i] = cmd_prom(i);
    //Serial.printf("C[%i] = %i\n", i, C[i]); // Wenn aktiviert, werden am Anfang die Kalibrierungskoeffs des Drucksensor aufgelistet
  } // read calibration coefficients
  n_crc = crc4(C);
  SPI.endTransaction();

  pinMode(DIR_Pin, OUTPUT);
  pinMode(STEP_Pin, OUTPUT);
  pinMode(SLEEP_Pin, OUTPUT);
  pinMode(Endlagen_Pin, INPUT_PULLUP);
  digitalWrite(DIR_Pin, LOW); // LOW = CCW = V down; HIGH = CW = V up
  digitalWrite(STEP_Pin, LOW);
  digitalWrite(SLEEP_Pin, LOW); // LOW = OFF; HIGH = ON


  // Auf Startsignal warten (Drucksensor)
  while (P < 1100) {
    P = berechneDruck();
  }

  digitalWrite(SLEEP_Pin, HIGH); // LOW = OFF; HIGH = ON
  Schrittzahl = resetPosition(); // Motor starten und zuruecksetzen
  delay(1000);

  //Koordinatentransformationen
  p0 = berechneDruck();
  p_ziel = p0 + 1000 * (z_ziel / 10);
}

void loop() {
  // Druck bestimmen
  P = berechneDruck();

  // Tauchtiefe berechnen
  z = 10 * ((P - p0) / 1000);

  // Geschw ueber SMO berechnen
  zpunkt = get_xhat2(z);

  // Regler auswerten und Winkel berechnen
  alpha = berechnePID();

  // Winkel anfahren
  winkel = step(alpha);

  // Daten (z, zpunkt, alpha, winkel) in Array schreiben, wenn das Array voll ist, die Daten in den EEPROM schreiben
  ++nsd;
  if (nsd >= 20) {
    WritetoArray();
  }

  // Looptime zuruecksetzen
  looptime = 0;
}


//********************************************************
//
// SMO berechnet xhat2 ueber xhat1
//
//********************************************************
// Schaetzt die Geschwindigkeit der Tauchzelle
double get_xhat2(double x1) {
  xhat1 = get_xhat1(x1);
  xhat2 = xhat2_prev + (double(iterationtime) / (double(1000) * tau)) * (-xhat2_prev - rho * sat(xhat1 - x1));
  xhat2_prev = xhat2;
  return xhat2;
}

// Schaetzt die Tiefe der Tauchzelle
double get_xhat1(double x1) {
  xhat1 = xhat1_prev - (double(iterationtime) / double(1000)) * rho * sat(xhat1_prev - x1);
  xhat1_prev = xhat1;
  return xhat1;
}

// Saettigungfunktion
double sat(double x) {
  double y = min(1, x);
  double ynew = max(y, -1);
  return ynew;
}


//********************************************************
//
// Reglergroessen werden berechnet
//
// Input: Druck P und/oder Tauchtiefe z
// Output: Winkel palpha vom Regler
//
//********************************************************
// Der PID-Regler wird ausgewertet und der zugehörige Winkel alpha berechnet
double berechnePID() {
  //Hier beginnt die Reglerberechnung, Platz für den (AF)SMC
  pdiff = P - p_ziel; // Differenzdruck berechnen
  pdiff_sum += pdiff; // Differenzdruck integrieren/aufaddieren
  palpha = Kp * pdiff + Ki * pdiff_sum + Kd * zpunkt; // palpha berechnen
  //Hier endet die Reglerberechnung

  // Saettigungsfunktion, palpha wird zu alpha (skaliert mit alpha_max / palpha_max = 57.5/50 = 1.15)
  if (palpha <= -palpha_max) { // palpha <= -palpha_max ==> alpha = -alpha_max (-57.5°)
    alpha = -alpha_max;
  }
  else if ((-palpha_max < palpha) && (palpha < palpha_max)) { // -palpha_max < palpha < palpha_max ==> alpha = palpha
    alpha = alpha_max / palpha_max * palpha;
  }
  else { // palpha >= palpha_max ==> alpha = alpha_max (57.5°)
    alpha = alpha_max;
  }
  return alpha;
}


//********************************************************
//
// Motor wird angesteuert
//
//********************************************************
// Ein Winkel wird explizit angefahren, solange die Zeit ausreicht
double step(double alpha) {
  digitalWrite(SLEEP_Pin, HIGH); // ON
  winkel = (double(Schrittzahl) * double(schrittwinkel)) / (double(iplanet) * double(6)) - alpha_max;
  double dalpha = alpha - winkel; // Winkeldifferenz zum Stellen
  int n = 6 * (int)abs((dalpha) * iplanet / schrittwinkel); // Anzahl der benötigten Schritte des Motors
  int i = 0; // Lokaler Schrittzaehler
  // Wenn noch genug Zeit in der Loop ist, soll der Motor solange arbeiten, bis die Zeit abgelaufen ist
  while (looptime < iterationtime - 1) {
    // Fuer den Fall, dass alpha < -alpha_max soll zusaetzlich mit Hilfe der Endlagen getestet werden, ob die Position stimmt
    if (alpha > -alpha_max) {
      if (dalpha > 0) {
        digitalWrite(DIR_Pin, HIGH);
        while (i <= n && looptime < iterationtime - 1) {
          digitalWrite(STEP_Pin, HIGH);
          delayMicroseconds(Schrittzeit_halb); // Halbe Schrittzeit warten
          digitalWrite(STEP_Pin, LOW);
          delayMicroseconds(Schrittzeit_halb); // Halbe Schrittzeit warten
          ++Schrittzahl; // Schrittzahl um 1 erhöhen (ehemals verringern)
          ++i;
        }
      }
      else if (dalpha < 0) {
        digitalWrite(DIR_Pin, LOW);
        while (i <= n && looptime < iterationtime - 1) {
          digitalWrite(STEP_Pin, HIGH);
          delayMicroseconds(Schrittzeit_halb); // Halbe Schrittzeit warten
          digitalWrite(STEP_Pin, LOW);
          delayMicroseconds(Schrittzeit_halb); // Halbe Schrittzeit warten
          --Schrittzahl; // Schrittzahl um 1 verringern (ehemals erhöhen)
          ++i;
        }
      }
      else { // Fuer den Fall, dass dalpha = 0 soll keine Aktion ausgefuehrt werden
      }
    }
    else { //alpha = alpha_max, Endlage wird ueberprueft
      digitalWrite(DIR_Pin, LOW);
      while (digitalReadFast(Endlagen_Pin) == HIGH && looptime < iterationtime - 1) { // Endlage noch nicht erreicht
        digitalWrite(STEP_Pin, HIGH);
        delayMicroseconds(Schrittzeit_halb); // Halbe Schrittzeit warten
        digitalWrite(STEP_Pin, LOW);
        delayMicroseconds(Schrittzeit_halb); // Halbe Schrittzeit warten
        --Schrittzahl;
      }
      if (digitalRead(Endlagen_Pin) == LOW) {
        Schrittzahl = 0;
      }
    }
  }
  winkel = (double(Schrittzahl) * double(schrittwinkel)) / (double(iplanet) * double(6)) - alpha_max;
  return winkel;
}

// Die Zahnstange wird gegen den Endlagenschalter gefahren
// und die Schrittzahl zurueckgesetzt
int resetPosition() {
  digitalWrite(SLEEP_Pin, HIGH);
  digitalWrite(DIR_Pin, LOW);
  while (digitalReadFast(Endlagen_Pin) == HIGH) {
    digitalWrite(STEP_Pin, HIGH);
    delayMicroseconds(Schrittzeit_halb); // Halbe Schrittzeit warten
    digitalWrite(STEP_Pin, LOW);
    delayMicroseconds(Schrittzeit_halb); // Halbe Schrittzeit warten
    delay(1); // Nach jedem Schritt 1 ms Pause, um langsamer anzufahren und den Endlagenschalter nicht zu beschaedigen
  }
  return 0; // Wenn Endlage erreicht, Schrittzahl auf 0 setzen
}


//********************************************************
//
// Daten werden in ein Array geschrieben. Wenn das Array
// voll ist, taucht die Tauchzelle wieder auf.
//
// ACHTUNG! ALLE WERTE WERDEN MIT 10 000 SKALIERT
// UND ANSCHLIESSEND IN LONG INTEGER VERWANDELT!
//
// Speicherbedarf von long: 4 byte
// Speicherbedarf pro Datensatz: 4*4 byte = 16 byte
// EEPROM-Groesse: 2 kibyte
// Maximale Anzahl an Datensätzen: 2048 byte / 16 byte = 128
// Wert-Quadrupel pro Sekunde: 2
// Maximale Tauchlänge: 128 / 2 = 64 s
//
//********************************************************
void WritetoArray() {
  int skalierung = 10000;
  long zArray = (long)(skalierung * z);
  long zpunktArray = (long)(skalierung * zpunkt);
  long alphaArray = (long)(skalierung * alpha);
  long winkelArray = (long)(skalierung * winkel);
  if (iArray < zeilenArrayMax) {
    datenArray[iArray][0] = zArray;
    datenArray[iArray][1] = zpunktArray;
    datenArray[iArray][2] = alphaArray;
    datenArray[iArray][3] = winkelArray;
    ++iArray;
  }
  else {
    // Daten in den EEPROM schreiben
    if (aufEEPROMgeschrieben == false) {
      WritetoEEPROM();
      aufEEPROMgeschrieben = true;
    }
    // iArray = 0;
    // Auftauchen
    while (winkel > -(alpha_max - 2)) {
      winkel = step(-alpha_max);
    }
    digitalWrite(SLEEP_Pin, LOW);
    Serial.println("Laa shay'a waqi'un moutlaq bale kouloun moumkine.");
    Serial.println("Nichts ist wahr, alles ist erlaubt.");
  }
  nsd = 0;
}


//********************************************************
//
// Daten werden in den EEPROM geschrieben
//
//********************************************************
void WritetoEEPROM() {
  byte data;
  int adress = 0;
  int  i = 0;
  while (adress < 2048) {
    for (int n = 0; n <= 3; ++n) {
      for (int a = 0; a <= 3; ++a) {
        data = (datenArray[i][n] >> ((3 - a) * 8)) & 0x000000ff;
        EEPROM.write(adress + a, data);
      }
      adress = adress + 4;
    }
    ++i;
  }
}


//********************************************************
//
// Druck wird berechnet
//
// Declaration of Copyright
// Copyright (c) 2009 MEAS Switzerland
// Edited 2015 Johann Lange
// This C code is for starter reference only. It was based on the
// MEAS Switzerland MS56xx pressure sensor modules and Atmel Atmega644p
// microcontroller code and has been by translated Johann Lange
// to work with Teensy 3.1 microcontroller.
//
//********************************************************
// Der Drucksensor wird ausgewertet und daraus der Druck bestimmt
double berechneDruck() {
  SPI.beginTransaction(settings_PS);
  // delay required in µs: OSR_4096: 9100, OSR_2048: 4600, OSR_1024: 2300, OSR_512: 1200, OSR_256: 700
  D1 = cmd_adc(CMD_ADC_D1 + CMD_ADC_4096, 9100); // read uncompensated pressure, Conversation Command + OSR, delay in µs
  D2 = cmd_adc(CMD_ADC_D2 + CMD_ADC_256, 700); // read uncompensated temperature, Conversation Command + OSR, delay in µs

  // calcualte 1st order temperature (MS5803_01b 1st order algorithm), base for 2nd order temperature and pressure
  dT = D2 - C[5] * pow(2, 8); //Serial.print("dT = "); Serial.println(dT);
  OFF = C[2] * pow(2, 16) + dT * C[4] / pow(2, 7); //Serial.print("OFF = "); Serial.println(OFF);
  SENS = C[1] * pow(2, 15) + dT * C[3] / pow(2, 8); //Serial.print("SENS = "); Serial.println(SENS);

  T = (2000 + (dT * C[6]) / pow(2, 23)) / 100;
  P = (((D1 * SENS) / pow(2, 21) - OFF) / pow(2, 15)) / 100;

  // calcualte 2nd order pressure and temperature (MS5803_01b 2nd order algorithm)
  if (T > 20) {
    T2 = 0;
    OFF2 = 0;
    SENS2 = 0;

    if (T > 45) {
      SENS2 -= pow(T - 4500, 2) / pow(2, 3);
    }
  }
  else {
    T2 = pow(dT, 2) / pow(2, 31);
    OFF2 = 3 * pow(100 * T - 2000, 2);
    SENS2 = 7 * pow(100 * T - 2000, 2) / pow(2, 3);

    if (T < 15) {
      SENS2 += 2 * pow(100 * T + 1500, 2);
    }
  }

  // Recalculate T, OFF, SENS based on T2, OFF2, SENS2
  T -= T2;
  OFF -= OFF2;
  SENS -= SENS2;

  P = (((D1 * SENS) / pow(2, 21) - OFF) / pow(2, 15)) / 100;
  return P;
  SPI.endTransaction();
}

void cmd_reset(void) {
  digitalWrite(CS_Pin_PS, LOW); // pull CSB low to start the command
  SPI.transfer(CMD_RESET); // send reset sequence
  delay(3); // wait for the reset sequence timing
  digitalWrite(CS_Pin_PS, HIGH); // pull CSB high to finish the command
}

//brief preform adc conversion
//return 24bit result
unsigned long cmd_adc(char cmd, int delaytime) {
  digitalWrite(CS_Pin_PS, LOW);
  unsigned long ret;
  unsigned long temp = 0;
  SPI.transfer(CMD_ADC_CONV + cmd); // send conversion command;
  cmd = SPI.transfer(0x00);
  delayMicroseconds(delaytime); // delay required in µs: OSR_4096: 9100, OSR_2048: 4600, OSR_1024: 2300, OSR_512: 1200, OSR_256: 700
  digitalWrite(CS_Pin_PS, HIGH);// pull CSB high to finish the conversion
  digitalWrite(CS_Pin_PS, LOW); // pull CSB low to start new command
  SPI.transfer(CMD_ADC_READ); // send ADC read command
  ret = SPI.transfer(0x00); // send 0 to read 1st byte (MSB)
  temp = 65536 * ret;
  ret = SPI.transfer(0x00); // send 0 to read 2nd byte
  temp = temp + 256 * ret;
  ret = SPI.transfer(0x00); // send 0 to read 3rd byte (LSB)
  temp = temp + ret;
  digitalWrite(CS_Pin_PS, HIGH); // pull CSB high to finish the read command
  return temp;
}

//brief Read calibration coefficients
//return coefficient
unsigned int cmd_prom(char coef_num) {
  unsigned int ret;
  unsigned int rC = 0;

  digitalWrite(CS_Pin_PS, LOW); // pull CSB low
  SPI.transfer(CMD_PROM_RD + coef_num * 2); // send PROM READ command
  ret = SPI.transfer(0x00); // send 0 to read the MSB
  rC = 256 * ret;
  ret = SPI.transfer(0x00); // send 0 to read the LSB
  rC = rC + ret;
  digitalWrite(CS_Pin_PS, HIGH);// pull CSB high
  return rC;
}

//brief calculate the CRC code for details look into CRC CODE NOTES
//return crc code
unsigned char crc4(unsigned int n_prom[]) {
  int cnt; // simple counter
  unsigned int n_rem; // crc reminder
  unsigned int crc_read; // original value of the crc
  unsigned char n_bit;
  n_rem = 0x00;
  crc_read = n_prom[7]; // save read CRC
  n_prom[7] = (0xFF00 & (n_prom[7])); // CRC byte is replaced by 0
  for (cnt = 0; cnt < 16; cnt++) // operation is performed on bytes
  { // choose LSB or MSB
    if (cnt % 2 == 1) n_rem ^= (unsigned short) ((n_prom[cnt >> 1]) & 0x00FF);
    else n_rem ^= (unsigned short) (n_prom[cnt >> 1] >> 8);
    for (n_bit = 8; n_bit > 0; n_bit--)
    {
      if (n_rem & (0x8000))
      {
        n_rem = (n_rem << 1) ^ 0x3000;
      }
      else
      {
        n_rem = (n_rem << 1);
      }
    }
  }
  n_rem = (0x000F & (n_rem >> 12)); // final 4-bit reminder is CRC code
  n_prom[7] = crc_read; // restore the crc_read to its original place
  return (n_rem ^ 0x00);
}
