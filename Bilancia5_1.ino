/**
 * Sistema di misura del Centro di Gravità (C.o.G.) - VERSIONE AUTO-START
 * per aeromodelli utilizzando quattro celle di carico.
 * Sviluppato da RZ.
 * Semplificato con avvio automatico: 05/10/2025
 * 
 * Il sistema utilizza:
 * - 4 celle di carico HX711 (una per ogni angolo)
 * - Display OLED SSD1309 da 2.42" con interfaccia SPI
 * - 2 pulsanti per l'interfaccia utente
 * 
 * FUNZIONAMENTO:
 * ---------------
 * All'accensione il sistema esegue automaticamente:
 * 1. Schermata di benvenuto (2 secondi)
 * 2. Richiesta di rimuovere il peso - countdown 5 secondi
 * 3. Tara automatica con barra di progresso
 * 4. Visualizzazione continua di peso e baricentro
 * 
 * COMANDI PULSANTI:
 * -----------------
 * K4 = Tara rapida (ripete la sequenza countdown + tara)
 * 
 * CALIBRAZIONE TRAMITE PORTA SERIALE:
 * ------------------------------------
 * Aprire il Serial Monitor (9600 baud) e inviare i seguenti comandi:
 * 
 * 1. CALIBRAZIONE COMPLETA (comando 'c'):
 *    a. Inviare 'c' sulla porta seriale
 *    b. Rimuovere tutto il peso dalla bilancia
 *    c. Inviare 't' per eseguire la tara
 *    d. Posizionare un peso noto al centro della bilancia
 *    e. Inserire il valore del peso in kg (es. 2.335)
 *    f. Attendere il completamento della calibrazione
 *    g. Copiare i nuovi fattori di calibrazione visualizzati
 *    h. Incollare i valori nel codice (sezione VARIABILI GLOBALI)
 *    i. Ricompilare e caricare il codice aggiornato
 * 
 * 2. VISUALIZZARE I FATTORI CORRENTI (comando 's'):
 *    Inviare 's' per visualizzare i fattori di calibrazione in uso
 * 
 * 3. TARA RAPIDA (comando 'z'):
 *    Inviare 'z' per eseguire una tara immediata
 * 
 * NOTE IMPORTANTI:
 * ----------------
 * - La calibrazione va eseguita una sola volta (o quando si cambiano le celle)
 * - Usare un peso di riferimento preciso (es. 2-5 kg)
 * - Posizionare il peso esattamente al centro della piattaforma
 * - Dopo la calibrazione, aggiornare i fattori nel codice e ricaricare
 * 
 * PARAMETRI DI FILTRAGGIO:
 * ------------------------
 * Il sistema implementa tre livelli di filtraggio per stabilizzare le letture:
 * 
 * 1. SMOOTHING (Media Mobile):
 *    - Numero campioni: 10 (SMOOTHING_SAMPLES)
 *    - Effetto: riduce le oscillazioni rapide mediando gli ultimi 10 valori
 *    - Aumentare per maggiore stabilità (risposta più lenta)
 *    - Diminuire per risposta più veloce (maggiori oscillazioni)
 * 
 * 2. DEAD ZONE (Soglia Minima):
 *    - Valore: 50g (MIN_WEIGHT_THRESHOLD = 0.05 kg)
 *    - Effetto: ignora pesi inferiori a 50g, mostrandoli come 0
 *    - Elimina le micro-oscillazioni quando la bilancia è vuota
 *    - Modificare in base alla sensibilità desiderata
 * 
 * 3. ARROTONDAMENTO:
 *    - Precisione: 50g (arrotonda a 0.00, 0.05, 0.10, 0.15, etc.)
 *    - Effetto: evita continui cambiamenti di 10-20g sul display
 *    - Per maggiore precisione: modificare la formula in roundWeightToDecagram()
 *      esempio per 10g: return round(weight * 100) / 100.0;
 * 
 * 4. INDICATORE DI STABILITÀ:
 *    - Soglia: variazione < 50g (WEIGHT_STABILITY_THRESHOLD)
 *    - Durata: 1 secondo (STABILITY_DURATION)
 *    - Effetto: il titolo "CG VIEW" lampeggia finché il peso non si stabilizza
 *    - Aiuta a sapere quando la lettura è affidabile
 */

//================ INCLUSIONI LIBRERIE ================//
#include "HX711.h"
#include <SPI.h>
#include <U8g2lib.h>

//================ DEFINIZIONE PIN ================//
const int LOADCELL1_DOUT_PIN = 2;
const int LOADCELL1_SCK_PIN = 3;
const int LOADCELL2_DOUT_PIN = 4;
const int LOADCELL2_SCK_PIN = 5;
const int LOADCELL3_DOUT_PIN = 6;
const int LOADCELL3_SCK_PIN = 7;
const int LOADCELL4_DOUT_PIN = 8;
const int LOADCELL4_SCK_PIN = 9;

const uint8_t BUTTON_K1 = A2;  // Menu (non usato nella versione semplificata)
const uint8_t BUTTON_K4 = A4;  // Tara rapida

#define OLED_MOSI     11
#define OLED_CLK      13
#define OLED_DC       A1
#define OLED_CS       10
#define OLED_RESET    A0

//================ DIMENSIONI FISICHE ================//
const float BASE_LENGTH = 250.0;
const float WING_PEG_DIST = 120.0;

//================ PARAMETRI ================//
const uint8_t SMOOTHING_SAMPLES = 10;
const uint16_t READ_INTERVAL = 500;  // Aggiornamento ogni 500ms per fluidità
const unsigned long DEBOUNCE_DELAY = 50;
const float MIN_WEIGHT_THRESHOLD = 0.05;

//================ OGGETTI ================//
HX711 scale1, scale2, scale3, scale4;
U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI display(U8G2_R0, OLED_CS, OLED_DC, OLED_RESET);

//================ FONT ================//
#define FONT_SMALL u8g2_font_5x8_tf
#define FONT_SMALL2 u8g2_font_u8glib_4_tf
#define FONT_MEDIUM u8g2_font_6x10_tf
#define FONT_LARGE u8g2_font_9x15_tf
#define FONT_HUGE u8g2_font_inb19_mr

//================ VARIABILI GLOBALI ================//
float calibration_factor1 = -418908.59;
float calibration_factor2 = -316792.09;
float calibration_factor3 = -401121.84;
float calibration_factor4 = -481890.66;

const float CORRECTION_FACTOR = 1.0;

unsigned long lastReadTime = 0;
unsigned long lastDebounceTime = 0;

float weight1Samples[SMOOTHING_SAMPLES] = {0};
float weight2Samples[SMOOTHING_SAMPLES] = {0};
float weight3Samples[SMOOTHING_SAMPLES] = {0};
float weight4Samples[SMOOTHING_SAMPLES] = {0};
uint8_t sampleIndex = 0;

float totalWeight = 0;
int16_t wingCogX = 0;
bool weightStable = false;
float previousWeight = 0;
unsigned long weightStableTime = 0;
const float WEIGHT_STABILITY_THRESHOLD = 0.05;  // Considera stabile se varia meno di 50g
const unsigned long STABILITY_DURATION = 1000;   // Deve rimanere stabile per 1 secondo

//================ PROTOTIPI ================//
void showWelcomeScreen();
void showCountdownAndTare();
void showCGViewContinuous();
void performTare();
void updateWeightData();
void checkButtons();
void checkSerialCommands();
void calibrateCellsViaSerial();
void showCalibrationFactors();
float smoothValue(float newValue, float samples[], uint8_t index);
float roundWeightToDecagram(float weight);

//================ FUNZIONI UTILITY ================//
inline float roundWeightToDecagram(float weight) {
  if (abs(weight) < MIN_WEIGHT_THRESHOLD) {
    return 0.0;
  }
  return round(weight * 100) / 100.0;  // 50/50 se vuoi una approx di 0.05 e non di 0.01)
}

float smoothValue(float newValue, float samples[], uint8_t index) {
  samples[index] = newValue;
  float sum = 0;
  for (uint8_t i = 0; i < SMOOTHING_SAMPLES; i++) {
    sum += samples[i];
  }
  return sum / SMOOTHING_SAMPLES;
}

//================ SETUP ================//
void setup() {
  Serial.begin(9600);
  Serial.println(F("C.o.G. System - Auto Start"));
  
  // Inizializza celle
  scale1.begin(LOADCELL1_DOUT_PIN, LOADCELL1_SCK_PIN);
  scale2.begin(LOADCELL2_DOUT_PIN, LOADCELL2_SCK_PIN);
  scale3.begin(LOADCELL3_DOUT_PIN, LOADCELL3_SCK_PIN);
  scale4.begin(LOADCELL4_DOUT_PIN, LOADCELL4_SCK_PIN);
  
  scale1.set_scale(calibration_factor1);
  scale2.set_scale(calibration_factor2);
  scale3.set_scale(calibration_factor3);
  scale4.set_scale(calibration_factor4);
  
  // Pulsanti
  pinMode(BUTTON_K4, INPUT_PULLUP);
  
  // Display
  display.begin();
  display.clearBuffer();
  
  // Sequenza di avvio automatica
  showWelcomeScreen();
  showCountdownAndTare();
  
  Serial.println(F("Ready - K4=Tare"));
  Serial.println(F("\nSerial Commands:"));
  Serial.println(F("  c - Start calibration"));
  Serial.println(F("  s - Show calibration factors"));
  Serial.println(F("  z - Quick tare"));
}

//================ LOOP ================//
void loop() {
  // Modalità visualizzazione continua CG
  showCGViewContinuous();
}

//================ SCHERMATE ================//
void showWelcomeScreen() {
  display.clearBuffer();
  display.drawFrame(0, 0, 128, 64);
  display.drawFrame(2, 2, 124, 60);
  
  display.setFont(FONT_HUGE);
  const char* mainTitle = "RZ CoG";
  int titleWidth = display.getStrWidth(mainTitle);
  display.setCursor((128 - titleWidth) / 2, 28);
  display.print(mainTitle);
  
  display.setFont(FONT_MEDIUM);
  const char* subTitle = "Scale System";
  int subTitleWidth = display.getStrWidth(subTitle);
  display.setCursor((128 - subTitleWidth) / 2, 45);
  display.print(subTitle);
  
  display.setFont(FONT_SMALL);
  const char* versionText = "Auto-Start v2.1";
  int versionWidth = display.getStrWidth(versionText);
  display.setCursor((128 - versionWidth) / 2, 58);
  display.print(versionText);
  
  display.sendBuffer();
  delay(2000);
}

void showCountdownAndTare() {
  // Messaggio iniziale
  display.clearBuffer();
  display.setFont(FONT_LARGE);
  
  const char* msg1 = "REMOVE ALL";
  int msg1Width = display.getStrWidth(msg1);
  display.setCursor((128 - msg1Width) / 2, 20);
  display.print(msg1);
  
  const char* msg2 = "WEIGHT";
  int msg2Width = display.getStrWidth(msg2);
  display.setCursor((128 - msg2Width) / 2, 38);
  display.print(msg2);
  
  display.setFont(FONT_SMALL);
  const char* msg3 = "Auto-tare in:";
  int msg3Width = display.getStrWidth(msg3);
  display.setCursor((128 - msg3Width) / 2, 52);
  display.print(msg3);
  
  display.sendBuffer();
  delay(500);
  
  // Countdown 5 secondi
  for (int i = 5; i > 0; i--) {
    // Cancella solo l'area del numero
    display.setDrawColor(0);
    display.drawBox(50, 54, 28, 10);
    display.setDrawColor(1);
    
    // Disegna il numero del countdown
    display.setFont(FONT_LARGE);
    char numStr[3];
    sprintf(numStr, "%d", i);
    int numWidth = display.getStrWidth(numStr);
    display.setCursor((128 - numWidth) / 2, 63);
    display.print(numStr);
    
    display.sendBuffer();
    delay(1000);
  }
  
  // Esegui la tara
  performTare();
}

void performTare() {
  display.clearBuffer();
  
  display.setFont(FONT_LARGE);
  const char* waitMsg = "WAIT...";
  int waitWidth = display.getStrWidth(waitMsg);
  display.setCursor((128 - waitWidth) / 2, 20);
  display.print(waitMsg);
  
  display.setFont(FONT_SMALL);
  const char* taringMsg = "Taring in progress";
  int taringWidth = display.getStrWidth(taringMsg);
  display.setCursor((128 - taringWidth) / 2, 35);
  display.print(taringMsg);
  
  // Barra di progresso
  display.drawFrame(10, 42, 108, 12);
  display.sendBuffer();
  
  const uint8_t barWidth = 25;
  
  // Tara celle con animazione
  display.drawBox(12, 44, barWidth - 2, 8);
  display.sendBuffer();
  scale1.tare();
  delay(200);
  
  display.drawBox(12 + barWidth, 44, barWidth - 2, 8);
  display.sendBuffer();
  scale2.tare();
  delay(200);
  
  display.drawBox(12 + 2 * barWidth, 44, barWidth - 2, 8);
  display.sendBuffer();
  scale3.tare();
  delay(200);
  
  display.drawBox(12 + 3 * barWidth, 44, barWidth - 2, 8);
  display.sendBuffer();
  scale4.tare();
  delay(200);
  
  // Azzera campioni smoothing
  for (uint8_t j = 0; j < SMOOTHING_SAMPLES; j++) {
    weight1Samples[j] = 0;
    weight2Samples[j] = 0;
    weight3Samples[j] = 0;
    weight4Samples[j] = 0;
  }
  
  // Messaggio completamento
  display.clearBuffer();
  display.setFont(FONT_LARGE);
  const char* doneMsg = "READY!";
  int doneWidth = display.getStrWidth(doneMsg);
  display.setCursor((128 - doneWidth) / 2, 32);
  display.print(doneMsg);
  display.sendBuffer();
  delay(1000);
  
  Serial.println(F("Tare completed"));
}

void showCGViewContinuous() {
  // Loop infinito di visualizzazione CG
  bool blinkState = false;
  unsigned long lastBlinkTime = 0;
  const unsigned long BLINK_INTERVAL = 500;  // Lampeggia ogni 500ms
  
  while (true) {
    // Controlla comandi seriali
    checkSerialCommands();
    
    // Controlla pulsanti
    if (digitalRead(BUTTON_K4) == LOW) {
      delay(100);
      // Reset stabilità quando si tara
      weightStable = false;
      previousWeight = 0;
      weightStableTime = 0;
      showCountdownAndTare();
      continue;
    }
    
    // Aggiorna dati
    unsigned long currentTime = millis();
    if (currentTime - lastReadTime >= READ_INTERVAL) {
      lastReadTime = currentTime;
      updateWeightData();
      
      // Verifica se il peso è stabile
      float weightDiff = abs(totalWeight - previousWeight);
      
      if (weightDiff < WEIGHT_STABILITY_THRESHOLD) {
        // Il peso è rimasto stabile
        if (weightStableTime == 0) {
          // Inizia il timer di stabilità
          weightStableTime = currentTime;
        } else if (currentTime - weightStableTime >= STABILITY_DURATION) {
          // Il peso è stabile da almeno 1 secondo
          weightStable = true;
        }
      } else {
        // Il peso è cambiato, reset del timer
        weightStable = false;
        weightStableTime = 0;
      }
      
      previousWeight = totalWeight;
      
      // Disegna schermata CG
      display.clearBuffer();
      
      // Titolo - lampeggia se il peso non è stabile
      if (!weightStable) {
        // Gestione lampeggio
        if (currentTime - lastBlinkTime >= BLINK_INTERVAL) {
          lastBlinkTime = currentTime;
          blinkState = !blinkState;
        }
        
        if (blinkState) {
          display.setFont(FONT_LARGE);
          const char* title = "CoG VIEW";
          int titleWidth = display.getStrWidth(title);
          display.setCursor((128 - titleWidth) / 2, 12);
          display.print(title);
        }
        // Se blinkState è false, non stampa nulla = effetto lampeggio
      } else {
        // Peso stabile - mostra sempre il titolo
        display.setFont(FONT_LARGE);
        const char* title = "CoG VIEW";
        int titleWidth = display.getStrWidth(title);
        display.setCursor((128 - titleWidth) / 2, 12);
        display.print(title);
      }
      
      // Scala - da 0 a 200mm con barra piena
      const uint8_t scaleY = 30;
      const uint8_t scaleWidth = 110;
      const uint8_t scaleX = (128 - scaleWidth) / 2;
      
      display.drawFrame(scaleX, scaleY - 3, scaleWidth, 6);
      
      // Graduazioni ogni 50mm
      display.setFont(FONT_SMALL2);
      for (uint8_t i = 0; i <= 200; i += 50) {
        uint8_t tickX = scaleX + map(i, 0, 200, 0, scaleWidth);
        display.drawLine(tickX, scaleY - 3, tickX, scaleY - 5);
        display.setCursor(tickX - 3, scaleY - 7);
        display.print(i);
      }
      
      // BARRA PIENA fino al punto del CG
      int16_t cogPosition = map(wingCogX, 0, 200, scaleX, scaleX + scaleWidth);
      cogPosition = constrain(cogPosition, scaleX, scaleX + scaleWidth);
      
      // Disegna la barra piena da 0 fino al CG
      int16_t barWidth = cogPosition - scaleX;
      if (barWidth > 0) {
        display.drawBox(scaleX + 1, scaleY - 2, barWidth, 4);
      }
      
      // Marker CG (cursore/triangolo sopra la barra)
      display.drawTriangle(
        cogPosition, scaleY + 6,
        cogPosition - 4, scaleY + 2,
        cogPosition + 4, scaleY + 2
      );
      display.drawBox(cogPosition - 2, scaleY + 2, 5, 4);
      
      // Valori - spostati più in alto per evitare sovrapposizione
      display.setFont(FONT_SMALL);
      int weightLabelWidth = display.getStrWidth("Weight: ");
      int cgXpos = 5 + weightLabelWidth - display.getStrWidth("CoG: ");
      
      display.setCursor(5, scaleY + 18);
      display.print(F("Weight: "));
      int valueXPos = display.getCursorX();
      display.print(totalWeight, 2);
      display.setCursor(display.getCursorX() + 3, scaleY + 18);
      display.print(F("kg"));
      
      display.setCursor(cgXpos, scaleY + 28);
      display.print(F("CoG: "));
      display.setCursor(valueXPos, scaleY + 28);
      display.print(wingCogX);
      display.setCursor(display.getCursorX() + 3, scaleY + 28);
      display.print(F("mm"));
      
      // Info pulsanti allineato a destra sulla stessa riga del CG
      display.setFont(FONT_SMALL2);
      const char* info = "K4=Tare";
      int infoWidth = display.getStrWidth(info);
      display.setCursor(128 - infoWidth - 2, scaleY + 28);  // -2 per margine dal bordo
      display.print(info);
      
      display.sendBuffer();
      
      // Output seriale
      Serial.print(F("Weight: "));
      Serial.print(totalWeight, 2);
      Serial.print(F(" kg | CG: "));
      Serial.print(wingCogX);
      Serial.println(F(" mm"));
      
      sampleIndex = (sampleIndex + 1) % SMOOTHING_SAMPLES;
    }
    
    delay(20);
  }
}

//================ AGGIORNAMENTO DATI ================//
void updateWeightData() {
  // Leggi valori
  float rawWeight1 = scale1.get_units(2);
  float rawWeight2 = scale2.get_units(2);
  float rawWeight3 = scale3.get_units(2);
  float rawWeight4 = scale4.get_units(2);
  
  // Applica correzione
  float correctedWeight1 = rawWeight1 * CORRECTION_FACTOR;
  float correctedWeight2 = rawWeight2 * CORRECTION_FACTOR;
  float correctedWeight3 = rawWeight3 * CORRECTION_FACTOR;
  float correctedWeight4 = rawWeight4 * CORRECTION_FACTOR;
  
  // Smooth e arrotonda
  float weight1 = roundWeightToDecagram(smoothValue(correctedWeight1, weight1Samples, sampleIndex));
  float weight2 = roundWeightToDecagram(smoothValue(correctedWeight2, weight2Samples, sampleIndex));
  float weight3 = roundWeightToDecagram(smoothValue(correctedWeight3, weight3Samples, sampleIndex));
  float weight4 = roundWeightToDecagram(smoothValue(correctedWeight4, weight4Samples, sampleIndex));
  
  totalWeight = weight1 + weight2 + weight3 + weight4;
  
  // Calcola CG
  if (totalWeight > MIN_WEIGHT_THRESHOLD) {
    const float Y1 = 0.0;
    const float Y2 = 0.0;
    const float Y3 = BASE_LENGTH;
    const float Y4 = BASE_LENGTH;
    
    float cogY_fromLE = (weight1*Y1 + weight2*Y2 + weight3*Y3 + weight4*Y4) / totalWeight;
    wingCogX = round(cogY_fromLE);
  } else {
    wingCogX = 0;
  }
}

//================ CALIBRAZIONE VIA SERIALE ================//
/**
 * Funzione per la calibrazione via seriale
 */
void calibrateCellsViaSerial() {
  Serial.println(F("\n========================================"));
  Serial.println(F("===  CALIBRATION PROCEDURE START  ==="));
  Serial.println(F("========================================"));
  
  // Mostra messaggio sul display
  display.clearBuffer();
  display.setFont(FONT_LARGE);
  display.setCursor(5, 15);
  display.print(F("CALIBRATION"));
  
  display.setFont(FONT_SMALL);
  display.setCursor(5, 30);
  display.print(F("Follow instructions"));
  display.setCursor(5, 42);
  display.print(F("on Serial Monitor"));
  display.setCursor(5, 54);
  display.print(F("Do not touch scale"));
  display.sendBuffer();
  
  Serial.println(F("\nSTEP 1: TARE"));
  Serial.println(F("Remove ALL weight from the scale"));
  Serial.println(F("Send 't' when ready to tare..."));
  
  // Attende comando di tara
  while (true) {
    if (Serial.available() > 0) {
      char cmd = Serial.read();
      if (cmd == 't' || cmd == 'T') {
        Serial.println(F("\n>>> Taring all cells..."));
        scale1.tare();
        scale2.tare();
        scale3.tare();
        scale4.tare();
        Serial.println(F(">>> Tare completed!"));
        delay(500);
        break;
      }
    }
    delay(10);
  }
  
  Serial.println(F("\nSTEP 2: CALIBRATION WEIGHT"));
  Serial.println(F("Place a known weight at the CENTER of the scale"));
  
  // Aggiorna display
  display.clearBuffer();
  display.setFont(FONT_LARGE);
  display.setCursor(5, 15);
  display.print(F("CALIBRATION"));
  
  display.setFont(FONT_SMALL);
  display.setCursor(5, 30);
  display.print(F("Place known weight"));
  display.setCursor(5, 42);
  display.print(F("at CENTER"));
  display.setCursor(5, 54);
  display.print(F("Enter value on serial"));
  display.sendBuffer();
  
  delay(1000);
  
  // Pulisci buffer seriale prima di attendere input
  while (Serial.available() > 0) {
    Serial.read();
    delay(10);
  }
  
  Serial.println(F("\nEnter the weight in kg (e.g., 2.335)"));
  Serial.print(F(">>> "));
  
  // Attende input del peso noto
  float knownWeight = 0;
  
  while (knownWeight <= 0) {
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      input.trim(); // Rimuove spazi e caratteri di controllo
      
      knownWeight = input.toFloat();
      
      if (knownWeight > 0) {
        Serial.println(input); // Echo dell'input
        Serial.print(F(">>> Weight entered: "));
        Serial.print(knownWeight, 3);
        Serial.println(F(" kg"));
      } else {
        Serial.println(input);
        Serial.println(F("!!! Invalid weight! Must be positive."));
        Serial.print(F(">>> Try again: "));
        knownWeight = 0; // Reset per continuare il loop
      }
    }
    delay(50);
  }
  
  Serial.println(F("\nSTEP 3: READING RAW VALUES"));
  Serial.println(F("Please wait..."));
  
  // Mostra barra di progresso
  display.clearBuffer();
  display.setFont(FONT_MEDIUM);
  display.setCursor(10, 15);
  display.print(F("Calculating..."));
  display.drawFrame(10, 25, 108, 12);
  display.sendBuffer();
  
  delay(1000); // Pausa per stabilizzazione
  
  // Leggi valori grezzi da tutte le celle
  Serial.println(F(">>> Reading cell 1..."));
  display.drawBox(12, 27, 25, 8);
  display.sendBuffer();
  float raw1 = scale1.get_value(10);
  Serial.print(F("    Raw value: "));
  Serial.println(raw1);
  
  Serial.println(F(">>> Reading cell 2..."));
  display.drawBox(39, 27, 25, 8);
  display.sendBuffer();
  float raw2 = scale2.get_value(10);
  Serial.print(F("    Raw value: "));
  Serial.println(raw2);
  
  Serial.println(F(">>> Reading cell 3..."));
  display.drawBox(66, 27, 25, 8);
  display.sendBuffer();
  float raw3 = scale3.get_value(10);
  Serial.print(F("    Raw value: "));
  Serial.println(raw3);
  
  Serial.println(F(">>> Reading cell 4..."));
  display.drawBox(93, 27, 25, 8);
  display.sendBuffer();
  float raw4 = scale4.get_value(10);
  Serial.print(F("    Raw value: "));
  Serial.println(raw4);
  
  // Calcola i nuovi fattori di calibrazione
  Serial.println(F("\nSTEP 4: CALCULATING CALIBRATION FACTORS"));
  
  Serial.println(F(">>> Individual raw values:"));
  Serial.print(F("    Cell 1: ")); Serial.println(raw1);
  Serial.print(F("    Cell 2: ")); Serial.println(raw2);
  Serial.print(F("    Cell 3: ")); Serial.println(raw3);
  Serial.print(F("    Cell 4: ")); Serial.println(raw4);
  
  float rawSum = raw1 + raw2 + raw3 + raw4;
  Serial.print(F(">>> Total raw value: "));
  Serial.println(rawSum);
  
  if (abs(rawSum) < 1000) {
    Serial.println(F("\n!!! ERROR: Raw sum too low!"));
    Serial.println(F("!!! Check connections and try again."));
    delay(3000);
    return;
  }
  
  // FORMULA CORRETTA per bilancia a 4 celle:
  // Quando il peso è distribuito su 4 celle, ogni cella legge circa 1/4 del peso totale
  // Quindi: calibration_factor = raw_value / (peso_totale / 4)
  // Che è equivalente a: calibration_factor = (raw_value * 4) / peso_totale
  
  float weightPerCell = knownWeight / 4.0;
  
  calibration_factor1 = raw1 / weightPerCell;
  calibration_factor2 = raw2 / weightPerCell;
  calibration_factor3 = raw3 / weightPerCell;
  calibration_factor4 = raw4 / weightPerCell;
  
  Serial.println(F(">>> Calculated factors:"));
  Serial.print(F("    Cell 1: ")); Serial.println(calibration_factor1, 2);
  Serial.print(F("    Cell 2: ")); Serial.println(calibration_factor2, 2);
  Serial.print(F("    Cell 3: ")); Serial.println(calibration_factor3, 2);
  Serial.print(F("    Cell 4: ")); Serial.println(calibration_factor4, 2);
  
  // Applica i nuovi fattori
  scale1.set_scale(calibration_factor1);
  scale2.set_scale(calibration_factor2);
  scale3.set_scale(calibration_factor3);
  scale4.set_scale(calibration_factor4);
  
  Serial.println(F("\n========================================"));
  Serial.println(F("===  NEW CALIBRATION FACTORS  ==="));
  Serial.println(F("========================================"));
  Serial.print(F("calibration_factor1 = "));
  Serial.print(calibration_factor1, 2);
  Serial.println(F(";"));
  Serial.print(F("calibration_factor2 = "));
  Serial.print(calibration_factor2, 2);
  Serial.println(F(";"));
  Serial.print(F("calibration_factor3 = "));
  Serial.print(calibration_factor3, 2);
  Serial.println(F(";"));
  Serial.print(F("calibration_factor4 = "));
  Serial.print(calibration_factor4, 2);
  Serial.println(F(";"));
  Serial.println(F("========================================"));
  
  Serial.println(F("\nCOPY these values to your code and reupload!"));
  
  // Test della calibrazione
  Serial.println(F("\nSTEP 5: VERIFICATION"));
  Serial.println(F("Testing calibration with reference weight..."));
  delay(1000);
  
  float test1 = scale1.get_units(5);
  float test2 = scale2.get_units(5);
  float test3 = scale3.get_units(5);
  float test4 = scale4.get_units(5);
  float testTotal = test1 + test2 + test3 + test4;
  
  Serial.println(F("\n>>> Measured weights:"));
  Serial.print(F("    Cell 1: "));
  Serial.print(test1, 3);
  Serial.println(F(" kg"));
  Serial.print(F("    Cell 2: "));
  Serial.print(test2, 3);
  Serial.println(F(" kg"));
  Serial.print(F("    Cell 3: "));
  Serial.print(test3, 3);
  Serial.println(F(" kg"));
  Serial.print(F("    Cell 4: "));
  Serial.print(test4, 3);
  Serial.println(F(" kg"));
  Serial.print(F("    TOTAL:  "));
  Serial.print(testTotal, 3);
  Serial.println(F(" kg"));
  
  float error = abs(testTotal - knownWeight);
  float errorPercent = (error / knownWeight) * 100.0;
  
  Serial.print(F("\n>>> Expected: "));
  Serial.print(knownWeight, 3);
  Serial.println(F(" kg"));
  Serial.print(F(">>> Error: "));
  Serial.print(error, 3);
  Serial.print(F(" kg ("));
  Serial.print(errorPercent, 2);
  Serial.println(F("%)"));
  
  if (errorPercent < 2.0) {
    Serial.println(F("\n✓ Calibration SUCCESSFUL!"));
  } else if (errorPercent < 5.0) {
    Serial.println(F("\n⚠ Calibration acceptable, but could be better"));
  } else {
    Serial.println(F("\n✗ Calibration needs improvement - try again"));
  }
  
  Serial.println(F("\n========================================"));
  Serial.println(F("===  CALIBRATION COMPLETE  ==="));
  Serial.println(F("========================================\n"));
  
  // Mostra risultato sul display
  display.clearBuffer();
  display.setFont(FONT_LARGE);
  display.setCursor(5, 20);
  display.print(F("CALIBRATION"));
  display.setCursor(20, 38);
  display.print(F("COMPLETE!"));
  
  display.setFont(FONT_SMALL);
  display.setCursor(5, 55);
  display.print(F("Check Serial Monitor"));
  display.sendBuffer();
  
  delay(5000);
}

/**
 * Mostra i fattori di calibrazione correnti
 */
void showCalibrationFactors() {
  Serial.println(F("\n========================================"));
  Serial.println(F("===  CURRENT CALIBRATION FACTORS  ==="));
  Serial.println(F("========================================"));
  Serial.print(F("calibration_factor1 = "));
  Serial.print(calibration_factor1, 2);
  Serial.println(F(";"));
  Serial.print(F("calibration_factor2 = "));
  Serial.print(calibration_factor2, 2);
  Serial.println(F(";"));
  Serial.print(F("calibration_factor3 = "));
  Serial.print(calibration_factor3, 2);
  Serial.println(F(";"));
  Serial.print(F("calibration_factor4 = "));
  Serial.print(calibration_factor4, 2);
  Serial.println(F(";"));
  Serial.println(F("========================================\n"));
}

/**
 * Controlla i comandi dalla porta seriale
 */
void checkSerialCommands() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    // Pulisci eventuali caratteri rimanenti
    while (Serial.available() > 0) {
      Serial.read();
      delay(10);
    }
    
    switch (cmd) {
      case 'c':
      case 'C':
        Serial.println(F("\n>>> Starting calibration procedure..."));
        calibrateCellsViaSerial();
        break;
        
      case 's':
      case 'S':
        showCalibrationFactors();
        break;
        
      case 'z':
      case 'Z':
        Serial.println(F("\n>>> Quick tare..."));
        performTare();
        Serial.println(F(">>> Tare completed"));
        break;
        
      default:
        // Ignora altri caratteri
        break;
    }
  }
}
