/**
 * Sistema di misura del Centro di Gravità (C.o.G.)
 * per aeromodelli utilizzando quattro celle di carico.
 * Sviluppato da RZ.
 * Ottimizzato e aggiornato con U8g2: 11/05/2025
 * 
 * Il sistema utilizza:
 * - 4 celle di carico HX711 (una per ogni angolo)
 * - Display OLED SSD1309 da 2.42" con interfaccia SPI
 * - 4 pulsanti per l'interfaccia utente
 * 
 * ISTRUZIONI PER LA CALIBRAZIONE:
 * ------------------------------
 * 1. CALIBRAZIONE INIZIALE (da fare una sola volta):
 *    a. Inviare 'c' sulla porta seriale per entrare in modalità calibrazione
 *    b. Seguire le istruzioni a schermo e sulla porta seriale
 *    c. Rimuovere tutto il peso dalla bilancia e inviare 't' per tarare
 *    d. Posizionare un peso noto al centro e inserire il valore in kg
 *    e. Attendere il completamento della calibrazione
 * 
 * 2. REGOLAZIONE DEL FATTORE DI CORREZIONE:
 *    a. Posizionare un peso di riferimento noto (es. 2.352 kg) sulla bilancia
 *    b. Osservare i valori "Raw Weights" e "Corrected Weights" sulla porta seriale
 *    c. Se il peso corretto non corrisponde al peso di riferimento, calcolare:
 *       Nuovo fattore = Fattore attuale * (Peso riferimento / Peso misurato)
 *    d. Inviare il comando: cf,nuovofattore (es. cf,4.71192)
 *    e. Annotare il nuovo fattore e modificare la costante CORRECTION_FACTOR 
 *       nella funzione updateWeightData()
 *    f. Ricompilare e caricare il codice
 *    g. Ripetere se necessario fino a ottenere misurazioni accurate
 */

//================ INCLUSIONI LIBRERIE ================//
#include "HX711.h"
#include <SPI.h>
#include <U8g2lib.h>

//================ DEFINIZIONE PIN E COSTANTI ================//
// Pin per le 4 celle di carico
const int LOADCELL1_DOUT_PIN = 2;  // Cella 1 (angolo sinistro anteriore)
const int LOADCELL1_SCK_PIN = 3;
const int LOADCELL2_DOUT_PIN = 4;  // Cella 2 (angolo destro anteriore)
const int LOADCELL2_SCK_PIN = 5;
const int LOADCELL3_DOUT_PIN = 6;  // Cella 3 (angolo sinistro posteriore)
const int LOADCELL3_SCK_PIN = 7;
const int LOADCELL4_DOUT_PIN = 8;  // Cella 4 (angolo destro posteriore)
const int LOADCELL4_SCK_PIN = 9;

// Pin per i pulsanti
const uint8_t BUTTON_K1 = A2;  // Menu/OK
const uint8_t BUTTON_K2 = A3;  // Su/Precedente
const uint8_t BUTTON_K3 = A5;  // Giù/Successivo
const uint8_t BUTTON_K4 = A4;  // Indietro/Tara

// Pin SPI per il display OLED SSD1309
#define OLED_MOSI     11    // Pin SDA/MOSI
#define OLED_CLK      13    // Pin SCL/SCK
#define OLED_DC       A1    // Pin DC (Data/Command)
#define OLED_CS       10    // Pin CS (Chip Select)
#define OLED_RESET    A0    // Pin RESET

// Dimensioni fisiche della struttura (in mm)
const float BASE_WIDTH = 270.0;     // Larghezza della base (asse X)
const float BASE_LENGTH = 250.0;    // Lunghezza della base (asse Y)
const float WING_PEG_DIST = 120.0;  // Distanza tra i due piattelli di appoggio dell'ala
const float LE_STOPPER_DIST = 30.0; // Distanza dal bordo sinistro al primo piattello

// Parametri di smoothing e timing
const uint8_t SMOOTHING_SAMPLES = 5;      // Numero di campioni per lo smoothing
const uint16_t READ_INTERVAL = 1000;      // Intervallo di lettura in ms
const unsigned long DEBOUNCE_DELAY = 50;  // Tempo di debounce per i pulsanti in ms

//================ DEFINIZIONE OGGETTI ================//
// Istanze delle celle di carico
HX711 scale1;
HX711 scale2;
HX711 scale3;
HX711 scale4;

// Definizione del display OLED con U8g2 - utilizziamo hardware SPI
U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI display(U8G2_R0, OLED_CS, OLED_DC, OLED_RESET);

//================ DEFINIZIONE FONT ================//
#define FONT_TINY u8g2_font_tom_thumb_4x6_tf     // Circa 4 pixel di altezza, molto compatto
#define FONT_SMALL2 u8g2_font_u8glib_4_tf        // Circa 5-6 pixel di altezza
#define FONT_SMALL1 u8g2_font_4x6_tf             // Circa 6-7 pixel di altezza
#define FONT_SMALL u8g2_font_5x8_tf              // Circa 8 pixel di altezza
#define FONT_MEDIUM_SMALL u8g2_font_6x12_tf      // Circa 8-9 pixel di altezza, intermedio
#define FONT_MEDIUM u8g2_font_6x10_tf            // Circa 10 pixel di altezza
#define FONT_LARGE u8g2_font_9x15_tf             // Circa 15 pixel di altezza
#define FONT_NUMERIC u8g2_font_logisoso16_tn     // Circa 16 pixel di altezza, solo numeri

//================ VARIABILI GLOBALI ================//
// Parametri di calibrazione
float calibration_factor1 = -411979.59;
float calibration_factor2 = -237715.98;
float calibration_factor3 = -350986.41;
float calibration_factor4 = -533862.25;

// Variabili per il loop principale
unsigned long lastReadTime = 0;
bool continuousMode = true;  // Modalità di lettura continua attiva di default
bool taraEffettuata = false;

// Gestione dei pulsanti
uint8_t buttonStates = 0xFF;  // Tutti i pulsanti rilasciati (HIGH)
unsigned long lastDebounceTime = 0;

// Gestione del menu
uint8_t menuState = 0;    // 0: visualizzazione principale, 1: menu, 2-5: sottomenu
uint8_t menuPosition = 0; // Posizione nel menu corrente
bool menuActive = false;

// Voci del menu principale
const char* const mainMenuItems[] = {
  "Scale Tare",
  "CG View",
  "Lateral Balance"
};
const uint8_t MAIN_MENU_ITEMS = sizeof(mainMenuItems) / sizeof(mainMenuItems[0]);

// Variabili per lo smoothing
float weight1Samples[SMOOTHING_SAMPLES] = {0};
float weight2Samples[SMOOTHING_SAMPLES] = {0};
float weight3Samples[SMOOTHING_SAMPLES] = {0};
float weight4Samples[SMOOTHING_SAMPLES] = {0};
uint8_t sampleIndex = 0;

// Variabili per il calcolo del centro di gravità
float totalWeight = 0;
int16_t cogX = 0;
int16_t cogY = 0;
int16_t wingCogX = 0;
int16_t lateralCogDiff = 0;
int16_t cogPercentage = 0;

//================ PROTOTIPI FUNZIONI ================//
void showWelcomeScreen();
void showReadyScreen();
void showMainMenu();
void showCGView();
void showLateralBalance();
void tareScales();
void toggleContinuousMode();
void updateWeightData();
void printSerialData();
void checkButtons();
void checkSerialCommands();
void calibrateCellsViaSerial();
void showCalibrationFactors();
void setCorrectionFactor(float newFactor);
float smoothValue(float newValue, float samples[], uint8_t index);
float roundWeightToDecagram(float weight);
int16_t roundToMillimeter(float value);

//================ FUNZIONI DI UTILITÀ ================//
/**
 * Funzione per arrotondare un valore alle decine di grammi (0.01 kg)
 */
inline float roundWeightToDecagram(float weight) {
  return round(weight * 100) / 100.0;  // Arrotonda a 0.01 kg (10g)
}

/**
 * Funzione per arrotondare un valore al millimetro
 */
inline int16_t roundToMillimeter(float value) {
  return round(value);  // Arrotonda al millimetro intero
}

/**
 * Funzione per effettuare lo smooth di una lettura
 */
float smoothValue(float newValue, float samples[], uint8_t index) {
  // Aggiorna il campione corrente
  samples[index] = newValue;
  
  // Calcola la media di tutti i campioni
  float sum = 0;
  for (uint8_t i = 0; i < SMOOTHING_SAMPLES; i++) {
    sum += samples[i];
  }
  
  return sum / SMOOTHING_SAMPLES;
}

//================ FUNZIONI PRINCIPALI ================//
/**
 * Configurazione iniziale del sistema
 */
void setup() {
  Serial.begin(9600);
  Serial.println(F("C.o.G. System v2.0 - Initialization..."));
  
  // DEBUG: Mostra i fattori di calibrazione all'avvio
  Serial.println(F("Current calibration factors:"));
  Serial.print(F("Cell 1: ")); Serial.println(calibration_factor1);
  Serial.print(F("Cell 2: ")); Serial.println(calibration_factor2);
  Serial.print(F("Cell 3: ")); Serial.println(calibration_factor3);
  Serial.print(F("Cell 4: ")); Serial.println(calibration_factor4);
  
  // Inizializzazione delle celle di carico
  Serial.println(F("Initializing load cells..."));
  
  scale1.begin(LOADCELL1_DOUT_PIN, LOADCELL1_SCK_PIN);
  scale2.begin(LOADCELL2_DOUT_PIN, LOADCELL2_SCK_PIN);
  scale3.begin(LOADCELL3_DOUT_PIN, LOADCELL3_SCK_PIN);
  scale4.begin(LOADCELL4_DOUT_PIN, LOADCELL4_SCK_PIN);
  
  // Imposta i fattori di calibrazione
  scale1.set_scale(calibration_factor1);
  scale2.set_scale(calibration_factor2);
  scale3.set_scale(calibration_factor3);
  scale4.set_scale(calibration_factor4);
  
  // Configurazione pulsanti come input con pull-up interno
  pinMode(BUTTON_K1, INPUT_PULLUP);
  pinMode(BUTTON_K2, INPUT_PULLUP);
  pinMode(BUTTON_K3, INPUT_PULLUP);
  pinMode(BUTTON_K4, INPUT_PULLUP);
  
  // Inizializzazione del display OLED con U8G2
  Serial.println(F("Initializing display..."));
  display.begin();
  display.clearBuffer();
  
  // Mostra la schermata di benvenuto
  showWelcomeScreen();
  
  // Tara iniziale di tutte le celle
  Serial.println(F("Taring all cells..."));
  scale1.tare();
  scale2.tare();
  scale3.tare();
  scale4.tare();
  
  taraEffettuata = true;
  
  // Mostra la schermata iniziale
  showReadyScreen();
  
  // Istruzioni per l'utente sulla seriale
  Serial.println(F("C.o.G. Measurement System Ready"));
  Serial.println(F("Available commands:"));
  Serial.println(F("z - Zero/Tare"));
  Serial.println(F("p - Pause/Resume continuous reading"));
  Serial.println(F("s - Show calibration factors"));
  Serial.println(F("c - Start calibration"));
  Serial.println(F("d - Debug mode (raw values)"));
  Serial.println(F("m - Show Main Menu"));
  Serial.println(F("cf,value - Set correction factor"));
  Serial.println(F("c1,value - Set calibration for cell 1"));
  Serial.println(F("Press K1 to access the main menu"));
  
  Serial.println(F("System ready - press K1 for menu"));
}

/**
 * Loop principale del programma
 */
void loop() {
  // Gestione dei pulsanti
  checkButtons();
  
  // Gestione dei comandi seriali
  checkSerialCommands();
  
  // Lettura e visualizzazione dei valori
  unsigned long currentTime = millis();
  if (continuousMode && (currentTime - lastReadTime >= READ_INTERVAL)) {
    lastReadTime = currentTime;
    
    // Legge i dati dalle celle di carico
    updateWeightData();
    
    // Se non siamo in un menu, mostra la schermata iniziale
    if (!menuActive) {
      // Visualizza la schermata "READY TO MEASURE" modificata
      showReadyScreen();
    }
    
    // Output seriale per debug o logging
    printSerialData();
    
    // Incrementa l'indice del campione per lo smoothing
    sampleIndex = (sampleIndex + 1) % SMOOTHING_SAMPLES;
  }
}

/**
 * Funzione per aggiornare i dati di peso e centro di gravità
 */
void updateWeightData() {
  // Ottieni i valori dalle celle di carico con smoothing
  float rawWeight1 = scale1.get_units(2);
  float rawWeight2 = scale2.get_units(2);
  float rawWeight3 = scale3.get_units(2);
  float rawWeight4 = scale4.get_units(2);
  
  // Applica il fattore di correzione manuale
  const float CORRECTION_FACTOR = 4.71192; // Calcolato da 2.352 / 0.440
  
  // Applica la correzione ai valori grezzi
  float correctedWeight1 = rawWeight1 * CORRECTION_FACTOR;
  float correctedWeight2 = rawWeight2 * CORRECTION_FACTOR;
  float correctedWeight3 = rawWeight3 * CORRECTION_FACTOR;
  float correctedWeight4 = rawWeight4 * CORRECTION_FACTOR;
  
  // Applica lo smooth e l'arrotondamento
  float weight1 = roundWeightToDecagram(smoothValue(correctedWeight1, weight1Samples, sampleIndex));
  float weight2 = roundWeightToDecagram(smoothValue(correctedWeight2, weight2Samples, sampleIndex));
  float weight3 = roundWeightToDecagram(smoothValue(correctedWeight3, weight3Samples, sampleIndex));
  float weight4 = roundWeightToDecagram(smoothValue(correctedWeight4, weight4Samples, sampleIndex));
  
  // Calcola il peso totale
  totalWeight = weight1 + weight2 + weight3 + weight4;
  
  // Debug per verificare il risultato
  Serial.print(F("Raw Weights: "));
  Serial.print(rawWeight1, 3); Serial.print(F(" + "));
  Serial.print(rawWeight2, 3); Serial.print(F(" + "));
  Serial.print(rawWeight3, 3); Serial.print(F(" + "));
  Serial.print(rawWeight4, 3); Serial.print(F(" = "));
  Serial.print(rawWeight1 + rawWeight2 + rawWeight3 + rawWeight4, 3);
  Serial.println(F(" kg"));
  
  Serial.print(F("Corrected Weights: "));
  Serial.print(weight1, 3); Serial.print(F(" + "));
  Serial.print(weight2, 3); Serial.print(F(" + "));
  Serial.print(weight3, 3); Serial.print(F(" + "));
  Serial.print(weight4, 3); Serial.print(F(" = "));
  Serial.print(totalWeight, 3);
  Serial.println(F(" kg"));
  
  // Calcola il baricentro solo se c'è peso sufficiente
  if (totalWeight > 0.01) {
    // Definiamo le coordinate X e Y delle 4 celle di carico
    // Assumendo che l'origine sia nel punto frontale medio (tra i Leading Edge pins):
    //
    // Coordinate X (laterali): 
    // - Negativo a sinistra del centro
    // - Positivo a destra del centro
    const float X1 = -BASE_WIDTH / 2.0;   // Cella 1 (LF): angolo sinistro anteriore
    const float X2 = BASE_WIDTH / 2.0;    // Cella 2 (RF): angolo destro anteriore
    const float X3 = -BASE_WIDTH / 2.0;   // Cella 3 (LR): angolo sinistro posteriore
    const float X4 = BASE_WIDTH / 2.0;    // Cella 4 (RR): angolo destro posteriore
    
    // Coordinate Y (longitudinali):
    // - Zero ai Leading Edge pins (fronte)
    // - Positivo verso il retro
    const float Y1 = 0.0;                 // Cella 1 (LF): angolo sinistro anteriore
    const float Y2 = 0.0;                 // Cella 2 (RF): angolo destro anteriore
    const float Y3 = BASE_LENGTH;         // Cella 3 (LR): angolo sinistro posteriore
    const float Y4 = BASE_LENGTH;         // Cella 4 (RR): angolo destro posteriore
    
    // Calcolo del centro di gravità usando la formula completa a 4 punti
    // X = (W1*X1 + W2*X2 + W3*X3 + W4*X4) / (W1 + W2 + W3 + W4)
    // Y = (W1*Y1 + W2*Y2 + W3*Y3 + W4*Y4) / (W1 + W2 + W3 + W4)
    
    // Calcolo del CoG laterale rispetto al centro (positivo verso destra)
    float cogX_centered = (weight1*X1 + weight2*X2 + weight3*X3 + weight4*X4) / totalWeight;
    lateralCogDiff = roundToMillimeter(cogX_centered);
    
    // Calcolo del CoG longitudinale (dal bordo d'attacco verso il bordo d'uscita)
    // Questo è il valore wingCogX che DIMINUISCE quando il peso si sposta in avanti
    float cogY_fromLE = (weight1*Y1 + weight2*Y2 + weight3*Y3 + weight4*Y4) / totalWeight;
    cogY = roundToMillimeter(cogY_fromLE);
    
    // Il baricentro dell'ala è esattamente cogY (misurato dal bordo d'attacco)
    wingCogX = cogY;
    
    // Calcolo percentuale rispetto alla distanza dei piattelli dell'ala
    if (WING_PEG_DIST > 0) {
      cogPercentage = round((wingCogX / WING_PEG_DIST) * 100.0);
    }

    // Debug
    Serial.print(F("Lateral balance: "));
    if (lateralCogDiff > 0) Serial.print(F("+"));
    Serial.print(lateralCogDiff);
    Serial.println(F(" mm from center"));
    
    Serial.print(F("Wing CG: "));
    Serial.print(wingCogX);
    Serial.println(F(" mm from Leading Edge"));
  } else {
    // Se non c'è peso sufficiente, imposta valori di default
    cogX = 0;
    cogY = 0;
    wingCogX = 0;
    lateralCogDiff = 0;
    cogPercentage = 0;
  }
}

//================ FUNZIONI PER L'INTERFACCIA UTENTE ================//
/**
 * Mostra la schermata di benvenuto
 */
void showWelcomeScreen() {
  display.clearBuffer();
  
  // Disegna un bordo decorativo più spesso
  display.drawFrame(0, 0, 128, 64);
  display.drawFrame(2, 2, 124, 60); // Doppio bordo per effetto maggiore
  
  // Titolo principale con font grande e bold
  display.setFont(u8g2_font_inb19_mr); // Font Impact 19px - molto bold e accattivante
  
  // Centra "C.o.G."
  const char* mainTitle = "C.o.G.";
  int titleWidth = display.getStrWidth(mainTitle);
  int xPos = (128 - titleWidth) / 2;
  display.setCursor(xPos, 25);
  display.print(mainTitle);
  
  // Sottotitolo "Scale" con font slightly smaller ma sempre bold
  display.setFont(u8g2_font_inb16_mr); // Font Impact 16px
  const char* subTitle = "Scale";
  int subTitleWidth = display.getStrWidth(subTitle);
  xPos = (128 - subTitleWidth) / 2;
  display.setCursor(xPos, 46);
  display.print(subTitle);
  
  // Versione e autore - più piccoli ma ben leggibili
  display.setFont(FONT_SMALL);
  const char* versionText = "v2.0 by RZ - 2025";
  int versionWidth = display.getStrWidth(versionText);
  xPos = (128 - versionWidth) / 2;
  display.setCursor(xPos, 60);
  display.print(versionText);
  
  // Aggiorna il display
  display.sendBuffer();
  delay(2500); // Un po' più di tempo per apprezzare la schermata
}

/**
 * Mostra la schermata Ready to Measure
 */
void showReadyScreen() {
  display.clearBuffer();
  
  // Disegna una cornice rettangolare che racchiude tutto
  display.drawFrame(2, 2, 124, 60); // Margine di 2 pixel su ogni lato
  
  // Titolo centrato in alto
  display.setFont(FONT_MEDIUM);
  const char* title = "Ready to Measure";
  int titleWidth = display.getStrWidth(title);
  int xPos = (128 - titleWidth) / 2;
  display.setCursor(xPos, 15);
  display.print(title);
  
  // Istruzioni - senza la scritta "Controls:"
  display.setFont(FONT_SMALL);
  
  // Comando K1 per il menu principale - centrato
  const char* k1Text = "K1 = Main Menu";
  int k1Width = display.getStrWidth(k1Text);
  display.setCursor((128 - k1Width) / 2, 30);
  display.print(k1Text);

  // Aggiunta descrizione comandi seriali - centrato
  const char* serialText = "Serial: z=tare, c=cal";
  int serialWidth = display.getStrWidth(serialText);
  display.setCursor((128 - serialWidth) / 2, 42);
  display.print(serialText);

  // Tare Scale before use - centrato
  const char* tareText = "Tare Scale before use";
  int tareWidth = display.getStrWidth(tareText);
  display.setCursor((128 - tareWidth) / 2, 54);
  display.print(tareText);
    
  // Invio del buffer al display
  display.sendBuffer();
}

/**
 * Funzione per la schermata CG View
 */
void showCGView() {
  display.clearBuffer();
  
  // Intestazione
  display.setFont(FONT_LARGE);
  // Centra il titolo
  const char* title = "CG VIEW";
  int titleWidth = display.getStrWidth(title);
  int xPos = (128 - titleWidth) / 2;
  display.setCursor(xPos, 12);
  display.print(title);
  
  // Calcola le coordinate per la scala
  const uint8_t scaleY = 30;
  const uint8_t scaleWidth = 110; // Leggermente più larga
  const uint8_t scaleX = (128 - scaleWidth) / 2;
  
  // Disegna la scala del baricentro (1-200mm)
  display.drawFrame(scaleX, scaleY - 3, scaleWidth, 6); // Ridotta altezza da 10 a 6 pixel
  
  // Disegna graduazioni ogni 50mm con etichette più piccole
  display.setFont(FONT_SMALL2); // Font più piccolo per le etichette
  for (uint8_t i = 0; i <= 200; i += 50) {
    uint8_t tickX = scaleX + map(i, 0, 200, 0, scaleWidth);
    display.drawLine(tickX, scaleY - 3, tickX, scaleY - 5); // Tick marks più corti
    
    // Etichette per le graduazioni
    display.setCursor(tickX - 3, scaleY - 7);
    display.print(i);
  }
  
  // Calcola la posizione proporzionale del baricentro
  int16_t cogPosition = map(wingCogX, 0, 200, scaleX, scaleX + scaleWidth);
  
  // Assicura che il baricentro non esca dai limiti
  cogPosition = constrain(cogPosition, scaleX, scaleX + scaleWidth);
  
  // Disegna un marker per il baricentro (un triangolo)
  display.drawTriangle(
    cogPosition, scaleY + 6,
    cogPosition - 4, scaleY + 2,
    cogPosition + 4, scaleY + 2
  );
  display.drawBox(
    cogPosition - 2, scaleY + 2,
    5, 4
  );
  
  // Informazioni testuali
  display.setFont(FONT_SMALL);
  
  // Prima determiniamo la larghezza del testo "Weight: " per allineare "CG: "
  int weightLabelWidth = display.getStrWidth("Weight: ");
  
  // Calcoliamo la posizione X per "CG" in modo che i due punti si allineino
  int cgXpos = 5 + weightLabelWidth - display.getStrWidth("CG: ");
  
  // Visualizza il peso totale - assicurandoci di cancellare qualsiasi resto precedente
  display.setDrawColor(0); // Modalità cancellazione (nero)
  display.drawBox(5, scaleY + 15, 120, 30); // Area ampia per cancellare qualsiasi sovrapposizione
  
  display.setDrawColor(1); // Torna alla modalità normale (bianco)
  display.setCursor(5, scaleY + 20);
  display.print(F("Weight: "));
  
  // Salviamo la posizione X dopo "Weight: "
  int valueXPos = display.getCursorX();
  
  // Stampiamo il valore del peso
  display.print(totalWeight, 2);
  
  // Sposta l'unità di misura "kg" a destra di 3 posizioni
  display.setCursor(display.getCursorX() + 3, scaleY + 20);
  display.print(F("kg"));
  
  // Visualizza il baricentro - con allineamento dei due punti
  display.setCursor(cgXpos, scaleY + 32); // Posizione X calcolata per allineare i due punti
  display.print(F("CG: "));
  
  // Ci assicuriamo che il valore del CG inizi alla stessa posizione X del peso
  display.setCursor(valueXPos, scaleY + 32);
  display.print(wingCogX);
  
  // Sposta l'unità di misura "mm" a destra di 3 posizioni
  display.setCursor(display.getCursorX() + 3, scaleY + 32);
  display.print(F("mm"));
  
  // Aggiorna il display
  display.sendBuffer();
  
  // Attendi la pressione di K4 per tornare indietro
  bool waitForBack = true;
  unsigned long updateTimer = 0;
  
  while (waitForBack) {
    if (digitalRead(BUTTON_K4) == LOW) {
      waitForBack = false;
      delay(100); // Debounce
    }
    
    // Aggiorna i dati in tempo reale (ogni 500ms)
    unsigned long currentTime = millis();
    if (currentTime - updateTimer >= 500) {
      updateTimer = currentTime;
      updateWeightData();
      
      // Aggiorna solo la parte dinamica del display
      display.setDrawColor(0); // Modalità cancellazione (nero)
      
      // Cancella il triangolo del marker
      display.drawBox(scaleX, scaleY, scaleWidth, 10);
      
      // Cancella le aree con valori - area più grande per evitare sovrapposizioni
      display.drawBox(5, scaleY + 15, 120, 30);
      
      display.setDrawColor(1); // Torna alla modalità normale (bianco)
      
      // Ridisegna il marker
      cogPosition = map(wingCogX, 0, 200, scaleX, scaleX + scaleWidth);
      cogPosition = constrain(cogPosition, scaleX, scaleX + scaleWidth);
      
      display.drawTriangle(
        cogPosition, scaleY + 6,
        cogPosition - 4, scaleY + 2,
        cogPosition + 4, scaleY + 2
      );
      display.drawBox(
        cogPosition - 2, scaleY + 2,
        5, 4
      );
      
      // Aggiorna testo
      display.setFont(FONT_SMALL);
      
      // Visualizza il peso totale
      display.setCursor(5, scaleY + 20);
      display.print(F("Weight: "));
      
      // Salviamo di nuovo la posizione X dopo "Weight: "
      valueXPos = display.getCursorX();
      
      // Stampiamo il valore del peso
      display.print(totalWeight, 2);
      
      // Sposta l'unità di misura "kg" a destra di 3 posizioni
      display.setCursor(display.getCursorX() + 3, scaleY + 20);
      display.print(F("kg"));
      
      // Visualizza il baricentro - con allineamento dei due punti
      display.setCursor(cgXpos, scaleY + 32);
      display.print(F("CG: "));
      
      // Ci assicuriamo che il valore del CG inizi alla stessa posizione X del peso
      display.setCursor(valueXPos, scaleY + 32);
      display.print(wingCogX);
      
      // Sposta l'unità di misura "mm" a destra di 3 posizioni
      display.setCursor(display.getCursorX() + 3, scaleY + 32);
      display.print(F("mm"));
      
      display.sendBuffer();
    }
    
    delay(20);
  }
}

/**
 * Funzione per la schermata Lateral Balance 
 */
void showLateralBalance() {
  display.clearBuffer();
  
  // Definisci i valori minimi e massimi come costanti
  const int16_t MIN_LATERAL_VALUE = -250;  // Valore minimo in mm
  const int16_t MAX_LATERAL_VALUE = 250;   // Valore massimo in mm
  const int16_t TICK_INTERVAL = 50;        // Intervallo tra i tick marks (tacche)
  
  // Intestazione centrata
  display.setFont(FONT_MEDIUM);
  const char* title = "Lateral Balance";
  int titleWidth = display.getStrWidth(title);
  int xPos = (128 - titleWidth) / 2;
  display.setCursor(xPos, 12);
  display.print(title);
  
  // Disegna la scala di bilanciamento laterale
  const uint8_t centerX = 64; // Centro dello schermo
  const uint8_t baseY = 28; // Alzata di 2 pixel
  const uint8_t scaleWidth = 110; // Stessa larghezza di CG View
  
  // Disegna la scala orizzontale - versione più sottile
  display.drawLine(centerX - scaleWidth/2, baseY, centerX + scaleWidth/2, baseY);
  
  // Aggiungi solo l'etichetta per lo zero, CENTRATA PRECISAMENTE
  display.setFont(FONT_SMALL2); // Font più piccolo per le etichette
  
  // Calcola larghezza testo "0" per centrarlo esattamente
  int zeroWidth = display.getStrWidth("0");
  display.setCursor(centerX - zeroWidth/2, baseY + 9); // Spostata più in basso
  display.print(F("0"));
  
  // Disegna i tick marks RIDOTTI usando i valori costanti
  for (int16_t i = MIN_LATERAL_VALUE; i <= MAX_LATERAL_VALUE; i += TICK_INTERVAL) {
    if (i != 0) { // Solo le tacche, niente etichette eccetto lo zero
      int16_t tickX = map(i, MIN_LATERAL_VALUE, MAX_LATERAL_VALUE, centerX - scaleWidth/2, centerX + scaleWidth/2);
      // Riduzione dei ticker: ora solo 2 pixel di altezza invece di 4
      display.drawLine(tickX, baseY - 1, tickX, baseY + 1);
    }
  }
  
  // Limita il valore lateralCogDiff all'intervallo consentito
  int16_t clampedLateralDiff = lateralCogDiff;
  if (clampedLateralDiff < MIN_LATERAL_VALUE) clampedLateralDiff = MIN_LATERAL_VALUE;
  if (clampedLateralDiff > MAX_LATERAL_VALUE) clampedLateralDiff = MAX_LATERAL_VALUE;
  
  // Mappa direttamente il valore senza invertirlo
  int16_t lateralIndicator = map(clampedLateralDiff, MIN_LATERAL_VALUE, MAX_LATERAL_VALUE, centerX + scaleWidth/2, centerX - scaleWidth/2);
  
  // Disegna un triangolo più piccolo per l'indicatore (ulteriormente ridotto)
  display.drawTriangle(
    lateralIndicator, baseY - 4, // Ridotto ulteriormente
    lateralIndicator - 2, baseY - 8, // Ridotto ulteriormente
    lateralIndicator + 2, baseY - 8
  );
  display.drawBox(
    lateralIndicator - 1, baseY - 8, // Ridotto ulteriormente
    3, 4  // Box più piccolo
  );
  
  // Cancella COMPLETAMENTE tutta la parte inferiore dello schermo
  display.setDrawColor(0);
  display.drawBox(0, baseY + 10, 128, 54); // Cancella fino alla fine dello schermo
  
  display.setDrawColor(1); // Torna alla modalità normale (bianco)
  
  // Testo "Side Balance" centrato - SPOSTATO PIÙ IN ALTO
  const char* sideText = "Side Balance";
  int sideTextWidth = display.getStrWidth(sideText);
  // Alzato da baseY + 20 a baseY + 18
  display.setCursor((128 - sideTextWidth) / 2, baseY + 18);
  display.print(sideText);
  
  // Posizionamento dei valori numerici ALZATI DI PARECCHIO
  display.setFont(FONT_SMALL);
  
  // MAGGIORE SEPARAZIONE TRA I VALORI
  // Definisci lo spazio di separazione (aumentato a 15 pixel da centro)
  const int LATERAL_OFFSET = 15;
  
  // ALZAMENTO IMPORTANTE: da baseY + 40 a baseY + 33 (7 pixel più in alto)
  if (clampedLateralDiff > 0) {
    // Valore positivo - posizionato a sinistra del centro con maggiore offset
    display.setCursor(centerX - LATERAL_OFFSET - 35, baseY + 33);
    display.print(F("+"));
    display.print(clampedLateralDiff);
    display.setCursor(display.getCursorX() + 3, baseY + 33);
    display.print(F("mm"));
  } else if (clampedLateralDiff < 0) {
    // Valore negativo - posizionato a destra del centro con maggiore offset
    display.setCursor(centerX + LATERAL_OFFSET, baseY + 33);
    display.print(clampedLateralDiff);
    display.setCursor(display.getCursorX() + 3, baseY + 33);
    display.print(F("mm"));
  } else {
    // Esattamente zero - centrato ma alzato
    display.setCursor(centerX - 10, baseY + 33);
    display.print(F("0 mm"));
  }
  
  // Aggiorna il display
  display.sendBuffer();
  
  // Attendi la pressione di K4 per tornare indietro
  bool waitForBack = true;
  unsigned long updateTimer = 0;
  
  while (waitForBack) {
    if (digitalRead(BUTTON_K4) == LOW) {
      waitForBack = false;
      delay(100);
    }
    
    // Aggiorna i dati in tempo reale
    unsigned long currentTime = millis();
    if (currentTime - updateTimer >= 500) {
      updateTimer = currentTime;
      updateWeightData();
      
      // AGGIORNAMENTO con pulizia più ampia
      
      // Aggiorna solo la parte dinamica del display
      display.setDrawColor(0); // Modalità cancellazione (nero)
      
      // PULIZIA PIÙ AMPIA: Cancella l'indicatore con area più ampia
      display.drawBox(centerX - scaleWidth/2 - 5, baseY - 10, scaleWidth + 10, 10);
      
      // PULIZIA COMPLETA dell'area dei valori fino alla fine dello schermo
      // IMPORTANTE: area alzata per corrispondere ai nuovi posizionamenti
      display.drawBox(0, baseY + 25, 128, 39); // Area spostata più in alto
      
      display.setDrawColor(1); // Torna alla modalità normale (bianco)
      
      // Prima limita il valore anche nell'aggiornamento
      clampedLateralDiff = lateralCogDiff;
      if (clampedLateralDiff < MIN_LATERAL_VALUE) clampedLateralDiff = MIN_LATERAL_VALUE;
      if (clampedLateralDiff > MAX_LATERAL_VALUE) clampedLateralDiff = MAX_LATERAL_VALUE;
      
      // Anche qui mappiamo direttamente senza invertire
      lateralIndicator = map(clampedLateralDiff, MIN_LATERAL_VALUE, MAX_LATERAL_VALUE, centerX + scaleWidth/2, centerX - scaleWidth/2);
      
      // Disegna triangolo ridotto
      display.drawTriangle(
        lateralIndicator, baseY - 4,
        lateralIndicator - 2, baseY - 8,
        lateralIndicator + 2, baseY - 8
      );
      display.drawBox(
        lateralIndicator - 1, baseY - 8,
        3, 4
      );
      
      // Ripeti il posizionamento con gli stessi valori alzati
      display.setFont(FONT_SMALL);
      
      // IMPORTANTE: Anche qui alzo i valori (baseY + 33 invece di baseY + 40)
      if (clampedLateralDiff > 0) {
        // Valore positivo - posizionato a sinistra del centro con maggiore offset
        display.setCursor(centerX - LATERAL_OFFSET - 35, baseY + 33);
        display.print(F("+"));
        display.print(clampedLateralDiff);
        display.setCursor(display.getCursorX() + 3, baseY + 33);
        display.print(F("mm"));
      } else if (clampedLateralDiff < 0) {
        // Valore negativo - posizionato a destra del centro con maggiore offset
        display.setCursor(centerX + LATERAL_OFFSET, baseY + 33);
        display.print(clampedLateralDiff);
        display.setCursor(display.getCursorX() + 3, baseY + 33);
        display.print(F("mm"));
      } else {
        // Esattamente zero - centrato ma alzato
        display.setCursor(centerX - 10, baseY + 33);
        display.print(F("0 mm"));
      }
      
      display.sendBuffer();
    }
    
    delay(20);
  }
}

/**
 * Funzione per mostrare il menu principale
 */
void showMainMenu() {
  menuActive = true;
  
  bool exitMenu = false;
  menuPosition = 0;
  
  // Assicurati che tutti i pulsanti siano rilasciati prima di entrare nel menu
  while (digitalRead(BUTTON_K1) == LOW ||
         digitalRead(BUTTON_K2) == LOW ||
         digitalRead(BUTTON_K3) == LOW ||
         digitalRead(BUTTON_K4) == LOW) {
    delay(50);
  }
  delay(200); // Ulteriore attesa per stabilizzazione
  
  while (!exitMenu) {
    display.clearBuffer();
    
    // Intestazione del menu - centrata
    display.setFont(FONT_LARGE);
    // Calcola la larghezza del testo per centrare
    const char* title = "MAIN MENU";
    int titleWidth = display.getStrWidth(title);
    int xPos = (128 - titleWidth) / 2; // Centra orizzontalmente
    
    display.setCursor(xPos, 15);
    display.print(title);
    
    // Disegna le voci di menu con font più piccolo
    display.setFont(FONT_SMALL); // Uso del font più piccolo per il menu
    
    for (uint8_t i = 0; i < MAIN_MENU_ITEMS; i++) {
      // Calcola la posizione Y di ogni voce - con maggiore spaziatura
      uint8_t yPos = 30 + i * 12; // Aumentata la spaziatura a 12 pixel (era 10)
      
      // Disegna il pulsante con dimensioni ridotte
      if (i == menuPosition) {
        // Pulsante selezionato (invertito)
        display.setDrawColor(1);
        display.drawBox(14, yPos - 8, 100, 10); // Rettangolo più stretto e più basso
        display.setDrawColor(0); // Modalità inversa per il testo
      } else {
        // Pulsante non selezionato
        display.setDrawColor(1);
        display.drawFrame(14, yPos - 8, 100, 10); // Rettangolo più stretto e più basso
      }
      
      // Centra il testo nel pulsante
      uint8_t textWidth = display.getStrWidth(mainMenuItems[i]);
      uint8_t textXPos = (128 - textWidth) / 2;
      display.setCursor(textXPos, yPos);
      display.print(mainMenuItems[i]);
      
      // Torna alla modalità normale di disegno
      display.setDrawColor(1);
    }
    
    // Aggiorna il display
    display.sendBuffer();
    
    // Attendi che un pulsante venga premuto e rilasciato
    bool k1Pressed = false;
    bool k2Pressed = false;
    bool k3Pressed = false;
    bool k4Pressed = false;
    
    // Controlla lo stato iniziale dei pulsanti
    delay(200);  // Attesa per stabilizzazione dopo l'aggiornamento del display
    
    // Loop di attesa del pulsante (esce quando un pulsante è premuto)
    while (!k1Pressed && !k2Pressed && !k3Pressed && !k4Pressed) {
      // Leggi lo stato attuale dei pulsanti
      k1Pressed = (digitalRead(BUTTON_K1) == LOW);
      k2Pressed = (digitalRead(BUTTON_K2) == LOW);
      k3Pressed = (digitalRead(BUTTON_K3) == LOW);
      k4Pressed = (digitalRead(BUTTON_K4) == LOW);
      
      delay(20); // Piccolo ritardo
    }
    
    // Elabora il pulsante premuto
    if (k1Pressed) {
      // Azione in base alla selezione
      switch (menuPosition) {
        case 0: // Scale Tare
          tareScales();
          break;
        case 1: // CG View
          showCGView();
          break;
        case 2: // Lateral Balance
          showLateralBalance();
          break;
      }
    }
    else if (k2Pressed) {
      // Muovi la selezione in su (con wrap-around)
      menuPosition = (menuPosition > 0) ? menuPosition - 1 : MAIN_MENU_ITEMS - 1;
    }
    else if (k3Pressed) {
      // Muovi la selezione in giù (con wrap-around)
      menuPosition = (menuPosition + 1) % MAIN_MENU_ITEMS;
    }
    else if (k4Pressed) {
      // Esci dal menu
      exitMenu = true;
    }
    
    // Attendi il rilascio di tutti i pulsanti prima di continuare
    while (digitalRead(BUTTON_K1) == LOW ||
           digitalRead(BUTTON_K2) == LOW ||
           digitalRead(BUTTON_K3) == LOW ||
           digitalRead(BUTTON_K4) == LOW) {
      delay(50);
    }
    delay(200); // Ulteriore attesa per stabilizzazione
  }
  
  // Quando si esce dal menu, torna alla modalità normale
  menuActive = false;
}

//================ FUNZIONI PER LA CALIBRAZIONE E IL CONTROLLO ================//
/**
 * Funzione per azzerare tutte le celle
 */
void tareScales() {
  display.clearBuffer();
  
  // Intestazione
  display.setFont(FONT_LARGE);
  display.setCursor(40, 12);
  display.print(F("TARE"));

  // Disegna indicatore grafico di progresso
  display.drawFrame(10, 20, 108, 20);
  display.setFont(FONT_SMALL);
  display.setCursor(15, 50);
  display.print(F("Tare in progress..."));
  display.sendBuffer();
  
  // Tara ogni cella mostrando il progresso
  const uint8_t barWidth = 25; // Larghezza per ogni segmento della barra
  
  // Tara cella 1
  display.drawBox(12, 22, barWidth - 2, 16);
  display.sendBuffer();
  scale1.tare();
  delay(200);
  
  // Tara cella 2
  display.drawBox(12 + barWidth, 22, barWidth - 2, 16);
  display.sendBuffer();
  scale2.tare();
  delay(200);
  
  // Tara cella 3
  display.drawBox(12 + 2 * barWidth, 22, barWidth - 2, 16);
  display.sendBuffer();
  scale3.tare();
  delay(200);
  
  // Tara cella 4
  display.drawBox(12 + 3 * barWidth, 22, barWidth - 2, 16);
  display.sendBuffer();
  scale4.tare();
  delay(200);
  
  // Azzera i campioni per lo smoothing
  for (uint8_t j = 0; j < SMOOTHING_SAMPLES; j++) {
    weight1Samples[j] = 0;
    weight2Samples[j] = 0;
    weight3Samples[j] = 0;
    weight4Samples[j] = 0;
  }
  
  taraEffettuata = true;
  
  // Messaggio di conferma con scritta centrata
  display.clearBuffer();
  
  // Calcola larghezza della stringa "TARE DONE!" per centrarla
  display.setFont(FONT_LARGE);
  const char* tareMessage = "TARE DONE!";
  int tareMessageWidth = display.getStrWidth(tareMessage);
  int centerX = (128 - tareMessageWidth) / 2;
  
  // Posiziona la scritta principale esattamente al centro
  display.setCursor(centerX, 30);
  display.print(tareMessage);
  
  // Aggiungi scritta più piccola sotto
  display.setFont(FONT_SMALL);
  const char* menuMessage = "Press K4 for Main Menu";
  int menuMessageWidth = display.getStrWidth(menuMessage);
  centerX = (128 - menuMessageWidth) / 2;
  
  display.setCursor(centerX, 50);
  display.print(menuMessage);
  
  display.sendBuffer();
  
  // Attendi la pressione del pulsante K4 per andare al menu principale
  bool waitForK4 = true;
  
  while (waitForK4) {
    if (digitalRead(BUTTON_K4) == LOW) {
      waitForK4 = false;
      delay(100); // Debounce
      
      // Mostra il menu principale
      menuActive = true;
      menuPosition = 0;
      showMainMenu();
    }
    delay(20); // Piccolo ritardo per evitare di saturare il CPU
  }
}

/**
 * Funzione per la calibrazione via seriale
 */
void calibrateCellsViaSerial() {
  bool prevMode = continuousMode;
  continuousMode = false;
  
  // Visualizzazione semplice sul display per informare l'utente
  display.clearBuffer();
  display.setFont(FONT_LARGE);
  display.setCursor(0, 12);
  display.print(F("CALIBRATION"));
  
  display.setFont(FONT_SMALL);
  display.setCursor(0, 25);
  display.print(F("Follow instructions"));
  display.setCursor(0, 35);
  display.print(F("on serial port"));
  display.setCursor(0, 45);
  display.print(F("Do not touch device"));
  display.setCursor(0, 55);
  display.print(F("during calibration"));
  display.sendBuffer();
  
  Serial.println(F("=== CALIBRATION PROCEDURE ==="));
  Serial.println(F("1. Remove all weight from platform"));
  Serial.println(F("2. Send 't' to tare cells"));
  
  // Attende il comando di tara dall'utente
  while (true) {
    if (Serial.available() > 0) {
      char cmd = Serial.read();
      if (cmd == 't') {
        // Esegue la tara su tutte le celle
        Serial.println(F("Taring cells..."));
        scale1.tare();
        scale2.tare();
        scale3.tare();
        scale4.tare();
        Serial.println(F("Tare completed."));
        break;
      }
    }
    delay(10);
  }
  
  Serial.println(F("3. Place a known weight (e.g. 500g) at the center of the platform"));
  Serial.println(F("4. Enter weight in kg (e.g. 0.500): "));
  
  // Aggiorna display per il passo successivo
  display.clearBuffer();
  display.setFont(FONT_LARGE);
  display.setCursor(0, 12);
  display.print(F("CALIBRATION"));
  
  display.setFont(FONT_SMALL);
  display.setCursor(0, 25);
  display.print(F("Place a known weight"));
  display.setCursor(0, 35);
  display.print(F("in the center"));
  display.setCursor(0, 45);
  display.print(F("Enter value on"));
  display.setCursor(0, 55);
  display.print(F("serial port"));
  display.sendBuffer();
  
  // Attende l'input del peso noto
  float knownWeight = 0;
  while (true) {
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      knownWeight = input.toFloat();
      if (knownWeight > 0) {
        Serial.print(F("Weight entered: "));
        Serial.print(knownWeight, 3);
        Serial.println(F(" kg"));
        break;
      } else {
        Serial.println(F("Invalid weight. Try again: "));
      }
    }
    delay(10);
  }
  
  // Mostra animazione di calibrazione
  display.clearBuffer();
  display.setFont(FONT_MEDIUM);
  display.setCursor(0, 15);
  display.print(F("Calibration"));
  display.setCursor(0, 30);
  display.print(F("in progress..."));
  
  // Barra di progresso animata
  display.drawFrame(0, 35, 128, 12);
  display.sendBuffer();
  
  // Leggi i valori grezzi da tutte le celle
  Serial.println(F("Reading raw values from cells..."));
  
  // Aggiorna la barra di progresso per la prima lettura
  display.drawBox(2, 37, 30, 8);
  display.sendBuffer();
  
  float raw1 = scale1.get_value(10);
  Serial.print(F("Cell 1 raw value: "));
  Serial.println(raw1);
  
  // Aggiorna la barra di progresso per la seconda lettura
  display.drawBox(2, 37, 60, 8);
  display.sendBuffer();
  
  float raw2 = scale2.get_value(10);
  Serial.print(F("Cell 2 raw value: "));
  Serial.println(raw2);
  
  // Aggiorna la barra di progresso per la terza lettura
  display.drawBox(2, 37, 90, 8);
  display.sendBuffer();
  
  float raw3 = scale3.get_value(10);
  Serial.print(F("Cell 3 raw value: "));
  Serial.println(raw3);
  
  // Aggiorna la barra di progresso per la quarta lettura
  display.drawBox(2, 37, 120, 8);
  display.sendBuffer();
  
  float raw4 = scale4.get_value(10);
  Serial.print(F("Cell 4 raw value: "));
  Serial.println(raw4);
  
  // Calcola i fattori di calibrazione
  Serial.println(F("Calculating calibration factors..."));
    
  // Somma dei valori grezzi
  float rawSum = raw1 + raw2 + raw3 + raw4;
    
  calibration_factor1 = (raw1 / rawSum) * knownWeight * 4.0;
  calibration_factor2 = (raw2 / rawSum) * knownWeight * 4.0;
  calibration_factor3 = (raw3 / rawSum) * knownWeight * 4.0;
  calibration_factor4 = (raw4 / rawSum) * knownWeight * 4.0;
  
  // Imposta i nuovi fattori di calibrazione
  scale1.set_scale(calibration_factor1);
  scale2.set_scale(calibration_factor2);
  scale3.set_scale(calibration_factor3);
  scale4.set_scale(calibration_factor4);
  
  // Mostra i nuovi fattori di calibrazione
  Serial.println(F("New calibration factors:"));
  Serial.print(F("Cell 1: "));
  Serial.println(calibration_factor1);
  Serial.print(F("Cell 2: "));
  Serial.println(calibration_factor2);
  Serial.print(F("Cell 3: "));
  Serial.println(calibration_factor3);
  Serial.print(F("Cell 4: "));
  Serial.println(calibration_factor4);
  
  // Test della calibrazione
  Serial.println(F("Testing calibration with reference weight..."));
  Serial.println(F("Measured weights after calibration:"));
  delay(500); // Breve pausa per stabilizzare le celle
  
  float measuredWeight1 = scale1.get_units(5);
  float measuredWeight2 = scale2.get_units(5);
  float measuredWeight3 = scale3.get_units(5);
  float measuredWeight4 = scale4.get_units(5);
  
  Serial.print(F("Cell 1: "));
  Serial.print(measuredWeight1, 3);
  Serial.println(F(" kg"));
  
  Serial.print(F("Cell 2: "));
  Serial.print(measuredWeight2, 3);
  Serial.println(F(" kg"));
  
  Serial.print(F("Cell 3: "));
  Serial.print(measuredWeight3, 3);
  Serial.println(F(" kg"));
  
  Serial.print(F("Cell 4: "));
  Serial.print(measuredWeight4, 3);
  Serial.println(F(" kg"));
  
  // Ripristina la modalità precedente
  continuousMode = prevMode;
  
  // Aggiorna display
  display.clearBuffer();
  display.setFont(FONT_LARGE);
  display.setCursor(0, 12);
  display.print(F("CALIBRATION DONE"));
  
  display.setFont(FONT_SMALL);
  display.setCursor(0, 25);
  display.print(F("Calibration values"));
  display.setCursor(0, 35);
  display.print(F("saved."));
  display.setCursor(0, 50);
  display.print(F("Return to normal"));
  display.setCursor(0, 60);
  display.print(F("operation..."));
  display.sendBuffer();
  
  delay(1500);
}

/**
 * Funzione per attivare/disattivare la modalità continua
 */
void toggleContinuousMode() {
  continuousMode = !continuousMode;
  
  display.clearBuffer();
  
  // Feedback grafico dello stato
  display.setFont(FONT_MEDIUM);
  display.setCursor(5, 12);
  display.println(continuousMode ? F("CONTINUOUS MODE") : F("PAUSED MODE"));
  
  // Disegna uno switch grafico
  display.drawRFrame(20, 20, 88, 30, 4);
  
  if (continuousMode) {
    // Switch attivato (destra)
    display.drawBox(64, 22, 42, 26);
    display.setDrawColor(0); // Inverti per il testo "ON"
    display.setCursor(72, 40);
    display.print(F("ON"));
    display.setDrawColor(1); // Torna al normale
    display.setCursor(36, 40);
    display.print(F("OFF"));
  } else {
    // Switch disattivato (sinistra)
    display.drawBox(22, 22, 42, 26);
    display.setDrawColor(0); // Inverti per il testo "OFF"
    display.setCursor(36, 40);
    display.print(F("OFF"));
    display.setDrawColor(1); // Torna al normale
    display.setCursor(72, 40);
    display.print(F("ON"));
  }
  
  display.sendBuffer();
  delay(500);
  
  Serial.println(continuousMode ? F("Continuous reading on") : F("Reading paused"));
}

/**
 * Funzione per visualizzare i fattori di calibrazione
 */
void showCalibrationFactors() {
  // Output dei fattori di calibrazione sulla seriale (completo)
  Serial.println(F("===== CURRENT CALIBRATION FACTORS ====="));
  Serial.print(F("Cell 1: "));
  Serial.println(calibration_factor1, 6);
  Serial.print(F("Cell 2: "));
  Serial.println(calibration_factor2, 6);
  Serial.print(F("Cell 3: "));
  Serial.println(calibration_factor3, 6);
  Serial.print(F("Cell 4: "));
  Serial.println(calibration_factor4, 6);
  Serial.println(F("======================================="));
  
  // Output su display (semplificato)
  display.clearBuffer();
  
  display.setFont(FONT_LARGE);
  display.setCursor(5, 12);
  display.print(F("CALIBRATION"));
  
  display.setFont(FONT_SMALL);
  
  // Messaggi di posizione delle celle
  display.setCursor(5, 25);
  display.println(F("Cell positions:"));
  display.setCursor(5, 38);
  display.println(F("LF=1  RF=2"));
  display.setCursor(5, 50);
  display.println(F("LR=3  RR=4"));
  
  display.setCursor(5, 62);
  display.println(F("See serial port for details"));
  
  display.sendBuffer();
  delay(2000);
}

/**
 * Funzione per impostare un nuovo fattore di correzione
 */
void setCorrectionFactor(float newFactor) {
  // Modifica la costante CORRECTION_FACTOR in updateWeightData
  // Questo verrà fatto manualmente nel codice dopo aver trovato il valore corretto
  
  Serial.print(F("New correction factor set: "));
  Serial.println(newFactor, 4);
  Serial.println(F("Copy this value and update the CORRECTION_FACTOR in updateWeightData()"));
  Serial.println(F("Then recompile and upload the code"));
}

//================ FUNZIONI DI INPUT E OUTPUT ================//
/**
 * Funzione per controllare e gestire i comandi dalla porta seriale
 */
void checkSerialCommands() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Rimuove spazi e caratteri di controllo
    
    // Gestione dei comandi base
    if (input.length() == 1) {
      char cmd = input.charAt(0);
      
      switch (cmd) {
        case 'z': // Azzeramento/Tara
          Serial.println(F("Starting tare process..."));
          tareScales();
          Serial.println(F("Tare completed"));
          break;
          
        case 'p': // Pausa/Riprendi
          toggleContinuousMode();
          break;
          
        case 's': // Mostra calibrazione
          showCalibrationFactors();
          break;
          
        case 'c': // Calibrazione
          Serial.println(F("Starting calibration process..."));
          calibrateCellsViaSerial();
          Serial.println(F("Calibration completed and ready for use"));
          break;

        case 'm': // Mostra menu principale
          Serial.println(F("Showing main menu via serial command"));
          menuActive = true;
          menuPosition = 0;
          showMainMenu();
          break;
          
        case 'd': // Modalità di debug
          Serial.println(F("Debug mode - Raw values:"));
          Serial.print(F("Cell 1 raw: "));
          Serial.println(scale1.read());
          Serial.print(F("Cell 2 raw: "));
          Serial.println(scale2.read());
          Serial.print(F("Cell 3 raw: "));
          Serial.println(scale3.read());
          Serial.print(F("Cell 4 raw: "));
          Serial.println(scale4.read());
          
          // Debug dei pulsanti
          Serial.println(F("Button states:"));
          Serial.print(F("K1 (pin "));
          Serial.print(BUTTON_K1);
          Serial.print(F("): "));
          Serial.println(digitalRead(BUTTON_K1));
          Serial.print(F("K2 (pin "));
          Serial.print(BUTTON_K2);
          Serial.print(F("): "));
          Serial.println(digitalRead(BUTTON_K2));
          Serial.print(F("K3 (pin "));
          Serial.print(BUTTON_K3);
          Serial.print(F("): "));
          Serial.println(digitalRead(BUTTON_K3));
          Serial.print(F("K4 (pin "));
          Serial.print(BUTTON_K4);
          Serial.print(F("): "));
          Serial.println(digitalRead(BUTTON_K4));
          break;
          
        default:
          Serial.println(F("Command not recognized"));
          break;
      }
    }
    // Comando per impostare il fattore di correzione (formato: cf,valore)
    else if (input.length() > 3 && input.startsWith("cf,")) {
      int commaPos = input.indexOf(',');
      String valueStr = input.substring(commaPos + 1);
      float value = valueStr.toFloat();
      
      if (value > 0) {
        setCorrectionFactor(value);
      } else {
        Serial.println(F("Invalid correction factor (must be > 0)"));
      }
    }
    // Gestione dei comandi di calibrazione individuale (formato: c1,valore)
    else if (input.length() > 2 && input.charAt(0) == 'c' && input.indexOf(',') > 0) {
      int cellNumber = input.charAt(1) - '0'; // Converte il carattere in numero
      
      if (cellNumber >= 1 && cellNumber <= 4) {
        int commaPos = input.indexOf(',');
        String valueStr = input.substring(commaPos + 1);
        float value = valueStr.toFloat();
        
        if (value != 0) { // Controllo base per verificare la validità del valore
          // Imposta il fattore di calibrazione per la cella specificata
          switch (cellNumber) {
            case 1:
              calibration_factor1 = value;
              scale1.set_scale(calibration_factor1);
              break;
            case 2:
              calibration_factor2 = value;
              scale2.set_scale(calibration_factor2);
              break;
            case 3:
              calibration_factor3 = value;
              scale3.set_scale(calibration_factor3);
              break;
            case 4:
              calibration_factor4 = value;
              scale4.set_scale(calibration_factor4);
              break;
          }
          
          Serial.print(F("Calibration factor for cell "));
          Serial.print(cellNumber);
          Serial.print(F(" set to: "));
          Serial.println(value);
        } else {
          Serial.println(F("Invalid calibration value"));
        }
      } else {
        Serial.println(F("Invalid cell number (must be 1-4)"));
      }
    }
  }
}

/**
 * Funzione per stampare i dati sulla seriale
 */
void printSerialData() {
  // Intestazione del report
  Serial.println(F("===== MEASUREMENT REPORT ====="));
  
  // Dati sul peso
  Serial.print(F("Total weight: "));
  Serial.print(totalWeight, 3); // Maggiore precisione per debug (3 decimali)
  Serial.println(F(" kg"));
  
  // Dati sulle celle individuali
  Serial.print(F("Cells [LF, RF, LR, RR]: ["));
  Serial.print(scale1.get_units(), 3);
  Serial.print(F(", "));
  Serial.print(scale2.get_units(), 3);
  Serial.print(F(", "));
  Serial.print(scale3.get_units(), 3);
  Serial.print(F(", "));
  Serial.print(scale4.get_units(), 3);
  Serial.println(F("]"));
  
  // Dati sul baricentro
  Serial.print(F("CG X: "));
  Serial.print(cogX);
  Serial.println(F(" mm from left"));
  
  Serial.print(F("CG Y: "));
  Serial.print(cogY);
  Serial.println(F(" mm from front"));
  
  Serial.print(F("Wing CG: "));
  Serial.print(wingCogX);
  Serial.println(F(" mm"));
  
  Serial.print(F("CG %: "));
  Serial.print(cogPercentage);
  Serial.println(F("% of wing peg distance"));
  
  Serial.print(F("Lateral balance: "));
  if (lateralCogDiff > 0) Serial.print(F("+"));
  Serial.print(lateralCogDiff);
  Serial.println(F(" mm from center"));
  
  Serial.println(F("============================="));
}

/**
 * Funzione per gestire i pulsanti
 */
void checkButtons() {
  // Legge lo stato corrente dei pulsanti (solo una volta)
  bool k1Pressed = digitalRead(BUTTON_K1) == LOW;
  bool k2Pressed = digitalRead(BUTTON_K2) == LOW;
  bool k3Pressed = digitalRead(BUTTON_K3) == LOW;
  bool k4Pressed = digitalRead(BUTTON_K4) == LOW;
  
  // Controllo debounce
  if ((k1Pressed || k2Pressed || k3Pressed || k4Pressed) && (millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    lastDebounceTime = millis();
    
    if (k1Pressed) {
      // Pulsante Menu - Sempre attiva il menu principale
      menuActive = true;
      menuPosition = 0;
      showMainMenu();
    }
    
    if (k4Pressed && !menuActive) {
      // K4 come tara rapida quando non in menu
      tareScales();
    }
    
    if (k4Pressed && menuActive) {
      // Pulsante Indietro - Esce dal menu attivo
      menuActive = false;
    }
  }
}