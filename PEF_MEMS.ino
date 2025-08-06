/**
 * =============================================================================
 * @file        PFE_MEMS.ino
 * @brief       Système de mesure de résistance de contact 
 *				et activation pour un relais MEMS
 * @author      Briac Samson--Tessier / Frixos Papacostidis / Hamza Moubtahji / Abdelhalim Ouikar / Ilias Ben Moussa
 * @version     1.0
 * @date        Août 2025
 * 
 * @description Système d'acquisition pour la mesure de résistance
 * 				de contact avec détection automatique et stabilisation.
 *				Utilise FreeRTOS pour le multi-threading
 *              et implémente un buffer circulaire pour l'acquisition.
 * 
 * @hardware    - ESP32 DevKit C4
 *              - Capteur de courant INA219
 *              - ADC 16-bit ADS1115
 *              - Écran LCD I2C 16x4
 *              - Carte SD SPI
 *              - Pont de Wheatstone avec résistances 10MΩ
 *				- AOP TLC084 et pour isoler les mesures
 *				- Transistor D44H11G NPN et AOP TLC081CP pour étage de puissance
 * 
 * @features    - Acquisition temps réel
 *              - Double thread (mesure/affichage)
 *              - Sauvegarde CSV
 *              - Détection automatique avec stabilisation
 *              - Protection contre les surintensités
 *              - Interface LCD anti-flickering
 * 
 * @note        - certaine constantes peuvent être mesuré préalablement pour optimiser les mesures
 * =============================================================================
 */

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <SD.h>

// =============================================================================
// CONFIGURATION HARDWARE
// =============================================================================

/** @defgroup GPIO Configuration des broches */
/**@{*/
const int SD_CS_PIN = 5;                          ///< Chip Select carte SD
const int DAC_RELAIS = 25;                        ///< Sortie DAC pour contrôle relais
const int BOUTON_BOOT = 0;                        ///< Bouton BOOT pour contrôle utilisateur
/**@}*/

// =============================================================================
// PARAMÈTRES DE SÉCURITÉ
// =============================================================================

/** @defgroup SAFETY Seuils de sécurité */
/**@{*/
const float SEUIL_COURANT_MAX_mA = 70.0;          ///< Courant maximum autorisé (mA)										 
/**@}*/

// =============================================================================
// CONFIGURATION PONT DE WHEATSTONE
// =============================================================================

/** @defgroup BRIDGE Configuration du pont de mesure (Optimisable par mesure préalable) */
/**@{*/
const float R1 = 10000000.0;                      ///< Résistance R1 du pont
const float R2 = 10000000.0;                      ///< Résistance R2 du pont
const float R3 = 10000000.0;                      ///< Résistance R3 du pont
const float Vexc = 3.313;                         ///< Tension d'excitation du pont(V)
/**@}*/

// =============================================================================
// PARAMÈTRES TEMPORELS ET ACQUISITION
// =============================================================================

/** @defgroup TIMING Configuration temporelle */
/**@{*/
const uint32_t INTERVALLE_AFFICHAGE_MS = 200;     ///< Période rafraîchissement LCD (ms)
const uint32_t INTERVALLE_SAUVEGARDE_MS = 10;     ///< Période écriture SD (ms)
const uint32_t TAILLE_BUFFER = 500;               ///< Taille buffer circulaire
const int NB_TESTS_TOTAL = 10;                    ///< Nombre de tests par cycle
/**@}*/

/** @defgroup DETECTION Paramètres de détection */
/**@{*/
const float SEUIL_RCONTACT_MIN = 100.0;          ///< Résistance minimum pour détection
const float SEUIL_RESISTANCE_MAX = 500000000.0;   ///< Résistance maximum mesurable
const uint32_t DUREE_MAX_TEST_MS = 300000;        ///< Timeout test (5 minutes)
/**@}*/

// =============================================================================
// STRUCTURES DE DONNÉES
// =============================================================================

/**
 * @struct MesureBrute
 * @brief  Structure de données pour une mesure brute
 */
struct MesureBrute {
  uint32_t temps_us;        ///< Timestamp en microsecondes
  float courant_mA;         ///< Courant mesuré (mA)
  int16_t raw_diff;         ///< Mesure différentielle ADC (pont de wheastone)
  int16_t raw_a2;           ///< Mesure single-ended A2 (activation)
  uint8_t dac;              ///< Valeur DAC courante
} __attribute__((packed));

/**
 * @struct BufferCirculaire
 * @brief  Buffer circulaire thread-safe pour les mesures
 */
struct BufferCirculaire {
  MesureBrute mesures[TAILLE_BUFFER];  ///< Tableau des mesures
  volatile uint32_t tete;               ///< Index d'écriture
  volatile uint32_t queue;              ///< Index de lecture
  volatile uint32_t count;              ///< Nombre d'éléments
};

/**
 * @struct StatsTest
 * @brief  stabilisation de la resistance pour avoire une valeur plus précise
 */
struct StatsTest {
  float r_courante;                     ///< Résistance courante
  bool contact_detecte;                 ///< Flag détection contact
  uint32_t temps_premier_contact_us;    ///< Timestamp premier contact
  float r_contact_initiale;             ///< Résistance de contact
  uint8_t dac_contact;                  ///< DAC lors du contact
};

/**
 * @struct EtatAffichage
 * @brief  État de l'affichage LCD pour anti-flickering
 */
struct EtatAffichage {
  String ligne1, ligne2, ligne3, ligne4;  ///< Contenu des lignes
  bool change;                             ///< Flag de changement
};

// =============================================================================
// FONCTIONS UTILITAIRES
// =============================================================================

/**
 * @brief  Valide une mesure de résistance
 * @param  r Résistance à valider
 * @return true si la résistance est valide
 */
inline bool resistanceValide(float r) {
  return (isfinite(r) && !isnan(r) && !isinf(r) && 
          r > 0 && r < SEUIL_RESISTANCE_MAX && r != -999999.0f);
}

/**
 * @brief  Calcule la résistance depuis la mesure différentielle
 * @param  rawDiff Valeur ADC différentielle
 * @return Résistance calculée ou -999999.0f si invalide
 */
inline float calculerResistance(int16_t rawDiff) {
  float Vd = rawDiff * 0.000125f;  // Conversion ADC vers volts
  float denom = Vexc - 2.0f * Vd;
  
  // Protection division par zéro
  if (fabsf(denom) < 1e-6f) return -999999.0f;
  
  float resistance = R1 * (Vexc + 2.0f * Vd) / denom;
  
  // Validation de la mesure
  if (!resistanceValide(resistance)) {
    return -999999.0f;
  }
  
  return resistance;
}

/**
 * @brief  Formate une valeur pour export CSV
 * @param  valeur Valeur à formater
 * @param  decimales Nombre de décimales
 * @return String formatée ou "N/A" si invalide
 */
inline String formaterValeurCSV(float valeur, int decimales = 3) {
  if (!isfinite(valeur) || isnan(valeur) || isinf(valeur) || 
      valeur < -999990.0f || valeur > 1e12f) {
    return "N/A";
  }
  return String(valeur, decimales);
}

// =============================================================================
// OBJETS GLOBAUX - CAPTEURS
// =============================================================================

Adafruit_INA219 ina219;                   ///< Capteur de courant
Adafruit_ADS1115 ads;                     ///< ADC 16-bit
LiquidCrystal_I2C lcd(0x27, 16, 4);       ///< Écran LCD I2C

// =============================================================================
// STRUCTURES DE DONNÉES PARTAGÉES
// =============================================================================

BufferCirculaire buffer;                  ///< Buffer circulaire principal
StatsTest stats;                          ///< stabilisation resistance du test
EtatAffichage affichage;                  ///< État affichage LCD

// =============================================================================
// SYNCHRONISATION MULTI-THREAD
// =============================================================================

/** @defgroup MUTEX Mutex pour synchronisation */
/**@{*/
SemaphoreHandle_t mutexBuffer;            ///< Protection buffer circulaire
SemaphoreHandle_t mutexStats;             ///< Protection statistiques
SemaphoreHandle_t mutexAffichage;         ///< Protection affichage
/**@}*/

/** @defgroup TASKS Handles des tâches FreeRTOS */
/**@{*/
TaskHandle_t tacheMesureHandle = NULL;    ///< Handle thread mesure
TaskHandle_t tacheAffichageHandle = NULL; ///< Handle thread affichage
/**@}*/

// =============================================================================
// VARIABLES DE CONTRÔLE GLOBALES
// =============================================================================

/** @defgroup CONTROL Variables de contrôle */
/**@{*/
volatile bool mesureActive = false;       ///< État acquisition active
volatile uint8_t dacCourant = 0;          ///< Valeur DAC courante
volatile uint32_t tempsDebutMicros = 0;   ///< Timestamp début test
volatile bool arretUrgence = false;       ///< Flag arrêt d'urgence
volatile bool contactDetecte = false;     ///< Flag contact détecté
volatile bool reinitialiserDemande = false; ///< Demande réinitialisation
/**@}*/

// =============================================================================
// GESTION FICHIERS
// =============================================================================

File dataFile;                            ///< Handle fichier données
String nomFichier = "";                   ///< Nom du fichier CSV
int numeroTest = 1;                       ///< Numéro test courant

// =============================================================================
// INTERFACE UTILISATEUR
// =============================================================================

volatile bool boutonAppuye = false;       ///< État bouton BOOT
unsigned long dernierAppui = 0;           ///< Anti-rebond bouton

// =============================================================================
// COMPTEURS DE PERFORMANCE
// =============================================================================

volatile uint32_t nbMesuresTotal = 0;     ///< Compteur mesures totales
volatile uint32_t nbMesuresSauvees = 0;   ///< Compteur mesures sauvegardées

// =============================================================================
// FONCTIONS DE CONTRÔLE
// =============================================================================

/**
 * @brief  Réinitialise toutes les variables de test de manière thread-safe
 * @note   Utilise les mutex pour éviter les conditions de course
 */
void reinitialiserVariablesTest() {
  // Protection des structures partagées
  if (xSemaphoreTake(mutexStats, pdMS_TO_TICKS(100)) == pdTRUE) {
    memset(&stats, 0, sizeof(stats));
    xSemaphoreGive(mutexStats);
  }
  
  if (xSemaphoreTake(mutexBuffer, pdMS_TO_TICKS(100)) == pdTRUE) {
    buffer.tete = 0;
    buffer.queue = 0;
    buffer.count = 0;
    xSemaphoreGive(mutexBuffer);
  }
  
  // Réinitialisation atomique des volatiles
  mesureActive = false;
  arretUrgence = false;
  contactDetecte = false;
  reinitialiserDemande = false;
  dacCourant = 0;
  nbMesuresTotal = 0;
  nbMesuresSauvees = 0;
  
  Serial.println(">>> Variables reinitialisees");
}

// =============================================================================
// THREAD 1 : ACQUISITION TENSION/COURANT
// =============================================================================

/**
 * @brief  Thread d'acquisition
 * @param  param Paramètre non utilisé (FreeRTOS)
 * @note   Exécuté sur Core 0 avec priorité maximale
 */
void tacheMesureUltraRapide(void *param) {
  uint32_t maintenant;
  MesureBrute mesureLocale;
  uint8_t dacLocal;
  bool contactDetecteLocal = false;
  
  Serial.println("Thread mesure demarre");
  
  while (mesureActive && !reinitialiserDemande) {
    maintenant = micros();
    dacLocal = dacCourant;
    
    // --- Acquisition synchrone des capteurs ---
    mesureLocale.temps_us = maintenant - tempsDebutMicros;
    mesureLocale.dac = dacLocal;
    mesureLocale.raw_diff = ads.readADC_Differential_0_1();
    mesureLocale.raw_a2 = ads.readADC_SingleEnded(2);
    mesureLocale.courant_mA = ina219.getCurrent_mA();
    
    // --- Calcul et validation ---
    float r_contact = calculerResistance(mesureLocale.raw_diff);
    
    // --- Protection surintensité ---
    if (mesureLocale.courant_mA > SEUIL_COURANT_MAX_mA) {
      arretUrgence = true;
      mesureActive = false;
      Serial.println(">>> SECURITE: Courant excessif");
      break;
    }
    
    // --- Détection et stabilisation du contact ---
    if (!contactDetecteLocal && xSemaphoreTake(mutexStats, 0) == pdTRUE) {
      stats.r_courante = r_contact;
      
      // Vérification conditions de détection
      if (resistanceValide(r_contact) && r_contact >= SEUIL_RCONTACT_MIN && !stats.contact_detecte) {
        
        Serial.print(">>> Contact potentiel DAC=");
        Serial.print(dacLocal);
        Serial.print(" R=");
        Serial.print(r_contact);
        Serial.println(" ohms");
        
        // --- Phase de stabilisation ---
        Serial.println(">>> Stabilisation...");
        
        const int NB_MESURES_STABILISATION = 5;
        float resistances[NB_MESURES_STABILISATION];
        bool stabilisationOK = true;
        
        // Acquisition des mesures de stabilisation
        for (int i = 0; i < NB_MESURES_STABILISATION; i++) {
          int16_t rawStab = ads.readADC_Differential_0_1();
          resistances[i] = calculerResistance(rawStab);
          
          Serial.print(">>> Mesure ");
          Serial.print(i + 1);
          Serial.print("/");
          Serial.print(NB_MESURES_STABILISATION);
          Serial.print(" R=");
          if (resistanceValide(resistances[i])) {
            Serial.print(resistances[i]);
            Serial.println(" ohms");
          } else {
            Serial.println("N/A");
            stabilisationOK = false;
          }   
          delay(20);  // 20ms entre mesures de stabilisation		  
        }
        
        // Vérifier la cohérence des mesures : Moyenne puis Calcul du coefficient de variation
        if (stabilisationOK) {
          float moyenne = 0;
          for (int i = 0; i < NB_MESURES_STABILISATION; i++) {
            moyenne += resistances[i];
          }
          moyenne /= NB_MESURES_STABILISATION;
          
          // Calcul coefficient de variation
          float variance = 0;
          for (int i = 0; i < NB_MESURES_STABILISATION; i++) {
            float diff = resistances[i] - moyenne;
            variance += diff * diff;
          }
          float ecartType = sqrt(variance / NB_MESURES_STABILISATION);
          float coefficient = (ecartType / moyenne) * 100;
          
          Serial.print(">>> Moyenne=");
          Serial.print(moyenne);
          Serial.print(" ohms, CV=");
          Serial.print(coefficient, 1);
          Serial.println("%");
          
          // Validation si coefficient de variation < 40% 
		  // Cela permet d’éviter une fausse détection causée par le contact accidentel d’un doigt
          if (coefficient < 40.0) {
            stats.contact_detecte = true;
            stats.temps_premier_contact_us = mesureLocale.temps_us;
            stats.r_contact_initiale = moyenne;
            stats.dac_contact = dacLocal;
            
            contactDetecte = true;
            contactDetecteLocal = true;
            
            Serial.print(">>> CONTACT CONFIRME R=");
            Serial.print(moyenne);
            Serial.println(" ohms");
          } else {
            Serial.println(">>> Mesure instable, poursuite");
          }
        } else {
          Serial.println(">>> Stabilisation echouee");
        }
      }
      
      xSemaphoreGive(mutexStats);
    }
    
    // --- Ajout au buffer circulaire ---
    if (xSemaphoreTake(mutexBuffer, 0) == pdTRUE) {
      if (buffer.count < TAILLE_BUFFER) {
        buffer.mesures[buffer.tete] = mesureLocale;
        buffer.tete = (buffer.tete + 1) % TAILLE_BUFFER;
        buffer.count++;
        nbMesuresTotal++;
      }
      xSemaphoreGive(mutexBuffer);
    }
    
    // Sortie si contact confirmé
    if (contactDetecteLocal) {
      Serial.println(">>> Thread mesure: arret après contact");
      break;
    }
  }
  
  Serial.println(">>> Thread mesure termine");
  vTaskDelete(NULL);
}

// =============================================================================
// THREAD 2 : AFFICHAGE ET SAUVEGARDE
// =============================================================================

/**
 * @brief  Thread d'affichage LCD et sauvegarde SD
 * @param  param Paramètre non utilisé (FreeRTOS)
 * @note   Exécuté sur Core 1 avec priorité haute
 *         Implémente l'anti-flickering LCD
 */
void tacheAffichageSauvegarde(void *param) {
  MesureBrute mesuresLocales[50];
  int nbMesures;
  unsigned long dernierAffichage = 0;
  unsigned long derniereSauvegarde = 0;
  unsigned long maintenant;
  String ligneCSV;
  ligneCSV.reserve(200);
  
  String nouvelleLigne1, nouvelleLigne2, nouvelleLigne3, nouvelleLigne4;
  
  Serial.println("Thread affichage/sauvegarde demarre");
  
  while ((mesureActive && !reinitialiserDemande) || buffer.count > 0) {
    maintenant = millis();
    
    // --- Sauvegarde sur carte SD ---
    if (maintenant - derniereSauvegarde >= INTERVALLE_SAUVEGARDE_MS) {
      nbMesures = 0;
      
      // Extraction des mesures du buffer
      if (xSemaphoreTake(mutexBuffer, pdMS_TO_TICKS(5)) == pdTRUE) {
        while (nbMesures < 50 && buffer.count > 0) {
          mesuresLocales[nbMesures] = buffer.mesures[buffer.queue];
          buffer.queue = (buffer.queue + 1) % TAILLE_BUFFER;
          buffer.count--;
          nbMesures++;
        }
        xSemaphoreGive(mutexBuffer);
      }
      
      // Écriture sur carte SD
      if (nbMesures > 0 && dataFile) {
        for (int i = 0; i < nbMesures; i++) {
          float r = calculerResistance(mesuresLocales[i].raw_diff);
          float v_pont = mesuresLocales[i].raw_diff * 0.125;  // mV
          float v_a2 = mesuresLocales[i].raw_a2 * 0.125;      // mV
          float puissance = mesuresLocales[i].courant_mA * v_a2 / 1000.0;  // mW
          
          // Construction ligne CSV
          ligneCSV = String(numeroTest) + "," + 
                     String(mesuresLocales[i].dac) + "," +
                     formaterValeurCSV(mesuresLocales[i].temps_us / 1000.0, 3) + "," +
                     formaterValeurCSV(mesuresLocales[i].courant_mA, 3) + "," +
                     formaterValeurCSV(v_a2, 3) + "," +
                     formaterValeurCSV(puissance, 3) + "," +
                     formaterValeurCSV(v_pont, 5) + "," +
                     formaterValeurCSV(r, 1);
          
          dataFile.println(ligneCSV);
          nbMesuresSauvees++;
          
          // Affichage périodique sur port série
          if (i % 50 == 0) {
            Serial.print("DAC="); Serial.print(mesuresLocales[i].dac);
            Serial.print(" | t="); Serial.print(mesuresLocales[i].temps_us / 1000.0, 1);
            Serial.print("ms | I="); Serial.print(mesuresLocales[i].courant_mA, 2);
            Serial.print("mA | V="); Serial.print(v_a2 / 1000.0, 3);
            Serial.print("V | P="); Serial.print(puissance, 2);
            Serial.print("mW | R=");
            
            if (resistanceValide(r)) {
              if (r > 1e6) {
                Serial.print(r / 1e6, 2); Serial.print("M");
              } else if (r > 1e3) {
                Serial.print(r / 1e3, 1); Serial.print("k");
              } else {
                Serial.print(r, 0);
              }
              Serial.print(" ohms");
            } else {
              Serial.print("N/A");
            }
            Serial.println();
          }
        }
        
        // Flush périodique pour éviter perte de données
        if (nbMesuresSauvees % 100 == 0) {
          dataFile.flush();
        }
      }
      
      derniereSauvegarde = maintenant;
    }
    
    // --- Mise à jour LCD avec anti-flickering ---
    if (maintenant - dernierAffichage >= INTERVALLE_AFFICHAGE_MS) {
      // Récupération des données actuelles avec timeout court
      StatsTest statsLocales;
      MesureBrute derniereMesure;
      bool statsValides = false, mesureDisponible = false;
      
      if (xSemaphoreTake(mutexStats, pdMS_TO_TICKS(2)) == pdTRUE) {
        statsLocales = stats;
        statsValides = true;
        xSemaphoreGive(mutexStats);
      }
      
      if (xSemaphoreTake(mutexBuffer, pdMS_TO_TICKS(2)) == pdTRUE) {
        if (buffer.count > 0) {
          int indexDerniere = (buffer.tete == 0) ? TAILLE_BUFFER - 1 : buffer.tete - 1;
          derniereMesure = buffer.mesures[indexDerniere];
          mesureDisponible = true;
        }
        xSemaphoreGive(mutexBuffer);
      }
      
      // Préparation contenu LCD
      if (mesureDisponible) {
        float courant_mA = derniereMesure.courant_mA;
        float tension_pont_V = derniereMesure.raw_diff * 0.000125f;
        float tension_A2_V = derniereMesure.raw_a2 * 0.000125f;
        float puissance_mW = courant_mA * tension_A2_V;
        float resistance = calculerResistance(derniereMesure.raw_diff);
        
        nouvelleLigne1 = "DAC:" + String(dacCourant) + " T:" + String(derniereMesure.temps_us / 1000.0, 1) + "ms";
        nouvelleLigne2 = "I:" + String(courant_mA, 1) + " V:" + String(tension_A2_V, 2) + " P:" + String(puissance_mW, 1);
        nouvelleLigne3 = "Vd:" + String(tension_pont_V, 4) + "V R:";
        
        if (resistanceValide(resistance)) {
          if (resistance > 1e6) {
            nouvelleLigne3 += String(resistance / 1e6, 1) + "M";
          } else if (resistance > 1e3) {
            nouvelleLigne3 += String(resistance / 1e3, 0) + "k";
          } else {
            nouvelleLigne3 += String(resistance, 0);
          }
        } else {
          nouvelleLigne3 += "N/A";
        }
        
        nouvelleLigne4 = "Test " + String(numeroTest) + "/" + String(NB_TESTS_TOTAL);
        if (statsValides && statsLocales.contact_detecte) {
          nouvelleLigne4 += " Contact!";
        } else if (mesureActive) {
          nouvelleLigne4 += " Recherche";
        } else {
          nouvelleLigne4 += " Arrete";
        }
      } else {
        nouvelleLigne1 = "DAC:" + String(dacCourant) + " T:0ms";
        nouvelleLigne2 = "I:0 V:0.00 P:0";
        nouvelleLigne3 = "Vd:0.0000V R:N/A";
        nouvelleLigne4 = "Test " + String(numeroTest) + "/" + String(NB_TESTS_TOTAL) + " Arrete";
      }
      
      // Vérification changement
      if (xSemaphoreTake(mutexAffichage, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (affichage.ligne1 != nouvelleLigne1 || 
            affichage.ligne2 != nouvelleLigne2 || 
            affichage.ligne3 != nouvelleLigne3 || 
            affichage.ligne4 != nouvelleLigne4) {
          
          affichage.ligne1 = nouvelleLigne1;
          affichage.ligne2 = nouvelleLigne2;
          affichage.ligne3 = nouvelleLigne3;
          affichage.ligne4 = nouvelleLigne4;
          affichage.change = true;
        }
        xSemaphoreGive(mutexAffichage);
      }
      
      // Rafraîchissement LCD sans clear() pour éviter le flickering
      if (xSemaphoreTake(mutexAffichage, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (affichage.change) {
          // Technique d'effacement sélectif ligne par ligne
          lcd.setCursor(0, 0); 
          lcd.print("                ");  // 16 espaces
          lcd.setCursor(0, 0); 
          lcd.print(affichage.ligne1);
          
          lcd.setCursor(0, 1); 
          lcd.print("                ");
          lcd.setCursor(0, 1); 
          lcd.print(affichage.ligne2);
          
          lcd.setCursor(0, 2); 
          lcd.print("                ");
          lcd.setCursor(0, 2); 
          lcd.print(affichage.ligne3);
          
          lcd.setCursor(0, 3); 
          lcd.print("                ");
          lcd.setCursor(0, 3); 
          lcd.print(affichage.ligne4);
          
          affichage.change = false;
        }
        xSemaphoreGive(mutexAffichage);
      }
      
      dernierAffichage = maintenant;
    }
    
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  
  Serial.println(">>> Thread affichage termine");
  vTaskDelete(NULL);
}

// =============================================================================
// GESTION DES THREADS
// =============================================================================

/**
 * @brief  Démarre les threads d'acquisition et d'affichage
 * @param  tempsDebut Timestamp de référence en microsecondes
 */
void demarrerThreads(uint32_t tempsDebut) {
  Serial.println(">>> Demarrage threads...");
  
  reinitialiserVariablesTest();
  
  tempsDebutMicros = tempsDebut;
  mesureActive = true;
  
  // Thread mesure sur Core 0 (priorité maximale)
  xTaskCreatePinnedToCore(
    tacheMesureUltraRapide,
    "Mesure",
    4096,
    NULL,
    23,
    &tacheMesureHandle,
    0
  );
  
  // Thread affichage sur Core 1 (priorité haute)
  xTaskCreatePinnedToCore(
    tacheAffichageSauvegarde,
    "Affichage",
    8192,
    NULL,
    21,
    &tacheAffichageHandle,
    1
  );
  
  vTaskDelay(pdMS_TO_TICKS(100));
  Serial.println(">>> Threads demarres");
}

/**
 * @brief  Arrête proprement les threads
 */
void arreterThreads() {
  Serial.println(">>> Arret threads...");
  
  reinitialiserDemande = true;
  mesureActive = false;
  
  // Attente de terminaison du thread mesure
  if (tacheMesureHandle) {
    while (eTaskGetState(tacheMesureHandle) != eDeleted) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    tacheMesureHandle = NULL;
  }
  
  // Attente de terminaison du thread affichage
  if (tacheAffichageHandle) {
    while (eTaskGetState(tacheAffichageHandle) != eDeleted) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    tacheAffichageHandle = NULL;
  }
  
  Serial.println(">>> Threads arretes");
}

// =============================================================================
// GESTION INTERFACE UTILISATEUR
// =============================================================================

/**
 * @brief  ISR pour gestion du bouton BOOT
 * @note   Fonction IRAM_ATTR pour exécution depuis la RAM
 */
void IRAM_ATTR gestionBoutonBoot() {
  unsigned long maintenant = millis();
  if (maintenant - dernierAppui > 200) {  // Anti-rebond 200ms
    boutonAppuye = true;
    dernierAppui = maintenant;
  }
}

/**
 * @brief  Vérifie et réinitialise l'état du bouton
 * @return true si le bouton a été appuyé
 */
bool boutonAppuyeEtReset() {
  if (boutonAppuye) {
    boutonAppuye = false;
    return true;
  }
  return false;
}

/**
 * @brief  Affiche un message et attend l'appui du bouton
 * @param  message Message à afficher sur le LCD
 */
void attendreAppuiBouton(String message) {
  // Mise à jour état affichage
  if (xSemaphoreTake(mutexAffichage, pdMS_TO_TICKS(100)) == pdTRUE) {
    affichage.ligne1 = message;
    affichage.ligne2 = "Appuyez BOOT";
    affichage.ligne3 = "";
    affichage.ligne4 = "";
    affichage.change = true;
    xSemaphoreGive(mutexAffichage);
  }
  
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(message);
  lcd.setCursor(0, 1); lcd.print("Appuyez BOOT");
  
  boutonAppuye = false;
  while (!boutonAppuyeEtReset()) {
    delay(100);
  }
  delay(500);  // Délai de stabilisation
}

// =============================================================================
// INITIALISATION MATÉRIELLE
// =============================================================================

/**
 * @brief  Initialise les capteurs I2C
 * @return true si l'initialisation réussit
 */
bool initialiserCapteurs() {
  // Configuration bus I2C
  Wire.begin(21, 22);
  Wire.setClock(400000);  // 400kHz fréquence max limitée par l'INA219

  // Initialisation INA219
  if (!ina219.begin()) {
    Serial.println("ERREUR: INA219");
    return false;
  }
  
  ina219.setCalibration_16V_400mA();

  // Initialisation ADS1115
  if (!ads.begin()) {
    Serial.println("ERREUR: ADS1115");
    return false;
  }

  ads.setGain(GAIN_ONE);           // ±4.096V
  ads.setDataRate(RATE_ADS1115_860SPS);  // 860 SPS
  
  // Lectures d'initialisation
  ads.readADC_Differential_0_1();
  ina219.getCurrent_mA();
  delay(10);

  return true;
}

// =============================================================================
// SETUP - INITIALISATION SYSTÈME
// =============================================================================

/**
 * @brief  Fonction d'initialisation Arduino
 * @note   Appelée une seule fois au démarrage
 */
void setup() {
  Serial.begin(115200);
  delay(500);
  
  // --- Initialisation LCD ---
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Initialisation...");
  
  // --- Initialisation capteurs ---
  if (!initialiserCapteurs()) {
    lcd.setCursor(0, 1);
    lcd.print("Erreur capteurs!");
    while(1) delay(1000);
  }
  
  // --- Initialisation carte SD ---
  if (!SD.begin(SD_CS_PIN, SPI, 40000000)) {
    lcd.setCursor(0, 1);
    lcd.print("Erreur SD!");
    while(1) delay(1000);
  }
  
  // --- Création nom fichier unique ---
  int index = 1;
  char buffer[32];
  while (true) {
    sprintf(buffer, "/TEST_%03d.csv", index);
    if (!SD.exists(buffer)) {
      nomFichier = String(buffer);
      break;
    }
    index++;
  }
  
  // --- Création en-tête CSV ---
  File f = SD.open(nomFichier, FILE_WRITE);
  if (f) {
    f.println("TestID,DAC,Time_ms,I_mA,V_A2_mV,P_mW,V_diff_mV,R_ohms");
    f.close();
  }
  
  // --- Création des mutex ---
  mutexBuffer = xSemaphoreCreateMutex();
  mutexStats = xSemaphoreCreateMutex();
  mutexAffichage = xSemaphoreCreateMutex();
  
  // --- Initialisation structures ---
  memset(&affichage, 0, sizeof(affichage));
  
  // --- Configuration E/S ---
  dacWrite(DAC_RELAIS, 0);
  pinMode(BOUTON_BOOT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BOUTON_BOOT), gestionBoutonBoot, FALLING);
  
  // --- Affichage prêt ---
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Systeme pret");
  lcd.setCursor(0, 1); lcd.print(nomFichier);
  lcd.setCursor(0, 2); lcd.print("Tests: 0/"); lcd.print(NB_TESTS_TOTAL);
  
  Serial.println("=============================================================================");
  Serial.println("  SYSTÈME DE MESURE DE RÉSISTANCE - INITIALISÉ");
  Serial.println("=============================================================================");
  Serial.print("  Fichier: "); Serial.println(nomFichier);
  Serial.print("  Nombre de tests: "); Serial.println(NB_TESTS_TOTAL);
  Serial.println("=============================================================================");
  
  delay(2000);
}

// =============================================================================
// LOOP - BOUCLE PRINCIPALE
// =============================================================================

/**
 * @brief  Boucle principale du programme
 * @note   Gère le séquencement des tests et le contrôle DAC
 */
void loop() {
  Serial.println("\n=============================================================================");
  Serial.print("  TEST ");
  Serial.print(numeroTest);
  Serial.print(" / ");
  Serial.println(NB_TESTS_TOTAL);
  Serial.println("=============================================================================");
  
  // --- Préparation du test ---
  reinitialiserVariablesTest();
  dacWrite(DAC_RELAIS, 0);
  
  // --- Synchronisation temporelle ---
  uint32_t tempsDebut = micros();
  uint32_t tempsDebutMs = millis();
  int dacValue = 0;
  bool testEnCours = true;
  
  Serial.print(">>> Timestamp début: "); 
  Serial.print(tempsDebut); 
  Serial.println(" µs");
  
  // --- Ouverture fichier données ---
  dataFile = SD.open(nomFichier, FILE_APPEND);
  if (!dataFile) {
    Serial.println("ERREUR: Impossible d'ouvrir le fichier!");
    return;
  }
  
  // --- Lancement des threads d'acquisition ---
  demarrerThreads(tempsDebut);
  delay(50);
  
  Serial.println(">>> Début balayage DAC...");
  
  // =============================================================================
  // BOUCLE DE CONTRÔLE DAC
  // =============================================================================
  
  while (dacValue <= 255 && testEnCours) {
    // Application de la valeur DAC
    dacWrite(DAC_RELAIS, dacValue);
    dacCourant = dacValue;
    
    Serial.print("DAC="); Serial.print(dacValue);
    
    // --- Temporisation avec surveillance ---
    uint32_t debutDelai = millis();
    const uint32_t DUREE_PALIER_MS = 500;  // 500ms par palier
    
    while (millis() - debutDelai < DUREE_PALIER_MS && testEnCours) {
      delay(25);
      
      // Lecture des états
      bool contactLocal = contactDetecte;
      bool urgenceLocal = arretUrgence;
      bool mesureActiveLocal = mesureActive;
      
      // --- Vérification timeout global ---
      if (millis() - tempsDebutMs > DUREE_MAX_TEST_MS) {
        Serial.println(" >>> TIMEOUT TEST");
        testEnCours = false;
        break;
      }
      
      // --- Arrêt d'urgence (surintensité) ---
      if (urgenceLocal) {
        Serial.println(" >>> ARRÊT URGENCE");
        testEnCours = false;
        break;
      }
      
      // --- Contact détecté et validé ---
      if (contactLocal) {
        Serial.println(" >>> CONTACT CONFIRME");
        testEnCours = false;
        break;
      }
      
      // --- Thread mesure arrêté anormalement ---
      if (!mesureActiveLocal && millis() - debutDelai > 200) {
        Serial.println(" >>> THREAD MESURE ARRÊTE");
        testEnCours = false;
        break;
      }
      
      // --- Gestion pause utilisateur ---
      if (boutonAppuyeEtReset()) {
        Serial.println(" >>> PAUSE UTILISATEUR");
        arreterThreads();
        dacWrite(DAC_RELAIS, 0);
        
        lcd.clear();
        lcd.setCursor(0, 0); lcd.print("PAUSE - Test ");
        lcd.setCursor(0, 1); lcd.print(numeroTest);
        lcd.setCursor(0, 2); lcd.print("Appuyez pour");
        lcd.setCursor(0, 3); lcd.print("reprendre");
        
        while (!boutonAppuyeEtReset()) delay(100);
        
        Serial.println(" >>> REPRISE TEST");
        demarrerThreads(tempsDebut);
        break;
      }
    }
    
    if (testEnCours) {
      dacValue++;
      Serial.println(" OK");
    }
  }
  
  // =============================================================================
  // FIN DE TEST ET RAPPORT
  // =============================================================================
  
  Serial.println(">>> Arrêt du test...");
  arreterThreads();
  dacWrite(DAC_RELAIS, 0);
  
  // Fermeture fichier
  if (dataFile) {
    dataFile.flush();
    dataFile.close();
  }
  
  // --- Calcul statistiques finales ---
  uint32_t dureeMs = millis() - tempsDebutMs;
  StatsTest statsFinal;
  bool statsValides = false;
  
  if (xSemaphoreTake(mutexStats, pdMS_TO_TICKS(100)) == pdTRUE) {
    statsFinal = stats;
    statsValides = true;
    xSemaphoreGive(mutexStats);
  }
  
  // --- Affichage rapport console ---
  Serial.println("\n-----------------------------------------------------------------------------");
  Serial.println("  RAPPORT DE TEST");
  Serial.println("-----------------------------------------------------------------------------");
  Serial.print("  Durée totale: "); Serial.print(dureeMs / 1000.0); Serial.println(" s");
  Serial.print("  Mesures acquises: "); Serial.println(nbMesuresTotal);
  Serial.print("  Mesures sauvegardées: "); Serial.println(nbMesuresSauvees);
  Serial.print("  Fréquence moyenne: "); Serial.print(nbMesuresTotal * 1000.0 / dureeMs); Serial.println(" Hz");
  
  // --- Affichage résultats LCD ---
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Test "); lcd.print(numeroTest); lcd.print(" termine");
  
  if (arretUrgence) {
    // Arrêt sécurité
    lcd.setCursor(0, 1); lcd.print("SECURITE!");
    lcd.setCursor(0, 2); lcd.print("I > "); lcd.print(SEUIL_COURANT_MAX_mA, 0); lcd.print("mA");
    Serial.println("  Résultat: ARRÊT SÉCURITÉ (surintensité)");
  } 
  else if (statsValides && statsFinal.contact_detecte) {
    // Contact détecté
    lcd.setCursor(0, 1);
    lcd.print("Contact DAC:"); lcd.print(statsFinal.dac_contact);
    lcd.setCursor(0, 2);
    lcd.print("R:");
    if (statsFinal.r_contact_initiale > 1e6) {
      lcd.print(statsFinal.r_contact_initiale / 1e6, 2); lcd.print("M");
    } else if (statsFinal.r_contact_initiale > 1e3) {
      lcd.print(statsFinal.r_contact_initiale / 1e3, 1); lcd.print("k");
    } else {
      lcd.print(statsFinal.r_contact_initiale, 0);
    }
    lcd.print(" ohms");
    lcd.setCursor(0, 3);
    lcd.print("t="); lcd.print(statsFinal.temps_premier_contact_us / 1000.0, 1); lcd.print("ms");
    
    Serial.println("  Résultat: CONTACT DÉTECTÉ");
    Serial.print("    - DAC: "); Serial.println(statsFinal.dac_contact);
    Serial.print("    - Résistance: "); Serial.print(statsFinal.r_contact_initiale); Serial.println(" Ω");
    Serial.print("    - Temps détection: "); Serial.print(statsFinal.temps_premier_contact_us / 1000.0); Serial.println(" ms");
  } 
  else {
    // Pas de contact
    lcd.setCursor(0, 1); lcd.print("Pas de contact");
    lcd.setCursor(0, 2); lcd.print("DAC max: "); lcd.print(dacValue > 255 ? 255 : dacValue);
    lcd.setCursor(0, 3); lcd.print("Duree: "); lcd.print(dureeMs / 1000.0, 1); lcd.print("s");
    Serial.println("  Résultat: AUCUN CONTACT DÉTECTÉ");
  }
  Serial.println("-----------------------------------------------------------------------------\n");
  
  // Pause affichage résultats
  delay(5000);
  
  // =============================================================================
  // PRÉPARATION TEST SUIVANT
  // =============================================================================
  
  numeroTest++;
  
  // --- Vérification fin de cycle ---
  if (numeroTest > NB_TESTS_TOTAL) {
    Serial.println("=============================================================================");
    Serial.println("  CYCLE COMPLET TERMINÉ");
    Serial.println("=============================================================================");
    numeroTest = 1;
    
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Cycle complet");
    lcd.setCursor(0, 1); lcd.print("Total: "); lcd.print(NB_TESTS_TOTAL); lcd.print(" tests");
    lcd.setCursor(0, 2); lcd.print("Nouveau cycle?");
    lcd.setCursor(0, 3); lcd.print("Appuyez BOOT");
    
    // Attente confirmation nouveau cycle
    boutonAppuye = false;
    while (!boutonAppuyeEtReset()) {
      delay(100);
    }
    
    Serial.println(">>> Démarrage nouveau cycle");
  }
  
  // --- Compte à rebours prochain test ---
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Preparation");
  lcd.setCursor(0, 1); lcd.print("Test "); lcd.print(numeroTest); lcd.print(" dans ");
  lcd.setCursor(0, 2); lcd.print("BOOT = pause");
  
  // Temporisation avec possibilité de pause
  for (int i = 3; i > 0; i--) {
    lcd.setCursor(14, 1); lcd.print(i); lcd.print("s ");
    
    for (int j = 0; j < 10; j++) {
      if (boutonAppuyeEtReset()) {
        attendreAppuiBouton("Test en pause");
        break;
      }
      delay(100);
    }
  }
  Serial.print(">>> Lancement test "); Serial.println(numeroTest);
}