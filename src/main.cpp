/***********************************************Smart Ruler Firmware (ARP-first + Device ID)*****************************************
 * Autor: Henrique Rosa 
 * Projeto: Smart Ruler (SMR)
 * Versão: 1.6.7 - Descoberta de PC robusta (ARP) + Busca ID dispositivo
 * Data: [Dez/2024]
 * Atualizado : 10/09/2025
 * Foco desta versão:
 *   - Descoberta do PC por ARP (rápida e confiável)
 *   - Busca do ID do dispositivo via HTTP GET em /elemidia_v4/util/iot_hub/scripts/dispositivo.json
 *   - Persistência de IP/MAC/ID em NVS
 *   - Inclusão do device_id no payload de status
 ***********************************************************************************************************************/

// -------------------------------------------------------------------------------------------
//  Bibliotecas Usadas
// -------------------------------------------------------------------------------------------
#include <ETH.h>
#include <ESP32Ping.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <ArduinoJson.h>  // Para parsing do JSON

// lwIP (ARP / netif)
extern "C" {
  #include "lwip/err.h"
  #include "lwip/ip4_addr.h"
  #include "lwip/netif.h"
  #include "lwip/etharp.h"
}

// -------------------------------------------------------------------------------------------
//                               Configurações Ethernet e Rede
// -------------------------------------------------------------------------------------------
#define ETH_PHY_ADDR        0
#define ETH_PHY_POWER_PIN   -1
#define ETH_PHY_MDC_PIN     23
#define ETH_PHY_MDIO_PIN    18
#undef  ETH_CLK_MODE
#define ETH_CLK_MODE        ETH_CLOCK_GPIO17_OUT
#define ETH_PHY_TYPE        ETH_PHY_LAN8720

// -------------------------------------------------------------------------------------------
//                               Definições de pinos
// -------------------------------------------------------------------------------------------
#define RELAY_MODEM_PIN     14    // RL1
#define RELAY_PC_PIN        32    // RL2
#define RELAY_K3            12    // RL3
#define RELAY_K4            33    // RL4

// -------------------------------------------------------------------------------------------
//                              Definições de ENDPOINT
// -------------------------------------------------------------------------------------------
#define STATUS_ENDPOINT        "https://iothub-v2-homolog.eletromidia.com.br/api/v1/reguas/pub"
#define COMMAND_SUB_ENDPOINT   "https://iothub-v2-homolog.eletromidia.com.br/api/v1/reguas/sub"
#define COMMAND_SUB_CONFIR     "https://iothub-v2-homolog.eletromidia.com.br/api/v1/reguas/subACK"

// Endpoint do dispositivo.json no NUC
#define DEVICE_INFO_PATH       "/elemidia_v4/util/iot_hub/scripts/dispositivo.json"

// -------------------------------------------------------------------------------------------
//                          Configurações e Timeouts
// -------------------------------------------------------------------------------------------
#define STATUS_INTERVAL_MS          120000  // 2 min
#define COMMAND_INTERVAL_MS         3000    // 3 s
#define CONNECTIVITY_INTERVAL_MS    80000   // 80 s

#define RESET_DURATION_MS           5000
#define MODEM_RESET_MIN_INTERVAL_MS 180000
#define PC_RESET_MIN_INTERVAL_MS    60000
#define MODEM_COOLDOWN_AFTER_RESET  180000
#define PC_COOLDOWN_AFTER_RESET     60000
#define MAX_RETRY                   4
#define MAX_CONSECUTIVE_FAILURES    7
#define EVENT_TYPE                  "keep_alive"
#define DETAIL                      "regua-boe"

#define HTTP_TIMEOUT_SHORT          2000
#define HTTP_TIMEOUT_MEDIUM         4000

// Descoberta de PC (teto total)
#define PC_DISCOVERY_TIMEOUT_MS     25000   // ~25s
#define PC_DISCOVERY_STEP_DELAY     8       // 8 ms entre probes ARP
#define ARP_REPLY_WAIT_MS           28      // espera curta por resposta ARP

// Watchdog
#define WDT_TIMEOUT                 45

// -------------------------------------------------------------------------------------------
//                              Estruturas
// -------------------------------------------------------------------------------------------
struct RelayResetState {
  unsigned long startTime;
  bool          relayActive;
  unsigned long lastResetTime;
  int           resetCount;
};

// -------------------------------------------------------------------------------------------
//                              Variáveis Globais
// -------------------------------------------------------------------------------------------
String modemIP = "";
String pcIP    = "";
String pcMAC   = "";
String deviceMAC = "";
String deviceID = "";  // ID do dispositivo (do JSON)
Preferences prefs;

// Estados dos relés
RelayResetState modemRelayState = {0, false, 0, 0};
RelayResetState pcRelayState    = {0, false, 0, 0};

// Flags de controle
volatile bool externalModemReset = false;
volatile bool externalPCReset    = false;
bool pcResetOccurred    = false;
bool modemResetOccurred = false;

// -------------------------------------------------------------------------------------------
//                              Configuração NTP
// -------------------------------------------------------------------------------------------
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", -3 * 3600, 60000);

// -------------------------------------------------------------------------------------------
//                              Protótipos de Funções
// -------------------------------------------------------------------------------------------
void   logMessage(const String &level, const String &tag, const String &message);
bool   checkModemConnectivity();
bool   checkPCConnectivity();
String discoverPC();                         // ARP-first
bool   validatePCIP(const IPAddress &ip);    // ARP validation + (opcional) ICMP
bool   arpProbe(const IPAddress &ip, String *macOut);
bool   isInfrastructureHost(uint8_t last);
void   savePC(const String &ip, const String &mac, const String &id);
void   loadPC(String &ip, String &mac, String &id);
String fetchDeviceID(const String &pcIP);    // Nova função para buscar ID

void   activateRelay(int relayPin, RelayResetState* state, const String &relayName);
void   deactivateRelay(int relayPin, RelayResetState* state, const String &relayName);
void   resetDeviceNonBlocking(int relayPin, unsigned long delayTime, RelayResetState* state);

void   sendStatusToEndpoint();
void   handleCommandsFromEndpoint();
String obterTimestamp();
void   confirmarRecebimentoComando(const String &comando, const String &descricao);
String getPCMacAddress();

// -------------------------------------------------------------------------------------------
//                              Funções Utilitárias
// -------------------------------------------------------------------------------------------
void logMessage(const String &level, const String &tag, const String &message) {
  Serial.print(level); Serial.print(" ["); Serial.print(tag); Serial.print("] ");
  Serial.println(message);
}

String getPCMacAddress() {
  if (deviceMAC.isEmpty()) {
    deviceMAC = ETH.macAddress();
    logMessage("[INFO]", "MAC", "MAC Address: " + deviceMAC);
  }
  return deviceMAC;
}

// Converte MAC binário para string "AA:BB:CC:DD:EE:FF"
static String macToString(const uint8_t *m) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", m[0],m[1],m[2],m[3],m[4],m[5]);
  return String(buf);
}

static struct netif* getEthNetif() {
  return netif_default; // em projetos simples, aponta para a interface ativa (ETH)
}

// Faz probe ARP no IP, aguardando curta janela por resposta e consulta o cache
bool arpProbe(const IPAddress &ip, String *macOut) {
  struct netif *nif = getEthNetif();
  if (!nif) return false;

  ip4_addr_t addr;
  IP4_ADDR(&addr, ip[0], ip[1], ip[2], ip[3]);

  // 1) Verifica se já está no cache
  struct eth_addr *eth_ret = nullptr;
  const ip4_addr_t *ip_ret = nullptr;
  s8_t r = etharp_find_addr(nif, &addr, &eth_ret, &ip_ret);
  if (r >= 0 && eth_ret) {
    if (macOut) *macOut = macToString(eth_ret->addr);
    return true;
  }

  // 2) Envia ARP request e espera retorno curto
  etharp_request(nif, &addr);
  uint32_t t0 = millis();
  while ((millis() - t0) < ARP_REPLY_WAIT_MS) {
    r = etharp_find_addr(nif, &addr, &eth_ret, &ip_ret);
    if (r >= 0 && eth_ret) {
      if (macOut) *macOut = macToString(eth_ret->addr);
      return true;
    }
    delay(2);
  }
  return false;
}

// Valida um IP candidato por ARP e, opcionalmente, por ICMP (1 tentativa)
bool validatePCIP(const IPAddress &ip) {
  String mac;
  bool ok = arpProbe(ip, &mac);
  if (!ok) return false;

  // Guarda MAC visto
  pcMAC = mac;

  // ICMP opcional: a lib não tem timeout custom; 1 tentativa é suficiente
  bool icmp = Ping.ping(ip, 1);
  if (!icmp) {
    logMessage("[WARN]", "DISCOVERY", "ARP OK, ICMP falhou (possível filtro). Aceitando por ARP.");
  }
  return true;
}

// Evita confundir com gateway/broadcast/infra
bool isInfrastructureHost(uint8_t last) {
  return (last == 0) || (last == 1) || (last == 254) || (last == 255) || (last == 201);
}

// Nova função para buscar o ID do dispositivo via HTTP
String fetchDeviceID(const String &pcIP) {
  if (pcIP.isEmpty()) return "";
  
  String url = "http://" + pcIP + DEVICE_INFO_PATH;
  logMessage("[INFO]", "DEVICE_ID", "Buscando ID em: " + url);
  
  HTTPClient http;
  http.begin(url);
  http.setTimeout(HTTP_TIMEOUT_SHORT);
  
  int httpResponseCode = http.GET();
  String id = "";
  
  if (httpResponseCode == 200) {
    String response = http.getString();
    
    // Parse JSON para extrair o ID
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, response);
    
    if (!error) {
      if (doc.containsKey("id")) {
        id = doc["id"].as<String>();
        logMessage("[INFO]", "DEVICE_ID", "ID encontrado: " + id);
      } else {
        logMessage("[WARN]", "DEVICE_ID", "Campo 'id' não encontrado no JSON");
      }
    } else {
      logMessage("[ERROR]", "DEVICE_ID", "Erro ao fazer parse do JSON: " + String(error.c_str()));
    }
  } else if (httpResponseCode > 0) {
    logMessage("[WARN]", "DEVICE_ID", "HTTP Response: " + String(httpResponseCode));
  } else {
    logMessage("[ERROR]", "DEVICE_ID", "Falha na requisição HTTP");
  }
  
  http.end();
  return id;
}

void savePC(const String &ip, const String &mac, const String &id) {
  prefs.begin("smr", false);
  prefs.putString("pc_ip",  ip);
  prefs.putString("pc_mac", mac);
  prefs.putString("pc_id", id);
  prefs.end();
}

void loadPC(String &ip, String &mac, String &id) {
  prefs.begin("smr", true);
  ip  = prefs.getString("pc_ip", "");
  mac = prefs.getString("pc_mac", "");
  id  = prefs.getString("pc_id", "");
  prefs.end();
}

// -------------------------------------------------------------------------------------------
//                               Tarefas (FreeRTOS)
// -------------------------------------------------------------------------------------------

void connectivityTask(void *parameter) {
  unsigned long lastCheck = 0;
  esp_task_wdt_add(NULL);

  for (;;) {
    esp_task_wdt_reset();

    // Comandos urgentes
    if (externalModemReset || externalPCReset) {
      logMessage("[INFO]", "CONNECTIVITY", "COMANDO URGENTE DETECTADO - PROCESSANDO");
      if (externalModemReset) {
        if (!modemRelayState.relayActive && (millis() - modemRelayState.lastResetTime) > MODEM_RESET_MIN_INTERVAL_MS) {
          activateRelay(RELAY_MODEM_PIN, &modemRelayState, "Modem");
          modemResetOccurred = true;
          confirmarRecebimentoComando("K1=0", "Resetar Modem");
        } else {
          confirmarRecebimentoComando("K1=0", "Reset modem negado - intervalo mínimo");
        }
        externalModemReset = false;
      }
      if (externalPCReset) {
        if (!pcRelayState.relayActive && (millis() - pcRelayState.lastResetTime) > PC_RESET_MIN_INTERVAL_MS) {
          activateRelay(RELAY_PC_PIN, &pcRelayState, "PC");
          pcResetOccurred = true;
          confirmarRecebimentoComando("K2=0", "Resetar PC");
        } else {
          confirmarRecebimentoComando("K2=0", "Reset PC negado - intervalo mínimo");
        }
        externalPCReset = false;
      }
    }

    if ((millis() - lastCheck) >= CONNECTIVITY_INTERVAL_MS) {
      lastCheck = millis();
      logMessage("[INFO]", "CONNECTIVITY", "Iniciando verificação de conectividade");

      esp_task_wdt_reset();
      bool modemOk = true;

      // Cooldown do modem
      if (modemRelayState.lastResetTime > 0 &&
          (millis() - modemRelayState.lastResetTime) < MODEM_COOLDOWN_AFTER_RESET) {
        unsigned long remainingCooldown = (MODEM_COOLDOWN_AFTER_RESET - (millis() - modemRelayState.lastResetTime)) / 1000;
        logMessage("[INFO]", "CONNECTIVITY", "Modem em cooldown por mais " + String(remainingCooldown) + "s");
      } else {
        modemOk = checkModemConnectivity();
        if (!modemOk) {
          logMessage("[WARN]", "CONNECTIVITY", "Problema detectado com o modem");
          if (!modemRelayState.relayActive &&
              (millis() - modemRelayState.lastResetTime) > MODEM_RESET_MIN_INTERVAL_MS) {
            activateRelay(RELAY_MODEM_PIN, &modemRelayState, "Modem");
            modemResetOccurred = true;
          }
        }
      }

      esp_task_wdt_reset();

      // Descoberta do PC (ARP-first) somente se modem OK
      if (modemOk && pcIP.isEmpty()) {
        logMessage("[INFO]", "CONNECTIVITY", "Descobrindo IP do PC (ARP-first)");
        pcIP = discoverPC();
        if (!pcIP.isEmpty()) {
          logMessage("[INFO]", "CONNECTIVITY", "PC encontrado: " + pcIP + (pcMAC.isEmpty() ? "" : "  MAC=" + pcMAC));
          
          // Buscar ID do dispositivo
          deviceID = fetchDeviceID(pcIP);
          if (!deviceID.isEmpty()) {
            logMessage("[INFO]", "CONNECTIVITY", "Device ID obtido: " + deviceID);
          }
          
          savePC(pcIP, pcMAC, deviceID);
        } else {
          logMessage("[WARN]", "CONNECTIVITY", "PC não encontrado após timeout");
        }
      }

      esp_task_wdt_reset();

      // Verificação do PC com cooldown
      if (modemOk && !pcIP.isEmpty()) {
        if (pcRelayState.lastResetTime > 0 &&
            (millis() - pcRelayState.lastResetTime) < PC_COOLDOWN_AFTER_RESET) {
          unsigned long remainingCooldown = (PC_COOLDOWN_AFTER_RESET - (millis() - pcRelayState.lastResetTime)) / 1000;
          logMessage("[INFO]", "CONNECTIVITY", "PC em cooldown por mais " + String(remainingCooldown) + "s");
        } else {
          if (!checkPCConnectivity()) {
            logMessage("[WARN]", "CONNECTIVITY", "Problema detectado com o PC");
            if (!pcRelayState.relayActive &&
                (millis() - pcRelayState.lastResetTime) > PC_RESET_MIN_INTERVAL_MS) {
              activateRelay(RELAY_PC_PIN, &pcRelayState, "PC");
              pcResetOccurred = true;
            }
          } else {
            // Se PC está OK e não temos ID, tentar buscar novamente
            if (deviceID.isEmpty()) {
              deviceID = fetchDeviceID(pcIP);
              if (!deviceID.isEmpty()) {
                logMessage("[INFO]", "CONNECTIVITY", "Device ID obtido na reverificação: " + deviceID);
                savePC(pcIP, pcMAC, deviceID);
              }
            }
          }
        }
      }

      esp_task_wdt_reset();

      if (modemResetOccurred) {
        confirmarRecebimentoComando("Local Reset K1=1", "Resetar Modem Localmente");
        modemResetOccurred = false;
      }
      if (pcResetOccurred) {
        confirmarRecebimentoComando("Local Reset K2=1", "Resetar PC Localmente");
        pcResetOccurred = false;
      }
      
      // Log com status completo
      String statusMsg = "Status: Modem=" + String(modemOk ? "OK" : "FAIL");
      statusMsg += " | PC=" + (pcIP.isEmpty() ? "Não encontrado" : pcIP);
      statusMsg += " | ID=" + (deviceID.isEmpty() ? "Não obtido" : deviceID);
      logMessage("[INFO]", "CONNECTIVITY", statusMsg);
      logMessage("[INFO]", "CONNECTIVITY", "Verificação concluída");
    }

    // Gestão de relés (sempre)
    resetDeviceNonBlocking(RELAY_MODEM_PIN, RESET_DURATION_MS, &modemRelayState);
    resetDeviceNonBlocking(RELAY_PC_PIN,   RESET_DURATION_MS, &pcRelayState);

    vTaskDelay(pdMS_TO_TICKS(500));
    esp_task_wdt_reset();
  }
}

void statusTask(void *parameter) {
  unsigned long lastStatus = 0;
  esp_task_wdt_add(NULL);
  for (;;) {
    esp_task_wdt_reset();
    if ((millis() - lastStatus) >= STATUS_INTERVAL_MS) {
      lastStatus = millis();
      logMessage("[INFO]", "STATUS", "Enviando status para o servidor");
      sendStatusToEndpoint();
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void commandsTask(void *parameter) {
  unsigned long lastCmdCheck = 0;
  esp_task_wdt_add(NULL);
  for (;;) {
    esp_task_wdt_reset();
    if ((millis() - lastCmdCheck) >= COMMAND_INTERVAL_MS) {
      lastCmdCheck = millis();
      logMessage("[DEBUG]", "COMMANDS", "Verificando comandos no servidor");
      handleCommandsFromEndpoint();
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// -------------------------------------------------------------------------------------------
//                                     Setup e Loop
// -------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  logMessage("[INFO]", "SYSTEM", "Inicializando Smart Ruler v1.9.1b (ARP + Device ID)");

  // Relés
  pinMode(RELAY_MODEM_PIN, OUTPUT);
  pinMode(RELAY_PC_PIN,   OUTPUT);
  pinMode(RELAY_K3,       OUTPUT);
  pinMode(RELAY_K4,       OUTPUT);
  digitalWrite(RELAY_MODEM_PIN, LOW);
  digitalWrite(RELAY_PC_PIN,   LOW);
  digitalWrite(RELAY_K3,       LOW);
  digitalWrite(RELAY_K4,       LOW);

  // Watchdog
  esp_task_wdt_init(WDT_TIMEOUT, true);

  // Ethernet
  if (!ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER_PIN, ETH_PHY_MDC_PIN, ETH_PHY_MDIO_PIN, ETH_PHY_TYPE, ETH_CLK_MODE)) {
    logMessage("[ERROR]", "NETWORK", "Falha ao inicializar Ethernet");
    ESP.restart();
  }
  logMessage("[INFO]", "NETWORK", "Ethernet inicializado");

  // DHCP com timeout
  const unsigned long DHCP_TIMEOUT = 60000;
  uint32_t t0 = millis();
  while (ETH.localIP() == INADDR_NONE) {
    if ((millis() - t0) > DHCP_TIMEOUT) {
      logMessage("[ERROR]", "NETWORK", "Timeout DHCP - Reiniciando");
      ESP.restart();
    }
    delay(250);
  }
  logMessage("[INFO]", "NETWORK", "IP obtido: " + ETH.localIP().toString());

  // Gateway
  IPAddress gw = ETH.gatewayIP();
  if (gw == INADDR_NONE) {
    modemIP = "192.168.0.1";
    logMessage("[WARN]", "NETWORK", "Gateway não encontrado, usando fallback: " + modemIP);
  } else {
    modemIP = gw.toString();
    logMessage("[INFO]", "NETWORK", "Gateway encontrado: " + modemIP);
  }

  // NTP
  timeClient.begin();
  timeClient.update();

  // Seed rand
  randomSeed(micros());

  // Carrega IP/MAC/ID persistidos do PC (se houver)
  loadPC(pcIP, pcMAC, deviceID);
  if (!pcIP.isEmpty()) {
    logMessage("[INFO]", "DISCOVERY", "PC salvo: " + pcIP + 
               (pcMAC.isEmpty() ? "" : "  MAC=" + pcMAC) +
               (deviceID.isEmpty() ? "" : "  ID=" + deviceID));
    IPAddress ip; ip.fromString(pcIP);
    if (!validatePCIP(ip)) {
      logMessage("[WARN]", "DISCOVERY", "PC salvo inválido. Nova descoberta será necessária.");
      pcIP  = "";
      pcMAC = "";
      deviceID = "";
    } else if (deviceID.isEmpty()) {
      // Se PC válido mas sem ID, tentar buscar
      deviceID = fetchDeviceID(pcIP);
      if (!deviceID.isEmpty()) {
        savePC(pcIP, pcMAC, deviceID);
      }
    }
  }

  // Tasks
  xTaskCreatePinnedToCore(connectivityTask, "ConnectivityTask", 8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(statusTask,       "StatusTask",       6144, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(commandsTask,     "CommandsTask",     6144, NULL, 3, NULL, 1);

  logMessage("[INFO]", "SYSTEM", "Sistema inicializado com sucesso");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(10000));
}

// -------------------------------------------------------------------------------------------
//                          Funções de Conectividade
// -------------------------------------------------------------------------------------------
bool checkModemConnectivity() {
  static int consecutiveFailures = 0;
  bool localConnectionOk = false;
  bool internetConnectionOk = false;

  logMessage("[DEBUG]", "MODEM", "Verificando conectividade: " + modemIP);

  for (int i = 0; i < MAX_RETRY; i++) {
    if (Ping.ping(modemIP.c_str())) { localConnectionOk = true; break; }
    vTaskDelay(pdMS_TO_TICKS(250));
  }

  if (localConnectionOk) {
    for (int i = 0; i < MAX_RETRY; i++) {
      if (Ping.ping("8.8.8.8")) { internetConnectionOk = true; break; }
      vTaskDelay(pdMS_TO_TICKS(250));
    }
  }

  if (!localConnectionOk || !internetConnectionOk) {
    consecutiveFailures++;
    logMessage("[WARN]", "MODEM", "Falha na conectividade. Consecutivas: " + String(consecutiveFailures));
    if (consecutiveFailures >= MAX_CONSECUTIVE_FAILURES) {
      logMessage("[ERROR]", "MODEM", "Excesso de falhas consecutivas - Reiniciando ESP");
      vTaskDelay(pdMS_TO_TICKS(500));
      ESP.restart();
    }
    return false;
  } else {
    if (consecutiveFailures > 0) logMessage("[INFO]", "MODEM", "Conectividade restaurada");
    consecutiveFailures = 0;
    return true;
  }
}

bool checkPCConnectivity() {
  if (pcIP.isEmpty()) return false;

  logMessage("[DEBUG]", "PC", "Verificando conectividade: " + pcIP);

  IPAddress ip; ip.fromString(pcIP);
  // Primeiro tenta ARP (rápido)
  String mac;
  bool arpOk = arpProbe(ip, &mac);
  if (arpOk && !pcMAC.isEmpty() && !mac.isEmpty() && (mac != pcMAC)) {
    logMessage("[WARN]", "PC", "MAC diferente do persistido. Atualizando registro.");
    pcMAC = mac; 
    savePC(pcIP, pcMAC, deviceID);
  }

  if (arpOk) {
    if (pcRelayState.relayActive) {
      logMessage("[INFO]", "PC", "Restauração detectada - Desativando relé");
      deactivateRelay(RELAY_PC_PIN, &pcRelayState, "PC");
    }
    return true;
  }

  // Fallback: 1 ping rápido
  bool icmp = Ping.ping(ip, 1);
  if (icmp) return true;

  logMessage("[WARN]", "PC", "PC offline (ARP e ICMP falharam)");
  return false;
}

// Descoberta ARP-first com persistência em NVS
String discoverPC() {
  const uint32_t TMAX = PC_DISCOVERY_TIMEOUT_MS;
  uint32_t t0 = millis();

  IPAddress myIP = ETH.localIP();
  IPAddress gwIP = ETH.gatewayIP();

  // Prefixo /24
  IPAddress base(myIP[0], myIP[1], myIP[2], 0);
  uint8_t myLast = myIP[3];

  logMessage("[INFO]", "DISCOVERY", "ARP-first na sub-rede: " + String(base[0]) + "." + String(base[1]) + "." + String(base[2]) + ".x");

  // 0) IPs comuns (prioridade)
  const uint8_t commons[] = {20,21,22,23,24,25, 10,11,12,13, 30,50,
                             100,101,102,103,104,105, 120,150, 200,201,205,210,220,230,240};
  for (size_t i=0; i<sizeof(commons); ++i) {
    if ((millis() - t0) > TMAX) { logMessage("[WARN]", "DISCOVERY", "Timeout nos IPs comuns"); return ""; }
    uint8_t h = commons[i];
    if (isInfrastructureHost(h) || h == myLast) continue;

    IPAddress ip(base[0], base[1], base[2], h);
    String mac;
    if (arpProbe(ip, &mac)) {
      if (h == gwIP[3]) { logMessage("[DEBUG]", "DISCOVERY", "Ignorando gateway (" + ip.toString() + ")"); continue; }
      pcMAC = mac;
      logMessage("[INFO]", "DISCOVERY", "Encontrado por ARP (comum): " + ip.toString() + "  MAC=" + mac);
      return ip.toString();
    }
    vTaskDelay(pdMS_TO_TICKS(PC_DISCOVERY_STEP_DELAY));
  }

  // 1) Janela ao redor do próprio IP (±32)
  int start = max(2, (int)myLast - 32);
  int end   = min(253, (int)myLast + 32);
  for (int h = start; h <= end; ++h) {
    if ((millis() - t0) > TMAX) { logMessage("[WARN]", "DISCOVERY", "Timeout na janela ±32"); return ""; }
    if (h == myLast || isInfrastructureHost(h) || h == gwIP[3]) continue;

    IPAddress ip(base[0], base[1], base[2], (uint8_t)h);
    String mac;
    if (arpProbe(ip, &mac)) {
      pcMAC = mac;
      logMessage("[INFO]", "DISCOVERY", "Encontrado por ARP (janela): " + ip.toString() + "  MAC=" + mac);
      return ip.toString();
    }
    vTaskDelay(pdMS_TO_TICKS(PC_DISCOVERY_STEP_DELAY));
  }

  // 2) Varredura restante do /24 (ARP-only)
  for (int h = 2; h <= 253; ++h) {
    if ((millis() - t0) > TMAX) { logMessage("[WARN]", "DISCOVERY", "Timeout no /24"); return ""; }
    if (h == myLast || isInfrastructureHost(h) || h == gwIP[3] || (h >= start && h <= end)) continue;

    IPAddress ip(base[0], base[1], base[2], (uint8_t)h);
    String mac;
    if (arpProbe(ip, &mac)) {
      pcMAC = mac;
      logMessage("[INFO]", "DISCOVERY", "Encontrado por ARP (/24): " + ip.toString() + "  MAC=" + mac);
      return ip.toString();
    }
    vTaskDelay(pdMS_TO_TICKS(PC_DISCOVERY_STEP_DELAY));
  }

  // 3) Último recurso: 3 IPs aleatórios com ICMP
  for (int k=0; k<3; ++k) {
    if ((millis() - t0) > TMAX) break;
    uint8_t h = 2 + (random(2, 253));
    if (isInfrastructureHost(h) || h == myLast) continue;
    IPAddress ip(base[0], base[1], base[2], h);
    if (Ping.ping(ip, 1)) {
      String mac;
      if (arpProbe(ip, &mac)) {
        pcMAC = mac;
        logMessage("[INFO]", "DISCOVERY", "Encontrado (ICMP+ARP): " + ip.toString() + "  MAC=" + mac);
        return ip.toString();
      }
    }
  }

  return "";
}

// -------------------------------------------------------------------------------------------
//                          Funções de Controle de Relés
// -------------------------------------------------------------------------------------------
void activateRelay(int relayPin, RelayResetState* state, const String &relayName) {
  if (!state->relayActive) {
    digitalWrite(relayPin, HIGH);
    state->startTime    = millis();
    state->lastResetTime= state->startTime;
    state->relayActive  = true;
    state->resetCount++;
    logMessage("[INFO]", "RELAY", relayName + " ativado (Reset #" + String(state->resetCount) + ") - " + String(RESET_DURATION_MS/1000) + "s");
  }
}

void deactivateRelay(int relayPin, RelayResetState* state, const String &relayName) {
  if (state->relayActive) {
    digitalWrite(relayPin, LOW);
    state->relayActive = false;
    logMessage("[INFO]", "RELAY", relayName + " desativado");
  }
}

void resetDeviceNonBlocking(int relayPin, unsigned long delayTime, RelayResetState* state) {
  if (state->relayActive) {
    unsigned long elapsed = millis() - state->startTime;
    if (elapsed >= delayTime) {
      digitalWrite(relayPin, LOW);
      state->relayActive = false;
      const char* dev = (relayPin == RELAY_MODEM_PIN) ? "Modem" : (relayPin == RELAY_PC_PIN) ? "PC" : "Dispositivo";
      logMessage("[INFO]", "RELAY", String(dev) + " religado após " + String(delayTime/1000) + "s (Pino: " + String(relayPin) + ")");
    } else {
      unsigned long remaining = (delayTime - elapsed) / 1000;
      if (remaining % 5 == 0 || remaining <= 3) {
        const char* dev = (relayPin == RELAY_MODEM_PIN) ? "Modem" : "PC";
        logMessage("[DEBUG]", "RELAY", String(dev) + " aguardando religação: " + String(remaining) + "s restantes");
      }
    }
  }
}

// -------------------------------------------------------------------------------------------
//                          Comunicação (com device_id no payload)
// -------------------------------------------------------------------------------------------
String obterTimestamp() {
  timeClient.update();
  time_t rawTime = timeClient.getEpochTime();
  struct tm* timeInfo = gmtime(&rawTime);
  char buffer[25];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", timeInfo);
  return String(buffer);
}

void sendStatusToEndpoint() {
  HTTPClient http;
  http.begin(STATUS_ENDPOINT);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(HTTP_TIMEOUT_MEDIUM);

  String k3Status = digitalRead(RELAY_K3) == HIGH ? "on" : "off";
  String k4Status = digitalRead(RELAY_K4) == HIGH ? "on" : "off";

  // Payload com device_id
  String payload = "{";
  payload += "\"csrf\": \"" + String(millis()) + "\",";
  payload += "\"event\": \"" + String(EVENT_TYPE) + "\",";
  payload += "\"detail\": \"" + String(DETAIL) + "\",";
  payload += "\"timestamp\": \"" + obterTimestamp() + "\",";
  payload += "\"mac_address\": \"" + getPCMacAddress() + "\",";
  if (!deviceID.isEmpty()) {
    payload += "\"device_id\": \"" + deviceID + "\",";  // Incluir ID se disponível
  }
  payload += "\"modem_status\": \"" + String(modemResetOccurred ? "reset" : "ok") + "\",";
  payload += "\"pc_status\": \"" + String(pcResetOccurred ? "reset" : "ok") + "\",";
  payload += "\"k3_status\": \"" + k3Status + "\",";
  payload += "\"k4_status\": \"" + k4Status + "\"}";
  
  logMessage("[DEBUG]", "STATUS", "Enviando payload" + (deviceID.isEmpty() ? " (sem device_id)" : " com device_id=" + deviceID));
  
  int httpResponseCode = http.POST(payload);

  if (httpResponseCode > 0 && httpResponseCode < 400) {
    logMessage("[INFO]", "STATUS", "Status enviado (" + String(httpResponseCode) + ")");
  } else {
    logMessage("[ERROR]", "STATUS", "Falha ao enviar status: " + String(httpResponseCode));
  }
  http.end();
  modemResetOccurred = false;
  pcResetOccurred    = false;
}

void handleCommandsFromEndpoint() {
  HTTPClient http;
  http.begin(COMMAND_SUB_ENDPOINT);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(HTTP_TIMEOUT_SHORT);

  String randomCSRF = String(random(100000, 999999));
  String payload = "{";
  payload += "\"csrf\":\"" + randomCSRF + "\",";
  payload += "\"mac_address\":\"" + getPCMacAddress() + "\"";
  payload += "}";

  int httpResponseCode = http.POST(payload);
  if (httpResponseCode == 202) {
    String response = http.getString();
    bool cmd = false;
    if (response.indexOf("K1=0") >= 0) { externalModemReset = true; cmd = true; }
    if (response.indexOf("K2=0") >= 0) { externalPCReset    = true; cmd = true; }
    if (response.indexOf("K2=1") >= 0) { deactivateRelay(RELAY_PC_PIN, &pcRelayState, "PC"); confirmarRecebimentoComando("K2=1", "Ligar PC"); cmd = true; }
    if (response.indexOf("K3=1") >= 0) { digitalWrite(RELAY_K3, HIGH); confirmarRecebimentoComando("K3=1", "Ativar K3"); cmd = true; }
    if (response.indexOf("K3=0") >= 0) { digitalWrite(RELAY_K3, LOW);  confirmarRecebimentoComando("K3=0", "Desativar K3"); cmd = true; }
    if (response.indexOf("K4=1") >= 0) { digitalWrite(RELAY_K4, HIGH); confirmarRecebimentoComando("K4=1", "Ativar K4"); cmd = true; }
    if (response.indexOf("K4=0") >= 0) { digitalWrite(RELAY_K4, LOW);  confirmarRecebimentoComando("K4=0", "Desativar K4"); cmd = true; }
    if (!cmd) { confirmarRecebimentoComando("desconhecido", "Comando não reconhecido"); }
  } else if (httpResponseCode == 204) {
    // nenhum comando
  } else if (httpResponseCode > 0) {
    logMessage("[WARN]", "COMMANDS", "Resposta inesperada: " + String(httpResponseCode));
  } else {
    logMessage("[ERROR]", "COMMANDS", "Falha HTTP: " + String(httpResponseCode));
  }
  http.end();
}

void confirmarRecebimentoComando(const String &comando, const String &descricao) {
  String randomCSRF = String(random(100000, 999999));
  String confirmUrl = COMMAND_SUB_CONFIR;
  confirmUrl += "?csrf=" + randomCSRF;
  confirmUrl += "&mac_address=" + getPCMacAddress();

  HTTPClient http;
  http.begin(confirmUrl);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(HTTP_TIMEOUT_SHORT);

  String payload = "{";
  payload += "\"csrf\": \"" + String(millis()) + "\",";
  payload += "\"mac_address\":\"" + getPCMacAddress() + "\",";
  payload += "\"comando\":\"" + comando + "\"";
  payload += "}";

  int httpResponseCode = http.POST(payload);
  if (httpResponseCode == 200) {
    String response = http.getString();
    logMessage("[INFO]", "CONFIRM", "Confirmação enviada");
    logMessage("[DEBUG]", "CONFIRM", "Resposta: " + response);
  } else if (httpResponseCode > 0) {
    logMessage("[WARN]", "CONFIRM", "Resposta inesperada: " + String(httpResponseCode));
  } else {
    logMessage("[ERROR]", "CONFIRM", "Falha ao enviar confirmação: " + String(httpResponseCode));
  }
  http.end();
}