#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
#include <Preferences.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <PubSubClient.h>  // MQTT client library
#include <ArduinoJson.h>   // JSON library for Discord and Gotify
#include "esp_camera.h"
#include "esp_sleep.h"
#include "esp_pm.h"

// Mesh configuration - set to false to disable mesh networking
#define ENABLE_MESH_NETWORK true

// WiFi credentials - CHANGE THESE TO YOUR NETWORK
const char* ssid = "Bespin";
const char* password = "letsgoshopping";

// Mesh network access point credentials
const char* mesh_ap_ssid = "ESP32CAM-Mesh";
const char* mesh_ap_password = "mesh123456";

// CodeProject.AI Server settings - These can be configured via web interface
String ai_server_ip = "192.168.1.100";  // Default IP, can be changed via web
int ai_server_port = 32168;             // Default port, can be changed via web
const char* ai_endpoint = "/v1/vision/detection";  // YOLO detection endpoint

// Notification Configuration
struct NotificationConfig {
  // MQTT Settings
  bool mqtt_enabled = false;
  String mqtt_server = "192.168.1.100";
  int mqtt_port = 1883;
  String mqtt_username = "";
  String mqtt_password = "";
  String mqtt_topic = "esp32cam/detection";
  
  // Gotify Settings
  bool gotify_enabled = false;
  String gotify_server = "192.168.1.100";
  int gotify_port = 80;
  String gotify_token = "";
  String gotify_title = "ESP32-CAM Detection";
  
  // Discord Settings
  bool discord_enabled = false;
  String discord_webhook = "";
  String discord_username = "ESP32-CAM";
} notificationConfig;

// Preferences for storing configuration
Preferences preferences;

// Notification clients
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
HTTPClient httpClient;

// Mesh Network Configuration
#define MESH_CHANNEL 1
#define MAX_MESH_NODES 10
#define MESH_SCAN_TIMEOUT 15000  // 15 seconds to find master
#define DISCOVERY_INTERVAL 3000  // 3 seconds between discovery broadcasts
#define HEARTBEAT_INTERVAL 5000  // 5 seconds heartbeat for faster status updates
#define NODE_TIMEOUT 300000      // 5 minutes before node considered offline

// Mesh node types
enum NodeType {
  NODE_UNKNOWN = 0,
  NODE_MASTER = 1,
  NODE_SLAVE = 2,
  NODE_RELAY = 3
};

// Mesh message types
enum MessageType {
  MSG_DISCOVERY = 0,
  MSG_JOIN_REQUEST = 1,
  MSG_JOIN_RESPONSE = 2,
  MSG_CONFIG_UPDATE = 3,
  MSG_DETECTION_ALERT = 4,
  MSG_HEARTBEAT = 5,
  MSG_STATUS_REQUEST = 6,
  MSG_STATUS_RESPONSE = 7,
  MSG_AI_REQUEST = 8,
  MSG_AI_RESPONSE = 9
};

// Mesh message structure
struct MeshMessage {
  uint8_t type;
  uint8_t nodeId;
  uint8_t targetId;
  uint8_t hopCount;
  uint32_t timestamp;
  uint8_t data[200];  // Payload
  uint8_t checksum;
};

// Node information structure
struct NodeInfo {
  uint8_t mac[6];
  uint8_t nodeId;
  NodeType nodeType;
  uint32_t lastSeen;
  int16_t rssi;
  uint8_t hopCount;
  bool isOnline;
  float batteryLevel;
  bool personDetected;
  char ipAddress[16];
  uint32_t uptime;
  uint32_t lastDetectionTime;  // When this node last detected a person
  float lastDetectionConfidence;  // Confidence level of last detection
};

// Mesh network state
NodeType myNodeType = NODE_UNKNOWN;
uint8_t myNodeId = 0;
uint8_t masterNodeId = 0;
uint8_t myMac[6];
NodeInfo meshNodes[MAX_MESH_NODES];
uint8_t nodeCount = 0;
bool meshInitialized = false;
unsigned long lastHeartbeat = 0;
unsigned long lastDiscovery = 0;
unsigned long meshStartTime = 0;

// Mesh message handlers (forward declarations)
void handleDiscoveryMessage(MeshMessage* msg, const uint8_t* mac);
void handleJoinRequest(MeshMessage* msg, const uint8_t* mac);
void handleJoinResponse(MeshMessage* msg, const uint8_t* mac);
void handleConfigUpdate(MeshMessage* msg, const uint8_t* mac);
void handleDetectionAlert(MeshMessage* msg, const uint8_t* mac);
void handleHeartbeat(MeshMessage* msg, const uint8_t* mac);
void handleStatusRequest(MeshMessage* msg, const uint8_t* mac);
void handleStatusResponse(MeshMessage* msg, const uint8_t* mac);
void handleAIRequest(MeshMessage* msg, const uint8_t* mac);
void handleAIResponse(MeshMessage* msg, const uint8_t* mac);
bool sendAIRequestToMaster(camera_fb_t* frame);
void sendStatusToMaster();
bool detectPersonWithAI(camera_fb_t* frame);

// Notification functions (forward declarations)
void initNotifications();
void loadNotificationConfig();
void saveNotificationConfig();
bool sendMQTTNotification(const String& message, const String& nodeId, float confidence);
bool sendGotifyNotification(const String& message, const String& nodeId, float confidence);
bool sendDiscordNotification(const String& message, const String& nodeId, float confidence);
void sendAllNotifications(const String& message, const String& nodeId, float confidence);

// Camera configuration functions (forward declarations)
void configureCameraForAI();
void configureCameraForWeb();

// Web server on port 80
WebServer server(80);

// Power management and detection settings for dual 18650 + 5W solar (Adelaide winter)
#define BATTERY_PIN 33          // ADC pin to read battery voltage
#define SLEEP_TIME_US 30000000  // 30 seconds in microseconds
#define LOW_BATTERY_THRESHOLD 3.3 // Emergency shutdown voltage
#define CRITICAL_BATTERY_THRESHOLD 3.6 // Reduce detection frequency
#define GOOD_BATTERY_THRESHOLD 3.8 // Normal operation voltage
#define DETECTION_INTERVAL 1000   // Base detection interval (adaptive based on battery)

// Performance optimization settings
static int ai_timeout = 8000;                      // Increased timeout for reliability
static int ai_image_quality = 12;                  // Slightly better quality for detection
static framesize_t ai_frame_size = FRAMESIZE_VGA;  // Smaller frame for faster AI (640x480)
static float ai_confidence_threshold = 0.10;       // Very low threshold for maximum sensitivity

// Person detection variables
static bool person_detected = false;
static bool streaming_active = false; // Flag to pause detection during streaming
static unsigned long last_detection = 0;
static unsigned long last_ai_check = 0;
static float detection_confidence = 0.0;

// AI Thinker pin map
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Power management functions
float getBatteryVoltage() {
  int adcValue = analogRead(BATTERY_PIN);
  // Convert ADC reading to voltage (assuming voltage divider)
  float voltage = (adcValue * 3.3) / 4095.0 * 2.0; // Adjust multiplier based on your voltage divider
  return voltage;
}

bool isLowBattery() {
  return getBatteryVoltage() < LOW_BATTERY_THRESHOLD;
}

// Adaptive detection interval based on battery level (optimized for dual 18650 + 5W solar)
unsigned long getAdaptiveDetectionInterval() {
  float voltage = getBatteryVoltage();
  
  if (voltage < CRITICAL_BATTERY_THRESHOLD) {
    // Critical battery: 30-second intervals to preserve power
    Serial.printf("🔋 Critical battery (%.2fV) - Extended detection interval (30s)\n", voltage);
    return 30000;
  } else if (voltage < GOOD_BATTERY_THRESHOLD) {
    // Medium battery: 10-second intervals
    Serial.printf("🔋 Medium battery (%.2fV) - Moderate detection interval (10s)\n", voltage);
    return 10000;
  } else {
    // Good battery: 1-second intervals (current setting)
    return DETECTION_INTERVAL;
  }
}

void enterDeepSleep() {
  Serial.println("Entering deep sleep mode...");
  esp_sleep_enable_timer_wakeup(SLEEP_TIME_US);
  esp_deep_sleep_start();
}

// Configuration management functions
void loadConfiguration() {
  preferences.begin("esp32cam", false);
  
  ai_server_ip = preferences.getString("ai_ip", "192.168.1.100");
  ai_server_port = preferences.getInt("ai_port", 32168);
  
  // Load performance settings with new optimized defaults
  ai_timeout = preferences.getInt("ai_timeout", 8000);
  ai_image_quality = preferences.getInt("ai_quality", 12);
  ai_frame_size = (framesize_t)preferences.getInt("ai_frame_size", (int)FRAMESIZE_VGA);
  ai_confidence_threshold = preferences.getFloat("ai_threshold", 0.10);  // Very low threshold for maximum sensitivity
  
  // Force update to ensure we use the low threshold
  preferences.putFloat("ai_threshold", 0.10);
  ai_confidence_threshold = 0.10;
  
  preferences.end();
  
  Serial.printf("Loaded configuration - AI Server: %s:%d\n", ai_server_ip.c_str(), ai_server_port);
  Serial.printf("Performance settings - Timeout: %dms, Quality: %d, Frame: %d, Threshold: %.3f\n", 
               ai_timeout, ai_image_quality, (int)ai_frame_size, ai_confidence_threshold);
}

void saveConfiguration() {
  preferences.begin("esp32cam", false);
  
  preferences.putString("ai_ip", ai_server_ip);
  preferences.putInt("ai_port", ai_server_port);
  
  preferences.end();
  
  Serial.printf("Saved configuration - AI Server: %s:%d\n", ai_server_ip.c_str(), ai_server_port);
}

// ========== MESH NETWORKING FUNCTIONS ==========

// Calculate simple checksum for message integrity
uint8_t calculateChecksum(const MeshMessage* msg) {
  uint8_t checksum = 0;
  const uint8_t* data = (const uint8_t*)msg;
  for (int i = 0; i < sizeof(MeshMessage) - 1; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

// Find node by MAC address
int findNodeByMAC(const uint8_t* mac) {
  for (int i = 0; i < nodeCount; i++) {
    if (memcmp(meshNodes[i].mac, mac, 6) == 0) {
      return i;
    }
  }
  return -1;
}

// Find node by ID
int findNodeById(uint8_t nodeId) {
  for (int i = 0; i < nodeCount; i++) {
    if (meshNodes[i].nodeId == nodeId) {
      return i;
    }
  }
  return -1;
}

// Add or update node in mesh table
void updateMeshNode(const uint8_t* mac, uint8_t nodeId, NodeType nodeType, int16_t rssi, uint8_t hopCount) {
  int nodeIndex = findNodeByMAC(mac);
  
  if (nodeIndex == -1 && nodeCount < MAX_MESH_NODES) {
    // Add new node
    nodeIndex = nodeCount++;
    memcpy(meshNodes[nodeIndex].mac, mac, 6);
    meshNodes[nodeIndex].nodeId = nodeId;
    meshNodes[nodeIndex].personDetected = false;  // Initialize detection state
    meshNodes[nodeIndex].lastDetectionTime = 0;   // Initialize detection time
    meshNodes[nodeIndex].lastDetectionConfidence = 0.0; // Initialize confidence
    strcpy(meshNodes[nodeIndex].ipAddress, "Unknown"); // Initialize IP
    Serial.printf("📡 Added new mesh node ID:%d, Type:%d\n", nodeId, nodeType);
  }
  
  if (nodeIndex != -1) {
    // Update existing node
    meshNodes[nodeIndex].nodeType = nodeType;
    meshNodes[nodeIndex].lastSeen = millis();
    meshNodes[nodeIndex].rssi = rssi;
    meshNodes[nodeIndex].hopCount = hopCount;
    meshNodes[nodeIndex].isOnline = true;
  }
}

// Send mesh message
bool sendMeshMessage(uint8_t targetId, MessageType msgType, const void* data, size_t dataSize) {
  if (dataSize > sizeof(((MeshMessage*)0)->data)) {
    Serial.println("❌ Message too large");
    return false;
  }
  
  // Check if ESP-NOW is available
  if (!meshInitialized && myNodeType == NODE_UNKNOWN) {
    // Allow discovery messages even if not fully initialized
    if (msgType != MSG_DISCOVERY) {
      Serial.println("⚠️ Mesh not initialized, skipping message");
      return false;
    }
  }
  
  MeshMessage msg;
  memset(&msg, 0, sizeof(msg));
  msg.type = msgType;
  msg.nodeId = myNodeId;
  msg.targetId = targetId;
  msg.hopCount = 0;
  msg.timestamp = millis();
  
  if (data && dataSize > 0) {
    memcpy(msg.data, data, dataSize);
  }
  
  msg.checksum = calculateChecksum(&msg);
  
  // For broadcast messages (targetId = 0), use broadcast address
  uint8_t broadcastAddr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  
  // Broadcast to all peers if target is 0 (broadcast)
  esp_err_t result;
  if (targetId == 0) {
    // Try to add broadcast peer if not already added
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, broadcastAddr, 6);
    peerInfo.channel = WiFi.channel(); // Use current WiFi channel
    peerInfo.encrypt = false;
    
    // Add peer (ignore if already exists)
    esp_now_add_peer(&peerInfo);
    
    result = esp_now_send(broadcastAddr, (uint8_t*)&msg, sizeof(msg));
  } else {
    result = esp_now_send(NULL, (uint8_t*)&msg, sizeof(msg));
  }
  
  if (result == ESP_OK) {
    Serial.printf("📤 Sent mesh message type:%d to node:%d\n", msgType, targetId);
    return true;
  } else {
    Serial.printf("❌ Failed to send mesh message: %s\n", esp_err_to_name(result));
    return false;
  }
}

// Broadcast discovery message
void broadcastDiscovery() {
  Serial.println("🔍 Broadcasting discovery message...");
  
  struct {
    uint8_t nodeType;
    char deviceName[32];
  } discoveryData;
  
  discoveryData.nodeType = myNodeType;
  snprintf(discoveryData.deviceName, sizeof(discoveryData.deviceName), "ESP32CAM-%02X%02X", myMac[4], myMac[5]);
  
  if (sendMeshMessage(0, MSG_DISCOVERY, &discoveryData, sizeof(discoveryData))) {
    Serial.println("✅ Discovery message sent");
  } else {
    Serial.println("❌ Failed to send discovery message");
  }
}

// Handle incoming mesh messages
void onMeshMessageReceived(const uint8_t* mac, const uint8_t* incomingData, int len) {
  Serial.printf("📻 ESP-NOW message received: %d bytes from %02X:%02X:%02X:%02X:%02X:%02X\n", 
                len, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  
  // Safety checks
  if (!mac || !incomingData || len <= 0) {
    Serial.println("❌ Invalid mesh message received");
    return;
  }
  
  if (len != sizeof(MeshMessage)) {
    Serial.printf("❌ Invalid message size: %d (expected %d)\n", len, sizeof(MeshMessage));
    return;
  }
  
  // Check heap before processing message
  size_t free_heap = ESP.getFreeHeap();
  if (free_heap < 10000) {
    Serial.printf("⚠️ Low heap (%d bytes), skipping mesh message processing\n", free_heap);
    return;
  }
  
  // Create a local copy of the message to avoid pointer issues
  MeshMessage msg;
  memcpy(&msg, incomingData, sizeof(MeshMessage));
  
  // Verify checksum
  uint8_t receivedChecksum = msg.checksum;
  msg.checksum = 0;
  uint8_t calculatedChecksum = calculateChecksum(&msg);
  msg.checksum = receivedChecksum;
  
  if (receivedChecksum != calculatedChecksum) {
    Serial.println("❌ Message checksum failed");
    return;
  }
  
  Serial.printf("📥 Received mesh message type:%d from node:%d\n", msg.type, msg.nodeId);
  
  // Update sender in mesh table (simplified for safety)
  int nodeIndex = findNodeByMAC(mac);
  if (nodeIndex == -1 && nodeCount < MAX_MESH_NODES) {
    nodeIndex = nodeCount++;
    memcpy(meshNodes[nodeIndex].mac, mac, 6);
    meshNodes[nodeIndex].nodeId = msg.nodeId;
    Serial.printf("📡 Added new mesh node ID:%d\n", msg.nodeId);
  }
  
  if (nodeIndex != -1) {
    meshNodes[nodeIndex].lastSeen = millis();
    meshNodes[nodeIndex].isOnline = true;
  }
  
  // Process message based on type with additional error checking
  switch (msg.type) {
    case MSG_DISCOVERY:
      handleDiscoveryMessage(&msg, mac);
      break;
      
    case MSG_JOIN_REQUEST:
      if (myNodeType == NODE_MASTER) {
        handleJoinRequest(&msg, mac);
      }
      break;
      
    case MSG_JOIN_RESPONSE:
      if (myNodeType == NODE_UNKNOWN) {
        handleJoinResponse(&msg, mac);
      }
      break;
      
    case MSG_CONFIG_UPDATE:
      handleConfigUpdate(&msg, mac);
      break;
      
    case MSG_DETECTION_ALERT:
      handleDetectionAlert(&msg, mac);
      break;
      
    case MSG_HEARTBEAT:
      handleHeartbeat(&msg, mac);
      break;
      
    case MSG_STATUS_REQUEST:
      handleStatusRequest(&msg, mac);
      break;
      
    case MSG_STATUS_RESPONSE:
      handleStatusResponse(&msg, mac);
      break;
      
    case MSG_AI_REQUEST:
      if (myNodeType == NODE_MASTER) {
        handleAIRequest(&msg, mac);
      }
      break;
      
    case MSG_AI_RESPONSE:
      if (myNodeType == NODE_SLAVE) {
        handleAIResponse(&msg, mac);
      }
      break;
      
    default:
      Serial.printf("❓ Unknown message type: %d\n", msg.type);
      break;
  }
  
  // Forward message if it's not for us and hop count is reasonable
  if (msg.targetId != myNodeId && msg.targetId != 0 && msg.hopCount < 3) {
    // Check heap before forwarding
    if (ESP.getFreeHeap() > 8000) {
      msg.hopCount++;
      esp_now_send(NULL, (uint8_t*)&msg, sizeof(MeshMessage));
      Serial.printf("🔄 Forwarded message to node:%d (hop:%d)\n", msg.targetId, msg.hopCount);
    } else {
      Serial.println("⚠️ Insufficient heap to forward mesh message");
    }
  }
}

// Handle discovery messages
void handleDiscoveryMessage(MeshMessage* msg, const uint8_t* mac) {
  struct {
    uint8_t nodeType;
    char deviceName[32];
  }* discoveryData = (decltype(discoveryData))msg->data;
  
  Serial.printf("📡 Discovery received from %s (type:%d)\n", discoveryData->deviceName, discoveryData->nodeType);
  
  // If we're unknown and receive a master discovery, respond with join request
  if (myNodeType == NODE_UNKNOWN && discoveryData->nodeType == NODE_MASTER) {
    Serial.println("🎯 Found master! Sending join request...");
    
    struct {
      char deviceName[32];
      uint8_t capabilities;
    } joinRequest;
    
    snprintf(joinRequest.deviceName, sizeof(joinRequest.deviceName), "ESP32CAM-%02X%02X", myMac[4], myMac[5]);
    joinRequest.capabilities = 1; // Camera capability
    
    sendMeshMessage(msg->nodeId, MSG_JOIN_REQUEST, &joinRequest, sizeof(joinRequest));
  }
  
  // If we're master and receive unknown node discovery, send join response
  if (myNodeType == NODE_MASTER && discoveryData->nodeType == NODE_UNKNOWN) {
    Serial.printf("🎯 Unknown node discovered, sending join response to %s\n", discoveryData->deviceName);
    
    struct {
      uint8_t assignedNodeId;
      char networkSSID[32];
      char networkPassword[64];
      char aiServerIP[16];
      uint16_t aiServerPort;
    } joinResponse;
    
    joinResponse.assignedNodeId = nodeCount + 2; // Start from 2 (master is 1)
    strncpy(joinResponse.networkSSID, ssid, sizeof(joinResponse.networkSSID));
    strncpy(joinResponse.networkPassword, password, sizeof(joinResponse.networkPassword));
    strncpy(joinResponse.aiServerIP, ai_server_ip.c_str(), sizeof(joinResponse.aiServerIP));
    joinResponse.aiServerPort = ai_server_port;
    
    sendMeshMessage(msg->nodeId, MSG_JOIN_RESPONSE, &joinResponse, sizeof(joinResponse));
    Serial.printf("🎯 Sent join response to node:%d with ID:%d\n", msg->nodeId, joinResponse.assignedNodeId);
  }
}

// Handle join requests
void handleJoinRequest(MeshMessage* msg, const uint8_t* mac) {
  if (myNodeType != NODE_MASTER) {
    return; // Only masters handle join requests
  }
  
  Serial.printf("📥 Received join request from node %d\n", msg->nodeId);
  
  // Extract join request data
  struct {
    uint8_t nodeType;
    char deviceName[32];
  }* joinRequest = (decltype(joinRequest))msg->data;
  
  Serial.printf("📝 Device: %s, Type: %d\n", joinRequest->deviceName, joinRequest->nodeType);
  
  // Add/update node in mesh table
  int nodeIndex = findNodeByMAC(mac);
  if (nodeIndex == -1 && nodeCount < MAX_MESH_NODES) {
    nodeIndex = nodeCount++;
    memcpy(meshNodes[nodeIndex].mac, mac, 6);
  }
  
  if (nodeIndex != -1) {
    // Assign a node ID (master is 1, slaves get 2, 3, 4...)
    uint8_t assignedNodeId = nodeIndex + 2; // Start from 2 for slaves
    meshNodes[nodeIndex].nodeId = assignedNodeId;
    meshNodes[nodeIndex].lastSeen = millis();
    meshNodes[nodeIndex].isOnline = true;
    
    // Send join response with configuration
    struct {
      uint8_t assignedNodeId;
      char networkSSID[32];
      char networkPassword[64];
      char aiServerIP[16];
      uint16_t aiServerPort;
    } joinResponse;
    
    joinResponse.assignedNodeId = assignedNodeId;
    strncpy(joinResponse.networkSSID, mesh_ap_ssid, sizeof(joinResponse.networkSSID));
    strncpy(joinResponse.networkPassword, mesh_ap_password, sizeof(joinResponse.networkPassword));
    strncpy(joinResponse.aiServerIP, ai_server_ip.c_str(), sizeof(joinResponse.aiServerIP));
    joinResponse.aiServerPort = ai_server_port;
    
    if (sendMeshMessage(assignedNodeId, MSG_JOIN_RESPONSE, &joinResponse, sizeof(joinResponse))) {
      Serial.printf("✅ Sent join response to node %d (assigned ID %d)\n", msg->nodeId, assignedNodeId);
    } else {
      Serial.printf("❌ Failed to send join response to node %d\n", msg->nodeId);
    }
  } else {
    Serial.println("❌ Mesh table full - cannot accept more nodes");
  }
}

// Handle join responses
void handleJoinResponse(MeshMessage* msg, const uint8_t* mac) {
  if (myNodeType != NODE_SLAVE) {
    return; // Only slaves handle join responses
  }
  
  struct {
    uint8_t assignedNodeId;
    char networkSSID[32];
    char networkPassword[64];
    char aiServerIP[16];
    uint16_t aiServerPort;
  }* joinData = (decltype(joinData))msg->data;
  
  // Accept configuration from master
  myNodeId = joinData->assignedNodeId;
  masterNodeId = msg->nodeId;
  
  // Update network configuration
  ai_server_ip = String(joinData->aiServerIP);
  ai_server_port = joinData->aiServerPort;
  saveConfiguration();
  
  Serial.printf("✅ Joined mesh as slave node ID:%d, Master ID:%d\n", myNodeId, masterNodeId);
  Serial.printf("📡 Received network config - AI Server: %s:%d\n", ai_server_ip.c_str(), ai_server_port);
  
  meshInitialized = true;
  
  // Send multiple status reports to ensure master gets our details
  Serial.println("📊 Sending initial status reports to master...");
  delay(100);
  sendStatusToMaster();
  delay(500);
  sendStatusToMaster();
  delay(500); 
  sendStatusToMaster();
}

// Handle configuration updates
void handleConfigUpdate(MeshMessage* msg, const uint8_t* mac) {
  // Update configuration from master
  struct {
    char aiServerIP[16];
    uint16_t aiServerPort;
  }* configData = (decltype(configData))msg->data;
  
  ai_server_ip = String(configData->aiServerIP);
  ai_server_port = configData->aiServerPort;
  saveConfiguration();
  
  Serial.printf("⚙️ Updated config from master - AI Server: %s:%d\n", ai_server_ip.c_str(), ai_server_port);
}

// Handle detection alerts
void handleDetectionAlert(MeshMessage* msg, const uint8_t* mac) {
  struct {
    float confidence;
    uint32_t timestamp;
    uint8_t nodeId;
  }* alertData = (decltype(alertData))msg->data;
  
  Serial.printf("🚨 MESH ALERT: Person detected by node:%d (confidence:%.2f)\n", 
                alertData->nodeId, alertData->confidence);
  
  // Send notifications for mesh detection alerts
  if (WiFi.status() == WL_CONNECTED) {
    sendAllNotifications("Person detected by mesh node", String(alertData->nodeId), alertData->confidence);
  }
  
  // Update the alerting node's status
  int nodeIndex = findNodeById(alertData->nodeId);
  if (nodeIndex != -1) {
    meshNodes[nodeIndex].personDetected = true;
    meshNodes[nodeIndex].lastSeen = millis();
    meshNodes[nodeIndex].lastDetectionTime = millis();  // Record when this node detected
    meshNodes[nodeIndex].lastDetectionConfidence = alertData->confidence;  // Record confidence
    
    Serial.printf("📊 Updated node %d: last detection at %lu ms, confidence %.2f\n", 
                  alertData->nodeId, meshNodes[nodeIndex].lastDetectionTime, alertData->confidence);
  }
}

// Handle heartbeat messages
void handleHeartbeat(MeshMessage* msg, const uint8_t* mac) {
  struct {
    float batteryLevel;
    bool personDetected;
    uint32_t uptime;
  }* heartbeatData = (decltype(heartbeatData))msg->data;
  
  // Update node status
  int nodeIndex = findNodeById(msg->nodeId);
  if (nodeIndex != -1) {
    meshNodes[nodeIndex].batteryLevel = heartbeatData->batteryLevel;
    meshNodes[nodeIndex].personDetected = heartbeatData->personDetected;
    meshNodes[nodeIndex].lastSeen = millis();
    meshNodes[nodeIndex].isOnline = true;
  }
}

// Handle status requests
void handleStatusRequest(MeshMessage* msg, const uint8_t* mac) {
  // Send our current status
  struct {
    float batteryLevel;
    bool personDetected;
    uint32_t uptime;
    uint8_t nodeCount;
  } statusData;
  
  statusData.batteryLevel = getBatteryVoltage();
  statusData.personDetected = person_detected;
  statusData.uptime = millis() / 1000;
  statusData.nodeCount = nodeCount;
  
  sendMeshMessage(msg->nodeId, MSG_STATUS_RESPONSE, &statusData, sizeof(statusData));
}

// Handle status responses
void handleStatusResponse(MeshMessage* msg, const uint8_t* mac) {
  Serial.printf("📊 Received status response from node %d\n", msg->nodeId);
  
  // Process status information from other nodes
  struct {
    char ipAddress[16];
    float batteryVoltage;
    int8_t rssi;
    uint32_t uptime;
    bool personDetected;
  }* statusData = (decltype(statusData))msg->data;
  
  Serial.printf("📊 Status data: IP=%s, Battery=%.2fV, RSSI=%ddBm, Uptime=%ds\n",
                statusData->ipAddress, statusData->batteryVoltage, statusData->rssi, statusData->uptime);
  
  // Find the node in our mesh table
  int nodeIndex = findNodeByMAC(mac);
  if (nodeIndex != -1) {
    Serial.printf("📊 Updating node index %d with status data\n", nodeIndex);
    // Update node information with received status
    strncpy(meshNodes[nodeIndex].ipAddress, statusData->ipAddress, sizeof(meshNodes[nodeIndex].ipAddress));
    meshNodes[nodeIndex].batteryLevel = statusData->batteryVoltage;
    meshNodes[nodeIndex].rssi = statusData->rssi;
    meshNodes[nodeIndex].uptime = statusData->uptime;
    meshNodes[nodeIndex].personDetected = statusData->personDetected;
    meshNodes[nodeIndex].lastSeen = millis();
    meshNodes[nodeIndex].isOnline = true;
    
    Serial.printf("✅ Updated node %d status: IP=%s, Battery=%.2fV, RSSI=%ddBm\n", 
                  msg->nodeId, statusData->ipAddress, statusData->batteryVoltage, statusData->rssi);
  } else {
    Serial.printf("❌ Could not find node with MAC in mesh table\n");
  }
}

// Handle AI detection requests from slave nodes (master only)
void handleAIRequest(MeshMessage* msg, const uint8_t* mac) {
  Serial.printf("📤 Received AI request from node %d\n", msg->nodeId);
  
  // Extract metadata from message
  struct {
    uint32_t imageSize;
    uint16_t width;
    uint16_t height;
    uint8_t requestId;
  }* aiRequest = (decltype(aiRequest))msg->data;
  
  Serial.printf("🔍 AI request metadata: %dx%d, %d bytes, ID=%d\n", 
                aiRequest->width, aiRequest->height, aiRequest->imageSize, aiRequest->requestId);
  
  // For now, capture our own frame for AI processing since we can't transfer full images via ESP-NOW
  // In a full implementation, you'd use HTTP endpoints to transfer the actual image data
  camera_fb_t* frame = esp_camera_fb_get();
  bool personDetected = false;
  float confidence = 0.0f;
  
  if (frame) {
    Serial.printf("🔍 Master processing local frame for slave request: %dx%d, %d bytes\n", 
                  frame->width, frame->height, frame->len);
                  
    // Process with our local AI detection
    personDetected = detectPersonWithAI(frame);
    confidence = personDetected ? 0.85f : 0.0f;
    
    esp_camera_fb_return(frame);
    
    Serial.printf("🔍 Master AI result for slave node %d: person=%s, confidence=%.2f\n", 
                  msg->nodeId, personDetected ? "YES" : "NO", confidence);
  } else {
    Serial.println("❌ Master failed to capture frame for slave AI request");
  }
  
  // Send response back to requesting node
  struct {
    bool personDetected;
    float confidence;
    uint8_t requestId;
  } aiResponse;
  
  aiResponse.personDetected = personDetected;
  aiResponse.confidence = confidence;
  aiResponse.requestId = aiRequest->requestId;
  
  if (sendMeshMessage(msg->nodeId, MSG_AI_RESPONSE, &aiResponse, sizeof(aiResponse))) {
    Serial.printf("✅ Sent AI response to node %d: person=%s, confidence=%.2f\n", 
                  msg->nodeId, personDetected ? "YES" : "NO", confidence);
  } else {
    Serial.printf("❌ Failed to send AI response to node %d\n", msg->nodeId);
  }
}

// Handle AI detection responses from master (slave only)
void handleAIResponse(MeshMessage* msg, const uint8_t* mac) {
  struct {
    bool personDetected;
    float confidence;
    uint8_t requestId;
  }* aiResponse = (decltype(aiResponse))msg->data;
  
  Serial.printf("📥 Received AI response ID=%d: person=%s, confidence=%.2f\n", 
                aiResponse->requestId, aiResponse->personDetected ? "YES" : "NO", aiResponse->confidence);
  
  // Update local detection status
  person_detected = aiResponse->personDetected;
  last_detection = millis();
  
  if (aiResponse->personDetected) {
    Serial.printf("🎯 Person detected via mesh AI! (confidence: %.2f)\n", 
                  aiResponse->confidence);
    
    // Send notifications for AI detection responses (slave node detected person)
    if (WiFi.status() == WL_CONNECTED) {
      sendAllNotifications("Person detected via mesh AI", String(myNodeId), aiResponse->confidence);
    }
  }
  
  if (person_detected) {
    Serial.printf("🚨 Person detected via mesh AI! Confidence: %.2f\n", aiResponse->confidence);
  }
}

// Send AI request to master (slave only)
bool sendAIRequestToMaster(camera_fb_t* frame) {
  if (myNodeType != NODE_SLAVE || !meshInitialized) {
    return false;
  }
  
  if (!frame || frame->len == 0) {
    Serial.println("❌ No frame to send for AI processing");
    return false;
  }
  
  Serial.printf("📤 Sending AI request to master: %dx%d, %d bytes\n", 
                frame->width, frame->height, frame->len);
  
  // Capture a lower quality frame specifically for mesh forwarding
  sensor_t* s = esp_camera_sensor_get();
  if (s == NULL) {
    Serial.println("❌ Failed to get camera sensor");
    return false;
  }
  
  // Save current settings
  int originalQuality = s->status.quality;
  framesize_t originalFramesize = s->status.framesize;
  
  // Use lower quality for mesh transfer
  s->set_quality(s, 15);  // Higher quality value = more compression
  s->set_framesize(s, FRAMESIZE_SVGA); // 800x600 instead of UXGA
  
  // Capture compressed frame
  camera_fb_t* meshFrame = esp_camera_fb_get();
  
  // Restore original settings
  s->set_quality(s, originalQuality);
  s->set_framesize(s, originalFramesize);
  
  if (!meshFrame) {
    Serial.println("❌ Failed to capture mesh frame");
    return false;
  }
  
  Serial.printf("📤 Mesh frame: %dx%d, %d bytes (compressed from %d bytes)\n", 
                meshFrame->width, meshFrame->height, meshFrame->len, frame->len);
  
  // Send compressed frame metadata to signal the master to process it
  struct {
    uint32_t imageSize;
    uint16_t width;
    uint16_t height;
    uint8_t requestId;
  } aiRequest;
  
  aiRequest.imageSize = meshFrame->len;
  aiRequest.width = meshFrame->width;
  aiRequest.height = meshFrame->height;
  aiRequest.requestId = (millis() / 1000) % 255; // Simple request ID
  
  // Store the compressed frame temporarily for the master to access via HTTP
  // This is a simplified approach - in production, you'd implement proper chunking
  
  bool success = sendMeshMessage(masterNodeId, MSG_AI_REQUEST, &aiRequest, sizeof(aiRequest));
  
  esp_camera_fb_return(meshFrame);
  
  if (success) {
    Serial.println("✅ AI request metadata sent to master");
  } else {
    Serial.println("❌ Failed to send AI request to master");
  }
  
  return success;
}

// Initialize mesh networking
bool initMeshNetwork() {
  Serial.println("🔧 Starting mesh network initialization...");
  
  // Initialize WiFi for ESP-NOW (maintain existing connection)
  bool wasConnected = (WiFi.status() == WL_CONNECTED);
  
  // First, try to scan for existing mesh network
  WiFi.mode(WIFI_STA);
  delay(100);
  
  Serial.println("🔍 Scanning for existing mesh network...");
  int n = WiFi.scanNetworks();
  bool masterFound = false;
  
  for (int i = 0; i < n; ++i) {
    if (WiFi.SSID(i) == String(mesh_ap_ssid)) {
      Serial.printf("📡 Found mesh master AP: %s (Channel %d, RSSI %d)\n", 
                    mesh_ap_ssid, WiFi.channel(i), WiFi.RSSI(i));
      masterFound = true;
      
      // Connect to mesh master as slave
      WiFi.mode(WIFI_STA);
      WiFi.begin(mesh_ap_ssid, mesh_ap_password);
      
      Serial.print("Connecting to mesh master");
      int attempts = 0;
      while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
      }
      
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.printf("✅ Connected to mesh master! IP: %s\n", WiFi.localIP().toString().c_str());
        myNodeType = NODE_SLAVE;
        Serial.printf("🔧 Set myNodeType to NODE_SLAVE (%d)\n", myNodeType);
        
        // Don't break here - continue with ESP-NOW setup and join request
        Serial.println("📡 Setting up ESP-NOW and requesting to join mesh...");
        masterFound = true;
        break;
      } else {
        Serial.println("\n❌ Failed to connect to mesh master");
        masterFound = false;
      }
    }
  }
  
  // If no master found, become the master
  if (!masterFound) {
    Serial.println("🏆 No mesh master found, becoming master...");
    
    // First, connect to home network (required for CodeProject.AI access)
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    Serial.print("Connecting to home network for internet access");
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println();
      Serial.printf("✅ Connected to home network! IP: %s\n", WiFi.localIP().toString().c_str());
      
      // Get home network channel for mesh AP consistency
      int homeChannel = WiFi.channel();
      Serial.printf("🔗 Home network on channel %d, using same for mesh AP\n", homeChannel);
      
      // Now set up AP+STA mode to also create mesh access point
      WiFi.mode(WIFI_AP_STA);
      delay(100);
      
      // Reconnect to home network in AP+STA mode
      WiFi.begin(ssid, password);
      delay(1000);
      
      // Create access point for mesh on same channel as home network
      bool ap_result = WiFi.softAP(mesh_ap_ssid, mesh_ap_password, homeChannel, 0, 4);
      if (!ap_result) {
        Serial.println("❌ Failed to create mesh access point");
        return false;
      }
      
      Serial.printf("✅ Mesh AP created: %s on channel %d\n", mesh_ap_ssid, homeChannel);
      Serial.printf("📱 Mesh AP IP: %s\n", WiFi.softAPIP().toString().c_str());
      Serial.printf("🌐 Home network IP: %s (for CodeProject.AI access)\n", WiFi.localIP().toString().c_str());
      
    } else {
      Serial.println("\n❌ Failed to connect to home network!");
      Serial.println("⚠️ Master cannot access CodeProject.AI server without home network");
      
      // Still create mesh AP for local mesh functionality
      WiFi.mode(WIFI_AP);
      bool ap_result = WiFi.softAP(mesh_ap_ssid, mesh_ap_password, 1, 0, 4);
      if (!ap_result) {
        Serial.println("❌ Failed to create mesh access point");
        return false;
      }
      
      Serial.printf("⚠️ Mesh AP created (no internet): %s on channel 1\n", mesh_ap_ssid);
      Serial.printf("📱 Mesh AP IP: %s\n", WiFi.softAPIP().toString().c_str());
    }
    
    myNodeType = NODE_MASTER;
    myNodeId = 1;
    masterNodeId = 1;
  }
  
  int currentChannel = WiFi.channel();
  Serial.printf("📡 WiFi channel: %d\n", currentChannel);
  
  // Get MAC address
  WiFi.macAddress(myMac);
  
  // Get ESP32 unique chip ID for cases where MAC addresses might be duplicated
  uint64_t chipId = ESP.getEfuseMac();
  
  // If MAC address ends with common clone values, modify it using chip ID
  // This handles ESP32 boards that may have duplicate MAC addresses
  if ((myMac[4] == 0x07 && myMac[5] == 0x1C) || 
      (myMac[0] == 0xC8 && myMac[1] == 0x2E && myMac[2] == 0x18)) {
    // Use parts of the unique chip ID to make MAC unique
    myMac[4] = (chipId >> 8) & 0xFF;
    myMac[5] = chipId & 0xFF;
    Serial.printf("🔧 Modified MAC using chip ID: %016llX\n", chipId);
  }
  
  Serial.printf("📱 Device MAC: %02X:%02X:%02X:%02X:%02X:%02X (Chip ID: %016llX)\n",
                myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5], chipId);
  
  // Initialize mesh variables (preserve myNodeType if already set as slave)
  Serial.printf("🔧 Preserving myNodeType=%d during ESP-NOW init\n", myNodeType);
  myNodeId = 0;
  masterNodeId = 0;
  nodeCount = 0;
  meshInitialized = false;
  lastHeartbeat = 0;
  lastDiscovery = 0;
  meshStartTime = millis();
  
  // Clear mesh nodes array
  memset(meshNodes, 0, sizeof(meshNodes));
  
  // Initialize ESP-NOW
  esp_err_t initResult = esp_now_init();
  if (initResult != ESP_OK) {
    Serial.printf("❌ ESP-NOW initialization failed: %s\n", esp_err_to_name(initResult));
    return false;
  }
  
  Serial.println("✅ ESP-NOW initialized successfully");
  
  // Register callback for received messages
  esp_err_t cbResult = esp_now_register_recv_cb(onMeshMessageReceived);
  if (cbResult != ESP_OK) {
    Serial.printf("❌ Failed to register ESP-NOW callback: %s\n", esp_err_to_name(cbResult));
    return false;
  }
  
  Serial.println("✅ ESP-NOW callback registered");
  
  // Add broadcast peer for mesh discovery
  esp_now_peer_info_t broadcastPeer;
  memset(&broadcastPeer, 0, sizeof(broadcastPeer));
  memset(broadcastPeer.peer_addr, 0xFF, 6); // Broadcast address
  broadcastPeer.channel = WiFi.channel();   // Use current WiFi channel
  broadcastPeer.encrypt = false;
  
  esp_err_t addResult = esp_now_add_peer(&broadcastPeer);
  if (addResult != ESP_OK) {
    Serial.printf("❌ Failed to add broadcast peer: %s\n", esp_err_to_name(addResult));
  } else {
    Serial.printf("✅ Broadcast peer added on channel %d\n", broadcastPeer.channel);
  }
  
  // Start mesh discovery process
  Serial.println("🔍 Starting mesh discovery...");
  
  // Debug: Print current node type
  Serial.printf("🔍 Current node type: %d (NODE_SLAVE=%d, NODE_MASTER=%d)\n", myNodeType, NODE_SLAVE, NODE_MASTER);
  
  // If we're a slave node (connected to mesh AP), send join request immediately
  if (myNodeType == NODE_SLAVE) {
    Serial.println("📤 Slave node: sending join request to master...");
    delay(1000); // Give master time to be ready
    
    struct {
      uint8_t nodeType;
      char deviceName[32];
    } joinRequest;
    
    joinRequest.nodeType = NODE_SLAVE;
    snprintf(joinRequest.deviceName, sizeof(joinRequest.deviceName), "ESP32CAM-%.8X", (uint32_t)(ESP.getEfuseMac() & 0xFFFFFFFF));
    
    // Send join request to master (node ID 1)
    if (sendMeshMessage(1, MSG_JOIN_REQUEST, &joinRequest, sizeof(joinRequest))) {
      Serial.println("✅ Join request sent to master");
      
      // Wait for join response with timeout
      Serial.println("⏳ Waiting for join response from master...");
      unsigned long joinWaitStart = millis();
      bool joinResponseReceived = false;
      
      while (!meshInitialized && (millis() - joinWaitStart) < 10000) { // 10 second timeout
        delay(100);
        if (meshInitialized) {
          joinResponseReceived = true;
          break;
        }
      }
      
      if (joinResponseReceived) {
        Serial.println("✅ Successfully joined mesh network as slave");
      } else {
        Serial.println("⚠️ Join response timeout - continuing anyway");
        meshInitialized = true; // Continue as slave even without explicit confirmation
      }
    } else {
      Serial.println("❌ Failed to send join request");
      meshInitialized = true; // Continue anyway
    }
  } else {
    Serial.printf("🔍 Not a slave node - myNodeType=%d, skipping join request\n", myNodeType);
  }
  
  // Small delay before first broadcast
  delay(500);
  
  return true;
}

// Check if we should become master
void checkBecomeMaster() {
  if (myNodeType == NODE_UNKNOWN && (millis() - meshStartTime) > MESH_SCAN_TIMEOUT) {
    // No master found, become master
    myNodeType = NODE_MASTER;
    myNodeId = 1; // Master is always node ID 1
    masterNodeId = myNodeId;
    meshInitialized = true;
    
    Serial.println("👑 Becoming mesh master - no existing master found");
    Serial.printf("📡 Master node ID:%d ready for connections\n", myNodeId);
    
    // Save our role
    preferences.begin("esp32cam", false);
    preferences.putUChar("nodeType", NODE_MASTER);
    preferences.putUChar("nodeId", myNodeId);
    preferences.end();
  }
}

// Send heartbeat to mesh
void sendMeshHeartbeat() {
  if (!meshInitialized || (millis() - lastHeartbeat) < HEARTBEAT_INTERVAL) {
    return;
  }
  
  if (myNodeType == NODE_SLAVE) {
    // Slaves send status updates to master instead of broadcast heartbeat
    sendStatusToMaster();
  } else {
    // Masters send heartbeat broadcasts
    struct {
      float batteryLevel;
      bool personDetected;
      uint32_t uptime;
    } heartbeatData;
    
    heartbeatData.batteryLevel = getBatteryVoltage();
    heartbeatData.personDetected = person_detected;
    heartbeatData.uptime = millis() / 1000;
    
    sendMeshMessage(0, MSG_HEARTBEAT, &heartbeatData, sizeof(heartbeatData));
  }
  
  lastHeartbeat = millis();
}

// Send detection alert to mesh
void sendDetectionAlert() {
  if (!meshInitialized || WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️ Mesh not available for detection alert");
    return;
  }
  
  struct {
    float confidence;
    uint32_t timestamp;
    uint8_t nodeId;
  } alertData;
  
  alertData.confidence = detection_confidence;
  alertData.timestamp = millis();
  alertData.nodeId = myNodeId;
  
  if (sendMeshMessage(0, MSG_DETECTION_ALERT, &alertData, sizeof(alertData))) {
    Serial.println("📡 Sent detection alert to mesh network");
  } else {
    Serial.println("❌ Failed to send detection alert");
  }
}

// Send status to master (used by slaves to report their details)
void sendStatusToMaster() {
  if (myNodeType != NODE_SLAVE || !meshInitialized) {
    Serial.printf("🚫 Status send blocked - NodeType: %d, MeshInit: %d\n", myNodeType, meshInitialized);
    return;
  }
  
  struct {
    char ipAddress[16];
    float batteryVoltage;
    int8_t rssi;
    uint32_t uptime;
    bool personDetected;
  } statusData;
  
  // Get IP address
  strncpy(statusData.ipAddress, WiFi.localIP().toString().c_str(), sizeof(statusData.ipAddress));
  
  // Get battery voltage
  statusData.batteryVoltage = getBatteryVoltage();
  
  // Get WiFi signal strength
  statusData.rssi = WiFi.RSSI();
  
  // Get uptime in seconds
  statusData.uptime = millis() / 1000;
  
  // Current person detection status
  statusData.personDetected = person_detected;
  
  Serial.printf("📊 Preparing status: IP=%s, Battery=%.2fV, RSSI=%ddBm, MasterID=%d\n", 
                statusData.ipAddress, statusData.batteryVoltage, statusData.rssi, masterNodeId);
  
  if (sendMeshMessage(masterNodeId, MSG_STATUS_RESPONSE, &statusData, sizeof(statusData))) {
    Serial.printf("✅ Sent status to master ID %d: IP=%s, Battery=%.2fV, RSSI=%ddBm\n", 
                  masterNodeId, statusData.ipAddress, statusData.batteryVoltage, statusData.rssi);
  } else {
    Serial.printf("❌ Failed to send status to master ID %d\n", masterNodeId);
  }
}

// Update mesh network configuration
void updateMeshConfig() {
  if (myNodeType != NODE_MASTER || !meshInitialized) return;
  
  struct {
    char aiServerIP[16];
    uint16_t aiServerPort;
  } configData;
  
  strncpy(configData.aiServerIP, ai_server_ip.c_str(), sizeof(configData.aiServerIP));
  configData.aiServerPort = ai_server_port;
  
  sendMeshMessage(0, MSG_CONFIG_UPDATE, &configData, sizeof(configData));
  Serial.println("⚙️ Broadcast configuration update to mesh");
}

// Clean up offline nodes
void cleanupMeshNodes() {
  for (int i = 0; i < nodeCount; i++) {
    if (meshNodes[i].isOnline && (millis() - meshNodes[i].lastSeen) > NODE_TIMEOUT) {
      meshNodes[i].isOnline = false;
      Serial.printf("📴 Node %d marked offline (timeout)\n", meshNodes[i].nodeId);
    }
  }
}

// Send image to CodeProject.AI server for person detection
bool detectPersonWithAI(camera_fb_t* frame) {
  if (!frame || frame->len == 0) {
    Serial.println("❌ No frame buffer for AI detection");
    return false;
  }
  
  // If we're a slave node, forward AI request to master
  if (myNodeType == NODE_SLAVE && meshInitialized) {
    Serial.println("📤 Slave node: forwarding AI request to master");
    return sendAIRequestToMaster(frame);
  }
  
  // Master node or standalone: process AI request locally
  Serial.println("🔍 Master node: processing AI request locally");
  
  // Check available heap before proceeding - more realistic calculation
  size_t free_heap = ESP.getFreeHeap();
  size_t min_heap_needed = frame->len + 10000; // Frame size + reasonable overhead for multipart formatting
  
  if (free_heap < min_heap_needed) {
    Serial.printf("❌ Insufficient heap: %d bytes free, %d needed\n", free_heap, min_heap_needed);
    return false;
  }
  
  unsigned long start_time = millis();
  float brightness_ratio = 0.0;  // Declare at function scope
  Serial.printf("🔍 AI Detection: Frame %dx%d, %d bytes (Free heap: %d)\n", 
                frame->width, frame->height, frame->len, free_heap);
  
  // Enhanced debugging - analyze frame characteristics
  if (frame->len < 5000) {
    Serial.printf("⚠️ Very small image (%d bytes) - likely very dark scene or corrupted\n", frame->len);
    Serial.println("💡 Try adding more lighting or moving closer to light source");
  } else if (frame->len > 200000) {
    Serial.printf("⚠️ Very large image (%d bytes) - may be too detailed for optimal AI processing\n", frame->len);
  } else {
    Serial.printf("✅ Good image size (%d bytes) for AI processing\n", frame->len);
  }
  
  // Check if we have sufficient contrast (basic heuristic)
  if (frame->format == PIXFORMAT_JPEG) {
    // Sample first few KB to guess image brightness
    int bright_pixels = 0, total_sampled = 0;
    int sample_size = (frame->len > 1000) ? 1000 : frame->len;
    for (int i = 0; i < sample_size && i < frame->len; i += 50) {
      if (frame->buf[i] > 128) bright_pixels++; // Simple brightness check
      total_sampled++;
    }
    brightness_ratio = (float)bright_pixels / total_sampled;  // Assign to function-scoped variable
    Serial.printf("🔆 Estimated image brightness: %.1f%% (sampled %d points)\n", 
                  brightness_ratio * 100, total_sampled);
    
    if (brightness_ratio < 0.1) {
      Serial.println("⚠️ Image appears very dark - person detection may be poor");
      Serial.println("💡 Consider adding lighting or checking camera position");
    } else if (brightness_ratio > 0.9) {
      Serial.println("⚠️ Image appears overexposed - may affect detection");
    }
  }
  
  HTTPClient http;
  WiFiClient client;
  
  String url = "http://" + ai_server_ip + ":" + String(ai_server_port) + String(ai_endpoint);
  
  if (!http.begin(client, url)) {
    Serial.println("❌ Failed to begin HTTP connection");
    return false;
  }
  
  // Set optimized timeouts for faster response
  http.setTimeout(ai_timeout);
  http.setConnectTimeout(2000); // 2 second connect timeout
  
  // Try the format that AgentDVR likely uses - standard multipart form
  String boundary = "----WebKitFormBoundary7MA4YWxkTrZu0gW";
  
  // Build the multipart headers
  String header_part = "--" + boundary + "\r\n";
  header_part += "Content-Disposition: form-data; name=\"file\"; filename=\"image.jpg\"\r\n";
  header_part += "Content-Type: image/jpeg\r\n\r\n";
  
  String footer_part = "\r\n--" + boundary + "--\r\n";
  
  // Calculate total size 
  size_t total_size = header_part.length() + frame->len + footer_part.length();
  
  Serial.printf("📤 Preparing AgentDVR format: %d bytes total\n", total_size);
  Serial.printf("🔧 Using boundary: %s\n", boundary.c_str());
  Serial.printf("🔧 Field name: 'file' (standard multipart upload)\n");
  Serial.printf("🧠 Free heap: %d bytes\n", ESP.getFreeHeap());
  
  // Check if we have enough memory - be more reasonable with limits
  const size_t max_safe_size = 150000; // 150KB safety limit - allow reasonable image sizes
  if (total_size > max_safe_size || ESP.getFreeHeap() < (total_size + 20000)) {
    Serial.printf("⚠️ Image too large (%d bytes) or low memory (heap: %d), reducing quality\n", 
                  total_size, ESP.getFreeHeap());
    
    // Try to capture a smaller image
    sensor_t* s = esp_camera_sensor_get();
    if (s) {
      s->set_quality(s, 15); // Lower quality = smaller file
      esp_camera_fb_return(frame);
      frame = esp_camera_fb_get();
      if (!frame) {
        Serial.println("❌ Failed to capture smaller frame");
        http.end();
        return false;
      }
      Serial.printf("🔄 Retrying with smaller frame: %d bytes\n", frame->len);
      total_size = header_part.length() + frame->len + footer_part.length();
    }
  }
  
  // Set HTTP headers - use standard multipart form-data
  String contentType = "multipart/form-data; boundary=" + boundary;
  http.addHeader("Content-Type", contentType);
  
  // Allocate buffer - with extra safety checks
  uint8_t* payload = (uint8_t*)malloc(total_size);
  if (!payload) {
    Serial.printf("❌ Cannot allocate %d bytes (free heap: %d)\n", total_size, ESP.getFreeHeap());
    http.end();
    return false;
  }
  
  // Build the complete payload
  size_t pos = 0;
  memcpy(payload + pos, header_part.c_str(), header_part.length());
  pos += header_part.length();
  
  memcpy(payload + pos, frame->buf, frame->len);
  pos += frame->len;
  
  memcpy(payload + pos, footer_part.c_str(), footer_part.length());
  pos += footer_part.length();
  
  Serial.printf("✅ Built %d byte payload successfully\n", pos);
  
  // Send the POST request
  int httpCode = http.POST(payload, pos);
  free(payload);
  
  if (httpCode <= 0) {
    Serial.printf("❌ HTTP request failed: %s\n", http.errorToString(httpCode).c_str());
    http.end();
    return false;
  }
  
  if (httpCode != 200) {
    Serial.printf("❌ AI server error: HTTP %d\n", httpCode);
    String error_response = http.getString();
    Serial.printf("❌ Error response: %s\n", error_response.c_str());
    http.end();
    return false;
  }
  
  String response = http.getString();
  http.end();
  
  unsigned long end_time = millis();
  Serial.printf("⚡ AI request completed in %d ms\n", end_time - start_time);
  Serial.printf("📥 AI Response: %s\n", response.c_str());
  
  // Enhanced debugging - analyze the complete response
  if (response.length() > 0) {
    // Check if any objects were detected at all (even non-person)
    int count_start = response.indexOf("\"count\":");
    if (count_start != -1) {
      int count_end = response.indexOf(",", count_start);
      if (count_end == -1) count_end = response.indexOf("}", count_start);
      if (count_end != -1) {
        String count_str = response.substring(count_start + 8, count_end);
        int total_objects = count_str.toInt();
        Serial.printf("🔍 AI detected %d total objects\n", total_objects);
        
        // If no objects detected, provide specific guidance
        if (total_objects == 0) {
          Serial.println("🔍 AI detected 0 objects - Troubleshooting:");
          Serial.printf("   • Brightness: %.1f%% (ideal: 20-80%%)\n", brightness_ratio * 100);
          Serial.printf("   • Image size: %d bytes (ideal: 50-200KB)\n", frame->len);
          Serial.printf("   • Resolution: %dx%d\n", frame->width, frame->height);
          Serial.println("   • Try: Better lighting, move closer, check camera focus");
        } else {
          Serial.printf("🔍 Processing %d detected objects for person detection...\n", total_objects);
        }
      }
    }
    if (count_start >= 0) {
      count_start = response.indexOf(":", count_start) + 1;
      int count_end = response.indexOf(",", count_start);
      if (count_end == -1) count_end = response.indexOf("}", count_start);
      if (count_end > count_start) {
        String count_str = response.substring(count_start, count_end);
        count_str.trim();
        int object_count = count_str.toInt();
        
        if (object_count > 0) {
          Serial.printf("🔍 AI detected %d objects total - analyzing for person...\n", object_count);
          
          // Extract all predictions for detailed analysis
          int pred_start = response.indexOf("\"predictions\":[");
          if (pred_start >= 0) {
            int pred_end = response.indexOf("]", pred_start);
            String predictions = response.substring(pred_start + 15, pred_end);
            Serial.printf("🔬 Predictions details: %s\n", predictions.c_str());
          }
        } else {
          Serial.println("🔍 AI detected 0 objects - scene may be too simple or lighting inadequate");
        }
      }
    }
  }
  
  // Reset confidence for this detection
  detection_confidence = 0.0;
  
  // Parse JSON response for person detection with multiple confidence field names
  bool person_found = false;
  
  // Check for different ways person might be labeled - be more permissive
  if (response.indexOf("\"label\":\"person\"") >= 0 || 
      response.indexOf("\"name\":\"person\"") >= 0 ||
      response.indexOf("\"class\":\"person\"") >= 0 ||
      response.indexOf("\"tag\":\"person\"") >= 0 ||
      response.indexOf("\"object\":\"person\"") >= 0) {
    person_found = true;
    Serial.println("🎯 Found 'person' label in response");
  }
  
  // Also check for human-related labels
  if (!person_found) {
    if (response.indexOf("\"label\":\"human\"") >= 0 || 
        response.indexOf("\"name\":\"human\"") >= 0 ||
        response.indexOf("\"class\":\"human\"") >= 0 ||
        response.indexOf("\"label\":\"man\"") >= 0 ||
        response.indexOf("\"label\":\"woman\"") >= 0 ||
        response.indexOf("\"label\":\"people\"") >= 0) {
      person_found = true;
      Serial.println("🎯 Found human-related label in response");
    }
  }
  
  // NEW: Also log ALL detected objects for debugging
  int predictions_start = response.indexOf("\"predictions\":[");
  if (predictions_start >= 0) {
    int predictions_end = response.indexOf("]", predictions_start);
    if (predictions_end > predictions_start) {
      String predictions = response.substring(predictions_start + 15, predictions_end);
      if (predictions.length() > 2) { // More than just "{}"
        Serial.printf("🔍 ALL DETECTED OBJECTS: %s\n", predictions.c_str());
        
        // Parse each prediction to show individual objects
        int obj_start = 0;
        int obj_count = 0;
        while ((obj_start = predictions.indexOf("{", obj_start)) >= 0) {
          int obj_end = predictions.indexOf("}", obj_start);
          if (obj_end > obj_start) {
            String obj = predictions.substring(obj_start, obj_end + 1);
            obj_count++;
            Serial.printf("   Object %d: %s\n", obj_count, obj.c_str());
            
            // Check if this object is person-related
            if (obj.indexOf("person") >= 0 || obj.indexOf("human") >= 0 || 
                obj.indexOf("man") >= 0 || obj.indexOf("woman") >= 0) {
              Serial.println("   ^^^ This is a person-related object! ^^^");
              person_found = true;
            }
          }
          obj_start = obj_end + 1;
        }
      }
    }
  }
  
  if (person_found) {
    // Look for confidence values with various field names
    String confStr = "";
    int confStart = -1;
    
    // Try different confidence field names
    const char* conf_fields[] = {"confidence", "score", "prob", "probability", "certainty"};
    for (int i = 0; i < 5 && confStart == -1; i++) {
      String field = "\"" + String(conf_fields[i]) + "\":";
      confStart = response.indexOf(field);
    }
    
    if (confStart > -1) {
      confStart = response.indexOf(":", confStart) + 1;
      int endIndex = response.indexOf(",", confStart);
      if (endIndex == -1) endIndex = response.indexOf("}", confStart);
      if (endIndex > confStart) {
        confStr = response.substring(confStart, endIndex);
        confStr.trim();
        confStr.replace("\"", ""); // Remove quotes if present
        if (confStr.length() > 0) {
          detection_confidence = confStr.toFloat();
        }
      }
    } else {
      // If no explicit confidence found, assume high confidence for detected object
      detection_confidence = 0.8;
      Serial.println("⚠️ No confidence value found, assuming 0.8");
    }
    
    Serial.printf("🔍 Person detected with confidence: %.3f (threshold: %.3f)\n", 
                 detection_confidence, ai_confidence_threshold);
    
    if (detection_confidence >= ai_confidence_threshold) {
      Serial.printf("✅ PERSON DETECTED by AI! Confidence: %.3f >= %.3f\n", 
                   detection_confidence, ai_confidence_threshold);
      return true;
    } else {
      Serial.printf("⚠️ Person confidence too low: %.3f < %.3f\n", 
                   detection_confidence, ai_confidence_threshold);
    }
  } else {
    Serial.println("ℹ️ No person/human detected in AI response");
    
    // Check if there were any objects detected at all for debugging
    if (response.indexOf("\"count\":0") >= 0 || response.indexOf("\"predictions\":[]") >= 0) {
      Serial.println("🔍 No objects detected at all - consider adjusting lighting or camera angle");
    } else {
      Serial.println("🔍 Other objects detected but no person - AI is working");
    }
  }
  
  return false;
}

// Check for person using AI detection (adaptive timing based on battery level)
void checkForPerson() {
  unsigned long adaptive_interval = getAdaptiveDetectionInterval();
  
  if (millis() - last_ai_check > adaptive_interval) {
    // Skip detection if streaming is active to avoid camera conflicts
    if (streaming_active) {
      Serial.println("🔄 Skipping detection - streaming is active");
      last_ai_check = millis(); // Update timestamp to maintain timing
      return;
    }
    
    // Check heap health before attempting detection
    size_t free_heap = ESP.getFreeHeap();
    size_t largest_block = ESP.getMaxAllocHeap();
    
    // Only proceed if we have sufficient heap
    if (free_heap < 50000 || largest_block < 40000) {
      Serial.printf("⚠️ Low memory, skipping AI detection (Free: %d, Max block: %d)\n", free_heap, largest_block);
      last_ai_check = millis(); // Update timestamp to avoid rapid retries
      return;
    }
    
    // Reset timer IMMEDIATELY for parallel processing - enables faster detection cycles
    last_ai_check = millis();
    
    Serial.println("🔍 Starting AI detection with parallel timing");
    
    // TEST: Use current camera settings instead of switching to AI mode
    // configureCameraForAI();
    // delay(100); // Let camera adjust to new settings
    
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Failed to get frame for AI detection");
      configureCameraForWeb(); // Restore camera settings
      return;
    }
    
    Serial.printf("Checking for person with AI (frame: %dx%d, %d bytes)...\n", 
                  fb->width, fb->height, fb->len);
    
    if (detectPersonWithAI(fb)) {
      person_detected = true;
      last_detection = millis();
      Serial.println("🚨 PERSON DETECTED by AI!");
      
      // Send notifications for direct AI detection
      if (WiFi.status() == WL_CONNECTED) {
        sendAllNotifications("Person detected directly", String(myNodeId), 0.85f);  // Use default confidence for direct detection
      }
      
#if ENABLE_MESH_NETWORK
      // Send alert to mesh network
      sendDetectionAlert();
#endif
    } else {
      Serial.println("No person detected by AI");
    }
    
    esp_camera_fb_return(fb);
    
    // TEST: Don't restore camera settings to see if that's the issue
    // configureCameraForWeb();
    
    // Give system time to clean up after AI processing
    delay(100);
    
    // Log heap status after detection
    Serial.printf("Heap after AI detection - Free: %d, Max block: %d\n", 
                 ESP.getFreeHeap(), ESP.getMaxAllocHeap());
  }
}

// Configure camera for optimal AI detection speed
void configureCameraForAI() {
  sensor_t* s = esp_camera_sensor_get();
  if (s != NULL) {
    // Set smaller frame size for faster AI detection
    s->set_framesize(s, ai_frame_size);
    s->set_quality(s, ai_image_quality);
    
    // Enhanced AI-optimized settings for better person detection
    s->set_brightness(s, 0);     // 0 = neutral brightness
    s->set_contrast(s, 2);       // Higher contrast for better edge detection
    s->set_saturation(s, 0);     // Neutral saturation
    s->set_special_effect(s, 0); // No special effects
    s->set_whitebal(s, 1);       // Enable auto white balance
    s->set_awb_gain(s, 1);       // Enable AWB gain
    s->set_wb_mode(s, 0);        // Auto white balance mode
    s->set_exposure_ctrl(s, 1);  // Enable auto exposure
    s->set_aec2(s, 0);           // Disable AEC2 for better control
    s->set_ae_level(s, 0);       // Neutral auto exposure level
    s->set_gain_ctrl(s, 1);      // Enable auto gain
    s->set_agc_gain(s, 0);       // Auto gain
    s->set_gainceiling(s, (gainceiling_t)6); // Much higher gain ceiling for low light
    
    // Additional optimizations for person detection
    s->set_sharpness(s, 1);      // Slightly sharpen edges for better features
    s->set_denoise(s, 0);        // Disable denoising to preserve detail
    s->set_bpc(s, 1);            // Enable bad pixel correction
    s->set_wpc(s, 1);            // Enable white pixel correction
    s->set_lenc(s, 1);           // Enable lens correction
    
    Serial.printf("📷 Camera configured for AI: %dx%d, quality=%d (enhanced for detection)\n", 
                 ai_frame_size == FRAMESIZE_VGA ? 640 : (ai_frame_size == FRAMESIZE_SVGA ? 800 : 1024),
                 ai_frame_size == FRAMESIZE_VGA ? 480 : (ai_frame_size == FRAMESIZE_SVGA ? 600 : 768),
                 ai_image_quality);
  }
}

// Restore camera to high quality for web interface
void configureCameraForWeb() {
  sensor_t* s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_framesize(s, psramFound() ? FRAMESIZE_UXGA : FRAMESIZE_SVGA);
    s->set_quality(s, psramFound() ? 10 : 12);
    Serial.println("📷 Camera restored to high quality for web interface");
  }
}

// Camera configuration
static bool configureCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_SVGA;  // 800x600 - good balance
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // Initialize with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_VGA;  // Safer for no PSRAM
    config.jpeg_quality = 15;
    config.fb_count = 1;
  }

  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return false;
  }

  // Set camera settings
  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_brightness(s, 0);     // -2 to 2
    s->set_contrast(s, 0);       // -2 to 2
    s->set_saturation(s, 0);     // -2 to 2
    s->set_special_effect(s, 0); // 0 to 6 (0-No Effect, 1-Negative, 2-Grayscale, 3-Red Tint, 4-Green Tint, 5-Blue Tint, 6-Sepia)
    s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
    s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
    s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
    s->set_aec2(s, 0);           // 0 = disable , 1 = enable
    s->set_ae_level(s, 0);       // -2 to 2
    s->set_aec_value(s, 300);    // 0 to 1200
    s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
    s->set_agc_gain(s, 0);       // 0 to 30
    s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
    s->set_bpc(s, 0);            // 0 = disable , 1 = enable
    s->set_wpc(s, 1);            // 0 = disable , 1 = enable
    s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
    s->set_lenc(s, 1);           // 0 = disable , 1 = enable
    s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
    s->set_vflip(s, 0);          // 0 = disable , 1 = enable
    s->set_dcw(s, 1);            // 0 = disable , 1 = enable
    s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
  }

  return true;
}

// HTML page for the webcam
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>ESP32-CAM Solar Person Detection</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { 
      font-family: Arial, sans-serif; 
      text-align: center; 
      margin: 0; 
      padding: 20px;
      background-color: #f0f0f0;
    }
    .container {
      max-width: 900px;
      margin: 0 auto;
      background-color: white;
      padding: 20px;
      border-radius: 10px;
      box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
    }
    h1 { 
      color: #333;
      margin-bottom: 30px;
    }
    h2 {
      color: #555;
      margin-top: 30px;
      margin-bottom: 15px;
      text-align: left;
    }
    img { 
      max-width: 100%; 
      height: auto;
      border: 2px solid #ddd;
      border-radius: 8px;
      margin: 20px 0;
    }
    .controls {
      margin: 20px 0;
    }
    .status {
      display: flex;
      justify-content: space-around;
      margin: 20px 0;
      padding: 15px;
      background-color: #f8f9fa;
      border-radius: 8px;
    }
    .status-item {
      text-align: center;
    }
    .status-value {
      font-size: 1.5em;
      font-weight: bold;
      margin-top: 5px;
    }
    .detected {
      color: #dc3545;
    }
    .normal {
      color: #28a745;
    }
    .info-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
      gap: 20px;
      margin: 20px 0;
    }
    .info-card {
      background-color: #f8f9fa;
      padding: 15px;
      border-radius: 8px;
      text-align: left;
    }
    .config-section {
      background-color: #fff3cd;
      padding: 20px;
      border-radius: 8px;
      margin: 20px 0;
      text-align: left;
    }
    button {
      background-color: #4CAF50;
      color: white;
      padding: 10px 20px;
      border: none;
      border-radius: 4px;
      cursor: pointer;
      margin: 5px;
      font-size: 16px;
    }
    button:hover {
      background-color: #45a049;
    }
    .btn-warning {
      background-color: #ffc107;
      color: #212529;
    }
    .btn-warning:hover {
      background-color: #e0a800;
    }
    .btn-success {
      background-color: #28a745;
      color: white;
    }
    .btn-success:hover {
      background-color: #218838;
    }
    .btn-info {
      background-color: #17a2b8;
      color: white;
    }
    .btn-info:hover {
      background-color: #138496;
    }
    /* Mesh Dashboard Styles */
    .mesh-controls {
      display: flex;
      align-items: center;
      gap: 15px;
      margin-bottom: 20px;
      padding: 15px;
      background-color: #e9ecef;
      border-radius: 8px;
    }
    .mesh-stats {
      margin-bottom: 25px;
    }
    .stat-card {
      background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
      color: white;
      padding: 20px;
      border-radius: 12px;
      text-align: center;
    }
    .stat-card h3 {
      margin-top: 0;
      color: white;
    }
    .nodes-grid {
      display: flex;
      flex-direction: row;
      flex-wrap: wrap;
      gap: 20px;
      justify-content: flex-start;
      align-items: stretch;
    }
    .node-card {
      background-color: white;
      border: 2px solid #dee2e6;
      border-radius: 12px;
      padding: 20px;
      box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
      transition: transform 0.2s, box-shadow 0.2s;
      flex: 0 0 auto;
      min-width: 300px;
      max-width: 400px;
    }
    .node-card:hover {
      transform: translateY(-5px);
      box-shadow: 0 8px 15px rgba(0, 0, 0, 0.2);
    }
    .node-card.online {
      border-color: #28a745;
      background: linear-gradient(145deg, #f8fff9, #ffffff);
    }
    .node-card.offline {
      border-color: #dc3545;
      background: linear-gradient(145deg, #fff8f8, #ffffff);
      opacity: 0.8;
    }
    .node-header {
      display: flex;
      justify-content: between;
      align-items: center;
      margin-bottom: 15px;
    }
    .node-id {
      font-size: 1.4em;
      font-weight: bold;
      color: #333;
    }
    .node-status {
      display: inline-block;
      padding: 4px 12px;
      border-radius: 20px;
      font-size: 0.8em;
      font-weight: bold;
      text-transform: uppercase;
    }
    .status-online {
      background-color: #28a745;
      color: white;
    }
    .status-offline {
      background-color: #dc3545;
      color: white;
    }
    .node-details {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      font-size: 0.9em;
    }
    .detail-item {
      display: flex;
      justify-content: space-between;
      padding: 5px 0;
      border-bottom: 1px solid #eee;
    }
    .detail-label {
      font-weight: bold;
      color: #555;
    }
    .detail-value {
      color: #333;
    }
    .battery-indicator {
      display: inline-block;
      width: 20px;
      height: 10px;
      background-color: #ddd;
      border-radius: 2px;
      position: relative;
      margin-left: 5px;
    }
    .battery-level {
      height: 100%;
      border-radius: 2px;
      transition: width 0.3s;
    }
    .battery-good { background-color: #28a745; }
    .battery-medium { background-color: #ffc107; }
    .battery-low { background-color: #dc3545; }
    .detection-indicator {
      display: inline-block;
      width: 12px;
      height: 12px;
      border-radius: 50%;
      margin-left: 5px;
    }
    .detection-active {
      background-color: #dc3545;
      animation: pulse 1s infinite;
    }
    .detection-inactive {
      background-color: #6c757d;
    }
    @keyframes pulse {
      0% { opacity: 1; }
      50% { opacity: 0.5; }
      100% { opacity: 1; }
    }
    input[type="text"], input[type="number"] {
      width: 200px;
      padding: 8px;
      margin: 5px;
      border: 1px solid #ddd;
      border-radius: 4px;
    }
    .form-group {
      margin: 10px 0;
    }
    label {
      display: inline-block;
      width: 120px;
      text-align: right;
      margin-right: 10px;
    }
    .success {
      color: #28a745;
      font-weight: bold;
    }
    .error {
      color: #dc3545;
      font-weight: bold;
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>ESP32-CAM Solar Person Detection</h1>
    
    <h2>System Information</h2>
    <div class="info-grid">
      <div class="info-card">
        <h3>WiFi Connection</h3>
        <div><strong>Network:</strong> <span id="wifiSSID">Loading...</span></div>
        <div><strong>IP Address:</strong> <span id="wifiIP">Loading...</span></div>
        <div><strong>Signal Strength:</strong> <span id="wifiRSSI">Loading...</span></div>
        <div><strong>MAC Address:</strong> <span id="wifiMAC">Loading...</span></div>
      </div>
      <div class="info-card">
        <h3>AI Server Status</h3>
        <div><strong>Server:</strong> <span id="aiServer">Loading...</span></div>
        <div><strong>Endpoint:</strong> <span id="aiEndpoint">Loading...</span></div>
        <div><strong>Last Check:</strong> <span id="lastAICheck">Loading...</span></div>
        <div><strong>Status:</strong> <span id="aiStatus">Unknown</span></div>
      </div>
      <div class="info-card">
        <h3>Mesh Network</h3>
        <div><strong>Node Type:</strong> <span id="nodeType">Loading...</span></div>
        <div><strong>Node ID:</strong> <span id="nodeId">Loading...</span></div>
        <div><strong>Master ID:</strong> <span id="masterId">Loading...</span></div>
        <div><strong>Connected Nodes:</strong> <span id="connectedNodes">Loading...</span></div>
      </div>
    </div>
    
    <div class="config-section" id="meshDashboard" style="display: none;">
      <h2>🕸️ Mesh Network Dashboard</h2>
      <div class="mesh-controls">
        <button onclick="refreshMeshNodes()" class="btn-success">🔄 Refresh Nodes</button>
        <button onclick="toggleAutoRefresh()" id="autoRefreshBtn" class="btn-info">🔁 Auto-Refresh ON</button>
        <span id="lastMeshUpdate">Last updated: Never</span>
      </div>
      
      <div id="meshStats" class="mesh-stats">
        <div class="stat-card">
          <h3>Network Overview</h3>
          <div><strong>Total Nodes:</strong> <span id="totalNodes">0</span></div>
          <div><strong>Online Nodes:</strong> <span id="onlineNodes">0</span></div>
          <div><strong>Avg Battery:</strong> <span id="avgBattery">0.00V</span></div>
          <div><strong>Detection Active:</strong> <span id="activeDetections">0</span></div>
        </div>
      </div>
      
      <div id="nodesList" class="nodes-grid">
        <!-- Dynamically populated with mesh nodes -->
      </div>
    </div>
    
    <div class="config-section">
      <h2>AI Server Configuration</h2>
      <p>Configure your CodeProject.AI server details below:</p>
      
      <div class="form-group">
        <label for="aiIP">Server IP:</label>
        <input type="text" id="aiIP" placeholder="192.168.1.100" value="">
      </div>
      
      <div class="form-group">
        <label for="aiPort">Server Port:</label>
        <input type="number" id="aiPort" placeholder="32168" value="" min="1" max="65535">
      </div>
      
      <div class="form-group">
        <button onclick="saveAIConfig()" class="btn-warning">Save Configuration</button>
        <button onclick="testAIConnection()" class="btn-success">Test AI Connection</button>
        <button onclick="testAIDetection()" class="btn-info">Test AI Detection</button>
        <button onclick="broadcastMeshConfig()" id="meshConfigBtn" style="display:none;">Broadcast to Mesh</button>
      </div>
      
      <div id="aiTestStatus" style="margin-top: 10px;"></div>
      <div id="configStatus" style="margin-top: 10px;"></div>
    </div>
    
    <div class="config-section">
      <h2>📱 Notification Configuration</h2>
      <p>Configure MQTT, Gotify, and Discord notifications for person detection alerts:</p>
      
      <!-- MQTT Configuration -->
      <div style="background-color: #f8f9fa; padding: 15px; margin: 10px 0; border-radius: 8px;">
        <h3>🔗 MQTT Settings</h3>
        <div class="form-group">
          <label for="mqttServer">MQTT Server:</label>
          <input type="text" id="mqttServer" placeholder="192.168.1.100" value="">
        </div>
        <div class="form-group">
          <label for="mqttPort">MQTT Port:</label>
          <input type="number" id="mqttPort" placeholder="1883" value="" min="1" max="65535">
        </div>
        <div class="form-group">
          <label for="mqttUsername">Username:</label>
          <input type="text" id="mqttUsername" placeholder="Optional" value="">
        </div>
        <div class="form-group">
          <label for="mqttPassword">Password:</label>
          <input type="password" id="mqttPassword" placeholder="Optional" value="">
        </div>
        <div class="form-group">
          <label for="mqttTopic">Topic:</label>
          <input type="text" id="mqttTopic" placeholder="esp32cam/detection" value="">
        </div>
      </div>
      
      <!-- Gotify Configuration -->
      <div style="background-color: #fff3cd; padding: 15px; margin: 10px 0; border-radius: 8px;">
        <h3>🔔 Gotify Settings</h3>
        <div class="form-group">
          <label for="gotifyServer">Gotify Server:</label>
          <input type="text" id="gotifyServer" placeholder="http://192.168.1.100:8080" value="">
        </div>
        <div class="form-group">
          <label for="gotifyToken">App Token:</label>
          <input type="password" id="gotifyToken" placeholder="Your Gotify app token" value="">
        </div>
      </div>
      
      <!-- Discord Configuration -->
      <div style="background-color: #e7f3ff; padding: 15px; margin: 10px 0; border-radius: 8px;">
        <h3>💬 Discord Settings</h3>
        <div class="form-group">
          <label for="discordWebhook">Webhook URL:</label>
          <input type="password" id="discordWebhook" placeholder="Discord webhook URL" value="" style="width: 300px;">
        </div>
        <div class="form-group">
          <label for="discordUsername">Bot Username:</label>
          <input type="text" id="discordUsername" placeholder="ESP32-CAM" value="">
        </div>
      </div>
      
      <div class="form-group">
        <button onclick="saveNotificationConfig()" class="btn-warning">💾 Save Notification Settings</button>
        <button onclick="testNotifications()" class="btn-success">🧪 Test All Notifications</button>
        <button onclick="loadNotificationConfig()" class="btn-info">🔄 Reload Settings</button>
      </div>
      
      <div id="notificationStatus" style="margin-top: 10px;"></div>
    </div>
    
    <div class="config-section" id="meshControls" style="display:none;">
      <h2>Mesh Network Controls</h2>
      <p>Manage your mesh network of ESP32-CAM devices:</p>
      
      <div class="form-group">
        <button onclick="scanMeshNodes()">Scan Network</button>
        <button onclick="resetMeshNetwork()" class="btn-warning">Reset Mesh</button>
        <button onclick="broadcastDiscovery()">Broadcast Discovery</button>
      </div>
      
      <div id="meshStatus" style="margin-top: 10px;"></div>
    </div>
    
    <p><a href="/stream" target="_blank">Open Live Stream</a> (Warning: May cause lockups)</p>
  </div>

  <script>
    function refreshSystemInfo() {
      fetch('/sysinfo')
        .then(response => response.json())
        .then(data => {
          document.getElementById('wifiSSID').textContent = data.wifi_ssid;
          document.getElementById('wifiIP').textContent = data.wifi_ip;
          document.getElementById('wifiRSSI').textContent = data.wifi_rssi + ' dBm';
          document.getElementById('wifiMAC').textContent = data.wifi_mac;
          document.getElementById('aiServer').textContent = data.ai_server;
          document.getElementById('aiEndpoint').textContent = data.ai_endpoint;
          document.getElementById('lastAICheck').textContent = data.last_ai_check;
          document.getElementById('aiStatus').textContent = data.ai_status;
          
          // Mesh network information
          document.getElementById('nodeType').textContent = data.node_type || 'Unknown';
          document.getElementById('nodeId').textContent = data.node_id || 'N/A';
          document.getElementById('masterId').textContent = data.master_id || 'N/A';
          document.getElementById('connectedNodes').textContent = data.connected_nodes || '0';
          
          // Show mesh controls if we're master
          if (data.node_type === 'Master') {
            document.getElementById('meshControls').style.display = 'block';
            document.getElementById('meshConfigBtn').style.display = 'inline-block';
          }
          
          // Show mesh dashboard if we have connected nodes or we're part of a mesh
          if (data.connected_nodes > 0 || data.node_type !== 'Unknown') {
            document.getElementById('meshDashboard').style.display = 'block';
            refreshMeshNodes(); // Load mesh nodes when dashboard becomes visible
          }
          
          // Update form fields
          document.getElementById('aiIP').value = data.ai_ip;
          document.getElementById('aiPort').value = data.ai_port;
        })
        .catch(error => {
          console.error('System info error:', error);
        });
    }
    
    function refreshMeshNodes() {
      fetch('/mesh-nodes')
        .then(response => response.json())
        .then(data => {
          const nodesList = document.getElementById('nodesList');
          nodesList.innerHTML = '';
          
          if (data.nodes && data.nodes.length > 0) {
            data.nodes.forEach(node => {
              const nodeDiv = document.createElement('div');
              nodeDiv.className = 'info-card';
              nodeDiv.innerHTML = `
                <h4>Node ${node.id} (${node.type})</h4>
                <div><strong>Status:</strong> ${node.online ? 'Online' : 'Offline'}</div>
                <div><strong>Battery:</strong> ${node.battery}V</div>
                <div><strong>RSSI:</strong> ${node.rssi} dBm</div>
                <div><strong>Last Seen:</strong> ${node.lastSeen}</div>
                <div><strong>Detection:</strong> ${node.personDetected ? 'ALERT' : 'Normal'}</div>
              `;
              nodesList.appendChild(nodeDiv);
            });
          } else {
            nodesList.innerHTML = '<p>No mesh nodes connected</p>';
          }
        })
        .catch(error => {
          console.error('Mesh nodes error:', error);
        });
    }
    
    function saveAIConfig() {
      const ip = document.getElementById('aiIP').value;
      const port = document.getElementById('aiPort').value;
      
      if (!ip || !port) {
        document.getElementById('configStatus').innerHTML = '<span class="error">Please fill in both IP and Port</span>';
        return;
      }
      
      const data = {
        ai_ip: ip,
        ai_port: parseInt(port)
      };
      
      fetch('/config', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify(data)
      })
      .then(response => response.json())
      .then(result => {
        if (result.success) {
          document.getElementById('configStatus').innerHTML = '<span class="success">Configuration saved successfully!</span>';
          refreshSystemInfo();
        } else {
          document.getElementById('configStatus').innerHTML = '<span class="error">Failed to save configuration</span>';
        }
      })
      .catch(error => {
        console.error('Config save error:', error);
        document.getElementById('configStatus').innerHTML = '<span class="error">Error saving configuration</span>';
      });
    }
    
    function testAIConnection() {
      document.getElementById('aiTestStatus').innerHTML = 'Testing AI server connection...';
      
      fetch('/test-ai')
        .then(response => response.json())
        .then(result => {
          if (result.success) {
            document.getElementById('aiTestStatus').innerHTML = '<span class="success">✅ AI server connection successful! HTTP ' + result.http_code + '</span>';
          } else {
            document.getElementById('aiTestStatus').innerHTML = '<span class="error">❌ AI server connection failed: ' + result.error + '</span>';
          }
        })
        .catch(error => {
        console.error('AI test error:', error);
        document.getElementById('aiTestStatus').innerHTML = '<span class="error">❌ Error testing AI connection</span>';
      });
    }
    
    // Notification Configuration Functions
    function loadNotificationConfig() {
      fetch('/config-notifications')
        .then(response => response.json())
        .then(data => {
          // Load MQTT settings
          document.getElementById('mqttServer').value = data.mqtt_server || '';
          document.getElementById('mqttPort').value = data.mqtt_port || '1883';
          document.getElementById('mqttUsername').value = data.mqtt_username || '';
          document.getElementById('mqttPassword').value = data.mqtt_password || '';
          document.getElementById('mqttTopic').value = data.mqtt_topic || 'esp32cam/detection';
          
          // Load Gotify settings
          document.getElementById('gotifyServer').value = data.gotify_server || '';
          document.getElementById('gotifyToken').value = data.gotify_token || '';
          
          // Load Discord settings
          document.getElementById('discordWebhook').value = data.discord_webhook || '';
          document.getElementById('discordUsername').value = data.discord_username || 'ESP32-CAM';
          
          document.getElementById('notificationStatus').innerHTML = '<span class="success">✅ Notification settings loaded</span>';
        })
        .catch(error => {
          console.error('Load notification config error:', error);
          document.getElementById('notificationStatus').innerHTML = '<span class="error">❌ Failed to load notification settings</span>';
        });
    }
    
    function saveNotificationConfig() {
      const config = {
        mqtt_server: document.getElementById('mqttServer').value,
        mqtt_port: parseInt(document.getElementById('mqttPort').value) || 1883,
        mqtt_username: document.getElementById('mqttUsername').value,
        mqtt_password: document.getElementById('mqttPassword').value,
        mqtt_topic: document.getElementById('mqttTopic').value || 'esp32cam/detection',
        gotify_server: document.getElementById('gotifyServer').value,
        gotify_token: document.getElementById('gotifyToken').value,
        discord_webhook: document.getElementById('discordWebhook').value,
        discord_username: document.getElementById('discordUsername').value || 'ESP32-CAM'
      };
      
      document.getElementById('notificationStatus').innerHTML = '💾 Saving notification settings...';
      
      fetch('/config-notifications', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify(config)
      })
      .then(response => response.json())
      .then(result => {
        if (result.success) {
          document.getElementById('notificationStatus').innerHTML = '<span class="success">✅ Notification settings saved successfully!</span>';
        } else {
          document.getElementById('notificationStatus').innerHTML = '<span class="error">❌ Failed to save: ' + (result.message || 'Unknown error') + '</span>';
        }
      })
      .catch(error => {
        console.error('Save notification config error:', error);
        document.getElementById('notificationStatus').innerHTML = '<span class="error">❌ Error saving notification settings</span>';
      });
    }
    
    function testNotifications() {
      document.getElementById('notificationStatus').innerHTML = '🧪 Testing all notification services...';
      
      // This would trigger a test detection event that sends to all configured services
      fetch('/test-detection')
        .then(response => response.json())
        .then(result => {
          if (result.person_detected) {
            document.getElementById('notificationStatus').innerHTML = '<span class="success">✅ Test notifications sent! Check your MQTT, Gotify, and Discord for alerts.</span>';
          } else {
            document.getElementById('notificationStatus').innerHTML = '<span class="error">❌ Test detection failed: ' + (result.message || 'No person detected') + '</span>';
          }
        })
        .catch(error => {
          console.error('Test notifications error:', error);
          document.getElementById('notificationStatus').innerHTML = '<span class="error">❌ Error testing notifications</span>';
        });
    }    function testAIDetection() {
      document.getElementById('aiTestStatus').innerHTML = '🔍 Testing AI detection with current camera frame...';
      
      fetch('/test-detection')
        .then(response => response.json())
        .then(result => {
          if (result.success) {
            const status = result.person_detected ? '✅ Person detected!' : '❌ No person detected';
            const confidence = result.confidence ? ' (confidence: ' + result.confidence.toFixed(2) + ')' : '';
            document.getElementById('aiTestStatus').innerHTML = '<span class="' + (result.person_detected ? 'success' : 'warning') + '">' + status + confidence + '</span>';
          } else {
            document.getElementById('aiTestStatus').innerHTML = '<span class="error">❌ AI detection test failed: ' + result.error + '</span>';
          }
        })
        .catch(error => {
          console.error('AI detection test error:', error);
          document.getElementById('aiTestStatus').innerHTML = '<span class="error">❌ Error testing AI detection</span>';
        });
    }
    
    function broadcastMeshConfig() {
      fetch('/mesh-broadcast-config', { method: 'POST' })
        .then(response => response.json())
        .then(result => {
          if (result.success) {
            document.getElementById('meshStatus').innerHTML = '<span class="success">Configuration broadcast to mesh network!</span>';
          } else {
            document.getElementById('meshStatus').innerHTML = '<span class="error">Failed to broadcast configuration</span>';
          }
        })
        .catch(error => {
          console.error('Mesh broadcast error:', error);
          document.getElementById('meshStatus').innerHTML = '<span class="error">Error broadcasting to mesh</span>';
        });
    }
    
    function scanMeshNodes() {
      document.getElementById('meshStatus').innerHTML = 'Scanning mesh network...';
      
      fetch('/mesh-scan', { method: 'POST' })
        .then(response => response.json())
        .then(result => {
          if (result.success) {
            document.getElementById('meshStatus').innerHTML = '<span class="success">Mesh scan completed!</span>';
            refreshMeshNodes();
          } else {
            document.getElementById('meshStatus').innerHTML = '<span class="error">Mesh scan failed</span>';
          }
        })
        .catch(error => {
          console.error('Mesh scan error:', error);
          document.getElementById('meshStatus').innerHTML = '<span class="error">Error scanning mesh</span>';
        });
    }
    
    function resetMeshNetwork() {
      if (confirm('Are you sure you want to reset the mesh network? This will disconnect all nodes.')) {
        fetch('/mesh-reset', { method: 'POST' })
          .then(response => response.json())
          .then(result => {
            if (result.success) {
              document.getElementById('meshStatus').innerHTML = '<span class="success">Mesh network reset!</span>';
              setTimeout(() => location.reload(), 2000);
            } else {
              document.getElementById('meshStatus').innerHTML = '<span class="error">Mesh reset failed</span>';
            }
          })
          .catch(error => {
            console.error('Mesh reset error:', error);
            document.getElementById('meshStatus').innerHTML = '<span class="error">Error resetting mesh</span>';
          });
      }
    }
    
    function broadcastDiscovery() {
      fetch('/mesh-discovery', { method: 'POST' })
        .then(response => response.json())
        .then(result => {
          if (result.success) {
            document.getElementById('meshStatus').innerHTML = '<span class="success">Discovery broadcast sent!</span>';
          } else {
            document.getElementById('meshStatus').innerHTML = '<span class="error">Discovery broadcast failed</span>';
          }
        })
        .catch(error => {
          console.error('Discovery broadcast error:', error);
          document.getElementById('meshStatus').innerHTML = '<span class="error">Error broadcasting discovery</span>';
        });
    }
    
    // Mesh Dashboard Functions
    let meshAutoRefresh = true;
    let meshRefreshInterval;
    
    function refreshMeshNodes() {
      fetch('/mesh-nodes')
        .then(response => response.json())
        .then(data => {
          updateMeshDashboard(data);
          document.getElementById('lastMeshUpdate').textContent = 'Last updated: ' + new Date().toLocaleTimeString();
        })
        .catch(error => {
          console.error('Mesh nodes error:', error);
          document.getElementById('nodesList').innerHTML = '<div style="color: red;">❌ Failed to load mesh nodes</div>';
        });
    }
    
    function updateMeshDashboard(data) {
      const nodes = data.nodes;
      
      // Update statistics
      const totalNodes = nodes.length;
      const onlineNodes = nodes.filter(node => node.online).length;
      const avgBattery = totalNodes > 0 ? 
        (nodes.reduce((sum, node) => sum + parseFloat(node.battery), 0) / totalNodes).toFixed(2) : 
        '0.00';
      const activeDetections = nodes.filter(node => node.personDetected).length;
      
      document.getElementById('totalNodes').textContent = totalNodes;
      document.getElementById('onlineNodes').textContent = onlineNodes;
      document.getElementById('avgBattery').textContent = avgBattery + 'V';
      
      // Enhanced detection display
      const detectionSpan = document.getElementById('activeDetections');
      if (activeDetections > 0) {
        const detectingNodes = nodes.filter(node => node.personDetected);
        const nodeList = detectingNodes.map(node => `Node ${node.id} (${node.type})`).join(', ');
        detectionSpan.innerHTML = `<span style="color: #ff6b35; font-weight: bold;">${activeDetections} 🚨</span><br><small style="color: #ff6b35;">${nodeList}</small>`;
      } else {
        detectionSpan.textContent = activeDetections;
      }
      
      // Update nodes list
      const nodesList = document.getElementById('nodesList');
      nodesList.innerHTML = '';
      
      nodes.forEach(node => {
        const nodeCard = createNodeCard(node);
        nodesList.appendChild(nodeCard);
      });
      
      // Show mesh dashboard if we have mesh nodes
      if (totalNodes > 0) {
        document.getElementById('meshDashboard').style.display = 'block';
      }
    }
    
    function createNodeCard(node) {
      const card = document.createElement('div');
      card.className = `node-card ${node.online ? 'online' : 'offline'}`;
      
      const battery = parseFloat(node.battery);
      const batteryClass = battery > 3.8 ? 'battery-good' : battery > 3.6 ? 'battery-medium' : 'battery-low';
      const batteryWidth = Math.max(0, Math.min(100, ((battery - 3.0) / 1.2) * 100));
      
      card.innerHTML = `
        <div class="node-header">
          <div class="node-id">
            📟 Node ${node.id}
            ${node.isSelf ? ' (Self)' : ''}
          </div>
          <div class="node-status status-${node.online ? 'online' : 'offline'}">
            ${node.online ? 'Online' : 'Offline'}
          </div>
        </div>
        
        <div class="node-details">
          <div class="detail-item">
            <span class="detail-label">Type:</span>
            <span class="detail-value">${node.type}</span>
          </div>
          <div class="detail-item">
            <span class="detail-label">Connection:</span>
            <span class="detail-value">${node.connectionType}</span>
          </div>
          <div class="detail-item">
            <span class="detail-label">IP Address:</span>
            <span class="detail-value">${node.ipAddress}</span>
          </div>
          <div class="detail-item">
            <span class="detail-label">MAC Address:</span>
            <span class="detail-value">${node.macAddress}</span>
          </div>
          <div class="detail-item">
            <span class="detail-label">Battery:</span>
            <span class="detail-value">
              ${battery.toFixed(2)}V
              <div class="battery-indicator">
                <div class="battery-level ${batteryClass}" style="width: ${batteryWidth}%"></div>
              </div>
            </span>
          </div>
          <div class="detail-item">
            <span class="detail-label">Signal:</span>
            <span class="detail-value">${node.rssi}dBm</span>
          </div>
          <div class="detail-item">
            <span class="detail-label">Last Seen:</span>
            <span class="detail-value">${node.lastSeen}</span>
          </div>
          <div class="detail-item">
            <span class="detail-label">Detection:</span>
            <span class="detail-value">
              ${node.personDetected ? '🚨 PERSON DETECTED' : '✅ Normal'}
              <div class="detection-indicator ${node.personDetected ? 'detection-active' : 'detection-inactive'}"></div>
            </span>
          </div>
          ${node.lastDetectionTime && node.lastDetectionTime !== 'Never' ? `
          <div class="detail-item">
            <span class="detail-label">Last Detection:</span>
            <span class="detail-value" style="color: #ff6b35; font-weight: bold;">${node.lastDetectionTime}</span>
          </div>
          ` : ''}
          ${node.lastDetectionConfidence && parseFloat(node.lastDetectionConfidence) > 0 ? `
          <div class="detail-item">
            <span class="detail-label">Confidence:</span>
            <span class="detail-value" style="color: #ff6b35; font-weight: bold;">${(parseFloat(node.lastDetectionConfidence) * 100).toFixed(1)}%</span>
          </div>
          ` : ''}
          ${node.hopCount !== undefined ? `
          <div class="detail-item">
            <span class="detail-label">Hop Count:</span>
            <span class="detail-value">${node.hopCount}</span>
          </div>
          ` : ''}
          ${node.uptime !== undefined ? `
          <div class="detail-item">
            <span class="detail-label">Uptime:</span>
            <span class="detail-value">${node.uptime}</span>
          </div>
          ` : ''}
        </div>
      `;
      
      return card;
    }
    
    function toggleAutoRefresh() {
      meshAutoRefresh = !meshAutoRefresh;
      const btn = document.getElementById('autoRefreshBtn');
      
      if (meshAutoRefresh) {
        btn.textContent = '🔁 Auto-Refresh ON';
        btn.className = 'btn-info';
        meshRefreshInterval = setInterval(refreshMeshNodes, 10000); // Every 10 seconds
      } else {
        btn.textContent = '🔁 Auto-Refresh OFF';
        btn.className = 'btn-warning';
        clearInterval(meshRefreshInterval);
      }
    }    // Auto-refresh functions
    setInterval(refreshSystemInfo, 30000);   // Every 30 seconds for system info
    
    // Initialize mesh auto-refresh (will start when mesh dashboard is visible)
    meshRefreshInterval = setInterval(refreshMeshNodes, 10000); // Every 10 seconds for mesh
    
    // Initial loads
    refreshSystemInfo();
    loadNotificationConfig(); // Load notification settings on page load
  </script>
</body>
</html>
)rawliteral";

// Handle root page
void handleRoot() {
  Serial.println("Root page requested");
  server.send(200, "text/html", index_html);
}

// Handle image capture with person detection
// Handle system information requests
void handleSysInfo() {
  String lastAICheck = "Never";
  if (last_ai_check > 0) {
    unsigned long timeSince = (millis() - last_ai_check) / 1000;
    if (timeSince < 60) {
      lastAICheck = String(timeSince) + "s ago";
    } else if (timeSince < 3600) {
      lastAICheck = String(timeSince / 60) + "m ago";
    } else {
      lastAICheck = String(timeSince / 3600) + "h ago";
    }
  }
  
  String nodeTypeStr = "Unknown";
  switch (myNodeType) {
    case NODE_MASTER: nodeTypeStr = "Master"; break;
    case NODE_SLAVE: nodeTypeStr = "Slave"; break;
    case NODE_RELAY: nodeTypeStr = "Relay"; break;
    default: nodeTypeStr = "Unknown"; break;
  }
  
  String response = "{";
  response += "\"wifi_ssid\":\"" + String(ssid) + "\",";
  response += "\"wifi_ip\":\"" + WiFi.localIP().toString() + "\",";
  response += "\"wifi_rssi\":" + String(WiFi.RSSI()) + ",";
  response += "\"wifi_mac\":\"" + WiFi.macAddress() + "\",";
  response += "\"ai_server\":\"" + ai_server_ip + ":" + String(ai_server_port) + "\",";
  response += "\"ai_endpoint\":\"" + String(ai_endpoint) + "\",";
  response += "\"ai_ip\":\"" + ai_server_ip + "\",";
  response += "\"ai_port\":" + String(ai_server_port) + ",";
  response += "\"last_ai_check\":\"" + lastAICheck + "\",";
  response += "\"ai_status\":\"" + String(person_detected ? "Active" : "Monitoring") + "\",";
  response += "\"node_type\":\"" + nodeTypeStr + "\",";
  response += "\"node_id\":" + String(myNodeId) + ",";
  response += "\"master_id\":" + String(masterNodeId) + ",";
  response += "\"connected_nodes\":" + String(nodeCount);
  response += "}";
  
  server.send(200, "application/json", response);
}

// Handle configuration updates
void handleConfig() {
  if (server.method() == HTTP_POST) {
    String body = server.arg("plain");
    Serial.println("Config update: " + body);
    
    // Parse JSON manually (simple parsing for our needs)
    int ipStart = body.indexOf("\"ai_ip\":\"") + 9;
    int ipEnd = body.indexOf("\"", ipStart);
    int portStart = body.indexOf("\"ai_port\":") + 10;
    int portEnd = body.indexOf("}", portStart);
    if (portEnd == -1) portEnd = body.indexOf(",", portStart);
    
    if (ipStart > 8 && ipEnd > ipStart && portStart > 9 && portEnd > portStart) {
      String newIP = body.substring(ipStart, ipEnd);
      String portStr = body.substring(portStart, portEnd);
      int newPort = portStr.toInt();
      
      if (newIP.length() > 0 && newPort > 0 && newPort <= 65535) {
        ai_server_ip = newIP;
        ai_server_port = newPort;
        saveConfiguration();
        
        server.send(200, "application/json", "{\"success\":true,\"message\":\"Configuration saved\"}");
        Serial.printf("Configuration updated: %s:%d\n", ai_server_ip.c_str(), ai_server_port);
        return;
      }
    }
    
    server.send(400, "application/json", "{\"success\":false,\"message\":\"Invalid configuration data\"}");
  } else {
    server.send(405, "text/plain", "Method not allowed");
  }
}

// Handle notification configuration updates
void handleNotificationConfig() {
  if (server.method() == HTTP_POST) {
    String body = server.arg("plain");
    Serial.println("Notification config update: " + body);
    
    // Parse JSON manually for notification settings
    // MQTT settings
    int mqttServerStart = body.indexOf("\"mqtt_server\":\"") + 15;
    int mqttServerEnd = body.indexOf("\"", mqttServerStart);
    int mqttPortStart = body.indexOf("\"mqtt_port\":") + 12;
    int mqttPortEnd = body.indexOf(",", mqttPortStart);
    int mqttUserStart = body.indexOf("\"mqtt_username\":\"") + 17;
    int mqttUserEnd = body.indexOf("\"", mqttUserStart);
    int mqttPassStart = body.indexOf("\"mqtt_password\":\"") + 17;
    int mqttPassEnd = body.indexOf("\"", mqttPassStart);
    int mqttTopicStart = body.indexOf("\"mqtt_topic\":\"") + 14;
    int mqttTopicEnd = body.indexOf("\"", mqttTopicStart);
    
    // Gotify settings
    int gotifyServerStart = body.indexOf("\"gotify_server\":\"") + 17;
    int gotifyServerEnd = body.indexOf("\"", gotifyServerStart);
    int gotifyTokenStart = body.indexOf("\"gotify_token\":\"") + 16;
    int gotifyTokenEnd = body.indexOf("\"", gotifyTokenStart);
    
    // Discord settings  
    int discordWebhookStart = body.indexOf("\"discord_webhook\":\"") + 19;
    int discordWebhookEnd = body.indexOf("\"", discordWebhookStart);
    int discordUsernameStart = body.indexOf("\"discord_username\":\"") + 20;
    int discordUsernameEnd = body.indexOf("\"", discordUsernameStart);
    
    // Update config if valid data found
    bool configUpdated = false;
    
    if (mqttServerStart > 14 && mqttServerEnd > mqttServerStart) {
      notificationConfig.mqtt_server = body.substring(mqttServerStart, mqttServerEnd);
      configUpdated = true;
    }
    if (mqttPortStart > 11 && mqttPortEnd > mqttPortStart) {
      notificationConfig.mqtt_port = body.substring(mqttPortStart, mqttPortEnd).toInt();
      configUpdated = true;
    }
    if (mqttUserStart > 16 && mqttUserEnd > mqttUserStart) {
      notificationConfig.mqtt_username = body.substring(mqttUserStart, mqttUserEnd);
      configUpdated = true;
    }
    if (mqttPassStart > 16 && mqttPassEnd > mqttPassStart) {
      notificationConfig.mqtt_password = body.substring(mqttPassStart, mqttPassEnd);
      configUpdated = true;
    }
    if (mqttTopicStart > 13 && mqttTopicEnd > mqttTopicStart) {
      notificationConfig.mqtt_topic = body.substring(mqttTopicStart, mqttTopicEnd);
      configUpdated = true;
    }
    if (gotifyServerStart > 16 && gotifyServerEnd > gotifyServerStart) {
      notificationConfig.gotify_server = body.substring(gotifyServerStart, gotifyServerEnd);
      configUpdated = true;
    }
    if (gotifyTokenStart > 15 && gotifyTokenEnd > gotifyTokenStart) {
      notificationConfig.gotify_token = body.substring(gotifyTokenStart, gotifyTokenEnd);
      configUpdated = true;
    }
    if (discordWebhookStart > 18 && discordWebhookEnd > discordWebhookStart) {
      notificationConfig.discord_webhook = body.substring(discordWebhookStart, discordWebhookEnd);
      configUpdated = true;
    }
    if (discordUsernameStart > 19 && discordUsernameEnd > discordUsernameStart) {
      notificationConfig.discord_username = body.substring(discordUsernameStart, discordUsernameEnd);
      configUpdated = true;
    }
    
    if (configUpdated) {
      saveNotificationConfig();
      server.send(200, "application/json", "{\"success\":true,\"message\":\"Notification configuration saved\"}");
      Serial.println("Notification configuration updated successfully");
    } else {
      server.send(400, "application/json", "{\"success\":false,\"message\":\"Invalid notification configuration data\"}");
    }
  } else {
    // GET request - return current notification config
    String response = "{";
    response += "\"mqtt_server\":\"" + notificationConfig.mqtt_server + "\",";
    response += "\"mqtt_port\":" + String(notificationConfig.mqtt_port) + ",";
    response += "\"mqtt_username\":\"" + notificationConfig.mqtt_username + "\",";
    response += "\"mqtt_password\":\"" + notificationConfig.mqtt_password + "\",";
    response += "\"mqtt_topic\":\"" + notificationConfig.mqtt_topic + "\",";
    response += "\"gotify_server\":\"" + notificationConfig.gotify_server + "\",";
    response += "\"gotify_token\":\"" + notificationConfig.gotify_token + "\",";
    response += "\"discord_webhook\":\"" + notificationConfig.discord_webhook + "\",";
    response += "\"discord_username\":\"" + notificationConfig.discord_username + "\"";
    response += "}";
    
    server.send(200, "application/json", response);
  }
}

// Handle AI connection test
void handleTestAI() {
  HTTPClient http;
  WiFiClient client;
  
  // Use the official status ping endpoint for testing connectivity
  String url = "http://" + ai_server_ip + ":" + String(ai_server_port) + "/v1/status/ping";
  
  http.begin(client, url);
  http.setTimeout(8000); // 8 second timeout for test
  
  // Send a GET request to the ping endpoint
  int httpCode = http.GET();
  String response = "{";
  
  if (httpCode > 0) {
    String serverResponse = http.getString();
    response += "\"success\":true,";
    response += "\"message\":\"AI server ping successful\",";
    response += "\"http_code\":" + String(httpCode) + ",";
    response += "\"server_response\":" + serverResponse; // Should be {"success": true}
    Serial.printf("✅ AI ping test SUCCESS - HTTP %d\n", httpCode);
    Serial.printf("📥 Server response: %s\n", serverResponse.c_str());
  } else {
    response += "\"success\":false,";
    response += "\"error\":\"Connection failed - " + http.errorToString(httpCode) + "\",";
    response += "\"http_code\":" + String(httpCode);
    Serial.printf("❌ AI ping test FAILED - HTTP %d: %s\n", httpCode, http.errorToString(httpCode).c_str());
  }
  
  response += "}";
  http.end();
  
  server.send(200, "application/json", response);
}

// Handle AI detection test with current camera frame
void handleTestAIDetection() {
  Serial.println("Testing AI detection with current camera frame...");
  
  // Configure camera for faster AI detection
  // configureCameraForAI();
  delay(100); // Let camera adjust
  
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    // configureCameraForWeb(); // Restore camera settings
    server.send(500, "application/json", "{\"success\":false,\"error\":\"Failed to capture image\"}");
    return;
  }
  
  Serial.printf("Captured test frame: %dx%d, %d bytes\n", fb->width, fb->height, fb->len);
  
  bool detected = detectPersonWithAI(fb);
  esp_camera_fb_return(fb);
  
  // Restore camera to high quality
  // configureCameraForWeb();
  
  String response = "{";
  response += "\"success\":true,";
  response += "\"person_detected\":" + String(detected ? "true" : "false") + ",";
  response += "\"confidence\":" + String(detection_confidence, 2) + ",";
  response += "\"message\":\"" + String(detected ? "Person detected!" : "No person detected") + "\"";
  response += "}";
  
  server.send(200, "application/json", response);
}

// Handle mesh nodes list request
void handleMeshNodes() {
  String response = "{\"nodes\":[";
  
  // Add self as first node
  String myIP = WiFi.localIP().toString();
  String myMAC = WiFi.macAddress();
  float myBattery = getBatteryVoltage();
  
  // Format last detection time for self
  String myLastDetection = "Never";
  if (last_detection > 0) {
    unsigned long timeSince = (millis() - last_detection) / 1000;
    if (timeSince < 60) {
      myLastDetection = String(timeSince) + "s ago";
    } else if (timeSince < 3600) {
      myLastDetection = String(timeSince / 60) + "m ago";
    } else {
      myLastDetection = String(timeSince / 3600) + "h ago";
    }
  }
  
  response += "{";
  response += "\"id\":" + String(myNodeId) + ",";
  response += "\"type\":\"" + String(myNodeType == NODE_MASTER ? "Master" : myNodeType == NODE_SLAVE ? "Slave" : myNodeType == NODE_RELAY ? "Relay" : "Unknown") + "\",";
  response += "\"online\":true,";
  response += "\"battery\":\"" + String(myBattery, 2) + "\",";
  response += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  response += "\"lastSeen\":\"Active (Self)\",";
  response += "\"personDetected\":" + String(person_detected ? "true" : "false") + ",";
  response += "\"lastDetectionTime\":\"" + myLastDetection + "\",";
  response += "\"lastDetectionConfidence\":" + String(person_detected ? "0.85" : "0.00") + ",";
  response += "\"ipAddress\":\"" + myIP + "\",";
  response += "\"macAddress\":\"" + myMAC + "\",";
  response += "\"connectionType\":\"WiFi\",";
  response += "\"uptime\":\"" + String(millis() / 1000) + "s\",";
  response += "\"isSelf\":true";
  response += "}";
  
  // Add other mesh nodes
  for (int i = 0; i < nodeCount; i++) {
    response += ",";
    
    String lastSeen = "Never";
    if (meshNodes[i].lastSeen > 0) {
      unsigned long timeSince = (millis() - meshNodes[i].lastSeen) / 1000;
      if (timeSince < 60) {
        lastSeen = String(timeSince) + "s ago";
      } else if (timeSince < 3600) {
        lastSeen = String(timeSince / 60) + "m ago";
      } else {
        lastSeen = String(timeSince / 3600) + "h ago";
      }
    }
    
    String nodeTypeStr = "Unknown";
    switch (meshNodes[i].nodeType) {
      case NODE_MASTER: nodeTypeStr = "Master"; break;
      case NODE_SLAVE: nodeTypeStr = "Slave"; break;
      case NODE_RELAY: nodeTypeStr = "Relay"; break;
      default: nodeTypeStr = "Unknown"; break;
    }
    
    // Format MAC address for display
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             meshNodes[i].mac[0], meshNodes[i].mac[1], meshNodes[i].mac[2],
             meshNodes[i].mac[3], meshNodes[i].mac[4], meshNodes[i].mac[5]);
    
    // Format last detection time
    String lastDetection = "Never";
    if (meshNodes[i].lastDetectionTime > 0) {
      unsigned long timeSince = (millis() - meshNodes[i].lastDetectionTime) / 1000;
      if (timeSince < 60) {
        lastDetection = String(timeSince) + "s ago";
      } else if (timeSince < 3600) {
        lastDetection = String(timeSince / 60) + "m ago";
      } else {
        lastDetection = String(timeSince / 3600) + "h ago";
      }
    }
    
    response += "{";
    response += "\"id\":" + String(meshNodes[i].nodeId) + ",";
    response += "\"type\":\"" + nodeTypeStr + "\",";
    response += "\"online\":" + String(meshNodes[i].isOnline ? "true" : "false") + ",";
    response += "\"battery\":\"" + String(meshNodes[i].batteryLevel, 2) + "\",";
    response += "\"rssi\":" + String(meshNodes[i].rssi) + ",";
    response += "\"lastSeen\":\"" + lastSeen + "\",";
    response += "\"personDetected\":" + String(meshNodes[i].personDetected ? "true" : "false") + ",";
    response += "\"lastDetectionTime\":\"" + lastDetection + "\",";
    response += "\"lastDetectionConfidence\":" + String(meshNodes[i].lastDetectionConfidence, 2) + ",";
    response += "\"ipAddress\":\"" + String(meshNodes[i].ipAddress) + "\",";
    response += "\"macAddress\":\"" + String(macStr) + "\",";
    response += "\"connectionType\":\"Mesh\",";
    response += "\"hopCount\":" + String(meshNodes[i].hopCount) + ",";
    response += "\"isSelf\":false";
    response += "}";
  }
  
  response += "]}";
  server.send(200, "application/json", response);
}

// Handle mesh configuration broadcast
void handleMeshBroadcastConfig() {
  if (server.method() == HTTP_POST) {
    if (myNodeType == NODE_MASTER) {
      updateMeshConfig();
      server.send(200, "application/json", "{\"success\":true,\"message\":\"Configuration broadcast to mesh\"}");
    } else {
      server.send(403, "application/json", "{\"success\":false,\"message\":\"Only master can broadcast configuration\"}");
    }
  } else {
    server.send(405, "text/plain", "Method not allowed");
  }
}

// Handle mesh scan request
void handleMeshScan() {
  if (server.method() == HTTP_POST) {
    broadcastDiscovery();
    server.send(200, "application/json", "{\"success\":true,\"message\":\"Mesh scan initiated\"}");
  } else {
    server.send(405, "text/plain", "Method not allowed");
  }
}

// Handle mesh reset request
void handleMeshReset() {
  if (server.method() == HTTP_POST) {
    if (myNodeType == NODE_MASTER) {
      // Clear mesh node table
      nodeCount = 0;
      memset(meshNodes, 0, sizeof(meshNodes));
      
      // Clear preferences
      preferences.begin("esp32cam", false);
      preferences.remove("nodeType");
      preferences.remove("nodeId");
      preferences.end();
      
      server.send(200, "application/json", "{\"success\":true,\"message\":\"Mesh network reset\"}");
      Serial.println("🔄 Mesh network reset by web request");
    } else {
      server.send(403, "application/json", "{\"success\":false,\"message\":\"Only master can reset mesh\"}");
    }
  } else {
    server.send(405, "text/plain", "Method not allowed");
  }
}

// Handle mesh discovery broadcast
void handleMeshDiscovery() {
  if (server.method() == HTTP_POST) {
    broadcastDiscovery();
    server.send(200, "application/json", "{\"success\":true,\"message\":\"Discovery broadcast sent\"}");
  } else {
    server.send(405, "text/plain", "Method not allowed");
  }
}

// Handle streaming
void handleStream() {
  Serial.println("Stream requested");
  streaming_active = true; // Set flag to pause automatic detection
  
  WiFiClient client = server.client();
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  client.print(response);
  
  while (client.connected()) {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      delay(100);
      continue;
    }

    String header = "--frame\r\n";
    header += "Content-Type: image/jpeg\r\n";
    header += "Content-Length: " + String(fb->len) + "\r\n\r\n";
    
    client.print(header);
    client.write(fb->buf, fb->len);
    client.print("\r\n");
    
    esp_camera_fb_return(fb);
    
    if (!client.connected()) {
      Serial.println("Client disconnected");
      break;
    }
    
    delay(30); // ~33 FPS
  }
  Serial.println("Stream ended");
  streaming_active = false; // Clear flag to resume automatic detection
}

// ===== NOTIFICATION FUNCTIONS =====

// Load notification configuration from preferences
void loadNotificationConfig() {
  if (preferences.begin("notifications", true)) {
    notificationConfig.mqtt_enabled = preferences.getBool("mqtt_en", false);
    notificationConfig.mqtt_server = preferences.getString("mqtt_srv", "192.168.1.100");
    notificationConfig.mqtt_port = preferences.getInt("mqtt_port", 1883);
    notificationConfig.mqtt_username = preferences.getString("mqtt_user", "");
    notificationConfig.mqtt_password = preferences.getString("mqtt_pass", "");
    notificationConfig.mqtt_topic = preferences.getString("mqtt_topic", "esp32cam/detection");
    
    notificationConfig.gotify_enabled = preferences.getBool("gotify_en", false);
    notificationConfig.gotify_server = preferences.getString("gotify_srv", "192.168.1.100");
    notificationConfig.gotify_port = preferences.getInt("gotify_port", 80);
    notificationConfig.gotify_token = preferences.getString("gotify_token", "");
    notificationConfig.gotify_title = preferences.getString("gotify_title", "ESP32-CAM Detection");
    
    notificationConfig.discord_enabled = preferences.getBool("discord_en", false);
    notificationConfig.discord_webhook = preferences.getString("discord_hook", "");
    notificationConfig.discord_username = preferences.getString("discord_user", "ESP32-CAM");
    
    preferences.end();
    Serial.println("📱 Notification config loaded");
  }
}

// Save notification configuration to preferences
void saveNotificationConfig() {
  if (preferences.begin("notifications", false)) {
    preferences.putBool("mqtt_en", notificationConfig.mqtt_enabled);
    preferences.putString("mqtt_srv", notificationConfig.mqtt_server);
    preferences.putInt("mqtt_port", notificationConfig.mqtt_port);
    preferences.putString("mqtt_user", notificationConfig.mqtt_username);
    preferences.putString("mqtt_pass", notificationConfig.mqtt_password);
    preferences.putString("mqtt_topic", notificationConfig.mqtt_topic);
    
    preferences.putBool("gotify_en", notificationConfig.gotify_enabled);
    preferences.putString("gotify_srv", notificationConfig.gotify_server);
    preferences.putInt("gotify_port", notificationConfig.gotify_port);
    preferences.putString("gotify_token", notificationConfig.gotify_token);
    preferences.putString("gotify_title", notificationConfig.gotify_title);
    
    preferences.putBool("discord_en", notificationConfig.discord_enabled);
    preferences.putString("discord_hook", notificationConfig.discord_webhook);
    preferences.putString("discord_user", notificationConfig.discord_username);
    
    preferences.end();
    Serial.println("📱 Notification config saved");
  }
}

// Initialize MQTT connection
void initNotifications() {
  if (notificationConfig.mqtt_enabled) {
    mqttClient.setServer(notificationConfig.mqtt_server.c_str(), notificationConfig.mqtt_port);
    Serial.printf("📱 MQTT configured: %s:%d\n", notificationConfig.mqtt_server.c_str(), notificationConfig.mqtt_port);
  }
}

// Send MQTT notification
bool sendMQTTNotification(const String& message, const String& nodeId, float confidence) {
  if (!notificationConfig.mqtt_enabled) return false;
  
  if (!mqttClient.connected()) {
    Serial.println("📱 Connecting to MQTT...");
    String clientId = "ESP32CAM-" + String(myNodeId);
    
    bool connected = false;
    if (notificationConfig.mqtt_username.length() > 0) {
      connected = mqttClient.connect(clientId.c_str(), 
                                   notificationConfig.mqtt_username.c_str(), 
                                   notificationConfig.mqtt_password.c_str());
    } else {
      connected = mqttClient.connect(clientId.c_str());
    }
    
    if (!connected) {
      Serial.println("❌ MQTT connection failed");
      return false;
    }
    Serial.println("✅ MQTT connected");
  }
  
  // Create JSON payload
  JsonDocument doc;
  doc["timestamp"] = millis() / 1000;
  doc["nodeId"] = nodeId;
  doc["nodeType"] = (myNodeType == NODE_MASTER) ? "Master" : "Slave";
  doc["message"] = message;
  doc["confidence"] = confidence;
  doc["battery"] = getBatteryVoltage();
  doc["rssi"] = WiFi.RSSI();
  
  String payload;
  serializeJson(doc, payload);
  
  bool success = mqttClient.publish(notificationConfig.mqtt_topic.c_str(), payload.c_str());
  if (success) {
    Serial.printf("📱 MQTT sent: %s\n", payload.c_str());
  } else {
    Serial.println("❌ MQTT publish failed");
  }
  
  return success;
}

// Send Gotify notification
bool sendGotifyNotification(const String& message, const String& nodeId, float confidence) {
  if (!notificationConfig.gotify_enabled || notificationConfig.gotify_token.length() == 0) {
    return false;
  }
  
  HTTPClient http;
  String url = "http://" + notificationConfig.gotify_server + ":" + 
               String(notificationConfig.gotify_port) + "/message";
  
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-Gotify-Key", notificationConfig.gotify_token);
  
  // Create JSON payload
  JsonDocument doc;
  doc["title"] = notificationConfig.gotify_title;
  doc["message"] = message + "\n\nNode: " + nodeId + " (" + 
                   ((myNodeType == NODE_MASTER) ? "Master" : "Slave") + ")\n" +
                   "Confidence: " + String(confidence, 1) + "%\n" +
                   "Battery: " + String(getBatteryVoltage(), 2) + "V";
  doc["priority"] = 5;  // High priority
  
  String payload;
  serializeJson(doc, payload);
  
  int httpCode = http.POST(payload);
  bool success = (httpCode == 200);
  
  if (success) {
    Serial.printf("📱 Gotify sent: %d\n", httpCode);
  } else {
    Serial.printf("❌ Gotify failed: %d\n", httpCode);
  }
  
  http.end();
  return success;
}

// Send Discord notification
bool sendDiscordNotification(const String& message, const String& nodeId, float confidence) {
  if (!notificationConfig.discord_enabled || notificationConfig.discord_webhook.length() == 0) {
    return false;
  }
  
  HTTPClient http;
  http.begin(notificationConfig.discord_webhook);
  http.addHeader("Content-Type", "application/json");
  
  // Create Discord embed
  JsonDocument doc;
  doc["username"] = notificationConfig.discord_username;
  
  JsonArray embeds = doc["embeds"].to<JsonArray>();
  JsonObject embed = embeds.add<JsonObject>();
  
  embed["title"] = "🚨 Person Detection Alert";
  embed["description"] = message;
  embed["color"] = 16744448; // Orange color
  embed["timestamp"] = "2023-01-01T00:00:00.000Z"; // Current timestamp would be better
  
  JsonArray fields = embed["fields"].to<JsonArray>();
  
  JsonObject nodeField = fields.add<JsonObject>();
  nodeField["name"] = "Node";
  nodeField["value"] = nodeId + " (" + ((myNodeType == NODE_MASTER) ? "Master" : "Slave") + ")";
  nodeField["inline"] = true;
  
  JsonObject confidenceField = fields.add<JsonObject>();
  confidenceField["name"] = "Confidence";
  confidenceField["value"] = String(confidence, 1) + "%";
  confidenceField["inline"] = true;
  
  JsonObject batteryField = fields.add<JsonObject>();
  batteryField["name"] = "Battery";
  batteryField["value"] = String(getBatteryVoltage(), 2) + "V";
  batteryField["inline"] = true;
  
  String payload;
  serializeJson(doc, payload);
  
  int httpCode = http.POST(payload);
  bool success = (httpCode == 200 || httpCode == 204);
  
  if (success) {
    Serial.printf("📱 Discord sent: %d\n", httpCode);
  } else {
    Serial.printf("❌ Discord failed: %d\n", httpCode);
  }
  
  http.end();
  return success;
}

// Send notifications to all enabled services
void sendAllNotifications(const String& message, const String& nodeId, float confidence) {
  Serial.printf("📱 Sending notifications: %s (Node %s, %.1f%%)\n", 
                message.c_str(), nodeId.c_str(), confidence);
                
  if (notificationConfig.mqtt_enabled) {
    sendMQTTNotification(message, nodeId, confidence);
  }
  
  if (notificationConfig.gotify_enabled) {
    sendGotifyNotification(message, nodeId, confidence);
  }
  
  if (notificationConfig.discord_enabled) {
    sendDiscordNotification(message, nodeId, confidence);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("=== ESP32-CAM Solar Person Detection with Mesh ===");

  // Load saved configuration
  loadConfiguration();
  loadNotificationConfig();  // Load notification settings

  // Initialize battery monitoring
  pinMode(BATTERY_PIN, INPUT);
  Serial.printf("Battery voltage: %.2fV\n", getBatteryVoltage());
  
  // Check if we should enter deep sleep due to low battery
  if (isLowBattery()) {
    Serial.println("Low battery detected! Entering deep sleep...");
    enterDeepSleep();
  }

  // Initialize camera first (most critical)
  if (!configureCamera()) {
    Serial.println("Camera configuration failed!");
    return;
  }
  Serial.println("Camera configured successfully");

  // Test camera capture
  camera_fb_t * fb = esp_camera_fb_get();
  if (fb) {
    Serial.printf("Test capture successful: %dx%d, %u bytes\n", fb->width, fb->height, fb->len);
    esp_camera_fb_return(fb);
  } else {
    Serial.println("Test capture failed");
  }

  // Connect to WiFi first (required for ESP-NOW)
#if ENABLE_MESH_NETWORK
  // Initialize mesh network (this will handle WiFi setup)
  if (!initMeshNetwork()) {
    Serial.println("❌ Mesh network initialization failed, falling back to home WiFi");
    // Fall back to connecting to home network
    WiFi.begin(ssid, password);
    Serial.print("Connecting to home WiFi");
    
    int wifi_attempts = 0;
    while (WiFi.status() != WL_CONNECTED && wifi_attempts < 20) {
      delay(1000);
      Serial.print(".");
      wifi_attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println();
      Serial.println("WiFi connected!");
      Serial.print("Camera ready at: http://");
      Serial.println(WiFi.localIP());
      // Initialize notifications for fallback WiFi connection
      initNotifications();
    } else {
      Serial.println("\nWiFi connection failed! Starting without network...");
    }
  } else {
    Serial.println("✅ Mesh network initialized successfully");
    Serial.print("Camera ready at: ");
    if (myNodeType == NODE_MASTER) {
      if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("http://%s (home network) and http://%s (mesh AP)\n", 
                     WiFi.localIP().toString().c_str(), WiFi.softAPIP().toString().c_str());
        // Initialize notifications for master nodes with internet access
        initNotifications();
      } else {
        Serial.printf("http://%s (mesh AP only)\n", WiFi.softAPIP().toString().c_str());
      }
    } else {
      Serial.printf("http://%s (mesh network)\n", WiFi.localIP().toString().c_str());
    }
    
    // Start mesh discovery after a delay  
    delay(1000);
    Serial.println("🔍 Starting mesh discovery...");
    broadcastDiscovery();
  }
#else
  // If mesh is disabled, just connect to home network
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  int wifi_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_attempts < 20) {
    delay(1000);
    Serial.print(".");
    wifi_attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("WiFi connected!");
    Serial.print("Camera ready at: http://");
    Serial.println(WiFi.localIP());
    // Initialize notifications for non-mesh WiFi connection
    initNotifications();
  } else {
    Serial.println("\nWiFi connection failed! Starting without network...");
  }
  Serial.println("📡 Mesh networking disabled in configuration");
#endif

  // Configure web server routes (always do this, regardless of WiFi status)
  server.on("/", handleRoot);
  server.on("/stream", handleStream);
  server.on("/sysinfo", handleSysInfo);
  server.on("/config", handleConfig);
  server.on("/config-notifications", handleNotificationConfig);
  server.on("/test-ai", handleTestAI);
  server.on("/mesh-nodes", handleMeshNodes);
  server.on("/mesh-broadcast-config", handleMeshBroadcastConfig);
  server.on("/mesh-scan", handleMeshScan);
  server.on("/mesh-reset", handleMeshReset);
  server.on("/mesh-discovery", handleMeshDiscovery);
  
  // Handle 404
  server.onNotFound([]() {
    String message = "File Not Found\n\n";
    message += "URI: " + server.uri() + "\n";
    message += "Method: " + String(server.method()) + "\n";
    Serial.println("404: " + message);
    server.send(404, "text/plain", message);
  });

  // Start web server
  server.begin();
  Serial.println("Web server started");
  
  // Print the correct IP address
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Open http://" + WiFi.localIP().toString() + " in your browser");
  } else {
    Serial.println("WiFi not connected - check network settings");
  }
  
  Serial.println("Person detection active!");
}

void loop() {
  server.handleClient();
  
  // Monitor heap health and trigger garbage collection if needed
  static unsigned long lastHeapCheck = 0;
  if (millis() - lastHeapCheck > 30000) { // Check every 30 seconds
    size_t free_heap = ESP.getFreeHeap();
    size_t largest_block = ESP.getMaxAllocHeap();
    
    Serial.printf("Heap status - Free: %d bytes, Largest block: %d bytes\n", free_heap, largest_block);
    
    if (free_heap < 30000) {
      Serial.println("⚠️ Low heap detected, triggering garbage collection");
      // Force garbage collection by creating and destroying a temporary object
      String temp = "garbage_collection_trigger";
      temp = "";
    }
    
    lastHeapCheck = millis();
  }
  
  // Only do mesh operations if mesh is available and enabled
#if ENABLE_MESH_NETWORK
  if (WiFi.status() == WL_CONNECTED) {
    // Check if we should become mesh master (only if role is still unknown)
    if (myNodeType == NODE_UNKNOWN) {
      checkBecomeMaster();
    }
    
    // Send discovery broadcasts regularly while looking for master or to announce presence
    if ((myNodeType == NODE_UNKNOWN && (millis() - lastDiscovery) > DISCOVERY_INTERVAL) ||
        (myNodeType == NODE_MASTER && (millis() - lastDiscovery) > DISCOVERY_INTERVAL * 2)) {
      broadcastDiscovery();
      lastDiscovery = millis();
    }
    
    // Send mesh heartbeat (only if mesh is initialized)
    if (meshInitialized) {
      sendMeshHeartbeat();
    }
    
    // Clean up offline mesh nodes
    static unsigned long lastMeshCleanup = 0;
    if (millis() - lastMeshCleanup > 60000) { // Every minute
      cleanupMeshNodes();
      lastMeshCleanup = millis();
    }
  }
#endif
  
  // Timed person detection - this runs every 5 seconds (not continuous)
  static unsigned long lastPersonCheck = 0;
  if (millis() - lastPersonCheck > 5000) { // 5 seconds interval
    checkForPerson();
    lastPersonCheck = millis();
  }
  
  // Check battery level every 60 seconds (reduced frequency)
  static unsigned long lastBatteryCheck = 0;
  if (millis() - lastBatteryCheck > 60000) {
    lastBatteryCheck = millis();
    
    float batteryVoltage = getBatteryVoltage();
    
    if (batteryVoltage < LOW_BATTERY_THRESHOLD) {
      Serial.println("Low battery detected in loop! Entering deep sleep...");
      enterDeepSleep();
    }
    
    Serial.printf("Battery: %.2fV\n", batteryVoltage);
  }
  
  // Optional: Enter deep sleep if no activity for extended period (power saving)
  static unsigned long lastActivity = millis();
  if (millis() - lastActivity > 600000) { // 10 minutes instead of 5
    Serial.println("No activity detected. Entering deep sleep for power saving...");
    enterDeepSleep();
  }
  
  // Reset activity timer when person is detected or web request is made
  if (person_detected || server.client()) {
    lastActivity = millis();
  }
  
  // Feed the watchdog and give system more time to breathe
  yield();
  delay(50); // Increased from 10ms to 50ms to give system more recovery time
}
