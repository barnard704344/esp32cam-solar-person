#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
#include <Preferences.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "esp_camera.h"
#include "esp_sleep.h"
#include "esp_pm.h"

// Mesh configuration - set to false to disable mesh networking
#define ENABLE_MESH_NETWORK true

// WiFi credentials - CHANGE THESE TO YOUR NETWORK
const char* ssid = "shs-0941";
const char* password = "L3TSG0SH0PP1NG";

// CodeProject.AI Server settings - These can be configured via web interface
String ai_server_ip = "192.168.1.100";  // Default IP, can be changed via web
int ai_server_port = 32168;             // Default port, can be changed via web
const char* ai_endpoint = "/v1/vision/detection";  // YOLO detection endpoint

// Preferences for storing configuration
Preferences preferences;

// Mesh Network Configuration
#define MESH_CHANNEL 1
#define MAX_MESH_NODES 10
#define MESH_SCAN_TIMEOUT 30000  // 30 seconds to find master
#define HEARTBEAT_INTERVAL 60000 // 1 minute heartbeat
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
  MSG_STATUS_RESPONSE = 7
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

// Web server on port 80
WebServer server(80);

// Power management and detection settings
#define BATTERY_PIN 33          // ADC pin to read battery voltage
#define SLEEP_TIME_US 30000000  // 30 seconds in microseconds
#define LOW_BATTERY_THRESHOLD 3.3 // Volts
#define DETECTION_INTERVAL 10000  // Check for person every 10 seconds (send to AI)

// Performance optimization settings
static int ai_timeout = 6000;                      // Reduced AI server timeout (was 10000ms)
static int ai_image_quality = 8;                   // Higher compression for faster upload (1-63, lower=smaller)
static framesize_t ai_frame_size = FRAMESIZE_SVGA; // Smaller frame for AI (800x600 vs 1600x1200)
static float ai_confidence_threshold = 0.4;        // Minimum confidence for person detection

// Person detection variables
static bool person_detected = false;
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
  
  // Load performance settings
  ai_timeout = preferences.getInt("ai_timeout", 6000);
  ai_image_quality = preferences.getInt("ai_quality", 8);
  ai_frame_size = (framesize_t)preferences.getInt("ai_frame_size", (int)FRAMESIZE_SVGA);
  ai_confidence_threshold = preferences.getFloat("ai_threshold", 0.4);
  
  preferences.end();
  
  Serial.printf("Loaded configuration - AI Server: %s:%d\n", ai_server_ip.c_str(), ai_server_port);
  Serial.printf("Performance settings - Timeout: %dms, Quality: %d, Frame: %d, Threshold: %.2f\n", 
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
    peerInfo.channel = 0;
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
  if (!meshInitialized && myNodeType == NODE_UNKNOWN) {
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
}

// Handle incoming mesh messages
void onMeshMessageReceived(const uint8_t* mac, const uint8_t* incomingData, int len) {
  // Safety checks
  if (!mac || !incomingData || len <= 0) {
    Serial.println("❌ Invalid mesh message received");
    return;
  }
  
  if (len != sizeof(MeshMessage)) {
    Serial.printf("❌ Invalid message size: %d (expected %d)\n", len, sizeof(MeshMessage));
    return;
  }
  
  MeshMessage* msg = (MeshMessage*)incomingData;
  
  // Basic validation
  if (!msg) {
    Serial.println("❌ Null message pointer");
    return;
  }
  
  // Verify checksum
  uint8_t receivedChecksum = msg->checksum;
  msg->checksum = 0;
  uint8_t calculatedChecksum = calculateChecksum(msg);
  msg->checksum = receivedChecksum;
  
  if (receivedChecksum != calculatedChecksum) {
    Serial.println("❌ Message checksum failed");
    return;
  }
  
  // Prevent processing old messages (simple time-based filtering)
  if (millis() - msg->timestamp > 30000) {
    Serial.println("⏰ Message too old, ignoring");
    return;
  }
  
  Serial.printf("📥 Received mesh message type:%d from node:%d\n", msg->type, msg->nodeId);
  
  // Update sender in mesh table (simplified for safety)
  int nodeIndex = findNodeByMAC(mac);
  if (nodeIndex == -1 && nodeCount < MAX_MESH_NODES) {
    nodeIndex = nodeCount++;
    memcpy(meshNodes[nodeIndex].mac, mac, 6);
    meshNodes[nodeIndex].nodeId = msg->nodeId;
    Serial.printf("� Added new mesh node ID:%d\n", msg->nodeId);
  }
  
  if (nodeIndex != -1) {
    meshNodes[nodeIndex].lastSeen = millis();
    meshNodes[nodeIndex].isOnline = true;
  }
  
  // Process message based on type
  switch (msg->type) {
    case MSG_DISCOVERY:
      if (myNodeType == NODE_MASTER) {
        handleDiscoveryMessage(msg, mac);
      }
      break;
      
    case MSG_JOIN_REQUEST:
      if (myNodeType == NODE_MASTER) {
        handleJoinRequest(msg, mac);
      }
      break;
      
    case MSG_JOIN_RESPONSE:
      if (myNodeType == NODE_UNKNOWN) {
        handleJoinResponse(msg, mac);
      }
      break;
      
    case MSG_CONFIG_UPDATE:
      handleConfigUpdate(msg, mac);
      break;
      
    case MSG_DETECTION_ALERT:
      handleDetectionAlert(msg, mac);
      break;
      
    case MSG_HEARTBEAT:
      handleHeartbeat(msg, mac);
      break;
      
    case MSG_STATUS_REQUEST:
      handleStatusRequest(msg, mac);
      break;
      
    case MSG_STATUS_RESPONSE:
      handleStatusResponse(msg, mac);
      break;
      
    default:
      Serial.printf("❓ Unknown message type: %d\n", msg->type);
      break;
  }
  
  // Forward message if it's not for us and hop count is reasonable
  if (msg->targetId != myNodeId && msg->targetId != 0 && msg->hopCount < 3) {
    msg->hopCount++;
    esp_now_send(NULL, (uint8_t*)msg, sizeof(MeshMessage));
    Serial.printf("🔄 Forwarded message to node:%d (hop:%d)\n", msg->targetId, msg->hopCount);
  }
}

// Handle discovery messages
void handleDiscoveryMessage(MeshMessage* msg, const uint8_t* mac) {
  if (myNodeType == NODE_MASTER) {
    // Master responds to discovery with join invitation
    struct {
      uint8_t assignedNodeId;
      char networkSSID[32];
      char networkPassword[64];
      char aiServerIP[16];
      uint16_t aiServerPort;
    } joinResponse;
    
    joinResponse.assignedNodeId = nodeCount + 1; // Simple ID assignment
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
  if (myNodeType == NODE_MASTER) {
    // Process join request and send configuration
    handleDiscoveryMessage(msg, mac); // Reuse discovery handler
  }
}

// Handle join responses
void handleJoinResponse(MeshMessage* msg, const uint8_t* mac) {
  if (myNodeType == NODE_UNKNOWN) {
    struct {
      uint8_t assignedNodeId;
      char networkSSID[32];
      char networkPassword[64];
      char aiServerIP[16];
      uint16_t aiServerPort;
    }* joinData = (decltype(joinData))msg->data;
    
    // Accept configuration from master
    myNodeId = joinData->assignedNodeId;
    myNodeType = NODE_SLAVE;
    masterNodeId = msg->nodeId;
    
    // Update network configuration
    ai_server_ip = String(joinData->aiServerIP);
    ai_server_port = joinData->aiServerPort;
    saveConfiguration();
    
    Serial.printf("✅ Joined mesh as node ID:%d, Master ID:%d\n", myNodeId, masterNodeId);
    Serial.printf("📡 Received network config - AI Server: %s:%d\n", ai_server_ip.c_str(), ai_server_port);
    
    meshInitialized = true;
  }
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
  
  // Update the alerting node's status
  int nodeIndex = findNodeById(alertData->nodeId);
  if (nodeIndex != -1) {
    meshNodes[nodeIndex].personDetected = true;
    meshNodes[nodeIndex].lastSeen = millis();
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
  // Process status information from other nodes
  // This would be displayed in the web interface
}

// Initialize mesh networking
bool initMeshNetwork() {
  Serial.println("🔧 Starting mesh network initialization...");
  
  // Initialize WiFi for ESP-NOW (maintain existing connection)
  bool wasConnected = (WiFi.status() == WL_CONNECTED);
  WiFi.mode(WIFI_AP_STA);
  delay(100);
  
  // If we were connected, try to maintain the connection
  if (wasConnected) {
    WiFi.begin(ssid, password);
    delay(1000); // Give time to reconnect
  }
  
  // Get MAC address
  WiFi.macAddress(myMac);
  Serial.printf("� Device MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5]);
  
  // Initialize mesh variables
  myNodeType = NODE_UNKNOWN;
  myNodeId = 0;
  masterNodeId = 0;
  nodeCount = 0;
  meshInitialized = false;
  lastHeartbeat = 0;
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
  
  // Start mesh discovery process
  Serial.println("🔍 Starting mesh discovery...");
  
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
  
  struct {
    float batteryLevel;
    bool personDetected;
    uint32_t uptime;
  } heartbeatData;
  
  heartbeatData.batteryLevel = getBatteryVoltage();
  heartbeatData.personDetected = person_detected;
  heartbeatData.uptime = millis() / 1000;
  
  sendMeshMessage(0, MSG_HEARTBEAT, &heartbeatData, sizeof(heartbeatData));
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
  
  unsigned long start_time = millis();
  Serial.printf("🔍 AI Detection: Frame %dx%d, %d bytes\n", frame->width, frame->height, frame->len);
  
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
  
  // Create multipart form data boundary
  String boundary = "----WebKitFormBoundary7MA4YWxkTrZu0gW";
  String contentType = "multipart/form-data; boundary=" + boundary;
  
  // Calculate payload size more efficiently
  size_t payload_size = 
    2 + boundary.length() + 2 +  // --boundary\r\n
    61 +                         // Content-Disposition: form-data; name="image"; filename="capture.jpg"\r\n
    25 +                         // Content-Type: image/jpeg\r\n
    2 +                          // \r\n
    frame->len +                 // Image data
    2 +                          // \r\n
    2 + boundary.length() + 4;   // --boundary--\r\n
  
  // Allocate payload buffer with error checking
  uint8_t* payload = (uint8_t*)malloc(payload_size);
  if (!payload) {
    Serial.printf("❌ Failed to allocate %d bytes for payload\n", payload_size);
    http.end();
    return false;
  }
  
  // Build multipart form data efficiently
  size_t pos = 0;
  
  // Add boundary start
  pos += sprintf((char*)payload + pos, "--%s\r\n", boundary.c_str());
  pos += sprintf((char*)payload + pos, "Content-Disposition: form-data; name=\"image\"; filename=\"capture.jpg\"\r\n");
  pos += sprintf((char*)payload + pos, "Content-Type: image/jpeg\r\n\r\n");
  
  // Copy image data
  memcpy(payload + pos, frame->buf, frame->len);
  pos += frame->len;
  
  // Add boundary end
  pos += sprintf((char*)payload + pos, "\r\n--%s--\r\n", boundary.c_str());
  
  Serial.printf("📤 Sending %d bytes to AI server...\n", pos);
  
  // Send HTTP POST request
  http.addHeader("Content-Type", contentType);
  http.addHeader("Content-Length", String(pos));
  
  int httpCode = http.POST(payload, pos);
  
  // Free payload memory immediately
  free(payload);
  
  if (httpCode <= 0) {
    Serial.printf("❌ HTTP request failed: %s\n", http.errorToString(httpCode).c_str());
    http.end();
    return false;
  }
  
  if (httpCode != 200) {
    Serial.printf("❌ AI server error: HTTP %d\n", httpCode);
    http.end();
    return false;
  }
  
  String response = http.getString();
  http.end();
  
  unsigned long end_time = millis();
  Serial.printf("⚡ AI request completed in %d ms\n", end_time - start_time);
  Serial.printf("📥 AI Response: %s\n", response.c_str());
  
  // Parse JSON response for person detection with multiple confidence field names
  if (response.indexOf("\"label\":\"person\"") >= 0 || 
      response.indexOf("\"name\":\"person\"") >= 0 ||
      response.indexOf("\"class\":\"person\"") >= 0) {
    
    // Look for confidence values with various field names
    String confStr = "";
    int confStart = -1;
    
    // Try "confidence" field
    confStart = response.indexOf("\"confidence\":");
    if (confStart == -1) confStart = response.indexOf("\"score\":");
    if (confStart == -1) confStart = response.indexOf("\"prob\":");
    
    if (confStart > -1) {
      confStart = response.indexOf(":", confStart) + 1;
      int endIndex = response.indexOf(",", confStart);
      if (endIndex == -1) endIndex = response.indexOf("}", confStart);
      if (endIndex > confStart) {
        confStr = response.substring(confStart, endIndex);
        confStr.trim();
        if (confStr.length() > 0) {
          detection_confidence = confStr.toFloat();
        }
      }
    }
    
    if (detection_confidence >= ai_confidence_threshold) {
      Serial.printf("✅ PERSON DETECTED by AI! Confidence: %.2f (threshold: %.2f)\n", 
                   detection_confidence, ai_confidence_threshold);
      return true;
    } else {
      Serial.printf("⚠️ Person confidence too low: %.2f < %.2f\n", 
                   detection_confidence, ai_confidence_threshold);
    }
  }
  
  Serial.println("ℹ️ No person detected by AI");
  return false;
}

// Check for person using AI detection
void checkForPerson() {
  if (millis() - last_ai_check > DETECTION_INTERVAL) {
    last_ai_check = millis();
    
    // Configure camera for faster AI detection
    configureCameraForAI();
    
    // Small delay to let camera adjust
    delay(100);
    
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
      
#if ENABLE_MESH_NETWORK
      // Send alert to mesh network
      sendDetectionAlert();
#endif
    } else {
      Serial.println("No person detected by AI");
    }
    
    esp_camera_fb_return(fb);
    
    // Restore camera to high quality for web interface
    configureCameraForWeb();
  }
}

// Configure camera for optimal AI detection speed
void configureCameraForAI() {
  sensor_t* s = esp_camera_sensor_get();
  if (s != NULL) {
    // Set smaller frame size for faster AI detection
    s->set_framesize(s, ai_frame_size);
    s->set_quality(s, ai_image_quality);  // Higher compression for speed
    
    // Optimize other settings for speed
    s->set_brightness(s, 0);     // -2 to 2
    s->set_contrast(s, 0);       // -2 to 2
    s->set_saturation(s, 0);     // -2 to 2
    s->set_special_effect(s, 0); // 0 to 6 (0=None, 1=Negative, 2=Grayscale, ...)
    
    Serial.printf("📷 Camera configured for AI: %dx%d, quality=%d\n", 
                 ai_frame_size == FRAMESIZE_VGA ? 640 : (ai_frame_size == FRAMESIZE_SVGA ? 800 : 1024),
                 ai_frame_size == FRAMESIZE_VGA ? 480 : (ai_frame_size == FRAMESIZE_SVGA ? 600 : 768),
                 ai_image_quality);
  }
}

// Restore camera to high quality for web interface
void configureCameraForWeb() {
  sensor_t* s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_framesize(s, psramFound() ? FRAMESIZE_UXGA : FRAMESIZE_VGA);
    s->set_quality(s, psramFound() ? 10 : 15);
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
    
    <div class="status">
      <div class="status-item">
        <div>Person Detection</div>
        <div class="status-value" id="detection">Normal</div>
      </div>
      <div class="status-item">
        <div>Battery</div>
        <div class="status-value" id="battery">--V</div>
      </div>
      <div class="status-item">
        <div>Last Detection</div>
        <div class="status-value" id="lastSeen">Never</div>
      </div>
    </div>
    
    <div class="controls">
      <button onclick="captureImage()">Take Photo</button>
      <button onclick="refreshStatus()">Refresh Status</button>
      <button onclick="location.reload()">Reload Page</button>
    </div>

    <div id="imageContainer" style="text-align: center; margin: 20px 0;">
      <img id="stream" style="max-width: 100%; display: none;" alt="Camera Image">
      <p id="imageStatus">Click "Take Photo" to capture an image</p>
    </div>
    
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
    
    <div class="info-grid" id="meshNodes" style="display: none;">
      <h2>Mesh Network Nodes</h2>
      <div id="nodesList"></div>
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
      <h2>Performance Optimization</h2>
      <p>Tune these settings to optimize AI detection speed vs quality:</p>
      
      <div class="form-group">
        <label for="aiTimeout">AI Timeout (ms):</label>
        <input type="number" id="aiTimeout" value="6000" min="2000" max="30000" step="1000">
        <small>Lower = faster but may timeout, Higher = more reliable but slower</small>
      </div>
      
      <div class="form-group">
        <label for="aiQuality">AI Image Quality:</label>
        <input type="number" id="aiQuality" value="8" min="1" max="63">
        <small>Lower = smaller file/faster upload, Higher = better quality</small>
      </div>
      
      <div class="form-group">
        <label for="aiFrameSize">AI Frame Size:</label>
        <select id="aiFrameSize">
          <option value="6">VGA (640x480) - Fastest</option>
          <option value="7" selected>SVGA (800x600) - Balanced</option>
          <option value="8">XGA (1024x768) - Better Quality</option>
        </select>
      </div>
      
      <div class="form-group">
        <label for="aiConfidenceThreshold">Confidence Threshold:</label>
        <input type="number" id="aiConfidenceThreshold" value="0.4" min="0.1" max="1.0" step="0.1">
        <small>Lower = more sensitive, Higher = more precise</small>
      </div>
      
      <div class="form-group">
        <button onclick="savePerformanceConfig()" class="btn-warning">Save Performance Settings</button>
        <button onclick="testPerformance()" class="btn-info">Test Performance</button>
      </div>
      
      <div id="performanceStatus" style="margin-top: 10px;"></div>
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
    function captureImage() {
      console.log('Capturing image');
      document.getElementById('imageStatus').textContent = 'Capturing image...';
      document.getElementById('stream').style.display = 'none';
      
      // Add timestamp to prevent caching
      const timestamp = new Date().getTime();
      const img = document.getElementById('stream');
      
      img.onload = function() {
        document.getElementById('imageStatus').textContent = 'Image captured successfully';
        img.style.display = 'block';
      };
      
      img.onerror = function() {
        document.getElementById('imageStatus').textContent = 'Failed to capture image';
        img.style.display = 'none';
      };
      
      img.src = '/capture?' + timestamp;
    }
    
    function refreshStatus() {
      fetch('/status')
        .then(response => response.json())
        .then(data => {
          document.getElementById('detection').textContent = data.person_detected ? 'DETECTED!' : 'Normal';
          document.getElementById('detection').className = 'status-value ' + (data.person_detected ? 'detected' : 'normal');
          document.getElementById('battery').textContent = data.battery + 'V';
          document.getElementById('lastSeen').textContent = data.last_detection;
        })
        .catch(error => {
          console.error('Status error:', error);
          document.getElementById('battery').textContent = 'Error';
        });
    }
    
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
          
          // Show mesh nodes if any exist
          if (data.connected_nodes > 0) {
            document.getElementById('meshNodes').style.display = 'block';
            refreshMeshNodes();
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
    
    function testAIDetection() {
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
    
    function savePerformanceConfig() {
      const timeout = document.getElementById('aiTimeout').value;
      const quality = document.getElementById('aiQuality').value;
      const frameSize = document.getElementById('aiFrameSize').value;
      const threshold = document.getElementById('aiConfidenceThreshold').value;
      
      document.getElementById('performanceStatus').innerHTML = '⚙️ Saving performance settings...';
      
      const data = {
        ai_timeout: parseInt(timeout),
        ai_quality: parseInt(quality),
        ai_frame_size: parseInt(frameSize),
        ai_threshold: parseFloat(threshold)
      };
      
      fetch('/config-performance', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify(data)
      })
      .then(response => response.json())
      .then(result => {
        if (result.success) {
          document.getElementById('performanceStatus').innerHTML = '<span class="success">✅ Performance settings saved!</span>';
        } else {
          document.getElementById('performanceStatus').innerHTML = '<span class="error">❌ Failed to save settings</span>';
        }
      })
      .catch(error => {
        console.error('Performance config error:', error);
        document.getElementById('performanceStatus').innerHTML = '<span class="error">❌ Error saving settings</span>';
      });
    }
    
    function testPerformance() {
      document.getElementById('performanceStatus').innerHTML = '⚡ Testing AI detection speed...';
      const startTime = Date.now();
      
      fetch('/test-detection')
        .then(response => response.json())
        .then(result => {
          const endTime = Date.now();
          const duration = endTime - startTime;
          
          if (result.success) {
            const status = result.person_detected ? '✅ Person detected!' : '❌ No person detected';
            const confidence = result.confidence ? ' (confidence: ' + result.confidence.toFixed(2) + ')' : '';
            document.getElementById('performanceStatus').innerHTML = 
              '<span class="' + (result.person_detected ? 'success' : 'info') + '">' + 
              status + confidence + ' in ' + duration + 'ms</span>';
          } else {
            document.getElementById('performanceStatus').innerHTML = 
              '<span class="error">❌ Performance test failed: ' + result.error + '</span>';
          }
        })
        .catch(error => {
          const endTime = Date.now();
          const duration = endTime - startTime;
          console.error('Performance test error:', error);
          document.getElementById('performanceStatus').innerHTML = 
            '<span class="error">❌ Performance test failed after ' + duration + 'ms</span>';
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
    }    // Auto-refresh functions
    setInterval(refreshStatus, 5000);        // Every 5 seconds for status
    setInterval(refreshSystemInfo, 30000);   // Every 30 seconds for system info
    
    // Initial loads
    refreshStatus();
    refreshSystemInfo();
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
void handleCapture() {
  Serial.println("Image capture requested");
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed!");
    server.send(500, "text/plain", "Camera capture failed");
    return;
  }

  // Person detection is now handled continuously in the main loop
  
  Serial.printf("Sending image: %dx%d, %u bytes\n", fb->width, fb->height, fb->len);
  server.sendHeader("Content-Type", "image/jpeg");
  server.sendHeader("Content-Length", String(fb->len));
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "-1");
  
  server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  Serial.println("Image sent successfully");
}

// Handle status requests
void handleStatus() {
  // Don't log every status request to reduce serial spam
  static unsigned long lastStatusLog = 0;
  if (millis() - lastStatusLog > 10000) { // Log every 10 seconds
    Serial.println("Status requested");
    lastStatusLog = millis();
  }
  
  float battery = getBatteryVoltage();
  String lastDetection = "Never";
  
  if (last_detection > 0) {
    unsigned long timeSince = (millis() - last_detection) / 1000;
    if (timeSince < 60) {
      lastDetection = String(timeSince) + "s ago";
    } else if (timeSince < 3600) {
      lastDetection = String(timeSince / 60) + "m ago";
    } else {
      lastDetection = String(timeSince / 3600) + "h ago";
    }
  }
  
  // Reset person detected flag after 5 seconds
  if (person_detected && (millis() - last_detection) > 5000) {
    person_detected = false;
  }
  
  // Use more efficient JSON building
  server.send(200, "application/json", 
    "{\"person_detected\":" + String(person_detected ? "true" : "false") + 
    ",\"battery\":" + String(battery, 2) + 
    ",\"last_detection\":\"" + lastDetection + "\"}");
}

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

// Handle AI connection test
void handleTestAI() {
  HTTPClient http;
  WiFiClient client;
  
  String url = "http://" + ai_server_ip + ":" + String(ai_server_port) + "/";
  
  http.begin(client, url);
  http.setTimeout(5000); // 5 second timeout for test
  
  int httpCode = http.GET();
  String response = "{";
  
  if (httpCode > 0) {
    String serverResponse = http.getString();
    response += "\"success\":true,";
    response += "\"message\":\"Connection successful\",";
    response += "\"http_code\":" + String(httpCode) + ",";
    response += "\"server_info\":\"" + serverResponse.substring(0, 100) + "\""; // First 100 chars
  } else {
    response += "\"success\":false,";
    response += "\"error\":\"Connection failed (HTTP " + String(httpCode) + ")\",";
    response += "\"http_code\":" + String(httpCode);
  }
  
  response += "}";
  http.end();
  
  server.send(200, "application/json", response);
  Serial.printf("AI connection test: %s - HTTP %d\n", ai_server_ip.c_str(), httpCode);
}

// Handle AI detection test with current camera frame
void handleTestAIDetection() {
  Serial.println("Testing AI detection with current camera frame...");
  
  // Configure camera for faster AI detection
  configureCameraForAI();
  delay(100); // Let camera adjust
  
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    configureCameraForWeb(); // Restore camera settings
    server.send(500, "application/json", "{\"success\":false,\"error\":\"Failed to capture image\"}");
    return;
  }
  
  Serial.printf("Captured test frame: %dx%d, %d bytes\n", fb->width, fb->height, fb->len);
  
  bool detected = detectPersonWithAI(fb);
  esp_camera_fb_return(fb);
  
  // Restore camera to high quality
  configureCameraForWeb();
  
  String response = "{";
  response += "\"success\":true,";
  response += "\"person_detected\":" + String(detected ? "true" : "false") + ",";
  response += "\"confidence\":" + String(detection_confidence, 2) + ",";
  response += "\"message\":\"" + String(detected ? "Person detected!" : "No person detected") + "\"";
  response += "}";
  
  server.send(200, "application/json", response);
}

// Handle performance configuration updates
void handlePerformanceConfig() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"success\":false,\"error\":\"No data received\"}");
    return;
  }
  
  String body = server.arg("plain");
  Serial.printf("Performance config received: %s\n", body.c_str());
  
  // Parse JSON manually for performance settings
  if (body.indexOf("ai_timeout") > -1) {
    int start = body.indexOf("ai_timeout") + 12;
    int end = body.indexOf(",", start);
    if (end == -1) end = body.indexOf("}", start);
    if (end > start) {
      ai_timeout = body.substring(start, end).toInt();
    }
  }
  
  if (body.indexOf("ai_quality") > -1) {
    int start = body.indexOf("ai_quality") + 12;
    int end = body.indexOf(",", start);
    if (end == -1) end = body.indexOf("}", start);
    if (end > start) {
      ai_image_quality = body.substring(start, end).toInt();
    }
  }
  
  if (body.indexOf("ai_frame_size") > -1) {
    int start = body.indexOf("ai_frame_size") + 15;
    int end = body.indexOf(",", start);
    if (end == -1) end = body.indexOf("}", start);
    if (end > start) {
      int frameVal = body.substring(start, end).toInt();
      switch(frameVal) {
        case 6: ai_frame_size = FRAMESIZE_VGA; break;
        case 7: ai_frame_size = FRAMESIZE_SVGA; break;
        case 8: ai_frame_size = FRAMESIZE_XGA; break;
        default: ai_frame_size = FRAMESIZE_SVGA; break;
      }
    }
  }
  
  if (body.indexOf("ai_threshold") > -1) {
    int start = body.indexOf("ai_threshold") + 14;
    int end = body.indexOf(",", start);
    if (end == -1) end = body.indexOf("}", start);
    if (end > start) {
      ai_confidence_threshold = body.substring(start, end).toFloat();
    }
  }
  
  // Save to preferences
  preferences.begin("esp32cam", false);
  preferences.putInt("ai_timeout", ai_timeout);
  preferences.putInt("ai_quality", ai_image_quality);
  preferences.putInt("ai_frame_size", (int)ai_frame_size);
  preferences.putFloat("ai_threshold", ai_confidence_threshold);
  preferences.end();
  
  Serial.printf("Performance settings saved - Timeout: %dms, Quality: %d, Frame: %d, Threshold: %.2f\n", 
               ai_timeout, ai_image_quality, (int)ai_frame_size, ai_confidence_threshold);
  
  String response = "{\"success\":true,\"message\":\"Performance settings saved\"}";
  server.send(200, "application/json", response);
}

// Handle mesh nodes list request
void handleMeshNodes() {
  String response = "{\"nodes\":[";
  
  bool first = true;
  for (int i = 0; i < nodeCount; i++) {
    if (!first) response += ",";
    first = false;
    
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
    
    response += "{";
    response += "\"id\":" + String(meshNodes[i].nodeId) + ",";
    response += "\"type\":\"" + nodeTypeStr + "\",";
    response += "\"online\":" + String(meshNodes[i].isOnline ? "true" : "false") + ",";
    response += "\"battery\":\"" + String(meshNodes[i].batteryLevel, 2) + "\",";
    response += "\"rssi\":" + String(meshNodes[i].rssi) + ",";
    response += "\"lastSeen\":\"" + lastSeen + "\",";
    response += "\"personDetected\":" + String(meshNodes[i].personDetected ? "true" : "false");
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
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("=== ESP32-CAM Solar Person Detection with Mesh ===");

  // Load saved configuration
  loadConfiguration();

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
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  int wifi_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_attempts < 20) {
    delay(1000);
    Serial.print(".");
    wifi_attempts++;
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi connection failed! Starting without mesh...");
    // Continue without mesh functionality but still start web server
  } else {
    Serial.println();
    Serial.println("WiFi connected!");
    Serial.print("Camera ready at: http://");
    Serial.println(WiFi.localIP());
    
#if ENABLE_MESH_NETWORK
    // Now initialize mesh network after WiFi is established
    if (!initMeshNetwork()) {
      Serial.println("❌ Mesh network initialization failed, continuing without mesh");
    } else {
      Serial.println("✅ Mesh network initialized successfully");
      
      // Start mesh discovery after a delay
      delay(1000);
      Serial.println("🔍 Starting mesh discovery...");
      broadcastDiscovery();
    }
#else
    Serial.println("📡 Mesh networking disabled in configuration");
#endif
  }

  // Configure web server routes (always do this, regardless of WiFi status)
  server.on("/", handleRoot);
  server.on("/capture", handleCapture);
  server.on("/stream", handleStream);
  server.on("/status", handleStatus);
  server.on("/sysinfo", handleSysInfo);
  server.on("/config", handleConfig);
  server.on("/test-ai", handleTestAI);
  server.on("/test-detection", handleTestAIDetection);
  server.on("/config-performance", HTTP_POST, handlePerformanceConfig);
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
  
  // Only do mesh operations if mesh is available and enabled
#if ENABLE_MESH_NETWORK
  if (WiFi.status() == WL_CONNECTED) {
    // Check if we should become mesh master
    checkBecomeMaster();
    
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
  
  // Continuous person detection - this runs every 10 seconds
  checkForPerson();
  
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
  
  delay(10); // Reduced delay for more responsive detection
}
