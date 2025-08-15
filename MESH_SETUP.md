# ESP32-CAM Mesh Network Setup Guide

## Overview
This enhanced ESP32-CAM system creates a self-organizing mesh network for distributed person detection using CodeProject.AI. The first device becomes the mesh master, and subsequent devices automatically join the network and receive configuration.

## 🔧 Mesh Network Features

### ✅ **Automatic Role Assignment**
- **First Device**: Automatically becomes the **Master Node** (Node ID: 1)
- **Additional Devices**: Become **Slave Nodes** with auto-assigned IDs
- **Relay Capability**: Nodes can relay messages through the mesh

### ✅ **Self-Configuration**
- New devices scan for existing master for 30 seconds
- If master found: Join mesh and receive all configuration
- If no master: Become new master node
- Configuration includes WiFi credentials and AI server settings

### ✅ **Mesh Communication (ESP-NOW)**
- **Discovery**: Broadcast to find mesh master
- **Join Process**: Automatic joining with configuration transfer
- **Heartbeat**: Regular status updates (every 60 seconds)
- **Detection Alerts**: Person detection shared across mesh
- **Configuration Updates**: Master can broadcast new settings

### ✅ **Robust Networking**
- **Multi-hop Routing**: Messages relay through intermediate nodes
- **Node Timeout**: Offline detection after 5 minutes
- **Mesh Healing**: Automatic route discovery
- **Battery Monitoring**: Shared across network

## 📱 Web Interface Enhancements

### **Mesh Network Information**
- Node Type (Master/Slave/Relay)
- Node ID and Master ID
- Connected Nodes Count
- Real-time mesh status

### **Master Node Controls**
- Broadcast configuration to all nodes
- Scan for new mesh devices
- Reset entire mesh network
- View all connected nodes with status

### **Node Status Display**
For each mesh node:
- Online/Offline status
- Battery level
- Signal strength (RSSI)
- Last seen time
- Person detection status

## 🚀 Setup Instructions

### **Step 1: Prepare First Device (Master)**
1. Flash the code to your first ESP32-CAM
2. Configure WiFi credentials in the code
3. Set up CodeProject.AI server details via web interface
4. Power on the device - it will become Master Node (ID: 1)

### **Step 2: Add Additional Devices**
1. Flash the **same code** to additional ESP32-CAM devices
2. Power them on within range of the master
3. They will automatically:
   - Scan for the master node
   - Join the mesh network
   - Receive WiFi and AI server configuration
   - Get assigned unique node IDs

### **Step 3: Mesh Network Expansion**
1. Devices can be placed within ESP-NOW range (~200m outdoors)
2. Out-of-range devices will relay through intermediate nodes
3. Maximum 10 nodes supported (configurable)
4. Each node operates independently but shares alerts

## 📡 Mesh Communication Protocol

### **Message Types**
- `DISCOVERY`: Find existing mesh master
- `JOIN_REQUEST`: Request to join mesh
- `JOIN_RESPONSE`: Master response with configuration
- `CONFIG_UPDATE`: Broadcast new settings
- `DETECTION_ALERT`: Person detection notification
- `HEARTBEAT`: Regular status update
- `STATUS_REQUEST/RESPONSE`: Request node information

### **Network Topology**
```
Master Node (1) ←→ Slave Node (2) ←→ Slave Node (3)
     ↕                    ↕
Slave Node (4)      Slave Node (5)
```

## 🔧 Configuration Management

### **Master Node Capabilities**
- Configure AI server settings via web interface
- Broadcast configuration to all mesh nodes
- View status of all connected nodes
- Reset entire mesh network
- Initiate discovery scans

### **Slave Node Capabilities**
- Automatically receive configuration from master
- Send detection alerts to mesh
- Relay messages for out-of-range nodes
- Report status via heartbeat

## 🔋 Power Management

### **Solar Power Considerations**
- Mesh heartbeat reduces to conserve battery
- Deep sleep coordination across mesh
- Battery levels shared across network
- Low battery alerts propagated to master

### **Communication Efficiency**
- ESP-NOW uses less power than WiFi
- Mesh messages are lightweight (~200 bytes)
- Heartbeat interval: 60 seconds
- Node timeout: 5 minutes

## 📊 Monitoring and Debugging

### **Serial Monitor Output**
```
📡 Added new mesh node ID:2, Type:1
📤 Sent mesh message type:4 to node:0
📥 Received message type:5 from node:2 (RSSI:-45)
🚨 MESH ALERT: Person detected by node:3 (confidence:0.85)
👑 Becoming mesh master - no existing master found
```

### **Web Interface Debugging**
- Real-time mesh node status
- Signal strength monitoring
- Message relay statistics
- Network topology visualization

## 🔒 Security Considerations

### **Mesh Security**
- Messages include timestamp verification
- Checksum validation for data integrity
- MAC address based node identification
- Configuration only from verified master

### **Network Isolation**
- Mesh operates on separate ESP-NOW channel
- WiFi credentials shared only within mesh
- AI server configuration protected
- Node authentication via MAC filtering

## ⚠️ Troubleshooting

### **Common Issues**

1. **Node Won't Join Mesh**
   - Check ESP-NOW range (~200m)
   - Verify master is powered and running
   - Check serial monitor for discovery messages

2. **Configuration Not Propagating**
   - Ensure master node has valid AI server settings
   - Use "Broadcast to Mesh" button in web interface
   - Check node connectivity in mesh status

3. **Nodes Going Offline**
   - Check battery levels
   - Verify ESP-NOW range
   - Check for interference on 2.4GHz

4. **Detection Alerts Not Shared**
   - Verify AI server connectivity
   - Check mesh heartbeat status
   - Ensure nodes are properly joined

### **Reset Procedures**

1. **Reset Single Node**: Power cycle the device
2. **Reset Mesh Network**: Use master web interface
3. **Factory Reset**: Clear preferences in code

## 🚀 Advanced Features

### **Future Enhancements**
- Automatic relay node promotion
- Mesh topology optimization
- Load balancing across nodes
- Advanced power management
- Mobile app integration

### **Scalability**
- Current limit: 10 nodes (configurable)
- Range: ~200m per hop
- Coverage: Theoretical unlimited with relays
- Performance: Sub-second alert propagation

## 📝 Code Structure

### **Key Files**
- `main.cpp`: Complete mesh + detection system
- `MESH_SETUP.md`: This setup guide

### **Key Functions**
- `initMeshNetwork()`: Initialize ESP-NOW mesh
- `sendMeshMessage()`: Send messages to mesh
- `onMeshMessageReceived()`: Handle incoming messages
- `checkBecomeMaster()`: Master election logic
- `updateMeshConfig()`: Broadcast configuration

This mesh networking system transforms your ESP32-CAM into a powerful, self-organizing surveillance network that can scale from a single device to a comprehensive area monitoring solution!
