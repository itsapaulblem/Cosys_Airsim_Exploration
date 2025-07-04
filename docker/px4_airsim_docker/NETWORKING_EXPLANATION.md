# AirSim-Docker Networking Explained

## 🤔 Why Setting LocalHostIp to 172.25.0.1 Breaks Detection

When you set `LocalHostIp` to the Docker gateway IP (172.25.0.1), AirSim can't detect drones because you're confusing **two different network connections**.

## 🔄 The Two Network Connections

### Connection 1: AirSim → PX4 (TCP)
- **Purpose**: AirSim sends commands to PX4
- **Direction**: Host (AirSim) → Container (PX4)
- **Setting**: `ControlIp` = Container IP address
- **Example**: `"ControlIp": "172.25.0.10"`

### Connection 2: PX4 → AirSim (MAVLink UDP)  
- **Purpose**: PX4 sends telemetry/GPS data back to AirSim
- **Direction**: Container (PX4) → Host (AirSim)
- **Setting**: `LocalHostIp` = Gateway IP address  
- **Example**: `"LocalHostIp": "172.25.0.1"`

## 🏗️ Network Architecture

```
┌─────────────────────────────────────────────────────┐
│                Docker Network                       │
│                172.25.0.0/16                       │
│                                                     │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐ │
│  │ PX4 Drone1  │  │ PX4 Drone2  │  │ PX4 Drone3  │ │
│  │172.25.0.10  │  │172.25.0.11  │  │172.25.0.12  │ │
│  │             │  │             │  │             │ │
│  │  TCP:4561   │  │  TCP:4562   │  │  TCP:4563   │ │
│  └─────────────┘  └─────────────┘  └─────────────┘ │
│                                                     │
│            Gateway: 172.25.0.1                     │
└─────────────────────────────────────────────────────┘
                         ↑
                         │ (MAVLink messages)
                         │
           ┌─────────────────────────────┐
           │      Windows Host            │
           │                             │
           │      AirSim (Unreal)        │
           │                             │
           │   ← TCP connections to      │
           │     172.25.0.10:4561        │
           │     172.25.0.11:4562        │
           │     172.25.0.12:4563        │
           └─────────────────────────────┘
```

## ❌ What Goes Wrong

### Wrong Configuration:
```json
{
  "ControlIp": "172.25.0.1",      // ❌ Gateway IP
  "LocalHostIp": "172.25.0.1"     // ✅ This part is correct
}
```

**Problem**: AirSim tries to connect to the gateway (172.25.0.1) instead of the actual PX4 container (172.25.0.10). The gateway doesn't have a PX4 instance running, so AirSim can't detect any drones!

## ✅ Correct Configuration  

### Right Configuration:
```json
{
  "ControlIp": "172.25.0.10",     // ✅ Container IP (where PX4 actually is)
  "LocalHostIp": "172.25.0.1"     // ✅ Gateway IP (how PX4 reaches AirSim)
}
```

**Result**: 
1. AirSim connects to 172.25.0.10:4561 and finds PX4 ✅
2. PX4 sends MAVLink messages to 172.25.0.1 and reaches AirSim ✅
3. GPS home location works ✅
4. armDisarm works ✅

## 🔧 How to Fix

Run the fix script:
```bash
python fix_airsim_docker_networking.py
```

This will:
1. Set `ControlIp` to the correct container IPs (172.25.0.10, 172.25.0.11, etc.)
2. Set `LocalHostIp` to the gateway IP (172.25.0.1)
3. Create a backup of your current settings
4. Explain what changed and why

## 🎯 Key Insight

**The confusion comes from thinking both connections use the same IP.** They don't!

- **AirSim needs to know where to FIND PX4** → Container IP
- **PX4 needs to know where to FIND AirSim** → Gateway IP

Think of it like phone calls:
- You call your friend's phone number (container IP)
- Your friend calls you back on your phone number (gateway IP)
- These are different numbers for different directions! 