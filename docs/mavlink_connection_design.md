# MavLinkCom Connection Design

## Overview

This document provides a detailed analysis of the connection design and networking architecture within the MavLinkCom library. It focuses on the transport layer implementation, connection management strategies, and network protocol handling.

## Table of Contents

1. [Connection Architecture](#connection-architecture)
2. [Transport Layer Design](#transport-layer-design)
3. [Connection Lifecycle](#connection-lifecycle)
4. [Protocol Implementation](#protocol-implementation)
5. [Multi-Connection Management](#multi-connection-management)
6. [Network Configuration](#network-configuration)
7. [Error Handling and Resilience](#error-handling-and-resilience)
8. [Performance Considerations](#performance-considerations)

## Connection Architecture

### Hierarchical Design

```
MavLinkConnection (Public Interface)
├── MavLinkConnectionImpl (Private Implementation)
│   ├── MessageQueue (Thread-safe message handling)
│   ├── SubscriberManager (Observer pattern implementation)
│   └── Port (Transport abstraction)
│       ├── UdpClientPort (UDP implementation)
│       ├── TcpClientPort (TCP implementation)
│       └── SerialPort (Serial implementation)
└── ConnectionState (Connection lifecycle management)
```

### Key Design Principles

1. **Transport Abstraction**: Unified interface for different transport types
2. **Thread Safety**: All operations are thread-safe with proper synchronization
3. **Asynchronous I/O**: Non-blocking operations with event-driven architecture
4. **Resource Management**: RAII principles for automatic cleanup
5. **Extensibility**: Easy to add new transport types

## Transport Layer Design

### Port Abstraction

The `Port` class provides a unified interface for all transport types:

```cpp
class Port {
public:
    virtual ~Port() = default;
    
    // Connection management
    virtual void connect() = 0;
    virtual void close() = 0;
    virtual bool isClosed() const = 0;
    
    // I/O operations
    virtual int read(uint8_t* buffer, int bytesToRead) = 0;
    virtual int write(const uint8_t* buffer, int bytesToWrite) = 0;
    
    // Configuration
    virtual void configure(const std::map<std::string, std::string>& settings) = 0;
    
    // Status information
    virtual std::string getDisplayName() const = 0;
    virtual bool isConnected() const = 0;
};
```

### UDP Implementation

#### UdpClientPort Design

```cpp
class UdpClientPort : public Port {
private:
    SOCKET socket_;
    sockaddr_in local_addr_;
    sockaddr_in remote_addr_;
    bool is_listening_;
    std::mutex socket_mutex_;
    
public:
    // Constructor variants
    UdpClientPort();  // For listening
    UdpClientPort(const std::string& localAddr, int localPort);
    UdpClientPort(const std::string& localAddr, int localPort,
                  const std::string& remoteAddr, int remotePort);
    
    // Port interface implementation
    void connect() override;
    void close() override;
    int read(uint8_t* buffer, int bytesToRead) override;
    int write(const uint8_t* buffer, int bytesToWrite) override;
};
```

#### UDP Connection Modes

1. **Local Listener Mode**:
   - Binds to local address and port
   - Accepts incoming connections from any source
   - Automatically learns remote endpoint from first message

```cpp
// Creates listener on 127.0.0.1:14550
auto connection = MavLinkConnection::connectLocalUdp("127.0.0.1", 14550);
```

2. **Remote Client Mode**:
   - Connects to specific remote endpoint
   - Sends initial heartbeat to establish connection
   - Maintains connection state

```cpp
// Connects to remote endpoint
auto connection = MavLinkConnection::connectRemoteUdp(
    "127.0.0.1", 14550,     // Local binding
    "192.168.1.100", 14560  // Remote target
);
```

### TCP Implementation

#### TcpClientPort Design

```cpp
class TcpClientPort : public Port {
private:
    SOCKET socket_;
    sockaddr_in local_addr_;
    sockaddr_in remote_addr_;
    bool is_connected_;
    std::mutex socket_mutex_;
    std::atomic<bool> auto_reconnect_;
    
public:
    TcpClientPort(const std::string& localAddr, int localPort,
                  const std::string& remoteAddr, int remotePort);
    
    // Port interface implementation
    void connect() override;
    void close() override;
    int read(uint8_t* buffer, int bytesToRead) override;
    int write(const uint8_t* buffer, int bytesToWrite) override;
    
    // TCP-specific features
    void setAutoReconnect(bool enable);
    bool isAutoReconnectEnabled() const;
};
```

#### TCP Features

1. **Reliable Delivery**: TCP guarantees ordered, reliable message delivery
2. **Connection State**: Maintains persistent connection state
3. **Auto-Reconnect**: Automatic reconnection on connection loss
4. **Flow Control**: Built-in TCP flow control

### Serial Implementation

#### SerialPort Design

```cpp
class SerialPort : public Port {
private:
    std::string port_name_;
    int baud_rate_;
    HANDLE handle_;  // Windows
    int fd_;         // Linux
    bool is_open_;
    std::mutex port_mutex_;
    
public:
    SerialPort(const std::string& portName, int baudRate = 57600);
    
    // Port interface implementation
    void connect() override;
    void close() override;
    int read(uint8_t* buffer, int bytesToRead) override;
    int write(const uint8_t* buffer, int bytesToWrite) override;
    
    // Serial-specific configuration
    void setBaudRate(int baudRate);
    void setDataBits(int dataBits);
    void setParity(Parity parity);
    void setStopBits(StopBits stopBits);
};
```

#### Serial Configuration

```cpp
// Basic serial connection
auto connection = MavLinkConnection::connectSerial("/dev/ttyUSB0", 57600);

// Advanced configuration
auto serialPort = std::make_shared<SerialPort>("/dev/ttyUSB0", 115200);
serialPort->setDataBits(8);
serialPort->setParity(Parity::None);
serialPort->setStopBits(StopBits::One);
```

## Connection Lifecycle

### State Machine

```
     [CREATED]
         │
         ▼
   [CONNECTING] ──────────► [FAILED]
         │                     ▲
         ▼                     │
   [CONNECTED] ────────────────┘
         │
         ▼
   [DISCONNECTING]
         │
         ▼
   [DISCONNECTED]
```

### Lifecycle Management

```cpp
enum class ConnectionState {
    Created,
    Connecting,
    Connected,
    Disconnecting,
    Disconnected,
    Failed
};

class MavLinkConnectionImpl {
private:
    std::atomic<ConnectionState> state_;
    std::condition_variable state_changed_;
    std::mutex state_mutex_;
    
public:
    void connect() {
        setState(ConnectionState::Connecting);
        try {
            port_->connect();
            startReaderThread();
            setState(ConnectionState::Connected);
        } catch (const std::exception& e) {
            setState(ConnectionState::Failed);
            throw;
        }
    }
    
    void waitForState(ConnectionState targetState, float timeout) {
        std::unique_lock<std::mutex> lock(state_mutex_);
        auto deadline = std::chrono::steady_clock::now() + 
                       std::chrono::duration<float>(timeout);
        
        return state_changed_.wait_until(lock, deadline, [&]() {
            return state_.load() == targetState;
        });
    }
};
```

## Protocol Implementation

### MAVLink Frame Processing

```cpp
class MavLinkFrameProcessor {
private:
    mavlink_status_t status_;
    mavlink_message_t message_;
    uint8_t buffer_[MAVLINK_MAX_PACKET_LEN];
    
public:
    bool processFrame(uint8_t byte, MavLinkMessage& outMessage) {
        if (mavlink_frame_char_buffer(&message_, &status_, byte, 
                                     buffer_, MAVLINK_MAX_PACKET_LEN)) {
            // Complete message received
            outMessage.decode(message_);
            return true;
        }
        return false;
    }
};
```

### Message Flow

```
Raw Bytes ──► Frame Parser ──► Message Decoder ──► Message Router ──► Subscribers
     ▲              │                   │                │               │
     │              ▼                   ▼                ▼               ▼
   Socket      Checksum Validation  Type Validation  Filter Logic   User Handlers
```

### Protocol Features

1. **Checksum Validation**: Automatic verification of message integrity
2. **Sequence Tracking**: Detection of lost messages
3. **Version Compatibility**: Support for MAVLink v1 and v2
4. **Message Filtering**: Configurable message filtering
5. **Signing Support**: Message authentication (MAVLink v2)

## Multi-Connection Management

### Connection Bridging

```cpp
class ConnectionBridge {
private:
    std::vector<std::shared_ptr<MavLinkConnection>> connections_;
    std::mutex bridge_mutex_;
    bool enabled_;
    
public:
    void addConnection(std::shared_ptr<MavLinkConnection> conn) {
        std::lock_guard<std::mutex> lock(bridge_mutex_);
        connections_.push_back(conn);
        
        // Subscribe to forward messages
        conn->subscribe([this, conn](const MavLinkMessage& msg) {
            forwardMessage(msg, conn);
        });
    }
    
private:
    void forwardMessage(const MavLinkMessage& msg, 
                       std::shared_ptr<MavLinkConnection> source) {
        if (!enabled_) return;
        
        std::lock_guard<std::mutex> lock(bridge_mutex_);
        for (auto& target : connections_) {
            if (target != source && !target->isClosed()) {
                target->sendMessage(msg);
            }
        }
    }
};
```

### Use Cases

1. **Autopilot Bridge**: Connect serial autopilot to UDP ground station
2. **Telemetry Splitter**: Distribute telemetry to multiple consumers
3. **Protocol Gateway**: Convert between different transport types
4. **Message Logger**: Tap into message stream for logging

### Example: Serial to UDP Bridge

```cpp
// Create connections
auto serialConn = MavLinkConnection::connectSerial("/dev/ttyUSB0", 57600);
auto udpConn = MavLinkConnection::connectLocalUdp("0.0.0.0", 14550);

// Create bridge
auto bridge = std::make_shared<ConnectionBridge>();
bridge->addConnection(serialConn);
bridge->addConnection(udpConn);

// Start both connections
serialConn->startListening();
udpConn->startListening();
```

## Network Configuration

### IP Address Configuration

```cpp
class NetworkConfig {
public:
    struct UdpConfig {
        std::string local_address = "127.0.0.1";
        int local_port = 14550;
        std::string remote_address;
        int remote_port = 14560;
        bool broadcast = false;
        int receive_buffer_size = 65536;
        int send_buffer_size = 65536;
    };
    
    struct TcpConfig {
        std::string local_address = "127.0.0.1";
        int local_port = 0;  // Auto-assign
        std::string remote_address;
        int remote_port = 5760;
        bool auto_reconnect = true;
        int connect_timeout_ms = 5000;
        int keepalive_interval_ms = 30000;
    };
};
```

### Multi-Interface Support

```cpp
// Bind to multiple interfaces
std::vector<std::shared_ptr<MavLinkConnection>> connections;

// Local loopback
connections.push_back(
    MavLinkConnection::connectLocalUdp("127.0.0.1", 14550));

// All interfaces
connections.push_back(
    MavLinkConnection::connectLocalUdp("0.0.0.0", 14551));

// Specific interface
connections.push_back(
    MavLinkConnection::connectLocalUdp("192.168.1.100", 14552));
```

### Firewall Considerations

```cpp
// Configure for Windows Firewall
auto config = UdpConfig{};
config.local_address = "0.0.0.0";     // Bind to all interfaces
config.local_port = 14550;            // Standard GCS port
config.broadcast = true;              // Enable broadcast
```

## Error Handling and Resilience

### Connection Monitoring

```cpp
class ConnectionMonitor {
private:
    std::shared_ptr<MavLinkConnection> connection_;
    std::atomic<bool> is_healthy_;
    std::chrono::steady_clock::time_point last_heartbeat_;
    std::thread monitor_thread_;
    
public:
    ConnectionMonitor(std::shared_ptr<MavLinkConnection> conn) 
        : connection_(conn), is_healthy_(false) {
        
        // Subscribe to heartbeat messages
        connection_->subscribe([this](const MavLinkMessage& msg) {
            if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                last_heartbeat_ = std::chrono::steady_clock::now();
                is_healthy_ = true;
            }
        });
        
        // Start monitoring thread
        monitor_thread_ = std::thread(&ConnectionMonitor::monitorLoop, this);
    }
    
private:
    void monitorLoop() {
        while (!stop_requested_) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                now - last_heartbeat_).count();
            
            if (elapsed > 5) {  // 5 second timeout
                is_healthy_ = false;
                onConnectionLost();
            }
            
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    
    void onConnectionLost() {
        // Attempt reconnection
        if (connection_->isClosed()) {
            try {
                connection_->reconnect();
            } catch (const std::exception& e) {
                // Log error and retry later
            }
        }
    }
};
```

### Retry Logic

```cpp
class RetryPolicy {
public:
    struct Config {
        int max_attempts = 3;
        std::chrono::milliseconds initial_delay{1000};
        double backoff_multiplier = 2.0;
        std::chrono::milliseconds max_delay{30000};
    };
    
    template<typename Func>
    auto execute(Func&& func, const Config& config = {}) {
        int attempt = 0;
        auto delay = config.initial_delay;
        
        while (attempt < config.max_attempts) {
            try {
                return func();
            } catch (const std::exception& e) {
                attempt++;
                if (attempt >= config.max_attempts) {
                    throw;
                }
                
                std::this_thread::sleep_for(delay);
                delay = std::min(
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        delay * config.backoff_multiplier),
                    config.max_delay);
            }
        }
    }
};
```

### Graceful Degradation

```cpp
class FallbackConnection {
private:
    std::vector<std::shared_ptr<MavLinkConnection>> connections_;
    std::shared_ptr<MavLinkConnection> active_connection_;
    
public:
    void addConnection(std::shared_ptr<MavLinkConnection> conn, int priority) {
        connections_.insert(
            connections_.begin() + priority, conn);
    }
    
    void sendMessage(const MavLinkMessage& msg) {
        for (auto& conn : connections_) {
            if (!conn->isClosed()) {
                try {
                    conn->sendMessage(msg);
                    active_connection_ = conn;
                    return;
                } catch (const std::exception& e) {
                    // Try next connection
                    continue;
                }
            }
        }
        throw std::runtime_error("All connections failed");
    }
};
```

## Performance Considerations

### Buffer Management

```cpp
class CircularBuffer {
private:
    std::vector<uint8_t> buffer_;
    size_t head_ = 0;
    size_t tail_ = 0;
    size_t size_ = 0;
    std::mutex mutex_;
    
public:
    CircularBuffer(size_t capacity) : buffer_(capacity) {}
    
    size_t write(const uint8_t* data, size_t length) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        size_t available = buffer_.size() - size_;
        size_t to_write = std::min(length, available);
        
        for (size_t i = 0; i < to_write; ++i) {
            buffer_[tail_] = data[i];
            tail_ = (tail_ + 1) % buffer_.size();
        }
        
        size_ += to_write;
        return to_write;
    }
    
    size_t read(uint8_t* data, size_t length) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        size_t to_read = std::min(length, size_);
        
        for (size_t i = 0; i < to_read; ++i) {
            data[i] = buffer_[head_];
            head_ = (head_ + 1) % buffer_.size();
        }
        
        size_ -= to_read;
        return to_read;
    }
};
```

### Threading Strategy

```cpp
class ConnectionThreadManager {
private:
    std::thread reader_thread_;
    std::thread writer_thread_;
    std::thread processor_thread_;
    
    ThreadSafeQueue<std::vector<uint8_t>> read_queue_;
    ThreadSafeQueue<MavLinkMessage> write_queue_;
    ThreadSafeQueue<MavLinkMessage> process_queue_;
    
public:
    void start() {
        reader_thread_ = std::thread(&ConnectionThreadManager::readerLoop, this);
        writer_thread_ = std::thread(&ConnectionThreadManager::writerLoop, this);
        processor_thread_ = std::thread(&ConnectionThreadManager::processorLoop, this);
    }
    
private:
    void readerLoop() {
        uint8_t buffer[1024];
        while (!stop_requested_) {
            int bytes_read = port_->read(buffer, sizeof(buffer));
            if (bytes_read > 0) {
                read_queue_.push(std::vector<uint8_t>(buffer, buffer + bytes_read));
            }
        }
    }
    
    void writerLoop() {
        while (!stop_requested_) {
            MavLinkMessage msg;
            if (write_queue_.pop(msg, std::chrono::milliseconds(100))) {
                sendMessage(msg);
            }
        }
    }
    
    void processorLoop() {
        while (!stop_requested_) {
            std::vector<uint8_t> data;
            if (read_queue_.pop(data, std::chrono::milliseconds(100))) {
                processRawData(data);
            }
        }
    }
};
```

### Memory Management

```cpp
// Object pooling for message objects
class MessagePool {
private:
    std::queue<std::unique_ptr<MavLinkMessage>> pool_;
    std::mutex pool_mutex_;
    std::atomic<size_t> allocated_count_{0};
    static constexpr size_t MAX_POOL_SIZE = 1000;
    
public:
    std::unique_ptr<MavLinkMessage> acquire() {
        std::lock_guard<std::mutex> lock(pool_mutex_);
        
        if (!pool_.empty()) {
            auto msg = std::move(pool_.front());
            pool_.pop();
            return msg;
        }
        
        allocated_count_++;
        return std::make_unique<MavLinkMessage>();
    }
    
    void release(std::unique_ptr<MavLinkMessage> msg) {
        if (!msg) return;
        
        msg->reset();  // Clear message data
        
        std::lock_guard<std::mutex> lock(pool_mutex_);
        if (pool_.size() < MAX_POOL_SIZE) {
            pool_.push(std::move(msg));
        } else {
            allocated_count_--;
        }
    }
};
```

## Conclusion

The MavLinkCom connection design provides:

1. **Unified Transport Interface**: Consistent API across UDP, TCP, and Serial
2. **Robust Connection Management**: Automatic reconnection and error recovery
3. **High Performance**: Optimized threading and memory management
4. **Flexible Configuration**: Support for various network topologies
5. **Multi-Connection Support**: Bridging and fallback capabilities
6. **Comprehensive Monitoring**: Connection health and performance metrics

This design enables reliable, high-performance MAVLink communication suitable for both real-world and simulation environments, with particular strength in the multi-vehicle scenarios common in AirSim deployments.