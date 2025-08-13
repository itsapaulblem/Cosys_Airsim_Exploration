# AirSim SetAltitude Service

This document describes the new `SetAltitude` service that allows you to control drone altitude in AirSim.

## Service Definition

**Service Name:** `SetAltitude`  
**Package:** `airsim_interfaces/srv`  
**Topic:** `/airsim_node/{vehicle_name}/set_altitude`

### Request Fields

```
float64 z                    # Target Z position in meters (NED coordinates: negative is up)
float64 velocity            # Movement velocity in m/s (must be > 0, defaults to 5.0 if invalid)
string vehicle_name         # Name of the vehicle (e.g., "Drone1")
bool wait_on_last_task      # Whether to wait for the altitude change to complete
```

### Response Fields

```
bool success                # True if the altitude change was successful
string message              # Status message with details about the operation
```

## Coordinate System

AirSim uses the **NED (North-East-Down)** coordinate system:
- **Positive Z** points downward (toward the ground)
- **Negative Z** points upward (away from the ground)
- **Z = 0** is typically the ground level or takeoff position

### Altitude Examples
- `z = -10.0` → 10 meters above ground
- `z = -20.0` → 20 meters above ground  
- `z = -5.0` → 5 meters above ground

## Usage Examples

### 1. Python Service Client

```python
import rclpy
from rclpy.node import Node
from airsim_interfaces.srv import SetAltitude

class AltitudeClient(Node):
    def __init__(self):
        super().__init__('altitude_client')
        self.client = self.create_client(SetAltitude, '/airsim_node/Drone1/set_altitude')
    
    def set_altitude(self, z, velocity=5.0):
        request = SetAltitude.Request()
        request.z = z
        request.velocity = velocity
        request.vehicle_name = 'Drone1'
        request.wait_on_last_task = True
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

# Usage
rclpy.init()
client = AltitudeClient()

# Move to 15 meters altitude
response = client.set_altitude(-15.0, velocity=8.0)
if response.success:
    print("Altitude changed successfully!")
```

### 2. Command Line Interface

Use the provided CLI tool:

```bash
# Move to 10 meters altitude with 5 m/s velocity, wait for completion
python3 set_altitude_cli.py -z -10.0 -v 5.0 --drone Drone1 --wait

# Move to 20 meters altitude asynchronously (don't wait)
python3 set_altitude_cli.py -z -20.0 -v 3.0 --drone Drone1 --no-wait

# Quick command (uses defaults: 5 m/s velocity, waits for completion)
python3 set_altitude_cli.py -z -15.0
```

### 3. ROS2 Command Line

```bash
# Call service directly with ros2 service call
ros2 service call /airsim_node/Drone1/set_altitude airsim_interfaces/srv/SetAltitude \
  "{z: -12.0, velocity: 6.0, vehicle_name: 'Drone1', wait_on_last_task: true}"
```

### 4. C++ Service Client

```cpp
#include <rclcpp/rclcpp.hpp>
#include <airsim_interfaces/srv/set_altitude.hpp>

class AltitudeClient : public rclcpp::Node {
public:
    AltitudeClient() : Node("altitude_client") {
        client_ = this->create_client<airsim_interfaces::srv::SetAltitude>("/airsim_node/Drone1/set_altitude");
    }
    
    bool set_altitude(double z, double velocity = 5.0) {
        auto request = std::make_shared<airsim_interfaces::srv::SetAltitude::Request>();
        request->z = z;
        request->velocity = velocity;
        request->vehicle_name = "Drone1";
        request->wait_on_last_task = true;
        
        auto result = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == 
            rclcpp::FutureReturnCode::SUCCESS) {
            return result.get()->success;
        }
        return false;
    }

private:
    rclcpp::Client<airsim_interfaces::srv::SetAltitude>::SharedPtr client_;
};
```

## Available Tools

The package includes several example tools:

1. **`set_altitude_example.py`** - Comprehensive Python example with multiple scenarios
2. **`set_altitude_cli.py`** - Command-line interface for quick altitude changes

## Service Behavior

- **Synchronous Mode** (`wait_on_last_task = true`): The service waits for the drone to reach the target altitude before responding
- **Asynchronous Mode** (`wait_on_last_task = false`): The service immediately returns after sending the command
- **Error Handling**: The service includes comprehensive error handling and logging
- **Default Velocity**: If velocity ≤ 0, defaults to 5.0 m/s

## Integration with Existing Services

The `SetAltitude` service complements existing AirSim services:

- **`Takeoff`** - Initial takeoff from ground
- **`Land`** - Landing the drone
- **`SetLocalPosition`** - Full 3D position control (X, Y, Z)
- **`SetAltitude`** - Z-axis only control (maintains current X, Y)

## Prerequisites

1. AirSim simulation running
2. ROS2 AirSim wrapper running:
   ```bash
   ros2 launch airsim_ros_pkgs airsim_node.launch.py
   ```
3. Drone in the simulation (typically named "Drone1")

## Troubleshooting

### Service Not Available
```bash
# Check if service exists
ros2 service list | grep set_altitude

# Check service type
ros2 service type /airsim_node/Drone1/set_altitude
```

### Common Issues
- **Service timeout**: Ensure AirSim and the ROS2 wrapper are running
- **Invalid vehicle name**: Check that the drone name matches your AirSim settings
- **Movement fails**: Ensure the drone is armed and in the correct mode (API control enabled)

## Building and Installation

1. Add the service to your workspace
2. Build the interfaces:
   ```bash
   colcon build --packages-select airsim_interfaces
   ```
3. Build the main package:
   ```bash
   colcon build --packages-select airsim_ros_pkgs
   ```
4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Performance Notes

- **Velocity Range**: Recommended 1-15 m/s (too high may cause instability)
- **Altitude Limits**: Respect ground collision and maximum altitude limits
- **Multiple Drones**: Each drone has its own service endpoint
- **Concurrent Calls**: Avoid sending rapid consecutive altitude commands 