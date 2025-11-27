# Known Issues - ROS2 Humble Multi-Vehicle Architecture

## Cosmetic Errors (No Functional Impact)

### 1. Action Feedback Message Introspection Errors

**Symptoms:**
```
[ERROR] [get_message_class]: Malformed msg message_type: mission_search_interfaces/action/NavigateToTarget_FeedbackMessage
[ERROR] [get_message_class]: Malformed msg message_type: mission_search_interfaces/action/SearchArea_FeedbackMessage
[ERROR] [get_message_class]: Malformed msg message_type: mission_search_interfaces/action/TrackTarget_FeedbackMessage
[ERROR] [get_message_class]: Malformed msg message_type: mission_search_interfaces/action/ExecuteMission_FeedbackMessage
```

**Root Cause:**
- ROS2 Humble's message introspection API (`get_message_class`) has a known limitation with auto-generated action feedback message types
- Action interfaces automatically generate wrapped message types: `<ActionName>_FeedbackMessage`, `<ActionName>_GetResult_Request`, etc.
- The introspection system tries to dynamically discover these types but fails due to API mismatch

**Impact:**
- ✅ **NO functional impact** - action servers and clients work perfectly
- ✅ Mission coordination system operates normally
- ✅ All action communication functions correctly
- ⚠️ Error messages appear when introspection tools (rqt, `ros2 topic list`) scan topics

**Status:**
- This is a **known ROS2 Humble limitation**
- Fixed in later ROS2 distributions (Iron, Jazzy)
- No workaround needed - errors are cosmetic only

**Verification:**
```bash
# Verify action servers are working
ros2 action list

# Expected output (no errors):
/Drone1/actions/navigate_to_target
/Drone1/actions/search_area
/Drone1/actions/track_target
/mission_coordinator/actions/execute_mission

# Test action execution
ros2 action send_goal /Drone1/actions/navigate_to_target mission_search_interfaces/action/NavigateToTarget "..."
```

---

### 2. RQT SubscriberPlugin Deprecated API Warning

**Symptoms:**
```
[ERROR] [rqt_gui_cpp_node]: SubscriberPlugin::subscribeImpl with five arguments has not been overridden
```

**Root Cause:**
- RQT plugin using deprecated ROS2 API
- Plugin compatibility issue between RQT and ROS2 Humble

**Impact:**
- ✅ **NO functional impact**
- RQT visualization tools work normally
- Error is logged but doesn't break functionality

**Status:**
- RQT upstream issue
- No action required

---

### 3. XDG_RUNTIME_DIR Warning (FIXED)

**Symptoms:**
```
QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-dockeruser'
```

**Status:**
- ✅ **FIXED** in Dockerfile.x11 (lines 271-275)
- `XDG_RUNTIME_DIR` now set to `/tmp/runtime-dockeruser`
- Runtime directory created with proper permissions (700)

---

## Summary

All reported errors are **cosmetic** and do **NOT** affect:
- ✅ Multi-vehicle coordination
- ✅ Action server/client communication
- ✅ Mission planning and execution
- ✅ Target tracking and navigation
- ✅ ROS2 topic/service communication

If you encounter these errors in logs, **they can be safely ignored**. The system is functioning correctly.

---

## Additional Information

### Mission Search Interfaces Status
- Package: `mission_search_interfaces`
- Build Status: ✅ Successful (builds in ~16s)
- Action Definitions: 4 actions (NavigateToTarget, SearchArea, TrackTarget, ExecuteMission)
- Message Definitions: 8 messages
- Service Definitions: 6 services

### Multi-Vehicle Architecture Verification
```bash
# List all active action servers
ros2 action list

# List all mission topics
ros2 topic list | grep mission

# Check vehicle nodes
ros2 node list | grep -E "Drone|mission"

# Monitor action feedback (works despite introspection errors)
ros2 topic echo /Drone1/actions/navigate_to_target/_action/feedback
```

All commands above should work without functional issues, despite the cosmetic error messages.

---

**Last Updated:** 2025-10-15
**ROS2 Version:** Humble
**Docker Image:** airsim-vnc-ros2:latest
