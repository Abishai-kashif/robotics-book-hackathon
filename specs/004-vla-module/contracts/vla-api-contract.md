# API Contract: Vision-Language-Action (VLA) System Integration

## Overview
This document defines the API contracts for the Vision-Language-Action (VLA) system components that will be covered in Module 4 of the Physical AI & Humanoid Robotics textbook.

## ROS 2 Message Definitions

### VLACommand.msg
Defines the structure for commands sent to the VLA system.

```
# Natural language command string
string command

# Optional context data
string context

# Timestamp of command
builtin_interfaces/Time timestamp

# Command priority (0-10, where 10 is highest)
uint8 priority
```

### VLAActionResult.msg
Defines the structure for results from VLA action execution.

```
# Unique identifier for the action
string action_id

# Success status of the action
bool success

# Result message
string message

# Execution time in seconds
float32 execution_time

# Confidence level of the result (0.0-1.0)
float32 confidence

# Error code if action failed (0 = success)
int32 error_code
```

### VLAPerceptionData.msg
Defines the structure for perception data from vision systems.

```
# Image data from camera
sensor_msgs/Image camera_image

# Object detections in the scene
object_recognition_msgs/RecognizedObjectArray objects

# Scene description in natural language
string scene_description

# Timestamp of perception data
builtin_interfaces/Time timestamp
```

## ROS 2 Service Definitions

### VLAPlanAction.srv
Service for planning actions based on vision-language input.

**Request:**
```
# Natural language command
string command

# Perception data
VLAPerceptionData perception_data

# Action constraints
string[] constraints
```

**Response:**
```
# Success status
bool success

# Action plan as sequence of commands
string[] action_sequence

# Estimated execution time
float32 estimated_time

# Confidence in plan (0.0-1.0)
float32 confidence

# Error message if failed
string error_message
```

### VLACalibrate.srv
Service for calibrating the VLA system.

**Request:**
```
# Calibration type
string calibration_type

# Calibration parameters
string[] parameters
```

**Response:**
```
# Success status
bool success

# Calibration result
string result

# Error message if failed
string error_message
```

## ROS 2 Action Definitions

### VLAExecuteAction.action
Action for executing a complete VLA task with feedback.

**Goal:**
```
# Natural language command to execute
string command

# Action timeout in seconds
float32 timeout

# Safety constraints
string[] safety_constraints
```

**Result:**
```
# Success status
bool success

# Final result message
string result_message

# Execution metrics
float32 execution_time
float32 success_rate
```

**Feedback:**
```
# Current status of execution
string status

# Progress percentage (0-100)
uint8 progress

# Current action being executed
string current_action

# Estimated time remaining
float32 time_remaining
```

## Topic Definitions

### Published Topics

- **`/vla/command_result`** - `VLAActionResult` - Results of VLA command execution
- **`/vla/system_status`** - `std_msgs/String` - Current status of the VLA system
- **`/vla/action_feedback`** - `VLAActionResult` - Feedback during action execution

### Subscribed Topics

- **`/vla/command`** - `VLACommand` - Commands to be processed by the VLA system
- **`/vla/perception_data`** - `VLAPerceptionData` - Perception data from vision systems
- **`/vla/user_feedback`** - `std_msgs/String` - User feedback for the VLA system

### Services

- **`/vla/plan_action`** - `VLAPlanAction` - Plan actions based on vision-language input
- **`/vla/calibrate`** - `VLACalibrate` - Calibrate the VLA system
- **`/vla/interrupt`** - `std_srvs/Trigger` - Interrupt current VLA action

## Action Servers

- **`/vla/execute_action`** - `VLAExecuteAction` - Execute complete VLA tasks

## Quality of Service (QoS) Profiles

### Command Topics
- Reliability: Reliable
- Durability: Volatile
- History: Keep Last
- Depth: 10

### Perception Topics
- Reliability: Best Effort (for performance)
- Durability: Volatile
- History: Keep Last
- Depth: 1 (for latest data)

### Result Topics
- Reliability: Reliable
- Durability: Volatile
- History: Keep Last
- Depth: 5

## Validation Rules

### VLACommand Validation
- command string must not be empty
- priority must be between 0 and 10
- timestamp must be current or past

### VLAPerceptionData Validation
- camera_image must be valid image format
- scene_description must be non-empty when objects are detected
- timestamp must be current

### VLAActionResult Validation
- confidence must be between 0.0 and 1.0
- execution_time must be non-negative
- error_code must be 0 for successful actions

## Error Handling

### Standard Error Codes
- 0: Success
- 1: General failure
- 2: Invalid command
- 3: Perception failure
- 4: Action planning failure
- 5: Execution timeout
- 6: Safety constraint violation
- 7: Calibration required
- 8: Hardware failure
- 9: Communication error

### Error Response Format
All services and actions follow this error response pattern:
- success: false
- error_message: Human-readable error description
- error_code: Standard error code from above list

## Performance Requirements

### Response Times
- VLAPlanAction service: < 2 seconds
- VLAActionResult publication: < 100ms after execution
- Perception data processing: < 500ms

### Throughput
- Handle up to 10 concurrent commands
- Process perception data at 10Hz minimum
- Support 1000+ commands per hour

## Security Considerations

### Authentication
- All services require ROS 2 authentication when security is enabled
- Command validation to prevent malicious inputs

### Authorization
- Only authorized nodes can publish to critical topics
- Action execution requires appropriate permissions

### Data Protection
- Personal data in voice/image inputs must be handled according to privacy policies
- Log data containing personal information should be anonymized