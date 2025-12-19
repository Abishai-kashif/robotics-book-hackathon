# Data Model: Module 4 Vision-Language-Action (VLA) Systems

## Overview
This document defines the key data structures and entities for the Vision-Language-Action (VLA) systems covered in Module 4, maintaining consistency with the educational approach established in Modules 1-3.

## Core Entities

### VLAInteraction
Represents a complete Vision-Language-Action interaction cycle in the educational content.

- **input_command** (string): Natural language command input from user (e.g., "Pick up the red ball")
- **visual_input** (string): Description of visual scene input to the system
- **parsed_intent** (string): Parsed intent from natural language processing
- **action_sequence** (array): Sequence of robotic actions to execute
- **execution_result** (object): Result of action execution with success/failure status
- **feedback** (string): System feedback to user about the interaction

### VisionLanguageModel
Represents the vision-language model component in VLA systems.

- **model_name** (string): Name of the vision-language model (e.g., "CLIP", "BLIP-2")
- **input_modalities** (array): Supported input types (e.g., ["image", "text"])
- **output_format** (string): Format of model output (e.g., "embedding", "classification", "caption")
- **confidence_threshold** (float): Minimum confidence for reliable outputs
- **processing_latency** (float): Expected processing time in seconds

### ActionPlan
Represents a structured plan for robotic actions based on VLA system output.

- **plan_id** (string): Unique identifier for the action plan
- **tasks** (array): List of tasks to be executed in sequence
- **prerequisites** (array): Conditions that must be met before execution
- **safety_constraints** (array): Safety requirements during execution
- **success_criteria** (array): Conditions for plan completion
- **recovery_procedures** (array): Actions to take if plan fails

### MultimodalInput
Represents combined input from multiple modalities in VLA systems.

- **text_input** (string): Natural language input from user
- **visual_input** (string): Image or scene description input
- **audio_input** (string): Audio input if speech recognition is used
- **context_data** (object): Additional contextual information
- **timestamp** (datetime): When the input was received
- **input_source** (string): Source of the input (e.g., "user", "environment")

### SimulationScenario
Represents a simulation scenario for demonstrating VLA concepts.

- **scenario_id** (string): Unique identifier for the scenario
- **environment** (string): Simulation environment (e.g., "Gazebo", "Isaac Sim", "Unity")
- **robot_model** (string): Type of robot used in scenario
- **objects** (array): Objects present in the environment
- **tasks** (array): Tasks to be performed in the scenario
- **evaluation_metrics** (array): Metrics for assessing performance

### ROS2Integration
Represents ROS 2 integration points for VLA systems.

- **node_name** (string): Name of the ROS 2 node
- **topics** (array): List of ROS 2 topics used by the system
- **services** (array): List of ROS 2 services used by the system
- **actions** (array): List of ROS 2 actions used by the system
- **message_types** (array): Custom message types if any
- **parameters** (object): ROS 2 parameters for the node

## Relationships

### VLAInteraction -> VisionLanguageModel
- One VLAInteraction uses one VisionLanguageModel for processing
- The VLAInteraction references the model via parsed_intent

### VLAInteraction -> ActionPlan
- One VLAInteraction generates one ActionPlan for execution
- The ActionPlan is derived from parsed_intent and visual_input

### VLAInteraction -> MultimodalInput
- One VLAInteraction processes one MultimodalInput
- The MultimodalInput provides the initial input_command and visual_input

### ActionPlan -> SimulationScenario
- One ActionPlan executes within one SimulationScenario
- The SimulationScenario provides the environment for plan execution

### VLAInteraction -> ROS2Integration
- One VLAInteraction operates within ROS 2 framework via ROS2Integration
- The ROS2Integration handles communication with other robot systems

## Validation Rules

### VLAInteraction Validation
- input_command must be non-empty string
- visual_input must be provided for action-based commands
- action_sequence must contain at least one valid action
- execution_result must include success status

### ActionPlan Validation
- tasks array must not be empty
- each task must have defined success criteria
- safety_constraints must be specified for physical actions
- recovery_procedures must be defined for critical tasks

### VisionLanguageModel Validation
- confidence_threshold must be between 0.0 and 1.0
- input_modalities must include at least one supported modality
- processing_latency must be positive

### MultimodalInput Validation
- at least one input modality must be provided
- timestamp must be current or past
- input_source must be valid (user, environment, etc.)

## State Transitions

### VLAInteraction States
1. **Received** - Input command and visual data received
2. **Processing** - Vision-language model processing input
3. **Planning** - Action plan being generated
4. **Executing** - Action plan being executed
5. **Completed** - Action plan completed successfully
6. **Failed** - Action plan failed or was interrupted

### ActionPlan States
1. **Created** - Plan structure defined
2. **Validated** - Plan validated against safety constraints
3. **Scheduled** - Plan scheduled for execution
4. **Executing** - Individual tasks being executed
5. **Completed** - All tasks completed successfully
6. **Aborted** - Plan execution stopped due to failure