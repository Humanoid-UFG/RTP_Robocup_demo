# RoboCup Demo - Booster T1

## Testing on Robot

The primary objective of this project is to execute a complete process demonstration of Booster T1 robot operating autonomously within the RoboCup environment. This includes all phases from sensing to decision-making and action execution during a match.

## Additional Dependency Installation

To ensure correct system operation, install the following dependency:

```bash
sudo apt-get install ros-humble-backward-ros
```

## Docker Setup for RoboCup Demo

This repository provides a **Docker-based setup** for the **robocup_demo** project, optimized for **NVIDIA Jetson** devices with **CUDA** support. Custom `Dockerfile` and `docker-compose.yml` files have been created to facilitate the startup of system components, including `robocup_demo` and the `ZED camera`.

### Docker Project Structure
The containerized structure of the project ensures a consistent and isolated environment for development and execution.
*   **`dockerfile.jetson`**: The main base image, built on **JetPack 6.2**, containing all necessary libraries to run `robocup_demo`.
*   **`docker-compose.yml`**: Orchestrates the simultaneous execution of both the `robocup_demo` and `zed` containers.
*   **`robocup_demo` container**: Contains the project source code and its dependencies.
*   **`zed` container**: Runs the ZED camera system, based on version **5.0**, compatible with JetPack 6.2.

### Prerequisites
*   [**Docker** and **Docker Compose** installed](https://www.cytron.io/tutorial/docker-setup-for-jetson-orin-nano-super-jp6.2?srsltid=AfmBOoobhJK4ev8m_QYQQtgzMrM9xdgf6AkJuBkOsLImfvIJPU8Zg)

> ## ⚠️ Important Docker Notes
> 
> 1.  **Image Build:**
>     A direct Docker build will fail due to missing CUDA/NVIDIA libraries at build time. The correct procedure is as follows:
>     1.  Build the base image using `dockerfile.jetson`.
>     2.  Enter the container.
>     3.  Build the repository manually inside the container.
>     4.  Perform a `docker commit` to create the final image.
> 
> 2.  **ZED Integration:**
>     When starting the `robocup_demo` service via `docker-compose`, the `zed` service is also automatically launched, as the demo depends on ZED camera data  

## Build and Run (Docker)

### 1. Build Jetson Images

Build the Docker images for the Jetson environment:
```bash
docker buildx build -f dockerfile.jetson -t robocup_demo_image
docker buildx build -f dockerfile.zed -t zed
```

> This creates the base images. The robocup_demo image will be finalized after compilation in step 4.

### 2. Enter the `robocup_demo` Container

Start the container in interactive mode:
```bash
docker compose run robocup_demo bash
```

### 3. Build Repository Inside the Container

Once inside the container, compile the project:
```bash
source /opt/ros/humble/setup.bash
./scripts/build.sh
```

Verify the build was successful:
```bash
ros2 pkg list | grep robocup
```
You should see the robocup packages listed.

**Exit the container** after successful compilation:
```bash
exit
```

### 4. Commit the Compiled Container

> This step must be done in a **new terminal** on the host machine (not inside the container).

Find the container ID and commit the changes:
```bash
# List running containers
docker ps

# Commit the container with compiled code
docker commit <CONTAINER_ID> robocup_demo_image
```

> This saves the compiled state into the image. If you skip this step, the compilation will be lost.

### 5. Run the System via Docker Compose

Start the complete system:
```bash
# Clean up any previous instances
docker compose down

# Start robocup_demo
docker compose up robocup_demo
```

## Rebuilding After Code Changes

If you modify the source code:
```bash
# 1. Enter container
docker compose run robocup_demo bash

# 2. Rebuild
source /opt/ros/humble/setup.bash
./scripts/build.sh
exit

# 3. Commit changes (in new terminal)
docker ps
docker commit <CONTAINER_ID> robocup_demo_image

# 4. Restart services
docker compose down
docker compose up robocup_demo
```

---

# Local Execution (Non-Docker)
If you prefer not to use Docker, you can build and run the programs directly on the host system.

## Vision Configuration for JetPack

This repository supports both JetPack 6.0 and 6.2. It is critical to correctly configure the TRT model in the `vision.yaml` file according to the installed JetPack version.

To check your JetPack version, execute:

```bash
dpkg -l | grep jetpack
```

#### `src/vision/config/vision.yaml`

**For JetPack 6.0:**
```yaml
detection_model:
  model_path: "./src/vision/model/best_orin.engine"
  confidence_threshold: 0.2
```

**For JetPack 6.2:**
```yaml
detection_model:
  model_path: "./src/vision/model/best_orin_10.3.engine"
  confidence_threshold: 0.2
```

## Determine Field Dimensions

Accurate field dimensions are crucial for the `Brain` module's positioning and distance calculations. If the field size designated for testing deviates from the default configurations, the relevant parameters within the `Brain` must be updated.

Before deployment, the following field dimensions (in meters) must be measured and confirmed:

*   **Field Length**
*   **Field Width**
*   **Distance from the Penalty Spot to the Goal Line**
*   **Goal Width**
*   **Center Circle Radius**
*   **Penalty Area Length**
*   **Penalty Area Width**
*   **Goal Area Length**
*   **Goal Area Width**

These values are configured as constants within the `Brain` module's source code, specifically in `robocup_demo/src/brain/include/types.h`. By default, the system utilizes constants defined for `FD_ADULTSIZE`. Should the tested field dimensions differ from these default values, the corresponding constants must be modified accordingly in `types.h`. 
You can create a customized variable like: 
```bash 
const FieldDimensions FD_CUSTOM{ }`
```

### Build the Programs

```bash
./scripts/build.sh
```

### Run on the Robot

To start the system on Booster T1 robot:
```bash
./scripts/start.sh
```

## Robot Positioning and Operation Instructions

In a standard 2v2 RoboCup match, each team typically consists of one striker and one goalkeeper. The robot's role and initial position must be configured prior to deployment.

### Role and Initial Positioning Configuration

When starting the robot, its designated `player_role` and `player_start_pos` must be specified in the configuration file.

> *   **Pre-match Placement:** Before the match commences, robots from both teams are placed at approximate predefined positions on the field. The robots will then utilize their onboard cameras to recognize ground markings for precise self-localization. The default configuration is set for a **left striker**. During initial testing, ensure the robot is placed at the designated position for the left striker.

### Operation Process

Follow these steps to transition the robot through its various operational modes for testing:

1.  **Enter Preparation Mode:**
    *   Initiate by pressing **Joystick RT + Y**.

2.  **Field Placement:**
    *   Remove the robot from its safety stand.
    *   Carefully position the robot on the sideline of the field, at the designated **left striker** starting position.

3.  **Switch to RoboCup Gait Mode:**
    *   Transition to the appropriate movement gait by pressing **Joystick LT + RT + A**.

4.  **Enter Localization State:**
    *   Activate the localization process by pressing **Joystick LT + A**.
    *   The robot's head will begin to rotate, actively searching for the ball. Once the ball is detected, the robot will maintain a persistent gaze on it. If the ball's position is manually altered, the robot's head will adjust to maintain its gaze.
>    *   **Troubleshooting:** If the robot's head continues to rotate indefinitely without locking onto the ball, this indicates a localization issue that must be resolved before proceeding.

5.  **Enter Walking Mode:**
    *   Enable the robot's walking capabilities by pressing **Joystick RT + A**.

6.  **Enter Match Mode (AI Control):**
    *   Transition the robot into autonomous AI control for the match by pressing **Joystick LT + B**.
    *   **Manual Override:** To disable AI control and revert to manual operation, press **Joystick LT + X**.
>   *   **Note:** After canceling AI control, the robot may continue to execute the last command issued by the AI (e.g., continued movement). Manual control can be regained by manipulating the joystick.

7.  **AI Control and GameController Integration:**
    *   If all steps are executed successfully, the robot will now be under full AI control and will await and respond to instructions received from the `GameController` to execute further match actions.

## Oficial Documentation

For more detailed information regarding the project, please refer to the complete documentation:

[English Version](https://booster.feishu.cn/wiki/XY6Kwrq1bizif4kq7X9c14twnle)
