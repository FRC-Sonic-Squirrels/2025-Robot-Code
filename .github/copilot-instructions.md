# Copilot Instructions for FRC Team 2930 - Sonic Squirrels

## Project Overview

This is the competition robot code for **FRC Team 2930 (The Sonic Squirrels)** for the **2025 FRC Reefscape** season. The robot is named **"Holocentrus"**. This codebase is designed to be educational and maintainable for high school students learning robotics programming.

**Awards Won:**
- Autonomous Award at GPK
- Innovation in Controls Award at SunDome

---

## Technology Stack

- **Language:** Java 17
- **Build System:** Gradle with GradleRIO
- **Framework:** WPILib Command-Based framework
- **Logging:** AdvantageKit (from Team 6328)
- **Path Planning:** Choreo (for pre-generated trajectories) and PathPlanner
- **Vision:** PhotonVision with AprilTags
- **Motor Controllers:** CTRE Phoenix 6 (TalonFX), REV (SparkMax)
- **Code Style:** Spotless formatter

---

## Project Structure

```
src/main/java/frc/
├── lib/                    # Reusable library code
│   ├── team2930/          # Team-specific utilities (logging, state machines, tuning)
│   └── team6328/          # Utilities adapted from Team 6328 (AdvantageKit patterns)
└── robot/
    ├── Robot.java         # Main robot class (entry point)
    ├── RobotContainer.java # Subsystem initialization, button bindings, auto setup
    ├── Constants.java     # Robot-wide constants and configuration
    ├── RobotStates.java   # Enum-based state definitions for subsystems
    ├── CommandComposer.java # Complex command compositions
    ├── autonomous/        # Autonomous routines and state machines
    ├── commands/          # Command classes organized by subsystem
    ├── configs/           # Robot-specific configurations (different robots)
    └── subsystems/        # Subsystem implementations
```

---

## Architecture Patterns

### 1. IO Abstraction Pattern (AdvantageKit Style)

Every subsystem uses an **IO interface pattern** for hardware abstraction, enabling simulation and replay:

```java
// Interface defines hardware interactions
public interface EndEffectorIO {
    class Inputs extends BaseInputs {
        public double velocityRPM;
        public double currentAmps;
        // ... other sensor values
    }

    void updateInputs(Inputs inputs);
    void setVoltage(double volts);
    void setVelocity(double rpm);
}

// Real implementation for actual hardware
public class EndEffectorIOReal implements EndEffectorIO { ... }

// Simulation implementation
public class EndEffectorIOSim implements EndEffectorIO { ... }

// Subsystem uses the interface
public class EndEffector extends SubsystemBase {
    private final EndEffectorIO io;
    private final EndEffectorIO.Inputs inputs;

    public EndEffector(EndEffectorIO io, RobotStates states) {
        this.io = io;
        // ...
    }
}
```

### 2. State Machine Pattern

Complex behaviors use a custom `StateMachine` class with state handlers:

```java
public class ScoreCoral extends StateMachine {
    public ScoreCoral(...) {
        super("ScoreCoral");
        setInitialState(this::stateStart);
    }

    private StateHandler stateStart() {
        // Perform actions
        return this::stateNext; // Transition to next state
    }
}
```

### 3. Periodic State-Based Control

Subsystems like `Mechanism`, `Intake`, and `EndEffector` use enum-based state machines in their `periodic()` methods:

```java
public void periodic() {
    switch (states.mechState) {
        case Idle:
            elevator.setPercentOut(0);
            arm.setPercentOut(0);
            break;
        case ReefPosition:
            goToPositionParallel(MechanismPositions.reefPosition(states.scoringLevel));
            break;
        // ... other states
    }
}
```

### 4. Separation of Concerns Pattern (EndEffector Example)

When refactoring complex subsystems, separate concerns into dedicated classes:

- **Context class** (`EndEffectorContext`): Snapshot of sensor state, provides clean query interface
- **Algorithm class** (`CoralAligner`): Encapsulates complex multi-phase logic with its own state machine
- **Output record** (`MotorOutput`): Separates "what to do" from "doing it" for testability

Organize `periodic()` into clear phases:
```java
public void periodic() {
    // 1. Read Inputs - io.updateInputs(inputs)
    // 2. Update Context - context.update(inputs, ...)
    // 3. Update Shared State - states.coralInEndEffector = ...
    // 4. Log Inputs - logVelocity.info(inputs.velocityRPM)
    // 5. Update Tunables - if (kP.hasChanged(hc)) setConstants()
    // 6. Run State Machine - MotorOutput output = runStateMachine()
    // 7. Apply Output - applyMotorOutput(output)
}
```

This pattern improves testability (test algorithm without hardware) and readability (each phase has one job).

### 4. Robot Configuration Pattern

Different physical robots share code via configuration classes:

```java
// Abstract base
public abstract class RobotConfig {
    public abstract boolean getPhoenix6Licensed();
    public abstract LoggedTunableNumber getDriveKP();
    // ...
}

// Specific robot implementation
public class RobotConfig2025 extends RobotConfig { ... }
public class SimulatorRobotConfig extends RobotConfig { ... }
```

---

## Key Classes and Their Purposes

| Class | Purpose |
|-------|---------|
| `RobotContainer` | Initializes all subsystems, configures button bindings, sets up autonomous |
| `RobotStates` | Central location for all subsystem state enums (`MechState`, `IntakeState`, etc.) |
| `Constants` | Robot-wide constants, CAN IDs, field positions, game piece locations |
| `Mechanism` | Coordinates elevator and arm movements with collision avoidance |
| `AutoStateMachine` | Controls autonomous routine execution with state-based logic |
| `AutosManager` | Manages auto selection and execution from dashboard |
| `EndEffectorContext` | Sensor state snapshot for EndEffector state machine decisions |
| `CoralAligner` | 4-phase coral alignment algorithm (CENTERING → BACKTRACK → FIND_EDGE → MOVE_TO_TARGET → ALIGNED) |
| `MotorOutput` | Record type representing motor commands (VELOCITY, PERCENT, STOP) |

---

## Logging and Tuning

### Logging with LoggerGroup and LoggerEntry

```java
private static final LoggerGroup logGroup = LoggerGroup.build("SubsystemName");
private static final LoggerEntry.Decimal logVelocity = logGroup.buildDecimal("Velocity");
private static final LoggerEntry.Bool logAtTarget = logGroup.buildBoolean("AtTarget");

// In periodic:
logVelocity.info(getCurrentVelocity());
logAtTarget.info(isAtTarget());
```

### Tunable Numbers for Real-Time Adjustment

```java
private static final TunableNumberGroup group = new TunableNumberGroup("EndEffector");
private static final LoggedTunableNumber kP = group.build("kP", 0.2);
private static final LoggedTunableNumber kV = group.build("kV", 0.115);

// Use in code:
io.setClosedLoopConstants(kP.get(), kV.get(), kS.get(), maxAccel.get());
```

---

## Subsystem Overview

### Drive (Swerve)
- 4-module swerve drive with TalonFX motors
- Dual gyros (Pigeon2) for redundancy
- `DrivetrainWrapper` provides speed-limited interface based on elevator height

### Vision
- PhotonVision cameras on Orange Pi coprocessors
- AprilTag pose estimation with Kalman filtering
- Vision-based auto-alignment for scoring

### Mechanism (Elevator + Arm)
- Coordinated motion with collision detection
- Segmented motion planning to avoid self-collision
- Position presets defined in `MechanismPositions`

### End Effector
- Coral/Algae manipulation
- Time-of-Flight sensors for game piece detection
- Alignment routines using motor position feedback
- Uses `EndEffectorContext` for sensor state, `CoralAligner` for alignment algorithm, `MotorOutput` for motor commands

### Intake
- Ground pickup for coral and algae
- Pass-off coordination with end effector

### LED
- Visual feedback for robot state
- Different patterns for scoring, intaking, disabled, etc.

---

## Autonomous System

### Structure
- Uses **Choreo** for pre-planned trajectories (100+ trajectory segments)
- **State machine** (`AutoStateMachine`) sequences actions:
  1. Drive to scoring position
  2. Score game piece
  3. Drive to pickup location
  4. Intake game piece
  5. Repeat

### Custom Auto Selection
- Dashboard choosers for starting position, scoring locations, pickup locations
- Flexible "procedural" auto that dynamically selects paths

---

## Coding Guidelines for This Project

### When Adding a New Subsystem

1. Create the IO interface with an `Inputs` class extending `BaseInputs`
2. Create `IOReal` implementation for hardware
3. Create `IOSim` implementation for simulation
4. Create the subsystem class extending `SubsystemBase`
5. Add state enum to `RobotStates` if needed
6. Instantiate in `RobotContainer` with appropriate IO based on robot type

### When Adding a New Command

1. Place in appropriate subfolder under `commands/`
2. Use subsystem methods, don't directly access IO
3. For complex sequences, consider using `StateMachine`
4. Compose with `Commands.sequence()`, `Commands.parallel()`, etc.

### When Modifying Constants

1. Put magic numbers in `Constants.java` inner classes
2. Use `LoggedTunableNumber` for values that may need adjustment
3. Group related tunables with `TunableNumberGroup`

### State Enum Naming Conventions

```java
public enum MechState {
    Idle,           // Default resting state
    Override,       // Manual control
    ReefPosition,   // Game-specific positions
    StowPosition,   // Safe transport position
    // ...
}
```

---

## Common Patterns to Follow

### Creating Commands That Require Multiple Subsystems

```java
// In CommandComposer.java or RobotContainer
public Command scoreAndRetract(Mechanism mech, EndEffector endEffector) {
    return Commands.sequence(
        new MechToPosition(mech, MechanismPositions.scoringPosition()),
        new EndEffectorSetRPM(endEffector, -3000).withTimeout(0.5),
        new MechToPosition(mech, MechanismPositions.stowPosition())
    );
}
```

### Handling Different Robot Types

```java
switch (robotType) {
    case ROBOT_2025_HOLO:
        intake = new Intake(new IntakeIOReal(), robotStates);
        break;
    case ROBOT_SIMBOT:
        intake = new Intake(new IntakeIOSim(), robotStates);
        break;
    default:
        intake = new Intake(new IntakeIO() {}, robotStates); // Empty/replay
}
```

---

## Tips for High School Students

1. **Start with `RobotContainer`** - This is where everything connects
2. **Follow existing patterns** - Copy similar subsystems when creating new ones
3. **Use the simulation** - Test code without the robot using `ROBOT_SIMBOT`
4. **Log everything** - Use `LoggerEntry` to track values in AdvantageScope
5. **Tune carefully** - Use `LoggedTunableNumber` so you can adjust values without redeploying
6. **Test incrementally** - Small changes, test often

---

## Build and Deploy Commands

```bash
# Build the project
./gradlew build

# Deploy to robot
./gradlew deploy

# Run simulation
./gradlew simulateJava

# Format code
./gradlew spotlessApply
```

---

## External Resources

- [WPILib Documentation](https://docs.wpilib.org/)
- [AdvantageKit Documentation](https://github.com/Mechanical-Advantage/AdvantageKit)
- [Choreo Path Planner](https://github.com/SleipnirGroup/Choreo)
- [PhotonVision](https://photonvision.org/)
- [CTRE Phoenix 6](https://pro.docs.ctr-electronics.com/)
- [REV Robotics](https://docs.revrobotics.com/)
