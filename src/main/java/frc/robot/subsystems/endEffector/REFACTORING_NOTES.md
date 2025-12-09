# EndEffector Refactoring Notes

> ⚠️ **EXPERIMENTAL** - This refactor was "vibe coded" as a thought experiment to share with students and give them ideas on how they might approach refactoring this code on their own. While this compiles and passes tests, it has NOT been deployed to the 2025 season robot to verify real-world functionality.

## Overview

This document explains the refactoring changes made to the EndEffector subsystem to improve code organization, readability, and testability.

## Problem: Mixed Concerns

The original `EndEffector.periodic()` method had a ~100 line switch statement that mixed several concerns:
- Reading sensor values
- Making state decisions
- Controlling motors
- Managing simulation state
- Logging

This made the code difficult to understand, test, and maintain.

## Solution: Separation of Concerns

We extracted distinct responsibilities into dedicated classes:

### 1. `EndEffectorContext` - Sensor State Snapshot

**Purpose:** Provides a clean interface between the state machine and hardware/sensor state.

```java
// Before: Direct sensor access scattered throughout
if (scoringSideTofSeenGamepiece() && nonScoringSideTOFSeenGamepiece()) { ... }

// After: Clean queries on context object
if (context.hasCoralBothSides()) { ... }
```

**Key methods:**
- `hasCoralScoringSide()` - Is coral detected on scoring side?
- `hasCoralNonScoringSide()` - Is coral detected on non-scoring side?
- `hasAnyCoral()` - Either sensor detecting?
- `hasCoralBothSides()` - Both sensors detecting?
- `getMotorPosition()` - Current motor position for alignment

### 2. `CoralAligner` - Alignment Algorithm

**Purpose:** Encapsulates the 4-phase coral alignment sequence with its own state machine.

**Alignment Phases:**
1. **CENTERING** - Move coral until non-scoring sensor sees it
2. **BACKTRACK** - Back up until non-scoring sensor loses it
3. **FIND_EDGE** - Move forward until non-scoring sensor sees it again (records zero position)
4. **MOVE_TO_TARGET** - Move to target offset from zero position
5. **ALIGNED** - Maintain position based on scoring level (L1-L4)

```java
// Before: Alignment logic spread across multiple switch cases
case AlignCoral:
case AlignCoralPhase2:
case AlignCoralPhase3:
case AlignCoralPhase4:
case AlignedCoral:

// After: Single class manages all alignment
CoralAligner aligner = new CoralAligner();
AlignResult result = aligner.update(context);
```

### 3. `MotorOutput` - Motor Command Abstraction

**Purpose:** Separates "what to do" from "doing it" for testability.

```java
// Before: Direct motor calls in switch cases
setVelocity(scoringVelocityRPM.get());

// After: Return a command object, apply later
return MotorOutput.velocity(scoringVelocityRPM.get());
// ... later ...
applyMotorOutput(output);
```

**Types:**
- `MotorOutput.velocity(rpm)` - Run at specific RPM
- `MotorOutput.percent(pct)` - Run at percentage of max voltage
- `MotorOutput.stop()` - Stop the motor

## Refactored `periodic()` Structure

The new `periodic()` method is organized into 7 clear phases:

```java
public void periodic() {
    // 1. Read Inputs
    io.updateInputs(inputs);

    // 2. Update Context
    context.update(inputs, states.scoringLevel);

    // 3. Update Shared State
    updateSharedCoralState();

    // 4. Log Inputs
    logAllInputs();

    // 5. Update Tunable Numbers
    updateTunableConstants();

    // 6. Run State Machine & Get Motor Output
    MotorOutput output = runStateMachine();

    // 7. Apply Motor Output
    applyMotorOutput(output);
}
```

## Additional Improvements

### Consolidated Scoring Methods

Before: Two nearly identical methods
```java
private MotorOutput handleScoreFastForward() { ... }
private MotorOutput handleScoreFastBackward() { ... }
```

After: Single parameterized method
```java
private MotorOutput handleScoreFast(int direction) {
    if (!context.hasAnyCoral()) {
        states.endEffectorDesiredAction = EndEffectorDesiredAction.Idle;
    }
    clearCoralInSim();
    return MotorOutput.velocity(direction * scoringVelocityRPM.get());
}
```

### Fixed Missing ScoreAlgae Case

The original switch statement didn't handle `ScoreAlgae`, which was used by `RobotContainer` for barge scoring. Now properly handled.

### Safety Limit on State Machine Loop

Added a maximum iteration limit (4) with error logging to catch potential infinite loops:

```java
final int maxLoops = 4; // Expected max is 2-3 transitions
while (true) {
    output = processState(desiredAction);
    if (states.endEffectorDesiredAction == desiredAction) break;
    if (++loopCount >= maxLoops) {
        System.err.println("EndEffector state machine exceeded max iterations!");
        break;
    }
}
```

### Simulation Logic Encapsulated

Moved simulation-specific coral clearing to `EndEffectorIOSim.clearCoralDetection()` instead of directly manipulating fields.

## Benefits

1. **Testability** - `CoralAligner` can be unit tested without hardware
2. **Readability** - Each phase/class has one job
3. **Maintainability** - Changes to alignment don't affect other parts
4. **Debugging** - Clearer logging and state tracking

## Files Changed

| File | Change |
|------|--------|
| `EndEffector.java` | Major refactor of `periodic()` |
| `EndEffectorContext.java` | **NEW** - Sensor state snapshot |
| `CoralAligner.java` | **NEW** - Alignment algorithm |
| `MotorOutput.java` | **NEW** - Motor command record |
| `EndEffectorIOSim.java` | Added `clearCoralDetection()` |

## For Students

This refactoring demonstrates several software engineering principles:

1. **Single Responsibility Principle** - Each class does one thing well
2. **Separation of Concerns** - Different aspects of the problem are handled separately
3. **Immutability** - `MotorOutput` is a record (immutable)
4. **Encapsulation** - `CoralAligner` hides its internal state machine

Consider applying similar patterns to other complex subsystems like `Mechanism` or `Intake`!
