// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.RobotMode.RobotType;
import frc.robot.RobotStates;
import frc.robot.RobotStates.EndEffectorDesiredAction;
import frc.robot.subsystems.endEffector.CoralAligner.AlignPhase;

public class EndEffector extends SubsystemBase {
  // Execution timing
  private static final ExecutionTiming timing =
      new ExecutionTiming(EndEffectorConstants.ROOT_TABLE);

  // Logging
  private static final LoggerGroup logGroup = LoggerGroup.build(EndEffectorConstants.ROOT_TABLE);
  private static final LoggerEntry.Decimal logInputs_velocityRPM =
      logGroup.buildDecimal("VelocityRPM");
  private static final LoggerEntry.Decimal logInputs_currentAmps =
      logGroup.buildDecimal("CurrentAmps");
  private static final LoggerEntry.Decimal logInputs_tempCelsius =
      logGroup.buildDecimal("TempCelsius");
  private static final LoggerEntry.Decimal logInputs_appliedVolts =
      logGroup.buildDecimal("AppliedVolts");
  private static final LoggerEntry.Decimal logInputs_scoringSideTofDist =
      logGroup.buildDecimal("ToF/ScoringSideTofDist");
  private static final LoggerEntry.Decimal logInputs_nonScoringSideTOFDist =
      logGroup.buildDecimal("ToF/NonScoringSideTOFDist");
  private static final LoggerEntry.Bool logInputs_scoringSideTofActivated =
      logGroup.buildBoolean("ToF/ScoringSideTofActivated");
  private static final LoggerEntry.Bool logInputs_nonScoringSideTOFActivated =
      logGroup.buildBoolean("ToF/NonScoringSideTOFActivated");
  private static final LoggerEntry.Decimal logInputs_scoringSideTofSignalStrength =
      logGroup.buildDecimal("ToF/ScoringSideTofSignalStrength");
  private static final LoggerEntry.Decimal logInputs_nonScoringSideTOFSignalStrength =
      logGroup.buildDecimal("ToF/NonScoringSideTOFSignalStrength");

  private static final LoggerEntry.Decimal logTargetVelocityRPM =
      logGroup.buildDecimal("TargetVelocityRPM");
  private static final LoggerEntry.EnumValue<ControlMode> logControlMode =
      logGroup.buildEnum("ControlMode");
  private static final LoggerEntry.Decimal logVelocityOverride =
      logGroup.buildDecimal("VelocityOverride");

  // Tunable numbers

  private static final TunableNumberGroup group =
      new TunableNumberGroup(EndEffectorConstants.ROOT_TABLE);

  private static final LoggedTunableNumber kS = group.build("kS");
  private static final LoggedTunableNumber kP = group.build("kP");
  private static final LoggedTunableNumber kV = group.build("kV");
  private static final LoggedTunableNumber targetAccelerationConfig =
      group.build("MaxAccelerationConstraint");

  private static final LoggedTunableNumber intakingVelocitySlow =
      group.build("intakingVelocitySlow", 800);
  private static final LoggedTunableNumber intakingVelocityHigh =
      group.build("intakingVelocity", 1500);

  private static final LoggedTunableNumber scoringVelocityRPM =
      group.build("ScoringVelocityRPM", -3000);
  private static final LoggedTunableNumber passOffVelocityRPM =
      group.build("passOffVelocityRPM", -1000);
  private static final LoggedTunableNumber holdAlgaeRPM = group.build("passOffVelocityRPM", -1500);

  // Motion constants

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2024_RETIRED_MAESTRO) {
      kS.initDefault(0);
      kP.initDefault(0.4);
      kV.initDefault(0.15);
      targetAccelerationConfig.initDefault(300.0);
    } else if (Constants.RobotMode.isSimBot()) {
      kS.initDefault(0);
      kP.initDefault(0.0006);
      kV.initDefault(0.0002);
      targetAccelerationConfig.initDefault(0.0);
    } else {
      kS.initDefault(0);
      kP.initDefault(0.2);
      kV.initDefault(0.115);
      targetAccelerationConfig.initDefault(400);
    }
  }

  private final EndEffectorIO io;
  private final RobotStates states;
  private final EndEffectorIO.Inputs inputs = new EndEffectorIO.Inputs(logGroup);

  // Refactored components for separation of concerns
  private final EndEffectorContext context;
  private final CoralAligner aligner = new CoralAligner();

  private ControlMode controlMode = ControlMode.OPEN_LOOP;

  private Trigger stallDetected =
      new Trigger(() -> inputs.velocityRPM <= 50 && inputs.appliedVolts >= 0.5).debounce(.1);

  /** Creates a new EndEffector. */
  public EndEffector(EndEffectorIO io, RobotStates states) {
    this.io = io;
    this.states = states;
    this.context = new EndEffectorContext(states);

    setConstants();

    io.setVoltage(0.0);
  }

  public EndEffectorIOSim getSim() {
    if (io instanceof EndEffectorIOSim) {
      return (EndEffectorIOSim) io;
    }

    return null;
  }

  private int counter = 0;
  private LoggerEntry.Integer counterLog = logGroup.buildInteger("counter");

  @Override
  public void periodic() {
    try (var ignored = timing.start()) {
      counter++;
      counterLog.info(counter % 100);

      // === PHASE 1: Read Inputs ===
      io.updateInputs(inputs);

      // === PHASE 2: Update Context ===
      context.update(inputs, states.scoringLevel);

      // === PHASE 3: Update Shared State ===
      updateSharedCoralState();

      // === PHASE 4: Log Inputs ===
      logAllInputs();

      // === PHASE 5: Update Tunable Numbers ===
      updateTunableConstants();

      // === PHASE 6: Run State Machine & Get Motor Output ===
      MotorOutput output = runStateMachine();

      // === PHASE 7: Apply Motor Output ===
      applyMotorOutput(output);
    }
  }

  // --- Separated Concerns ---

  /** Update shared robot state based on sensor readings. */
  private void updateSharedCoralState() {
    var scoringSide = context.hasCoralScoringSide();
    var nonScoringSide = context.hasCoralNonScoringSide();
    var anyCoral = context.hasAnyCoral();

    states.coralInEndEffectorScoringSide = scoringSide;
    states.coralInEndEffectorNonScoringSide = nonScoringSide;
    states.coralInEndEffector = states.coralExpectedInEndEffector && anyCoral;
  }

  /** Log all input values. */
  private void logAllInputs() {
    logInputs_velocityRPM.info(inputs.velocityRPM);
    logInputs_currentAmps.info(inputs.currentAmps);
    logInputs_tempCelsius.info(inputs.tempCelsius);
    logInputs_appliedVolts.info(inputs.appliedVolts);
    logInputs_scoringSideTofDist.info(inputs.scoringSideTofDistInches);
    logInputs_nonScoringSideTOFDist.info(inputs.nonScoringSideTofDistInches);
    logInputs_scoringSideTofActivated.info(inputs.scoringSideTofDetecting);
    logInputs_nonScoringSideTOFActivated.info(inputs.nonScoringSideTofDetecting);
    logInputs_scoringSideTofSignalStrength.info(inputs.scoringSideSignalStrength);
    logInputs_nonScoringSideTOFSignalStrength.info(inputs.nonScoringSideSignalStrength);
    logControlMode.info(controlMode);
    logVelocityOverride.info(states.endEffectorOverrideVelocity);
  }

  /** Check and apply tunable constant updates. */
  private void updateTunableConstants() {
    var hc = hashCode();
    if (kS.hasChanged(hc)
        || kP.hasChanged(hc)
        || kV.hasChanged(hc)
        || targetAccelerationConfig.hasChanged(hc)) {
      setConstants();
    }
  }

  /**
   * Run the state machine and return the motor output to apply. This separates the decision logic
   * from the motor control.
   */
  private MotorOutput runStateMachine() {
    var desiredAction = states.endEffectorDesiredAction;

    // Handle disabled state
    if (DriverStation.isDisabled()) {
      if (desiredAction != EndEffectorDesiredAction.AlignedCoral) {
        desiredAction = EndEffectorDesiredAction.Idle;
      }
      states.endEffectorOverrideVelocity = Double.NaN;
    }

    // Handle velocity override
    if (Double.isFinite(states.endEffectorOverrideVelocity)) {
      return MotorOutput.velocity(states.endEffectorOverrideVelocity);
    }

    // Process state machine with potential immediate transitions
    // The loop continues until a state does NOT request an immediate transition.
    // Only the final state's motor output is applied (matching original behavior
    // where setVelocity was called, then state changed, then loop re-ran).
    MotorOutput output;
    int loopCount = 0;
    // In practice, max 2-3 transitions should happen (e.g., Idle -> AlignCoral -> Phase2)
    // A higher number indicates a bug (state oscillation or missing break condition)
    final int maxLoops = 4;

    while (true) {
      output = processState(desiredAction);

      // Check if state changed (immediate transition requested)
      if (states.endEffectorDesiredAction == desiredAction) {
        // No transition - use this state's output
        break;
      }

      // State transition requested - loop will process the new state
      // and use ITS output instead (the current output is discarded)
      desiredAction = states.endEffectorDesiredAction;

      // Safety check
      if (++loopCount >= maxLoops) {
        System.err.println("EndEffector state machine exceeded max iterations!");
        break;
      }
    }

    return output;
  }

  /**
   * Process a single state and return the motor output. May update states.endEffectorDesiredAction
   * for immediate transitions.
   */
  private MotorOutput processState(EndEffectorDesiredAction action) {
    return switch (action) {
      case Idle -> handleIdle();
      case CoralStationIntake -> handleCoralStationIntake();
      case GroundIntake -> handleGroundIntake();
      case AlignCoral -> handleAlignCoral();
      case AlignCoralPhase2 -> handleAlignCoralPhase2();
      case AlignCoralPhase3 -> handleAlignCoralPhase3();
      case AlignCoralPhase4 -> handleAlignCoralPhase4();
      case AlignedCoral -> handleAlignedCoral();
      case ScoreFastForward -> handleScoreFastForward();
      case ScoreFastBackward -> handleScoreFastBackward();
      case PassCoralToEndEffector -> MotorOutput.velocity(passOffVelocityRPM.get());
      case PassToIntake -> MotorOutput.velocity(-passOffVelocityRPM.get());
      case HoldAlgae -> MotorOutput.velocity(holdAlgaeRPM.get());
      case ScoreAlgae -> MotorOutput.velocity(scoringVelocityRPM.get());
    };
  }

  // --- State Handlers ---

  private MotorOutput handleIdle() {
    if (context.hasCoralBothSides()) {
      states.endEffectorDesiredAction = EndEffectorDesiredAction.AlignCoral;
      aligner.reset();
    }
    return MotorOutput.percent(0);
  }

  private MotorOutput handleCoralStationIntake() {
    if (context.hasCoralNonScoringSide()) {
      states.endEffectorDesiredAction = EndEffectorDesiredAction.AlignCoral;
      aligner.reset();
      return MotorOutput.stop();
    } else if (context.hasCoralScoringSide()) {
      return MotorOutput.velocity(intakingVelocitySlow.get());
    } else {
      return MotorOutput.velocity(intakingVelocityHigh.get());
    }
  }

  private MotorOutput handleGroundIntake() {
    if (context.hasCoralScoringSide()) {
      states.endEffectorDesiredAction = EndEffectorDesiredAction.AlignCoral;
      aligner.reset();
      return MotorOutput.stop();
    } else if (context.hasCoralNonScoringSide()) {
      return MotorOutput.velocity(-intakingVelocitySlow.get());
    } else {
      return MotorOutput.velocity(-intakingVelocityHigh.get());
    }
  }

  private MotorOutput handleAlignCoral() {
    // Use the CoralAligner for the alignment sequence
    aligner.setPhase(AlignPhase.CENTERING);
    var result = aligner.update(context);

    if (!context.hasAnyCoral()) {
      states.endEffectorDesiredAction = EndEffectorDesiredAction.Idle;
      return MotorOutput.stop();
    }

    // Map aligner phase to EndEffectorDesiredAction for compatibility
    states.endEffectorDesiredAction =
        switch (aligner.getPhase()) {
          case CENTERING -> EndEffectorDesiredAction.AlignCoral;
          case BACKTRACK -> EndEffectorDesiredAction.AlignCoralPhase2;
          case FIND_EDGE -> EndEffectorDesiredAction.AlignCoralPhase3;
          case MOVE_TO_TARGET -> EndEffectorDesiredAction.AlignCoralPhase4;
          case ALIGNED -> EndEffectorDesiredAction.AlignedCoral;
        };

    return alignResultToMotorOutput(result);
  }

  private MotorOutput handleAlignCoralPhase2() {
    aligner.setPhase(AlignPhase.BACKTRACK);
    var result = aligner.update(context);
    updateAlignmentState();
    return alignResultToMotorOutput(result);
  }

  private MotorOutput handleAlignCoralPhase3() {
    aligner.setPhase(AlignPhase.FIND_EDGE);
    var result = aligner.update(context);
    updateAlignmentState();
    return alignResultToMotorOutput(result);
  }

  private MotorOutput handleAlignCoralPhase4() {
    aligner.setPhase(AlignPhase.MOVE_TO_TARGET);
    var result = aligner.update(context);
    updateAlignmentState();
    return alignResultToMotorOutput(result);
  }

  private MotorOutput handleAlignedCoral() {
    aligner.setPhase(AlignPhase.ALIGNED);
    var result = aligner.update(context);
    return alignResultToMotorOutput(result);
  }

  private void updateAlignmentState() {
    states.endEffectorDesiredAction =
        switch (aligner.getPhase()) {
          case CENTERING -> EndEffectorDesiredAction.AlignCoral;
          case BACKTRACK -> EndEffectorDesiredAction.AlignCoralPhase2;
          case FIND_EDGE -> EndEffectorDesiredAction.AlignCoralPhase3;
          case MOVE_TO_TARGET -> EndEffectorDesiredAction.AlignCoralPhase4;
          case ALIGNED -> EndEffectorDesiredAction.AlignedCoral;
        };
  }

  private MotorOutput alignResultToMotorOutput(CoralAligner.AlignResult result) {
    return switch (result.command()) {
      case VELOCITY -> MotorOutput.velocity(result.velocityRPM());
      case STOP -> MotorOutput.percent(0);
    };
  }

  private MotorOutput handleScoreFastForward() {
    return handleScoreFast(1);
  }

  private MotorOutput handleScoreFastBackward() {
    return handleScoreFast(-1);
  }

  /**
   * Common scoring logic for both forward and backward directions.
   *
   * @param direction 1 for forward, -1 for backward
   */
  private MotorOutput handleScoreFast(int direction) {
    if (!context.hasAnyCoral()) {
      states.endEffectorDesiredAction = EndEffectorDesiredAction.Idle;
    }
    clearCoralInSim();
    return MotorOutput.velocity(direction * scoringVelocityRPM.get());
  }

  /** Clear coral detection in simulation when scoring. */
  private void clearCoralInSim() {
    var sim = getSim();
    if (sim != null) {
      sim.clearCoralDetection();
    }
  }

  /** Apply the motor output to the hardware. */
  private void applyMotorOutput(MotorOutput output) {
    switch (output.type()) {
      case VELOCITY -> setVelocity(output.value());
      case PERCENT -> setPercentOut(output.value());
      case STOP -> setPercentOut(0);
    }
  }

  // Setters

  private void setConstants() {
    io.setClosedLoopConstants(kP.get(), kV.get(), kS.get(), targetAccelerationConfig.get());
  }

  private void setPercentOut(double percent) {
    io.setVoltage(percent * Constants.MAX_VOLTAGE);
    controlMode = ControlMode.OPEN_LOOP;
  }

  private void setVelocity(double revPerMin) {
    io.setVelocity(revPerMin);
    logTargetVelocityRPM.info(revPerMin);
    controlMode = ControlMode.CLOSED_LOOP;
  }

  // Getters
  public Current getCurrentDraw() {
    return Units.Amps.of(inputs.currentAmps);
  }

  public AngularVelocity getVelocity() {
    return Units.RPM.of(inputs.velocityRPM);
  }

  public Distance scoringSideTofDistance() {
    return Units.Inches.of(inputs.scoringSideTofDistInches);
  }

  public Distance nonScoringSideTofDistance() {
    return Units.Inches.of(inputs.nonScoringSideTofDistInches);
  }

  public boolean scoringSideTofSeenGamepiece() {
    return inputs.scoringSideTofDetecting;
  }

  public boolean nonScoringSideTOFSeenGamepiece() {
    return inputs.nonScoringSideTofDetecting;
  }

  public double getMotorPosition() {
    return inputs.position;
  }

  public boolean rollerStallDetected() {
    return stallDetected.getAsBoolean();
  }
}
