// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector;

import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.RobotStates.ScoringLevel;

/**
 * Handles the 4-phase coral alignment sequence. This class encapsulates the complex alignment logic
 * that positions coral precisely within the end effector.
 *
 * <p>Alignment phases:
 *
 * <ol>
 *   <li>CENTERING: Move coral until non-scoring sensor sees it
 *   <li>BACKTRACK: Back up until non-scoring sensor loses it
 *   <li>FIND_EDGE: Move forward until non-scoring sensor sees it again (records zero position)
 *   <li>MOVE_TO_TARGET: Move to target offset from zero position
 *   <li>ALIGNED: Maintain position based on scoring level
 * </ol>
 */
public class CoralAligner {

  /** The current phase of alignment. */
  public enum AlignPhase {
    CENTERING,
    BACKTRACK,
    FIND_EDGE,
    MOVE_TO_TARGET,
    ALIGNED
  }

  /** Result of an alignment update - tells the caller what motor action to take. */
  public record AlignResult(MotorCommand command, double velocityRPM, boolean isAligned) {
    public static AlignResult velocity(double rpm) {
      return new AlignResult(MotorCommand.VELOCITY, rpm, false);
    }

    public static AlignResult stop() {
      return new AlignResult(MotorCommand.STOP, 0, false);
    }

    public static AlignResult aligned() {
      return new AlignResult(MotorCommand.STOP, 0, true);
    }

    public static AlignResult alignedWithVelocity(double rpm) {
      return new AlignResult(MotorCommand.VELOCITY, rpm, true);
    }
  }

  public enum MotorCommand {
    VELOCITY,
    STOP
  }

  // Tunable numbers
  private static final TunableNumberGroup group = new TunableNumberGroup("CoralAligner");
  private static final LoggedTunableNumber correctionVelocity = group.build("alignVelocity", 300);
  private static final LoggedTunableNumber alignTarget = group.build("alignTarget", 1);
  private static final LoggedTunableNumber alignTolerance = group.build("alignTolerance", 2);
  private static final LoggedTunableNumber alignL1Turns = group.build("alignL1Turns", 0.75);
  private static final LoggedTunableNumber alignL2Turns = group.build("alignL2Turns", 0.15);
  private static final LoggedTunableNumber alignL3Turns = group.build("alignL3Turns", 1.8);
  private static final LoggedTunableNumber alignL4Turns = group.build("alignL4Turns", 1.8);

  // Logging
  private static final LoggerGroup logGroup = LoggerGroup.build("CoralAligner");
  private static final LoggerEntry.EnumValue<AlignPhase> logPhase = logGroup.buildEnum("Phase");
  private static final LoggerEntry.Decimal logZeroPosition = logGroup.buildDecimal("ZeroPosition");
  private static final LoggerEntry.Decimal logCurrentOffset =
      logGroup.buildDecimal("CurrentOffset");
  private static final LoggerEntry.Decimal logTargetOffset = logGroup.buildDecimal("TargetOffset");

  // State
  private AlignPhase phase = AlignPhase.CENTERING;
  private double zeroCoralPosition = 0;

  /** Reset the aligner to start a new alignment sequence. */
  public void reset() {
    phase = AlignPhase.CENTERING;
    zeroCoralPosition = 0;
  }

  /** Get the current alignment phase. */
  public AlignPhase getPhase() {
    return phase;
  }

  /** Check if alignment is complete. */
  public boolean isAligned() {
    return phase == AlignPhase.ALIGNED;
  }

  /**
   * Update the alignment state machine and return the motor command to execute.
   *
   * @param ctx The end effector context with current sensor state
   * @return The motor command to execute
   */
  public AlignResult update(EndEffectorContext ctx) {
    boolean hasScoringSide = ctx.hasCoralScoringSide();
    boolean hasNonScoringSide = ctx.hasCoralNonScoringSide();
    boolean hasAnyCoral = ctx.hasAnyCoral();
    double motorPosition = ctx.getMotorPosition();

    // Log current state
    logPhase.info(phase);
    logZeroPosition.info(zeroCoralPosition);

    // If coral is lost during alignment, reset
    if (!hasAnyCoral && phase != AlignPhase.ALIGNED) {
      reset();
      return AlignResult.stop();
    }

    AlignResult result =
        switch (phase) {
          case CENTERING -> handleCentering(hasNonScoringSide);
          case BACKTRACK -> handleBacktrack(hasNonScoringSide);
          case FIND_EDGE -> handleFindEdge(hasNonScoringSide, motorPosition);
          case MOVE_TO_TARGET -> handleMoveToTarget(motorPosition);
          case ALIGNED -> handleAligned(motorPosition, ctx.getScoringLevel());
        };

    return result;
  }

  private AlignResult handleCentering(boolean hasNonScoringSide) {
    if (!hasNonScoringSide) {
      // Keep moving the coral in until non-scoring side sees it
      return AlignResult.velocity(correctionVelocity.get());
    } else {
      // Start backtracking
      phase = AlignPhase.BACKTRACK;
      return AlignResult.velocity(-correctionVelocity.get());
    }
  }

  private AlignResult handleBacktrack(boolean hasNonScoringSide) {
    if (hasNonScoringSide) {
      // Keep backtracking until non-scoring side loses it
      return AlignResult.velocity(-correctionVelocity.get());
    } else {
      // Now reverse until we see it again
      phase = AlignPhase.FIND_EDGE;
      return AlignResult.velocity(correctionVelocity.get());
    }
  }

  private AlignResult handleFindEdge(boolean hasNonScoringSide, double motorPosition) {
    if (!hasNonScoringSide) {
      // Keep moving until we find the edge
      return AlignResult.velocity(correctionVelocity.get());
    } else {
      // Found the edge - record this as zero position
      zeroCoralPosition = motorPosition;
      phase = AlignPhase.MOVE_TO_TARGET;
      return AlignResult.velocity(correctionVelocity.get());
    }
  }

  private AlignResult handleMoveToTarget(double motorPosition) {
    double diff = Math.abs(motorPosition - zeroCoralPosition);
    if (diff >= alignTarget.get()) {
      // Coral moved to target position
      zeroCoralPosition = motorPosition;
      phase = AlignPhase.ALIGNED;
      return AlignResult.aligned();
    }
    // Keep moving to target
    return AlignResult.velocity(correctionVelocity.get());
  }

  private AlignResult handleAligned(double motorPosition, ScoringLevel scoringLevel) {
    double currentOffset = motorPosition - zeroCoralPosition;
    double desiredOffset = getDesiredOffset(scoringLevel);

    logCurrentOffset.info(currentOffset);
    logTargetOffset.info(desiredOffset);

    // Adjust position based on scoring level
    double error = currentOffset - desiredOffset;

    if (Math.abs(error) < alignTolerance.get()) {
      return AlignResult.aligned();
    } else if (error > 0) {
      return AlignResult.alignedWithVelocity(-correctionVelocity.get());
    } else {
      return AlignResult.alignedWithVelocity(correctionVelocity.get());
    }
  }

  private double getDesiredOffset(ScoringLevel level) {
    return switch (level) {
      case L1 -> alignL1Turns.get();
      case L2 -> alignL2Turns.get();
      case L3 -> alignL3Turns.get();
      case L4 -> alignL4Turns.get();
    };
  }

  /** Get the zero position recorded during alignment. */
  public double getZeroPosition() {
    return zeroCoralPosition;
  }

  /** Manually set the zero position (for compatibility during refactor). */
  public void setZeroPosition(double position) {
    this.zeroCoralPosition = position;
  }

  /** Manually set the phase (for compatibility during refactor). */
  public void setPhase(AlignPhase phase) {
    this.phase = phase;
  }
}
