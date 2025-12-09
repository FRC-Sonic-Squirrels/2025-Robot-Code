// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector;

import frc.robot.RobotStates;
import frc.robot.RobotStates.ScoringLevel;

/**
 * Context object that provides a clean interface between the EndEffector state machine and
 * hardware/sensor state. This separates the concerns of "what sensors see" from "what actions to
 * take".
 *
 * <p>The context is updated once per periodic cycle, then passed to state handlers which can make
 * decisions based on the snapshot of sensor state without directly coupling to IO.
 */
public class EndEffectorContext {
  // Sensor state (read-only snapshot for state machine)
  private boolean scoringSideTofDetecting;
  private boolean nonScoringSideTofDetecting;
  private double motorPosition;
  private ScoringLevel scoringLevel;

  // Reference to robot states for reading/writing shared state
  private final RobotStates states;

  public EndEffectorContext(RobotStates states) {
    this.states = states;
  }

  /** Update the context with current sensor readings. Called once at start of periodic(). */
  public void update(EndEffectorIO.Inputs inputs, ScoringLevel scoringLevel) {
    this.scoringSideTofDetecting = inputs.scoringSideTofDetecting;
    this.nonScoringSideTofDetecting = inputs.nonScoringSideTofDetecting;
    this.motorPosition = inputs.position;
    this.scoringLevel = scoringLevel;
  }

  // --- Sensor State Queries ---

  /** Returns true if the scoring-side ToF sensor detects a game piece. */
  public boolean hasCoralScoringSide() {
    return scoringSideTofDetecting;
  }

  /** Returns true if the non-scoring-side ToF sensor detects a game piece. */
  public boolean hasCoralNonScoringSide() {
    return nonScoringSideTofDetecting;
  }

  /** Returns true if either ToF sensor detects a game piece. */
  public boolean hasAnyCoral() {
    return scoringSideTofDetecting || nonScoringSideTofDetecting;
  }

  /** Returns true if both ToF sensors detect a game piece (coral is fully inside). */
  public boolean hasCoralBothSides() {
    return scoringSideTofDetecting && nonScoringSideTofDetecting;
  }

  /** Returns the current motor position in rotations. */
  public double getMotorPosition() {
    return motorPosition;
  }

  /** Returns the current scoring level target. */
  public ScoringLevel getScoringLevel() {
    return scoringLevel;
  }

  // --- Robot States Access ---

  /** Get the shared robot states object for reading/writing cross-subsystem state. */
  public RobotStates getStates() {
    return states;
  }
}
