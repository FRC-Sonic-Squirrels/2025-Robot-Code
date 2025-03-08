// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.robot.RobotStates;
import frc.robot.RobotStates.ScoringLevel;
import frc.robot.subsystems.mechanism.MechanismPositions.MechanismPosition;
import frc.robot.subsystems.mechanism.arm.Arm;
import frc.robot.subsystems.mechanism.elevator.Elevator;

/** Add your docs here. */
public class Mechanism {
  private static final String ROOT_TABLE = "MechanismActions";

  private static final LoggerGroup logGroup = LoggerGroup.build(ROOT_TABLE);

  private final Elevator elevator;
  private final Arm arm;

  private static final LoggerEntry.Bool log_ElevatorInPosition =
      logGroup.buildBoolean("ElevatorInPosition");
  private static final LoggerEntry.Bool log_ArmInPosition = logGroup.buildBoolean("ArmInPosition");
  private static final LoggerEntry.Text log_currentMotionState =
      logGroup.buildString("CurrentMotionState");
  private static final LoggerEntry.Text log_currentSection = logGroup.buildString("CurrentSection");
  private static final LoggerEntry.Text log_targetSection = logGroup.buildString("TargetSection");

  private boolean elevatorInPosition = false;
  private boolean armInPosition = false;

  public Mechanism(Elevator elevator, Arm arm) {
    this.elevator = elevator;
    this.arm = arm;
  }

  public void periodic() {
    switch (RobotStates.mechState) {
      case Idle:
        elevator.setPercentOut(0);
        arm.setPercentOut(0);
        break;
      case Override:
        break;
      case ReefPosition:
        goToPositionParallel(MechanismPositions.reefPosition(RobotStates.scoringLevel));
        break;
      case ReefL1Position:
        goToPositionParallel(MechanismPositions.reefPosition(ScoringLevel.L1));
        break;
      case ReefL2Position:
        goToPositionParallel(MechanismPositions.reefPosition(ScoringLevel.L2));
        break;
      case ReefL3Position:
        goToPositionParallel(MechanismPositions.reefPosition(ScoringLevel.L3));
        break;
      case ReefL4Position:
        goToPositionParallel(MechanismPositions.reefPosition(ScoringLevel.L4));
        break;
      case ReefPrepPosition:
        goToPositionParallel(MechanismPositions.reefPrepPosition(RobotStates.scoringLevel));
        break;
      case CoralStationPosition:
        goToPositionParallel(MechanismPositions.coralStationPosition());
        break;
      case ClearAlgaeLowPosition:
        goToPositionParallel(MechanismPositions.clearAlgaeLowPosition());
        break;
      case ClearAlgaeHighPosition:
        goToPositionParallel(MechanismPositions.clearAlgaeHighPosition());
        break;
      case StowPosition:
        goToPositionParallel(MechanismPositions.stowPosition());
        break;
      case ClimbPosition:
        goToPositionParallel(MechanismPositions.reefPrepPosition(ScoringLevel.L3));
        break;
      case PrepPassoffPosition:
        goToPositionParallel(MechanismPositions.prepForPassoffPosition());
        break;
      case PassoffPosition:
        goToPositionParallel(MechanismPositions.passoffPosition());
        break;
      default:
        break;
    }
  }

  public boolean mechInPosition() {
    return elevatorInPosition && armInPosition;
  }

  public Elevator getElevator() {
    return elevator;
  }

  public Arm getArm() {
    return arm;
  }

  private void goToPositionParallel(MechanismPosition position) {
    goToPositionParallel(position, Double.NaN, Double.NaN);
  }

  private void goToPositionParallel(
      MechanismPosition position, double elevatorAccel, double armAccel) {

    MechSection targetMechSection = getMechSection(position);
    MechanismPosition currentMechPos = new MechanismPosition(elevator.getHeight(), arm.getAngle());
    MechSection currentMechSection = getMechSection(currentMechPos);

    log_currentSection.info(currentMechSection.name());
    log_targetSection.info(targetMechSection.name());

    if (currentMechPos.armAngle().getDegrees() > 130 && position.armAngle().getDegrees() <= 130) {
      arm.setAngle(Rotation2d.fromDegrees(0));
    } else if (compatibleMechSections(currentMechSection, targetMechSection)) {
      log_currentMotionState.info("Compatible");
      goToPositionParallelSimple(position, elevatorAccel, armAccel);
    } else {
      if (currentMechSection == MechSection.S1) {
        log_currentMotionState.info("Getting out of S1");
        goToPositionParallelSimple(MechanismPositions.intermediateLowBackPosition());
      } else if (currentMechSection == MechSection.S2
          || (currentMechSection == MechSection.S3 && targetMechSection != MechSection.S1)) {
        log_currentMotionState.info("Getting out of " + currentMechSection.name());
        goToPositionParallelSimple(MechanismPositions.intermediateLowPosition());
      } else if (currentMechSection == MechSection.S9
          || currentMechSection == MechSection.S8
          || currentMechSection == MechSection.S5) {
        log_currentMotionState.info("Getting out of " + currentMechSection.name());
        goToPositionParallelSimple(MechanismPositions.intermediateHighPosition());
      } else if (targetMechSection == MechSection.S2 || targetMechSection == MechSection.S3) {
        log_currentMotionState.info("Getting into " + targetMechSection.name());
        goToPositionParallelSimple(MechanismPositions.intermediateLowPosition());
      } else if (targetMechSection == MechSection.S9 || targetMechSection == MechSection.S8) {
        log_currentMotionState.info("Getting into " + targetMechSection.name());
        goToPositionParallelSimple(MechanismPositions.intermediateHighPosition());
      } else {
        log_currentMotionState.info("Getting into S1");
        if (targetMechSection == MechSection.S1
            && (currentMechSection == MechSection.S4 || currentMechSection == MechSection.S3)) {
          goToPositionParallelSimple(MechanismPositions.intermediateLowBackPosition());
        } else {
          goToPositionParallelSimple(MechanismPositions.intermediateLowPosition());
        }
      }
    }

    elevatorInPosition = elevator.isAtTarget(position.elevatorHeight());
    log_ElevatorInPosition.info(elevatorInPosition);
    armInPosition = arm.isAtTargetAngle(position.armAngle());
    log_ArmInPosition.info(armInPosition);
  }

  private static boolean compatibleMechSections(MechSection mech1, MechSection mech2) {
    if (mech1 == mech2) return true;
    if ((mech1 == MechSection.S4 || mech1 == MechSection.S6 || mech1 == MechSection.S7)
        && (mech2 == MechSection.S4 || mech2 == MechSection.S6 || mech2 == MechSection.S7))
      return true;

    if ((mech1 == MechSection.S2 || mech1 == MechSection.S3 || mech1 == MechSection.S4)
        && (mech2 == MechSection.S2 || mech2 == MechSection.S3 || mech2 == MechSection.S4))
      return true;

    if ((mech1 == MechSection.S7 || mech1 == MechSection.S8 || mech1 == MechSection.S9)
        && (mech2 == MechSection.S7 || mech2 == MechSection.S8 || mech2 == MechSection.S9))
      return true;

    if (mech1 == MechSection.S1 && mech2 == MechSection.S2) return true;

    if (mech1 == MechSection.S2 && mech2 == MechSection.S1) return true;

    return false;
  }

  private void goToPositionParallelSimple(MechanismPosition position) {
    goToPositionParallelSimple(position, Double.NaN, Double.NaN);
  }

  private void goToPositionParallelSimple(
      MechanismPosition position, double elevatorAccel, double armAccel) {
    MechanismPosition targetPos = position;
    if (elevatorAccel == Double.NaN) {
      elevator.setHeight(targetPos.elevatorHeight());
    } else {
      elevator.setHeight(targetPos.elevatorHeight(), elevatorAccel);
    }

    if (armAccel == Double.NaN) {
      arm.setAngle(targetPos.armAngle());
    } else {
      arm.setAngle(targetPos.armAngle(), armAccel);
    }
  }

  private static MechSection getMechSection(MechanismPosition position) {
    if (position.armAngle().getDegrees() > 125) {
      if (position.elevatorHeight().in(Units.Inches) < 0.1) {
        return MechSection.S2;
      } else if (position.elevatorHeight().in(Units.Inches) < 25) {
        return MechSection.S1;
      } else return MechSection.S9;
    } else if (position.armAngle().getDegrees() > 75) {
      if (position.elevatorHeight().in(Units.Inches) < 0.1) {
        return MechSection.S3;
      } else if (position.elevatorHeight().in(Units.Inches) < 25) {
        return MechSection.S5;
      } else return MechSection.S8;
    } else {
      if (position.elevatorHeight().in(Units.Inches) < 0.1) {
        return MechSection.S4;
      } else if (position.elevatorHeight().in(Units.Inches) < 25) {
        return MechSection.S6;
      } else return MechSection.S7;
    }
  }

  private enum MechSection {
    S1, // elevator above 0.1, below top tube, arm is back
    S2, // elevator below 0.1, arm is back
    S3, // elevator below top tube, arm under top tube
    S4, // elevator below 0.1, arm is forward
    S5, // below top tube, elevator above 0.1, arm is up
    S6, // below top tube, elevator above 0.1, arm is forward
    S7, // above top tube, arm is forward
    S8, // above top tube, arm is near straight up
    S9 // above top tube, arm is back
  }
}
