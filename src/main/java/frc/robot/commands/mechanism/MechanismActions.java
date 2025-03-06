package frc.robot.commands.mechanism;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants.RobotMode;
import frc.robot.RobotStates.ScoringLevel;
import frc.robot.commands.mechanism.MechanismPositions.MechanismPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.Supplier;

public class MechanismActions {
  private static final String ROOT_TABLE = "MechanismActions";

  private static final LoggerGroup logGroup = LoggerGroup.build(ROOT_TABLE);
  private static final LoggerEntry.Bool log_runningArm = logGroup.buildBoolean("runningArm");
  private static final LoggerEntry.Bool log_runningElevator =
      logGroup.buildBoolean("runningElevator");
  private static final LoggerEntry.Bool log_ElevatorInPosition =
      logGroup.buildBoolean("ElevatorInPosition");
  private static final LoggerEntry.Bool log_ArmInPosition = logGroup.buildBoolean("ArmInPosition");
  private static final LoggerEntry.Text log_currentMotionState =
      logGroup.buildString("CurrentMotionState");
  private static final LoggerEntry.Text log_currentSection = logGroup.buildString("CurrentSection");
  private static final LoggerEntry.Text log_targetSection = logGroup.buildString("TargetSection");

  private static final TunableNumberGroup group = new TunableNumberGroup(ROOT_TABLE);

  public static final LoggedTunableNumber climbDownElevatorVelocity =
      group.build("MechanismActions/climbDownElevatorVelocity", 500);
  public static final LoggedTunableNumber climbDownElevatorAcceleration =
      group.build("climbDownElevatorAcceleration", 500);

  public static final LoggedTunableNumber tunableArmVoltage =
      group.build("MechanismActions/tunableArmVoltage", 5.0);

  public static Command reefPosition(Elevator elevator, Arm arm, ScoringLevel scoringLevel) {
    return goToPositionParallel(elevator, arm, () -> MechanismPositions.reefPosition(scoringLevel))
        .withName("ReefPosition");
  }

  public static Command reefPrepPosition(Elevator elevator, Arm arm, ScoringLevel scoringLevel) {
    boolean slowerMotion = scoringLevel == ScoringLevel.L4 && !RobotMode.isSimBot();
    return goToPositionParallel(
            elevator,
            arm,
            () -> MechanismPositions.reefPrepPosition(scoringLevel),
            slowerMotion ? 1000 : -1,
            slowerMotion ? 3 : -1)
        .withName("ReefPrepPosition");
  }

  public static Command coralStationPosition(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::coralStationPosition, 2000, 3)
        .withName("CoralStation");
  }

  public static Command clearAlgaeLow1Position(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::clearAlgaeLow1Position)
        .withName("ClearAlgaeLow1Position");
  }

  public static Command clearAlgaeLow2Position(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::clearAlgaeLow2Position)
        .withName("ClearAlgaeLow2Position");
  }

  public static Command clearAlgaeHigh1Position(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::clearAlgaeHigh1Position)
        .withName("ClearAlgaeHigh1Position");
  }

  public static Command clearAlgaeHigh2Position(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::clearAlgaeHigh2Position)
        .withName("ClearAlgaeHigh2Position");
  }

  public static Command stowPosition(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::stowPosition)
        .withName("StowPosition");
  }

  private static Command goToPositionParallel(
      Elevator elevator, Arm arm, Supplier<MechanismPosition> position) {
    return goToPositionParallel(elevator, arm, position, -1, -1);
  }

  private static Command goToPositionParallel(
      Elevator elevator,
      Arm arm,
      Supplier<MechanismPosition> position,
      double elevatorAccel,
      double armAccel) {

    var cmd =
        new Command() {
          private boolean elevatorInPosition = false;
          private boolean armInPosition = false;

          @Override
          public void execute() {
            MechSection targetMechSection = getMechSection(position.get());
            MechanismPosition currentMechPos =
                new MechanismPosition(elevator.getHeight(), arm.getAngle());
            MechSection currentMechSection = getMechSection(currentMechPos);

            log_currentSection.info(currentMechSection.name());
            log_targetSection.info(targetMechSection.name());

            if (compatibleMechSections(currentMechSection, targetMechSection)) {
              log_currentMotionState.info("Compatible");
              goToPositionParallelSimple(elevator, arm, position, elevatorAccel, armAccel);
            } else {
              if (currentMechSection == MechSection.S1) {
                log_currentMotionState.info("Getting out of S1");
                goToPositionParallelSimple(
                    elevator, arm, MechanismPositions::intermediateLowBackPosition);
              } else if (currentMechSection == MechSection.S2
                  || (currentMechSection == MechSection.S3
                      && targetMechSection != MechSection.S1)) {
                log_currentMotionState.info("Getting out of " + currentMechSection.name());
                goToPositionParallelSimple(
                    elevator, arm, MechanismPositions::intermediateLowPosition);
              } else if (currentMechSection == MechSection.S9
                  || currentMechSection == MechSection.S8
                  || currentMechSection == MechSection.S5) {
                log_currentMotionState.info("Getting out of " + currentMechSection.name());
                goToPositionParallelSimple(
                    elevator, arm, MechanismPositions::intermediateHighPosition);
              } else if (targetMechSection == MechSection.S2
                  || targetMechSection == MechSection.S3) {
                log_currentMotionState.info("Getting into " + targetMechSection.name());
                goToPositionParallelSimple(
                    elevator, arm, MechanismPositions::intermediateLowPosition);
              } else if (targetMechSection == MechSection.S9
                  || targetMechSection == MechSection.S8) {
                log_currentMotionState.info("Getting into " + targetMechSection.name());
                goToPositionParallelSimple(
                    elevator, arm, MechanismPositions::intermediateHighPosition);
              } else {
                log_currentMotionState.info("Getting into S1");
                if (targetMechSection == MechSection.S1
                    && (currentMechSection == MechSection.S4
                        || currentMechSection == MechSection.S3)) {
                  goToPositionParallelSimple(
                      elevator, arm, MechanismPositions::intermediateLowBackPosition);
                } else {
                  goToPositionParallelSimple(
                      elevator, arm, MechanismPositions::intermediateLowPosition);
                }
              }
            }

            elevatorInPosition = elevator.isAtTarget(position.get().elevatorHeight());
            log_ElevatorInPosition.info(elevatorInPosition);
            armInPosition = arm.isAtTargetAngle(position.get().armAngle());
            log_ArmInPosition.info(armInPosition);
          }

          @Override
          public boolean isFinished() {
            return elevatorInPosition && armInPosition;
          }
        };

    cmd.addRequirements(elevator, arm);
    cmd.setName("MechanismAction");
    return cmd;
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

  private static void goToPositionParallelSimple(
      Elevator elevator, Arm arm, Supplier<MechanismPosition> position) {
    goToPositionParallelSimple(elevator, arm, position, -1, -1);
  }

  private static void goToPositionParallelSimple(
      Elevator elevator,
      Arm arm,
      Supplier<MechanismPosition> position,
      double elevatorAccel,
      double armAccel) {
    MechanismPosition targetPos = position.get();
    if (elevatorAccel == -1) {
      elevator.setHeight(targetPos.elevatorHeight());
    } else {
      elevator.setHeight(targetPos.elevatorHeight(), elevatorAccel);
    }

    if (armAccel == -1) {
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
