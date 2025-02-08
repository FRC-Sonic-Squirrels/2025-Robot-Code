package frc.robot.commands.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants.PivotConstants;
import frc.robot.RobotStates.ScoringLevel;
import frc.robot.commands.mechanism.MechanismPositions.MechanismPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import java.util.function.Supplier;

public class MechanismActions {
  private static final String ROOT_TABLE = "MechanismActions";

  private static final LoggerGroup logGroup = LoggerGroup.build(ROOT_TABLE);
  private static final LoggerEntry.Bool log_runningArm = logGroup.buildBoolean("runningArm");
  private static final LoggerEntry.Bool log_runningElevator =
      logGroup.buildBoolean("runningElevator");
  private static final LoggerEntry.Bool log_runningPivot = logGroup.buildBoolean("runningPivot");
  private static final LoggerEntry.Bool log_ElevatorInPosition =
      logGroup.buildBoolean("ElevatorInPosition");
  private static final LoggerEntry.Bool log_PivotInPosition =
      logGroup.buildBoolean("PivotInPosition");
  private static final LoggerEntry.Bool log_ArmInPosition = logGroup.buildBoolean("ArmInPosition");

  private static final LoggerEntry.Struct<Transform2d> log_EstimatedElevatorPosision =
      logGroup.buildStruct(Transform2d.class, "EstimatedElevatorPosision");
  private static final LoggerEntry.Struct<Transform2d> log_EstimatedArmPosision =
      logGroup.buildStruct(Transform2d.class, "EstimatedArmPosision");
  private static final LoggerEntry.Struct<Transform2d> log_EstimatedPivotPosision =
      logGroup.buildStruct(Transform2d.class, "EstimatedPivotPosision");

  private static final TunableNumberGroup group = new TunableNumberGroup(ROOT_TABLE);

  public static final LoggedTunableNumber climbDownElevatorVelocity =
      group.build("MechanismActions/climbDownElevatorVelocity", 500);
  public static final LoggedTunableNumber climbDownElevatorAcceleration =
      group.build("climbDownElevatorAcceleration", 500);

  public static final LoggedTunableNumber tunableArmVoltage =
      group.build("MechanismActions/tunableArmVoltage", 5.0);

  public static Command reefPosition(
      Elevator elevator, Arm arm, Intake intake, ScoringLevel scoringLevel) {
    return goToPositionParallel(
        elevator, arm, intake, () -> MechanismPositions.reefPosition(scoringLevel));
  }

  public static Command coralStationPosition(Elevator elevator, Arm arm, Intake intake) {
    return goToPositionParallel(elevator, arm, intake, MechanismPositions::coralStationPosition);
  }

  public static Command stowPosition(Elevator elevator, Arm arm, Intake intake) {
    return goToPositionParallel(elevator, arm, intake, MechanismPositions::stowPosition);
  }

  public static Command clearAlgaeLow1Position(Elevator elevator, Arm arm, Intake intake) {
    return goToPositionParallel(elevator, arm, intake, MechanismPositions::clearAlgaeLow1Position);
  }

  public static Command clearAlgaeLow2Position(Elevator elevator, Arm arm, Intake intake) {
    return goToPositionParallel(elevator, arm, intake, MechanismPositions::clearAlgaeLow2Position);
  }

  public static Command clearAlgaeHigh1Position(Elevator elevator, Arm arm, Intake intake) {
    return goToPositionParallel(elevator, arm, intake, MechanismPositions::clearAlgaeHigh1Position);
  }

  public static Command clearAlgaeHigh2Position(Elevator elevator, Arm arm, Intake intake) {
    return goToPositionParallel(elevator, arm, intake, MechanismPositions::clearAlgaeHigh2Position);
  }

  // TODO: Change Logic for 2025 Robot Geometry
  private static Command goToPositionParallel(
      Elevator elevator, Arm arm, Intake intake, Supplier<MechanismPosition> position) {
    return goToPositionParallel(elevator, arm, intake, position, false);
  }

  private static Command goToPositionParallel(
      Elevator elevator,
      Arm arm,
      Intake intake,
      Supplier<MechanismPosition> position,
      boolean ignoreSafety) {

    var cmd =
        new Command() {

          boolean elevatorInPosition = false;
          boolean armInPosition = false;
          boolean pivotInPosition = false;
          MechanismPosition targetPosition = position.get();
          // the first 3 are the elevator, arm, and pivot
          // 0,0 is the bottom of the elevator
          // 1 unit is 1 inch
          // positive x is from the elevator to the intake
          // positive y is up
          // TODO: get actual values and poses for colldiers
          Translation2d[] colliders = {
            null,
            null,
            null,
            new Translation2d(0, -1),
            new Translation2d(-34.35, -1),
            new Translation2d(0, 11.4),
            new Translation2d(18.8, -6.0)
          };
          double[] radiii = {2.0, 3.375, 7.0, 1.0, 1.0, 0.5};
          double SAFE_MULT = 0.0;
          Translation2d intakePos = new Translation2d(19, 6.2);

          @Override
          public void initialize() {}

          @Override
          public void execute() {
            // update colliders for the moving parts
            // elevator
            colliders[0] = new Translation2d(0.0, elevator.getHeight().in(Units.Inches));
            // arm, elevator plus arm stuff
            colliders[1] =
                colliders[0].plus(
                    new Translation2d(ArmConstants.ARM_LENGTH.in(Units.Inches), 0)
                        .rotateBy(arm.getAngle()));
            // pivot
            colliders[2] =
                intakePos.plus(
                    new Translation2d(ArmConstants.ARM_LENGTH.in(Units.Inches), 0)
                        .rotateBy(intake.getPivotAngle()));
            // 0 means it is fine
            // 1 means it is close to another collider
            // 2 means it is hitting another collider
            int elevatorMovePriority = 0;
            // check elevator
            boolean runningElevator = true;
            for (int c = 0; c < colliders.length; c++) { // check against all other colliders
              if (c == 0) { // dont check if the other is myself
                continue;
              }
              if (colliders[0].getDistance(colliders[c])
                  < (radiii[0] + radiii[c])) { // circle col with raw radii
                // they hit eachother
                // if the thing we are hitting is lower than us
                if (colliders[0].getY() < colliders[c].getY()) {
                  // move down
                  elevator.setHeight(Units.Inches.of(0));
                } else {
                  // otherwise move up
                  elevator.setHeight(ElevatorConstants.MAX_HEIGHT);
                }
                elevatorMovePriority = 2;
                runningElevator = false;
                break;
              } else if (colliders[0].getDistance(colliders[c])
                  < (radiii[0] + radiii[c]) * SAFE_MULT) { // circle col with safety barrier
                // oh no they are getting very close
                // same logic as above
                if (colliders[0].getY() < colliders[c].getY()) {
                  elevator.setHeight(Units.Inches.of(0));
                } else {
                  elevator.setHeight(ElevatorConstants.MAX_HEIGHT);
                }
                elevatorMovePriority = 1;
                runningElevator = false;
                break;
              }
            }
            // same collision logic for arm and pivot
            // check arm
            boolean runningArm = true;
            for (int c = 0; c < colliders.length; c++) {
              if (c == 1) {
                continue;
              }
              if (colliders[1].getDistance(colliders[c]) < (radiii[1] + radiii[c])) {
                // only move the elevator if it is ok
                if (elevatorMovePriority < 2) {
                  if (colliders[1].getY() < colliders[c].getY()) {
                    elevator.setHeight(Units.Inches.of(0));
                  } else {
                    elevator.setHeight(ElevatorConstants.MAX_HEIGHT);
                  }
                  elevator.setHeight(ElevatorConstants.SAFE_HEIGHT);
                  runningElevator = false;
                }
                // if the thing we are colliding with is to our left
                // TODO: make sure this math is correct
                // source: https://stackoverflow.com/questions/1560492
                if ((colliders[1].getX())
                            * (colliders[c].getY() - elevator.getHeight().in(Units.Inches))
                        - (colliders[1].getY() - elevator.getHeight().in(Units.Inches))
                            * (colliders[c].getX())
                    > 0) {
                  // move right
                  arm.setAngle(ArmConstants.MAX_ARM_ANGLE);
                } else {
                  // otherwise move left
                  arm.setAngle(ArmConstants.MIN_ARM_ANGLE);
                }
                runningArm = false;
                break;
              } else if (colliders[1].getDistance(colliders[c])
                  < (radiii[1] + radiii[c]) * SAFE_MULT) {

                // only move the elevator if it is ok
                if (elevatorMovePriority < 1) {
                  if (colliders[1].getY() < colliders[c].getY()) {
                    elevator.setHeight(Units.Inches.of(0));
                  } else {
                    elevator.setHeight(ElevatorConstants.MAX_HEIGHT);
                  }
                  elevator.setHeight(ElevatorConstants.SAFE_HEIGHT);
                  runningElevator = false;
                }
                // same logic as above
                if ((colliders[1].getX())
                            * (colliders[c].getY() - elevator.getHeight().in(Units.Inches))
                        - (colliders[1].getY() - elevator.getHeight().in(Units.Inches))
                            * (colliders[c].getX())
                    > 0) {
                  arm.setAngle(ArmConstants.MAX_ARM_ANGLE);
                } else {
                  arm.setAngle(ArmConstants.MIN_ARM_ANGLE);
                }
                runningArm = false;
                break;
              }
            }
            // check pivot
            boolean runningPivot = true;
            for (int c = 0; c < colliders.length; c++) {
              if (c == 2) {
                continue;
              }
              if (colliders[2].getDistance(colliders[c]) < (radiii[2] + radiii[c])) {

                // same logic as above
                if ((colliders[2].getX() - intakePos.getX())
                            * (colliders[c].getY() - intakePos.getY())
                        - (colliders[2].getY() - intakePos.getY())
                            * (colliders[c].getX() - intakePos.getX())
                    > 0) {
                  intake.setPivotAngle(PivotConstants.MAX_PIVOT_ANGLE);
                } else {
                  intake.setPivotAngle(PivotConstants.MIN_PIVOT_ANGLE);
                }
                runningPivot = false;
                break;
              } else if (colliders[2].getDistance(colliders[c])
                  < (radiii[2] + radiii[c]) * SAFE_MULT) {
                // same logic as above
                if ((colliders[2].getX() - intakePos.getX())
                            * (colliders[c].getY() - intakePos.getY())
                        - (colliders[2].getY() - intakePos.getY())
                            * (colliders[c].getX() - intakePos.getX())
                    > 0) {
                  intake.setPivotAngle(PivotConstants.MAX_PIVOT_ANGLE);
                } else {
                  intake.setPivotAngle(PivotConstants.MIN_PIVOT_ANGLE);
                }
                runningPivot = false;
                break;
              }
            }
            if (runningElevator) {
              elevator.setHeight(targetPosition.elevatorHeight());
            }
            if (runningArm) {
              arm.setAngle(targetPosition.armAngle());
            }
            if (runningPivot) {
              intake.setPivotAngle(targetPosition.intakeAngle());
            }

            elevatorInPosition = elevator.isAtTarget();
            armInPosition = arm.isAtTargetAngle();
            pivotInPosition = intake.isPivotAtTargetAngle();

            log_runningArm.info(runningArm);
            log_runningElevator.info(runningElevator);
            log_runningPivot.info(runningPivot);

            log_ElevatorInPosition.info(elevatorInPosition);
            log_ArmInPosition.info(armInPosition);
            log_PivotInPosition.info(pivotInPosition);

            log_EstimatedElevatorPosision.info(new Transform2d(colliders[0], Rotation2d.kZero));
            log_EstimatedArmPosision.info(new Transform2d(colliders[1], Rotation2d.kZero));
            log_EstimatedPivotPosision.info(new Transform2d(colliders[2], Rotation2d.kZero));
          }

          @Override
          public boolean isFinished() {
            return elevatorInPosition && armInPosition && pivotInPosition;
          }
        };

    cmd.addRequirements(elevator, arm, intake);
    cmd.setName("MechanismAction");
    return cmd;
  }
}
