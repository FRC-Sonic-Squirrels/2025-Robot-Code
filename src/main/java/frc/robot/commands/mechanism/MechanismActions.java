package frc.robot.commands.mechanism;

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
          MechanismPosition targetPosition = position.get();
          // the first 3 are the elevator, arm, and pivot
          // 0,0 is the bottom of the elevator
          // 1 unit is 1 inch
          // TODO: get actual values and poses for colldiers
          Translation2d[] colliders = {null, null, null, new Translation2d(0, 0)};
          double[] radiii = {1.0, 1.0, 1.0, 1.0};
          double SAFE_MULT = 1.2;

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
                new Translation2d(ArmConstants.ARM_LENGTH.in(Units.Inches), 0)
                    .rotateBy(intake.getPivotAngle());
            // check elevator
            boolean runningElevator = true;
            for (int c = 0; c < colliders.length; c++) { // check against all other colliders
              if (c == 0) { // dont check if the other is myself
                continue;
              }
              Translation2d diff = colliders[0].minus(colliders[c]);
              if (Math.pow(diff.getX(), 2) + Math.pow(diff.getY(), 2)
                  < (radiii[0] + radiii[c])) { // circle col with raw radii
                // they hit eachother
                elevator.setHeight(ElevatorConstants.SAFE_HEIGHT);
                runningElevator = false;
                continue;
              } else if (Math.pow(diff.getX(), 2) + Math.pow(diff.getY(), 2)
                  < (radiii[0] + radiii[c]) * SAFE_MULT) { // circle col with safety barrier
                // oh no they are getting very close
                elevator.setHeight(ElevatorConstants.SAFE_HEIGHT);
                runningElevator = false;
                continue;
              }
            }
            // check arm
            boolean runningArm = true;
            for (int c = 0; c < colliders.length; c++) { // check against all other colliders
              if (c == 1) { // dont check if the other is myself
                continue;
              }
              Translation2d diff = colliders[1].minus(colliders[c]);
              if (Math.pow(diff.getX(), 2) + Math.pow(diff.getY(), 2)
                  < (radiii[1] + radiii[c])) { // circle col with raw radii
                // they hit eachother
                elevator.setHeight(ElevatorConstants.SAFE_HEIGHT);
                runningElevator = false;
                arm.setAngle(ArmConstants.ARM_SAFE_ANGLE);
                runningArm = false;
                continue;
              } else if (Math.pow(diff.getX(), 2) + Math.pow(diff.getY(), 2)
                  < (radiii[1] + radiii[c]) * SAFE_MULT) { // circle col with safety barrier
                // oh no they are getting very close
                elevator.setHeight(ElevatorConstants.SAFE_HEIGHT);
                runningElevator = false;
                arm.setAngle(ArmConstants.ARM_SAFE_ANGLE);
                runningArm = false;
                continue;
              }
            }
            // check pivot
            boolean runningPivot = true;
            for (int c = 0; c < colliders.length; c++) { // check against all other colliders
              if (c == 2) { // dont check if the other is myself
                continue;
              }
              Translation2d diff = colliders[2].minus(colliders[c]);
              if (Math.pow(diff.getX(), 2) + Math.pow(diff.getY(), 2)
                  < (radiii[2] + radiii[c])) { // circle col with raw radii
                // they hit eachother
                intake.setPivotAngle(PivotConstants.MAX_PIVOT_ANGLE);
                runningPivot = false;
                continue;
              } else if (Math.pow(diff.getX(), 2) + Math.pow(diff.getY(), 2)
                  < (radiii[2] + radiii[c]) * SAFE_MULT) { // circle col with safety barrier
                // oh no they are getting very close
                intake.setPivotAngle(PivotConstants.MAX_PIVOT_ANGLE);
                runningPivot = false;
                continue;
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
            log_runningArm.info(runningArm);
            log_runningElevator.info(runningElevator);
            log_runningPivot.info(runningPivot);
            log_ElevatorInPosition.info(elevator.isAtTarget());
            log_ArmInPosition.info(arm.isAtTargetAngle());
            log_PivotInPosition.info(intake.isPivotAtTargetAngle());
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
}
