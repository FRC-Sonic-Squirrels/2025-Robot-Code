package frc.robot.commands.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
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

  private static final TunableNumberGroup group = new TunableNumberGroup(ROOT_TABLE);

  public static final LoggedTunableNumber climbDownElevatorVelocity =
      group.build("MechanismActions/climbDownElevatorVelocity", 500);
  public static final LoggedTunableNumber climbDownElevatorAcceleration =
      group.build("climbDownElevatorAcceleration", 500);

  public static final LoggedTunableNumber tunableArmVoltage =
      group.build("MechanismActions/tunableArmVoltage", 5.0);

  public static Command reefPosition(Elevator elevator, Arm arm, ScoringLevel scoringLevel) {
    return goToPositionParallel(elevator, arm, () -> MechanismPositions.reefPosition(scoringLevel));
  }

  public static Command coralStationPosition(Elevator elevator, Arm arm) {
    return goToPositionParallel(elevator, arm, MechanismPositions::coralStationPosition);
  }

  // TODO: Change Logic for 2025 Robot Geometry
  private static Command goToPositionParallel(
      Elevator elevator, Arm arm, Supplier<MechanismPosition> position) {
    return goToPositionParallel(elevator, arm, position, false);
  }

  private static Command goToPositionParallel(
      Elevator elevator, Arm arm, Supplier<MechanismPosition> position, boolean ignoreSafety) {

    var cmd =
        new Command() {
          boolean elevatorInPosition = false;
          boolean armInPosition = false;

          @Override
          public void execute() {
            MechanismPosition targetPosition = position.get();
            Distance safeHeight = Constants.ElevatorConstants.SAFE_HEIGHT;
            boolean runningArm =
                elevator.getHeight().in(Units.Inches)
                        >= safeHeight.minus(Units.Inches.of(1.0)).in(Units.Inches)
                    || (arm.getAngle().getRadians()
                            > Constants.ArmConstants.ARM_SAFE_ANGLE.getRadians()
                        && position.get().armAngle().getRadians()
                            > Constants.ArmConstants.ARM_SAFE_ANGLE.getRadians());
            if (ignoreSafety) runningArm = true;
            if (runningArm) {
              arm.setAngle(targetPosition.armAngle());
            }

            boolean runningElevatorSafety =
                targetPosition.elevatorHeight().lte(safeHeight)
                    && ((arm.getAngle().getRadians()
                                >= Constants.ArmConstants.ARM_SAFE_ANGLE.getRadians()
                            && targetPosition.armAngle().getRadians()
                                <= Constants.ArmConstants.ARM_SAFE_ANGLE.getRadians())
                        || (arm.getAngle().getRadians()
                                <= Constants.ArmConstants.ARM_SAFE_ANGLE.getRadians()
                            && position.get().armAngle().getRadians()
                                >= Constants.ArmConstants.ARM_SAFE_ANGLE.getRadians()));
            if (ignoreSafety) runningElevatorSafety = false;

            if (runningElevatorSafety) {
              elevator.setHeight(safeHeight);
            } else {
              elevator.setHeight(targetPosition.elevatorHeight());
            }
            log_runningArm.info(runningArm);
            log_runningElevator.info(runningElevatorSafety);
            elevatorInPosition = elevator.isAtTarget(position.get().elevatorHeight());
            log_ElevatorInPosition.info(elevatorInPosition);
            armInPosition =
                arm.isAtTargetAngle(position.get().armAngle(), Rotation2d.fromDegrees(5.0));
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
}
