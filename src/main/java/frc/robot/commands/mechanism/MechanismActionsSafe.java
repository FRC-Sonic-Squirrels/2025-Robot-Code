package frc.robot.commands.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.robot.Constants;
import frc.robot.commands.mechanism.MechanismPositions.MechanismPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.Supplier;

public class MechanismActionsSafe {
  private static final String ROOT_TABLE = "MechanismActionsSafe";

  private static final LoggerGroup logGroup = LoggerGroup.build(ROOT_TABLE);
  private static final LoggerEntry.Bool log_armCanRotateFreelyAtThisHeight =
      logGroup.buildBoolean("armCanRotateFreelyAtThisHeight");
  private static final LoggerEntry.Decimal log_maxArmExtensionDuringRotation =
      logGroup.buildDecimal("maxArmExtensionDuringRotation");
  private static final LoggerEntry.Text log_state = logGroup.buildString("state");
  private static final LoggerEntry.Decimal log_currentSafeHeight =
      logGroup.buildDecimal("currentSafeHeight");

  private static Command goToPositionParallel(
      Elevator elevator, Arm arm, Supplier<MechanismPosition> position) {

    return new Command() {
      private double safeElevatorHeightInchesForIntake;
      private double maxLegalHeightInches;

      private double armLength;
      private double safeAngleIntakeDegrees;

      private Distance targetElevatorHeight;
      private double targetElevatorHeightInches;
      private double elevatorToleranceInches = 0.2;

      private Rotation2d targetArmAngle;
      private double targetArmAngleDegrees;
      private final Rotation2d armTolerance = Rotation2d.fromDegrees(5.0);

      @Override
      public void initialize() {
        var targetPosition = position.get();
        targetElevatorHeight = targetPosition.elevatorHeight();
        targetElevatorHeightInches = targetElevatorHeight.in(Units.Inches);
        targetArmAngle = targetPosition.armAngle();
        targetArmAngleDegrees = targetArmAngle.getDegrees();

        safeElevatorHeightInchesForIntake =
            Constants.ElevatorConstants.SAFE_HEIGHT.in(Units.Inches);
        maxLegalHeightInches = Constants.ElevatorConstants.MAX_LEGAL_HEIGHT.in(Units.Inches);
        armLength = Constants.ArmConstants.ARM_LENGTH.in(Units.Inches);

        // Above this angle, we hit the intake
        safeAngleIntakeDegrees = Constants.ArmConstants.ARM_SAFE_ANGLE.getDegrees();
      }

      @Override
      public void execute() {
        var currentElevatorHeightInches = elevator.getHeightInches();
        var armAngle = arm.getAngle();
        var currentArmVerticalOffset = armLength * armAngle.getSin();
        var currentArmAngleDegrees = armAngle.getDegrees();

        boolean armCanRotateFreelyAtThisHeight =
            currentElevatorHeightInches + armLength < maxLegalHeightInches;

        log_armCanRotateFreelyAtThisHeight.info(armCanRotateFreelyAtThisHeight);

        //
        // Compute the maximum vertical extension of the arm during rotation.
        //
        double maxArmExtensionDuringRotation;
        var currentArmIsBehindElevator = currentArmAngleDegrees > 90;
        var targetArmIsBehindElevator = targetArmAngleDegrees > 90;

        if (currentArmIsBehindElevator != targetArmIsBehindElevator) {
          // It needs to go through the vertical position, assume max length
          maxArmExtensionDuringRotation = armLength;
        } else {
          // Arm staying on the same side of the elevator,
          // get the highest between current and target position.
          var targetArmVerticalOffset = armLength * targetArmAngle.getSin();
          maxArmExtensionDuringRotation =
              Math.max(targetArmVerticalOffset, currentArmVerticalOffset);
        }

        log_maxArmExtensionDuringRotation.info(maxArmExtensionDuringRotation);

        var maxElevatorHeadroom = maxLegalHeightInches - maxArmExtensionDuringRotation;
        var currentSafeHeight =
            Math.min(maxElevatorHeadroom - elevatorToleranceInches, targetElevatorHeightInches);

        String actionName;
        var safeArmAngle = targetArmAngle;

        if (targetArmAngleDegrees < safeAngleIntakeDegrees) {
          //
          // Arm moving to the home position, more or less.
          //
          if (currentArmAngleDegrees < safeAngleIntakeDegrees) {
            //
            // Safe to move to target position directly
            //
            log_state.info("HomeDirectly");
            // elevator.setHeight(targetElevatorHeight);
            arm.setAngle(targetArmAngle);
            return;
          }

          //
          // Move elevator at most to the safe height.
          //
          currentSafeHeight = Math.max(safeElevatorHeightInchesForIntake, currentSafeHeight);

          actionName = "Home";
        } else {
          actionName = "Target";

          if (targetArmAngleDegrees > 90 && currentArmAngleDegrees < 90) {
            if (currentElevatorHeightInches < MechanismPositions.reefElevatorHeightInches.get()) {
              // Elevator too low to move arm all the way back.
              safeArmAngle = Rotation2d.fromDegrees(90);
            }
          }
        }

        log_currentSafeHeight.info(currentSafeHeight);

        boolean aboveMinimumHeight =
            currentElevatorHeightInches > currentSafeHeight - elevatorToleranceInches;

        boolean belowMaximumHeight =
            currentElevatorHeightInches + maxArmExtensionDuringRotation < maxLegalHeightInches;

        // We are not going to exceed the maximum legal height, start rotation.
        if (aboveMinimumHeight && belowMaximumHeight) {
          arm.setAngle(safeArmAngle);
          log_state.info(actionName + "Arm");
        } else {
          log_state.info(actionName + "NoArm");
        }

        elevator.setHeight(Units.Inches.of(currentSafeHeight));
      }

      @Override
      public boolean isFinished() {
        return elevator.isAtTarget(targetElevatorHeight)
            && arm.isAtTargetAngle(targetArmAngle, armTolerance);
      }
    };
  }
}
