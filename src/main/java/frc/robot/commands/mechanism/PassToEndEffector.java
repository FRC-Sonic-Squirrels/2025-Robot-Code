package frc.robot.commands.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStates;
import frc.robot.RobotStates.EndEffectorDesiredAction;
import frc.robot.RobotStates.IntakeState;
import frc.robot.RobotStates.MechState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.mechanism.MechanismPositions;
import frc.robot.subsystems.mechanism.MechanismPositions.MechanismPosition;
import frc.robot.subsystems.mechanism.arm.Arm;
import frc.robot.subsystems.mechanism.elevator.Elevator;

public class PassToEndEffector extends Command {
  private final Arm arm;
  private final Elevator elevator;
  private final Intake intake;

  public PassToEndEffector(Arm arm, Elevator elevator, Intake intake) {
    this.arm = arm;
    this.elevator = elevator;
    this.intake = intake;
  }

  @Override
  public void initialize() {
    RobotStates.mechState = MechState.PassoffPosition;
    RobotStates.intakeState = IntakeState.PrepPassoff;
    if (!RobotStates.coralInIntake) this.cancel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    MechanismPosition handOffPosition = MechanismPositions.intakeToEndEffectorPassOffPosition();
    if (elevator.isAtTarget(handOffPosition.elevatorHeight())
        && arm.isAtTargetAngle(handOffPosition.armAngle(), Rotation2d.fromDegrees(3))
        && intake.isPivotAtTargetAngle(intake.getPassOffPivotAngle())) {
      RobotStates.intakeState = IntakeState.Passoff;
    }

    RobotStates.endEffectorDesiredAction = EndEffectorDesiredAction.PassToEndEffector;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotStates.intakeState = IntakeState.Stow;
    RobotStates.endEffectorDesiredAction = EndEffectorDesiredAction.AlignCoral;
    RobotStates.mechState = MechState.StowPosition;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotStates.coralInEndEffectorScoringSide;
  }
}
