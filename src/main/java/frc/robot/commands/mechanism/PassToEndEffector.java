package frc.robot.commands.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

  private final Trigger endTrigger =
      new Trigger(() -> RobotStates.coralInEndEffectorScoringSide).debounce(0.3);

  public PassToEndEffector(Arm arm, Elevator elevator, Intake intake) {
    this.arm = arm;
    this.elevator = elevator;
    this.intake = intake;
  }

  @Override
  public void initialize() {
    RobotStates.mechState = MechState.PrepPassoffPosition;
    RobotStates.intakeState = IntakeState.PrepPassoff;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    MechanismPosition prepHandOffPosition = MechanismPositions.prepForPassoffPosition();
    boolean ele = elevator.isAtTarget(prepHandOffPosition.elevatorHeight());
    boolean armp = arm.isAtTargetAngle(prepHandOffPosition.armAngle(), Rotation2d.fromDegrees(3));
    boolean inta =
        intake.isPivotAtTargetAngle(intake.getPassOffPivotAngle(), Rotation2d.fromDegrees(5));
    System.out.println("ele: " + ele + "arm: " + armp + "intake: " + inta);
    if (ele && armp && inta) {
      RobotStates.mechState = MechState.PassoffPosition;
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
    return endTrigger.getAsBoolean();
  }
}
