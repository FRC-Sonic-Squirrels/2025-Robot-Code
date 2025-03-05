package frc.robot.commands.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants.IntakeConstants.PivotConstants;
import frc.robot.RobotStates;
import frc.robot.RobotStates.EndEffectorDesiredAction;
import frc.robot.commands.mechanism.MechanismPositions.MechanismPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

public class PassToEndEffector extends Command {
  private static final TunableNumberGroup group = new TunableNumberGroup("PassToEndEffector");
  private static final LoggedTunableNumber intakeVelocity = group.build("intakeVel", -500);
  private final Intake intake;
  private final Arm arm;
  private final Elevator elevator;
  private boolean coralAtStartOfCommand;

  public PassToEndEffector(Intake intake, Arm arm, Elevator elevator) {

    this.intake = intake;
    this.arm = arm;
    this.elevator = elevator;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    coralAtStartOfCommand = RobotStates.coralInIntake;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setPivotAngle(PivotConstants.PASSOFF_PIVOT_ANGLE);
    MechanismPosition handOffPosition = MechanismPositions.intakeToEndEffectorPassOffPosition();
    if (elevator.isAtTarget(handOffPosition.elevatorHeight())
        && arm.isAtTargetAngle(handOffPosition.armAngle(), Rotation2d.fromDegrees(3)))
      intake.setRollerVelocity(intakeVelocity.get());
    RobotStates.endEffectorDesiredAction = EndEffectorDesiredAction.PassToEndEffector;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotStates.endEffectorDesiredAction = EndEffectorDesiredAction.AlignCoral;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotStates.coralInEndEffectorScoringSide || !coralAtStartOfCommand;
  }
}
