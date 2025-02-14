package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants.PivotConstants;
import frc.robot.Constants.MotorConstants.KrakenConstants;
import frc.robot.RobotStates;
import frc.robot.subsystems.intake.Intake;

public class IntakeGround extends Command {
  // boolean coralInIntake = RobotStates.coralInIntake;
  private Intake intake;
  private final Trigger gamepieceInRobot =
      new Trigger(() -> RobotStates.coralInIntake).debounce(0.5);

  /** Creates a new IntakGround */
  public IntakeGround(Intake intake) {

    this.intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setPivotAngle(PivotConstants.MAX_PIVOT_ANGLE);
    intake.setRollerVelocity(KrakenConstants.FREE_SPEED_RPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setPivotAngle(PivotConstants.MIN_PIVOT_ANGLE);
    intake.setRollerVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotStates.coralInIntake;
  }
}
