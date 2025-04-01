package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStates;
import frc.robot.RobotStates.IntakeState;
import frc.robot.RobotStates.MechState;
import frc.robot.subsystems.intake.Intake;

public class IntakeAlgaeGround extends Command {
  private final Intake intake;
  private final RobotStates states;

  /** Creates a new IntakeGround */
  public IntakeAlgaeGround(Intake intake, RobotStates states) {

    this.intake = intake;
    this.states = states;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    states.coralExpectedInIntake = false;
    states.mechState = MechState.StowPosition;
    states.intakeState = IntakeState.IntakeAlgae;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    states.intakeState = IntakeState.Stow;
    if (states.algaeInRobot && !states.coralInIntake) {
      intake.setHoldAlgae(true);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return states.coralInIntake;
  }
}
