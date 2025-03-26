package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStates;
import frc.robot.RobotStates.IntakeState;
import frc.robot.RobotStates.MechState;
import frc.robot.RobotStates.ScoringLevel;
import frc.robot.subsystems.intake.Intake;

public class IntakeCoralGround extends Command {
  private final RobotStates states;

  /** Creates a new IntakeGround */
  public IntakeCoralGround(Intake intake, RobotStates states) {

    this.states = states;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (states.scoringLevel == ScoringLevel.L1) {
      states.mechState = MechState.AvoidIntake;
    } else states.mechState = MechState.PrepPassoffPosition;
    states.intakeState = IntakeState.IntakeCoral;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    states.mechanismInUse = true;
    states.intakeInUse = true;
    if (states.scoringLevel == ScoringLevel.L1) {
      states.mechState = MechState.AvoidIntake;
    } else states.mechState = MechState.PrepPassoffPosition;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (states.coralInIntake) {
      if (states.scoringLevel == ScoringLevel.L1) {
        states.mechState = MechState.AvoidIntake;
        states.intakeState = IntakeState.Stow;
      } else states.mechState = MechState.PrepPassoffPosition;
    } else {
      states.intakeState = IntakeState.Stow;
      states.mechState = MechState.Default;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return states.triggerForCoralInIntake.getAsBoolean();
  }
}
