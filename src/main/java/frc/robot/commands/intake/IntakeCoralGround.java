package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.robot.RobotStates;
import frc.robot.RobotStates.IntakeState;
import frc.robot.RobotStates.MechState;
import frc.robot.RobotStates.ScoringLevel;
import frc.robot.subsystems.intake.Intake;

public class IntakeCoralGround extends Command {
  // boolean coralInIntake = RobotStates.coralInIntake;
  private final Intake intake;
  private final RobotStates states;
  private static final String root = "Intake Coral Ground";
  private static final LoggerGroup logGroup = LoggerGroup.build(root);

  private static final LoggerEntry.Bool logInputs_algaeInRobot =
      logGroup.buildBoolean("algaeInRobot");

  private static final LoggerEntry.Bool logInputs_coralInIntake =
      logGroup.buildBoolean("coralInIntake");

  /** Creates a new IntakeGround */
  public IntakeCoralGround(Intake intake, RobotStates states) {

    this.intake = intake;
    this.states = states;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (states.scoringLevel == ScoringLevel.L1) {
      states.mechState = MechState.AvoidIntake;
    } else states.mechState = MechState.PassoffPosition;
    states.intakeState = IntakeState.IntakeCoral;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    states.mechanismInUse = true;
    states.intakeInUse = true;
    if (states.scoringLevel == ScoringLevel.L1) {
      states.mechState = MechState.AvoidIntake;
    } else states.mechState = MechState.PassoffPosition;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (states.coralInIntake) {
      if (states.scoringLevel == ScoringLevel.L1) {
        states.mechState = MechState.AvoidIntake;
        states.intakeState = IntakeState.Stow;
      } else states.mechState = MechState.PassoffPosition;
    } else {
      states.intakeState = IntakeState.Stow;
      states.mechState = MechState.CoralStationPosition;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return states.triggerForCoralInIntake.getAsBoolean();
  }
}
