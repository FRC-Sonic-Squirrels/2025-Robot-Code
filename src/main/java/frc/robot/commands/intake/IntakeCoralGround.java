package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.robot.RobotStates;
import frc.robot.RobotStates.IntakeState;
import frc.robot.RobotStates.MechState;
import frc.robot.subsystems.intake.Intake;

public class IntakeCoralGround extends Command {
  // boolean coralInIntake = RobotStates.coralInIntake;
  private final Intake intake;
  private static final String root = "Intake Coral Ground";
  private static final LoggerGroup logGroup = LoggerGroup.build(root);

  private static final LoggerEntry.Bool logInputs_algaeInRobot =
      logGroup.buildBoolean("algaeInRobot");

  private static final LoggerEntry.Bool logInputs_coralInIntake =
      logGroup.buildBoolean("coralInIntake");

  /** Creates a new IntakeGround */
  public IntakeCoralGround(Intake intake) {

    this.intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotStates.mechState = MechState.PassoffPosition;
    RobotStates.intakeState = IntakeState.IntakeCoral;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      RobotStates.intakeState = IntakeState.PrepPassoff;
    } else RobotStates.intakeState = IntakeState.PrepPassoff;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotStates.triggerForCoralInIntake.getAsBoolean();
  }
}
