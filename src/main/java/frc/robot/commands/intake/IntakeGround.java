package frc.robot.commands.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants.IntakeConstants.PivotConstants;
import frc.robot.RobotStates;
import frc.robot.subsystems.intake.Intake;

public class IntakeGround extends Command {
  // boolean coralInIntake = RobotStates.coralInIntake;
  private final Intake intake;
  private final Trigger gamepieceInIntake =
      new Trigger(() -> RobotStates.coralInIntake || RobotStates.algaeInRobot).debounce(0.25);

  private static final LoggerGroup logGroup = LoggerGroup.build("Intake Gamepiece");

  private static final LoggerEntry.Bool logInputs_algaeInRobot =
      logGroup.buildBoolean("algaeInRobot");

  private static final LoggerEntry.Bool logInputs_coralInIntake =
      logGroup.buildBoolean("coralInIntake");

  private static final TunableNumberGroup group = new TunableNumberGroup("Intake Gamepiece");

  private static final LoggedTunableNumber maximumPivotAngle =
      group.build("maxPivotAngle", PivotConstants.MAX_PIVOT_ANGLE.getDegrees());

  /** Creates a new IntakeGround */
  public IntakeGround(Intake intake) {

    this.intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setPivotAngle(Rotation2d.fromDegrees(maximumPivotAngle.get()));
    intake.setRollerVelocity(PivotConstants.GAMEPIECE_INTAKE_SPEED_RPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    logInputs_algaeInRobot.info(RobotStates.algaeInRobot);
    logInputs_coralInIntake.info(RobotStates.coralInIntake);
    if (gamepieceInIntake.getAsBoolean()) {
      intake.setPivotAngle(PivotConstants.ALGAE_SCORE_ANGLE);
      intake.setRollerPercentOut(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return gamepieceInIntake.getAsBoolean() && intake.isPivotAtTargetAngle();
  }
}
