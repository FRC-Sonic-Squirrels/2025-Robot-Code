package frc.robot.commands.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotStates;
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

  private static final TunableNumberGroup group = new TunableNumberGroup(root);

  private static final LoggedTunableNumber intakingPivotAngle = group.build("IntakingAngleDeg", 0);
  private static final LoggedTunableNumber intakingVel = group.build("intakingVel", 2000);

  /** Creates a new IntakeGround */
  public IntakeCoralGround(Intake intake) {

    this.intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setPivotAngle(Rotation2d.fromDegrees(intakingPivotAngle.get()));
    intake.setRollerVelocity(intakingVel.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    logInputs_algaeInRobot.info(RobotStates.algaeInRobot);
    logInputs_coralInIntake.info(RobotStates.coralInIntake);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setRollerPercentOut(0);
    intake.setPivotAngle(IntakeConstants.PivotConstants.PIVOT_STOWED_ANGLE);
    // if (RobotStates.algaeInRobot && !RobotStates.coralInIntake) {
    //   intake.setHoldAlgae(true);
    // }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // RobotStates.coralInIntake;
  }
}
