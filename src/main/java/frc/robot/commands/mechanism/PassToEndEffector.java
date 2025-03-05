package frc.robot.commands.mechanism;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants.PivotConstants;
import frc.robot.RobotStates;
import frc.robot.RobotStates.EndEffectorDesiredAction;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

public class PassToEndEffector extends Command {
  private static final TunableNumberGroup group = new TunableNumberGroup("PassToEndEffector");
  private static final LoggedTunableNumber intakingVelocity = group.build("intakingVelocity", 500);
  private final Intake intake;
  private final Arm arm;
  private final Elevator elevator;
  private boolean coralAtStartOfCommand;
  private final Trigger gamepieceInEndEffector =
      new Trigger(() -> RobotStates.coralInIntake).debounce(0.5);

  public PassToEndEffector(Intake intake, Arm arm, Elevator elevator) {

    this.intake = intake;
    this.arm = arm;
    this.elevator = elevator;

    addRequirements(intake);
    addRequirements(arm);
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    coralAtStartOfCommand = RobotStates.coralInIntake;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setPivotAngle(PivotConstants.PASSOFF_PIVOT_ANGLE);
    intake.setRollerPercentOut(0);
    MechanismActions.passOffPosition(elevator, arm);
    RobotStates.endEffectorDesiredAction = EndEffectorDesiredAction.GroundIntake;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotStates.endEffectorDesiredAction = EndEffectorDesiredAction.AlignCoral;
    intake.setPivotAngle(PivotConstants.HOME_POSITION);
    elevator.setHeight(ElevatorConstants.SAFE_HEIGHT);
    MechanismActions.coralStationPosition(elevator, arm);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotStates.coralInEndEffector || !coralAtStartOfCommand;
  }
}
