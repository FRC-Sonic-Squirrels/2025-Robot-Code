// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.robot.Constants.IntakeConstants.PivotConstants;
import frc.robot.RobotStates;
import frc.robot.subsystems.intake.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAlgae extends Command {
  /** Creates a new IntakeAlgae. */
  private static final LoggerGroup logGroup = LoggerGroup.build("Intake Algae");

  private static final LoggerEntry.Bool logInputs_algaeInRobot =
      logGroup.buildBoolean("algaeIntRobot");

  private static final LoggerEntry.Bool logInputs_rotateAtPos =
      logGroup.buildBoolean("rotateatPos");

  private Intake intake;

  private Trigger end =
      new Trigger(() -> RobotStates.algaeInRobot || intake.isPivotAtTargetAngle()).debounce(0.5);

  public IntakeAlgae(Intake intake) {
    this.intake = intake;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setRollerVelocity(-1000.0);
    intake.setPivotAngle(PivotConstants.EXTERNAL_ANGLE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // check if tof for intake is ful if so rotate to pos and end
    logInputs_algaeInRobot.info(RobotStates.algaeInRobot);
    logInputs_rotateAtPos.info(intake.isPivotAtTargetAngle());
    if (intake.timeOfFlight()) {
      RobotStates.algaeInRobot = true;
      intake.setPivotAngle(PivotConstants.ALGAE_SCORE_ANGLE);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setRollerPercentOut(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // !end.getAsBoolean();
  }
}
