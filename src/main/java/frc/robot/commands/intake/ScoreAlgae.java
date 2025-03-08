// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.TunableNumberGroup;
import frc.robot.RobotStates;
import frc.robot.RobotStates.IntakeState;
import frc.robot.subsystems.intake.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreAlgae extends Command {
  /** Creates a new ScoreAlgae. */
  // REMEMBER TO UPDATE CONSTANTS AFTER CHANGING TUNABLE NUMBERS! THESE ARE JUST HERE TO DEBUG AND
  // TEST.
  private static final TunableNumberGroup group = new TunableNumberGroup("ScoreAlgae");

  private final Intake intake;

  public ScoreAlgae(Intake intake) {
    this.intake = intake;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotStates.intakeState = IntakeState.ScoreAlgae;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.holdAlgae = false;
    RobotStates.intakeState = IntakeState.Stow;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !RobotStates.algaeInRobot;
  }
}
