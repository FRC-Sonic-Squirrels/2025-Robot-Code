// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.RobotStates;

public class IntakeGamepiece extends Command {
  private static final TunableNumberGroup group = new TunableNumberGroup("IntakeGamepiece");
  private static final LoggedTunableNumber rumbleIntensityPercent =
      group.build("rumbleIntensityPercent", 0.5);
  private static final LoggedTunableNumber intakingVelocity = group.build("intakingVelocity", 2500);

  /** Creates a new IntakeDefaultIdleRPM. */
  public IntakeGamepiece() {
    // TODO: add subsystems

    // TODO: add subsystem requirements
    addRequirements();
    setName("IntakeGamepiece");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var rumbleValue = rumbleIntensityPercent.get();

    // TODO: add logic to intake gamepiece
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // TODO: stop intaking
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO: change to: if gamepiece is seen (debounced)
    return RobotStates.coralInRobot;
  }
}
