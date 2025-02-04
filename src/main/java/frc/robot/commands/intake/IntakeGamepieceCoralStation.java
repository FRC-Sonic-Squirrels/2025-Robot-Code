// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.RobotStates;
import frc.robot.subsystems.endEffector.EndEffector;

public class IntakeGamepieceCoralStation extends Command {
  private static final TunableNumberGroup group = new TunableNumberGroup("IntakeGamepiece");
  private static final LoggedTunableNumber intakingVelocity = group.build("intakingVelocity", -500);
  private final EndEffector endEffector;
  private final Trigger gamepieceInRobot =
      new Trigger(() -> RobotStates.coralInEndEffectorNonScoringSide || RobotStates.coralInIntake)
          .debounce(0.5);

  /** Creates a new IntakeDefaultIdleRPM. */
  public IntakeGamepieceCoralStation(EndEffector endEffector) {
    this.endEffector = endEffector;
    addRequirements(endEffector);
    setName("IntakeGamepieceCoralStation");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((RobotStates.coralInEndEffectorScoringSide && RobotStates.coralInEndEffectorNonScoringSide)
        || RobotStates.coralInIntake) {
      endEffector.setPercentOut(0);
      endEffector.setGamepieceInRobot(true);
    } else {
      endEffector.setVelocity(intakingVelocity.get());
      endEffector.setGamepieceInRobot(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.setPercentOut(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return gamepieceInRobot.getAsBoolean();
  }
}
