// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.RobotStates;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.intake.Intake;

public class IntakeGamepieceCoralStation extends Command {
  private static final TunableNumberGroup group = new TunableNumberGroup("IntakeGamepiece");
  private static final LoggedTunableNumber rumbleIntensityPercent =
      group.build("rumbleIntensityPercent", 0.5);
  private static final LoggedTunableNumber intakingVelocity = group.build("intakingVelocity", 2500);
  private final Intake intake;
  private final EndEffector endEffector;

  /** Creates a new IntakeDefaultIdleRPM. */
  public IntakeGamepieceCoralStation(Intake intake, EndEffector endEffector) {
    // TODO: add subsystems
    this.intake = intake;
    this.endEffector = endEffector;
    // TODO: add subsystem requirements
    addRequirements(intake);
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
    if (RobotStates.coralInEndEffector) {
      intake.setRollerVelocity(0);
      endEffector.setVelocity(0);

    } else {
      if (RobotStates.coralInIntake) {
        intake.setRollerVelocity(0);
        endEffector.setVelocity(0);

      } else {
        intake.setRollerVelocity(intakingVelocity.get());
        endEffector.setVelocity(intakingVelocity.get());
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // TODO: stop intaking
    intake.setRollerVelocity(0.0);
    endEffector.setVelocity(0.0);
    intake.setVelocity(0.0);
    endEffector.setVelocity(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO: change to: if gamepiece is seen (debounced)
    return RobotStates.coralInRobot;
  }
}
