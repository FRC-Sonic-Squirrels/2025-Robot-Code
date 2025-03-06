// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStates;
import java.util.function.DoubleSupplier;

public class EndEffectorSetRPM extends Command {
  private final DoubleSupplier RPM;

  /** Creates a new IntakeSetRPM. */
  public EndEffectorSetRPM(DoubleSupplier RPM) {
    this.RPM = RPM;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  public EndEffectorSetRPM(double RPM) {
    this(() -> RPM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotStates.endEffectorOverrideVelocity = RPM.getAsDouble();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotStates.endEffectorDesiredAction = RobotStates.EndEffectorDesiredAction.Idle;
    RobotStates.endEffectorOverrideVelocity = Double.NaN;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
