// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.endEffector.EndEffector;
import java.util.function.DoubleSupplier;

public class EndEffectorSetRPM extends Command {

  private EndEffector endEffector;
  private DoubleSupplier RPM;

  /** Creates a new IntakeSetRPM. */
  public EndEffectorSetRPM(EndEffector endEffector, DoubleSupplier RPM) {

    this.endEffector = endEffector;
    this.RPM = RPM;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(endEffector);
  }

  public EndEffectorSetRPM(EndEffector endEffector, double RPM) {
    this(endEffector, () -> RPM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    endEffector.setVelocity(RPM.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.setPercentOut(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
