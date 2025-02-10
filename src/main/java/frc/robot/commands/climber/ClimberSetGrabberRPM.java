// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import java.util.function.DoubleSupplier;

public class ClimberSetGrabberRPM extends Command {

  private Climber climber;
  private DoubleSupplier RPM;

  /** Creates a new IntakeSetRPM. */
  public ClimberSetGrabberRPM(Climber climber, DoubleSupplier RPM) {

    this.climber = climber;
    this.RPM = RPM;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  public ClimberSetGrabberRPM(Climber climber, double RPM) {
    this(climber, () -> RPM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setGrabberVelocity(RPM.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setGrabberPercentOut(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
