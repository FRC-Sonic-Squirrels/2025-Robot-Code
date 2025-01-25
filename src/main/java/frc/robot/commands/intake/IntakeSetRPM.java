// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import java.util.function.DoubleSupplier;

public class IntakeSetRPM extends Command {

  private Intake intake;
  private DoubleSupplier RPM;

  /** Creates a new IntakeSetRPM. */
  public IntakeSetRPM(Intake intake, DoubleSupplier RPM) {

    this.intake = intake;
    this.RPM = RPM;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  public IntakeSetRPM(Intake intake, double RPM) {
    this(intake, () -> RPM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setRollerVelocity(RPM.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setRollerPercentOut(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
