// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotIntakeToAngle extends Command {
  /** Creates a new PivotIntake. */
  private Intake intake;

  private Supplier<Rotation2d> angleSupplier;

  public PivotIntakeToAngle(Intake intake, Rotation2d angle) {
    this(intake, () -> angle);
  }

  public PivotIntakeToAngle(Intake intake, Supplier<Rotation2d> angleSupplier) {
    this.intake = intake;
    this.angleSupplier = angleSupplier;

    addRequirements(intake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setPivotAngle(angleSupplier.get());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
