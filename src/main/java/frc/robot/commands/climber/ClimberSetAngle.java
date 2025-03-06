// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import java.util.function.Supplier;

public class ClimberSetAngle extends Command {
  /** Creates a new ArmSetAngle. */
  Climber climber;

  Supplier<Rotation2d> angleSupplier;

  public ClimberSetAngle(Climber climber, Rotation2d angle) {
    this(climber, () -> angle);
  }

  public ClimberSetAngle(Climber climber, Supplier<Rotation2d> angleSupplier) {
    this.climber = climber;
    this.angleSupplier = angleSupplier;

    addRequirements(climber);
  }

  @Override
  public void execute() {
    climber.setWinchAngle(angleSupplier.get());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
