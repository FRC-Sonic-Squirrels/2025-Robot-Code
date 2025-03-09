// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.swerve.DrivetrainWrapper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WheelRadiusCharacterization extends Command {
  /** Creates a new WheelRadiusCharacterization. */
  private static final LoggerGroup loggerGroup = LoggerGroup.build("WheelRadiusCharacterization");

  private static final LoggerEntry.Decimal loggerCurrentEstimatedRadius =
      loggerGroup.buildDecimal("CurrentEstimatedRadius");

  private static final TunableNumberGroup group =
      new TunableNumberGroup("WheelRadiusCharacterization");
  // radians per second
  private static final LoggedTunableNumber characterizationSpeed =
      group.build("characterizationSpeed", 0.5);

  private final DrivetrainWrapper drivetrainWrapper;
  // inches
  private final double driveBaseRadius;
  // radians
  private double lastYaw;
  // radians
  private double totalYaw;
  // radians
  private Angle[] initialWheelRotations;

  // rotates the robot in place at characterizationSpeed rads/sec
  // and outputs the radius of the wheels in inches
  public WheelRadiusCharacterization(DrivetrainWrapper drivetrainWrapper, RobotConfig config) {
    this.drivetrainWrapper = drivetrainWrapper;
    driveBaseRadius = Units.Meters.of(config.getDriveBaseRadius()).in(Units.Inches);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastYaw = drivetrainWrapper.getReefPoseEstimatorPose(true).getRotation().getRadians();
    totalYaw = 0.0;
    initialWheelRotations = drivetrainWrapper.getModuleRotations();

    drivetrainWrapper.setRotationOverride(characterizationSpeed.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    totalYaw +=
        MathUtil.angleModulus(
            drivetrainWrapper.getReefPoseEstimatorPose(true).getRotation().getRadians() - lastYaw);
    lastYaw = drivetrainWrapper.getReefPoseEstimatorPose(true).getRotation().getRadians();

    double averageWheelRotation = 0.0;
    Angle[] wheelRotations = drivetrainWrapper.getModuleRotations();
    for (int i = 0; i < 4; i++) {
      averageWheelRotation +=
          Math.abs(
              wheelRotations[i].in(Units.Radians) - initialWheelRotations[i].in(Units.Radians));
    }
    averageWheelRotation /= 4;
    double currentEstimatedRadius = (totalYaw * driveBaseRadius) / averageWheelRotation;

    loggerCurrentEstimatedRadius.info(currentEstimatedRadius);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainWrapper.resetRotationOverride();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // do two rotations before ending
    return totalYaw > Math.PI * 4;
  }
}
