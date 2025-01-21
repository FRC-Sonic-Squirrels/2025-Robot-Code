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
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.swerve.DrivetrainWrapper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WheelRadiusCharacterization extends Command {
  /** Creates a new WheelRadiusCharacterization. */
  private static final LoggerGroup loggerGroup = LoggerGroup.build("WheelRadiusCharacterization");

  private static final LoggerEntry.Decimal loggerCurrentEstimatedRadius =
      loggerGroup.buildDecimal("CurrentEstimatedRadius");
  private static final LoggerEntry.Decimal loggerCurrentEstimatedRadius0 =
      loggerGroup.buildDecimal("CurrentEstimatedRadius0");
  private static final LoggerEntry.Decimal loggerCurrentEstimatedRadius1 =
      loggerGroup.buildDecimal("CurrentEstimatedRadius1");
  private static final LoggerEntry.Decimal loggerCurrentEstimatedRadius2 =
      loggerGroup.buildDecimal("CurrentEstimatedRadius2");
  private static final LoggerEntry.Decimal loggerCurrentEstimatedRadius3 =
      loggerGroup.buildDecimal("CurrentEstimatedRadius3");
  private static final LoggerEntry.Decimal loggerCurrentYaw =
      loggerGroup.buildDecimal("CurrentYaw");
  private static final LoggerEntry.Decimal loggerCurrentAverageWheelPos =
      loggerGroup.buildDecimal("CurrentAverageWheelPos");
  private final DrivetrainWrapper drivetrainWrapper;
  // radians per second
  private final double characterizationSpeed = 0.5;
  // inches
  private final double driveBaseRadius;
  // inches
  public double currentEstimatedRadius;
  // radians
  private double lastYaw;
  // radians
  private double totalYaw;
  // radians
  private Angle[] initialWheelRotations;

  public WheelRadiusCharacterization(DrivetrainWrapper drivetrainWrapper, RobotConfig config) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrainWrapper = drivetrainWrapper;
    driveBaseRadius = Units.Meters.of(config.getDriveBaseRadius()).in(Units.Inches);
    // addRequirements(drivetrainWrapper.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastYaw = drivetrainWrapper.getPoseEstimatorPose(true).getRotation().getRadians();
    totalYaw = 0.0;
    initialWheelRotations = drivetrainWrapper.getModuleRotations();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrainWrapper.setRotationOverride(characterizationSpeed);
    drivetrainWrapper.apply();

    totalYaw +=
        MathUtil.angleModulus(
            drivetrainWrapper.getPoseEstimatorPose(true).getRotation().getRadians() - lastYaw);
    lastYaw = drivetrainWrapper.getPoseEstimatorPose(true).getRotation().getRadians();

    double averageWheelRotation = 0.0;
    Angle[] wheelRotations = drivetrainWrapper.getModuleRotations();
    for (int i = 0; i < 4; i++) {
      averageWheelRotation +=
          wheelRotations[i].in(Units.Radians) - initialWheelRotations[i].in(Units.Radians);
    }
    // TODO: wheel rotation slows down after a bit? makes the radius go up to an incorrect value
    averageWheelRotation /= 4;
    double currentEstimatedRadius = (totalYaw * driveBaseRadius) / averageWheelRotation;

    double currentEstimatedRadius0 =
        (totalYaw * driveBaseRadius)
            / (wheelRotations[0].in(Units.Radians) - initialWheelRotations[0].in(Units.Radians));
    double currentEstimatedRadius1 =
        (totalYaw * driveBaseRadius)
            / (wheelRotations[1].in(Units.Radians) - initialWheelRotations[1].in(Units.Radians));
    double currentEstimatedRadius2 =
        (totalYaw * driveBaseRadius)
            / (wheelRotations[2].in(Units.Radians) - initialWheelRotations[2].in(Units.Radians));
    double currentEstimatedRadius3 =
        (totalYaw * driveBaseRadius)
            / (wheelRotations[3].in(Units.Radians) - initialWheelRotations[3].in(Units.Radians));

    loggerCurrentEstimatedRadius.info(currentEstimatedRadius);
    loggerCurrentEstimatedRadius0.info(currentEstimatedRadius0);
    loggerCurrentEstimatedRadius1.info(currentEstimatedRadius1);
    loggerCurrentEstimatedRadius2.info(currentEstimatedRadius2);
    loggerCurrentEstimatedRadius3.info(currentEstimatedRadius3);
    loggerCurrentYaw.info(totalYaw);
    loggerCurrentAverageWheelPos.info(averageWheelRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainWrapper.resetRotationOverride();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
