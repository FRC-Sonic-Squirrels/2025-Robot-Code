// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.RobotMode.RobotType;

public class Arm extends SubsystemBase {
  // Execution timing
  private static final ExecutionTiming timing = new ExecutionTiming(ArmConstants.ROOT_TABLE);

  // Logging
  private static final LoggerGroup logGroup = LoggerGroup.build(ArmConstants.ROOT_TABLE);
  private static final LoggerEntry.Decimal logInputs_angle = logGroup.buildDecimal("AngleDegrees");
  private static final LoggerEntry.Decimal logInputs_appliedVolts =
      logGroup.buildDecimal("AppliedVolts");
  private static final LoggerEntry.Decimal logInputs_currentAmps =
      logGroup.buildDecimal("CurrentAmps");
  private static final LoggerEntry.Decimal logInputs_tempCelsius =
      logGroup.buildDecimal("TempCelsius");
  private static final LoggerEntry.Decimal logInputs_velocityDegreesPerSecond =
      logGroup.buildDecimal("VelocityDegreesPerSecond");
  private static final LoggerEntry.EnumValue<ControlMode> logControlMode =
      logGroup.buildEnum("ControlMode");
  private static final LoggerEntry.Decimal logTargetAngleDegrees =
      logGroup.buildDecimal("targetAngleDegrees");

  // Tunable Numbers
  private static final TunableNumberGroup group = new TunableNumberGroup(ArmConstants.ROOT_TABLE);

  private static final LoggedTunableNumber kP = group.build("kP");
  private static final LoggedTunableNumber kD = group.build("kD");
  private static final LoggedTunableNumber kG = group.build("kG");

  private static final LoggedTunableNumber maxVelocityConfig = group.build("MaxVelocityConfig");
  private static final LoggedTunableNumber targetAccelerationConfig =
      group.build("TargetAccelerationConfig");
  private static final LoggedTunableNumber toleranceDegrees = group.build("ToleranceDegrees", 1);

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2024_RETIRED_MAESTRO) {
      kP.initDefault(70.0);
      kD.initDefault(1.6);
      kG.initDefault(0.0);

      // FIXME: find the theoretical from the JVN docs
      maxVelocityConfig.initDefault(10);
      targetAccelerationConfig.initDefault(10);
    } else if (Constants.RobotMode.getRobot() == RobotType.ROBOT_SIMBOT) {

      kP.initDefault(4);
      kD.initDefault(0);
      kG.initDefault(0.0);

      maxVelocityConfig.initDefault(40);
      targetAccelerationConfig.initDefault(80);
    } else {

      kP.initDefault(50);
      kD.initDefault(0.06);
      kG.initDefault(0.65);

      maxVelocityConfig.initDefault(640);
      targetAccelerationConfig.initDefault(1500);
    }
  }

  private double setAccel = targetAccelerationConfig.get();

  private final ArmIO io;
  private final ArmIO.Inputs inputs = new ArmIO.Inputs(logGroup);

  private ControlMode controlMode = ControlMode.OPEN_LOOP;
  private Rotation2d targetAngleDegrees = Constants.zeroRotation2d;

  /** Creates a new ArmSubsystem. */
  public Arm(ArmIO io) {
    this.io = io;

    io.setVoltage(0.0);

    setConstants();
  }

  @Override
  public void periodic() {
    try (var ignored = timing.start()) {
      // Arm logging
      io.updateInputs(inputs);
      logInputs_angle.info(inputs.armPosition);
      logInputs_appliedVolts.info(inputs.armAppliedVolts);
      logInputs_currentAmps.info(inputs.armCurrentAmps);
      logInputs_tempCelsius.info(inputs.armTempCelsius);
      logInputs_velocityDegreesPerSecond.info(inputs.armVelocityDegreesPerSecond);

      logControlMode.info(controlMode);

      // Updating tunable numbers
      var hc = hashCode();
      if (kP.hasChanged(hc)
          || kD.hasChanged(hc)
          || kG.hasChanged(hc)
          || maxVelocityConfig.hasChanged(hc)
          || targetAccelerationConfig.hasChanged(hc)) {
        setConstants();
      }
    }
  }

  // Setters

  private void setConstants() {
    MotionMagicConfigs configs = new MotionMagicConfigs();
    configs.MotionMagicCruiseVelocity = maxVelocityConfig.get();
    configs.MotionMagicAcceleration = targetAccelerationConfig.get();
    io.setClosedLoopConstants(kP.get(), kD.get(), kG.get(), configs);
    setAccel = targetAccelerationConfig.get();
  }

  public void setAngle(Rotation2d angle) {
    setAngle(angle, targetAccelerationConfig.get());
  }

  public void setAngle(Rotation2d angle, double accel) {
    angle =
        Rotation2d.fromRadians(
            MathUtil.clamp(
                angle.getRadians(),
                Constants.ArmConstants.MIN_ARM_ANGLE.getRadians(),
                Constants.ArmConstants.MAX_ARM_ANGLE.getRadians()));

    controlMode = ControlMode.CLOSED_LOOP;
    targetAngleDegrees = angle;
    if (accel != setAccel) {
      MotionMagicConfigs mmConfigs = new MotionMagicConfigs();

      mmConfigs.MotionMagicCruiseVelocity = maxVelocityConfig.get();
      mmConfigs.MotionMagicAcceleration = accel;
      setAccel = accel;

      io.setClosedLoopConstants(kP.get(), kD.get(), kG.get(), mmConfigs);
    }
    io.setClosedLoopPosition(angle);
    logTargetAngleDegrees.info(targetAngleDegrees);
  }

  public void resetSubsystem() {
    controlMode = ControlMode.OPEN_LOOP;
    io.setVoltage(0);
  }

  public void setVoltage(double percent) {
    controlMode = ControlMode.OPEN_LOOP;
    io.setVoltage(percent);
  }

  public void resetSensorToHomePosition() {
    io.resetSensorPosition(Constants.ArmConstants.HOME_POSITION);
  }

  public boolean setNeutralMode(NeutralModeValue value) {
    return io.setNeutralMode(value);
  }

  public void setArmManualControl(double percent) {
    setVoltage(2 * (percent) + kG.get() * Math.cos(getAngle().getRadians()));
  }

  // Getters

  public Rotation2d getAngle() {
    return inputs.armPosition;
  }

  public boolean isAtTargetAngle() {
    return isAtTargetAngle(targetAngleDegrees);
  }

  public boolean isAtTargetAngle(Rotation2d target, Rotation2d tolerance) {
    var error = inputs.armPosition.minus(target).getRadians();
    return Math.abs(error) <= tolerance.getRadians();
  }

  public boolean isAtTargetAngle(Rotation2d target) {
    return isAtTargetAngle(target, Rotation2d.fromDegrees(toleranceDegrees.get()));
  }

  public Voltage getVoltage() {
    return Units.Volts.of(inputs.armAppliedVolts);
  }

  public AngularVelocity getVelocity() {
    return Units.DegreesPerSecond.of(inputs.armVelocityDegreesPerSecond);
  }
}
