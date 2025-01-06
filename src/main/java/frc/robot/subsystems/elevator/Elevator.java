// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.ControlMode;
import frc.lib.team2930.ExecutionTiming;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotMode.RobotType;

public class Elevator extends SubsystemBase {
  // Execution timing
  private static final ExecutionTiming timing = new ExecutionTiming(ElevatorConstants.ROOT_TABLE);

  // Logging

  private static final LoggerGroup logGroup = LoggerGroup.build(ElevatorConstants.ROOT_TABLE);
  private static final LoggerEntry.Decimal logInputs_heightInches =
      logGroup.buildDecimal("HeightInches");
  private static final LoggerEntry.Decimal logInputs_velocityInchesPerSecond =
      logGroup.buildDecimal("VelocityInchesPerSecond");
  private static final LoggerEntry.Decimal logInputs_appliedVolts =
      logGroup.buildDecimal("AppliedVolts");
  private static final LoggerEntry.Decimal logInputs_currentAmps =
      logGroup.buildDecimal("CurrentAmps");
  private static final LoggerEntry.Decimal logInputs_tempCelsius =
      logGroup.buildDecimal("TempCelsius");

  private static final LoggerEntry.Decimal logTargetHeight = logGroup.buildDecimal("targetHeight");
  private static final LoggerEntry.EnumValue<ControlMode> logControlMode =
      logGroup.buildEnum("ControlMode");

  // Tunable numbers

  private static final TunableNumberGroup group =
      new TunableNumberGroup(ElevatorConstants.ROOT_TABLE);

  private static final LoggedTunableNumber kP = group.build("kP");
  private static final LoggedTunableNumber kD = group.build("kD");
  private static final LoggedTunableNumber kG = group.build("kG");

  private static final LoggedTunableNumber maxVelocityConfig = group.build("MaxVelocityConfig");
  private static final LoggedTunableNumber targetAccelerationConfig =
      group.build("TargetAccelerationConfig");

  private final LoggedTunableNumber tolerance = group.build("toleranceInches", 0.1);

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_SIMBOT) {
      kP.initDefault(0.01);
      kD.initDefault(0.0);
      kG.initDefault(1.84255);

      maxVelocityConfig.initDefault(640.0);
      targetAccelerationConfig.initDefault(640.0);

    } else if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2024_RETIRED_MAESTRO) {
      kP.initDefault(5.0);
      kD.initDefault(0.0);
      kG.initDefault(0.153);

      maxVelocityConfig.initDefault(1000.0);
      targetAccelerationConfig.initDefault(2000.0);
    }
  }

  private final ElevatorIO io;
  private final ElevatorIO.Inputs inputs = new ElevatorIO.Inputs(logGroup);

  private Distance targetHeight = Units.Meters.zero();
  private ControlMode controlMode = ControlMode.OPEN_LOOP;

  /** Creates a new ElevatorSubsystem. */
  public Elevator(ElevatorIO io) {
    this.io = io;

    setConstants();

    io.setVoltage(0.0);
  }

  @Override
  public void periodic() {
    try (var ignored = timing.start()) {
      // Logging
      io.updateInputs(inputs);
      logInputs_heightInches.info(inputs.heightInches);
      logInputs_velocityInchesPerSecond.info(inputs.velocityInchesPerSecond);
      logInputs_appliedVolts.info(inputs.appliedVolts);
      logInputs_currentAmps.info(inputs.currentAmps);
      logInputs_tempCelsius.info(inputs.tempCelsius);

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

  public void setPercentOut(double percent) {
    io.setVoltage(percent);
    controlMode = ControlMode.OPEN_LOOP;
  }

  public void setHeight(Distance height) {
    io.setHeight(height);
    targetHeight = height;
    logTargetHeight.info(targetHeight.in(Units.Inches));
    controlMode = ControlMode.CLOSED_LOOP;
  }

  public void resetSensorToHomePosition() {
    io.setSensorPosition(Constants.ElevatorConstants.HOME_POSITION);
  }

  public boolean setNeutralMode(NeutralModeValue value) {
    return io.setNeutralMode(value);
  }

  public void setConstants() {
    MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
    mmConfigs.MotionMagicAcceleration = targetAccelerationConfig.get();
    mmConfigs.MotionMagicCruiseVelocity = maxVelocityConfig.get();
    io.setClosedLoopConstants(kP.get(), kD.get(), kG.get(), mmConfigs);
  }

  // Getters

  public boolean isAtTarget() {
    return Math.abs(targetHeight.in(Units.Inches) - inputs.heightInches) <= tolerance.get();
  }

  public boolean isAtTarget(Distance height) {
    return Math.abs(height.in(Units.Inches) - inputs.heightInches) <= tolerance.get();
  }

  public double getHeightInches() {
    return inputs.heightInches;
  }

  public double getVoltage() {
    return inputs.appliedVolts;
  }

  public double getVelocity() {
    return inputs.velocityInchesPerSecond;
  }
}
