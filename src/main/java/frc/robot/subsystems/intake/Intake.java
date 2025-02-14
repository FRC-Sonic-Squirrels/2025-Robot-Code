// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.ControlMode;
import frc.lib.team2930.ExecutionTiming;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.PivotConstants;
import frc.robot.Constants.RobotMode.RobotType;

public class Intake extends SubsystemBase {
  // Execution timing
  private static final ExecutionTiming timing = new ExecutionTiming(IntakeConstants.ROOT_TABLE);

  // Logging
  private static final LoggerGroup logGroup = LoggerGroup.build(IntakeConstants.ROOT_TABLE);
  private static final LoggerEntry.Decimal logInputs_rollerVelocityRPM =
      logGroup.buildDecimal("RollerVelocityRPM");
  private static final LoggerEntry.Decimal logInputs_rollerCurrentAmps =
      logGroup.buildDecimal("RollerCurrentAmps");
  private static final LoggerEntry.Decimal logInputs_rollerTempCelsius =
      logGroup.buildDecimal("RollerTempCelsius");
  private static final LoggerEntry.Decimal logInputs_rollerAppliedVolts =
      logGroup.buildDecimal("RollerAppliedVolts");

  private static final LoggerEntry.Decimal logRollerTargetVelocityRPM =
      logGroup.buildDecimal("RollerTargetVelocityRPM");
  private static final LoggerEntry.EnumValue<ControlMode> logRollerControlMode =
      logGroup.buildEnum("RollerControlMode");

  private static final LoggerEntry.Decimal logInputs_pivotAngle =
      logGroup.buildDecimal("PivotAngle");
  private static final LoggerEntry.Decimal logInputs_pivotAppliedVolts =
      logGroup.buildDecimal("PivotAppliedVolts");
  private static final LoggerEntry.Decimal logInputs_pivotCurrentAmps =
      logGroup.buildDecimal("PivotCurrentAmps");
  private static final LoggerEntry.Decimal logInputs_pivotTempCelsius =
      logGroup.buildDecimal("PivotTempCelsius");
  private static final LoggerEntry.Decimal logInputs_pivotVelocityDegreesPerSecond =
      logGroup.buildDecimal("PivotVelocityDegreesPerSecond");
  private static final LoggerEntry.EnumValue<ControlMode> logPivotControlMode =
      logGroup.buildEnum("PivotControlMode");
  private static final LoggerEntry.Decimal logPivotTargetAngleDegrees =
      logGroup.buildDecimal("PivotTargetAngleDegrees");

  // Tunable numbers

  private static final TunableNumberGroup group =
      new TunableNumberGroup(IntakeConstants.ROOT_TABLE);

  private static final LoggedTunableNumber rKS = group.build("rKS");
  private static final LoggedTunableNumber rKP = group.build("rKP");
  private static final LoggedTunableNumber rKV = group.build("rKV");
  private static final LoggedTunableNumber rollerTargetAccelerationConfig =
      group.build("RollerMaxAccelerationConstraint");

  private static final LoggedTunableNumber pKP = group.build("pKP");
  private static final LoggedTunableNumber pKD = group.build("pKD");
  private static final LoggedTunableNumber pKG = group.build("pKG");

  private static final LoggedTunableNumber pivotMaxVelocityConfig =
      group.build("PivotMaxVelocityConfig");
  private static final LoggedTunableNumber pivotTargetAccelerationConfig =
      group.build("PivotTargetAccelerationConfig");
  private static final LoggedTunableNumber pivotToleranceDegrees =
      group.build("PivotToleranceDegrees", 1);

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2024_RETIRED_MAESTRO) {
      rKS.initDefault(0);
      rKP.initDefault(0.8);
      rKV.initDefault(0.15);
      rollerTargetAccelerationConfig.initDefault(300.0);

      pKP.initDefault(70.0);
      pKD.initDefault(1.6);
      pKG.initDefault(0.0);

      pivotMaxVelocityConfig.initDefault(10);
      pivotTargetAccelerationConfig.initDefault(10);
    } else if (Constants.RobotMode.isSimBot()) {
      rKS.initDefault(0);
      rKP.initDefault(0.0006);
      rKV.initDefault(0.0002);
      rollerTargetAccelerationConfig.initDefault(0.0);

<<<<<<< HEAD
      pKP.initDefault(10);
      pKD.initDefault(9);
=======
      pKP.initDefault(.2);
      pKD.initDefault(0);
>>>>>>> dbac697 (Added the ScoreAlgae command to the controller. Fixed the simulator so it would work.)
      pKG.initDefault(0.0);

      pivotMaxVelocityConfig.initDefault(40);
      pivotTargetAccelerationConfig.initDefault(80);
    }
  }

  private final IntakeIO io;
  private final IntakeIO.Inputs inputs = new IntakeIO.Inputs(logGroup);

  private double rollerTargetRPM;

  private ControlMode pivotControlMode = ControlMode.OPEN_LOOP;
  private Rotation2d pivotTargetAngle;

  private double pivotTargetAngleDegrees;

  private ControlMode rollerControlMode = ControlMode.OPEN_LOOP;

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;

    io.setPivotVoltage(0.0);

    setPivotConstants();

    io.setRollerVoltage(0.0);

    setRollerConstants();
  }

  @Override
  public void periodic() {
    try (var ignored = timing.start()) {
      // Logging
      io.updateInputs(inputs);
      logInputs_rollerVelocityRPM.info(inputs.rollerVelocityRPM);
      logInputs_rollerCurrentAmps.info(inputs.rollerCurrentAmps);
      logInputs_rollerTempCelsius.info(inputs.rollerTempCelsius);
      logInputs_rollerAppliedVolts.info(inputs.rollerAppliedVolts);

      logRollerControlMode.info(rollerControlMode);

      logInputs_pivotAngle.info(inputs.pivotPosition);
      logInputs_pivotAppliedVolts.info(inputs.pivotAppliedVolts);
      logInputs_pivotCurrentAmps.info(inputs.pivotCurrentAmps);
      logInputs_pivotTempCelsius.info(inputs.pivotTempCelsius);
      logInputs_pivotVelocityDegreesPerSecond.info(inputs.pivotVelocityDegreesPerSecond);

      logPivotControlMode.info(pivotControlMode);

      // Update tunable numbers

      var rhc = hashCode();
      if (rKS.hasChanged(rhc)
          || rKP.hasChanged(rhc)
          || rKV.hasChanged(rhc)
          || rollerTargetAccelerationConfig.hasChanged(rhc)) {
        setRollerConstants();
      }
      var phc = hashCode();
      if (pKP.hasChanged(phc)
          || pKD.hasChanged(phc)
          || pKG.hasChanged(phc)
          || pivotMaxVelocityConfig.hasChanged(phc)
          || pivotTargetAccelerationConfig.hasChanged(phc)) {
        setPivotConstants();
      }
    }
  }

  // Setters

  private void setRollerConstants() {
    io.setRollerClosedLoopConstants(
        rKP.get(), rKV.get(), rKS.get(), rollerTargetAccelerationConfig.get());
  }

  public void setRollerPercentOut(double percent) {
    io.setRollerVoltage(percent * Constants.MAX_VOLTAGE);
    rollerControlMode = ControlMode.OPEN_LOOP;
  }

  public void setRollerVelocity(double revPerMin) {
    io.setRollerVelocity(revPerMin);
    rollerTargetRPM = revPerMin;
    logRollerTargetVelocityRPM.info(rollerTargetRPM);
    rollerControlMode = ControlMode.CLOSED_LOOP;
  }

  private void setPivotConstants() {
    MotionMagicConfigs configs = new MotionMagicConfigs();
    configs.MotionMagicCruiseVelocity = pivotMaxVelocityConfig.get();
    configs.MotionMagicAcceleration = pivotTargetAccelerationConfig.get();
    io.setPivotClosedLoopConstants(pKP.get(), pKD.get(), pKG.get(), configs);
  }

  public void setPivotAngle(Rotation2d angle) {
    Rotation2d targetAngle =
        Rotation2d.fromRadians(
            MathUtil.clamp(
                angle.getRadians(),
                PivotConstants.MIN_PIVOT_ANGLE.getRadians(),
                PivotConstants.MAX_PIVOT_ANGLE.getRadians()));

    pivotControlMode = ControlMode.CLOSED_LOOP;
    pivotTargetAngle = targetAngle;
    pivotTargetAngleDegrees = targetAngle.getDegrees();
    io.setPivotClosedLoopPosition(targetAngle);
    logPivotTargetAngleDegrees.info(pivotTargetAngleDegrees);
  }

  public void resetPivotSubsystem() {
    pivotControlMode = ControlMode.OPEN_LOOP;
    io.setPivotVoltage(0.0);
  }

  public void setPivotVoltage(double percent) {
    pivotControlMode = ControlMode.OPEN_LOOP;
    io.setPivotVoltage(percent);
  }

  public void resetPivotSensorToHomePosition() {
    io.resetPivotSensorPosition(PivotConstants.MAX_PIVOT_ANGLE);
  }

  public boolean setPivotNeutralMode(NeutralModeValue value) {
    return io.setPivotNeutralMode(value);
  }

  // Getters

  public Current getRollerCurrentDraw() {
    return Units.Amps.of(inputs.rollerCurrentAmps);
  }

  public AngularVelocity getRollerVelocity() {
    return Units.RPM.of(inputs.rollerVelocityRPM);
  }

  public Rotation2d getPivotAngle() {
    return inputs.pivotPosition;
  }

  public boolean isPivotAtTargetAngle() {
    return isPivotAtTargetAngle(pivotTargetAngle);
  }

  public boolean isPivotAtTargetAngle(Rotation2d target, Rotation2d tolerance) {
    var error = inputs.pivotPosition.minus(target).getRadians();
    return Math.abs(error) <= tolerance.getRadians();
  }

  public boolean isPivotAtTargetAngle(Rotation2d target) {
    return isPivotAtTargetAngle(target, Rotation2d.fromDegrees(pivotToleranceDegrees.get()));
  }

  public Voltage getPivotVoltage() {
    return Units.Volts.of(inputs.pivotAppliedVolts);
  }

  public AngularVelocity getPivotVelocity() {
    return Units.DegreesPerSecond.of(inputs.pivotVelocityDegreesPerSecond);
  }

  public boolean timeOfFlight() {
    // TODO: get an actual value fot this this code is only for testing purposes
    //return inputs.tofDistanceInches <= 1;
    return true;
  }
}
