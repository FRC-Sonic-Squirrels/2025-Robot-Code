package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants.PivotConstants;
import frc.robot.Constants.IntakeConstants.RollerConstants;
import frc.robot.Constants.MotorConstants.KrakenConstants;

public class IntakeIOReal implements IntakeIO {
  private final TalonFX rollerMotor = new TalonFX(Constants.CanIDs.INTAKE_ROLLER_CAN_ID);
  private final TalonFX starMotor = new TalonFX(Constants.CanIDs.INTAKE_STAR_CAN_ID);

  private final StatusSignal<Current> rollerCurrent;
  private final StatusSignal<Temperature> rollerDeviceTemp;
  private final StatusSignal<Voltage> rollerAppliedVoltage;
  private final StatusSignal<AngularVelocity> rollerVelocity;

  private final VoltageOut rollerOpenLoopControl = new VoltageOut(0.0).withEnableFOC(true);

  private final MotionMagicVelocityVoltage rollerClosedLoopControl =
      new MotionMagicVelocityVoltage(0).withEnableFOC(true);
  private final CANrange intakeTOF = new CANrange(Constants.CanIDs.INTAKE_TOF_CAN_ID, "CANivore");

  private final StatusSignal<Distance> intakeTofDistance;
  private final StatusSignal<Boolean> intakeTofDetected;
  private final StatusSignal<Double> intakeTofSignalStrength;

  private final StatusSignal<Voltage> pivotAppliedVoltage;
  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<Current> pivotCurrent;
  private final StatusSignal<Temperature> pivotTemp;
  private final StatusSignal<AngularVelocity> pivotVelocity;

  private final MotionMagicVoltage pivotClosedLoopControl =
      new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final VoltageOut pivotOpenLoopControl = new VoltageOut(0.0).withEnableFOC(true);

  private final TalonFX pivotMotor = new TalonFX(Constants.CanIDs.INTAKE_PIVOT_CAN_ID);

  private final BaseStatusSignal[] refreshSet;
  private final BaseStatusSignal[] refreshSetSensors;

  public IntakeIOReal() {
    // Motor config
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs rollerCurrentLimitConfig = new CurrentLimitsConfigs();

    rollerCurrentLimitConfig.SupplyCurrentLimit = RollerConstants.SUPPLY_CURRENT_LIMIT;
    rollerCurrentLimitConfig.SupplyCurrentLimitEnable = true;
    rollerCurrentLimitConfig.StatorCurrentLimit = RollerConstants.STATOR_CURRENT_LIMIT;
    rollerCurrentLimitConfig.StatorCurrentLimitEnable = true;

    rollerConfig.CurrentLimits = rollerCurrentLimitConfig;

    rollerConfig.Feedback.SensorToMechanismRatio = RollerConstants.ROLLER_GEARING;
    rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    rollerConfig.Voltage.SupplyVoltageTimeConstant = KrakenConstants.SUPPLY_VOLTAGE_TIME;

    rollerMotor.getConfigurator().apply(rollerConfig);
    starMotor.getConfigurator().apply(rollerConfig);

    // Status signals

    rollerCurrent = rollerMotor.getStatorCurrent();
    rollerDeviceTemp = rollerMotor.getDeviceTemp();
    rollerAppliedVoltage = rollerMotor.getMotorVoltage();
    rollerVelocity = rollerMotor.getVelocity();

    // Update status signals

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, rollerAppliedVoltage, rollerCurrent, rollerVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(1, rollerDeviceTemp);

    rollerMotor.optimizeBusUtilization();

    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

    pivotConfig.CurrentLimits.SupplyCurrentLimit = PivotConstants.SUPPLY_CURRENT_LIMIT;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.CurrentLimits.StatorCurrentLimit = PivotConstants.STATOR_CURRENT_LIMIT;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        PivotConstants.MAX_PIVOT_ANGLE.getRotations();
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        PivotConstants.MIN_PIVOT_ANGLE.minus(Rotation2d.fromDegrees(2.0)).getRotations();

    pivotConfig.Feedback.SensorToMechanismRatio = PivotConstants.GEAR_RATIO;
    pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    pivotConfig.Voltage.SupplyVoltageTimeConstant = KrakenConstants.SUPPLY_VOLTAGE_TIME;

    pivotMotor.getConfigurator().apply(pivotConfig);

    starMotor.setControl(new Follower(Constants.CanIDs.INTAKE_ROLLER_CAN_ID, false));

    // Status signals

    pivotAppliedVoltage = pivotMotor.getMotorVoltage();
    pivotPosition = pivotMotor.getPosition();
    pivotCurrent = pivotMotor.getStatorCurrent();
    pivotTemp = pivotMotor.getDeviceTemp();
    pivotVelocity = pivotMotor.getVelocity();

    // Update status signals

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, pivotAppliedVoltage, pivotPosition, pivotVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(50, pivotCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(1, pivotTemp);

    pivotMotor.optimizeBusUtilization();

    refreshSet =
        new BaseStatusSignal[] {
          rollerCurrent,
          rollerDeviceTemp,
          rollerAppliedVoltage,
          rollerVelocity,
          pivotCurrent,
          pivotTemp,
          pivotAppliedVoltage,
          pivotVelocity,
          pivotPosition
        };
    // Time of Flight
    CANrangeConfiguration canRangeConfigIntake = new CANrangeConfiguration();

    canRangeConfigIntake.FovParams.FOVRangeX = 6.75;
    canRangeConfigIntake.FovParams.FOVRangeY = 6.75;

    canRangeConfigIntake.ProximityParams.ProximityThreshold = Units.Inches.of(2.5).in(Units.Meters);
    canRangeConfigIntake.ProximityParams.ProximityHysteresis = 0.01;
    canRangeConfigIntake.ProximityParams.MinSignalStrengthForValidMeasurement = 30000;

    canRangeConfigIntake.ToFParams.UpdateFrequency = 100;
    canRangeConfigIntake.ToFParams.UpdateMode = UpdateModeValue.ShortRangeUserFreq;

    intakeTOF.getConfigurator().apply(canRangeConfigIntake);

    intakeTofDistance = intakeTOF.getDistance();
    intakeTofDetected = intakeTOF.getIsDetected();
    intakeTofSignalStrength = intakeTOF.getSignalStrength();

    BaseStatusSignal.setUpdateFrequencyForAll(20, intakeTofSignalStrength);

    BaseStatusSignal.setUpdateFrequencyForAll(100, intakeTofDistance, intakeTofDetected);

    intakeTOF.optimizeBusUtilization();

    refreshSetSensors =
        new BaseStatusSignal[] {
          intakeTofDistance, intakeTofDetected, intakeTofSignalStrength,
        };
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.refreshAll(refreshSet, refreshSetSensors);

    inputs.rollerCurrentAmps = rollerCurrent.getValue().in(Units.Amps);
    inputs.rollerTempCelsius = rollerDeviceTemp.getValue().in(Units.Celsius);
    inputs.rollerAppliedVolts = rollerAppliedVoltage.getValue().in(Units.Volts);
    inputs.rollerVelocityRPM = rollerVelocity.getValue().in(Units.RPM);

    inputs.pivotPosition = Rotation2d.fromRotations(pivotPosition.getValue().in(Units.Rotations));
    inputs.pivotAppliedVolts = pivotAppliedVoltage.getValue().in(Units.Volts);
    inputs.pivotCurrentAmps = pivotCurrent.getValue().in(Units.Amps);
    inputs.pivotTempCelsius = pivotTemp.getValue().in(Units.Celsius);
    inputs.pivotVelocityDegreesPerSecond = pivotVelocity.getValue().in(Units.DegreesPerSecond);

    inputs.intakeTofDetected = intakeTofDetected.getValue();
    inputs.intakeTofDistanceInches = intakeTofDistance.getValue().in(Units.Inches);
    inputs.intakeTofSignalStrength = intakeTofSignalStrength.getValue();
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerMotor.setControl(rollerOpenLoopControl.withOutput(volts));
  }

  @Override
  public void setRollerVelocity(double revPerMin) {
    rollerMotor.setControl(
        rollerClosedLoopControl.withVelocity(Units.RPM.of(revPerMin).in(Units.RotationsPerSecond)));
  }

  @Override
  public void setRollerClosedLoopConstants(
      double rKP, double rKV, double rKS, double targetAccelerationConfig) {
    Slot0Configs pidConfig = new Slot0Configs();
    MotionMagicConfigs mmConfig = new MotionMagicConfigs();

    var config = rollerMotor.getConfigurator();

    config.refresh(pidConfig);
    config.refresh(mmConfig);

    pidConfig.kP = rKP;
    pidConfig.kV = rKV;
    pidConfig.kS = rKS;

    mmConfig.MotionMagicAcceleration = targetAccelerationConfig;

    config.apply(pidConfig);
    config.apply(mmConfig);
  }

  @Override
  public void setPivotClosedLoopPosition(Rotation2d angle) {
    pivotClosedLoopControl.withPosition(angle.getRotations());
    pivotMotor.setControl(pivotClosedLoopControl);
  }

  @Override
  public void setPivotClosedLoopConstants(
      double kP, double kD, double kG, MotionMagicConfigs mmConfigs) {
    var slot0Configs = new Slot0Configs();

    pivotMotor.getConfigurator().refresh(slot0Configs);

    slot0Configs.kP = kP;
    slot0Configs.kD = kD;
    slot0Configs.kG = kG;

    pivotMotor.getConfigurator().apply(slot0Configs);
    pivotMotor.getConfigurator().apply(mmConfigs);
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivotMotor.setControl(pivotOpenLoopControl.withOutput(volts));
  }

  @Override
  public void resetPivotSensorPosition(Rotation2d angle) {
    pivotMotor.setPosition(angle.getRotations());
  }

  @Override
  public boolean setPivotNeutralMode(NeutralModeValue value) {
    var config = new MotorOutputConfigs();

    var status = pivotMotor.getConfigurator().refresh(config);

    if (status != StatusCode.OK) return false;

    config.NeutralMode = value;

    pivotMotor.getConfigurator().apply(config);
    return true;
  }
}
