package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.MotorConstants.KrakenConstants;

public class EndEffectorIOReal implements EndEffectorIO {
  private TalonFX motor = new TalonFX(Constants.CanIDs.END_EFFECTOR_CAN_ID);

  private final StatusSignal<Current> current;
  private final StatusSignal<Temperature> deviceTemp;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<AngularVelocity> velocity;

  private final VoltageOut openLoopControl = new VoltageOut(0.0).withEnableFOC(true);

  private final MotionMagicVelocityVoltage closedLoopControl =
      new MotionMagicVelocityVoltage(0).withEnableFOC(true);

  private final BaseStatusSignal[] refreshSet;

  private final CANrange scoringSideEndEffectorTOF =
      new CANrange(Constants.CanIDs.END_EFFECTOR_SCORING_SIDE_TOF_CAN_ID, "CANivore");

  private final CANrange nonScoringSideEndEffectorTOF =
      new CANrange(Constants.CanIDs.END_EFFECTOR_NON_SCORING_SIDE_TOF_CAN_ID, "CANivore");

  private final StatusSignal<Distance> scoringSideTofDistance;
  private final StatusSignal<Distance> nonScoringSideTofDistance;
  private final StatusSignal<Boolean> scoringSideTofDetected;
  private final StatusSignal<Boolean> nonScoringSideTofDetected;

  public EndEffectorIOReal() {
    // Motor config
    TalonFXConfiguration config = new TalonFXConfiguration();
    CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();

    currentLimitConfig.SupplyCurrentLimit = EndEffectorConstants.SUPPLY_CURRENT_LIMIT;
    currentLimitConfig.SupplyCurrentLimitEnable = true;

    config.CurrentLimits = currentLimitConfig;

    config.Feedback.SensorToMechanismRatio = EndEffectorConstants.GEARING;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.Voltage.SupplyVoltageTimeConstant = KrakenConstants.SUPPLY_VOLTAGE_TIME;

    motor.getConfigurator().apply(config);

    // Status signals

    current = motor.getStatorCurrent();
    deviceTemp = motor.getDeviceTemp();
    appliedVoltage = motor.getMotorVoltage();
    velocity = motor.getVelocity();

    // Update status signals

    BaseStatusSignal.setUpdateFrequencyForAll(50, appliedVoltage, current, velocity);
    BaseStatusSignal.setUpdateFrequencyForAll(1, deviceTemp);

    motor.optimizeBusUtilization();

    // Time of Flight

    CANrangeConfiguration canRangeConfig = new CANrangeConfiguration();

    canRangeConfig.FovParams.FOVRangeX = 6.75;
    canRangeConfig.FovParams.FOVRangeY = 6.75;

    canRangeConfig.ProximityParams.ProximityThreshold = 0.4;
    canRangeConfig.ProximityParams.ProximityHysteresis = 0.01;
    canRangeConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 2500;

    canRangeConfig.ToFParams.UpdateFrequency = 100;
    canRangeConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRangeUserFreq;

    scoringSideEndEffectorTOF.getConfigurator().apply(canRangeConfig);
    nonScoringSideEndEffectorTOF.getConfigurator().apply(canRangeConfig);

    scoringSideTofDistance = scoringSideEndEffectorTOF.getDistance();
    nonScoringSideTofDistance = nonScoringSideEndEffectorTOF.getDistance();
    scoringSideTofDetected = scoringSideEndEffectorTOF.getIsDetected();
    nonScoringSideTofDetected = nonScoringSideEndEffectorTOF.getIsDetected();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        scoringSideTofDistance,
        nonScoringSideTofDistance,
        scoringSideTofDetected,
        nonScoringSideTofDetected);

    scoringSideEndEffectorTOF.optimizeBusUtilization();
    nonScoringSideEndEffectorTOF.optimizeBusUtilization();

    refreshSet =
        new BaseStatusSignal[] {
          current,
          deviceTemp,
          appliedVoltage,
          velocity,
          scoringSideTofDistance,
          nonScoringSideTofDistance,
          scoringSideTofDetected,
          nonScoringSideTofDetected
        };
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.refreshAll(refreshSet);

    inputs.currentAmps = current.getValue().in(Units.Amps);
    inputs.tempCelsius = deviceTemp.getValue().in(Units.Celsius);
    inputs.appliedVolts = appliedVoltage.getValue().in(Units.Volts);
    inputs.velocityRPM = velocity.getValue().in(Units.RPM);
    inputs.scoringSideTofDistInches = scoringSideTofDistance.getValue().in(Units.Inches);
    inputs.nonScoringSideTofDistInches = nonScoringSideTofDistance.getValue().in(Units.Inches);
    inputs.scoringSideTofDetecting = scoringSideTofDetected.getValue().booleanValue();
    inputs.nonScoringSideTofDetecting = nonScoringSideTofDetected.getValue().booleanValue();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(openLoopControl.withOutput(volts));
  }

  @Override
  public void setVelocity(double revPerMin) {
    motor.setControl(
        closedLoopControl.withVelocity(Units.RPM.of(revPerMin).in(Units.RotationsPerSecond)));
  }

  @Override
  public void setClosedLoopConstants(
      double kP, double kV, double kS, double targetAccelerationConfig) {
    Slot0Configs pidConfig = new Slot0Configs();
    MotionMagicConfigs mmConfig = new MotionMagicConfigs();

    var config = motor.getConfigurator();

    config.refresh(pidConfig);

    pidConfig.kP = kP;
    pidConfig.kV = kV;
    pidConfig.kS = kS;

    mmConfig.MotionMagicAcceleration = targetAccelerationConfig;

    config.apply(pidConfig);
    config.apply(mmConfig);
  }
}
