package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants.KrakenConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.LauncherConstants;
import frc.robot.Constants.ShooterConstants.PivotConstants;

public class ShooterIOReal implements ShooterIO {

  private final TalonFX launcher = new TalonFX(Constants.CanIDs.SHOOTER_CAN_ID);
  private final TalonFX pivot = new TalonFX(Constants.CanIDs.SHOOTER_PIVOT_CAN_ID);

  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<AngularVelocity> pivotVelocity;
  private final StatusSignal<Voltage> pivotVoltage;
  private final StatusSignal<Current> pivotCurrent;
  private final StatusSignal<Temperature> pivotTemp;

  private final StatusSignal<AngularVelocity> launcherVelocity;
  private final StatusSignal<Voltage> launcherVoltage;
  private final StatusSignal<Current> launcherCurrent;
  private final StatusSignal<Temperature> launcherTemp;

  private final BaseStatusSignal[] refreshSet;

  // FIX: add FOC
  private final MotionMagicVoltage pivotClosedLoopControl =
      new MotionMagicVoltage(0).withEnableFOC(true);
  private final VoltageOut pivotOpenLoop = new VoltageOut(0.0).withEnableFOC(true);

  private final MotionMagicVelocityVoltage launcherClosedLoop =
      new MotionMagicVelocityVoltage(0.0).withEnableFOC(true);
  private final VoltageOut launcherOpenLoop = new VoltageOut(0.0).withEnableFOC(true);

  public ShooterIOReal() {
    // Launcher config
    TalonFXConfiguration launcherConfig = new TalonFXConfiguration();

    launcherConfig.CurrentLimits.SupplyCurrentLimit = LauncherConstants.SUPPLY_CURRENT_LIMIT;
    launcherConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    launcherConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    launcherConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    launcherConfig.Feedback.SensorToMechanismRatio = ShooterConstants.LauncherConstants.GEARING;

    launcherConfig.Voltage.PeakReverseVoltage = 0.0;
    launcherConfig.Voltage.SupplyVoltageTimeConstant = KrakenConstants.SUPPLY_VOLTAGE_TIME;

    launcher.getConfigurator().apply(launcherConfig);

    // Pivot config
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

    pivotConfig.CurrentLimits.SupplyCurrentLimit = PivotConstants.SUPPLY_CURRENT_LIMIT;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    pivotConfig.Feedback.SensorToMechanismRatio = PivotConstants.GEARING;
    pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        PivotConstants.MAX_ANGLE_RAD.getRotations();
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        PivotConstants.MIN_ANGLE_RAD.getRotations();

    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    pivot.getConfigurator().apply(pivotConfig);

    // Pivot status signals

    pivotPosition = pivot.getPosition();
    pivotVelocity = pivot.getVelocity();
    pivotVoltage = pivot.getMotorVoltage();
    pivotCurrent = pivot.getStatorCurrent();
    pivotTemp = pivot.getDeviceTemp();

    // Launcher status signals

    launcherVelocity = launcher.getVelocity();
    launcherVoltage = launcher.getMotorVoltage();
    launcherCurrent = launcher.getStatorCurrent();
    launcherTemp = launcher.getDeviceTemp();

    // Update status signals

    BaseStatusSignal.setUpdateFrequencyForAll(100, pivotPosition, launcherVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(50, pivotVelocity, pivotVoltage, pivotCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(10, launcherVoltage, launcherCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(1, launcherTemp, pivotTemp);

    launcher.optimizeBusUtilization();
    pivot.optimizeBusUtilization();

    refreshSet =
        new BaseStatusSignal[] {
          pivotPosition,
          pivotVelocity,
          pivotVoltage,
          pivotCurrent,
          pivotTemp,
          // --
          launcherVelocity,
          launcherVoltage,
          launcherCurrent,
          launcherTemp
        };
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.refreshAll(refreshSet);

    inputs.pivotPosition = Rotation2d.fromRotations(pivotPosition.getValue().in(Units.Rotations));
    inputs.pivotVelocityDegreesPerSec =
        pivotVelocity.getValue().in(Units.DegreesPerSecond);
    inputs.pivotAppliedVolts = pivotVoltage.getValue().in(Units.Volts);
    inputs.pivotCurrentAmps = pivotCurrent.getValue().in(Units.Amps);

    inputs.launcherRPM =
        launcherVelocity.getValue().in(Units.RPM);

    inputs.launcherAppliedVolts = launcherVoltage.getValue().in(Units.Volts);

    inputs.launcherCurrentAmps = launcherCurrent.getValue().in(Units.Amps);

    inputs.tempsCelcius[0] = launcherTemp.getValue().in(Units.Celsius);
    inputs.tempsCelcius[1] = pivotTemp.getValue().in(Units.Celsius);
  }

  // PIVOT

  @Override
  public void setPivotPosition(Rotation2d rot) {
    pivotClosedLoopControl.withPosition(rot.getRotations());
    pivot.setControl(pivotClosedLoopControl);
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivot.setControl(pivotOpenLoop.withOutput(volts));
  }

  // LAUNCHER
  @Override
  public void setLauncherVoltage(double volts) {
    launcher.setControl(launcherOpenLoop.withOutput(volts));
  }

  @Override
  public void setLauncherRPM(double rollerRPM) {
    launcher.setControl(launcherClosedLoop.withVelocity(rollerRPM / 60));
  }

  @Override
  public void resetPivotSensorPosition(Rotation2d position) {
    pivot.setPosition(position.getRotations());
  }

  @Override
  public void setPivotClosedLoopConstants(
      double kP, double kD, double kG, MotionMagicConfigs mmConfigs) {
    Slot0Configs pidConfig = new Slot0Configs();

    var configurator = pivot.getConfigurator();
    configurator.refresh(pidConfig);

    pidConfig.kP = kP;
    pidConfig.kD = kD;
    pidConfig.kG = kG;

    configurator.apply(pidConfig);
    configurator.apply(mmConfigs);
  }

  @Override
  public void setLauncherClosedLoopConstants(
      double kP, double kV, double kS, double targetAccelerationConfig) {
    Slot0Configs pidConfig = new Slot0Configs();
    MotionMagicConfigs mmConfig = new MotionMagicConfigs();

    var leadConfigurator = launcher.getConfigurator();

    leadConfigurator.refresh(pidConfig);
    leadConfigurator.refresh(mmConfig);

    pidConfig.kP = kP;
    pidConfig.kV = kV;
    pidConfig.kS = kS;

    mmConfig.MotionMagicAcceleration = targetAccelerationConfig;

    leadConfigurator.apply(pidConfig);
    leadConfigurator.apply(mmConfig);
  }

  @Override
  public boolean setNeutralMode(NeutralModeValue value) {
    var config = new MotorOutputConfigs();

    var status = pivot.getConfigurator().refresh(config);

    if (status != StatusCode.OK) return false;

    config.NeutralMode = value;

    pivot.getConfigurator().apply(config);
    return true;
  }
}
