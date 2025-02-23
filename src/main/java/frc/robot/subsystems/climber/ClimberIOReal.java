package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.MotorConstants.KrakenConstants;

public class ClimberIOReal implements ClimberIO {
  private final StatusSignal<Voltage> winchAppliedVoltage;
  private final StatusSignal<Angle> winchPosition;
  private final StatusSignal<Current> winchCurrent;
  private final StatusSignal<Temperature> winchTemp;
  private final StatusSignal<AngularVelocity> winchVelocity;

  private final MotionMagicVoltage winchClosedLoopControl =
      new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final VoltageOut winchOpenLoopControl = new VoltageOut(0.0).withEnableFOC(true);

  private final TalonFX winchMotor = new TalonFX(Constants.CanIDs.CLIMBER_WINCH_CAN_ID);

  private final BaseStatusSignal[] winchRefreshSet;

  private final VoltageOut grabberOpenLoopControl = new VoltageOut(0.0).withEnableFOC(true);

  private final MotionMagicVelocityVoltage grabberClosedLoopControl =
      new MotionMagicVelocityVoltage(0).withEnableFOC(true);

  private Servo climberServo = new Servo(0);

  public ClimberIOReal() {
    // Winch motor config
    TalonFXConfiguration winchConfig = new TalonFXConfiguration();

    winchConfig.CurrentLimits.SupplyCurrentLimit = ClimberConstants.SUPPLY_CURRENT_LIMIT;
    winchConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    winchConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    winchConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    winchConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    winchConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    winchConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Constants.ClimberConstants.MAX_CLIMBER_ANGLE.getRotations();
    winchConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Constants.ClimberConstants.MIN_CLIMBER_ANGLE
            .minus(Rotation2d.fromDegrees(2.0))
            .getRotations();

    winchConfig.Feedback.SensorToMechanismRatio = ClimberConstants.WINCH_GEAR_RATIO;
    winchConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    winchConfig.Voltage.SupplyVoltageTimeConstant = KrakenConstants.SUPPLY_VOLTAGE_TIME;

    winchMotor.getConfigurator().apply(winchConfig);

    // Grabber motor config
    TalonFXConfiguration grabberConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs grabberCurrentLimitConfig = new CurrentLimitsConfigs();

    grabberCurrentLimitConfig.SupplyCurrentLimit = ClimberConstants.SUPPLY_CURRENT_LIMIT;
    grabberCurrentLimitConfig.SupplyCurrentLimitEnable = true;

    grabberConfig.CurrentLimits = grabberCurrentLimitConfig;

    grabberConfig.Feedback.SensorToMechanismRatio = ClimberConstants.GRABBER_GEAR_RATIO;
    grabberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    grabberConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    grabberConfig.Voltage.SupplyVoltageTimeConstant = KrakenConstants.SUPPLY_VOLTAGE_TIME;

    // Status signals

    winchAppliedVoltage = winchMotor.getMotorVoltage();
    winchPosition = winchMotor.getPosition();
    winchCurrent = winchMotor.getStatorCurrent();
    winchTemp = winchMotor.getDeviceTemp();
    winchVelocity = winchMotor.getVelocity();

    // Update status signals

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, winchAppliedVoltage, winchPosition, winchVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(50, winchCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(1, winchTemp);

    winchMotor.optimizeBusUtilization();

    winchRefreshSet =
        new BaseStatusSignal[] {
          winchAppliedVoltage, winchPosition, winchCurrent, winchTemp, winchVelocity
        };
  }

  @Override
  public void updateWinchInputs(Inputs inputs) {
    inputs.refreshAll(winchRefreshSet);

    inputs.winchPosition = winchPosition.getValue().in(Units.Degrees);
    inputs.winchAppliedVolts = winchAppliedVoltage.getValue().in(Units.Volts);
    inputs.winchCurrentAmps = winchCurrent.getValue().in(Units.Amps);
    inputs.winchTempCelsius = winchTemp.getValue().in(Units.Celsius);
    inputs.winchVelocityDegreesPerSecond = winchVelocity.getValue().in(Units.DegreesPerSecond);
  }

  // Winch

  @Override
  public void setWinchClosedLoopPosition(Rotation2d angle) {
    winchClosedLoopControl.withPosition(angle.getRotations());
    winchMotor.setControl(winchClosedLoopControl);
  }

  @Override
  public void setWinchClosedLoopConstants(
      double kP, double kD, double kG, MotionMagicConfigs mmConfigs) {
    var slot0Configs = new Slot0Configs();

    winchMotor.getConfigurator().refresh(slot0Configs);
    winchMotor.getConfigurator().refresh(mmConfigs);

    slot0Configs.kP = kP;
    slot0Configs.kD = kD;
    slot0Configs.kG = kG;

    winchMotor.getConfigurator().apply(slot0Configs);
    winchMotor.getConfigurator().apply(mmConfigs);
  }

  @Override
  public void setWinchVoltage(double volts) {
    winchMotor.setControl(winchOpenLoopControl.withOutput(volts));
  }

  @Override
  public void resetWinchSensorPosition(Rotation2d angle) {
    winchMotor.setPosition(angle.getRotations());
  }

  @Override
  public boolean setWinchNeutralMode(NeutralModeValue value) {
    var config = new MotorOutputConfigs();

    var status = winchMotor.getConfigurator().refresh(config);

    if (status != StatusCode.OK) return false;

    config.NeutralMode = value;

    winchMotor.getConfigurator().apply(config);
    return true;
  }

  @Override
  public void setClimberServoAngle(Rotation2d angle) {
    climberServo.setAngle(angle.getDegrees());
  }
}
