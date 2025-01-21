package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MotorConstants.KrakenConstants;

public class ElevatorIOReal implements ElevatorIO {

  private final TalonFX motor = new TalonFX(Constants.CanIDs.ELEVATOR_CAN_ID);

  private final MotionMagicVoltage closedLoopControl =
      new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final VoltageOut openLoopControl = new VoltageOut(0.0).withEnableFOC(true);

  private StatusSignal<Angle> rotorPosition;
  private StatusSignal<AngularVelocity> rotorVelocity;
  private StatusSignal<Voltage> appliedVoltage;
  private StatusSignal<Current> current;
  private StatusSignal<Temperature> temp;

  private final BaseStatusSignal[] refreshSet;

  public ElevatorIOReal() {
    // Motor config
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ElevatorConstants.MAX_HEIGHT.in(Units.Inches) * ElevatorConstants.INCHES_TO_MOTOR_ROT;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    config.Voltage.SupplyVoltageTimeConstant = KrakenConstants.SUPPLY_VOLTAGE_TIME;

    motor.getConfigurator().apply(config);

    // Status signals

    rotorPosition = motor.getRotorPosition();
    rotorVelocity = motor.getRotorVelocity();
    appliedVoltage = motor.getMotorVoltage();
    current = motor.getStatorCurrent();
    temp = motor.getDeviceTemp();

    // Update status signals

    BaseStatusSignal.setUpdateFrequencyForAll(100, rotorPosition, rotorVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(50, appliedVoltage, current);
    BaseStatusSignal.setUpdateFrequencyForAll(1, temp);

    motor.optimizeBusUtilization();

    refreshSet =
        new BaseStatusSignal[] {rotorPosition, rotorVelocity, appliedVoltage, current, temp};
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.refreshAll(refreshSet);

    inputs.heightInches =
        rotorPosition.getValue().in(Units.Rotations) / ElevatorConstants.INCHES_TO_MOTOR_ROT;
    inputs.velocityInchesPerSecond =
        rotorVelocity.getValue().in(Units.RPM) / ElevatorConstants.INCHES_TO_MOTOR_ROT;
    inputs.appliedVolts = appliedVoltage.getValue().in(Units.Volts);
    inputs.currentAmps = current.getValue().in(Units.Amps);
    inputs.tempCelsius = temp.getValue().in(Units.Celsius);
  }

  @Override
  public void setVoltage(double volts) {
    openLoopControl.withOutput(volts);
    motor.setControl(openLoopControl);
  }

  @Override
  public void setHeight(Distance height) {
    closedLoopControl.withPosition(height.in(Units.Inches) * ElevatorConstants.INCHES_TO_MOTOR_ROT);
    motor.setControl(closedLoopControl);
  }

  @Override
  public void setSensorPosition(Distance position) {
    motor.setPosition(position.in(Units.Inches) * ElevatorConstants.INCHES_TO_MOTOR_ROT);
  }

  @Override
  public void setClosedLoopConstants(
      double kP, double kD, double kG, MotionMagicConfigs mmConfigs) {
    Slot0Configs pidConfig = new Slot0Configs();

    motor.getConfigurator().refresh(pidConfig);

    pidConfig.kP = kP;
    pidConfig.kD = kD;
    pidConfig.kG = kG;

    motor.getConfigurator().apply(pidConfig);
    motor.getConfigurator().apply(mmConfigs);
  }

  @Override
  public boolean setNeutralMode(NeutralModeValue value) {
    var config = new MotorOutputConfigs();

    var status = motor.getConfigurator().refresh(config);

    if (status != StatusCode.OK) return false;

    config.NeutralMode = value;

    motor.getConfigurator().apply(config);
    return true;
  }
}
