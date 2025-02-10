package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import frc.lib.team2930.TalonFXSim;
import frc.robot.Constants;

// anything commented out is arm code that we are not using yet
public class ClimberIOSim implements ClimberIO {
  private TalonFXSim climberSim =
      new TalonFXSim(
          DCMotor.getKrakenX60Foc(1),
          Constants.ClimberConstants.WINCH_GEAR_RATIO,
          Constants.ClimberConstants.WINCH_MOI);

  private TalonFXSim grabberMotor =
      new TalonFXSim(
          DCMotor.getKrakenX60Foc(1),
          Constants.ClimberConstants.GRABBER_GEAR_RATIO,
          Constants.ClimberConstants.GRABBER_MOI);

  private VoltageOut openLoopControl = new VoltageOut(0);
  private VelocityVoltage grabberClosedLoopControl = new VelocityVoltage(0);
  private MotionMagicVoltage winchClosedLoopControl = new MotionMagicVoltage(0);

  public ClimberIOSim() {}

  @Override
  public void updateWinchInputs(Inputs inputs) {
    climberSim.update(Constants.kDefaultPeriod);

    inputs.winchPosition = climberSim.getPosition().in(Units.Degree);
    inputs.winchAppliedVolts = climberSim.getVoltage();
    inputs.winchVelocityDegreesPerSecond = climberSim.getVelocity().in(Units.DegreesPerSecond);
  }

  @Override
  public void updateGrabberInputs(Inputs inputs) {
    climberSim.update(Constants.kDefaultPeriod);

    grabberMotor.update(Constants.kDefaultPeriod);
    inputs.grabberAppliedVolts = grabberMotor.getVoltage();
    inputs.grabberVelocityRPM = grabberMotor.getVelocity().in(Units.RPM);
  }

  @Override
  public void setWinchVoltage(double volts) {
    climberSim.setControl(openLoopControl.withOutput(volts));
  }

  @Override
  public void setWinchClosedLoopPosition(Rotation2d angle) {
    climberSim.setControl(winchClosedLoopControl.withPosition(angle.getRadians()));
  }

  @Override
  public void setWinchClosedLoopConstants(
      double kP, double kD, double kG, MotionMagicConfigs mmConfigs) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs slot0Configs = new Slot0Configs();

    slot0Configs.kP = kP;
    slot0Configs.kD = kD;
    slot0Configs.kG = kG;

    config.Slot0 = slot0Configs;
    config.MotionMagic = mmConfigs;

    climberSim.setConfig(config);
  }

  @Override
  public void resetWinchSensorPosition(Rotation2d angle) {
    climberSim.setState(angle.getRadians(), 0.0);
  }

  @Override
  public void setGrabberVoltage(double volts) {
    grabberMotor.setControl(openLoopControl.withOutput(volts));
  }

  @Override
  public void setGrabberVelocity(double revPerMin) {
    grabberMotor.setControl(grabberClosedLoopControl.withVelocity(revPerMin));
  }

  @Override
  public void setGrabberClosedLoopConstants(
      double kP, double kV, double kS, double maxProfiledAcceleration) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs slot0Configs = new Slot0Configs();

    slot0Configs.kP = kP;
    slot0Configs.kV = kV;
    slot0Configs.kS = kS;

    config.Slot0 = slot0Configs;

    grabberMotor.setConfig(config);
  }
}
