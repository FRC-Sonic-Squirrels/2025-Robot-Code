package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import frc.lib.team2930.TalonFXSim;
import frc.robot.Constants;

public class EndEffectorIOSim implements EndEffectorIO {

  private TalonFXSim motor =
      new TalonFXSim(
          DCMotor.getKrakenX60Foc(1),
          Constants.EndEffectorConstants.GEARING,
          Constants.EndEffectorConstants.MOI);

  private VoltageOut openLoopControl = new VoltageOut(0);
  private VelocityVoltage closedLoopControl = new VelocityVoltage(0);

  public EndEffectorIOSim() {}

  @Override
  public void updateInputs(Inputs inputs) {
    motor.update(Constants.kDefaultPeriod);
    inputs.appliedVolts = motor.getVoltage();
    inputs.velocityRPM = motor.getVelocity().in(Units.RPM);
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(openLoopControl.withOutput(volts));
  }

  @Override
  public void setVelocity(double revPerMin) {
    motor.setControl(closedLoopControl.withVelocity(revPerMin));
  }

  @Override
  public void setClosedLoopConstants(
      double kP, double kV, double kS, double maxProfiledAcceleration) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs slot0Configs = new Slot0Configs();

    slot0Configs.kP = kP;
    slot0Configs.kV = kV;
    slot0Configs.kS = kS;

    config.Slot0 = slot0Configs;

    motor.setConfig(config);
  }
}
