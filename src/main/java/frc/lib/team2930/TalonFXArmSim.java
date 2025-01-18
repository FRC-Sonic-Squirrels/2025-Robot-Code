package frc.lib.team2930;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.team2930.TalonFXSim.ControlMode;
import frc.robot.Constants;

public class TalonFXArmSim {

  private final SingleJointedArmSim sim;

  private final ProfiledPIDController feedback =
      new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));

  private ControlMode control = ControlMode.VOLTAGE;

  private double targetVoltage;
  private Angle targetPosition;

  private double output;

  private double kG;

  public TalonFXArmSim(SingleJointedArmSim coreSim) {
    sim = coreSim;
  }

  public void update(double dtSeconds) {
    sim.update(dtSeconds);

    if (control == ControlMode.VOLTAGE) {
      output = targetVoltage;
    } else {
      output =
          feedback.calculate(sim.getAngleRads(), targetPosition.in(Units.Radians))
              + kG * Math.cos(sim.getAngleRads());
    }

    output = MathUtil.clamp(output, -Constants.MAX_VOLTAGE, Constants.MAX_VOLTAGE);

    sim.setInputVoltage(output);

    if (output == 0.0) sim.setState(sim.getAngleRads(), sim.getVelocityRadPerSec() / 2.0);
  }

  public void setControl(VoltageOut request) {
    targetVoltage = request.Output;
    control = ControlMode.VOLTAGE;
  }

  /**
   * @param request position should be in Rotations
   */
  public void setControl(MotionMagicVoltage request) {
    targetPosition = Units.Rotations.of(request.Position);
    control = ControlMode.POSITION;
  }

  public Voltage getVoltage() {
    return Units.Volts.of(output);
  }

  public AngularVelocity getVelocity() {
    return Units.RadiansPerSecond.of(sim.getVelocityRadPerSec());
  }

  public Angle getPosition() {
    return Units.Radians.of(sim.getAngleRads());
  }

  public void setConfig(TalonFXConfiguration config) {
    Slot0Configs slot0Configs = config.Slot0;
    MotionMagicConfigs mmConfigs = config.MotionMagic;

    kG = slot0Configs.kG;

    feedback.setPID(slot0Configs.kP, slot0Configs.kI, slot0Configs.kD);
    feedback.setConstraints(
        new Constraints(mmConfigs.MotionMagicCruiseVelocity, mmConfigs.MotionMagicAcceleration));
  }

  public void setState(double angleRadians, double velocityRadPerSec) {
    sim.setState(angleRadians, velocityRadPerSec);
  }
}
