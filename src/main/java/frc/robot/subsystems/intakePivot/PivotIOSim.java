package frc.robot.subsystems.intakePivot;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.team2930.TalonFXArmSim;
import frc.robot.Constants;

public class PivotIOSim implements PivotIO {

  private TalonFXArmSim pivotSim =
      new TalonFXArmSim(
          new SingleJointedArmSim(
              DCMotor.getFalcon500Foc(1),
              Constants.ArmConstants.GEAR_RATIO,
              Constants.ArmConstants.MOI,
              Constants.ArmConstants.ARM_LENGTH.in(Units.Meters),
              Constants.ArmConstants.MIN_ARM_ANGLE.getRadians(),
              Constants.ArmConstants.MAX_ARM_ANGLE.getRadians(),
              false,
              Math.PI));

  private VoltageOut openLoopControl = new VoltageOut(0);
  private MotionMagicVoltage closedLoopControl = new MotionMagicVoltage(0);

  public PivotIOSim() {}

  @Override
  public void updateInputs(Inputs inputs) {
    pivotSim.update(Constants.kDefaultPeriod);

    inputs.pivotPosition = new Rotation2d(pivotSim.getPosition().in(Units.Radian));
    inputs.pivotAppliedVolts = pivotSim.getVoltage().in(Units.Volts);
    inputs.pivotVelocityDegreesPerSecond = pivotSim.getVelocity().in(Units.DegreesPerSecond);
  }

  @Override
  public void setVoltage(double volts) {
    pivotSim.setControl(openLoopControl.withOutput(volts));
  }

  @Override
  public void setClosedLoopPosition(Rotation2d angle) {
    pivotSim.setControl(
        closedLoopControl.withPosition(Units.Degrees.of(angle.getDegrees()).in(Units.Rotations)));
  }

  @Override
  public void setClosedLoopConstants(
      double kP, double kD, double kG, MotionMagicConfigs mmConfigs) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs slot0Configs = new Slot0Configs();

    slot0Configs.kP = kP;
    slot0Configs.kD = kD;
    slot0Configs.kG = kG;

    config.Slot0 = slot0Configs;
    config.MotionMagic = mmConfigs;

    pivotSim.setConfig(config);
  }

  @Override
  public void resetSensorPosition(Rotation2d angle) {
    pivotSim.setState(angle.getRadians(), 0.0);
  }
}
