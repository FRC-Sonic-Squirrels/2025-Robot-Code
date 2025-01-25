package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.team2930.TalonFXArmSim;
import frc.lib.team2930.TalonFXSim;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants.PivotConstants;

public class IntakeIOSim implements IntakeIO {

  private TalonFXSim rollerMotor =
      new TalonFXSim(
          DCMotor.getKrakenX60Foc(1),
          Constants.IntakeConstants.GEARING,
          Constants.IntakeConstants.MOI);

  private TalonFXArmSim pivotSim =
      new TalonFXArmSim(
          new SingleJointedArmSim(
              DCMotor.getFalcon500Foc(1),
              frc.robot.Constants.IntakeConstants.PivotConstants.GEAR_RATIO,
              PivotConstants.MOI,
              PivotConstants.PIVOT_LENGTH.in(Units.Meters),
              PivotConstants.MIN_PIVOT_ANGLE.getRadians(),
              PivotConstants.MAX_PIVOT_ANGLE.getRadians(),
              false,
              Math.PI));

  private VoltageOut rollerOpenLoopControl = new VoltageOut(0);
  private VelocityVoltage rollerClosedLoopControl = new VelocityVoltage(0);

  private VoltageOut pivotOpenLoopControl = new VoltageOut(0);
  private MotionMagicVoltage pivotClosedLoopControl = new MotionMagicVoltage(0);

  public IntakeIOSim() {}

  @Override
  public void updateInputs(Inputs inputs) {
    rollerMotor.update(Constants.kDefaultPeriod);
    inputs.rollerAppliedVolts = rollerMotor.getVoltage();
    inputs.rollerVelocityRPM = rollerMotor.getVelocity().in(Units.RPM);

    pivotSim.update(Constants.kDefaultPeriod);

    inputs.pivotPosition = new Rotation2d(pivotSim.getPosition().in(Units.Radian));
    inputs.pivotAppliedVolts = pivotSim.getVoltage().in(Units.Volts);
    inputs.pivotVelocityDegreesPerSecond = pivotSim.getVelocity().in(Units.DegreesPerSecond);
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerMotor.setControl(rollerOpenLoopControl.withOutput(volts));
  }

  @Override
  public void setRollerVelocity(double revPerMin) {
    rollerMotor.setControl(rollerClosedLoopControl.withVelocity(revPerMin));
  }

  @Override
  public void setRollerClosedLoopConstants(
      double kP, double kV, double kS, double maxProfiledAcceleration) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs slot0Configs = new Slot0Configs();

    slot0Configs.kP = kP;
    slot0Configs.kV = kV;
    slot0Configs.kS = kS;

    config.Slot0 = slot0Configs;

    rollerMotor.setConfig(config);
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivotSim.setControl(pivotOpenLoopControl.withOutput(volts));
  }

  @Override
  public void setPivotClosedLoopPosition(Rotation2d angle) {
    pivotSim.setControl(
        pivotClosedLoopControl.withPosition(
            Units.Degrees.of(angle.getDegrees()).in(Units.Rotations)));
  }

  @Override
  public void setPivotClosedLoopConstants(
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
  public void resetPivotSensorPosition(Rotation2d angle) {
    pivotSim.setState(angle.getRadians(), 0.0);
  }
}
