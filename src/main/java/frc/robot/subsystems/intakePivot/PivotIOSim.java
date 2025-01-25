package frc.robot.subsystems.intakePivot;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import edu.wpi.first.math.geometry.Rotation2d;

public class PivotIOSim implements PivotIO {

  public PivotIOSim() {}

  @Override
  public void updateInputs(Inputs inputs) {}

  @Override
  public void setVoltage(double volts) {}

  @Override
  public void setClosedLoopPosition(Rotation2d angle) {}

  @Override
  public void setClosedLoopConstants(
      double kP, double kD, double kG, MotionMagicConfigs mmConfigs) {}

  @Override
  public void resetSensorPosition(Rotation2d angle) {}
}
