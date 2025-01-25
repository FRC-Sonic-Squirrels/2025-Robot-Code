package frc.robot.subsystems.intakePivot;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.team2930.LoggerGroup;
import frc.robot.Constants;
import frc.robot.subsystems.BaseInputs;

public interface PivotIO {
  /** Contains all of the input data received from hardware. */
  class Inputs extends BaseInputs {
    public Rotation2d pivotPosition = Constants.zeroRotation2d;
    public double pivotAppliedVolts;
    public double pivotCurrentAmps;
    public double pivotTempCelsius;
    public double pivotVelocityDegreesPerSecond;

    public Inputs(LoggerGroup logInputs) {
      super(logInputs);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(Inputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void resetSensorPosition(Rotation2d angle) {}

  public default void setClosedLoopPosition(Rotation2d angle) {}

  public default void setClosedLoopConstants(
      double kP, double kD, double kG, MotionMagicConfigs mmConfigs) {}

  public default boolean setNeutralMode(NeutralModeValue value) {
    return false;
  }
}
