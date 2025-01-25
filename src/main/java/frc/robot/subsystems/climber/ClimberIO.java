package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.team2930.LoggerGroup;
import frc.robot.Constants;
import frc.robot.subsystems.BaseInputs;

public interface ClimberIO {
    class Inputs extends BaseInputs {
    public Rotation2d climberPosition = Constants.zeroRotation2d;
    public double armAppliedVolts;
    public double armCurrentAmps;
    public double armTempCelsius;
    public double armVelocityDegreesPerSecond;

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
