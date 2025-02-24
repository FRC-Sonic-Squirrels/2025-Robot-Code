package frc.robot.subsystems.climber;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.team2930.LoggerGroup;
import frc.robot.subsystems.BaseInputs;

public interface ClimberIO {
  class Inputs extends BaseInputs {
    public double winchPosition;
    public double winchAppliedVolts;
    public double winchCurrentAmps;
    public double winchTempCelsius;
    public double winchVelocityDegreesPerSecond;

    public Inputs(LoggerGroup logInputs) {
      super(logInputs);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateWinchInputs(Inputs inputs) {}

  public default void setWinchVoltage(double volts) {}

  public default void resetWinchSensorPosition(Rotation2d angle) {}

  public default void setWinchClosedLoopPosition(Rotation2d angle) {}

  public default void setClimberServoAngle(Rotation2d angle) {}

  public default void setWinchClosedLoopConstants(double kP, double kD, double kG) {}

  public default boolean setWinchNeutralMode(NeutralModeValue value) {
    return false;
  }
}
