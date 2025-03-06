package frc.robot.subsystems.mechanism.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Distance;
import frc.lib.team2930.LoggerGroup;
import frc.robot.subsystems.BaseInputs;

public interface ElevatorIO {
  /** Contains all of the input data received from hardware. */
  class Inputs extends BaseInputs {
    public double heightInches;
    public double velocityInchesPerSecond;
    public double appliedVolts;
    public double currentAmps;
    public double tempCelsius;

    public Inputs(LoggerGroup logInputs) {
      super(logInputs);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(Inputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setHeight(Distance height) {}

  public default void setClosedLoopConstants(
      double kP, double kD, double kG, MotionMagicConfigs mmConfigs) {}

  public default void setSensorPosition(Distance position) {}

  public default boolean setNeutralMode(NeutralModeValue value) {
    return false;
  }
}
