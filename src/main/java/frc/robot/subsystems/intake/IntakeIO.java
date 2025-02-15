package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.team2930.LoggerGroup;
import frc.robot.Constants;
import frc.robot.subsystems.BaseInputs;

public interface IntakeIO {
  /** Contains all of the input data received from hardware. */
  class Inputs extends BaseInputs {
    public double rollerVelocityRPM;
    public double rollerCurrentAmps;
    public double rollerTempCelsius;
    public double rollerAppliedVolts;
    public boolean rollerStallDetected;

    public double tofDistanceInches = 5;

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

  public default void setRollerVoltage(double volts) {}

  public default void setRollerVelocity(double revPerMin) {}

  public default void setRollerClosedLoopConstants(
      double kP, double kV, double kS, double targetAccelerationConfig) {}

  public default void setPivotVoltage(double volts) {}

  public default void resetPivotSensorPosition(Rotation2d angle) {}

  public default void setPivotClosedLoopPosition(Rotation2d angle) {}

  public default void setPivotClosedLoopConstants(
      double kP, double kD, double kG, MotionMagicConfigs mmConfigs) {}

  public default boolean setPivotNeutralMode(NeutralModeValue value) {
    return false;
  }
}
