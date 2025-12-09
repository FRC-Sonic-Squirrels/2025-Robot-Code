// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector;

/**
 * Represents the output command for the end effector motor. This separates the decision of "what to
 * do" from the action of "doing it", making the state machine logic easier to test and understand.
 */
public record MotorOutput(Type type, double value) {

  /** The type of motor command. */
  public enum Type {
    /** Run at a specific velocity (RPM). */
    VELOCITY,
    /** Run at a percentage of max voltage (-1 to 1). */
    PERCENT,
    /** Stop the motor (coast or brake depending on config). */
    STOP
  }

  /** Create a velocity command. */
  public static MotorOutput velocity(double rpm) {
    return new MotorOutput(Type.VELOCITY, rpm);
  }

  /** Create a percent output command. */
  public static MotorOutput percent(double percent) {
    return new MotorOutput(Type.PERCENT, percent);
  }

  /** Create a stop command. */
  public static MotorOutput stop() {
    return new MotorOutput(Type.STOP, 0);
  }

  /** Check if this is a stop command. */
  public boolean isStopped() {
    return type == Type.STOP || (type == Type.PERCENT && value == 0);
  }
}
