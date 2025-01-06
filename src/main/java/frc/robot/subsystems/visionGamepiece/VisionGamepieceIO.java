package frc.robot.subsystems.visionGamepiece;

public interface VisionGamepieceIO {

  public static class Inputs {
    public boolean isConnected;
    public boolean validTarget;
    public double timestamp;
    public double[] pitch = new double[] {};
    public double[] yaw = new double[] {};
    public double[] area = new double[] {};
    public int targetCount;
    public double aprilTagYaw;
    public int pipelineIndex;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(Inputs inputs) {}

  public default void setPipelineIndex(int index) {}
}
