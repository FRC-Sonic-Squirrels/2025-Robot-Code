package frc.robot.subsystems.visionGamepiece;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.lib.team2930.GeometryUtil;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.Gamepieces;

public class ProcessedGamepieceData {
  public Pose2d globalPose;
  public double timestamp_RIOFPGA_capture;

  public ProcessedGamepieceData(Pose2d globalPose, double timestamp_RIOFPGA_capture) {
    this.globalPose = globalPose;
    this.timestamp_RIOFPGA_capture = timestamp_RIOFPGA_capture;
  }

  /**
   * @param pose current global pose
   */
  public Pose2d getRobotCentricPose(Pose2d pose) {
    return globalPose.relativeTo(pose);
  }

  /**
   * @param pose current global pose
   */
  public Rotation2d getYaw(Pose2d pose) {
    Pose2d relativePose = getRobotCentricPose(pose);
    return Rotation2d.fromRadians(Math.atan2(relativePose.getY(), relativePose.getX()));
  }

  /**
   * @param pose current global pose
   */
  public Rotation2d getPitch(Pose2d pose) {
    double distance = getDistance(pose).in(Units.Meters);
    double height = Constants.VisionGamepieceConstants.GAMEPIECE_CAMERA_POSE.getZ();
    return Rotation2d.fromRadians(Math.atan2(height, distance));
  }

  /**
   * @param pose current global pose
   */
  public Distance getDistance(Pose2d pose) {
    return Units.Meters.of(GeometryUtil.getDist(globalPose, pose));
  }

  public Distance getDistance(Translation2d translation2d) {
    return Units.Meters.of(GeometryUtil.getDist(globalPose.getTranslation(), translation2d));
  }

  public boolean sameGamepiece(ProcessedGamepieceData gm) {
    return getDistance(gm.globalPose).lt(Gamepieces.GAMEPIECE_TOLERANCE);
  }

  public boolean isStale(double timestamp) {
    return (timestamp - timestamp_RIOFPGA_capture) > Gamepieces.GAMEPIECE_PERSISTENCE;
  }
}
