package frc.lib.team2930;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import frc.robot.Constants;

public class AllianceFlipUtil {

  public static Translation2d flipVelocitiesForAlliance(Translation2d originalVelocity) {
    return Constants.isRedAlliance() ? originalVelocity.unaryMinus() : originalVelocity;
  }

  public static Translation2d rotateTranslation2DAroundCenterPoint(
      Translation2d originalTranslation) {
    return new Translation2d(
        Constants.FieldConstants.FIELD_LENGTH.in(Units.Meters) - originalTranslation.getX(),
        Constants.FieldConstants.FIELD_WIDTH.in(Units.Meters) - originalTranslation.getY());
  }

  public static Pose2d rotatePose2DAroundCenterPoint(Pose2d originalPose) {
    return new Pose2d(
        rotateTranslation2DAroundCenterPoint(originalPose.getTranslation()),
        originalPose.getRotation().plus(Rotation2d.k180deg));
  }

  public static Pose2d[] rotatePose2DArrayAroundCenterPoint(Pose2d[] originalPoses) {
    Pose2d[] newPoses = new Pose2d[originalPoses.length];
    for (int i = 0; i < originalPoses.length; i++) {
      newPoses[i] = flipPoseForAlliance(originalPoses[i]);
    }
    return newPoses;
  }

  public static Pose3d rotatePose3DAroundCenterPoint(Pose3d originalPose) {
    Pose2d internalPose2d = originalPose.toPose2d();
    Pose2d flippedInternalPose2d = rotatePose2DAroundCenterPoint(internalPose2d);
    return new Pose3d(
        flippedInternalPose2d.getX(),
        flippedInternalPose2d.getY(),
        originalPose.getZ(),
        new Rotation3d(
            originalPose.getRotation().getX(),
            originalPose.getRotation().getY(),
            flippedInternalPose2d.getRotation().getRadians()));
  }

  /**
   * @return blue alliance reference pose
   */
  public static Pose2d flipPoseForAlliance(Pose2d originalPose) {
    return Constants.isRedAlliance() ? rotatePose2DAroundCenterPoint(originalPose) : originalPose;
  }

  public static Pose2d[] flipPoseArrayForAlliance(Pose2d[] originalPose) {
    return Constants.isRedAlliance()
        ? rotatePose2DArrayAroundCenterPoint(originalPose)
        : originalPose;
  }

  public static Pose3d flipPoseForAlliance(Pose3d originalPose) {
    return Constants.isRedAlliance() ? rotatePose3DAroundCenterPoint(originalPose) : originalPose;
  }

  /**
   * @return blue alliance reference translation
   */
  public static Translation2d flipTranslationForAlliance(Translation2d originalTranslation) {
    return Constants.isRedAlliance()
        ? rotateTranslation2DAroundCenterPoint(originalTranslation)
        : originalTranslation;
  }
}
