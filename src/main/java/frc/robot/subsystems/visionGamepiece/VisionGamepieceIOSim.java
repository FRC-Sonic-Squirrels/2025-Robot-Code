package frc.robot.subsystems.visionGamepiece;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import frc.robot.Constants;
import frc.robot.Constants.VisionGamepieceConstants;
import frc.robot.configs.RobotConfig;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionGamepieceIOSim implements VisionGamepieceIO {

  private final PhotonCamera camera;
  private final PhotonCameraSim cameraSim;
  private final VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;

  /** Designed for simulated april tag detection of gamepiece camera */
  public VisionGamepieceIOSim(RobotConfig config, Supplier<Pose2d> poseSupplier) {

    this.poseSupplier = poseSupplier;
    var cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(
        VisionGamepieceConstants.RESOLUTION_WIDTH_PIXELS,
        VisionGamepieceConstants.RESOLUTION_HEIGHT_PIXELS,
        VisionGamepieceConstants.FOV_DIAGONAL);
    cameraProp.setCalibError(
        VisionGamepieceConstants.AVERAGE_ERROR_PIXELS,
        VisionGamepieceConstants.AVERAGE_STANDARD_DEVIATION_PIXELS);
    cameraProp.setFPS(VisionGamepieceConstants.FPS);
    cameraProp.setAvgLatencyMs(VisionGamepieceConstants.AVERAGE_LATENCY_MS);
    cameraProp.setLatencyStdDevMs(VisionGamepieceConstants.AVERAGE_STANDARD_DEVIATION_MS);

    var networkName = "gamepieceCameraSim";
    this.camera = new PhotonCamera(networkName);
    this.visionSim = new VisionSystemSim(networkName);
    visionSim.addAprilTags(config.getAprilTagFieldLayout());
    this.cameraSim = new PhotonCameraSim(camera, cameraProp);

    var asTransform = Constants.VisionGamepieceConstants.GAMEPIECE_CAMERA_POSE.minus(new Pose3d());
    var robotToCamera =
        asTransform.plus(
            new Transform3d(0, 0, Units.Inch.of(20).in(Units.Meter), new Rotation3d()));

    visionSim.addCamera(cameraSim, robotToCamera);

    cameraSim.enableDrawWireframe(true);
    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);
  }

  @Override
  public void updateInputs(Inputs inputs) {
    visionSim.update(this.poseSupplier.get());

    for (var results : camera.getAllUnreadResults()) {
      var aprilTagYaw = 0.0;
      for (int i = 0; i < results.getTargets().size(); i++) {
        PhotonTrackedTarget target = results.targets.get(i);
        aprilTagYaw = target.getYaw();
      }

      inputs.aprilTagYaw = aprilTagYaw;
    }
  }
}
