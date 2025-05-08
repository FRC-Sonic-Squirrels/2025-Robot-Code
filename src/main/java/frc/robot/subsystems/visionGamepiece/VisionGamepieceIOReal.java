package frc.robot.subsystems.visionGamepiece;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionGamepieceIOReal implements VisionGamepieceIO {

  final PhotonCamera camera;

  public VisionGamepieceIOReal() {
    camera = new PhotonCamera(Constants.VisionGamepieceConstants.CAMERA_NAME);
    camera.setDriverMode(false);
    camera.setLED(VisionLEDMode.kOff);
    camera.setPipelineIndex(0);
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.isConnected = camera.isConnected();
    inputs.pipelineIndex = camera.getPipelineIndex();

    for (PhotonPipelineResult results : camera.getAllUnreadResults()) {
      inputs.validTarget = results.hasTargets();
      List<PhotonTrackedTarget> targets = results.targets;
      inputs.pitch = new double[targets.size()];
      inputs.yaw = new double[targets.size()];
      inputs.area = new double[targets.size()];
      for (int index = 0; index < targets.size(); index++) {
        inputs.pitch[index] = targets.get(index).getPitch();
        inputs.yaw[index] = targets.get(index).getYaw();
        inputs.area[index] = targets.get(index).getArea();
      }
      inputs.targetCount = targets.size();

      var timestamp = results.getTimestampSeconds();
      var fpga = Timer.getFPGATimestamp();
      var ctre = Utils.getCurrentTimeSeconds();

      // we use CTRE time here because drivetrain odometry uses CTRE time NOT FPGA
      timestamp -= fpga;
      timestamp += ctre;

      inputs.timestamp = timestamp;

      // April tag code (for if gamepiece camera is to be used for april tag detection)

      var aprilTagYaw = 0.0;
      for (int i = 0; i < results.getTargets().size(); i++) {
        PhotonTrackedTarget target = results.targets.get(i);
        aprilTagYaw = target.getYaw();
      }

      inputs.aprilTagYaw = aprilTagYaw;
    }
  }

  @Override
  public void setPipelineIndex(int index) {
    camera.setPipelineIndex(index);
  }
}
