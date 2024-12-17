package frc.robot.subsystems.photon;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.VisionConstants;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Photon extends SubsystemBase { // Actual subsystem
  private PhotonIO[] cameras;
  private List<PhotonIOInputsAutoLogged> cameraInputs = new ArrayList<PhotonIOInputsAutoLogged>();
  private VisionSystemSim visionSim;
  private boolean isSim = false;

  public Photon(PhotonIO[] cams) {
    if (Constants.CURRENT_MODE == Mode.SIM) {
      visionSim = new VisionSystemSim("main");
      isSim = true;
      visionSim.addAprilTags(VisionConstants.TAG_LAYOUT);
    }
    cameras = cams;
    for (int i = 0; i < cameras.length; i++) {
      cameraInputs.add(new PhotonIOInputsAutoLogged());
      if (isSim) {
        visionSim.addCamera(cameras[i].getCameraSim(), cameras[i].getTransform());
      }
    }
  }

  @Override
  public void periodic() { // Periodic function needs to update inputs
    for (int i = 0; i < cameras.length; i++) {
      cameras[i].updateInputs(cameraInputs.get(i));
      Logger.processInputs("Photon/Camera" + Integer.toString(i), cameraInputs.get(i));
    }
  }

  public PhotonTrackedTarget getAprilTag(int tagID) {
    List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
    for (int i = 0; i < cameraInputs.size(); i++) {
      targets.addAll(cameraInputs.get(i).pipelineResult.getTargets());
    }
    PhotonTrackedTarget desiredTarget = null;
    for (PhotonTrackedTarget target : targets) {
      if (target.getFiducialId() == tagID) {
        desiredTarget = target;
      }
    }
    return desiredTarget;
  }

  public List<PoseData> getPoses() {
    List<PoseData> poses = new ArrayList<PoseData>();
    for (int i = 0; i < cameras.length; i++) {
      if (cameraInputs.get(i).connected) {
        var cInput = cameraInputs.get(i);

        if (cInput.pose != null) {
          var stddevs = getEstimationStdDevs(cInput.pose, cInput);
          poses.add(new PoseData(cInput.pose, stddevs, cInput.timestamp));
        }
        ;
      }
    }
    return poses;
  }

  public Matrix<N3, N1> getEstimationStdDevs(
      Pose2d estimatedPose, PhotonIOInputsAutoLogged inputs) {
    var estStdDevs = VisionConstants.SINGLE_TAG_STD_DEVS;
    var targets = inputs.pipelineResult.getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = VisionConstants.TAG_LAYOUT.getTagPose(tgt.getFiducialId());

      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estStdDevs;
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = VisionConstants.MULTI_TAG_STD_DEVS;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }

  public class PoseData { // Made custom class to make it simpler to pass output to Drive subsystem
    public final Pose2d pose;
    public final Matrix<N3, N1> stddevs;
    public final double timestamp;

    public PoseData(Pose2d pose, Matrix<N3, N1> stddevs, double timestamp) {
      this.pose = pose;
      this.stddevs = stddevs;
      this.timestamp = timestamp;
    }
  }

  public void updateSim(Pose2d pose) {
    if (isSim) {
      visionSim.update(pose);
    }
  }
}
