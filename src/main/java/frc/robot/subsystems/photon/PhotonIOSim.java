// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.photon;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;

public class PhotonIOSim implements PhotonIO {
  private PhotonCamera cam;
  private PhotonCameraSim simCam;
  private PhotonPoseEstimator estimator;
  private Transform3d transform;
  public PhotonIOSim(String camName, Transform3d transform) { // Implentation of CameraIO
    cam = new PhotonCamera(camName);
    this.transform = transform;
    simCam = new PhotonCameraSim(cam);
    estimator =
        new PhotonPoseEstimator(
            VisionConstants.TAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, transform);
  }

  @Override
  public void updateInputs(PhotonIOInputs inputs) { // You need to make functions to set values
    inputs.connected = cam.isConnected();
    inputs.pipelineResult = cam.getLatestResult();
    inputs.latency = inputs.pipelineResult.getLatencyMillis();
    var est = estimator.update();
    if (est.isPresent()) {
      inputs.pose = est.get().estimatedPose.toPose2d();
      inputs.timestamp = est.get().timestampSeconds;
    } else {
      inputs.pose = null;
      inputs.timestamp = 0;
    }
  }

  public PhotonCameraSim getCameraSim() {
    return simCam;
  }

  public Transform3d getTransform() {
    return transform;
  }
}
