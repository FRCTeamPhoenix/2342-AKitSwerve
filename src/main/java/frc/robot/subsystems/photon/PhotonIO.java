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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;

// import java.util.List;

public interface PhotonIO { // IO is the most basic part of a subsystem, just values
  @AutoLog
  public static class PhotonIOInputs {
    public boolean connected = false; // Akit logs this stuff automatically
    public PhotonPipelineResult pipelineResult = new PhotonPipelineResult();
    public double latency = 0;
    public double timestamp = 0;
    public Pose2d pose =
        null; // Make sure the datatype isnt complex, like an optional or a list, because Akit can't
    // log those and it will crash
  }

  public default void updateInputs(PhotonIOInputs inputs) {}

  public default PhotonCameraSim getCameraSim() {return null;}

  public default Transform3d getTransform() {return null;}
}
