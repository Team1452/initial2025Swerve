// Copyright 2021-2025 FRC 6328
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

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import org.photonvision.PhotonCamera;

/** IO implementation for PhotonVision hardware that provides only target observations. */
public class VisionIOTargetOnly implements VisionIO {
  private final PhotonCamera camera;

  /**
   * Creates a new VisionIOTargetOnly.
   *
   * @param name The configured name of the camera.
   */
  public VisionIOTargetOnly(String name) {
    camera = new PhotonCamera(name);
    
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    
    inputs.connected = camera.isConnected();
    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
      }
    }
  }
}
