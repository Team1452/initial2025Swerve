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

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.TargetObservation;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class Vision extends SubsystemBase {
  // private final Drive drive;
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final Drive drive;

  public boolean alignToReef = false;
  public boolean alignToBranch = false;
  public boolean driveReady = false;

  private static final double ANGLE_KP = 100.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_KI = 0.8;
  private static final double ANGLE_MAX_VELOCITY = 8.0; // rad/s
  private static final double ANGLE_MAX_ACCELERATION = 20.0; // rad/s^2

  private static final ProfiledPIDController angleController =
      new ProfiledPIDController(
          ANGLE_KP,
          ANGLE_KI,
          ANGLE_KD,
          new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));

  public Vision(Drive drive, VisionConsumer consumer, VisionIO... io) {
    this.drive = drive;
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public TargetObservation getTargetObservation(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation;
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // TODO: Memory Usage?
    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    // Only iterate over the first two cameras, the others don't track april tags.
    for (int cameraIndex = 0; cameraIndex < 2; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // TODO: Memory Usage?
      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));

    TargetObservation targetObservation = getTargetObservation(1);
    double targetError = targetObservation.ty().getDegrees();

    // System.out.println(alignToBranch);

    if (alignToReef == true) {
      var result = VisionConstants.camera1.getLatestResult();

      if (result.hasTargets() == true) {

        PhotonTrackedTarget target = result.getBestTarget();

        if (target.getFiducialId() == 1) {
          var x = target.getDetectedCorners();
          TargetCorner corner0 = x.get(0);
          TargetCorner corner1 = x.get(1);
          TargetCorner corner2 = x.get(2);
          TargetCorner corner3 = x.get(3);

          double targetErrorYaw = Units.degreesToRadians(target.getYaw());
          double omega = angleController.calculate(targetErrorYaw, 0.0);

          double c0 = corner0.y;
          double c1 = corner1.y;
          double c2 = corner2.y;
          double c3 = corner3.y;

          System.out.println("CORNER 1: " + c0);
          System.out.println("CORNER 2: " + c0);
          System.out.println("CORNER 3: " + c0);
          System.out.println("CORNER 4: " + c0);

          if (Math.abs(c0 - c1) > 2 && Math.abs(c2 - c3) > 2) {
            if (target.getYaw() > 0) {
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      new ChassisSpeeds(1, 0.0, omega), drive.getRotation()));
            } else if (target.getYaw() < 0) {
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      new ChassisSpeeds(-1, 0.0, omega), drive.getRotation()));
            }
          } else {
            drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    new ChassisSpeeds(0, 0.0, 0), drive.getRotation()));
            alignToReef = false;
          }
        }
      }
    }

    if (alignToBranch == true) {
      var result = VisionConstants.camera1.getLatestResult();
      // System.out.println("check 1");
      if (result.hasTargets() == true) {
        // System.out.println("check 2");

        PhotonTrackedTarget target = result.getBestTarget();

        if (target.getFiducialId() == 1) {

          // System.out.println("check 3");

          double targetRange =
              PhotonUtils.calculateDistanceToTargetMeters(
                  0.254, // Measured with a tape measure, or in CAD.
                  0.22225, // From 2024 game manual for ID 7
                  Units.degreesToRadians(0), // Measured with a protractor, or in CAD.
                  Units.degreesToRadians(targetError));

          System.out.println(targetRange);

          if (Math.abs(targetRange) > 0.3) {
            driveReady = true;
          } else {
            driveReady = false;
          }

          System.out.println(driveReady);
          // BooleanSupplier condition = () -> !driveReady;

          if (driveReady == true) {
            drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    new ChassisSpeeds(0, 1, 0), drive.getRotation()));
            System.out.println("COMMAND OVER");
          } else {
            drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    new ChassisSpeeds(0, 0.0, 0), drive.getRotation()));
            alignToBranch = false;
          }
        }
      } else {
        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(0, 0.0, 0), drive.getRotation()));
        alignToBranch = false;
      }
    }
  }

  public void setAlign(boolean j) {
    alignToBranch = j;
  }

  public void setAlignToReef(boolean j) {
    alignToReef = j;
  }

  public boolean getDriveStatus() {
    return driveReady;
  }

  public boolean getAlignToBranch() {
    return alignToBranch;
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
