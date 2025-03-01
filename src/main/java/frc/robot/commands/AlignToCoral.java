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

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Command that automatically rotates the robot to align with the coral target.
 *
 * <p>This command uses the vision subsystem (assumed to provide a TargetObservation from the
 * coral-dedicated camera) and a profiled PID controller (with similar parameters to the ones from
 * {@link DriveCommands}) to adjust the robot's heading until the target's horizontal offset is near
 * zero.
 *
 * <p>It is intended to be scheduled while a button is held (button logic is handled in {@link
 * frc.robot.RobotContainer}).
 *
 * <p>NOTE: This implementation assumes that the {@code TargetObservation} (retrieved from {@link
 * Vision#getTargetObservation(int)}) provides methods {@code isValid()} and {@code getXAngle()}
 * which return whether the target is valid and the horizontal error (in radians) from center,
 * respectively.
 */
public class AlignToCoral extends Command {
  private final Drive drive;
  private final Vision vision;
  private final ProfiledPIDController angleController;

  // PID/trajectory constraints (using the same values as in DriveCommands)
  private static final double ANGLE_KP = 35.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0; // rad/s
  private static final double ANGLE_MAX_ACCELERATION = 20.0; // rad/s^2

  // Deadband for vision target alignment error (radians)
  private static final double TARGET_DEADBAND = 0.05;

  // The coral target is assumed to be provided by camera index 2.
  private final int cameraIndex;
  private DoubleSupplier xSupplier;
  private DoubleSupplier ySupplier;
  private boolean aligned;

  public double omega;

  /**
   * Constructs a new AlignToCoral command.
   *
   * @param drive The drive subsystem.
   * @param vision The vision subsystem.
   * @param cameraIndex The index of the vision camera used to track coral.
   * @param forwardSupplier A DoubleSupplier for forward drive input (usually from a joystick, range
   *     -1.0 to 1.0).
   */
  public AlignToCoral(Drive drive, Vision vision, int cameraIndex, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    this.drive = drive;
    this.vision = vision;
    this.cameraIndex = cameraIndex;
    this.angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.3,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    // Enable continuous input around the circle if you are using absolute angle measurements.
    // Here, the vision provided error is a relative offset and typically small, so this is
    // optional.
    this.angleController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
  }

  @Override
  public void initialize() {
    // Reset the controller error when the command starts.
    angleController.reset(0.0);
  }

  @Override
  public void execute() {
    var result = VisionConstants.camera2.getLatestResult();
    if (result.hasTargets() == true) {
      PhotonTrackedTarget target = result.getBestTarget();
      if (target.getDetectedObjectClassID() == 1) {
        var targetObservation = vision.getTargetObservation(cameraIndex);
        double targetError = 0.0;

        // Assume getXAngle() returns a Rotation2d; convert the horizontal angle offset to radians.
        targetError = targetObservation.tx().getRadians();

        // Apply a deadband so that very small alignment errors do not generate corrective commands.
        if (Math.abs(targetError) < TARGET_DEADBAND) {
          targetError = 0.0;
        }

        // Calculate the required rotational speed (omega) using the profiled PID controller.
        // We want to reduce the error to 0 so the setpoint is 0.
        omega = angleController.calculate(targetError, 0.0);

        Supplier<Rotation2d> rotationSupplier =
        () -> drive.getRotation().minus(new Rotation2d(omega));


        // Command the drive with zero linear speed and the calculated angular speed.
        // We use field-relative speeds even for a pure rotation.
        DriveCommands.joystickDriveAtAngle(drive, xSupplier, ySupplier, rotationSupplier);

      }
    }
  }

  public double getOmega() {
    return omega;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop any drive commands when the command ends.
    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  @Override
  public boolean isFinished() {
    // This command is intended to be held (i.e. run while a button is pressed), so it never
    // finishes on its own.
    return false;
  }
}
