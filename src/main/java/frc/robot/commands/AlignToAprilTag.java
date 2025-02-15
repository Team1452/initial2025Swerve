package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AlignToAprilTag extends Command {
  private final Drive drive;
  private final Vision vision;
  private final ProfiledPIDController angleController;

  // PID/trajectory constraints (using the same values as in DriveCommands)
  private static final double ANGLE_KP = 100.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_KI = 0.8;
  private static final double ANGLE_MAX_VELOCITY = 8.0; // rad/s
  private static final double ANGLE_MAX_ACCELERATION = 20.0; // rad/s^2

  // Deadband for vision target alignment error (radians)
  private static final double TARGET_DEADBAND = 0.05;

  private final int cameraIndex;

  public AlignToAprilTag(Drive drive, Vision vision, int cameraIndex) {
    this.drive = drive;
    this.vision = vision;
    this.cameraIndex = cameraIndex;

    this.angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            ANGLE_KI,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    // Enable continuous input around the circle if you are using absolute angle measurements.
    // Here, the vision provided error is a relative offset and typically small, so this is
    // optional.
    this.angleController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Reset the controller error when the command starts.
    angleController.reset(0.0);
  }

  public void execute() {
    var result = VisionConstants.camera1.getLatestResult();
    if (result.hasTargets() == true) {
      PhotonTrackedTarget target = result.getBestTarget();
      if (target.getFiducialId() == 1) {

        double targetError = Units.degreesToRadians(target.getYaw());

        // Assume getXAngle() returns a Rotation2d; convert the horizontal angle offset to radians.
        // Rotation2d targetError = new Rotation2d(targetYaw);

        // Apply a deadband so that very small alignment errors do not generate corrective commands.
        if (Math.abs(targetError) < TARGET_DEADBAND) {
          targetError = 0.0;
        }

        // Calculate the required rotational speed (omega) using the profiled PID controller.
        // We want to reduce the error to 0 so the setpoint is 0.
        double omega = angleController.calculate(targetError, 0.0);

        // Command the drive with zero linear speed and the calculated angular speed.
        // We use field-relative speeds even for a pure rotation.
        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(0.0, 0.0, omega), drive.getRotation()));
      }
    }
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
