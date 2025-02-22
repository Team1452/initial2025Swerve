package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class MoveToReefRed extends Command {
  private final Drive drive;
  private final Vision vision;
  private final ProfiledPIDController angleController;

  // PID/trajectory constraints (using the same values as in DriveCommands)
  private static final double ANGLE_KP = 60.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_KI = 0.8;
  private static final double ANGLE_MAX_VELOCITY = 8.0; // rad/s
  private static final double ANGLE_MAX_ACCELERATION = 20.0; // rad/s^2

  // Deadband for vision target alignment error (radians)
  private static final double TARGET_DEADBAND = 0.05;
  private double targetRange;

  private double targetError;

  public MoveToReefRed(Drive drive, Vision vision) {
    this.drive = drive;
    this.vision = vision;

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
    vision.setAlign(true);
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
