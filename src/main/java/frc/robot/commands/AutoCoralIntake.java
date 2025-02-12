package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.Vision;
import java.util.function.DoubleSupplier;

public class AutoCoralIntake extends SequentialCommandGroup {
  // Gain constant for rotational alignment; adjust by testing.
  private static final double ALIGN_KP = 2.0;
  // Threshold (in degrees) for the vision target pitch at which we consider the coral "in range."
  // (This value will need to be tuned based on your mounting and target profile.)
  private static final double IN_RANGE_PITCH_THRESHOLD_DEGREES = 10.0;

  /**
   * Constructs an AutoCoralIntake command.
   *
   * @param drive The drive subsystem.
   * @param vision The vision subsystem (which uses the target-only IO for camera index 2).
   * @param intake The intake subsystem.
   * @param forwardSupplier A DoubleSupplier for forward (y-axis) drive input.
   */
  public AutoCoralIntake(
      Drive drive, Vision vision, Intake intake, DoubleSupplier forwardSupplier) {
    addCommands(
        // Step 1: Pre-align the robot using camera 2.
        new ParallelRaceGroup(
            // Continually adjust rotation based on target x error.
            new RunCommand(
                () -> {
                  var target = vision.getTargetObservation(2);
                  double yawErrorRadians = target.tx().getRadians();
                  double rotationCorrection = ALIGN_KP * yawErrorRadians;
                  // Since we're not commanding any linear motion here, send zero forward speed.
                  drive.runVelocity(
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          0.0, 0.0, rotationCorrection, drive.getRotation()));
                },
                drive),
            // Terminate this phase once the yaw error is small.
            new WaitUntilCommand(
                () -> {
                  var target = vision.getTargetObservation(2);
                  return Math.abs(target.tx().getDegrees()) < 2.0;
                })),
        // Step 2: Drive forward (using joystick input) while maintaining alignment.
        // This runs until the vision "pitch" indicates that the coral is in range.
        new ParallelRaceGroup(
            new RunCommand(
                () -> {
                  double forwardSpeed =
                      forwardSupplier.getAsDouble() * drive.getMaxLinearSpeedMetersPerSec();
                  var target = vision.getTargetObservation(2);
                  double yawErrorRadians = target.tx().getRadians();
                  double rotationCorrection = ALIGN_KP * yawErrorRadians;
                  drive.runVelocity(
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          forwardSpeed, 0.0, rotationCorrection, drive.getRotation()));
                },
                drive),
            new WaitUntilCommand(
                () -> {
                  // Use the target pitch as a proxy for distance. Adjust the logic as needed.
                  // REPLACE WITH CANRANGE DETECTION
                  var target = vision.getTargetObservation(2);
                  double pitchDegrees = target.ty().getDegrees();
                  return Math.abs(pitchDegrees) > IN_RANGE_PITCH_THRESHOLD_DEGREES;
                })),
        // Step 3: Stop the drivetrain once in range.
        new InstantCommand(drive::stop, drive),
        // Step 4: Execute the intake routine (open arm, suck in coral, and close arm).
        IntakeCommands.runIntakeRoutine(intake));
  }
}
