package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.Vision;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * An autonomous command for intake of CORAL using a vision-based alignment strategy.
 *
 * <p>This command uses the new {@link DriveCommands#joystickDriveAtAngle(Drive, DoubleSupplier,
 * DoubleSupplier, Supplier)} command to control field-relative motion. The PID used internally by
 * that command (configured following WPILib documentation for ProfiledPIDController with a
 * TrapezoidProfile) continuously drives the angular motion toward a setpoint computed from the
 * current heading plus vision yaw error. The vision system (implemented per AdvantageKit and WPILib
 * vision docs) provides the target observations.
 *
 * <p>The routine consists of several phases:
 *
 * <ol>
 *   <li>Pre-align: Rotate until the yaw error is less than 2°.
 *   <li>Approach: Drive forward (using the provided forward joystick input) while maintaining
 *       alignment, until the target pitch (a proxy for distance) exceeds a threshold.
 *   <li>Stop: Halt the drive.
 *   <li>Intake: Run the coral intake routine.
 * </ol>
 *
 * <p>References:
 *
 * <ul>
 *   <li>WPILib Command-Based Programming
 *       https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html
 *   <li>WPILib's ProfiledPIDController and TrapezoidProfile docs for trajectory control.
 *   <li>AdvantageKit guidelines.
 *   <li>REVLib and CTREPhoenix documentation (for drivetrain and sensor interfaces).
 * </ul>
 */
public class AutoCoralIntake extends SequentialCommandGroup {
  // Threshold (in degrees) for the vision target pitch indicating that the coral is in range.
  private static final double IN_RANGE_PITCH_THRESHOLD_DEGREES = 10.0;

  /**
   * Constructs an AutoCoralIntake command.
   *
   * @param drive The drive subsystem.
   * @param vision The vision subsystem (using camera index 2 for target observations).
   * @param intake The intake subsystem.
   * @param forwardSupplier A DoubleSupplier for forward drive input (usually from a joystick, range
   *     -1.0 to 1.0).
   */
  public AutoCoralIntake(
      Drive drive,
      Vision vision,
      Intake intake,
      Elevator elevator,
      DoubleSupplier forwardSupplier) {
    // Create a rotation supplier that uses vision to guide the desired angle.
    // The setpoint is computed as the robot's current heading plus the vision yaw error.
    // This provides a continuously updated target for the built-in PID in joystickDriveAtAngle.
    Supplier<Rotation2d> rotationSupplier =
        () -> drive.getRotation().plus(vision.getTargetObservation(2).tx());

    addCommands(
        // Phase 1: Pre-align the robot using vision until the yaw error is below 2°.
        new ParallelRaceGroup(
            // Use joystickDriveAtAngle with zero translation (only rotation control).
            DriveCommands.joystickDriveAtAngle(drive, () -> 0.0, () -> 0.0, rotationSupplier),
            // End this phase when the vision yaw error is small.
            new WaitUntilCommand(
                () -> Math.abs(vision.getTargetObservation(2).tx().getDegrees()) < 2.0)),
        // Phase 2: Drive forward (using the provided forward supplier) while maintaining alignment.
        new ParallelRaceGroup(
            // Drive forward while the PID corrects the angle based on vision.
            DriveCommands.joystickDriveAtAngle(drive, () -> 0.0, forwardSupplier, rotationSupplier),
            // End when the vision pitch indicates the coral is in range.
            new WaitUntilCommand(
                () ->
                    Math.abs(vision.getTargetObservation(2).ty().getDegrees())
                        > IN_RANGE_PITCH_THRESHOLD_DEGREES)),
        // Phase 3: Stop the drivetrain.
        new InstantCommand(drive::stop, drive),
        // Phase 4: Execute the coral intake routine.
        IntakeCommands.intakeCoralAndStow(intake));
  }
}
