package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
public class ReefAuto extends SequentialCommandGroup {
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
  public ReefAuto(
      Drive drive,
      Vision vision,
      Intake intake,
      Elevator elevator,
      DoubleSupplier forwardSupplier) {
    addCommands(
        // Pre-align: Rotate until the yaw error is less than 2°.
        new AlignToReef(drive, vision).until(() -> !vision.getAlignToReef()),
        // Approach: Drive forward to set distance away from AprilTag
        new MoveToReef(drive, vision)
            .until(() -> !vision.getMoveReady()), // TODO: add move offset to align branch
        // Spit Coral
        IntakeCommands.spitOut(intake, false).withTimeout(0.5) // Spit out coral
        );
  }
}
