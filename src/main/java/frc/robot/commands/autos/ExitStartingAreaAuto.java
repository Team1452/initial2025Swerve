package frc.robot.commands.autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;

public class ExitStartingAreaAuto extends SequentialCommandGroup {
  // Threshold (in degrees) for the vision target pitch indicating that the coral is in range.

  /**
   * Constructs an AutoCoralIntake command.
   *
   * @param drive The drive subsystem.
   * @param vision The vision subsystem (using camera index 2 for target observations).
   * @param intake The intake subsystem.
   * @param forwardSupplier A DoubleSupplier for forward drive input (usually from a joystick, range
   *     -1.0 to 1.0).
   */
  public ExitStartingAreaAuto(Drive drive) {
    // Create a rotation supplier that uses vision to guide the desired angle.
    // The setpoint is computed as the robot's current heading plus the vision yaw error.
    // This provides a continuously updated target for the built-in PID in joystickDriveAtAngle.

    addCommands(
      Commands.repeatingSequence(
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-1, 0.0, 0.0)))
      )
    );
  }
}
