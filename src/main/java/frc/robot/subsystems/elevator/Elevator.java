package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Elevator subsystem for controlling the robot's elevator mechanism.
 *
 * <p>This implementation uses a hardware interface defined by {@link ElevatorIO} and logs sensor
 * inputs using Junction's Logger. The elevator's motion is controlled using preset upward/downward
 * speeds defined in {@link ElevatorConstants}.
 *
 * <p>References:
 *
 * <ul>
 *   <li>WPILib Command-Based Programming:
 *       https://docs.wpilib.org/en/stable/docs/software/command-based/index.html
 *   <li>AdvantageKit: https://github.com/Mechanical-Advantage/AdvantageKit
 *   <li>REVLib and CTREPhoenix Documentation (as applicable) for motor and sensor control.
 * </ul>
 */
public class Elevator extends SubsystemBase {
  // Hardware interface for the elevator subsystem.
  private final ElevatorIO io;

  // Inputs from the elevator hardware.
  // ElevatorIOInputs could include fields like height (in meters), velocity, and limit switch
  // statuses.
  private final ElevatorIO.ElevatorIOInputs inputs = new ElevatorIO.ElevatorIOInputs();

  /**
   * Constructs an Elevator subsystem.
   *
   * @param io The concrete hardware interface for the elevator.
   */
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update sensor inputs from the hardware.
    io.updateInputs(inputs);

    // Log sensor values to the Dashboard (or any logging mechanism you use).
    Logger.recordOutput("Elevator/Height", inputs.height);
    Logger.recordOutput("Elevator/Velocity", inputs.velocity);
    Logger.recordOutput("Elevator/AtTop", inputs.atTop);
    Logger.recordOutput("Elevator/AtBottom", inputs.atBottom);
    Logger.recordOutput("Elevator/HasCoral", inputs.hasCoral);
  }

  /** Raises the elevator using the configured upward speed. */
  public void raiseElevator() {
    io.setMotorOutput(ElevatorConstants.kupSpeed);
  }

  /** Lowers the elevator using the configured downward speed. */
  public void lowerElevator() {
    io.setMotorOutput(-ElevatorConstants.kdownSpeed);
  }

  /** Stops the elevator by setting the motor output to zero. */
  public void stopElevator() {
    io.setMotorOutput(0.0);
  }

  /**
   * Passes a custom motor output directly to the elevator. Typically, the output should be in the
   * range [-1.0, 1.0].
   *
   * @param output The motor output value.
   */
  public void setMotorOutput(double output) {
    io.setMotorOutput(output);
  }

  /**
   * Returns the current measured height of the elevator.
   *
   * @return The elevator height in meters.
   */
  public double getHeight() {
    return inputs.height;
  }
}
