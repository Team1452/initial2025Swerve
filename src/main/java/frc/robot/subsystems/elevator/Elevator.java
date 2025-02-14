package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

import org.littletonrobotics.junction.Logger;

/**
 * Elevator subsystem for controlling the robot's elevator mechanism.
 *
 * <p>This implementation uses a closed-loop control method via REVLib's SparkClosedLoopController.
 * The ElevatorIOSparkClosedLoop (or equivalent) is expected to implement setPosition() to drive
 * the motor using its onboard PID.
 *
 * <p>References:
 *   <ul>
 *     <li>WPILib Command-Based Programming:
 *         https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html</li>
 *     <li>AdvantageKit:
 *         https://github.com/Mechanical-Advantage/AdvantageKit</li>
 *     <li>REVLib:
 *         https://codedocs.revrobotics.com/java/</li>
 *     <li>CTREPhoenix Documentation:
 *         https://phoenix-documentation.readthedocs.io/en/latest/</li>
 *   </ul>
 */
public class Elevator extends SubsystemBase {
  // Hardware interface for the elevator.
  private final ElevatorIO io;

  // Inputs from the elevator hardware.
  private final ElevatorIOInputs inputs = new ElevatorIOInputs();

  /**
   * Constructs the Elevator subsystem.
   *
   * @param io The closed-loop hardware interface (e.g. ElevatorIOSparkClosedLoop)
   */
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update sensor inputs.
    io.updateInputs(inputs);

    // Log sensor values using AdvantageKit's Logger.
    Logger.recordOutput("Elevator/Height", inputs.height);
    Logger.recordOutput("Elevator/Velocity", inputs.velocity);
    Logger.recordOutput("Elevator/HasCoral", inputs.hasCoral);
  }

  /** Raises the elevator using a predefined open-loop upward speed. */
  public void raiseElevator() {
    io.setMotorOutput(ElevatorConstants.kupSpeed);
  }

  /** Lowers the elevator using a predefined open-loop downward speed. */
  public void lowerElevator() {
    io.setMotorOutput(-ElevatorConstants.kdownSpeed);
  }

  /** Stops the elevator. */
  public void stopElevator() {
    io.setMotorOutput(0.0);
  }

  /** Provides direct open-loop control of the elevator motor output. */
  public void setMotorOutput(double output) {
    io.setMotorOutput(output);
  }

  /** Returns the current measured height of the elevator. */
  public double getHeight() {
    return inputs.height;
  }

  /**
   * Moves the elevator to the specified setpoint using closed-loop position control.
   * This method leverages REVLib's internal PID controller via the IO's setPosition() method.
   *
   * @param setpoint The target elevator position in native encoder units.
   */
  public void moveToPosition(double setpoint) {
    io.setPosition(setpoint);
  }
}
