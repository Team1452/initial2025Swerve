package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

import org.littletonrobotics.junction.Logger;


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
    Logger.recordOutput("Elevator/ShoulderPosition", inputs.shoulderPos);
  }

  /** Raises the elevator using a predefined open-loop upward speed. */
  

  /** Returns the current measured height of the elevator. */
  public double getHeight() {
    return inputs.height;
  }
  public double getShoulderPos() {
    return inputs.shoulderPos;
  }

  /**
   * Moves the elevator to the specified setpoint using closed-loop position control.
   * This method leverages REVLib's internal PID controller via the IO's setPosition() method.
   *
   * @param setpoint The target elevator position in native encoder units.
   */
  public void moveToPosition(double setpoint) {
    io.setHeight(setpoint);
  }

  public void moveToShoulderPosition(double setpoint){
    io.setShoulderPosition(setpoint);
  }
}
