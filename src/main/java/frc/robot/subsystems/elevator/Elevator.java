package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ElevatorCommands;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  // Hardware interface for the elevator.
  private final ElevatorIO io;
  static double minHeight;
  static double newSetPoint;
  // Inputs from the elevator hardware.
  private final ElevatorIOInputs inputs = new ElevatorIOInputs();
  /**
   * .
   *
   * @param io The interfce)
   */
  public Elevator(ElevatorIO io) {

    this.io = io;
  }

  @Override
  public void periodic() {
    // Update sensor inputs.
    io.updateInputs(inputs);

    // Log sensor values using AdvantageKit's Logger.
    Logger.recordOutput("Elevator/Tier", ElevatorCommands.currentTier);
    Logger.recordOutput("Elevator/RequestedPos", ElevatorCommands.requestedPosition);
    Logger.recordOutput("Elevator/MinHeight", Elevator.minHeight);
    Logger.recordOutput("Elevator/Position", inputs.height);
    Logger.recordOutput("Elevator/ShoulderAngle", inputs.shoulderAngle);
  }

  /** Raises the elevator using a predefined open-loop upward speed. */

  /** Returns the current measured height of the elevator. */
  public double getHeight() {
    return inputs.height;
  }

  public double getShoulderAngle() {
    return inputs.shoulderAngle;
  }

  /**
   * Moves the elevator to the specified setpoint using closed-loop position control. This method
   * leverages REVLib's internal PID controller via the IO's setPosition() method.
   *
   * @param setpoint The target elevator position in native encoder units.
   */
  public void moveToPosition(double setpoint) {

    minHeight =
        (inputs.shoulderAngle >= 0.25 && inputs.shoulderAngle <= 0.75)
            ? Math.abs(
                ElevatorConstants.shoulderLength
                    * Math.sin((inputs.shoulderAngle * 2 * Math.PI) + Math.PI / 2))
            : 0;
    io.setHeight(
        (setpoint < minHeight)
            ? minHeight
            : setpoint > ElevatorConstants.maxHeight ? ElevatorConstants.maxHeight : setpoint);
  }

  public void moveToShoulderAngle(double setpoint) {
    io.setShoulderAngle(setpoint);
  }

  public void moveShoulderBy(double setpoint) {
    io.rotateShoulder(setpoint);
  }

  public double getMinHeight() {
    return minHeight;
  }
}
