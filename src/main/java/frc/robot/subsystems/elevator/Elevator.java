package frc.robot.subsystems.elevator;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ElevatorCommands;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  // Hardware interface for the elevator.
  private final ElevatorIO io;

  // Inputs from the elevator hardware.
  private final ElevatorIOInputs inputs = new ElevatorIOInputs();
  /**
   *.
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
    io.setHeight(setpoint >= ElevatorConstants.shoulderLength ? setpoint : Math.max((ElevatorConstants.shoulderLength *  Math.cos(getShoulderAngle() * 2* 3.14))+0.3, setpoint));
    //When at a height where arm may impact, Set the height the max of either the requested Setpoint or the minSetPoint,
    //Which is the cos of the shoulder angle (conv to radians) * the length of the arm, so minSetPoint is the height of the end of the shoulder, +0.3 for good measure.
  }

  public void moveToShoulderAngle(double setpoint) {
    io.setShoulderAngle(setpoint);
  }
}
