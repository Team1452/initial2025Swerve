package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  // Hardware interface for the elevator.
  private final ElevatorIO io;
  private double elevatorRHeight = 0;
  // Inputs from the elevator hardware.
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  /**
   * @param io The interfce)
   */
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.setHeight(MathUtil.clamp(elevatorRHeight, 0, ElevatorConstants.maxHeight));
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/ElevatorRHeight", elevatorRHeight);
    Logger.recordOutput("Elevator/ElevatorHeight", inputs.height);
  }

  public boolean nearRPosition() {
    return MathUtil.isNear(elevatorRHeight, inputs.height, 0.05);
  }

  public void adjustRHeight(double height) {
    elevatorRHeight += height;
  }

  public void setRHeight(double height) {
    elevatorRHeight = height;
  }

  public void resetEncoder() {
    io.resetEncoder();
  }

  public boolean eLimitSwitch() {
    return inputs.elevatorlimitSwtich;
  }

  public double getHeight() {
    return inputs.height;
  }

  public double getRHeight() {
    return elevatorRHeight;
  }
}
