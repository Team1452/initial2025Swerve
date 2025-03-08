package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  // Hardware interface for the elevator.
  private final ElevatorIO io;
  public static double elevatorRHeight = 0;
  public static double elevatorRAngle = 0.275;
  public static double minHeight = 0;
  BooleanSupplier intakeStateSupplier;

  // Inputs from the elevator hardware.
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  /**
   * @param io The interfce)
   */
  public Elevator(ElevatorIO io, BooleanSupplier intakeStateSupplier) {
    this.io = io;
    this.intakeStateSupplier =
        intakeStateSupplier; // Allows some communication between the intake and the elevator
    // subsystems.
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs); // Refresh the inputs.
    calcMinHeight(); // Guess.
    io.setHeight(elevatorRHeight); // Set the height of the elevator to the modifiedRHeight.
    io.setShoulderAngle(elevatorRAngle); // Set the angle of elevator every cycle. This
    // prevents the elevator from
    // moving when no signal is sent.

    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/Height", inputs.height);
    Logger.recordOutput("Elevator/RequestedHeight", elevatorRHeight);
    Logger.recordOutput("Elevator/RequestedAngle", elevatorRAngle);
    Logger.recordOutput("Elevator/MinHeight", minHeight);
    Logger.recordOutput("Elevator/AbsoluteAngle", inputs.internalAngle);
    Logger.recordOutput("Elevator/InternalAngle", inputs.shoulderAngle);
    Logger.recordOutput("Elevator/LimitSwtich", inputs.elevatorlimitSwtich);
  }

  public void adjustRHeight(double height) {
    elevatorRHeight =
        MathUtil.clamp(
            elevatorRHeight + height,
            minHeight,
            ElevatorConstants.maxHeight); // Clamp the height between the min and max positions.
  }

  public void adjustRAngle(double angle) {
    elevatorRAngle += angle;
  }

  public void setRHeight(double height) {
    elevatorRHeight =
        MathUtil.clamp(
            height,
            minHeight,
            ElevatorConstants.maxHeight); // Clamp the height between the min and max positions.
  }

  public void setRAngle(double angle) {
    elevatorRAngle = angle;
  }

  public double getShoulderAngle() {
    return inputs.shoulderAngle;
  }

  public void resetEncoder() {
    io.resetEncoder();
  }

  public double getHeight() {
    return inputs.height;
  }

  public BooleanSupplier eLimitSwitch() {
    return (() -> inputs.elevatorlimitSwtich);
  }

  private void calcMinHeight() {
    minHeight =
        !intakeStateSupplier.getAsBoolean() // If the intake is closed,
            ? (inputs.shoulderAngle < 0.5 // and the shoulder is up
                ? ElevatorConstants.intakeHeight // Then the min height is the intake height.
                : ElevatorConstants.intakeHeight
                    + ElevatorConstants.shoulderLength
                        * Math.sin(
                            2
                                * Math.PI
                                * (1
                                    - inputs
                                        .shoulderAngle))) // If it's down, it's the intake height +
            // a sin compliment
            : (inputs.shoulderAngle > 0.5 // If the intake is open
                ? ElevatorConstants.shoulderLength
                    * Math.sin(
                        2
                            * Math.PI
                            * (1 - inputs.shoulderAngle)) // And the shoulder is down, then the min
                // height is the calculated sin compliement
                : -2); // If it's up, then the min height is -2 (so we have some play)
  }
}
