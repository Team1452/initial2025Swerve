package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  // Hardware interface for the elevator.
  private final ElevatorIO io;
  public static double elevatorRHeight = 0;
  public static double elevatorRAngle = 0.3;
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
    // intakeStateSupplier.getAsBoolean()
    //     ? angle
    //    : // If the intake is open, then set the angle to whatever. Otherwise:
    //   MathUtil.clamp(angle, 0, 0.5); // Ensure that it faces up.
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
    // intakeStateSupplier.getAsBoolean()
    //   ? angle
    // : // If the intake is open, then set the angle to whatever. Otherwise:
    // MathUtil.clamp(angle, 0, 0.5); // Ensure that it faces up.
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

  private void calcMinHeight() {
    if (!intakeStateSupplier.getAsBoolean()) {
      minHeight =
          ElevatorConstants
              .intakeHeight; // If the intake is closed, then the min is the lowest height possible
      // for intake to pass through.
      // I am assuming here that intakeHeight > 20, which is the min height when the shoulder is
      // rotated all the way down. If it is not, then there needs to be another ternary above.
    } else { // If it's open, then the min height depends on the angle of the shoulder.
      minHeight =
          ((inputs.shoulderAngle >= 0.5 && inputs.shoulderAngle <= 1))
              ? // if the shoulder is facing down:
              Math.abs( // Then return the abs value of the length of the shoulder times the sin of
                  // the shoulder angle. Who doesn't love trig?
                  ElevatorConstants.shoulderLength * Math.sin((inputs.shoulderAngle * 2 * Math.PI)))
              : 0; // Otherwise, return 0, as that is the minimum height set by the limit switch.
    }
  }
}
