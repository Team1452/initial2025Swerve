package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import java.util.function.DoubleSupplier;

/** Commands for moving the elevator to preset positions using closed-loop control. */
public class ElevatorCommands {

  // Current tier index for cycling through elevator positions.
  // This value will update as you cycle through the presets.
  public static int currentTier = 0;
  public static double requestedPosition;
  public static double requestedAngle;

  /** Command to move the elevator to the specified tier position. */
  public static InstantCommand goToTier(Elevator elevator, int tier) {
    // Update the current tier index for consistency with cycling commands.
    currentTier = tier;
    return new InstantCommand(
        () -> elevator.moveToPosition(ElevatorConstants.kElevatorHeights[tier]), elevator);
  }

  /** Command to cycle to the next elevator tier (D-Pad Up). */
  public static InstantCommand tierUp(Elevator elevator) {
    System.out.println("Up");
    return new InstantCommand(
        () -> {
          currentTier = (currentTier + 1) % ElevatorConstants.kElevatorHeights.length;
          elevator.moveToPosition(ElevatorConstants.kElevatorHeights[currentTier]);
        },
        elevator);
  }

  /** Command to cycle to the previous elevator tier (D-Pad Down). */
  public static InstantCommand tierDown(Elevator elevator) {
    System.out.println("Down");
    return new InstantCommand(
        () -> {
          // Adding the length before modulo to keep the result positive.
          currentTier =
              (currentTier - 1 + ElevatorConstants.kElevatorHeights.length)
                  % ElevatorConstants.kElevatorHeights.length;
          elevator.moveToPosition(ElevatorConstants.kElevatorHeights[currentTier]);
        },
        elevator);
  }

  public static Command moveElevatorUD(Elevator elevator, DoubleSupplier input) {
    requestedPosition = elevator.getHeight();
    return Commands.run(
            () -> {
              requestedPosition += (MathUtil.applyDeadband(input.getAsDouble(), 0.3)) / 5;
              elevator.moveToPosition(requestedPosition);
            },
            elevator)
        .repeatedly()
        .handleInterrupt(() -> elevator.moveToPosition(elevator.getHeight()));
  }

  public static Command controlElevatorXY(
    Elevator elevator, DoubleSupplier inputx, DoubleSupplier inputy) {
    requestedPosition = elevator.getHeight();
    return Commands.run(
            () -> {
              requestedPosition += 
              (requestedPosition < elevator.getMinHeight())
                  ? Math.max(0, (MathUtil.applyDeadband(inputy.getAsDouble(), 0.3)) / 5)
                  : (requestedPosition > ElevatorConstants.maxHeight)
                      ? Math.min(0, (MathUtil.applyDeadband(inputy.getAsDouble(), 0.3)) / 5)
                      : (MathUtil.applyDeadband(inputy.getAsDouble(), 0.3)) / 5;
              elevator.moveToPosition(requestedPosition);
              elevator.moveShoulderBy(MathUtil.applyDeadband(inputx.getAsDouble(), 0.3) / 3.5);
            },
            elevator)
        .repeatedly()
        .handleInterrupt(
            () -> {
              elevator.moveToPosition(elevator.getHeight());
              elevator.moveToShoulderAngle(elevator.getShoulderAngle());
            });
  }

  public static Command rotateShoulder(Elevator elevator, DoubleSupplier input) {
    requestedAngle = elevator.getShoulderAngle();
    return Commands.run(
            () -> {
              requestedAngle += (MathUtil.applyDeadband(Math.pow(input.getAsDouble(), 1), 0.3)) / 5;
              elevator.moveToShoulderAngle(requestedAngle);
            },
            elevator)
        .repeatedly()
        .handleInterrupt(() -> elevator.moveToShoulderAngle(elevator.getShoulderAngle()));
  }
}
