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
}
