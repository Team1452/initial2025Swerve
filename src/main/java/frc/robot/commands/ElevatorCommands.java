package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

/** Commands for moving the elevator to preset positions using closed-loop control. */
public class ElevatorCommands {

  // Current tier index for cycling through elevator positions.
  // This value will update as you cycle through the presets.
  private static int currentTier = 0;

  /** Command to move the elevator to the specified tier position. */
  public static InstantCommand goToTier(Elevator elevator, int tier) {
    // Update the current tier index for consistency with cycling commands.
    currentTier = tier;
    return new InstantCommand(
        () -> elevator.moveToPosition(ElevatorConstants.kElevatorHeights[tier]), elevator);
  }

  public static InstantCommand coralLeft(Elevator elevator) {
    return new InstantCommand(
        () -> elevator.moveToShoulderPosition(-ElevatorConstants.coralPosition), elevator);
  }

  public static InstantCommand coralRight(Elevator elevator) {
    return new InstantCommand(
        () -> elevator.moveToShoulderPosition(ElevatorConstants.coralPosition), elevator);
  }

  /** Command to cycle to the next elevator tier (D-Pad Up). */
  public static InstantCommand tierUp(Elevator elevator) {
    return new InstantCommand(
        () -> {
          currentTier = (currentTier + 1) % ElevatorConstants.kElevatorHeights.length;
          elevator.moveToPosition(ElevatorConstants.kElevatorHeights[currentTier]);
        },
        elevator);
  }

  /** Command to cycle to the previous elevator tier (D-Pad Down). */
  public static InstantCommand tierDown(Elevator elevator) {
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
}
