package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

/**
 * Commands for moving the elevator to preset positions using closed-loop control.
 */
public class ElevatorCommands {

  /** Command to move the elevator to the Tier One position. */
  public static InstantCommand goToTier(Elevator elevator, int tier ) {
    return new InstantCommand(() -> elevator.moveToPosition(ElevatorConstants.kElevatorHeights[tier]), elevator);
  }

  public static InstantCommand coralLeft(Elevator elevator) {
    return new InstantCommand( () -> elevator.moveToShoulderPosition(-ElevatorConstants.coralPosition), elevator);
  }
  public static InstantCommand coralRight(Elevator elevator) {
    return new InstantCommand( () -> elevator.moveToShoulderPosition(ElevatorConstants.coralPosition), elevator);
  }


}
