package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

/**
 * Commands for moving the elevator to preset positions using closed-loop control.
 */
public class ElevatorCommands {

  /** Command to move the elevator to the Tier One position. */
  public static InstantCommand goToTierOne(Elevator elevator) {
    return new InstantCommand(() -> elevator.moveToPosition(ElevatorConstants.kTierOneHeight), elevator);
  }

  /** Command to move the elevator to the Tier Two position. */
  public static InstantCommand goToTierTwo(Elevator elevator) {
    return new InstantCommand(() -> elevator.moveToPosition(ElevatorConstants.kTierTwoHeight), elevator);
  }

  /** Command to move the elevator to the Tier Three position. */
  public static InstantCommand goToTierThree(Elevator elevator) {
    return new InstantCommand(() -> elevator.moveToPosition(ElevatorConstants.kTierThreeHeight), elevator);
  }
}
