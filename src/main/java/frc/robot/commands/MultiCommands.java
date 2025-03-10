package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shoulder.Shoulder;

public class MultiCommands {
  private MultiCommands() {}

  public static Command handOff(Intake intake, Elevator elevator, Shoulder shoulder) {
    return Commands.sequence(
        Commands.parallel(
            ElevatorCommands.moveElevatorTo(
                elevator, ElevatorConstants.kElevatorHeights[0] + 5), // move the elevator up.
            ShoulderCommands.moveShoulderTo(shoulder, 0.75) // Move shoulder down
            ),
        ElevatorCommands.moveElevatorTo(
            elevator,
            ElevatorConstants.kElevatorHeights[0]
                + 1), // move the elevator to right above handoff height.
        Commands.parallel(
            IntakeCommands.spitOut(intake, true), // spit out the coral
            ElevatorCommands.moveElevatorTo(
                elevator, ElevatorConstants.kElevatorHeights[0]) // handoff
            ),
        ElevatorCommands.moveElevatorTo(
            elevator, ElevatorConstants.kElevatorHeights[0] + 1), // move back up
        ShoulderCommands.moveShoulderTo(shoulder, 0.25), // rotate shoulder back up
        ElevatorCommands.moveElevatorTo(
            elevator,
            ElevatorConstants.intakeHeight
                + 2) // move the elevator down to get closer to scoring pos.
        );
  }
}
