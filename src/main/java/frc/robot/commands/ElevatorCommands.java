package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class ElevatorCommands {
  private ElevatorCommands() {}

  public static Command moveElevatorTo(Elevator elevator, double height) {
    return Commands.run(() -> elevator.setRHeight(height), elevator).until(elevator::nearRPosition);
  }

  public static Command goToTier(Elevator elevator, int tier) {
    return moveElevatorTo(elevator, ElevatorConstants.kElevatorHeights[tier]);
  }
}
