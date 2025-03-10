package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.shoulder.Shoulder;

public class ShoulderCommands {
  public static Command moveShoulderTo(Shoulder shoulder, double angle) {
    return Commands.run(() -> shoulder.setRAngle(angle), shoulder).until(shoulder::nearRPosition);
  }

  public static Command place(Shoulder shoulder) {
    return moveShoulderTo(shoulder, ElevatorConstants.scoringAngle);
  }
}
