package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
import java.util.function.BooleanSupplier;

public class IntakeCommandClosedLoop {
  private IntakeCommandClosedLoop() {}
  public static Command rotateTo(Intake intake, double angle) {
    return Commands.runOnce(() -> intake.setRotatorAngle(angle), intake);
  }
}
