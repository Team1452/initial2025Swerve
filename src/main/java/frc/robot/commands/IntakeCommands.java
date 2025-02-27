package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {
  private IntakeCommands() {}

  public static Command autoStopIntake(Intake intake) {
    return Commands.sequence(
        Commands.print("Suck"),
        Commands.runOnce(intake::suckSucker, intake),
        Commands.print("Waiting for spike"),
        Commands.waitUntil(intake::getSuckerStop),
        Commands.runOnce(intake::stopSucker, intake),
        Commands.print("Stop"));
  }

  public static Command rotateOut(Intake intake) {
    return Commands.sequence(
        Commands.runOnce(intake::rotateOutIntake, intake),
        Commands.waitUntil(intake::getRotatorStop),
        Commands.print("Limit!"),
        Commands.runOnce(intake::stopIntake, intake));
  }

  public static Command runIntakeRoutine(Intake intake) {
    return Commands.sequence(
        Commands.runOnce(intake::rotateOutIntake, intake),
        Commands.waitUntil(intake::getRotatorStop),
        Commands.print("Limit!"),
        Commands.runOnce(intake::stopIntake, intake),
        Commands.print("Suck"),
        Commands.runOnce(intake::suckSucker, intake),
        Commands.print("Waiting for spike"),
        Commands.waitUntil(intake::getSuckerStop),
        Commands.runOnce(intake::stopSucker, intake),
        Commands.print("Stop"),
        Commands.runOnce(
            () -> {
              intake.rotateInIntake(); // Once we have a coral, start going back in.
              intake.spinSucker(
                  0.03); // Spin the sucker slightly to keep the coral during rotation in.
            },
            intake),
        Commands.waitUntil(intake::getRotatorStop), // Wait until it's in the handoff position.
        Commands.runOnce(intake::stopIntake, intake), // Stop the intake.
        Commands.runOnce(intake::stopSucker, intake) // Stop the sucker.
        );
  }

  public static Command fullIntakeHandOff(Intake intake, Elevator elevator) {
    return Commands.sequence(
        runIntakeRoutine(intake), // Pick up a coral
        ElevatorCommands.pickUpCoralFromIntake(elevator) // And handoff to the elevator.
        );
  }
}
