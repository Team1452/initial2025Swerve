package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {
  private IntakeCommands() {}

  public static Command rotateOut(Intake intake) {
    return Commands.sequence(
        Commands.runOnce(intake::rotateOutIntake, intake),
        Commands.waitUntil(intake::getRotatorStopForward),
        Commands.print("Limit!"),
        Commands.runOnce(intake::stopIntake, intake));
  }

  public static Command runIntakeRoutine(Intake intake, Elevator elevator) {
    return Commands.sequence(
        Commands.runOnce(intake::suckSucker, intake),
        Commands.runOnce(intake::rotateOutIntake, intake),
        Commands.waitUntil(intake::getRotatorStopForward),
        Commands.runOnce(intake::stopIntake, intake),
        Commands.waitUntil(intake::getSuckerStop),
        Commands.runOnce(intake::stopSucker, intake),
        Commands.runOnce(
            () -> {
              if (elevator.getHeight() < ElevatorConstants.intakeHeight) {
                elevator.setRHeight(ElevatorConstants.intakeHeight + 2);
              }
              intake.rotateInIntake(); // Once we have a coral, start going back in.
              intake.spinSucker(
                  0.03); // Spin the sucker slightly to keep the coral during rotation in.
            },
            intake),
        Commands.waitUntil(intake::getRotatorStopBack), // Wait until it's in the handoff position
        Commands.runOnce(() -> intake.spinSucker(-0.1), intake), // Spit out.
        Commands.waitSeconds(0.02),
        Commands.runOnce(intake::stopIntake, intake),
        Commands.runOnce(intake::stopSucker, intake) // Stop the sucker.
        ).handleInterrupt( ()-> {
          intake.stopIntake();
          intake.stopSucker();
        }
        );
  }

  public static Command fullIntakeHandOff(Intake intake, Elevator elevator) {
    return Commands.sequence(
        runIntakeRoutine(intake, elevator), // Pick up a coral
        ElevatorCommands.pickUpCoralFromIntake(elevator, intake) // And handoff to the elevator.
        );
  }
}
