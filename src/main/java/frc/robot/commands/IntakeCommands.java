package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeCommands {
  private IntakeCommands() {}

  public static Command autoStopIntake(Intake intake) {
    return Commands.sequence(
        Commands.run(() -> intake.spinSucker(true), intake),
        Commands.waitUntil(() -> intake.getSuckerCurrent() > IntakeConstants.suckerSpikeThreshhold),
        Commands.run(intake::stopSucker, intake));
  }

  public static Command runIntakeRoutine(Intake intake) {
    return Commands.run(intake::rotateOutIntake, intake) // Start opening the intake
        .until(
            () -> intake.getIntakeAngle() > IntakeConstants.INTAKE_OPEN_ANGLE) // Until its too far.
        .andThen(
            () -> {
              intake.stopIntake(); // Stop the rotation out.
              autoStopIntake(intake); // Suck in a coral and stop once its in.
              intake.rotateInIntake(); // Once we have a coral, start going back in.
              intake.spinSucker(
                  0.03); // Spin the sucker slightly to keep the coral during rotation in.
              Commands.waitUntil(
                  () ->
                      intake.getIntakeAngle()
                          >= IntakeConstants
                              .intakeHandOffAngle); // Wait until it's in the handoff position.
              intake.stopIntake(); // Stop the intake.
              intake.stopSucker(); // Stop the sucker.
            });
  }

  public static Command fullIntakeHandOff(Intake intake, Elevator elevator) {
    return Commands.sequence(
        runIntakeRoutine(intake), // Pick up a coral
        ElevatorCommands.pickUpCoralFromIntake(elevator) // And handoff to the elevator.
        );
  }
}
