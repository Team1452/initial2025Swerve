package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeCommands {
  private IntakeCommands() {}

  public static Command suckAndHold(Intake intake) {
    return Commands.sequence(
        new InstantCommand(intake::suckSucker, intake), // start sucking
        new InstantCommand(() -> intake.setSlopState(true), intake), // allow "wiggle" of intake
        Commands.waitUntil(intake::getSuckerGo), // wait until sucking is going
        Commands.waitUntil(
            intake::getSuckerStop), // after we know that we've started up, wait until we've stopped
        // (coral intake)
        new InstantCommand(
            () -> intake.setSlopState(false), intake), // stop allowing "wiggle" of intake
        new InstantCommand(intake::slightSuck, intake) // tension the coral in just a little.
        );
  }

  public static Command spitOut(Intake intake, boolean bubble) {
    return Commands.sequence(
        new InstantCommand(intake::spitSucker, intake), // Spit it out
        Commands.waitSeconds(
            bubble ? 0.1 : 0.5), // for half a second (or in bubbleUp mode, for a very short time)
        new InstantCommand(intake::stopSucker, intake) // quit it.
        );
  }

  public static Command intakeCoralAndStow(Intake intake) {
    return Commands.sequence(
        new InstantCommand(() -> intake.setIntakeAngle(IntakeConstants.intakeIntakeAngle), intake),
        suckAndHold(intake),
        new InstantCommand(
            () -> intake.setIntakeAngle(IntakeConstants.intakeHandOffAngle), intake));
  }

  public static Command scoreL1(Intake intake) {
    return Commands.sequence(
        new InstantCommand(
            () -> intake.setIntakeAngle(IntakeConstants.intakeLevelOneAngle), intake),
        Commands.waitUntil(intake::nearRPosition),
        spitOut(intake, false),
        new InstantCommand(
            () -> intake.setIntakeAngle(IntakeConstants.intakeStartUpAngle), intake));
  }
}
