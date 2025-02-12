package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import java.util.function.BooleanSupplier;

public class IntakeCommands {
  private IntakeCommands() {}

  /**
   * Creates a command that rotates the intake.
   *
   * @param intake The intake subsystem.
   * @param open If true, rotates to open the intake; if false, rotates to close.
   * @return a command that runs the intake rotation until the desired state is reached, then stops
   *     the intake.
   */
  private static Command rotateIntakeCommand(Intake intake, boolean open) {
    // When opening, the condition is met if getIntakeState() returns true.
    // When closing, the condition is met if getIntakeState() returns false.
    BooleanSupplier condition = open ? intake::getIntakeState : () -> !intake.getIntakeState();
    return Commands.sequence(
        Commands.run(() -> intake.rotateIntake(open), intake).until(condition),
        Commands.runOnce(intake::stopIntake, intake));
  }

  public static Command openIntake(Intake intake) {
    return rotateIntakeCommand(intake, true);
  }

  public static Command closeIntake(Intake intake) {
    return rotateIntakeCommand(intake, false);
  }

  /**
   * Creates a command that spins the sucker either to suck in or spit out.
   *
   * @param intake The intake subsystem.
   * @param suck If true, spins to suck in; if false, spins to spit out.
   * @return a command that runs the sucker motor for a fixed duration and then stops it.
   */
  private static Command spinSuckerCommand(Intake intake, boolean direction) {
    return Commands.sequence(
        Commands.run(() -> intake.spinSucker(direction), intake),
        Commands.waitSeconds(0.5), // ADJUST THIS VALUE IF NEEDED.
        Commands.runOnce(intake::stopSucker, intake));
  }

  public static Command suckInCoral(Intake intake) {
    return spinSuckerCommand(intake, true);
  }

  public static Command spitOutCoral(Intake intake) {
    return spinSuckerCommand(intake, false);
  }

  public static Command runIntakeRoutine(Intake intake) {
    return Commands.sequence(
            openIntake(intake), // Open the intake
            suckInCoral(intake), // Suck in the coral
            closeIntake(intake) // Close the intake
            )
        .handleInterrupt(
            // If the command is interrupted, spit out the coral and close the intake ("panic
            // mode").
            () -> Commands.parallel(spitOutCoral(intake), closeIntake(intake)));
  }
}
