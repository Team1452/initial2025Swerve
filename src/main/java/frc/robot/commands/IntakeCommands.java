package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;



public class IntakeCommands {
  private IntakeCommands() {}
  public static boolean intakeState = false; //start with closed intake;\
  public static boolean hasCoral = false;
  /**
   * Creates a command that rotates the intake.
   *
   * @param intake The intake subsystem.
   * @param open If true, rotates to open the intake; if false, rotates to close.
   * @return a command that runs the intake rotation until the desired state is reached, then stops
   *     the intake.
   */
  private static Command rotateIntakeCommand(Intake intake, Angle angle) {

    return Commands.run( ()-> intake.setIntakeAngle(angle), intake);
  }

  public static Command openIntake(Intake intake) {
    return rotateIntakeCommand(intake, IntakeConstants.INTAKE_OPEN_ANGLE) //Set the ref angle to open
    .until(()-> intake.getIntakeAngle().isNear(IntakeConstants.INTAKE_OPEN_ANGLE,0.05))  //Keep setting until it's open.
    .finallyDo(() -> intakeState = true); //Then set the state.
  }

  public static Command closeIntake(Intake intake) {
    return rotateIntakeCommand(intake, IntakeConstants.INTAKE_CLOSED_ANGLE) //Set the ref angle to open
    .until(()-> intake.getIntakeAngle().isNear(IntakeConstants.INTAKE_CLOSED_ANGLE,0.05))  //Keep setting until it's closed.
    .finallyDo(() -> intakeState = false); //Then set the state.
  }

  /**
   * Creates a command that spins the sucker either to suck in or spit out.
   *
   * @param intake The intake subsystem.
   * @param suck If true, spins to suck in; if false, spins to spit out.
   * @return a command that runs the sucker motor for a fixed duration and then stops it.
   */
  private static Command spinSuckerCommand(Intake intake, boolean direction) {
     return Commands.run(() -> intake.spinSucker(direction), intake);
        
  }
  public static Command suckInCoral(Intake intake) {
    return spinSuckerCommand( intake, true)
    .until(() -> (intake.getSuckerCurrent() > IntakeConstants.suckerSpikeThreshhold) )
    .andThen(() -> hasCoral = true);
  }
  public static Command spitOutCoral(Intake intake) {
    hasCoral = false;
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
