package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {
    private IntakeCommands() {}
    public static Command openIntake(Intake intake) {
        // This command rotates the intake until it reaches the desired open angle defined in IntakeConstants.
        // The command runs the intake motor (via rotateIntake()) until the error is within a tolerance (here, 0.05 units),
        // then stops the motor.
        return Commands.sequence(
            Commands.run(() -> {
                intake.rotateIntake(true); //Rotate the intake 
            }, intake).until(() -> //until 
            intake.getIntakeState()), //the intake is open
            // Once the target is reached,
            Commands.runOnce(() -> intake.stopIntake(), intake) // stop the intake.
        );
    }
    public static Command closeIntake(Intake intake) {
        // This command rotates the intake until it reaches the desired open angle defined in IntakeConstants.
        // The command runs the intake motor (via rotateIntake()) until the error is within a tolerance (here, 0.05 units),
        // then stops the motor.
        return Commands.sequence(
            Commands.run(() -> {
                intake.rotateIntake(false); //Rotate the intake in reverse
            }, intake).until(() -> //until 
            !intake.getIntakeState()), //the intake is closed
            // Once the target is reached,
            Commands.runOnce(() -> intake.stopIntake(), intake) // stop the intake.
        );
    }
    public static Command suckInCoral(Intake intake) {
        return Commands.sequence(
            Commands.run(() -> intake.spinSucker(true), intake),
            Commands.waitSeconds(0.5), //ADJUST THIS VALUE IF NEEDED.
            Commands.runOnce(() -> intake.stopSucker(), intake)
        );
    }
    public static Command spitOutCoral(Intake intake) {
        return Commands.sequence(
            Commands.run(() -> intake.spinSucker(false), intake),
            Commands.waitSeconds(0.5), //ADJUST THIS VALUE IF NEEDED.
            Commands.runOnce(() -> intake.stopSucker(), intake)
        );
    }

    public static Command runIntakeRoutine(Intake intake) {
        return Commands.sequence(
            openIntake(intake), //Open the intake
            suckInCoral(intake), //Suck in the coral
            closeIntake(intake) //Close the intake
        ).handleInterrupt( //If the command is interrupted, spit out the coral and close the intake, "panic mode".
            () -> Commands.parallel(
                spitOutCoral(intake),
                closeIntake(intake)
        ));
    }
}