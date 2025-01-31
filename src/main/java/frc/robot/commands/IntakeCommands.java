package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {
    private IntakeCommands() {}

    public static Command runIntakeSeconds(Intake intake, double seconds) {
        return Commands.runOnce(
            () -> {
                intake.run();
                Commands.waitSeconds(seconds);
                intake.stop();
            },  intake);
    }

}