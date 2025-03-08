package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeCommandClosedLoop {
  private IntakeCommandClosedLoop() {}

  public static Command rotateTo(Intake intake, double angle) {
    return Commands.runOnce(() -> intake.setRotatorAngle(angle), intake);
  }

  public static Command spitOut(Intake intake) {
    return Commands.sequence(
        Commands.runOnce(() -> intake.spinSucker(false), intake),
        Commands.waitSeconds(0.5), // Spin for a half second.
        Commands.runOnce(intake::stopSucker, intake));
  }

  public static Command scoreLevelOne(Intake intake) {
    return Commands.sequence(
        rotateTo(intake, IntakeConstants.intakeLevelOneAngle),
        Commands.waitSeconds(0.5),
        spitOut(intake));
  }

  public static Command intakeCoral(Intake intake) {
    return Commands.sequence(
      rotateTo(intake, IntakeConstants.intakeIntakeAngle),
      Commands.run(intake::suckSucker, intake).handleInterrupt(intake::stopSucker).until(intake::getSuckerStop)
    );
  }
}
