package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
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
        Commands.waitUntil(
            () ->
                MathUtil.isNear(IntakeConstants.intakeLevelOneAngle, intake.getIntakeAngle(), 0.1)),
        spitOut(intake));
  }

  public static Command intakeCoral(Intake intake) {
    return Commands.sequence(
        rotateTo(intake, IntakeConstants.intakeIntakeAngle), // Go to the intake angle
        Commands.waitUntil(
            () ->
                MathUtil.isNear(
                    IntakeConstants.intakeIntakeAngle,
                    intake.getIntakeAngle(),
                    0.2)), // Wait until it's there
        Commands.runOnce(
            () -> intake.setSlopState(true),
            intake), // Set the slop state to true so the intake moves around
        Commands.runOnce(intake::suckSucker, intake), // suck the suck
        Commands.waitSeconds(0.3), // wait for it to start sucking
        Commands.waitUntil(
            intake::getSuckerStop), // wait for it to stop sucking (means that there is a coral)
        Commands.runOnce(
            () -> {
              intake.spinSucker(0.1);
              intake.setSlopState(false);
              intake.setRotatorAngle(IntakeConstants.intakeLevelOneAngle);
            },
            intake) // Keep sucking a little to keep it in.
        );
  }
}
