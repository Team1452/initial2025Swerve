package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.IntakeCommands;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  // The hardware interface for the intake subsystem.
  private final IntakeIO io;

  private final IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update sensor inputs from the hardware and log them.
    io.updateInputs(inputs);
    Logger.recordOutput("Intake/IntakeAngle", inputs.intakeAngle);
    Logger.recordOutput("Intake/SuckerSpeed", inputs.suckerSpeed);
    Logger.recordOutput("Intake/SuckerCurrent", inputs.suckerCurrent);
  }
  /**
   * Rotates the intake using the configured rotation speed. This is used to drive the intake
   * towards an open position.
   */

  /** Stops the intake rotation. */
  public void stopIntake() {
    io.setRotatorVelocity(0);
  }
  /** Spins the sucker roller using the configured suck speed. */
  public void spinSucker(boolean forward) {
    io.setSuckerVelocity(
        forward ? IntakeConstants.INTAKE_SUCK_SPEED : -IntakeConstants.INTAKE_SUCK_SPEED);
  }
  /** Stops the sucker roller. */
  public void stopSucker() {
    io.setSuckerVelocity(0);
  }

  public double getSuckerCurrent() {
    return inputs.suckerCurrent;
  }

  public Angle getIntakeAngle() {
    return inputs.intakeAngle;
  }

  public void setIntakeAngle(Angle angle) {
    io.setIntakeAngle(angle);
  }
}
