package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    Logger.recordOutput("Intake/IntakeOpen", inputs.intakeState);
    Logger.recordOutput("Intake/SuckerSpeed", inputs.suckerSpeed);
    Logger.recordOutput("Intake/SuckerRunning", inputs.suckerRunning);
  }
  /**
   * Rotates the intake using the configured rotation speed. This is used to drive the intake
   * towards an open position.
   */
  public void rotateIntake(boolean forward) {
    io.setRotatorVelocity(
        forward ? IntakeConstants.INTAKE_ROTATION_SPEED : -IntakeConstants.INTAKE_ROTATION_SPEED);
  }
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

  public Angle getIntakeAngle() {
    return inputs.intakeAngle;
  }

  public boolean getIntakeState() {
    return inputs.intakeState; // True for open, false for closed
  }
}
