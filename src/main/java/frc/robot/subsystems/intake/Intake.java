package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  // The hardware interface for the intake subsystem.
  private final IntakeIO io;

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update sensor inputs from the hardware and log them.
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/IntakeAngle", inputs.intakeAngle);
    Logger.recordOutput("Intake/IntakeOpen", inputs.intakeOpen);
    Logger.recordOutput("Intake/rotatorCurrent", inputs.rotatorCurrent);
    Logger.recordOutput("Intake/suckerSpeed", inputs.suckerSpeed);
  }

  /** Spins the sucker roller using the aconfigured suck speed. */
  public void spinSucker(boolean forward) {
    io.setSuckerVelocity(
        forward ? IntakeConstants.intakeSuckSpeed : -IntakeConstants.intakeSuckSpeed);
  }

  public void spinSucker(double speed) {
    io.setSuckerVelocity(speed);
  }

  public void suckSucker() {
    io.setSuckerVelocity(0.4);
  }

  public void stopSucker() {
    io.setSuckerVelocity(0);
  }

  public double getSuckerCurrent() {
    return inputs.suckerCurrent;
  }

  public boolean getSuckerStop() {
    return inputs.suckerSpeed < 2;
  }

  public boolean getRotatorStopForward() {
    return inputs.rotatorSpeed < 10 && inputs.intakeAngle > 8;
  }

  public boolean getRotatorStopBack() {
    return inputs.rotatorSpeed < 10 && inputs.intakeAngle < -1.5;
  }

  public double getRotatorCurrent() {
    return inputs.rotatorCurrent;
  }

  public void rotateOutIntake() {
    io.setRotatorVelocity(IntakeConstants.intakeRotateOutSpeed);
  }

  public void rotateInIntake() {
    io.setRotatorVelocity(IntakeConstants.intakeRotateInSpeed);
  }

  public void rotateInDead() {
    io.setRotatorVelocity(IntakeConstants.intakeRotateInSpeed);
    Commands.waitSeconds(0.5);
    stopIntake();
  }

  public void stopIntake() {
    io.setRotatorVelocity(0);
  }

  public double getIntakeAngle() {
    return inputs.intakeAngle;
  }

  public boolean getIntakeOpen() {
    return inputs.intakeOpen;
  }
}
