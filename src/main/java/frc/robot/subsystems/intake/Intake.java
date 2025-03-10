package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  // The hardware interface for the intake subsystem.
  private final IntakeIO io;
  private boolean slopState = false;
  private double intakeRAngle = IntakeConstants.intakeStartUpAngle;

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update sensor inputs from the hardware and log them.
    io.updateInputs(inputs);
    if (slopState) {
      io.disableVoltage();
    } else {
      io.setRotatorAngle(MathUtil.clamp(intakeRAngle, 0, IntakeConstants.intakeIntakeAngle));
    }
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/IntakeAngle", inputs.intakeAngle);
    Logger.recordOutput("Intake/IntakeOpen", getIntakeOpen());
    Logger.recordOutput("Intake/suckerSpeed", inputs.suckerSpeed);
    Logger.recordOutput("Intake/RequestedAngle", intakeRAngle);
  }

  public boolean nearRPosition() {
    return MathUtil.isNear(intakeRAngle, inputs.intakeAngle, 0.01);
  }

  public double getIntakeAngle() {
    return inputs.intakeAngle;
  }

  public double getRAngle() {
    return intakeRAngle;
  }

  public boolean getRIntakeOpen() {
    return intakeRAngle > IntakeConstants.intakeStartUpAngle;
  }

  public boolean getIntakeOpen() {
    return inputs.intakeAngle > IntakeConstants.intakeStartUpAngle;
  }

  public void slightSuck() {
    io.setSuckerVelocity(0.05);
  }

  public void setSlopState(boolean state) {
    slopState = state;
  }

  public void suckSucker() {
    io.setSuckerVelocity(IntakeConstants.intakeSuckSpeed);
  }

  public void spitSucker() {
    io.setSuckerVelocity(-IntakeConstants.intakeSuckSpeed);
  }

  public void stopSucker() {
    io.setSuckerVelocity(0);
  }

  public boolean getSuckerGo() {
    return inputs.suckerSpeed > 10;
  }

  public boolean getSuckerStop() {
    return inputs.suckerSpeed < 1;
  }

  public void setIntakeAngle(double angle) {
    intakeRAngle = angle;
  }

  public void adjustRotatorAngle(double angle) {
    intakeRAngle += angle;
  }
}
