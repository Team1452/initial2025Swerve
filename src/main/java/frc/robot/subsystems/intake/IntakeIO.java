package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double intakeAngle = 0;
    public double suckerCurrent = 0;
    public boolean intakeOpen = false; // Whether the intake is open or closed
  }

  public default void setRotatorVelocity(double speed) {}

  public default void setSuckerVelocity(double speed) {}

  public default void updateInputs(IntakeIOInputs inputs) {}
}
