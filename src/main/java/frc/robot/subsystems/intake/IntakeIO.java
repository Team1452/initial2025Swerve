package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public Angle intakeAngle = Rotations.of(0);
    public double suckerSpeed = 0;
    public double suckerCurrent = 0;
  }

  public default void setRotatorVelocity(double speed) {}

  public default void setSuckerVelocity(double speed) {}

  public default void setIntakeAngle(Angle angle){};

  public default void updateInputs(IntakeIOInputs inputs) {}
}
