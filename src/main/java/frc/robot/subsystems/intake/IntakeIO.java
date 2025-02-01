package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;


public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public Angle intakeAngle = Rotations.of(0);
    public boolean intakeState = false; //False for closed, true for open
    public double suckerSpeed = 0;
    public boolean suckerRunning = false;
    

  }
  public default void setRotatorVelocity(double speed) {}
  public default void setSuckerVelocity(double speed) {}
  public default void updateInputs(IntakeIOInputs inputs) {}
}
