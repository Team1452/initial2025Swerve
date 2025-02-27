package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double height = 0;
    public double internalAngle = 0;
    public double shoulderAngle = 0;
  }

  public default void setHeight(double setpoint) {}

  public default void setShoulderAngle(double setpoint) {}

  public default void rotateShoulder(double setpoint) {}

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void resetEncoder() {}
}
