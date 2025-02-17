package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double height = 0;
    public double velocity = 0;
    public double shoulderPos = 0;
    public int level = 0;
  }

  public default void setHeight(double setPoint) {}

  public default void setShoulderPosition(double setPoint) {}

  public default void updateInputs(ElevatorIOInputs inputs) {}
}
