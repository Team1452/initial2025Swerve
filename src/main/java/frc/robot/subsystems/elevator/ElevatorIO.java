package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double height = 0;
    public double shoulderAngle = 0;
  }

  public default void setHeight(double setPoint) {}

  public default void setShoulderAngle(double setPoint) {}

  public default void updateInputs(ElevatorIOInputs inputs) {}


}
