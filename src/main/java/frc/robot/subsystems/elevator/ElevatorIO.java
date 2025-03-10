package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double height = 0;
    public boolean elevatorlimitSwtich = false;
  }

  public default void setHeight(double setpoint) {}

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void resetEncoder() {}
}
