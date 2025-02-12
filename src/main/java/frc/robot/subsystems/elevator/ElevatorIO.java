package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double height = 0;
    public double velocity = 0;
    public boolean atTop = false;
    public boolean atBottom = false;
    public boolean hasCoral = false;
  }

  public default void setMotorOutput(double output) {}

  public default void updateInputs(ElevatorIOInputs inputs) {}
}
