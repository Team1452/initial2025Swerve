package frc.robot.subsystems.intake;

public final class IntakeConstants {
  public static final int IntakeID = 50;
  public static final int RollerID = 11;
  public static final double intakeSuckSpeed = 0.4;
  public static final double intakeHandOffAngle = 0;
  public static final double intakeIntakeAngle = 17;
  public static final double intakeLevelOneAngle = 7.15;
  public static final double intakeStartUpAngle = 2.93;
  public static final int suckerSpikeThreshhold = 20;
  public static final int rotatorSpikeThreshhold = 20;

  public static final boolean reversedRotator = true;
  public static final boolean reversedSucker = true;
  public static final double[] kIntakeGains = {
    0.5, // P
    0, // I
    0, // D
    0.1 // F
  };
  public static final double intakeRotateOutSpeed = 0.4;
  public static final double intakeRotateInSpeed = -0.4;
}
