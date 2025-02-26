package frc.robot.subsystems.intake;

public final class IntakeConstants {
  public static final int IntakeID = 50; // CAN ID for the intake rotator NEEDS TO BE ADJUSTED
  public static final int RollerID = 11; // CAN ID for the intake suckewr NEEDS TO BE ADJUSTED
  public static final double intakeSuckSpeed = 0.2; // Adjust speed as needed
  public static final double intakeHandOffAngle = 1;
  public static final double INTAKE_OPEN_ANGLE = -17;
  public static final int suckerSpikeThreshhold = 60;

  public static final boolean reversedRotator = false;
  public static final boolean reversedSucker = true;
  public static final double[] kIntakeGains = {
    0.5, // P
    0, // I
    0, // D
    0 // F
  };
  public static final double intakeRotateOutSpeed = -0.3; //Negative is forward.
  public static final double intakeRotateInSpeed = 0.5;
}
