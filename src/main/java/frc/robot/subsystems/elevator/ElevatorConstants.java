package frc.robot.subsystems.elevator;

public class ElevatorConstants {

  public static final int kmotorOnePort = 8;
  public static final int kmotorTwoPort = 9;
  public static final int kshoulderPort = 10;
  public static final boolean motorsInverted = true;
  public static final double[] kElevatorHeights = {
    27.8, 15, 20, 25
  }; // Handoff height, tier 1, 2, 3.
  public static final double scoringAngle = 0.13;
  public static final double[] kElevatorGains = {
    0.3, // P
    0, // I
    0, // D
    0 // F
  };
  public static final double[] kShoulderGains = {
    3, // P
    0, // I[/.]
    0, // D
    0.05 // F
  };
  public static final double kShoulderOffset = 0.329; // Re adjust.
  public static final double shoulderLength = 19.5;
  public static final double maxHeight = 40;
  public static final double intakeHeight = 10;
  public static final int klimitSwitchPort = 1;
  public static final double kShoulderConversionFactor = 1.0 / 45;
}
