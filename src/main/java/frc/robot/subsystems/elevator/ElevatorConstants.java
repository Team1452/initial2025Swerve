package frc.robot.subsystems.elevator;

public class ElevatorConstants {

  public static final int kmotorOnePort = 8;
  public static final int kmotorTwoPort = 9;
  public static final int kshoulderPort = 10;
  public static final boolean motorsInverted = true;
  public static final double[] kElevatorHeights = {10, 11, 12, 13}; // Handoff height, tier 1, 2, 3.
  public static final double coralPosition = 3; // UPDATE
  public static final double[] kElevatorGains = {
    0.3, // P
    0, // I
    0, // D
    0 // F
  };
  public static final double[] kShoulderGains = {
    0.2, // P
    0, // I
    0, // D
    0 // F
  };
  public static final double kShoulderOffset = 0.7566;
  public static final double shoulderLength = 20;
  public static final double maxHeight = 42; // Conversion factor needs to be implemented.
}
