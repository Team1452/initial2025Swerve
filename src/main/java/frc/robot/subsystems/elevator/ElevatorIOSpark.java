package frc.robot.subsystems.elevator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.SparkUtil;

public class ElevatorIOSpark implements ElevatorIO {
  private final SparkMax m_one;
  private final SparkMax m_two;
  private final SparkMax m_shoulder;
  private final SparkMaxConfig m_oneConfig;
  private final SparkMaxConfig m_twoConfig;
  private final SparkMaxConfig m_shoulderConfig;
  private final SparkClosedLoopController m_oneController;
  private final SparkClosedLoopController m_ShoulderController;
  private final AbsoluteEncoder shoulderAbsoluteEncoder;
  private final RelativeEncoder shoulderRelativeEncoder;
  private final DigitalInput elevatorlimitSwtich;

  public ElevatorIOSpark() {
    elevatorlimitSwtich = new DigitalInput(1);
    m_one = new SparkMax(ElevatorConstants.kmotorOnePort, MotorType.kBrushless);
    m_two = new SparkMax(ElevatorConstants.kmotorTwoPort, MotorType.kBrushless);
    m_shoulder = new SparkMax(ElevatorConstants.kshoulderPort, MotorType.kBrushless);
    m_oneConfig = new SparkMaxConfig();
    m_twoConfig = new SparkMaxConfig();
    m_shoulderConfig = new SparkMaxConfig();
    m_oneConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .maxOutput(0.45) // Limit speed
        .minOutput(-0.45) // Limit speed
        .positionWrappingEnabled(false)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pidf(
            ElevatorConstants.kElevatorGains[0],
            ElevatorConstants.kElevatorGains[1],
            ElevatorConstants.kElevatorGains[2],
            ElevatorConstants.kElevatorGains[3]);
    m_shoulderConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .maxOutput(0.35) // Limit speed
        .minOutput(-0.35) // Limit speed
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, 1)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pidf(
            ElevatorConstants.kShoulderGains[0],
            ElevatorConstants.kShoulderGains[1],
            ElevatorConstants.kShoulderGains[2],
            ElevatorConstants.kShoulderGains[3]);
    m_twoConfig.apply(m_oneConfig).follow(m_one, ElevatorConstants.motorsInverted);
    m_shoulderConfig.encoder.positionConversionFactor(ElevatorConstants.kShoulderConversionFactor);
    m_shoulderConfig.absoluteEncoder.zeroOffset(ElevatorConstants.kShoulderOffset).inverted(true);

    SparkUtil.tryUntilOk(
        m_one,
        5,
        () ->
            m_one.configure(
                m_oneConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    SparkUtil.tryUntilOk(
        m_two,
        5,
        () ->
            m_two.configure(
                m_twoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkUtil.tryUntilOk(
        m_shoulder,
        5,
        () ->
            m_shoulder.configure(
                m_shoulderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkUtil.tryUntilOk(m_one, 20, () -> m_one.getEncoder().setPosition(0));
    m_oneController = m_one.getClosedLoopController();
    m_ShoulderController = m_shoulder.getClosedLoopController();
    shoulderAbsoluteEncoder = m_shoulder.getAbsoluteEncoder();
    shoulderRelativeEncoder = m_shoulder.getEncoder();
    SparkUtil.tryUntilOk(
        m_shoulder,
        20,
        () -> shoulderRelativeEncoder.setPosition(shoulderAbsoluteEncoder.getPosition()));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.elevatorlimitSwtich = elevatorlimitSwtich.get();

    inputs.shoulderAngle = shoulderAbsoluteEncoder.getPosition();
    inputs.internalAngle = shoulderRelativeEncoder.getPosition();
    // inputs.shoulderAngle = m_shoulder.getEncoder().getPosition(); // make sure this is homed on
    // startup. (Straight up)
    inputs.height = m_one.getEncoder().getPosition();
  }

  @Override
  public void setShoulderAngle(double setpoint) {
    m_ShoulderController.setReference(setpoint, ControlType.kPosition);
  }

  @Override
  public void rotateShoulder(double setpoint) {
    m_shoulder.set(setpoint);
  }

  @Override
  public void setHeight(double setpoint) {
    m_oneController.setReference(setpoint, ControlType.kPosition);
  }

  @Override
  public void resetEncoder() {
    SparkUtil.tryUntilOk(m_one, 20, () -> m_one.getEncoder().setPosition(0));
  }
}
