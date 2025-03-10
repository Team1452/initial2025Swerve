package frc.robot.subsystems.shoulder;

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
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.util.SparkUtil;

public class ShoulderIOSpark implements ShoulderIO {
  private final SparkMax m_shoulder;
  private final SparkMaxConfig m_shoulderConfig;

  private final SparkClosedLoopController m_ShoulderController;
  private final AbsoluteEncoder shoulderAbsoluteEncoder;
  private final RelativeEncoder shoulderRelativeEncoder;

  public ShoulderIOSpark() {
    m_shoulder = new SparkMax(ElevatorConstants.kshoulderPort, MotorType.kBrushless);
    m_shoulderConfig = new SparkMaxConfig();
    m_shoulderConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .maxOutput(0.35) // Limit speed
        .minOutput(-0.35) // Limit speed
        .positionWrappingEnabled(true)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingInputRange(0, 1)
        .pidf(
            ElevatorConstants.kShoulderGains[0],
            ElevatorConstants.kShoulderGains[1],
            ElevatorConstants.kShoulderGains[2],
            ElevatorConstants.kShoulderGains[3]);
    m_shoulderConfig.encoder.positionConversionFactor(ElevatorConstants.kShoulderConversionFactor);
    m_shoulderConfig.absoluteEncoder.zeroOffset(ElevatorConstants.kShoulderOffset).inverted(true);
    SparkUtil.tryUntilOk(
        m_shoulder,
        5,
        () ->
            m_shoulder.configure(
                m_shoulderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    m_ShoulderController = m_shoulder.getClosedLoopController();
    shoulderAbsoluteEncoder = m_shoulder.getAbsoluteEncoder();
    shoulderRelativeEncoder = m_shoulder.getEncoder();
    SparkUtil.tryUntilOk(
        m_shoulder,
        20,
        () -> shoulderRelativeEncoder.setPosition(shoulderAbsoluteEncoder.getPosition()));
  }

  @Override
  public void updateInputs(ShoulderIOInputs inputs) {
    inputs.shoulderAngle = shoulderAbsoluteEncoder.getPosition();
    inputs.internalAngle = shoulderRelativeEncoder.getPosition();
  }

  @Override
  public void setShoulderAngle(double setpoint) {
    m_ShoulderController.setReference(setpoint, ControlType.kPosition);
  }
}
