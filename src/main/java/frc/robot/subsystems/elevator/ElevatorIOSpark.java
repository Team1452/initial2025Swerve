package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.util.SparkUtil;

public class ElevatorIOSpark implements ElevatorIO {
  private final SparkMax m_one;
  private final SparkMax m_two;
  private final SparkMax m_shoulder;
  private final SparkMaxConfig m_oneConfig;
  private final SparkMaxConfig m_twoConfig;
  private final SparkMaxConfig m_shoulderConfig;
  private final SparkClosedLoopController m_UDController;
  private final SparkClosedLoopController m_ShoulderController;

  public ElevatorIOSpark() {
    m_one = new SparkMax(ElevatorConstants.kmotorOnePort, MotorType.kBrushless);
    m_two = new SparkMax(ElevatorConstants.kmotorTwoPort, MotorType.kBrushless);
    m_shoulder = new SparkMax(ElevatorConstants.kshoulderPort, MotorType.kBrushless);
    m_oneConfig = new SparkMaxConfig();
    m_twoConfig = new SparkMaxConfig();
    m_shoulderConfig = new SparkMaxConfig();
    m_twoConfig.follow(m_one, ElevatorConstants.motorsInverted);

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
    m_UDController = m_one.getClosedLoopController();
    m_ShoulderController = m_shoulder.getClosedLoopController();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.shoulderPos = m_shoulder.getEncoder().getPosition();
    inputs.height = m_one.getEncoder().getPosition();
    inputs.velocity = m_one.getEncoder().getVelocity();
  }

  @Override
  public void setShoulderPosition(double setpoint) {
    m_ShoulderController.setReference(setpoint, ControlType.kPosition);
  }

  @Override
  public void setHeight(double setpoint) {
    m_UDController.setReference(setpoint, ControlType.kPosition);
  }
}
