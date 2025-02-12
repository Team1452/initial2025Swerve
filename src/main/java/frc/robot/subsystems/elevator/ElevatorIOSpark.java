package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.util.SparkUtil;

public class ElevatorIOSpark implements ElevatorIO {
  private final SparkMax m_one;
  private final SparkMax m_two;
  private final SparkMaxConfig m_oneConfig;
  private final SparkMaxConfig m_twoConfig;

  public ElevatorIOSpark() {
    m_one = new SparkMax(ElevatorConstants.kmotorOnePort, MotorType.kBrushless);
    m_two = new SparkMax(ElevatorConstants.kmotorTwoPort, MotorType.kBrushless);
    m_oneConfig = new SparkMaxConfig();
    m_twoConfig = new SparkMaxConfig();
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
  }

  @Override
  public void setMotorOutput(double output) {
    m_one.set(output);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.height = m_one.getEncoder().getPosition();
    inputs.velocity = m_one.getEncoder().getVelocity();
    inputs.atTop = m_one.getEncoder().getPosition() >= ElevatorConstants.kmaxHeight;
    inputs.atBottom = m_one.getEncoder().getPosition() <= ElevatorConstants.kminHeight;
  }
}
