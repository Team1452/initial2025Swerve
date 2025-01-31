package frc.robot.subsystems.intake;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.util.SparkUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final SparkMax m_intake = new SparkMax(IntakeConstants.IntakeID, MotorType.kBrushless);
    
    public Intake() {

        SparkMaxConfig sparkConfig = new SparkMaxConfig();
        SparkUtil.tryUntilOk(
        m_intake,
        5,
        () ->
            m_intake.configure(
                sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

                
    }

    public void run() {
        m_intake.set(1);
    }

    public void runVelocity(double speed) {
        m_intake.set(speed);
    }

    public void stop() {
        m_intake.set(0);
    }
}
