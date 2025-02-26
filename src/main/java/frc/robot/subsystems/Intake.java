package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DigitalInputConstants;
import frc.robot.Constants.MechanismConstants;

public class Intake extends SubsystemBase {
    private final SparkMax intakeMotor = new SparkMax(MechanismConstants.kIntakeSparkMaxPort, MotorType.kBrushless);
    private final SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    private final SparkClosedLoopController intakeMotorPIDController = intakeMotor.getClosedLoopController();


    public Intake() {
        intakeMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake);
            intakeMotorConfig.encoder
                .positionConversionFactor(MechanismConstants.kIntakeConversionFactor)
                .velocityConversionFactor(1.0);

                intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

                intakeMotor.getEncoder().setPosition(0);
    }




    public void take() {
        intakeMotor.set(MechanismConstants.kMaxIntakeSpeed);
    }



    public void stop() {
        intakeMotor.set(0);
    }

    public double getPosition() {
        return intakeMotor.getEncoder().getPosition();
    }
}