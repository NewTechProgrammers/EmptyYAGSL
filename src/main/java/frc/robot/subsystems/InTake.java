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

public class InTake extends SubsystemBase {
    private final SparkMax inTakeMotor = new SparkMax(MechanismConstants.kInTakeSparkMaxPort, MotorType.kBrushless);
    private final SparkMaxConfig inTakeMotorConfig = new SparkMaxConfig();
    private final SparkClosedLoopController inTakeMotorPIDController = inTakeMotor.getClosedLoopController();


    public InTake() {
        inTakeMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake);
        inTakeMotorConfig.encoder
                .positionConversionFactor(MechanismConstants.kInTakeConversionFactor)
                .velocityConversionFactor(1.0);

                inTakeMotor.configure(inTakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

                inTakeMotor.getEncoder().setPosition(0);
    }




    public void take() {
        inTakeMotor.set(MechanismConstants.kMaxInTakeSpeed);
    }



    public void stop() {
        inTakeMotor.set(0);
    }

    public double getPosition() {
        return inTakeMotor.getEncoder().getPosition();
    }
}
