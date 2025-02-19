package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final SparkMax intakeMotor = new SparkMax(18, MotorType.kBrushless);
    private final SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    private final SparkClosedLoopController intakeMotorPIDController = intakeMotor.getClosedLoopController();

    DigitalInput topLimitSwitch = new DigitalInput(9);
    DigitalInput bottomLimitSwitch = new DigitalInput(8);

    public Elevator() {
        intakeMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake);
        intakeMotorConfig.encoder
                .positionConversionFactor(2.0)
                .velocityConversionFactor(2.0);
        intakeMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.001, 0.0, 0.0);

        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakeMotor.getEncoder().setPosition(0);

        SmartDashboard.putBoolean("Elevator", false);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("topLimitSwitch", isTopLimitSwitchPressed());
        SmartDashboard.putBoolean("bottomLimitSwitch", isBottomLimitSwitchPressed());
    }

    public boolean isTopLimitSwitchPressed() {
        return !topLimitSwitch.get();
    }

    public boolean isBottomLimitSwitchPressed() {
        return !bottomLimitSwitch.get();
    }

    public void runUp() {
        if (isTopLimitSwitchPressed()) {
            intakeMotor.set(0);
        } else {
            intakeMotor.set(0.15);

        }
    }

    public void runDown() {
        if (isBottomLimitSwitchPressed()) {
            intakeMotor.set(0);
        } else {
            intakeMotor.set(-0.15);
        }

    }

    public void stop() {
        intakeMotor.set(0);
    }

}
