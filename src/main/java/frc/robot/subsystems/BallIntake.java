package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DigitalInputConstants;
import frc.robot.Constants.MechanismConstants;

public class BallIntake extends SubsystemBase {
    private final SparkMax ballIntakeLeftMotor = new SparkMax(MechanismConstants.kBallIntakeLeftSparkMaxPort, MotorType.kBrushless);
    private final SparkMax ballIntakeRightMotor = new SparkMax(MechanismConstants.kBallIntakeRightSparkMaxPort, MotorType.kBrushless);
    private final PWM ball = new PWM(3);
    private final SparkMaxConfig ballIntakeMotorConfig = new SparkMaxConfig();
    private final SparkClosedLoopController ballIntakeMotorPIDController = ballIntakeLeftMotor.getClosedLoopController();

    public BallIntake() {
        ballIntakeMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake);
                ballIntakeMotorConfig.encoder
                .positionConversionFactor(MechanismConstants.kBallIntakeConversionFactor)
                .velocityConversionFactor(1.0);

                ballIntakeLeftMotor.configure(ballIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

                ballIntakeLeftMotor.getEncoder().setPosition(0);

                ballIntakeRightMotor.configure(ballIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

                ballIntakeRightMotor.getEncoder().setPosition(0);
    }



    public void take() {
        ballIntakeRightMotor.set(-MechanismConstants.kMaxIntakeSpeed);
        ballIntakeLeftMotor.set(MechanismConstants.kMaxIntakeSpeed);
    }

    public void shoot() {
        ballIntakeRightMotor.set(MechanismConstants.kMaxIntakeShootSpeed);
        ballIntakeLeftMotor.set(-MechanismConstants.kMaxIntakeShootSpeed);
    }
    public void stop() {
        ballIntakeLeftMotor.set(0);
        ballIntakeRightMotor.set(0);
    }
}