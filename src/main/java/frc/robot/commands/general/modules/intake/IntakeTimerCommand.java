package frc.robot.commands.general.modules.intake;

import java.util.Set;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Intake;

public class IntakeTimerCommand extends Command{
    private final Intake intake;
    private final Timer timer = new Timer();
    private final double duration;  

    private boolean shootmod = false;

    public IntakeTimerCommand(Intake intake, double durationSeconds) {
        this.intake = intake;
        this.duration = durationSeconds;
    }

    @Override
    public void initialize() { 
        timer.reset(); 
        timer.start();  
        if (intake.intakeLimitSwitch()) {
            shootmod = true;
        } else {
            shootmod = false;
        }
    }

    @Override
    public void execute() {
        if (shootmod) {
            intake.shoot();
            if (!intake.intakeLimitSwitch()) {
                shootmod = false;
            }
        }
        else {
            if (intake.intakeLimitSwitch()) {
                intake.stop();
            } else {
                intake.take();
            }
        }

    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(intake);
    }
}
