package frc.robot.commands.general.modules.intake;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Intake;

public class IntakeGoCommand extends Command{
    private final Intake intake;

    private boolean shootmod = false;

    public IntakeGoCommand(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {    
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
    }

    @Override
    public boolean isFinished() {
        return intake.intakeLimitSwitch();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(intake);
    }
}