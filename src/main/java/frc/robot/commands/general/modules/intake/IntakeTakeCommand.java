package frc.robot.commands.intake;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Intake;

public class IntakeTakeCommand extends Command{
    private final Intake intake;

    private boolean pieceDetected = false;
    private boolean shootmod = false;

    public IntakeTakeCommand(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {    
        if (pieceDetected) {
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
                pieceDetected = false;
                shootmod = false;
            }
        }
        else {
            if (intake.intakeLimitSwitch()) {
                intake.stop();
                pieceDetected = true;
            } else {
                intake.take();
                pieceDetected = false;
            }
        }

    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(intake);
    }
}
