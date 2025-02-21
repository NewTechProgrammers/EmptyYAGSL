package frc.robot.commands.lift;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;

import frc.robot.subsystems.Lift;

public class RaiseLiftCommand extends Command {
    private final Lift lift;

    public RaiseLiftCommand(Lift lift) {
        this.lift = lift;
    }

    @Override
    public void initialize() {       
    }

    @Override
    public void execute() {
        lift.runUp();
    }

    @Override
    public void end(boolean interrupted) {
        lift.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(lift);
    }
    
}
