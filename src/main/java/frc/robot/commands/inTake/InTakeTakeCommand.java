package frc.robot.commands.inTake;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.InTake;

public class InTakeTakeCommand extends Command{
    private final InTake inTake;

    public InTakeTakeCommand(InTake inTake) {
        this.inTake = inTake;
    }

    @Override
    public void initialize() {       
    }

    @Override
    public void execute() {
        inTake.take();
    }

    @Override
    public void end(boolean interrupted) {
        inTake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(inTake);
    }
}
