package frc.robot.commands.general.modules.ballintake;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.subsystems.BallIntake;

public class ballIntakeTakeCommand  extends Command {
    private final BallIntake ballIntake;

    

    public ballIntakeTakeCommand (BallIntake ballIntake) {
        this.ballIntake = ballIntake;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        ballIntake.take();
    }
    @Override
    public void end(boolean interrupted) {
        ballIntake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(ballIntake);
    }
}
