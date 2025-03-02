package frc.robot.commands.general.levels;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.general.modules.elevator.ElevatorToPositionCommand;
import frc.robot.commands.general.modules.elevatorInternal.InternalElevatorToPositionCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.InternalElevator;

public class L3Command extends ParallelCommandGroup {
    private final Elevator elevator;
    private final InternalElevator internalElevator;
    
    public L3Command(Elevator elevator, InternalElevator internalElevator) {
        this.elevator = elevator; this.internalElevator = internalElevator;
        addCommands(new ElevatorToPositionCommand(this.elevator, 115), new InternalElevatorToPositionCommand(this.internalElevator, 392));
    }
}
