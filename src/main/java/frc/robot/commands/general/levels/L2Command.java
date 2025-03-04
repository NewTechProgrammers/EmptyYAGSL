package frc.robot.commands.general.levels;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.general.modules.elevator.ElevatorToPositionCommand;
import frc.robot.commands.general.modules.elevator.ElevatorToPositionWPILIBCommand;
import frc.robot.commands.general.modules.elevatorInternal.InternalElevatorToPositionCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.InternalElevator;

public class L2Command extends ParallelCommandGroup {
    private final Elevator elevator;
    private final InternalElevator internalElevator;
    
    public L2Command(Elevator elevator, InternalElevator internalElevator) {
        this.elevator = elevator; this.internalElevator = internalElevator;
        addCommands(new ElevatorToPositionWPILIBCommand(this.elevator, 0), new InternalElevatorToPositionCommand(this.internalElevator, 120));
    }
}
