package frc.robot.commands.telescopic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class RatchetRight extends CommandBase {
    Climber clim;
    public RatchetRight(Climber c) {
        clim = c;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        clim.rightServo.set(0);
    }
    
}
