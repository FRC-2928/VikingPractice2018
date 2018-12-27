package org.usfirst.frc.team2928.Autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc.team2928.Command.Chassis.RotateToSetpoint;
import edu.wpi.first.wpilibj.command.Command;

public class SpinnyBoi extends CommandGroup {
    public SpinnyBoi(){

        Command rotateninety = new RotateToSetpoint(90, -1/100, -1/6000, -1/5500);
        Command rotatebackninety = new RotateToSetpoint(-90,-1/100, -1/6000, -1/5500);
        addSequential(rotateninety);
        addSequential(rotatebackninety);
    }

}
