package org.usfirst.frc.team2928.Autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc.team2928.Command.Chassis.RotateNinety;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2928.Subsystem.Chassis.Drivetrain;

public class SpinnyBoi extends CommandGroup {
    public SpinnyBoi(){
        
        Command rotateninety = new RotateNinety(90);
        addSequential(rotateninety);
    }

}
