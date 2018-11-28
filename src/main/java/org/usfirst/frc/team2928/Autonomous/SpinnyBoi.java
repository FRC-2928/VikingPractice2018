package org.usfirst.frc.team2928.Autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team2928.Command.Chassis.RotateToSetpoint;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2928.Subsystem.Chassis.Drivetrain;

public class SpinnyBoi extends CommandGroup {

    public SpinnyBoi(){


        Command rotateninety = new RotateToSetpoint(90);

        SmartDashboard.putData("RotateNinety", rotateninety);
    }

}
