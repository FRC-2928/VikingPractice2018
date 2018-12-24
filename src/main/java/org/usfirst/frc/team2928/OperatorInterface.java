package org.usfirst.frc.team2928;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import com.ctre.phoenix.*;
import org.usfirst.frc.team2928.Command.Chassis.Shift;
import org.usfirst.frc.team2928.Subsystem.Chassis.Transmission;
import org.usfirst.frc.team2928.Subsystem.Chassis.Drivetrain;


public class    OperatorInterface {

    private static final Joystick driveStick = new Joystick(0);


    private static final JoystickButton gearButton = new JoystickButton(driveStick, 9);

    private JoystickButton brakeButton = new JoystickButton(driveStick, 8);


    OperatorInterface() {


        gearButton.whenPressed(new Shift(Transmission.GearState.LOW));
        gearButton.whenReleased(new Shift(Transmission.GearState.HIGH));


    }

    //We're assuming same drive setup as last year.
    public double getDriveY() {
        return -driveStick.getY();
    }

    public double getDriveX() {
        return driveStick.getX();
    }
}