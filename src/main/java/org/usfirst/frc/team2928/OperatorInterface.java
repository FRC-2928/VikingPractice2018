package org.usfirst.frc.team2928;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import org.usfirst.frc.team2928.Command.Chassis.Shift;
import org.usfirst.frc.team2928.Subsystem.Chassis.Transmission;
import org.usfirst.frc.team2928.Autonomous.SpinnyBoi;
import org.usfirst.frc.team2928.Command.Chassis.RotateToSetpoint;

/*
Naming conventions are different in this file. This is so that drivers can make changes here if need be; their names for
things don't match up with ours.
frontplate = Arm.Slider
fourbar = Arm.Shoulder
intake/outtake = Intake.Motors
Gripper = Arm.Grabber
intake open/close = Intake.Clamp
 */
public class    OperatorInterface {

    private static final Joystick driveStick = new Joystick(0);


    private static final JoystickButton gearButton = new JoystickButton(driveStick, 9);




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