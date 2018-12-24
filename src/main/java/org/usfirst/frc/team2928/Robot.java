package org.usfirst.frc.team2928;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team2928.Autonomous.*;
import org.usfirst.frc.team2928.Command.Chassis.*;
import org.usfirst.frc.team2928.Command.CommandGroupBuilder;
import org.usfirst.frc.team2928.Subsystem.Chassis.Chassis;

/**
 * Robot for 2018.
 */
@SuppressWarnings("FieldCanBeLocal")
public class Robot extends IterativeRobot {

    private double counter;
    private SendableChooser<Auto> autoSelector;
    private SendableChooser<Field.FieldPosition> startingPositionSelector;
    private Compressor compressor;
    public static Chassis chassis;
    public static OperatorInterface oi;
    Command rotateninety;

    @Override
    public void robotInit() {
        compressor = new Compressor();
        chassis = new Chassis();
        compressor.start();
        oi = new OperatorInterface();
    }

    @Override
    public void teleopInit() {
        Scheduler.getInstance().removeAll();
        chassis.drivetrain.resetTalons();
        chassis.drivetrain.setMotorSafetyEnabled(true);
        chassis.drivetrain.stopProfileDrive();
        new ResetSensors().start();
    }

    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();


    }

    @Override
    public void autonomousInit() {
        Scheduler.getInstance().removeAll();
        chassis.drivetrain.setMotorSafetyEnabled(false);
        new SpinnyBoi().start();



        }



    @Override
    public void autonomousPeriodic() { //Scheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        Scheduler.getInstance().removeAll();
        chassis.drivetrain.stopProfileDrive();
        new ResetSensors().start();
    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }
}
