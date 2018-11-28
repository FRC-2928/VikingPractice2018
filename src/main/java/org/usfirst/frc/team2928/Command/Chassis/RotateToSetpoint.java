package org.usfirst.frc.team2928.Command.Chassis;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team2928.Robot;
import org.usfirst.frc.team2928.RobotConstants;
import org.usfirst.frc.team2928.RobotMap;
/* we need to make the robot turn 90 degrees and angle+90 if x>360  -360
gyro stuff
*/

public class RotateToSetpoint extends Command {

    private double errorSum;
    private double counter;
    private double setpoint; // For the right talon, left talon is -setpoint.
    private boolean motorSafetyBackup = true;
    private double previousVelocity;
    private int decelCounter;
    private boolean hasStartedDecel;
    private PigeonIMU pigeon;
    public RotateToSetpoint(double degrees) {
        // requires(Robot.chassis.drivetrain);
        //this.setpoint = (int)(RobotConstants.DRIVE_TICKS_PER_FOOT * (degrees / 360 * Math.PI * RobotConstants.AXLE_LENGTH_FEET));
        this.setpoint = degrees;
        this.counter = 0;
        this.errorSum = 0;

    }

    @Override
    protected boolean isFinished() {

        if (java.lang.Math.abs(getRotationError()) <= 1.0){

            return true;

        }
        return false;
    }


    public void rotateToAngel(double targetBoi){
        double angle = Robot.chassis.drivetrain.getYaw();
        double error = targetBoi - angle;
        Robot.chassis.drivetrain.setAngleSetpoint(error);
    }

    @Override
    public void initialize(){
        motorSafetyBackup = Robot.chassis.drivetrain.getMotorSafetyEnabled();
        // Safety has to be disabled whenever we use a mode that isn't
        Robot.chassis.drivetrain.setMotorSafetyEnabled(false);
        Robot.chassis.drivetrain.zeroEncoders();
        previousVelocity = -1;
        decelCounter = 0;
        hasStartedDecel = false;
        // We can do this because we disabled the motor safety.
        // Robot.chassis.drivetrain.setTalons(ControlMode.MotionMagic, -setpoint, setpoint);
    }

    @Override
    public void execute(){

        double error = getRotationError();
        double kp = -1.0/80;
        this.errorSum += error;
        double ki = -1.0/20000;
        double pi = kp * error + ki * errorSum;
        Robot.chassis.drivetrain.drive(0 , pi);
        // rotateToAngel(this.setpoint);
        SmartDashboard.putNumber("Error", pi);
        SmartDashboard.putNumber("P",kp * error);
        SmartDashboard.putNumber("I", ki * errorSum);





    }

    public double getRotationError() {
        double currentAngle = Robot.chassis.drivetrain.getYaw();
        return this.setpoint - currentAngle;
    }

    @Override
    public void interrupted(){
        end();
    }

    @Override
    public void end(){
        Robot.chassis.drivetrain.resetTalons();
        Robot.chassis.drivetrain.setMotorSafetyEnabled(motorSafetyBackup);
    }
}
