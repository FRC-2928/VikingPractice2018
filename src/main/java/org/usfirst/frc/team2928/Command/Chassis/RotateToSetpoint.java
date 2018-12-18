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
    private double previousError = 0;
    public RotateToSetpoint(double degrees) {
        // requires(Robot.chassis.drivetrain);
        //this.setpoint = (int)(RobotConstants.DRIVE_TICKS_PER_FOOT * (degrees / 360 * Math.PI * RobotConstants.AXLE_LENGTH_FEET));
        this.setpoint = degrees;
        this.counter = 0;
        this.errorSum = 0;

    }

    @Override
    protected boolean isFinished() {

        if (java.lang.Math.abs(getRotationError()) <= 1.0/4){

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

        // We're using the gyro to set a setpoint and then rotate to it.

        double error = getRotationError();
        double kp = -1.0/105;
        this.errorSum += error;
        double derivative = (error - this.previousError)/0.02;
        double ki = -1.0/6000;
        double kd = -1.0/5500;
        double pid = (kp * error) + (ki * errorSum) + (kd * derivative);
        //Check out FRC Programming done right for a good explaination on PID
        Robot.chassis.drivetrain.drive(0 , pid);
        // rotateToAngel(this.setpoint);
        SmartDashboard.putNumber("Error", pid);
        SmartDashboard.putNumber("P",kp * error);
        SmartDashboard.putNumber("I", ki * errorSum);
        SmartDashboard.putNumber("D", kd * derivative);
        // Displaying it all in smartdashboard
        SmartDashboard.putNumber("CurrentError", this.previousError);
        SmartDashboard.putNumber("PreviousError", error);

        this.previousError = error;
        // Saving the previous error to use it in the derivative 



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
