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

    private double setpoint; // For the right talon, left talon is -setpoint.
    private double kp;
    private double ki;
    private double kd;
    private double errorSum;
    private double previousError;
    private boolean motorSafetyBackup = true;
    private double previousVelocity;
    private int decelCounter;
    private boolean hasStartedDecel;
    private PigeonIMU pigeon;
    public RotateToSetpoint(double degrees, double kp, double ki, double kd) {
        requires(Robot.chassis.drivetrain);
        //this.setpoint = (int)(RobotConstants.DRIVE_TICKS_PER_FOOT * (degrees / 360 * Math.PI * RobotConstants.AXLE_LENGTH_FEET));
        this.setpoint = degrees;
        this.kp = 0;
        this.ki = 0;
        this.kd = 0;
        this.errorSum = 0;
        this.previousError = 0;

    }

    @Override
    protected boolean isFinished() {
        return hasStartedDecel && Robot.chassis.drivetrain.getAverageVelocityMagnitude() < RobotConstants.TALON_CRUISE_VELOCITY * 0.02;
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


        double currentAngle = Robot.chassis.drivetrain.getYaw();
        double error = this.setpoint - currentAngle;
        this.errorSum += error;
        double derivative = (error - this.previousError)/0.02;
        double pid = (kp * error) + (ki * errorSum) + (kd * derivative);
        Robot.chassis.drivetrain.drive(0 , pid);


        this.previousError = error;
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
