package frc.robot.subsystems.drivetrains;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import frc.lightning.subsystems.LightningDrivetrain;
import frc.lightning.util.RamseteGains;

public class HowitzerDrivetrain implements LightningDrivetrain {

    @Override
    public void brake() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void coast() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public RamseteGains getConstants() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public DifferentialDriveKinematics getKinematics() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public double getLeftDistance() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public PIDController getLeftPidController() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public double getLeftTemp() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getLeftVelocity() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getLeftVolts() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public Pose2d getPose() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Pose2d getRelativePose() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public double getRightDistance() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public PIDController getRightPidController() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public double getRightTemp() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getRightVelocity() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getRightVolts() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public DifferentialDriveWheelSpeeds getSpeeds() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void initMotorDirections() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void resetDistance() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void resetSensorVals() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setOutput(double arg0, double arg1) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setPower(double arg0, double arg1) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setRamseteOutput(double arg0, double arg1) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setRelativePose() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setVelocity(double arg0, double arg1) {
        // TODO Auto-generated method stub
        
    }
    
}
