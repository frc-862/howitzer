package com.lightningrobotics.howitzer.commands;

import com.lightningrobotics.howitzer.Constants.DrivetrainConstants;
import com.lightningrobotics.howitzer.subsystems.Drivetrain;
import com.lightningrobotics.howitzer.util.DrivetrainSpeed;
import com.lightningrobotics.howitzer.util.LightningIMU;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lightning.util.JoystickFilter;

public class SwerveDriveCommand extends CommandBase {
    
    private final Drivetrain drivetrain;
    private final LightningIMU imu;
    private final XboxController controller;
    private final JoystickFilter filter;
    private final boolean fieldCentric;

    public SwerveDriveCommand(Drivetrain drivetrain, LightningIMU imu, XboxController controller, boolean fieldCentric) {
        this.drivetrain = drivetrain;
        this.imu = imu;
        addRequirements(drivetrain); // we do not add IMU as a requirement because it's use is read-only
        this.controller = controller;
        this.fieldCentric = fieldCentric;
        this.filter = new JoystickFilter(0.05, -1d, 1d, JoystickFilter.Mode.CUBED);
    }

    @Override
    public void execute() {

        // Get filtered joystick input
        final var xInput    = filter.filter(-controller.getY( GenericHID.Hand.kLeft  ));
        final var yInput    = filter.filter(-controller.getX( GenericHID.Hand.kLeft  ));
        final var rotInput  = filter.filter(-controller.getX( GenericHID.Hand.kRight )); 

        // Scale joystick input to robot speed
        var xSpeed    =  xInput   * DrivetrainConstants.MAX_SPEED;
        var ySpeed    =  yInput   * DrivetrainConstants.MAX_SPEED;
        var rotSpeed  =  rotInput * DrivetrainConstants.MAX_ANGULAR_SPEED;

        // Placeholder for drive speed
        DrivetrainSpeed driveSpeed;

        // Convert to field centric if necessary
        if(fieldCentric) {
            final var theta = imu.getHeading();
            driveSpeed = DrivetrainSpeed.fromFieldCentricSpeed(xSpeed, ySpeed, rotSpeed, theta);
        } else {
            driveSpeed = new DrivetrainSpeed(xSpeed, ySpeed, rotSpeed);
        }
        
        // Set drive speed
        drivetrain.drive(driveSpeed);
    
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        drivetrain.stop();
    }

}
