package com.lightningrobotics.howitzer.commands;

import com.lightningrobotics.howitzer.Constants.DrivetrainConstants;
import com.lightningrobotics.howitzer.subsystems.Drivetrain;
import com.lightningrobotics.howitzer.util.DrivetrainSpeed;
import com.lightningrobotics.howitzer.util.LightningIMU;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        this.filter = new JoystickFilter(0.2d, 0d, 1d, JoystickFilter.Mode.CUBED);
    }

    @Override
    public void execute() {

        // Get filtered joystick input
        var xInput    = filter.filter(-controller.getY(GenericHID.Hand.kLeft));
        var yInput    = filter.filter(+controller.getX(GenericHID.Hand.kLeft));
        var rotInput  = filter.filter(+controller.getX(GenericHID.Hand.kRight));

        // Constrain magnitude vector from joysticks to w/in practical range
        var magnitude = Math.sqrt((xInput * xInput) + (yInput * yInput));
        if(magnitude > 1d) {
            xInput = xInput / magnitude;
            yInput = yInput / magnitude;
        }

        // Scale joystick input to robot speed
        var xSpeed    =  xInput   * DrivetrainConstants.MAX_SPEED;
        var ySpeed    =  yInput   * DrivetrainConstants.MAX_SPEED;
        var rotSpeed  =  rotInput * DrivetrainConstants.MAX_ANGULAR_SPEED;

        // Placeholder for drive speed
        DrivetrainSpeed driveSpeed = null;

        // Convert to field centric if necessary
        if(fieldCentric) {
            final var theta = imu.getHeading();
            driveSpeed = DrivetrainSpeed.fromFieldCentricSpeed(xSpeed, ySpeed, rotSpeed, theta);
        } else {
            driveSpeed = new DrivetrainSpeed(xSpeed, ySpeed, rotSpeed);
        }

        // Set drive speed
        drivetrain.drive(driveSpeed);

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("xInput", xInput);
        SmartDashboard.putNumber("yInput", yInput);
        SmartDashboard.putString("Heading", imu.getHeading().toString());

        //SmartDashboard.putString("Target Speed", driveSpeed.toString());
        //SmartDashboard.putString("Real Speed", drivetrain.getKinematics().forward(drivetrain.getStates()).toString());
        //SmartDashboard.putNumber("Max Linear Speed", DrivetrainConstants.MAX_SPEED);
        //SmartDashboard.putNumber("Max Angular Speed", DrivetrainConstants.MAX_ANGULAR_SPEED);
        //SmartDashboard.putNumber("X Speed", xSpeed);
        //SmartDashboard.putNumber("Y Speed", ySpeed);
        //SmartDashboard.putNumber("ROT Speed", rotSpeed);
        //SmartDashboard.putNumber("X Input", xInput);
        //SmartDashboard.putNumber("Y Input", yInput);
        //SmartDashboard.putNumber("ROT Input", rotInput);
        //SmartDashboard.putNumber("Heading", imu.getHeading().getDegrees());

    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        drivetrain.stop();
    }

}
