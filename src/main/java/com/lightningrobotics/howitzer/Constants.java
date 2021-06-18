// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.lightningrobotics.howitzer;

import com.lightningrobotics.howitzer.controller.PIDFController;

import edu.wpi.first.wpilibj.util.Units;

public final class Constants {

    public static class RobotMap {
        public static final int FRONT_LEFT_DRIVE_MOTOR = 8;
        public static final int FRONT_LEFT_ANGLE_MOTOR = 7;
        public static final int FRONT_LEFT_CANCODER = 16;
        public static final int FRONT_RIGHT_DRIVE_MOTOR = 11;
        public static final int FRONT_RIGHT_ANGLE_MOTOR = 12;
        public static final int FRONT_RIGHT_CANCODER = 17;
        public static final int BACK_LEFT_DRIVE_MOTOR = 10;
        public static final int BACK_LEFT_ANGLE_MOTOR = 9;
        public static final int BACK_LEFT_CANCODER = 15;
        public static final int BACK_RIGHT_DRIVE_MOTOR = 13;
        public static final int BACK_RIGHT_ANGLE_MOTOR = 14;
        public static final int BACK_RIGHT_CANCODER = 18;
        public static final int PIGEON = 19;
    }

    public static class ModuleConstants {

        public static final double DRIVE_P = 0.0;
        public static final double DRIVE_I = 0.0;
        public static final double DRIVE_D = 0.0;
        public static final double DRIVE_F = 0.0;
        public static final PIDFController DRIVE_CONTROLLER = new PIDFController(DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_F);

        public static final double ANGLE_P = 0.0;
        public static final double ANGLE_I = 0.0;
        public static final double ANGLE_D = 0.0;
        public static final double ANGLE_F = 0.0;
        public static final PIDFController ANGLE_CONTROLLER = new PIDFController(ANGLE_P, ANGLE_I, ANGLE_D, ANGLE_F);

        public static final double TICKS_PER_REV_CANCODER = 4096; // CANCoder has 4096 ticks/rotation

    }

    public static class DrivetrainConstants {
        public static final int NUM_MODULES = 4;

        // NOTE that in a perfect world, these two would be the same thing as we would be comfortable driving as fast as possible
        public static final double MAX_SPEED = Units.feetToMeters(5.5); // Max speed we WANT the robot to go
        public static final double REAL_MAX_SPEED = Units.feetToMeters(16.2); // Max speed the robot CAN go

        public static final double MAX_ANGULAR_SPEED = Math.PI; // 1/2 rotation per second, pi rad/sec
        public static final double MAX_ANGULAR_ACCEL = 2 * Math.PI;
    }

    public static class Wheelbase {
        public static final double W = Units.inchesToMeters(22.5); // Width
        public static final double L = Units.inchesToMeters(22.5); // Length
        public static final double R = Math.sqrt((W * W) + (L * L)); // Diagonal
        public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4d) * Math.PI;
        public static final double GEARING = 6.86d;
    }

    public static class JoystickConstants {
        public static final int DRIVER_CONTROLLER = 0;
    }

}
