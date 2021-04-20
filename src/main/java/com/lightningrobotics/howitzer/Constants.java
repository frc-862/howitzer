package com.lightningrobotics.howitzer;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;

public class Constants {

    public static class RobotMap {
        public static final int PIGEON = 9;
        public static final int FRONT_LEFT_DRIVE_MOTOR = 1;
        public static final int FRONT_LEFT_ANGLE_MOTOR = 2;
        public static final int FRONT_LEFT_CANCODER = 0;
        public static final int FRONT_RIGHT_DRIVE_MOTOR = 3;
        public static final int FRONT_RIGHT_ANGLE_MOTOR = 4;
        public static final int FRONT_RIGHT_CANCODER = 1;
        public static final int BACK_LEFT_DRIVE_MOTOR = 5;
        public static final int BACK_LEFT_ANGLE_MOTOR = 6;
        public static final int BACK_LEFT_CANCODER = 2;
        public static final int BACK_RIGHT_DRIVE_MOTOR = 7;
        public static final int BACK_RIGHT_ANGLE_MOTOR = 8;
        public static final int BACK_RIGHT_CANCODER = 3;
    }

    public static class ModuleConstants {
        public static final double DRIVE_P = 15.0;
        public static final double DRIVE_I = 0.01;
        public static final double DRIVE_D = 0.1;
        public static final double DRIVE_F = 0.2;
        public static final double ANGLE_P = 1.0;
        public static final double ANGLE_I = 0.0;
        public static final double ANGLE_D = 0.0;
        public static final double TICKS_PER_REV_CANCODER = 4096; // CANCoder has 4096 ticks/rotation
    }

    public static class DrivetrainConstants {
        public static final double MAX_SPEED = Units.feetToMeters(13.6); // 13.6 feet per second
        public static final double MAX_ANGULAR_SPEED = Math.PI; // 1/2 rotation per second
        public static final Translation2d FRONT_LEFT_POS = new Translation2d(Units.inchesToMeters(10), Units.inchesToMeters(10));
        public static final Translation2d FRONT_RIGHT_POS = new Translation2d(Units.inchesToMeters(10), Units.inchesToMeters(-10));
        public static final Translation2d BACK_LEFT_POS = new Translation2d(Units.inchesToMeters(-10), Units.inchesToMeters(10));
        public static final Translation2d BACK_RIGHT_POS = new Translation2d(Units.inchesToMeters(-10), Units.inchesToMeters(-10));
    }

    public static class JoystickConstants {
        public static final int DRIVER_CONTROLLER = 0;
    }

}
