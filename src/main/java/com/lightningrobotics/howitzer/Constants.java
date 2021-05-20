package com.lightningrobotics.howitzer;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;

public class Constants {

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
        public static final double MAX_SPEED = 1.0; // this should be in feet/sec //Units.feetToMeters(13.6); // TODO - 10 for now, figure out later - 13.6 feet per second?
        public static final double MAX_ANGULAR_SPEED = 0.25 * Math.PI; //Math.PI; // 1/2 rotation per second
        public static final Translation2d FRONT_LEFT_POS = new Translation2d(Units.inchesToMeters(11.25), Units.inchesToMeters(11.25));
        public static final Translation2d FRONT_RIGHT_POS = new Translation2d(Units.inchesToMeters(11.25), Units.inchesToMeters(-11.25));
        public static final Translation2d BACK_LEFT_POS = new Translation2d(Units.inchesToMeters(-11.25), Units.inchesToMeters(11.25));
        public static final Translation2d BACK_RIGHT_POS = new Translation2d(Units.inchesToMeters(-11.25), Units.inchesToMeters(-11.25));
    }

    public static class JoystickConstants {
        public static final int DRIVER_CONTROLLER = 0;
    }

}
