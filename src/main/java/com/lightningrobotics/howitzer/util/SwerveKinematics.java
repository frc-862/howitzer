package com.lightningrobotics.howitzer.util;

import com.lightningrobotics.howitzer.Constants.DrivetrainConstants;
import com.lightningrobotics.howitzer.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class SwerveKinematics {

    private static class Wheelbase {
        public final double W; // Width
        public final double L; // Length
        public final double R; // Diagonal
        Wheelbase(double width, double length) {
            this.W = width;
            this.L = length;
            this.R = Math.sqrt((W * W) + (L * L));
        }
    }

    private final Wheelbase wb;

    private SwerveModuleState[] states;

    public SwerveKinematics(double width, double length) {
        this.wb = new Wheelbase(width, length);
        this.states = new SwerveModuleState[DrivetrainConstants.NUM_MODULES];
    }
    
    public SwerveModuleState[] inverse(DrivetrainSpeed speed) {

        var a = speed.vy - speed.omega * (wb.L / wb.R);
        var b = speed.vy + speed.omega * (wb.L / wb.R);
        var c = speed.vx - speed.omega * (wb.W / wb.R);
        var d = speed.vx + speed.omega * (wb.W / wb.R);

        states[Drivetrain.Modules.FRONT_RIGHT.getIdx()]  = new SwerveModuleState(Math.sqrt((b*b)+(c*c)), new Rotation2d(Math.atan2(b,c)));
        states[Drivetrain.Modules.FRONT_LEFT.getIdx()]   = new SwerveModuleState(Math.sqrt((b*b)+(d*d)), new Rotation2d(Math.atan2(b,d)));
        states[Drivetrain.Modules.BACK_RIGHT.getIdx()]   = new SwerveModuleState(Math.sqrt((a*a)+(c*c)), new Rotation2d(Math.atan2(a,c)));
        states[Drivetrain.Modules.BACK_LEFT.getIdx()]    = new SwerveModuleState(Math.sqrt((a*a)+(d*d)), new Rotation2d(Math.atan2(a,d)));

        // Normalize if necessary
        var max = UtilMath.max(states[Drivetrain.Modules.FRONT_RIGHT.getIdx()].velocity,
                                states[Drivetrain.Modules.FRONT_LEFT.getIdx()].velocity,
                                states[Drivetrain.Modules.BACK_RIGHT.getIdx()].velocity,
                                states[Drivetrain.Modules.BACK_LEFT.getIdx()].velocity);
        
        if(max > DrivetrainConstants.MAX_SPEED) {
            states[Drivetrain.Modules.FRONT_RIGHT.getIdx()].velocity /= max;
            states[Drivetrain.Modules.FRONT_LEFT.getIdx()].velocity /= max;
            states[Drivetrain.Modules.BACK_RIGHT.getIdx()].velocity /= max;
            states[Drivetrain.Modules.BACK_LEFT.getIdx()].velocity /= max;
        }

        return states;

    }

    public DrivetrainSpeed forward(Rotation2d theta) {

        // TODO this is a lot of math that should probably be reviewed
        var FL = states[Drivetrain.Modules.FRONT_LEFT.getIdx()];
        var FR = states[Drivetrain.Modules.FRONT_RIGHT.getIdx()];
        var BL = states[Drivetrain.Modules.BACK_LEFT.getIdx()];
        var BR = states[Drivetrain.Modules.BACK_RIGHT.getIdx()];

        var BFL = FL.angle.getSin() * FL.velocity;
        var DFL = FL.angle.getCos() * FL.velocity;
        var BFR = FR.angle.getSin() * FR.velocity;
        var CFR = FR.angle.getCos() * FR.velocity;
        var ARL = BL.angle.getSin() * BL.velocity;
        var DRL = BL.angle.getCos() * BL.velocity;
        var ARR = BR.angle.getSin() * BR.velocity;
        var CRL = BR.angle.getCos() * BR.velocity;

        var A = (ARR + ARL) / 2d;
        var B = (BFL + BFR) / 2d;
        var C = (CFR + CRL) / 2d;
        var D = (DFL + DRL) / 2d;
        
        var ROT1 = (B - A) / wb.L;
        var ROT2 = (C - D) / wb.W;
        var ROT = (ROT1 + ROT2) / 2d;

        var FWD1 = ROT * (wb.L / 2d) + A;
        var FWD2 = -ROT * (wb.L / 2d) + B;
        var FWD = (FWD1 + FWD2) / 2d;

        var STR1 = ROT * (wb.W / 2d) + C;
        var STR2 = -ROT * (wb.W / 2d) + D;
        var STR = (STR1 + STR2) / 2d;

        var speed = DrivetrainSpeed.fromFieldCentricSpeed(FWD, STR, ROT, theta);

        return speed;

    }

    public SwerveModuleState[] getStates() {
        return states;
    }

}
