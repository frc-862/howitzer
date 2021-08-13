package com.lightningrobotics.howitzer.controller;

import com.lightningrobotics.howitzer.util.DrivetrainSpeed;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

public class DriveController {

    private Pose2d poseError = new Pose2d();
    private Pose2d poseTolerance = new Pose2d();
    private boolean enabled = true;

    private final PIDFController xController;
    private final PIDFController yController;
    private final PIDFController thetaController;

    public DriveController(PIDFController xController, PIDFController yController, PIDFController thetaController) {
        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;
    }

    public boolean atReference() {
        final var eTranslate = poseError.getTranslation();
        final var eRotate = poseError.getRotation();
        final var tolTranslate = poseTolerance.getTranslation();
        final var tolRotate = poseTolerance.getRotation();
        return Math.abs(eTranslate.getX()) < tolTranslate.getX() && Math.abs(eTranslate.getY()) < tolTranslate.getY() && Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
    }

    public void setTolerance(Pose2d tolerance) {
        poseTolerance = tolerance;
    }

    public DrivetrainSpeed calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters, Rotation2d angleRef) {
        // Calculate feedforward velocities (field-relative).
        double xFF = linearVelocityRefMeters * poseRef.getRotation().getCos();
        double yFF = linearVelocityRefMeters * poseRef.getRotation().getSin();
        double thetaFF = thetaController.calculate(currentPose.getRotation().getRadians(), angleRef.getRadians());

        poseError = poseRef.relativeTo(currentPose);

        if (!enabled) {
            return DrivetrainSpeed.fromFieldCentricSpeed(xFF, yFF, thetaFF, currentPose.getRotation());
        }

        // Calculate feedback velocities (based on position error).
        double xFeedback = xController.calculate(currentPose.getX(), poseRef.getX());
        double yFeedback = yController.calculate(currentPose.getY(), poseRef.getY());

        // Return next output.
        return DrivetrainSpeed.fromFieldCentricSpeed(xFF + xFeedback, yFF + yFeedback, thetaFF, currentPose.getRotation());
    }

    public DrivetrainSpeed calculate(Pose2d currentPose, Trajectory.State desiredState, Rotation2d angleRef) {
        return calculate(currentPose, desiredState.poseMeters, desiredState.velocityMetersPerSecond, angleRef);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }
}
