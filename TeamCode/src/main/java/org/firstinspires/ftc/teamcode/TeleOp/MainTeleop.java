package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@TeleOp(name = "Main TeleOp")
public class MainTeleop extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    SparkFunOTOS sparkfunOTOS;

    private double targetHeading = 0.0;
    private final double P_GAIN = 0.03;

    @Override
    public void init() {
        drive.init(hardwareMap);

        sparkfunOTOS = hardwareMap.get(SparkFunOTOS.class, "otos");
        configureOTOS();

    }

    private void configureOTOS() {
        sparkfunOTOS.setLinearUnit(DistanceUnit.INCH);
        sparkfunOTOS.setAngularUnit(AngleUnit.DEGREES);
        sparkfunOTOS.setOffset(new SparkFunOTOS.Pose2D(0, 0, 0));
        sparkfunOTOS.setLinearScalar(1.0);
        sparkfunOTOS.setAngularScalar(1.0);
        sparkfunOTOS.resetTracking();
        sparkfunOTOS.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
        sparkfunOTOS.calibrateImu(255, false);
    }

    private void driveFieldRelative(double forward, double right, double rotate) {
        SparkFunOTOS.Pose2D pos = sparkfunOTOS.getPosition();
        double robotAngle = Math.toRadians(pos.h);
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        theta = AngleUnit.normalizeRadians(theta - robotAngle);

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        double headingCorrection = getHeadingCorrection(pos.h);
        rotate += headingCorrection;

        drive.drive(newForward, newRight, rotate);
    }

    private double getHeadingCorrection(double currentHeading) {
        double error = AngleUnit.normalizeDegrees(targetHeading - currentHeading);
        return error * P_GAIN;
    }

    @Override
    public void loop() {
        SparkFunOTOS.Pose2D pos = sparkfunOTOS.getPosition();
        telemetry.addData("X (inch)", pos. x);
        telemetry.addData("Y (inch)", pos.y);
        telemetry.addData("Heading (degrees)", pos.h);
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        if (gamepad1.right_bumper) {
            targetHeading = pos.h;
        }

        driveFieldRelative(forward, right, rotate);
    }
}
