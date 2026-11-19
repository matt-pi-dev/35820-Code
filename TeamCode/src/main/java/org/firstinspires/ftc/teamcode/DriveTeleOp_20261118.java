package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "DriveTeleOp_20261118")
public class DriveTeleOp_20261118 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // =====================================================
        // DRIVE MOTORS
        // =====================================================
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor pollenShooter = hardwareMap.dcMotor.get("shooterPollenMotor");
        DcMotor nectarShooter = hardwareMap.dcMotor.get("shooterNectarMotor");
        nectarShooter.setDirection(DcMotorSimple.Direction.REVERSE);


        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotor transportMotor = hardwareMap.dcMotor.get("transportMotor");
        transportMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        /*
         * IMPORTANT:
         * Replace these with your actual odometry pod offsets
         * once you measure them.
         */
        pinpoint.setOffsets(0, 0, DistanceUnit.MM);

        // goBILDA 4-Bar Odometry Pods
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * These may need to be changed depending
         * on how your pods are mounted.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpoint.resetPosAndIMU();

        telemetry.addLine("Field Centric + Pinpoint + Intake Ready!");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        double intakePower = 0.8;
        double transportPower = 1;
        double pollenShooterPower = 0.75;
        double nectarShooterPower = 0.85;

        while (opModeIsActive()) {

            pinpoint.update();

            Pose2D position = pinpoint.getPosition();
            double heading = -position.getHeading(AngleUnit.RADIANS);

            double sense = 0.8;

            // Forward / backward
            //
            // NO negative sign here because this fixes the
            // reversed forward/backward you were experiencing.
            double y =  -gamepad1.left_stick_y * sense;
            // Strafe left / right
            double x =   gamepad1.left_stick_x * sense;
            // Rotate
            double rx = gamepad1.right_stick_x * (sense+0.1);

            // -------------------------------------------------
            // FIELD-CENTRIC CALCULATION
            // -------------------------------------------------
            double rotX =  x * Math.cos(-heading) + y * Math.sin(-heading);
            double rotY = -x * Math.sin(-heading) + y * Math.cos(-heading);

            // Compensate for mecanum strafing
            rotX *= 1.2;

            // -------------------------------------------------
            // MECANUM MOTOR POWERS
            // -------------------------------------------------
            double denominator =
                    Math.max(
                            Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx),
                            1
                    );

            double frontLeftPower =  (rotY + rotX + rx) / denominator;
            double backLeftPower =   (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower =  (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


            if (gamepad1.dpad_right) {
                intakeMotor.setPower(intakePower);
            }

            if (gamepad1.dpad_down) {
                transportMotor.setPower(transportPower);
            }

            if (gamepad1.dpad_up) {
                pollenShooter.setPower(pollenShooterPower);
                nectarShooter.setPower(nectarShooterPower);
            }

            if (gamepad1.options) {
                pinpoint.resetPosAndIMU();
            }


            telemetry.addData("X", "%.1f mm", position.getX(DistanceUnit.MM));
            telemetry.addData("Y", "%.1f mm", position.getY(DistanceUnit.MM));
            telemetry.addData("Heading", "%.1f degrees", position.getHeading(AngleUnit.DEGREES));

            telemetry.addData("FrontLeft Motor Position", "%d", frontLeftMotor.getCurrentPosition());
            telemetry.addData("FrontRight Motor Position", "%d", frontRightMotor.getCurrentPosition());
            telemetry.addData("BackLeft Motor Position", "%d", backLeftMotor.getCurrentPosition());
            telemetry.addData("BackRight Motor Position", "%d", backRightMotor.getCurrentPosition());

            telemetry.addData("Intake Power", "%.1f", intakePower * 100);
            telemetry.addData("Shooter Power", "%.1f", pollenShooterPower * 100);
            telemetry.addData("IntakePwr", intakeMotor.getPower());
            telemetry.addData("ShooterPwr", pollenShooter.getPower());

            telemetry.update();
        }

    }
}


