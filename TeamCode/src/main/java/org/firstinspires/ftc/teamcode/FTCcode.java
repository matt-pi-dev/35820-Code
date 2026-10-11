package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;




@TeleOp(name = "Field Centric + Pinpoint + Intake")
public class FTCcode extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {


        // =====================================================
        // DRIVE MOTORS
        // =====================================================


        DcMotor frontLeftMotor =
                hardwareMap.dcMotor.get("frontLeftMotor");


        DcMotor backLeftMotor =
                hardwareMap.dcMotor.get("backLeftMotor");


        DcMotor frontRightMotor =
                hardwareMap.dcMotor.get("frontRightMotor");


        DcMotor backRightMotor =
                hardwareMap.dcMotor.get("backRightMotor");




        // Keep these directions because your strafe/turn works correctly
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);




        // =====================================================
        // INTAKE MOTOR
        // =====================================================


        DcMotor intakeMotor =
                hardwareMap.dcMotor.get("intakeMotor");


        intakeMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );




        // =====================================================
        // PINPOINT ODOMETRY
        // =====================================================


        GoBildaPinpointDriver pinpoint =
                hardwareMap.get(
                        GoBildaPinpointDriver.class,
                        "pinpoint"
                );




        /*
         * IMPORTANT:
         * Replace these with your actual odometry pod offsets
         * once you measure them.
         */
        pinpoint.setOffsets(
                -14,
                0,
                DistanceUnit.MM
        );




        // goBILDA 4-Bar Odometry Pods
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );




        /*
         * These may need to be changed depending
         * on how your pods are mounted.
         */
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );




        // Reset starting position and heading
        pinpoint.resetPosAndIMU();




        telemetry.addLine(
                "Field Centric + Pinpoint + Intake Ready!"
        );
        telemetry.update();




        // =====================================================
        // WAIT FOR START
        // =====================================================


        waitForStart();


        if (isStopRequested()) {
            return;
        }




        // =====================================================
        // MAIN LOOP
        // =====================================================


        while (opModeIsActive()) {


            // -------------------------------------------------
            // UPDATE PINPOINT
            // -------------------------------------------------


            pinpoint.update();


            Pose2D position =
                    pinpoint.getPosition();


            double heading =
                    position.getHeading(AngleUnit.RADIANS);




            // -------------------------------------------------
            // DRIVE CONTROLS
            // -------------------------------------------------


            double sense = 0.8;




            // Forward / backward
            //
            // NO negative sign here because this fixes the
            // reversed forward/backward you were experiencing.
            double y =
                    gamepad1.left_stick_y * sense;




            // Strafe left / right
            double x =
                    gamepad1.left_stick_x * sense;




            // Rotate
            double rx =
                    gamepad1.right_stick_x * sense;




            // -------------------------------------------------
            // FIELD-CENTRIC CALCULATION
            // -------------------------------------------------


            double rotX =
                    x * Math.cos(-heading)
                            + y * Math.sin(-heading);


            double rotY =
                    -x * Math.sin(-heading)
                            + y * Math.cos(-heading);




            // Compensate for mecanum strafing
            rotX *= 1.2;




            // -------------------------------------------------
            // MECANUM MOTOR POWERS
            // -------------------------------------------------


            double denominator =
                    Math.max(
                            Math.abs(rotY)
                                    + Math.abs(rotX)
                                    + Math.abs(rx),
                            1
                    );




            double frontLeftPower =
                    (rotY + rotX + rx)
                            / denominator;


            double backLeftPower =
                    (rotY - rotX + rx)
                            / denominator;


            double frontRightPower =
                    (rotY - rotX - rx)
                            / denominator;


            double backRightPower =
                    (rotY + rotX - rx)
                            / denominator;




            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);




            // -------------------------------------------------
            // INTAKE CONTROLS
            // -------------------------------------------------


            /*
             * RIGHT TRIGGER = full-speed intake
             * LEFT TRIGGER  = full-speed reverse
             *
             * Trigger only needs to be pressed past 10%.
             */


            double intakePower = 0;




            if (gamepad1.right_trigger > 0.1) {


                // Full speed IN
                intakePower = 1.0;


            } else if (gamepad1.left_trigger > 0.1) {


                // Full speed OUT
                intakePower = -1.0;


            }




            intakeMotor.setPower(intakePower);




            // -------------------------------------------------
            // RESET PINPOINT / FIELD HEADING
            // -------------------------------------------------


            /*
             * Press OPTIONS on Gamepad 1.
             *
             * The direction the robot is facing becomes
             * the new field-forward direction.
             */
            if (gamepad1.options) {
                pinpoint.resetPosAndIMU();
            }




            // -------------------------------------------------
            // TELEMETRY
            // -------------------------------------------------


            telemetry.addData(
                    "X",
                    "%.1f mm",
                    position.getX(DistanceUnit.MM)
            );


            telemetry.addData(
                    "Y",
                    "%.1f mm",
                    position.getY(DistanceUnit.MM)
            );


            telemetry.addData(
                    "Heading",
                    "%.1f degrees",
                    position.getHeading(AngleUnit.DEGREES)
            );


            telemetry.addData(
                    "Intake Power",
                    "%.1f",
                    intakePower
            );


            telemetry.update();
        }
    }
}

