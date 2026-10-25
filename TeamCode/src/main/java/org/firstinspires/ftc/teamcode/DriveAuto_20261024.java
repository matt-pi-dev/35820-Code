package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Autonomous(name="DriveAuto_20261024")

public class DriveAuto_20261024 extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor frontLeftMotor   = null;
    private DcMotor         frontRightMotor  = null;

    private DcMotor         backLeftMotor   = null;

    private DcMotor           backRightMotor = null;

    private DcMotor         intakeMotor = null;

    private ElapsedTime     runtime = new ElapsedTime();


    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.09449;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.2;
    static final double INTAKE_SPEED = DRIVE_SPEED * 2;


    @Override
    public void runOpMode() throws InterruptedException {


        // =====================================================
        // DRIVE MOTORS
        // =====================================================


        frontLeftMotor =
                hardwareMap.dcMotor.get("frontLeftMotor");


        backLeftMotor =
                hardwareMap.dcMotor.get("backLeftMotor");


        frontRightMotor =
                hardwareMap.dcMotor.get("frontRightMotor");


        backRightMotor =
                hardwareMap.dcMotor.get("backRightMotor");


        intakeMotor =
                hardwareMap.dcMotor.get("intakeMotor");


        intakeMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED, 90, 40, 10);  // S1: Forward 47 Inches with 5 Sec timeout


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftBackTarget;
        int newLeftFrontTarget;
        int newRightBackTarget;
        int newRightFrontTarget;
        double intakePower = 0.8;

        if (opModeIsActive()) {
            intakeMotor.setPower(intakePower);

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = backLeftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newLeftFrontTarget = frontLeftMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newRightBackTarget = backRightMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = frontRightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            backLeftMotor.setTargetPosition(newLeftBackTarget);
            frontLeftMotor.setTargetPosition(newLeftFrontTarget);
            backRightMotor.setTargetPosition(newRightBackTarget);
            frontRightMotor.setTargetPosition(newRightFrontTarget);


            // Turn On RUN_TO_POSITION


            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            backLeftMotor.setPower(Math.abs(speed));
            frontLeftMotor.setPower(Math.abs(speed));
            backRightMotor.setPower(Math.abs(speed));
            frontRightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeftMotor.isBusy() && frontRightMotor.isBusy())) {
            }

            // Stop all motion;
            backLeftMotor.setPower(0);
            frontLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            frontRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
}