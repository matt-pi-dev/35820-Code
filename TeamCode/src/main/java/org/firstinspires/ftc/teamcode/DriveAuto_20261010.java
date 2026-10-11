/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Drive Forward", group="Robot")

public class DriveAuto_20261010 extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftDrivefront   = null;
    private DcMotor         rightDrivefront  = null;

    private DcMotor         leftDriveback   = null;

    private DcMotor           rightDriveback = null;

    private DcMotor         intakeMotor = null;

    private ElapsedTime     runtime = new ElapsedTime();

    GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.09449 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.2;
    static final double     INTAKE_SPEED              = DRIVE_SPEED*2;
    double Kp = 0.1;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        pinpoint.resetPosAndIMU();

         leftDrivefront =
                hardwareMap.dcMotor.get("frontLeftMotor");


         leftDriveback =
                hardwareMap.dcMotor.get("backLeftMotor");


         rightDrivefront =
                hardwareMap.dcMotor.get("frontRightMotor");


         rightDriveback =
                hardwareMap.dcMotor.get("backRightMotor");


         intakeMotor =
                hardwareMap.dcMotor.get("intakeMotor");


        intakeMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDriveback.setDirection(DcMotor.Direction.REVERSE);
        leftDrivefront.setDirection(DcMotor.Direction.REVERSE);

        leftDrivefront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrivefront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrivefront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrivefront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // Wait for the game to start (driver presses START)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  12,  12, 10);  // S1: Forward 47 Inches with 5 Sec timeout


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
        int newleftFrontTarget;
        int newRightBackTarget;
        int newRightFrontTarget;
        double targetheading = pinpoint.getHeading(AngleUnit.DEGREES);

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = leftDriveback.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newleftFrontTarget = leftDrivefront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightBackTarget = rightDriveback.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightDrivefront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            leftDriveback.setTargetPosition(newLeftBackTarget) ;
            leftDrivefront.setTargetPosition(newleftFrontTarget) ;
            rightDriveback.setTargetPosition(newRightBackTarget) ;
            rightDrivefront.setTargetPosition(newRightFrontTarget);


            // Turn On RUN_TO_POSITION


            leftDriveback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDrivefront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDriveback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrivefront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            leftDriveback.setPower(Math.abs(speed));
            leftDrivefront.setPower(Math.abs(speed));
            rightDriveback.setPower(Math.abs(speed));
            rightDrivefront.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDrivefront.isBusy() && rightDrivefront.isBusy())) {
                double error  = targetheading - pinpoint.getHeading(AngleUnit.DEGREES);
                double output = error*Kp;
                leftDriveback.setPower(Math.abs(speed)-output);
                leftDrivefront.setPower(Math.abs(speed)-output);

                rightDriveback.setPower(Math.abs(speed)+output);
                rightDrivefront.setPower(Math.abs(speed)+output);

            }

            // Stop all motion;
            leftDriveback.setPower(0);
            leftDrivefront.setPower(0);
            rightDriveback.setPower(0);
            rightDrivefront.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDriveback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDrivefront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDriveback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrivefront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }


public void PIDDrive(double targetPosition) {
    double targetheading = pinpoint.getHeading(AngleUnit.DEGREES);
    targetPosition *= COUNTS_PER_INCH;

    double leftPosition = 0 ;
    double rightPosition = 0;
    double robotPosition = 0;
    leftDrivefront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightDrivefront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftDriveback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightDriveback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    leftDrivefront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightDrivefront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftDriveback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightDriveback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);






    if (opModeIsActive()) {

        // Determine new target position, and pass to motor controller

        runtime.reset();


        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() &&
                robotPosition>=targetPosition) {

            leftPosition = leftDrivefront.getCurrentPosition();
            rightPosition = rightDrivefront.getCurrentPosition();
            robotPosition = (leftPosition+rightPosition)/2.0;

            double error  = targetheading - pinpoint.getHeading(AngleUnit.DEGREES);
            double output = error*Kp;


            double leftError = targetPosition - leftPosition;
            double rightError = targetPosition -rightPosition;
            double leftSpeed = leftError * Kp;
            double rightSpeed = rightError  *Kp;


            leftDriveback.setPower(leftSpeed-output);
            leftDrivefront.setPower(leftSpeed-output);

            rightDriveback.setPower(rightSpeed+output);
            rightDrivefront.setPower(rightSpeed+output);

        }

        // Stop all motion;
        leftDriveback.setPower(0);
        leftDrivefront.setPower(0);
        rightDriveback.setPower(0);
        rightDrivefront.setPower(0);



        sleep(250);   // optional pause after each move.
    }
}
}
