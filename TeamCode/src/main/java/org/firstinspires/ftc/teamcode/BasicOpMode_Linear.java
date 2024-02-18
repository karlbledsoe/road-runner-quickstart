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

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
//
@TeleOp(name = "Basic: Linear OpMode", group = "Linear OpMode")
@Config
//@Disabled
public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.

    GamepadEx rightTrigger1 = new GamepadEx();
    GamepadEx leftTrigger1 = new GamepadEx();

    GamepadEx rightBumper1 = new GamepadEx();
    GamepadEx leftBumper1 = new GamepadEx();

    public static double ssStartPosition = 0.22;
    public static double ssScorePosition = 0.8;
    public static int slidesUp = 100;
    public static int slidesDown = 0;

    public static int planeOn = 1;
    public static double intakePower = 0.8;
    public static double startingPosPlane = 0;
    public static double firePlane = 1;
    boolean planeFire = false;
    boolean backFront = false;
    boolean intakeChanged = false;
    boolean frontSide = true;
    int reversed = 1;
    int front = 0;


    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        MecanumDrive robot = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        /* double forward = gamepad1.left_stick_y;
        double strafe = ;
        double turn = ; */


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
//        robot.leftSlide.setPower(Math.abs(2));
//        robot.rightSlide.setPower(Math.abs(2));
        robot.leftSlide.setTargetPosition(slidesDown);
        robot.rightSlide.setTargetPosition(slidesDown);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            rightTrigger1.updateButton(gamepad1.right_trigger >= 0);
            leftTrigger1.updateButton(gamepad1.right_trigger >= 0);
            rightBumper1.updateButton(gamepad1.right_bumper);
            leftBumper1.updateButton(gamepad1.left_bumper);


            if (gamepad1.right_trigger > 0.1 && reversed != 1 && !intakeChanged) {
                reversed = 1;
                robot.intake.setPower(intakePower);
                intakeChanged = true;
            } else if (gamepad1.right_trigger > 0.1 && reversed == 1 && !intakeChanged) {
                reversed = 0;
                robot.intake.setPower(0);
                intakeChanged = true;
            } else if (gamepad1.left_trigger > 0.1 && reversed != 2 && !intakeChanged) {
                reversed = 2;
                robot.intake.setPower(-1);
                intakeChanged = true;
            } else if (gamepad1.left_trigger > 0.1 && reversed == 2 && !intakeChanged) {
                reversed = 0;
                robot.intake.setPower(0);
                intakeChanged = true;
            }
            if (gamepad1.right_trigger < 0.1 && gamepad1.left_trigger<0.1) {
                intakeChanged = false;
            }


            if (rightBumper1.isPressed()) {
                frontSide=!frontSide;
            }
            if (frontSide) {
                front = 1;
            } else {
                front = -1;
            }


            if (leftBumper1.isToggled()) {
                robot.leftFront.setPower(front * (-gamepad1.left_stick_y + gamepad1.left_stick_x) + gamepad1.right_stick_x);
                robot.rightFront.setPower(front * (-gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x);
                robot.leftBack.setPower(front * (-gamepad1.left_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x);
                robot.rightBack.setPower(front * (-gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x);
            } else {
                robot.leftFront.setPower ((front * (-gamepad1.left_stick_y + gamepad1.left_stick_x) + gamepad1.right_stick_x) / 2);
                robot.rightFront.setPower((front * (-gamepad1.left_stick_y - gamepad1.left_stick_x) - gamepad1.right_stick_x) / 2);
                robot.leftBack.setPower  ((front * (-gamepad1.left_stick_y - gamepad1.left_stick_x) + gamepad1.right_stick_x) / 2);
                robot.rightBack.setPower ((front * (-gamepad1.left_stick_y + gamepad1.left_stick_x) - gamepad1.right_stick_x) / 2);
            }
            // Setup a variable for each drive wheel to save power level for telemetry
            double forward = gamepad1.left_stick_y;


            if (gamepad2.dpad_up) {
//                robot.leftSlide.setTargetPosition(slidesUp);
//                robot.rightSlide.setTargetPosition(slidesUp);
                robot.leftSlide.setPower(500);
                robot.rightSlide.setPower(500);

                robot.rightSlide.setTargetPosition(500);

            } else if (gamepad2.dpad_down) {
//                robot.leftSlide.setTargetPosition(slidesDown);
//                robot.rightSlide.setTargetPosition(slidesDown);
                robot.leftSlide.setTargetPosition(-500);
                robot.rightSlide.setTargetPosition(-500);
                robot.leftSlide.setPower(-500);
                robot.rightSlide.setPower(-500);
            } else {
//                robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftSlide.setPower(0);
                robot.rightSlide.setPower(0);
            }
            if (gamepad2.a) {
                robot.scoringServo.setPosition(ssScorePosition - 0.1);
                sleep(500);
                robot.scoringServo.setPosition(ssScorePosition);
            } else {
                robot.scoringServo.setPosition(ssStartPosition);
            }
            //AIRPLANE CODE!!!
            // if (gamepad1.y) {
//              robot.shooterServo.setPosition(planeFire);
//            } else {
//              //robot.shooterServo.setPosition(planeStart);
//            }
            if (gamepad2.y) {
                planeFire = true;
            }
            if (gamepad2.x) {
                planeFire = false;
            }
            if (planeFire) {
                robot.planeS.setPosition(firePlane);
            } else {
                robot.planeS.setPosition(startingPosPlane);
            }

            // Show the elapsed game time and wheel power.
            // telemetry.addData("Status", "Run Time: " + runtime.toString());


            // telemetry.addData("Motors", "left (%.2f), right (%.2f)", frontLeftPower, frontRightPower);

            telemetry.addData("leftstickY", gamepad1.left_stick_y);
            telemetry.addData("leftstickX", gamepad1.left_stick_x);
            telemetry.addData("rightstickX", gamepad1.right_stick_x);
            telemetry.addData("rightstickX", gamepad1.right_stick_x);
            telemetry.addData("rightSlide", robot.rightSlide.getTargetPosition());
            telemetry.addData("leftSlide", robot.leftSlide.getTargetPosition());
            telemetry.update();



            telemetry.update();
        }
    }
}
