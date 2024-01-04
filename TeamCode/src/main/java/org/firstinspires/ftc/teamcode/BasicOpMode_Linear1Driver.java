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
@TeleOp
@Config
//@Disabled
public class BasicOpMode_Linear1Driver extends LinearOpMode {

    // Declare OpMode members.
    public static double ssStartPosition=0.26;
    public static double ssScorePosition=0.9;
    public static int slidesUp=100;
    public static int slidesDown=0;

    public static double planeOn=1;
    public static double intakePower=8;
    public static double startingPosPlane=0;
    public static double firePlane=1;

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
            robot.leftFront.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
            robot.rightFront.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
            robot.leftBack.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
            robot.rightBack.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);
            // Setup a variable for each drive wheel to save power level for telemetry
            double forward = gamepad1.left_stick_y;

            // Intake button
            robot.intake.setPower((gamepad1.right_trigger - gamepad1.left_trigger) * intakePower);
            if (gamepad1.dpad_up) {
//                robot.leftSlide.setTargetPosition(slidesUp);
//                robot.rightSlide.setTargetPosition(slidesUp);
                robot.leftSlide.setPower(1);
                robot.rightSlide.setPower(1);

            } else if (gamepad1.dpad_down) {
//                robot.leftSlide.setTargetPosition(slidesDown);
//                robot.rightSlide.setTargetPosition(slidesDown);
                robot.leftSlide.setPower(-1);
                robot.rightSlide.setPower(-1);
            } else {
//                robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftSlide.setPower(0);
                robot.rightSlide.setPower(0);
            }
            if (gamepad1.a) {
                robot.scoringServo.setPosition(ssScorePosition-0.1);
                sleep(500);
                robot.scoringServo.setPosition(ssScorePosition);
            } else {
                robot.scoringServo.setPosition(ssStartPosition);
            }
            //AIRPLANE CODE!!!
            // if (gamepad1.y) {
//              robot.shooterMotor.setPower(1);
//              sleep(1000);
//              robot.shooterServo.setPosition(0.3);
//            } else {
//                robot.shooterMotor.setPower(0);
//              //robot.shooterServo.setPosition(0);
//            }
            if(gamepad1.y){
                planeOn++;

            }
            if(planeOn%2==0){
                robot.plane.setVelocity(2000);
            }else{
                robot.plane.setVelocity(0);
            }
            if(gamepad1.y){
                robot.planeS.setPosition(firePlane);
            }
            else{
                robot.planeS.setPosition(startingPosPlane);
            }

            // Show the elapsed game time and wheel power.
            // telemetry.addData("Status", "Run Time: " + runtime.toString());


            // telemetry.addData("Motors", "left (%.2f), right (%.2f)", frontLeftPower, frontRightPower);

            telemetry.addData("leftstickY", gamepad1.left_stick_y);
            telemetry.addData("leftstickX", gamepad1.left_stick_x);
            telemetry.addData("rightstickX", gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}
