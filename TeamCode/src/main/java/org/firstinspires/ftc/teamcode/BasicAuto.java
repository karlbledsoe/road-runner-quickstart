package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(preselectTeleOp = "BasicOpMode_Linear")
public class BasicAuto extends LinearOpMode {

    public ColorDetection.BlueDeterminationPipeline pipelineBlue;
    public ColorDetection.RedDeterminationPipeline pipelineRed;
    private ElapsedTime timer = new ElapsedTime();
    Object placement;
    OpenCvCamera camera;
    int randomizationPosition = 0;

//    String team = "Blue";

//    String position = "Back";
//    String parking = "Right";


    int xMod = -1;

    int yMod = -1;
    int hMod = 180;

    Rotation2d turnConstant = new Rotation2d(1, 1);
//    int redVBlue() {
//        return team % 2;
//    }
    /**
     * 0==blue
     * 1==red
     */
    Pose2d finalStartingPos = new Pose2d(0, 0, 0);
    Pose2d startingPosBB = new Pose2d(13, 59.666, Math.toRadians(90));
    Pose2d startingPosFB = new Pose2d(-37, 59.666, Math.toRadians(90));
    Pose2d startingPosBR = new Pose2d(13, -59.666, Math.toRadians(-90));
    Pose2d startingPosFR = new Pose2d(-37, -59.666, Math.toRadians(-90));
    Pose2d redFrontScore = new Pose2d(12.134397, -33.48671, Math.toRadians(-90));
    Pose2d redFrontScoreRight = new Pose2d(19.134397 - 0.5, -35.48671 - 0.5, Math.toRadians(-90));

    Pose2d redFrontScoreMid = new Pose2d(12.134397, -35.48671, Math.toRadians(-90));

    Pose2d finalScoreLeft = new Pose2d(12.134397, -33.48671, Math.toRadians(-90));
    Pose2d finalScoreRight = new Pose2d(19.134397 - 0.5, -35.48671 - 0.5, Math.toRadians(-90));

    Pose2d finalScoreMid = new Pose2d(12.134397, -35.48671, Math.toRadians(-90));
    Pose2d parkingCornerRed = new Pose2d(60, -59.666, Math.toRadians(-90));
    Pose2d parkingCornerBlue = new Pose2d(60, 59.666, Math.toRadians(-90));

    Vector2d initialOnset = new Vector2d(16, 50);
    Pose2d finalParking = new Pose2d(60, -59.666, Math.toRadians(-90));

    GamepadEx a1 = new GamepadEx();
    GamepadEx b1 = new GamepadEx();
    GamepadEx x1 = new GamepadEx();
    GamepadEx y1 = new GamepadEx();


    boolean isBlue = false;
    boolean isFront = false;
    boolean confirm = false;
    boolean parkMiddle = false;
    /**
     * 0==Red
     * 1==Blue
     */
//    int Team() {
//        return team % 2;
//    }


    /**
     * 0==Back
     * 1==Front
     */
//    int Position(){
//        return position%2;
//    }

    /**
     * 0==Confirm
     * 1==Uncomfirmed
     */
//    int Confirm(){
//        return confirm%2;
//    }

    /**
     * 0==Middle
     * 1==Corner
     */
//    int Parking(){
//        return confirm%2;
//    }


//    int backVFront() {
//        return position % 2;
//    }
    @Override
    public void runOpMode() {

        //   MecanumDrive robot = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // if statements for the element being in the right zone, change placment's value.

        //if(isRight==true){
        //placement==2;
        //}
        //else if(isLeft==true){
        //placement==1;
        //}

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipelineBlue = new ColorDetection.BlueDeterminationPipeline();
        pipelineRed = new ColorDetection.RedDeterminationPipeline();
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
//        // out when the RC activity is in portrait. We do our actual image processing assuming
//        // landscape orientation, though.
//        camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 960, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        // Wait for the game to start (driver presses PLAY)
        // waitForStart();
        while (!isStarted() && !isStopRequested()) {
            y1.updateButton(gamepad1.y);
            if (y1.isPressed()) {
                confirm = !confirm;
            }
//            if (team) {
//                telemetry.addData("Team", "Blue");
//            } else {
//                telemetry.addData("Team", "Red");
//            }
//            if (Position() == 0) {
//                telemetry.addData("Position", "Back");
//            } else {
//                telemetry.addData("Position", "Front");
//            }
//            if (Parking() == 0) {
//                telemetry.addData("Park", "Middle");
//            } else {
//                telemetry.addData("Park", "Corner");
//            }
//            if(Confirm()==1) {
//                telemetry.addData("Confirmed", false);
//            }
            if (confirm) {
                telemetry.addData("Confirmed", true);
                if (isBlue) {
                    camera.setPipeline(pipelineBlue);
                    placement = pipelineBlue.getAnalysis();
                } else {
                    camera.setPipeline(pipelineRed);
                    placement = pipelineRed.getAnalysis();
                }
                telemetry.addData("Analysis", placement);
            } else {
                a1.updateButton(gamepad1.a);
                b1.updateButton(gamepad1.b);
                x1.updateButton(gamepad1.x);
                if (a1.isPressed()) {
                    isBlue = !isBlue;
                }
                if (b1.isPressed()) {
                    isFront = !isFront;
                }
                if (x1.isPressed()) {
                    parkMiddle = !parkMiddle;
                }

            }
            if (isBlue) {
                telemetry.addData("Team", "Blue");

            } else {
                telemetry.addData("Team", "Red");

            }
            if (isFront) {
                telemetry.addData("Position", "Front");
            } else {
                telemetry.addData("Position", "Back");
            }
            if (parkMiddle) {
                telemetry.addData("Parking", "Middle");
            } else {
                telemetry.addData("Parking", "Corner");
            }

            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
        if (isBlue && isFront) {
            finalStartingPos = new Pose2d(-37, 59.666, Math.toRadians(90));
            xMod = -1;
            yMod = 1;
            hMod = 0;
//            finalScoreLeft = new Pose2d(-37.134397, 33.48671, Math.toRadians(90));
//            finalScoreRight = new Pose2d(-37.134397 - 0.5, 35.48671 - 0.5, Math.toRadians(90));
//            finalScoreMid = new Pose2d(-37.134397, 35.48671, Math.toRadians(90));
            initialOnset =new Vector2d(-16,50);
            finalScoreLeft = new Pose2d(-21.134397, 35.48671, Math.toRadians(90));
            finalScoreRight = new Pose2d(-2.134397 - 0.5, 35.48671 - 0.5, Math.toRadians(90));
            finalScoreMid = new Pose2d(-19.134397, 25.48671, Math.toRadians(0));
        } else if (isBlue && !isFront) {

            finalStartingPos = new Pose2d(13, 59.666, Math.toRadians(90));
            ;
            xMod = 1;
            yMod = 1;
            hMod = 0;
            finalParking = parkingCornerBlue;
            initialOnset = new Vector2d(16, 50);
            finalScoreLeft = new Pose2d(25.134397-0.5, 35.48671, Math.toRadians(90));
            finalScoreRight = new Pose2d(2.134397 - 0.5, 35.48671 - 0.5, Math.toRadians(90));
            finalScoreMid = new Pose2d(19.134397, 25.48671, Math.toRadians(0));
        } else if (!isBlue && !isFront) {

            finalStartingPos = new Pose2d(13, 59.666, Math.toRadians(-90));
            xMod=1;
            yMod = -1;
            hMod = 180;
            finalParking = parkingCornerRed;
            initialOnset =new Vector2d(16,-50);
            finalScoreLeft = new Pose2d(21.134397, -35.48671, Math.toRadians(90));
            finalScoreRight = new Pose2d(2.134397 - 0.5, -35.48671 - 0.5, Math.toRadians(90));
            finalScoreMid = new Pose2d(19.134397, -25.48671, Math.toRadians(0));
        } else if (!isBlue && isFront) {

            finalStartingPos = new Pose2d(-37, -59.666, Math.toRadians(-90));
            ;
            xMod = -1;
            yMod = -1;
            hMod = 180;
            initialOnset =new Vector2d(-16,50);
            finalScoreLeft = new Pose2d(-21.134397, -35.48671, Math.toRadians(90));
            finalScoreRight = new Pose2d(-2.134397 - 0.5, -35.48671 - 0.5, Math.toRadians(90));
            finalScoreMid = new Pose2d(-19.134397, -25.48671, Math.toRadians(0));
        }
        if (placement == ColorDetection.BlueDeterminationPipeline.TeamElementPosition.LEFT) {
            randomizationPosition = -1;
        } else if (placement == ColorDetection.BlueDeterminationPipeline.TeamElementPosition.RIGHT) {
            randomizationPosition = 1;
        }
        if (placement == ColorDetection.RedDeterminationPipeline.TeamElementPosition.LEFT) {
            randomizationPosition = -1;
        } else if (placement == ColorDetection.RedDeterminationPipeline.TeamElementPosition.RIGHT) {
            randomizationPosition = 1;
        }
        waitForStart();
        if (finalStartingPos.equals(new Pose2d(0, 0, 0))) {
            waitEx(100000);
        }

        MecanumDrive drive = new MecanumDrive(hardwareMap, finalStartingPos);
//        telemetry.addData("YPos",drive.pose.position.y);
//        telemetry.addData("XPos",drive.pose.position.x);
//        telemetry.addData("Heading",drive.pose.heading);
//        telemetry.update();
//        waitEx(1000000);
        if (randomizationPosition == -1) {
//          Actions.runBlocking(drive.actionBuilder(drive.pose)
//                  .strafeToLinearHeading(new Vector2d(13 * xMod, 37.666 * yMod), Math.toRadians(-90 * -yMod))
//                  .build()
//          );


            drive.updatePoseEstimate();
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(initialOnset,Math.toRadians(90))
                    .setReversed(true)
                    .splineToSplineHeading(finalScoreLeft, finalScoreLeft.heading.plus(Math.PI))
                    .splineToConstantHeading(finalScoreLeft.position, finalScoreLeft.heading.plus(Math.PI))
                    .build());
            ;
//          Actions.runBlocking(drive.actionBuilder(drive.pose)
//                  .strafeToLinearHeading(new Vector2d(16 * xMod, 33.666 * yMod), Math.toRadians(180 + hMod))
//                  .build()
//          );
        } else if (randomizationPosition == 1) {
//          Actions.runBlocking(drive.actionBuilder(drive.pose)
//                  .strafeToLinearHeading(new Vector2d(13 * xMod, 37.666 * yMod), Math.toRadians(-90 * -yMod))
//                  .build()
//          );
//          drive.updatePoseEstimate();
            drive.updatePoseEstimate();
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(initialOnset,Math.toRadians(90))
                    .setReversed(true)
                    .splineToSplineHeading(finalScoreRight, finalScoreRight.heading.plus(Math.PI))
                    .splineToConstantHeading(finalScoreRight.position, finalScoreRight.heading.plus(Math.PI))
                    .build());
//          Actions.runBlocking(drive.actionBuilder(drive.pose)
//                  .strafeToLinearHeading(new Vector2d(16 * xMod, 33.666 * yMod), Math.toRadians(0 + hMod))
//                  .build()
//          );
        } else {
//          Actions.runBlocking(drive.actionBuilder(drive.pose)
//                  .strafeToLinearHeading(new Vector2d(13 * xMod, 32.666 * yMod), Math.toRadians(-90*-yMod))
//                  .build()
//          );
//          drive.updatePoseEstimate();
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(initialOnset,Math.toRadians(90))
                    .setReversed(true)
                    .splineToSplineHeading(finalScoreMid, finalScoreMid.heading.plus(Math.PI))
                    .splineToConstantHeading(finalScoreMid.position, finalScoreMid.heading.plus(Math.PI))
                    .build());
//          drive.updatePoseEstimate();
        }
        drive.updatePoseEstimate();
//        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                .strafeToLinearHeading(new Vector2d(13*xMod,59.999*yMod),Math.toRadians(-90*yMod))
//                .build()
//        );
        drive.intake.setPower(0.5);
        waitEx(2000);
        drive.intake.setPower(0);
        drive.updatePoseEstimate();
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(finalStartingPos.position, finalStartingPos.heading)
                .build()
        );
        drive.updatePoseEstimate();
        if (!parkMiddle && !isFront) {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(finalParking.position, finalStartingPos.heading)
                    .build()
            );
            drive.updatePoseEstimate();

        }

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds
//        robot.leftBack.setPower  (0.3);
//        robot.rightBack.setPower (0.3);
//        robot.leftFront.setPower (0.3);
//        robot.rightFront.setPower(0.3);
//        runtime.reset();
//
//        while (opModeIsActive() && !isStopRequested() && (runtime.seconds() < 2.0)) {
//            telemetry.addData("Running", true);
//            telemetry.update();
//        }
//
//
//        // Step 4:  Stop
//        robot.leftBack.setPower(0);
//        robot.rightBack.setPower(0);
//        robot.leftFront.setPower(0);
//        robot.rightFront.setPower(0);
//        robot.scoringServo.setPosition(0);

//        telemetry.addData("Path", "Complete");
//        telemetry.update();
//        waitEx(100000);

    }

    private void waitEx(double milliseconds) {
        ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        waitTimer.reset();
        while (waitTimer.time(TimeUnit.MILLISECONDS) < milliseconds && opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("Waiting", "");
//            telemetry.update();
        }
    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw, MecanumDrive drive) {
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        drive.leftFront.setPower(leftFrontPower);
        drive.rightFront.setPower(rightFrontPower);
        drive.leftBack.setPower(leftBackPower);
        drive.rightBack.setPower(rightBackPower);
    }
}
