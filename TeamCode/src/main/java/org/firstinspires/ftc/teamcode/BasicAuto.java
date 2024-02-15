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

@Autonomous(preselectTeleOp = "BasicOpMode_Linear")
public class BasicAuto extends LinearOpMode {
    public ColorDetection.BlueDeterminationPipeline pipelineBlue;
    public ColorDetection.RedDeterminationPipeline pipelineRed;
    private ElapsedTime timer = new ElapsedTime();
    Object placement;
    OpenCvCamera camera;
    int randomizationPosition = 0;
    /**
     * //    String team = "Blue";
     * //    String position = "Back";
     * //    String parking = "Right";
     */
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
    /**
     * Pose2d startingPosBB = new Pose2d(13, 59.666, Math.toRadians(90));
     * Pose2d startingPosFB = new Pose2d(-37, 59.666, Math.toRadians(90));
     * Pose2d startingPosBR = new Pose2d(13, -59.666, Math.toRadians(-90));
     * Pose2d startingPosFR = new Pose2d(-37, -59.666, Math.toRadians(-90));
     * Pose2d redFrontScore = new Pose2d(12.134397, -33.48671, Math.toRadians(-90));
     * Pose2d redFrontScoreRight = new Pose2d(19.134397 - 0.5, -35.48671 - 0.5, Math.toRadians(-90));
     * <p>
     * Pose2d redFrontScoreMid = new Pose2d(12.134397, -35.48671, Math.toRadians(-90));
     */
    Pose2d finalScoreLeft = new Pose2d(12.134397, -33.48671, Math.toRadians(-90));
    Pose2d finalScoreRight = new Pose2d(19.134397 - 0.5, -35.48671 - 0.5, Math.toRadians(-90));

    Pose2d finalScoreMid = new Pose2d(12.134397, -35.48671, Math.toRadians(-90));
//    Pose2d parkingCornerRed = new Pose2d(60, -59.666, Math.toRadians(-90));
//    Pose2d parkingCornerBlue = new Pose2d(60, 59.666, Math.toRadians(-90));

    Vector2d initialOnset = new Vector2d(-49, 50);
    Vector2d pixelScoreSetup = new Vector2d(48, 12);
    Vector2d hangingDoorPrep = new Vector2d(-40, 12);
    Vector2d lineBack = new Vector2d(-40, 38);
//    Pose2d finalParking = new Pose2d(60, -59.666, Math.toRadians(-90));

    GamepadEx a1 = new GamepadEx();
    GamepadEx b1 = new GamepadEx();
    GamepadEx x1 = new GamepadEx();
    GamepadEx y1 = new GamepadEx();
    boolean isBlue = false;
    boolean isFront = false;
    boolean confirm = false;
    boolean parkMiddle = false;


    @Override
    public void runOpMode() {


        telemetry.addData("Status \nA:TEAM\nB:POSITION (FRONT OR BACK) \n DPAND, LEFT OR RIGHT: PARKING \nY:CONFIRM", "Ready to run");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipelineBlue = new ColorDetection.BlueDeterminationPipeline();
        pipelineRed = new ColorDetection.RedDeterminationPipeline();
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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
            telemetry.addData("BLUE", "FRONT");

            telemetry.update();
            finalStartingPos = new Pose2d(-37, 59.666, Math.toRadians(90));
            xMod = -1;
            yMod = -1;
            hMod = 0;

            initialOnset = new Vector2d(-49, 50);
            pixelScoreSetup = new Vector2d(48, 12);
            hangingDoorPrep = new Vector2d(-40, 12);
            lineBack = new Vector2d(-40, 38);
            finalScoreLeft = new Pose2d(-25.134397, 35.48671, Math.toRadians(90));
            finalScoreRight = new Pose2d(-24-2.134397 + 4.5, 35.48671 - 0.5, Math.toRadians(90));
            finalScoreMid = new Pose2d(-43.134397, 25.48671, Math.toRadians(0));
        } else if (!isBlue && isFront) {
            telemetry.addData("RED", "FRONT");

            telemetry.update();
            finalStartingPos = new Pose2d(-37, -59.666, Math.toRadians(-90));
            xMod = -1;
            yMod = 1;
            hMod = 180;
            initialOnset = new Vector2d(-49, -50);
            pixelScoreSetup = new Vector2d(48, -12);
            hangingDoorPrep = new Vector2d(-40, -12);
            lineBack = new Vector2d(-40, -38);
            finalScoreLeft = new Pose2d(-25.134397, -35.48671, Math.toRadians(-90));
            finalScoreRight = new Pose2d(-24-2.134397 + 4.5, 35.48671 - 0.5, Math.toRadians(-90));
            finalScoreMid = new Pose2d(-43.134397, -25.48671, Math.toRadians(0));
        } else if (isBlue && !isFront) {
            telemetry.addData("BLUE", "BACK");

            telemetry.update();
            xMod = 1;
            hMod = 0;
//            finalParking = parkingCornerBlue;
            finalStartingPos = new Pose2d(13, 59.666, Math.toRadians(90));
            initialOnset = new Vector2d(49 - 15, 50);
            pixelScoreSetup = new Vector2d(48, 13);
            hangingDoorPrep = new Vector2d(49 - 15, 35.48671 - 0.5);
            lineBack = new Vector2d(40 - 24, 38);
            finalScoreLeft = new Pose2d(25.134397 - 24, 35.48671, Math.toRadians(90));
            finalScoreRight = new Pose2d(2.134397 + 4.5, 35.48671 - 0.5, Math.toRadians(90));
            finalScoreMid = new Pose2d(14.134397, 35.48671 - 0.5, Math.toRadians(90));
            yMod = -1;
        } else if (!isBlue && !isFront) {
            telemetry.addData("RED", "BACK");

            telemetry.update();

            finalStartingPos = new Pose2d(13, -59.666, Math.toRadians(-90));
            xMod = 1;
            yMod = 1;
            hMod = 180;
//            finalParking = parkingCornerRed;
            initialOnset = new Vector2d(49 - 24, -50);
            pixelScoreSetup = new Vector2d(48, -13);
            hangingDoorPrep = new Vector2d(49 - 24, -50);
            lineBack = new Vector2d(40 - 24, -38);
            finalScoreLeft = new Pose2d(25.134397 - 24, -35.48671, Math.toRadians(-90));
            finalScoreRight = new Pose2d(-2.134397 + 4.5-24, 35.48671 - 0.5, Math.toRadians(-90));
            finalScoreMid = new Pose2d(19.134397, -35.48671 - 0.5, Math.toRadians(90));
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


//        Vector2d initialOnset = new Vector2d(-49, -50);
//        Vector2d pixelScoreSetup = new Vector2d(48, -12);
//        Vector2d hangingDoorPrep = new Vector2d(-40, -12);
//        Vector2d lineBack = new Vector2d(-40, -38);
//        Pose2d finalScoreLeft = new Pose2d(-25.134397, -35.48671, Math.toRadians(-90));
//        randomizationPosition = 1;

//        randomizationPosition = 1;
        MecanumDrive drive = new MecanumDrive(hardwareMap, finalStartingPos);

        if (randomizationPosition == -1) {
            drive.updatePoseEstimate();
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(initialOnset, Math.toRadians(90 * yMod))
                    .setReversed(true)
                    .splineToConstantHeading(finalScoreLeft.position, finalScoreLeft.heading.plus(Math.PI / -2))
                    .build());
            drive.updatePoseEstimate();
            drive.intake.setPower(0.5);
            waitEx(2000);
            drive.intake.setPower(0);
            drive.updatePoseEstimate();
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeTo(lineBack)
                    .strafeToLinearHeading(hangingDoorPrep, Math.toRadians(0))
                    .strafeToConstantHeading(pixelScoreSetup)
                    .build());
        } else if (randomizationPosition == 1) {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(initialOnset, Math.toRadians(-90 * yMod))
                    .setReversed(true)
                    .splineToConstantHeading(finalScoreRight.position, finalScoreRight.heading.plus(Math.PI / -2 * yMod))
                    .build());
            drive.updatePoseEstimate();
            drive.intake.setPower(0.5);
            waitEx(2000);
            drive.intake.setPower(0);
            drive.updatePoseEstimate();
//            Actions.runBlocking(drive.actionBuilder(drive.pose)
//                    .strafeToLinearHeading(hangingDoorPrep, Math.toRadians(0))
////                    .strafeToConstantHeading(pixelScoreSetup)
//                    .build());
//            drive.updatePoseEstimate();
        } else {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(initialOnset, Math.toRadians(90))
                    .setReversed(true)
                    .splineToSplineHeading(finalScoreMid, finalScoreMid.heading.plus(Math.PI))
                    .splineToConstantHeading(finalScoreMid.position, finalScoreMid.heading.plus(Math.PI))
                    .build());
        }

        drive.updatePoseEstimate();
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(finalStartingPos.position, finalStartingPos.heading)
                .build()
        );
        drive.updatePoseEstimate();
        drive.intake.setPower(0.5);
        waitEx(2000);
        drive.intake.setPower(0);
        drive.updatePoseEstimate();
        if (!parkMiddle && !isFront) {
//            Actions.runBlocking(drive.actionBuilder(drive.pose)
//                    .strafeToLinearHeading(finalParking.position, finalStartingPos.heading)
//                    .build()
//            );
            drive.updatePoseEstimate();
        }
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
