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
public class AutoRedFront extends LinearOpMode {
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


            camera.setPipeline(pipelineRed);
            placement = pipelineRed.getAnalysis();

            telemetry.addData("Analysis", placement);


            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        if (placement == ColorDetection.RedDeterminationPipeline.TeamElementPosition.LEFT) {
            randomizationPosition = 1;
        } else if (placement == ColorDetection.RedDeterminationPipeline.TeamElementPosition.RIGHT) {
            randomizationPosition = -1;
        }

        waitForStart();
        finalStartingPos = new Pose2d(-13-24, -59.666, Math.toRadians(-90));

        if (finalStartingPos.equals(new Pose2d(0, 0, 0))) {
            waitEx(100000);
        }

        MecanumDrive drive = new MecanumDrive(hardwareMap, finalStartingPos);

        if (placement == ColorDetection.RedDeterminationPipeline.TeamElementPosition.RIGHT) {
            drive.updatePoseEstimate();
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(-34-24, -50), Math.toRadians(-90))
                    .setReversed(true)
                    .splineToConstantHeading(new Vector2d(-27-24,-35), Math.toRadians(-90))
                    .lineToY(-30)
                    .lineToY(-35)
                    .build());
            drive.updatePoseEstimate();
            drive.intake.setPower(0.5);
            waitEx(2000);
            drive.intake.setPower(0);
            drive.updatePoseEstimate();
        } else if (placement == ColorDetection.RedDeterminationPipeline.TeamElementPosition.LEFT) {

            Actions.runBlocking(drive.actionBuilder(drive.pose)

                    .strafeToLinearHeading(new Vector2d(-34-24, -50), Math.toRadians(-90))
                    .setReversed(true)
                    .splineToConstantHeading(new Vector2d(-10.5-24, -34), Math.toRadians(0))
                    .build());
            drive.updatePoseEstimate();
            drive.intake.setPower(0.5);
            waitEx(2000);
            drive.intake.setPower(0);
            drive.updatePoseEstimate();
            //CENTER
        } else {
            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(new Vector2d(-34-24, -50), Math.toRadians(-90))
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(-17-24, -25, Math.toRadians(0)), Math.toRadians(190))
                    .lineToX(-10-24)
                    .lineToX(-17-24)
                    .build());
            drive.updatePoseEstimate();
            drive.intake.setPower(0.5);
            waitEx(2000);
            drive.intake.setPower(0);
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
