package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Pose2d;
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

    String team = "Blue";

    boolean confirm = false;
    String position = "Back";

    Pose2d finalStartingPos = new Pose2d(0, 0, 0);
    Pose2d startingPosFB = new Pose2d(13, 59.666, Math.toRadians(90));
    Pose2d startingPosBB = new Pose2d(-13, 59.666, Math.toRadians(90));
    Pose2d startingPosFR = new Pose2d(13, -59.666, Math.toRadians(-90));
    Pose2d startingPosBR = new Pose2d(-13, -59.666, Math.toRadians(-90));

    int xMod = -1;

    int yMod = -1;
    int hMod = 180;

    /**
     * 0==blue
     * 1==red
     */
//    int redVBlue() {
//        return team % 2;
//    }

    /**
     * 0==Back
     * 1==Front
     */
//    int backVFront() {
//        return position % 2;
//    }

    boolean aWasPressed = false;
    boolean bWasPressed = false;
    boolean yWasPressed = false;
    boolean xWasPressed = false;

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
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"Webcam 1"), cameraMonitorViewId);

        pipelineBlue = new ColorDetection.BlueDeterminationPipeline();
        pipelineRed = new ColorDetection.RedDeterminationPipeline();

//        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
//        // out when the RC activity is in portrait. We do our actual image processing assuming
//        // landscape orientation, though.
        camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);


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
            placement = pipelineBlue.getAnalysis();
            telemetry.addData("Analysis", placement);
            if (gamepad1.dpad_down) {
                confirm = true;
            }
            if (gamepad1.dpad_up){
                confirm = false;
            }
            if (gamepad1.b && !bWasPressed) {
                team="Red";
                bWasPressed = true;
            }
            if (gamepad1.x && !xWasPressed) {
                team="Blue";
                xWasPressed = true;
            }
            if (gamepad1.y && !yWasPressed) {
                position="Front";
                yWasPressed = true;
            }
            if (gamepad1.a && !aWasPressed) {
                position="Back";
                aWasPressed = true;
            }
            telemetry.addData("Team", team);
            telemetry.addData("Unconfirmed","");
            telemetry.addData("Position", position);
            telemetry.update();
            if(confirm){
                if(team == "Blue") {
                    camera.setPipeline(pipelineBlue);
                }
                if(team == "Red"){
                    camera.setPipeline(pipelineRed);
                }
            }

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
        if (team == "Blue" && position == "Back") {
            finalStartingPos = startingPosBB;
            xMod = -1;
            yMod = 1;
            hMod = 0;
        } else if (team == "Blue" && position == "Front") {
            finalStartingPos = startingPosFB;
            xMod = 1;
            yMod = 1;
            hMod = 0;
        } else if (team == "Red" && position == "Front") {
            finalStartingPos = startingPosFR;
            xMod = 1;
            yMod = -1;
            hMod=180;
        } else if (team == "Red" && position == "Back") {
            finalStartingPos = startingPosBR;
            xMod = -1;
            yMod = -1;
            hMod = 180;
        }
        if (placement == ColorDetection.BlueDeterminationPipeline.TeamElementPosition.LEFT) {
            randomizationPosition = -1;
        } else if (placement == ColorDetection.BlueDeterminationPipeline.TeamElementPosition.RIGHT) {
            randomizationPosition = 1;
        }
        waitForStart();
        if(finalStartingPos.equals(new Pose2d(0,0,0))) {
            waitEx(100000);
        }

        MecanumDrive drive = new MecanumDrive(hardwareMap, finalStartingPos);
      /** MIDDLE
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(13 * xMod, 32.666 * yMod), Math.toRadians(-90))
                .build()
        );
        drive.updatePoseEstimate();
        drive.intake.setPower(0.5);
        waitEx(1000);
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(13 * xMod, 37.666 * yMod), Math.toRadians(-90))
                .build()
        );
        drive.updatePoseEstimate();
        drive.intake.setPower(0);
        */
      /**LEFT
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(13 * xMod, 37.666 * yMod), Math.toRadians(-90*-yMod))
                .build()
        );
        drive.updatePoseEstimate();
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(14 * xMod+yMod*3, 33.666 * yMod), Math.toRadians(180+hMod))
                .build()
        );
        drive.updatePoseEstimate();
        drive.intake.setPower(0.5);
        waitEx(1000);
        drive.intake.setPower(0);
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(16 * xMod, 33.666 * yMod), Math.toRadians(180+hMod))
                .build()
        );
        drive.updatePoseEstimate();
       */
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(13 * xMod, 37.666 * yMod), Math.toRadians(-90*-yMod))
                .build()
        );
        drive.updatePoseEstimate();
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(14 * xMod-yMod*3, 33.666 * yMod), Math.toRadians(0+hMod))
                .build()
        );
        drive.updatePoseEstimate();
        drive.intake.setPower(0.5);
        waitEx(1000);
        drive.intake.setPower(0);
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(16 * xMod, 33.666 * yMod), Math.toRadians(0+hMod))
                .build()
        );
        drive.updatePoseEstimate();
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

        telemetry.addData("Path", "Complete");
        telemetry.update();
        waitEx(100000);

    }

    private void waitEx(double milliseconds) {
        ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        waitTimer.reset();
        while (waitTimer.time(TimeUnit.MILLISECONDS) < milliseconds && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Waiting", "");
            telemetry.update();
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
