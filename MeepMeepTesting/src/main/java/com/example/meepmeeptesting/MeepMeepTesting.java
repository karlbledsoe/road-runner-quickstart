package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d     finalScoreLeft = new Pose2d(-25.134397, 35.48671, Math.toRadians(90));
        Pose2d     pixelScoreSetup = new Pose2d(48, 10, Math.toRadians(90));
        Pose2d     pixelScore = new Pose2d(48, 24, Math.toRadians(0));
        Pose2d parkingCornerBlue = new Pose2d(60, 59.666, Math.toRadians(0));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15.375)
                .setDimensions(16.375,15)





                .setStartPose(new Pose2d(-37, 59.666, Math.toRadians(90)))
                .build();

//        myBot.runAction(myBot.getDrive().actionBuilder(myBot.getPose())
//                .strafeToLinearHeading(new Vector2d(16,50), Math.toRadians(90))
//                .setReversed(true)
//                .splineToSplineHeading(finalScoreLeft, finalScoreLeft.heading.plus(Math.PI))
//                .splineToConstantHeading(finalScoreLeft.position, 90)
        myBot.runAction(myBot.getDrive().actionBuilder(myBot.getPose())
                .strafeToLinearHeading(new Vector2d(-49,50), Math.toRadians(90))
                .setReversed(true)
//                .splineToSplineHeading(finalScoreLeft, finalScoreLeft.heading.plus(Math.PI))

                .splineToConstantHeading(finalScoreLeft.position, finalScoreLeft.heading.plus(Math.PI/-2))
                        .strafeTo(new Vector2d(-40,38))
                        .strafeToLinearHeading(new Vector2d(-40,12),Math.toRadians(0))

                        .strafeToConstantHeading(new Vector2d(48,12))

//                .splineToSplineHeading(new Pose2d(-49,25, Math.toRadians(0)),Math.toRadians(-90))
//
//                .splineToConstantHeading(finalScoreLeft.position.plus(new Vector2d(0,-25)), finalScoreLeft.heading.plus(Math.PI/-2))
//                        .splineToConstantHeading(pixelScoreSetup.position, pixelScoreSetup.heading.plus(Math.PI/-2))
//                .splineToConstantHeading(pixelScore.position, pixelScore.heading.plus(Math.PI/-2))
//                .splineToConstantHeading(pixelScore.position.plus(new Vector2d(4,0)), pixelScore.heading.plus(Math.PI/-2))
//                .splineToConstantHeading(pixelScore.position.plus(new Vector2d(0,-10)), pixelScore.heading.plus(Math.PI/-2))
//                .splineToConstantHeading(pixelScore.position.plus(new Vector2d(10,-15)), pixelScore.heading.plus(Math.PI/-2))

                .build());
//                .build());


//        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15.375)
//                .setDimensions(16.375,15)
//
//
//
//
//
//                .setStartPose(new Pose2d(13, 59.666, Math.toRadians(90)))
//                .build();
//
//        myBot.runAction(myBot.getDrive().actionBuilder(myBot.getPose())
////                .strafeToLinearHeading(new Vector2d(16,50), Math.toRadians(90))
//                .setReversed(true)
//                .splineToSplineHeading(finalScoreMid, finalScoreMid.heading.plus(Math.PI))
//                .splineToConstantHeading(finalScoreMid.position, finalScoreMid.heading.plus(Math.PI))
//                .build());







        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}