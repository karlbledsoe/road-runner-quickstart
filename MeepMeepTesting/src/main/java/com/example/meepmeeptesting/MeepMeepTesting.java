package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Vector2d initialOnset = new Vector2d(-49, -50);
        Pose2d     finalScoreLeft = new Pose2d(-25.134397, -35.48671, Math.toRadians(90));
        Vector2d     pixelScoreSetup = new Vector2d(48,-12);
        Vector2d     hangingDoorPrep = new Vector2d(-40,-12);
        Vector2d lineBack = new Vector2d(-40,-38);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15.375)
                .setDimensions(16.375,15)





                .setStartPose(new Pose2d(13, 59.666, Math.toRadians(-90)))

                .build();

//        myBot.runAction(myBot.getDrive().actionBuilder(myBot.getPose())
//                .strafeToLinearHeading(new Vector2d(16,50), Math.toRadians(90))
//                .setReversed(true)
//                .splineToSplineHeading(finalScoreLeft, finalScoreLeft.heading.plus(Math.PI))
//                .splineToConstantHeading(finalScoreLeft.position, 90)
        myBot.runAction(myBot.getDrive().actionBuilder(myBot.getPose())
                .strafeToLinearHeading(initialOnset, Math.toRadians(-90))
                .setReversed(true)

                .splineToConstantHeading(finalScoreLeft.position, finalScoreLeft.heading.plus(Math.PI/-2))
                .strafeTo(lineBack)
                .strafeToLinearHeading(hangingDoorPrep,Math.toRadians(0))
                .strafeToConstantHeading(pixelScoreSetup)
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