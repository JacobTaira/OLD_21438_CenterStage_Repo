package org.firstinspires.ftc.teamcode.Autos;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Pipelines.DetectRed;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.MechanismTemplates.ColorSensorMech;

@Autonomous
@Config
public class ColorSensorTest extends LinearOpMode{
    ColorSensorMech color1;
    ColorSensorMech color2;
    DistanceSensor dist;
    public static double distance= 2.5 ;



    @Override
    public void runOpMode() {

        Pose2d startPose = new Pose2d(0,0,Math.toRadians(0));
        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);
        bot.setPoseEstimate(startPose);

        TrajectorySequence right = bot.trajectorySequenceBuilder(startPose)
                .strafeRight(distance)
                .build();

        TrajectorySequence left = bot.trajectorySequenceBuilder(startPose)
                .strafeLeft(distance)
                .build();

        TrajectorySequence recorrectRight = bot.trajectorySequenceBuilder(new Pose2d())
                .lineTo(new Vector2d(10, -2.5))
                //.strafeRight(2.5)
                //.forward(10)
                .build();
        TrajectorySequence recorrectLeft = bot.trajectorySequenceBuilder(new Pose2d())
                .lineTo(new Vector2d(10, 2.5))
                //.strafeLeft(2.5)
                //.forward(10)
                .build();




        color1 = new ColorSensorMech(hardwareMap, "Color");
        color2 = new ColorSensorMech(hardwareMap, "Color2");
        dist = hardwareMap.get(DistanceSensor.class,"Distance");
        //color.enableLed(false);

        String followingRed = "";

        waitForStart();

        if(!color2.isDetectingRed() && !color1.isDetectingRed()){
            followingRed = "following red";
        }
        if(!color2.isDetectingRed() && color1.isDetectingRed()){
            followingRed = "too far left";
            bot.followTrajectorySequenceAsync(recorrectRight);
        }
        if(color2.isDetectingRed() && !color1.isDetectingRed()){
            followingRed = "too far right";
            bot.followTrajectorySequenceAsync(recorrectLeft);
        }


        while (opModeIsActive() && !isStopRequested()) {
            /*
            telemetry.addData(" c1 Red", color1.red());
            telemetry.addData(" c2 Red", color2.red());
            //telemetry.addData("Alpha", color.alpha());
            //telemetry.addData("argb", color.argb());
            telemetry.addData(" c1 Green", color1.green());
            telemetry.addData(" c2 Green", color2.green());
            telemetry.addData(" c1 Blue", color1.blue());
            telemetry.addData(" c2 Blue", color2.blue());
            */
            telemetry.addData("distance in inches", dist.getDistance(DistanceUnit.INCH));

            telemetry.addData("status: ", followingRed);
            bot.update();
            color1.update(telemetry);
            color2.update(telemetry);
            telemetry.update();
        }

    }
}