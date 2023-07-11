package org.firstinspires.ftc.teamcode.Autos;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class ColorSensorTest extends LinearOpMode{
    ColorSensor color1;
    ColorSensor color2;
    DistanceSensor dist;

    @Override
    public void runOpMode() {

        Pose2d startPose = new Pose2d(0,0,Math.toRadians(180));
        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);
        bot.setPoseEstimate(startPose);

        TrajectorySequence right = bot.trajectorySequenceBuilder(startPose)
                .strafeRight(2)
                .build();

        TrajectorySequence left = bot.trajectorySequenceBuilder(startPose)
                .strafeLeft(2)
                .build();



        color1 = hardwareMap.get(ColorSensor.class, "Color");
        color2 = hardwareMap.get(ColorSensor.class, "Color2");
        dist = hardwareMap.get(DistanceSensor.class,"Distance");
        //color.enableLed(false);

        String followingRed = "";


        waitForStart();
/*
        if(((color2.red() > color2.green()) && (color2.red() > color2.blue()) ))//if red
            bot.followTrajectorySequenceAsync(right);
        else if(((color2.blue() > color2.green()) && (color2.blue() > color2.red()) ))//if blue
            bot.followTrajectorySequenceAsync(left);
*/
        if(!((color2.red() > color2.green()) && (color2.red() > color2.blue()) ) && !((color1.red() > color1.green()) && (color1.red() > color1.blue()) )){
            followingRed = "following red";
        }
        if(!((color2.red() > color2.green()) && (color2.red() > color2.blue()) ) && ((color1.red() > color1.green()) && (color1.red() > color1.blue()) )){
            followingRed = "too far left";
            bot.followTrajectorySequenceAsync(right);
        }
        if(((color2.red() > color2.green()) && (color2.red() > color2.blue()) ) && !((color1.red() > color1.green()) && (color1.red() > color1.blue()))){
            followingRed = "too far right";
            bot.followTrajectorySequenceAsync(left);
        }

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData(" c1 Red", color1.red());
            telemetry.addData(" c2 Red", color2.red());
            //telemetry.addData("Alpha", color.alpha());
            //telemetry.addData("argb", color.argb());
            telemetry.addData(" c1 Green", color1.green());
            telemetry.addData(" c2 Green", color2.green());
            telemetry.addData(" c1 Blue", color1.blue());
            telemetry.addData(" c2 Blue", color2.blue());
            telemetry.addData("distance in inches", dist.getDistance(DistanceUnit.INCH));

            telemetry.addData("status: ", followingRed);
            bot.update();
            telemetry.update();
        }

    }
}