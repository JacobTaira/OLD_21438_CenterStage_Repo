package org.firstinspires.ftc.teamcode.Autos;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous
public class DistSensorTest extends LinearOpMode{
    DistanceSensor dist;
    public double distance;
    public double distanceLeft =0;
    public double distanceExtra =0;

    @Override
    public void runOpMode() {

        Pose2d startPose = new Pose2d(0,0,Math.toRadians(0));
        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);
        bot.setPoseEstimate(startPose);

        dist = hardwareMap.get(DistanceSensor.class,"Distance");

        waitForStart();
        /*
        distance = dist.getDistance(DistanceUnit.INCH) - 11.0;

        TrajectorySequence move = bot.trajectorySequenceBuilder(startPose)
                .forward(distance)
                .waitSeconds(2)
                .build();

        bot.followTrajectorySequenceAsync(move);
         */

        double measureDist = dist.getDistance(DistanceUnit.INCH);

        if(measureDist > 4.5 ) {
            distanceLeft = measureDist - 4;

            TrajectorySequence moveForward = bot.trajectorySequenceBuilder(new Pose2d())
                    .forward(distanceLeft)
                    .build();

            bot.followTrajectorySequenceAsync(moveForward);
        }

       measureDist = dist.getDistance(DistanceUnit.INCH);

       if(measureDist < 3.5) {
           distanceExtra = Math.abs(measureDist - 4);
           TrajectorySequence moveBack = bot.trajectorySequenceBuilder(new Pose2d())
                   .back(distanceExtra)
                   .build();

           bot.followTrajectorySequenceAsync(moveBack);
       }

        while (opModeIsActive() && !isStopRequested()) {

            telemetry.addData("distance in inches", dist.getDistance(DistanceUnit.INCH));
            //telemetry.addData("dist to travel", distance);
            telemetry.addData("dist left", distanceLeft);
            telemetry.addData("dist extra", distanceExtra);
            telemetry.update();
            bot.update();
        }

    }
}