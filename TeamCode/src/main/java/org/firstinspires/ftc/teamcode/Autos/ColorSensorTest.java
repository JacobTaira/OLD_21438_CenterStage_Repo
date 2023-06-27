package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class ColorSensorTest extends LinearOpMode{
    ColorSensor color;

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(0,0,Math.toRadians(180));
        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);
        bot.setPoseEstimate(startPose);

        TrajectorySequence right = bot.trajectorySequenceBuilder(startPose)
                .strafeRight(12)
                .waitSeconds(1)
                .forward(12)
                .build();

        TrajectorySequence left = bot.trajectorySequenceBuilder(startPose)
                .strafeLeft(12)
                .waitSeconds(1)
                .forward(12)
                .build();

        color = hardwareMap.get(ColorSensor.class, "Color");

        waitForStart();

        if(color.red() > 20)
            bot.followTrajectorySequenceAsync(right);
        else if(color.blue() > 20)
            bot.followTrajectorySequenceAsync(left);

        while (opModeIsActive()) {
            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.addData("Alpha", color.alpha());
            telemetry.update();
        }

    }
}
