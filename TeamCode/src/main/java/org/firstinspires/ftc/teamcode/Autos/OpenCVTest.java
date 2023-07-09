package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Pipelines.DetectRed;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "EOCV test")
public class OpenCVTest extends LinearOpMode {

    private int width;
    private int height;
    private OpenCvWebcam backCam;

    private Pose2d x = new Pose2d();
    private SampleMecanumDrive drive;

    private DetectRed detector;

    public void initialize() {
        //initializing the basic variables and objects
        width = 160;
        height = 120;
        drive = new SampleMecanumDrive(hardwareMap);
        detector = new DetectRed(width, telemetry);

        // Initialize the back-facing camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        backCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WebcamFront"), cameraMonitorViewId);
        backCam.setPipeline(detector);

        backCam.setMillisecondsPermissionTimeout(2500);
        backCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                telemetry.addLine("started");
                backCam.startStreaming(160, 120, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("not open");
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();
    }


    @Override
    public void runOpMode() throws InterruptedException {
        //initialize all of the variables
        initialize();
        telemetry.update();

        //wait until the start button is clicked before doing anything



        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        TrajectorySequence moveLeft = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeLeft(2)
                .build();

        TrajectorySequence moveRight = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(2)
                .build();

        if(detector.getLocate() == DetectRed.RedLocation.LEFT)
            x = moveLeft.end();
        else if(detector.getLocate() == DetectRed.RedLocation.RIGHT)
            x = moveRight.end();

        TrajectorySequence turn = drive.trajectorySequenceBuilder(x)
                .turn(Math.toRadians(91))
                .build();

        x = turn.end();

        TrajectorySequence recorrect = drive.trajectorySequenceBuilder(x)
                .lineTo(new Vector2d(0,10))
                .build();

        TrajectorySequence work = drive.trajectorySequenceBuilder(x)

               /* .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Pose2d x = drive.getPoseEstimate();
                    if (detector.getLocate() == DetectRed.RedLocation.LEFT) {
                        backCam.startStreaming(width, height);
                        telemetry.addLine("this is running1");
                        TrajectorySequence realign = drive.trajectorySequenceBuilder(x)
                                .strafeRight(-2)
                                .build();
                        drive.followTrajectorySequenceAsync(realign);
                        x = drive.getPoseEstimate();
                        // backCam.stopStreaming();

                    }
                    /*while (detector.getLocate() == DetectRed.RedLocation.RIGHT) {

                        TrajectorySequence realign = drive.trajectorySequenceBuilder(x)
                                .strafeLeft(0.1)
                                .build();
                        drive.followTrajectorySequence(realign);
                        backCam.startStreaming(160, 120, OpenCvCameraRotation.UPRIGHT);
                        x = drive.getPoseEstimate();
                    }


                })*/
                .forward(10)
                .build();




        waitForStart();


        drive.followTrajectorySequence(turn);

        x = new Pose2d(x.getX()+(detector.getDistanceFromMidPoint()*(2.2/width)),0,91);

        drive.followTrajectorySequence(recorrect);

        

        while (opModeIsActive() && !isStopRequested())
            drive.update();


    }
}
