package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Pipelines.DetectRed;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class OpenCVTest extends LinearOpMode {

    private int width;
    private int height;
    private OpenCvCamera backCam;
    private SampleMecanumDrive drive;

    private DetectRed detector;

    void initialize() {
        //initializing the basic variables and objects
        width = 180;
        height = 240;
        drive = new SampleMecanumDrive(hardwareMap);
        detector = new DetectRed(width);

        // Initialize the back-facing camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        backCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        // Connect to the camera
        backCam.openCameraDevice();//apperently this line of code can be removed later but for now, its staying
        // Use the pipeline created
        // processFrame() will be called to process the frame
        backCam.setPipeline(detector);
        // Remember to change the camera rotation
        backCam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        //initialize all of the variables
        initialize();
        //wait until the start button is clicked before doing anything
        waitForStart();

        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));

        TrajectorySequence work = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    Pose2d x = drive.getPoseEstimate();
                    if (detector.getLocate() == DetectRed.RedLocation.LEFT) {

                        while (detector.getLocate() != DetectRed.RedLocation.CENTER) {
                            TrajectorySequence realign = drive.trajectorySequenceBuilder(x)
                                    .strafeRight(0.3)
                                    .build();
                            x = drive.getPoseEstimate();
                        }
                    } else if (detector.getLocate() == DetectRed.RedLocation.RIGHT) {
                        while (detector.getLocate() != DetectRed.RedLocation.CENTER) {
                            TrajectorySequence realign = drive.trajectorySequenceBuilder(x)
                                    .strafeLeft(0.3)
                                    .build();
                            x = drive.getPoseEstimate();
                        }
                    }
                })
                .waitSeconds(6)
                .forward(10)
                .build();

    }
}
