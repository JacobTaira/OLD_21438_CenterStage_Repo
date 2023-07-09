package org.firstinspires.ftc.teamcode.Pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PipelineTemplate extends OpenCvPipeline {

    private Scalar lower;
    private Scalar upper;

    private Telemetry telemetry;

    public PipelineTemplate(Scalar upperBound, Scalar lowerBound, Telemetry tel){
        upper = upperBound;
        lower = lowerBound;
        this.telemetry = tel;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        if (mat.empty()) {
            telemetry.addLine("Something went wrong");
            return input;
        }

        Mat thresh = new Mat();

        Core.inRange(mat, lower, upper, thresh);

        Mat edges = new Mat();

        Imgproc.Canny(thresh, edges, 100, 300);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        return edges;
    }
}
