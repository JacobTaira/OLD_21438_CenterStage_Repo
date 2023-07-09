package org.firstinspires.ftc.teamcode.Pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class DetectJunction extends OpenCvPipeline {

    public Scalar lower = new Scalar(17, 102, 110);
    public Scalar upper = new Scalar(25.5, 255, 255);
    Telemetry telemetry;
    double focalLength;

    public DetectJunction(Telemetry telemetry, double focalLength) {
        this.telemetry = telemetry;
        this.focalLength = focalLength;
    }

    public DetectJunction(Telemetry telemetry) {
        this.telemetry = telemetry;
        focalLength = 1430;
    }

    @Override
    public Mat processFrame(Mat input) {

        Mat mat = new Mat();

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        if (mat.empty()) {
            telemetry.addLine("There is no junction seen");
            return input;
        }


        Mat thresh = new Mat();

        Core.inRange(mat, lower, upper, thresh);

        Mat edges = new Mat();

        Imgproc.Canny(thresh, edges, 100, 1000);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_TC89_KCOS);

        double maxArea = -1;
        int maxContourIdx = -1;
        for (int i = 0; i < contours.size(); i++) {
            double area = Imgproc.contourArea(contours.get(i));
            if (area > maxArea) {
                maxArea = area;
                maxContourIdx = i;
            }
        }

        // Draw the largest contour on the original image
        Mat resultImage = input.clone();
        Imgproc.drawContours(resultImage, contours, maxContourIdx, new Scalar(0, 255, 0), 1);

        Point center = findContourCenter(contours, maxContourIdx);


        telemetry.update();
        telemetry.addLine("Angle is " + calculateAngle(input, center, 23));
        return resultImage;
    }

    public Point findContourCenter(List<MatOfPoint> contours, int i) {
        if (!contours.isEmpty()) {
            MatOfPoint select = contours.get(i);

            Moments moments = Imgproc.moments(select);

            double x = moments.m10 / moments.m00;
            double y = moments.m01 / moments.m00;

            return new Point(x, y);
        }

        return null;
    }

    public double calculateAngle(Mat image, Point center, double focalLength) {
        int imageHeight = image.rows();
        int imageWidth = image.cols();

        Point imageCenter = new Point(imageWidth / 2, imageHeight / 2);

        double distancePixels = Math.sqrt(Math.pow(center.x - imageCenter.x, 2) + Math.pow(center.y - imageCenter.y, 2));

        double angleRad = Math.atan(distancePixels / focalLength);

        return Math.toDegrees(angleRad);
    }
}
