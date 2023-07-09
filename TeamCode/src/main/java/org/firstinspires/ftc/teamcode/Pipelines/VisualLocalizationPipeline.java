package org.firstinspires.ftc.teamcode.Pipelines;

import com.vuforia.Rectangle;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class VisualLocalizationPipeline extends OpenCvPipeline {

    public Scalar lowerBound = new Scalar(90, 100, 100);
    public Scalar upperBound = new Scalar(100, 255, 255);

    private double focalLengthPixels;
    private  double focalLength;
    private double sensorHeight;
    private Telemetry telemetry;
    private double realHeight;

    private double angle;

    private double distanceX;

    private double distanceY;

    public VisualLocalizationPipeline(double focalLengthPixels, double focalLength, double sensorHeight, Telemetry telemetry, double realHeight){
        this.focalLengthPixels = focalLengthPixels;
        this.focalLength = focalLength;
        this.sensorHeight = sensorHeight;
        this.telemetry = telemetry;
        this.realHeight = realHeight;
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

        Core.inRange(mat, lowerBound, upperBound, thresh);

        Mat edges = new Mat();

        Imgproc.Canny(thresh, edges, 100, 300);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        if (contours.isEmpty()) return edges;

        Point center = findCenter(contours.get(0));

        angle = findAngle(center,focalLengthPixels,input);

        MatOfPoint x = contours.get(0);

        distanceX = findDistanceX( focalLength, realHeight, x,input, sensorHeight);

        distanceY = findDistanceY(angle, distanceX);


        return edges;
    }

    public double findAngle(Point objectCenter, double focalLength, Mat input) {
        int height = input.rows();
        int width = input.cols();

        Point center = new Point(width / 2, height / 2);

        double distancePixels = Math.sqrt(Math.pow(objectCenter.x - center.x, 2) + Math.pow(objectCenter.y - center.y, 2));
        double angleRad = Math.atan(distancePixels / focalLength);

        return Math.toDegrees(angleRad);

    }

    public Point findCenter(MatOfPoint x) {
        if (!x.empty()) {
            Moments moments = Imgproc.moments(x);

            double centerX = moments.m10 / moments.m00;
            double centerY = moments.m01 / moments.m00;

            return new Point(centerX, centerY);

        }

        return null;
    }

    public double findDistanceX(double focalLength, double realHeight, MatOfPoint x, Mat input, double sensorHeight){
        if(x.empty() || input.empty())
            return 0;

        Rect rectangle = Imgproc.boundingRect(x);

        int height = rectangle.height;

        int imageHeight = input.height();

        double distance = (focalLength*realHeight*imageHeight)/(height*sensorHeight);

        double distanceInInches = distance/25.4;

        return distanceInInches;

    }

    public double findDistanceY(double angle, double distanceX){
        return Math.tan(Math.toRadians(angle)) * distanceX;
    }

    public double getAngle() {
        return angle;
    }

    public double getDistanceX() {
        return distanceX;
    }

    public double getDistanceY() {
        return distanceY;
    }
}
