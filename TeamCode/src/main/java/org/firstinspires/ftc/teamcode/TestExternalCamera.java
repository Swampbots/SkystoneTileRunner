package org.firstinspires.ftc.teamcode;

import android.graphics.ImageFormat;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Disabled
@Autonomous(name = "Test: External Camera", group = "Testing")
public class TestExternalCamera extends LinearOpMode {

    SLICBotHardware hardware = new SLICBotHardware();
    OpenCvWebcam webcam;

    public void runOpMode() {

        //hardware.init(hardwareMap);
        webcam = new OpenCvWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName())
        );


        webcam.openCameraDevice();
        webcam.setPipeline(new StageSwitchingPipeline());
        webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);


        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();


        while(opModeIsActive()) {


            telemetry.addLine("Running");
            telemetry.addData("Webcam data", webcam.getCameraCharacteristics().getSizes(ImageFormat.YUY2).length);
            telemetry.update();
        }

    }

    static class StageSwitchingPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {

//            numCols = input.cols();
//            numRows = input.rows();


            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    new Scalar(0, 255, 0), 4);

            return input;
        }
    }
}


