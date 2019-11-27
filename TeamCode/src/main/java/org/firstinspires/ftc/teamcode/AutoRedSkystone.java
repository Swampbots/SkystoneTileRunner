package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.SkystonePlacement.CENTER;
import static org.firstinspires.ftc.teamcode.SkystonePlacement.LEFT;
import static org.firstinspires.ftc.teamcode.SkystonePlacement.RIGHT;


@Autonomous(name = "Red Skystone", group = "Autonomous")
public class AutoRedSkystone extends LinearOpMode {


    SLICBotHardware hardware = new SLICBotHardware();
    BNO055IMU imu;

    OpenCvCamera phoneCam;
    SkystonePatternPipeline skystonePatternPipeline;

    GamepadCooldowns gp1 = new GamepadCooldowns();
    double runtime = 0.0;
    private final double TRIGGER_THRESHOLD = 0.7;

    // HSV Threshold input variables
    private final double THRESHOLD_STEP = 1.0;

    private final double HUE_MAX = 180.0;
    private final double SAT_MAX = 255.0;
    private final double VAL_MAX = 255.0;
    private final double HSV_MIN = 0.0;

    private static double[] hsvHue = new double[]{80.0, 119.0};     // Starts with red outside of threshold (red is hue of 120)
    private static double[] hsvSat = new double[]{175.0, 255.0};
    private static double[] hsvVal = new double[]{50.0, 255.0};      // Testing showed 50 (min) to be enough to cut out most of the Skystone


    private static double rectTop   = 235.0;
    private static double rectLeft  = 5.0;
    private static double rectBot   = 333.0;
    private static double rectRight = 480.0;

    private final double RECT_STEP = 0.04;
    private final double RECT_MIN = 0.0;

    private final int IMG_WIDTH = 480;
    private final int IMG_HEIGHT = 640;

    private static boolean returnHSV = false;
    private static boolean drawRect = false;

    private static double leftBound = 0;
    private static double centerBound = 0;

    private SkystonePlacement skystonePlacement;


    List<MatOfPoint> contours; // Contours from pipeline after filtering


    public void runOpMode() throws InterruptedException {

        hardware.init(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        skystonePatternPipeline = new SkystonePatternPipeline();
        phoneCam.setPipeline(skystonePatternPipeline);
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);


        // Loop during initialization
        while(!opModeIsActive() && !isStopRequested()) {
                runtime = getRuntime();

                //--------------------------------------------------------------------------------------
                // START HSV THRESHOLD CONTROLS
                //--------------------------------------------------------------------------------------

            /*
                CONTROLS: (increase, decrease)
                Hue min: gp1.up,    gp1.down
                Hue max: gp1.y,     gp1.a

                Sat min: gp1.right, gp1.left
                Sat max: gp1.b,     gp1.x

                Val min: gp1.lb,    gp1.lt
                Val max: gp1.rb,    gp1.rt
             */

                // Modify threshold variables if the buttons are pressed and thresholds are within outer limits 0 & 255

                // HUE MINIMUM
                if(gamepad1.dpad_down && gp1.dpDown.ready(runtime)) {
                    if (hsvHue[0] > HSV_MIN)   hsvHue[0] -= THRESHOLD_STEP;
                    else                        hsvHue[0] = HSV_MIN;
                    gp1.dpDown.updateSnapshot(runtime);
                }

                if(gamepad1.dpad_up && gp1.dpUp.ready(runtime)) {
                    if(hsvHue[0] < hsvHue[1])  hsvHue[0] += THRESHOLD_STEP;
                    else                        hsvHue[0] = hsvHue[1];
                    gp1.dpUp.updateSnapshot(runtime);
                }


                // HUE MAXIMUM
                if(gamepad1.y && gp1.y.ready(runtime)) {
                    if (hsvHue[1] < HUE_MAX)   hsvHue[1] += THRESHOLD_STEP;
                    else                        hsvHue[1] = HUE_MAX;
                    gp1.y.updateSnapshot(runtime);
                }

                if(gamepad1.a && gp1.a.ready(runtime)) {
                    if(hsvHue[1] > hsvHue[0])  hsvHue[1] -= THRESHOLD_STEP;
                    else                        hsvHue[1] = hsvHue[0];
                    gp1.a.updateSnapshot(runtime);
                }




                // SAT MINIMUM
                if(gamepad1.dpad_left && gp1.dpLeft.ready(runtime)) {
                    if (hsvSat[0] > HSV_MIN)   hsvSat[0] -= THRESHOLD_STEP;
                    else                        hsvSat[0] = HSV_MIN;
                    gp1.dpLeft.updateSnapshot(runtime);
                }

                if(gamepad1.dpad_right && gp1.dpRight.ready(runtime)) {
                    if(hsvSat[0] < hsvSat[1])  hsvSat[0] += THRESHOLD_STEP;
                    else                        hsvSat[0] = hsvSat[1];
                    gp1.dpRight.updateSnapshot(runtime);
                }


                // SAT MAXIMUM
                if(gamepad1.b && gp1.b.ready(runtime)) {
                    if (hsvSat[1] < SAT_MAX)   hsvSat[1] += THRESHOLD_STEP;
                    else                        hsvSat[1] = SAT_MAX;
                    gp1.b.updateSnapshot(runtime);
                }

                if(gamepad1.x && gp1.x.ready(runtime)) {
                    if(hsvSat[1] > hsvSat[0])  hsvSat[1] -= THRESHOLD_STEP;
                    else                        hsvSat[1] = hsvSat[0];
                    gp1.x.updateSnapshot(runtime);
                }




                // VAL MINIMUM
                if(gamepad1.left_trigger > TRIGGER_THRESHOLD && gp1.lt.ready(runtime)) {
                    if (hsvVal[0] > HSV_MIN)   hsvVal[0] -= THRESHOLD_STEP;
                    else                        hsvVal[0] = HSV_MIN;
                    gp1.lt.updateSnapshot(runtime);
                }

                if(gamepad1.left_bumper && gp1.lb.ready(runtime)) {
                    if(hsvVal[0] < hsvVal[1])  hsvVal[0] += THRESHOLD_STEP;
                    else                        hsvVal[0] = hsvVal[1];
                    gp1.lb.updateSnapshot(runtime);
                }



                // VAL MAXIMUM
                if(gamepad1.right_trigger > TRIGGER_THRESHOLD && gp1.rt.ready(runtime)) {
                    if (hsvVal[1] > hsvVal[0])  hsvVal[1] -= THRESHOLD_STEP;
                    else                        hsvVal[1] = hsvVal[0];
                    gp1.rt.updateSnapshot(runtime);
                }

                if(gamepad1.right_bumper && gp1.rb.ready(runtime)) {
                    if(hsvVal[1] < VAL_MAX)     hsvVal[1] += THRESHOLD_STEP;
                    else                        hsvVal[1] = VAL_MAX;
                    gp1.rb.updateSnapshot(runtime);
                }





                //--------------------------------------------------------------------------------------
                // END HSV THRESHOLD CONTROLS
                //--------------------------------------------------------------------------------------


            /*
                NEW Controls: (left stick and right stick configuration)
                    - Left stick: change top-left corner values relative to
                        ~ left_stick_x (changes left bound)
                        ~ left_stick_y (changes top bound)
                    - Right stick: change bottom-right corner values relative to
                        ~ right_stick_x (changes right bound)
                        ~ right_stick_y (changes bottom bound)
             */

                rectTop     = trim(rectTop      + (gamepad2.left_stick_y * RECT_STEP), RECT_MIN, IMG_HEIGHT);
                rectLeft    = trim(rectLeft     + (gamepad2.left_stick_x * RECT_STEP), RECT_MIN, IMG_WIDTH);

                rectBot     = trim(rectBot      + (gamepad2.right_stick_y * RECT_STEP), RECT_MIN, IMG_HEIGHT);
                rectRight   = trim(rectRight    + (gamepad2.right_stick_x * RECT_STEP), RECT_MIN, IMG_WIDTH);


                returnHSV = gamepad2.a;
                if(gamepad2.x) drawRect = false;
                else if(gamepad2.y) drawRect = true;




                contours = skystonePatternPipeline.filterContoursOutput();
                int numContoursInRect   = 0;
                int numContoursLeft     = 0;
                int numContoursCenter   = 0;
                int numContoursRight    = 0;


                // Calculate left and center boundary lines for cropping rectangle
                leftBound = rectLeft + Math.abs((rectRight - rectLeft) / 3.0);         // x position plus 1/3 of the width
                centerBound = rectLeft + Math.abs((rectRight - rectLeft) * 2.0 / 3.0); // x position plus 2/3 of the width

                // Create Point variable holding center coordinates of boundingRect
                Point rectCenter = new Point();




                try {
                    for(MatOfPoint c : contours) {
                        Rect boundingRect = Imgproc.boundingRect(c);

                        // See if boundingRect is inside of the cropping rectangle
                        if(boundingRect.x >= rectLeft &&
                                boundingRect.y >= rectTop &&
                                boundingRect.x + boundingRect.width <= rectRight &&
                                boundingRect.y + boundingRect.height <= rectBot) {
                            // We've got a valid contour!
                            numContoursInRect ++;
                            // Now classify as left, center, or right
                            rectCenter.x = (2 * boundingRect.x + boundingRect.width) / 2.0;     // Get the center of the rectangle
                            rectCenter.y = (2 * boundingRect.y + boundingRect.height) / 2.0;
                            if(rectCenter.x < leftBound)        numContoursLeft     += boundingRect.area(); // rectangle in left 1/3 of the screen
                            else if(rectCenter.x < centerBound) numContoursCenter   += boundingRect.area(); // rectangle in center 1/3 of the screen
                            else                                numContoursRight    += boundingRect.area(); // rectangle in right 1/3 of the screen
                        }
                    }
                } catch(Exception e) {
                    telemetry.addLine("Error while iterating through contours!");
                }


                // Compare contour area tallies to see which third of the bounding rectangle
                // has the least (which will be the third with the Skystone in it)
                skystonePlacement =
                        compareAreaTallies(numContoursLeft, numContoursCenter, numContoursRight);





                telemetry.addLine("Looping during initialization");
                telemetry.addData("Runtime", runtime);
                telemetry.addLine(String.format("Hue: [%s, %s]", hsvHue[0], hsvHue[1]));
                telemetry.addLine(String.format("Sat: [%s, %s]", hsvSat[0], hsvSat[1]));
                telemetry.addLine(String.format("Val: [%s, %s]", hsvVal[0], hsvVal[1]));
                telemetry.addData("Contours size", contours.size());
                telemetry.addLine();
                telemetry.addData("rectTop", rectTop);
                telemetry.addData("rectLeft", rectLeft);
                telemetry.addData("rectBot", rectBot);
                telemetry.addData("rectRight", rectRight);
                telemetry.addLine();
                telemetry.addData("leftBound", leftBound);
                telemetry.addData("centerBound", centerBound);
                telemetry.addLine();
                telemetry.addData("numContoursInRect",  numContoursInRect);
                telemetry.addData("numContoursLeft",    numContoursLeft);
                telemetry.addData("numContoursCenter",  numContoursCenter);
                telemetry.addData("numContoursRight",   numContoursRight);
                telemetry.addLine();
                telemetry.addData("skystonePlacement", skystonePlacement);
                telemetry.update();
        }

        // Auto program starts here

        switch(skystonePlacement) {
            case LEFT:
                runLeft();
                break;
            case CENTER:
                runCenter();
                break;
            case RIGHT:
                runRight();
                break;
        }


        while(opModeIsActive()) {
            telemetry.addLine("Running");
            telemetry.update();
        }
    }


    //----------------------------------------------------------------------------------------------
    // Path methods
    //----------------------------------------------------------------------------------------------

    public void runLeft() throws InterruptedException {
        telemetry.addLine("Running LEFT");
        telemetry.update();
        sleep(1000);

        // Drive away from wall
        driveInches(5.0, 0.4);

        // Lower arm and open clamp
        moveArmCounts(hardware.ARM_COUNTS_DEPLOY, 0.3);
        hardware.clamp.setPosition(0.0);
        sleep(500);

        // Turn towards left stone
        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID(10);
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);


        // Drive to Skystone
        driveInches(31.0,0.4);

        // Turn towards left stone
        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID((int)(heading() + 15));
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        // Close clamp
        hardware.clamp.setPosition(1.0);
        sleep(500);

        // Drive back before turning
        driveInches(-7.0, 0.4);

        // Bring arm back inside the robot
        moveArmCounts(-hardware.ARM_COUNTS_DEPLOY, 0.3);

        // Turn to go under the bridge
        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID(-90);
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID(-90);
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        // Drive parallel to Foundation
        driveInches(93.0, 0.8);

        // Turn towards Foundation
        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID(0);
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        // Drive into Foundation
        driveInches(8.0, 0.4);

        // Place Skystone
        moveArmCounts(hardware.ARM_COUNTS_DEPLOY + 100, 0.3);   // Adding 300 to ARM_COUNTS_DEPLOY because negative is out of the robot
        hardware.clamp.setPosition(0.0);
        sleep(500);
        moveArmCounts(-(hardware.ARM_COUNTS_DEPLOY + 100), 0.3);

        // Drive away from Foundation
        driveInches(-8.0, 0.3);

        // Turn towards bridge
        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID(90);
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID(90);
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        // Park
        driveInches(47.0, 0.8);

        hardware.clamp.setPosition(1.0);
    }

    public void runCenter() throws InterruptedException {
        telemetry.addLine("Running CENTER");
        telemetry.update();
        sleep(1000);

        // Drive away from wall
        driveInches(5.0, 0.4);

        // Lower arm and open clamp
        moveArmCounts(hardware.ARM_COUNTS_DEPLOY, 0.3);
        hardware.clamp.setPosition(0.0);
        sleep(500);

        // Test will show if turning is necessary for center stone
        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID(0);
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);


        // Drive to Skystone
        driveInches(31.0,0.4);


        // Turn towards left stone
        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID((int)(heading() + 10));
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        // Close clamp
        hardware.clamp.setPosition(1.0);
        sleep(500);

        // Drive back before turning
        driveInches(-7.0, 0.4);

        // Bring arm back inside the robot
        moveArmCounts(-hardware.ARM_COUNTS_DEPLOY, 0.3);

        // Turn to go under the bridge
        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID(-90);
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID(-90);
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        // Drive parallel to Foundation
        driveInches(93.0, 0.8);

        // Turn towards Foundation
        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID(0);
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        // Drive into Foundation
        driveInches(8.0, 0.4);

        // Place Skystone
        moveArmCounts(hardware.ARM_COUNTS_DEPLOY + 100, 0.3);   // Adding 300 to ARM_COUNTS_DEPLOY because negative is out of the robot
        hardware.clamp.setPosition(0.0);
        sleep(500);
        moveArmCounts(-(hardware.ARM_COUNTS_DEPLOY + 100), 0.3);

        // Drive away from Foundation
        driveInches(-5.0, 0.3);

        // Turn towards bridge
        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID(90);
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID(90);
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        // Park
        driveInches(51.0, 0.8);

        hardware.clamp.setPosition(1.0);
    }

    public void runRight() throws InterruptedException {
        telemetry.addLine("Running RIGHT");
        telemetry.update();
        sleep(1000);

        // Drive away from wall
        driveInches(5.0, 0.4);

        // Lower arm and open clamp
        moveArmCounts(hardware.ARM_COUNTS_DEPLOY, 0.3);
        hardware.clamp.setPosition(0.0);
        sleep(500);

        // Turn towards left stone
        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID(-13);
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        // Drive to Skystone
        driveInches(31.0,0.4);

        // Turn towards left stone
        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID((int)(heading() + 10));
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        // Close clamp
        hardware.clamp.setPosition(1.0);
        sleep(500);

        // Drive back before turning
        driveInches(-7.0, 0.4);

        // Bring arm back inside the robot
        moveArmCounts(-hardware.ARM_COUNTS_DEPLOY, 0.3);

        // Turn to go under the bridge
        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID(-90);
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID(-90);
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        // Drive parallel to Foundation
        driveInches(93.0, 0.8);

        // Turn towards Foundation
        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID(0);
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        // Drive into Foundation
        driveInches(8.0, 0.4);

        // Place Skystone
        moveArmCounts(hardware.ARM_COUNTS_DEPLOY + 100, 0.3);   // Adding 300 to ARM_COUNTS_DEPLOY because negative is out of the robot
        hardware.clamp.setPosition(0.0);
        sleep(500);
        moveArmCounts(-(hardware.ARM_COUNTS_DEPLOY + 100), 0.3);

        // Drive away from Foundation
        driveInches(-8.0, 0.3);

        // Turn towards bridge
        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID(90);
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID(90);
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        // Park
        driveInches(55.0, 0.8);

        hardware.clamp.setPosition(1.0);
    }


    //----------------------------------------------------------------------------------------------
    // End path methods
    //----------------------------------------------------------------------------------------------


    //----------------------------------------------------------------------------------------------
    // Movement methods
    //----------------------------------------------------------------------------------------------

    public void turnToHeadingPID(int target) throws InterruptedException {

        telemetry.addData("Turning to target", target);
        telemetry.addLine("Press dpad_down to stop.");

        hardware.pid.setSetpoint(target);                                       // Set target final heading relative to current
        //hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);   // Set maximum motor power
        hardware.pid.setDeadband(hardware.TOLERANCE);                           // Set how far off you can safely be from your target

        while (opModeIsActive()) {
            double error = normalize180(target - heading());
            double power = hardware.pid.calculateGivenError(error);

            telemetry.addData("Current error", error);
            telemetry.addData("Current power", power);

            hardware.setLeftPower(-power);
            hardware.setRightPower(power);

            if (Math.abs(error) < hardware.TOLERANCE || gamepad2.dpad_down) {
                break;
            }

            Thread.sleep(1);

            telemetry.update();
        }

        hardware.setLeftPower(0);
        hardware.setRightPower(0);
    }

    public double normalize180(double angle) {
        while(angle > 180) {
            angle -= 360;
        }
        while(angle <= -180) {
            angle += 360;
        }
        return angle;
    }

    public float heading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }



    public void moveArmCounts(int counts, double speed) {
        hardware.arm.setTargetPosition (hardware.arm.getCurrentPosition() + counts);

        hardware.arm.setMode  (DcMotor.RunMode.RUN_TO_POSITION);

        hardware.arm.setPower(speed);

        double timeout = getRuntime();
        while(opModeIsActive() && hardware.arm.isBusy() && getRuntime() - timeout < 2.0) {
            telemetry.addData("Arm encoder", hardware.arm.getCurrentPosition());
            telemetry.update();
        }

        hardware.arm.setPower(0.0);

        hardware.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void driveInches(double inches, double speed) {
        driveEncoderCounts((int) (inches * hardware.COUNTS_PER_INCH), speed);

    }

    public void driveEncoderCounts(int counts, double speed) {
        hardware.frontLeft.setTargetPosition    (hardware.frontLeft.getCurrentPosition() + counts);
        hardware.frontRight.setTargetPosition   (hardware.frontRight.getCurrentPosition() + counts);
        hardware.rearLeft.setTargetPosition     (hardware.rearLeft.getCurrentPosition() + counts);
        hardware.rearRight.setTargetPosition    (hardware.rearRight.getCurrentPosition() + counts);

        hardware.frontLeft.setMode  (DcMotor.RunMode.RUN_TO_POSITION);
        hardware.frontRight.setMode (DcMotor.RunMode.RUN_TO_POSITION);
        hardware.rearLeft.setMode   (DcMotor.RunMode.RUN_TO_POSITION);
        hardware.rearRight.setMode  (DcMotor.RunMode.RUN_TO_POSITION);

        hardware.setLeftPower(speed);
        hardware.setRightPower(speed);

        while(opModeIsActive() &&
                hardware.frontLeft.isBusy() &&
                hardware.frontRight.isBusy() &&
                hardware.rearLeft.isBusy() &&
                hardware.rearRight.isBusy()) {
            telemetry.addData("Front left encoder", hardware.frontLeft.getCurrentPosition());
            telemetry.addData("Front right encoder", hardware.frontRight.getCurrentPosition());
            telemetry.update();
        }

        hardware.setLeftPower(0.0);
        hardware.setRightPower(0.0);

        hardware.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }



    //----------------------------------------------------------------------------------------------
    // End movement methods
    //----------------------------------------------------------------------------------------------


    //----------------------------------------------------------------------------------------------
    // Vision processing methods
    //----------------------------------------------------------------------------------------------

    public double trim(double input, double min, double max) {
        if(input < min) input = min;
        if(input > max) input = max;
        return input;
    }


    public SkystonePlacement compareAreaTallies(double tallyLeft, double tallyCenter, double tallyRight) {
        // Tally counts the area contained by yellow blobs in each third of the screen
        if(tallyLeft < tallyCenter &&
                tallyLeft < tallyRight)     return LEFT;    // Skystone is in the left position
        if(tallyCenter < tallyLeft &&
                tallyCenter < tallyRight)   return CENTER;  // Skystone is in the center position
        if(tallyRight < tallyLeft &&
                tallyRight < tallyCenter)   return RIGHT;   // Skystone is in the right position

        return CENTER;                                      // Default case
    }




    /**
     * SkystonePatternPipeline class.
     *
     * <p>An OpenCV pipeline generated by GRIP.
     *
     * @author GRIP
     */
    private static class SkystonePatternPipeline extends OpenCvPipeline {


        //Outputs
        private Mat hsvThresholdOutput = new Mat();
        private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
        private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();

        @Override
        public Mat processFrame(Mat input) {

            // Step HSV_Threshold0:
            Mat hsvThresholdInput = input;

            if(drawRect) {
                Imgproc.rectangle(
                        input,
                        new Point(  // Top left corner
                                rectLeft,   // Left value
                                rectTop),   // Top value
                        new Point( // Bottom right corner
                                rectRight,  // Right value
                                rectBot),   // Bottom value
                        new Scalar(0, 255, 0), 4);
            }
            Imgproc.line(
                    input,
                    new Point(
                            leftBound,
                            rectBot),
                    new Point(
                            leftBound,
                            rectTop),
                    new Scalar(0, 0, 255), 4);
            Imgproc.line(
                    input,
                    new Point(
                            centerBound,
                            rectBot),
                    new Point(
                            centerBound,
                            rectTop),
                    new Scalar(255, 0, 0), 4);

            double[] hsvThresholdHue =          hsvHue;
            double[] hsvThresholdSaturation =   hsvSat;
            double[] hsvThresholdValue =        hsvVal;
            hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);


            // Step Find_Contours0:
            Mat findContoursInput = hsvThresholdOutput;
            boolean findContoursExternalOnly = false;
            findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

            // Step Filter_Contours0:
            ArrayList<MatOfPoint> filterContoursContours = findContoursOutput;
            double filterContoursMinArea = 100.0;
            double filterContoursMinPerimeter = 0.0;
            double filterContoursMinWidth = 0.0;
            double filterContoursMaxWidth = 2.147483647E9;
            double filterContoursMinHeight = 0.0;
            double filterContoursMaxHeight = 2.147483647E9;
            double[] filterContoursSolidity = {0, 100};
            double filterContoursMaxVertices = 2.147483647E9;
            double filterContoursMinVertices = 0.0;
            double filterContoursMinRatio = 0.0;
            double filterContoursMaxRatio = 2.147483647E9;
            filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);



            return (returnHSV ? hsvThresholdOutput : input);
        }


        /**
         * This method is a generated getter for the output of a HSV_Threshold.
         * @return Mat output from HSV_Threshold.
         */
        public Mat hsvThresholdOutput() {
            return hsvThresholdOutput;
        }

        /**
         * This method is a generated getter for the output of a Find_Contours.
         * @return ArrayList<MatOfPoint> output from Find_Contours.
         */
        public ArrayList<MatOfPoint> findContoursOutput() {
            return findContoursOutput;
        }

        /**
         * This method is a generated getter for the output of a Filter_Contours.
         * @return ArrayList<MatOfPoint> output from Filter_Contours.
         */
        public ArrayList<MatOfPoint> filterContoursOutput() {
            return filterContoursOutput;
        }


        /**
         * Segment an image based on hue, saturation, and value ranges.
         *
         * @param input The image on which to perform the HSL threshold.
         * @param hue The min and max hue
         * @param sat The min and max saturation
         * @param val The min and max value
         * @param output The image in which to store the output.
         */
        private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
                                  Mat out) {
            Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
            Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
                    new Scalar(hue[1], sat[1], val[1]), out);
        }

        /**
         * Sets the values of pixels in a binary image to their distance to the nearest black pixel.
         * @param input The image on which to perform the Distance Transform.
         * @param type The Transform.
         * @param maskSize the size of the mask.
         * @param output The image in which to store the output.
         */
        private void findContours(Mat input, boolean externalOnly,
                                  List<MatOfPoint> contours) {
            Mat hierarchy = new Mat();
            contours.clear();
            int mode;
            if (externalOnly) {
                mode = Imgproc.RETR_EXTERNAL;
            }
            else {
                mode = Imgproc.RETR_LIST;
            }
            int method = Imgproc.CHAIN_APPROX_SIMPLE;
            Imgproc.findContours(input, contours, hierarchy, mode, method);
        }


        /**
         * Filters out contours that do not meet certain criteria.
         * @param inputContours is the input list of contours
         * @param output is the the output list of contours
         * @param minArea is the minimum area of a contour that will be kept
         * @param minPerimeter is the minimum perimeter of a contour that will be kept
         * @param minWidth minimum width of a contour
         * @param maxWidth maximum width
         * @param minHeight minimum height
         * @param maxHeight maximimum height
         * @param Solidity the minimum and maximum solidity of a contour
         * @param minVertexCount minimum vertex Count of the contours
         * @param maxVertexCount maximum vertex Count
         * @param minRatio minimum ratio of width to height
         * @param maxRatio maximum ratio of width to height
         */
        private void filterContours(List<MatOfPoint> inputContours, double minArea,
                                    double minPerimeter, double minWidth, double maxWidth, double minHeight, double
                                            maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
                                            minRatio, double maxRatio, List<MatOfPoint> output) {
            final MatOfInt hull = new MatOfInt();
            output.clear();
            //operation
            for (int i = 0; i < inputContours.size(); i++) {
                final MatOfPoint contour = inputContours.get(i);
                final Rect bb = Imgproc.boundingRect(contour);
                if (bb.width < minWidth || bb.width > maxWidth) continue;
                if (bb.height < minHeight || bb.height > maxHeight) continue;
                final double area = Imgproc.contourArea(contour);
                if (area < minArea) continue;
                if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
                Imgproc.convexHull(contour, hull);
                MatOfPoint mopHull = new MatOfPoint();
                mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
                for (int j = 0; j < hull.size().height; j++) {
                    int index = (int)hull.get(j, 0)[0];
                    double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
                    mopHull.put(j, 0, point);
                }
                final double solid = 100 * area / Imgproc.contourArea(mopHull);
                if (solid < solidity[0] || solid > solidity[1]) continue;
                if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)	continue;
                final double ratio = bb.width / (double)bb.height;
                if (ratio < minRatio || ratio > maxRatio) continue;
                output.add(contour);
            }
        }
    }

    //----------------------------------------------------------------------------------------------
    // End vision processing methods
    //----------------------------------------------------------------------------------------------
}
