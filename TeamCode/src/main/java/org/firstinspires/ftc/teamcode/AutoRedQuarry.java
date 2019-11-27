package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Disabled
@Autonomous(name = "Red Quarry", group = "Autonomous")
public class AutoRedQuarry extends LinearOpMode {


    // Hardware class
    SLICBotHardware hardware = new SLICBotHardware();

    // IMU
    BNO055IMU imu;


    @Override public void runOpMode() throws InterruptedException {

        telemetry.addLine("Initializing hardware");
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        hardware.init(hardwareMap);


        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        telemetry.addLine("Step 1");
        telemetry.update();
//        sleep(1000);

        //------------------------
        driveInches(5.0, 0.4);
        //------------------------

        hardware.clamp.setPosition(0.0);
        moveArmCounts(hardware.ARM_COUNTS_DEPLOY, 0.3);

        telemetry.addLine("Step 2");
        telemetry.update();
//        sleep(1000);

        //------------------------
        driveInches(32.0, 0.4);
        //------------------------

        telemetry.addLine("Step 3");
        telemetry.update();
//        sleep(1000);

        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID(15);
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        hardware.clamp.setPosition(1.0);
        sleep(500);


        telemetry.addLine("Step 4");
        telemetry.update();
//        sleep(1000);

        moveArmCounts(-hardware.ARM_COUNTS_DEPLOY, 0.3);

        telemetry.addLine("Step 5");
        telemetry.update();
//        sleep(1000);

        driveInches(-6.0, 0.4);

        telemetry.addLine("Step 6");
        telemetry.addData("Heading", heading());
        telemetry.update();
//        sleep(1000);

        turnToHeadingPID(-90);

        sleep(750);        // Allow time to settle

        turnToHeadingPID(-90);   // Call twice in order to account for occasional errors in heading


        telemetry.addLine("Step 7");
        telemetry.addData("Heading", heading());
        telemetry.update();
//        sleep(1000);

        driveInches(93.0, 0.4);


        telemetry.addLine("Step 8");
        telemetry.update();
//        sleep(1000);

        turnToHeadingPID(0);


        telemetry.addLine("Step 9");
        telemetry.update();
//        sleep(1000);

        driveInches(5.0, 0.4);


        telemetry.addLine("Step 10");
        telemetry.update();
//        sleep(1000);

        moveArmCounts(hardware.ARM_COUNTS_DEPLOY + 200, 0.3);

        sleep(1000);

        hardware.clamp.setPosition(0.0);
        sleep(500);


        telemetry.addLine("Step 11");
        telemetry.update();
//        sleep(1000);

        moveArmCounts(-(hardware.ARM_COUNTS_DEPLOY + 300), 0.3);
        hardware.clamp.setPosition(1.0);


        telemetry.addLine("Step 12");
        telemetry.update();
//        sleep(1000);

        driveInches(-7.0, 0.4);


        telemetry.addLine("Step 13");
        telemetry.update();
//        sleep(1000);

        turnToHeadingPID(90);
        sleep(750);
        turnToHeadingPID(90);


        telemetry.addLine("Step 14");
        telemetry.update();
//        sleep(1000);

        driveInches(55.0, 0.4);




        while(opModeIsActive()) {
            telemetry.addLine("Finished.");
            telemetry.addData("Heading", heading());
            telemetry.addData("Runtime", getRuntime());
            telemetry.update();
        }

    }




    //----------------------------------------------------------------------------------------------
    // PID controller methods
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

        while(opModeIsActive() && hardware.arm.isBusy()) {
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
}
