package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static org.firstinspires.ftc.teamcode.SkystonePlacement.CENTER;
import static org.firstinspires.ftc.teamcode.SkystonePlacement.LEFT;
import static org.firstinspires.ftc.teamcode.SkystonePlacement.RIGHT;

@Autonomous(name = "Test: Quarry-Side Paths", group = "Testing")
public class TestQuarryPaths extends LinearOpMode {

    SLICBotHardware hardware = new SLICBotHardware();

    BNO055IMU imu;


    SkystonePlacement placement = CENTER; // Default is center



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


        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {

            if(gamepad1.dpad_left)  placement   = LEFT;
            if(gamepad1.dpad_up)    placement   = CENTER;
            if(gamepad1.dpad_right) placement   = RIGHT;

            if(gamepad1.y) {
                switch(placement) {
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
            }


            telemetry.addLine("Running");
            telemetry.addLine();
            telemetry.addData("Skystone Placement", placement);
            telemetry.update();
        }

    }


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
        driveInches(-8.0, 0.4);

        // Bring arm back inside the robot
        moveArmCounts(-hardware.ARM_COUNTS_DEPLOY, 0.3);

        // Turn to go under the bridge
        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID(90);
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        // Drive parallel to Foundation
        driveInches(93.0, 0.8);

        // Turn towards Foundation
        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID(0);
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        // Drive into Foundation
        driveInches(5.0, 0.4);

        // Place Skystone
        moveArmCounts(hardware.ARM_COUNTS_DEPLOY + 300, 0.3);   // Adding 300 to ARM_COUNTS_DEPLOY because negative is out of the robot
        hardware.clamp.setPosition(0.0);
        sleep(500);
        moveArmCounts(-(hardware.ARM_COUNTS_DEPLOY + 300), 0.3);

        // Drive away from Foundation
        driveInches(-5.0, 0.3);

        // Turn towards bridge
        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID(-90);
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        // Park
        driveInches(55.0, 0.8);
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
        driveInches(-8.0, 0.4);

        // Bring arm back inside the robot
        moveArmCounts(-hardware.ARM_COUNTS_DEPLOY, 0.3);

        // Turn to go under the bridge
        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID(90);
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        // Drive parallel to Foundation
        driveInches(93.0, 0.8);

        // Turn towards Foundation
        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID(0);
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        // Drive into Foundation
        driveInches(5.0, 0.4);

        // Place Skystone
        moveArmCounts(hardware.ARM_COUNTS_DEPLOY + 300, 0.3);   // Adding 300 to ARM_COUNTS_DEPLOY because negative is out of the robot
        hardware.clamp.setPosition(0.0);
        sleep(500);
        moveArmCounts(-(hardware.ARM_COUNTS_DEPLOY + 300), 0.3);

        // Drive away from Foundation
        driveInches(-5.0, 0.3);

        // Turn towards bridge
        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID(-90);
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        // Park
        driveInches(55.0, 0.8);
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
        driveInches(-8.0, 0.4);

        // Bring arm back inside the robot
        moveArmCounts(-hardware.ARM_COUNTS_DEPLOY, 0.3);

        // Turn to go under the bridge
        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID(90);
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        // Drive parallel to Foundation
        driveInches(93.0, 0.8);

        // Turn towards Foundation
        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID(0);
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        // Drive into Foundation
        driveInches(5.0, 0.4);

        // Place Skystone
        moveArmCounts(hardware.ARM_COUNTS_DEPLOY + 300, 0.3);   // Adding 300 to ARM_COUNTS_DEPLOY because negative is out of the robot
        hardware.clamp.setPosition(0.0);
        sleep(500);
        moveArmCounts(-(hardware.ARM_COUNTS_DEPLOY + 300), 0.3);

        // Drive away from Foundation
        driveInches(-5.0, 0.3);

        // Turn towards bridge
        hardware.pid.setOutputRange(-(hardware.MAX_SPEED / 2.0), (hardware.MAX_SPEED / 2.0));
        turnToHeadingPID(-90);
        hardware.pid.setOutputRange(-hardware.MAX_SPEED, hardware.MAX_SPEED);

        // Park
        driveInches(55.0, 0.8);
    }




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
