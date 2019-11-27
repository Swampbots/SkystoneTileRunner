package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled
@Autonomous(name = "Test: driveInches()", group = "Testing")
public class TestDriveInches extends LinearOpMode {

    SLICBotHardware hardware = new SLICBotHardware();

    public final double COUNTS_PER_REV_HD_20    = 560; // REV HD Hex 20:1 motor
    public final double DRIVE_GEAR_REDUCTION    = 20.0 / 26.0; // 15 tooth on motor shaft to 15 tooth on wheel shaft
    public final double WHEEL_DI_INCHES         = 90.0 / 25.4; // 90mm diameter wheel divided by 25.4(in/mm)
    public final double COUNTS_PER_INCH         = (COUNTS_PER_REV_HD_20 * DRIVE_GEAR_REDUCTION) / (WHEEL_DI_INCHES * Math.PI);

    public final int ARM_COUNTS_DEPLOY   = -1200;

    public void runOpMode() {

        hardware.init(hardwareMap);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

            if(gamepad1.x) resetDriveEncoders();
            if(gamepad2.y) {
                hardware.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hardware.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if(gamepad1.a) driveInches(24.0, 0.4);
            if(gamepad2.a) moveArmCounts(ARM_COUNTS_DEPLOY, 0.2);


            telemetry.addData("FL encoder", hardware.frontLeft.getCurrentPosition());
            telemetry.addData("FR encoder", hardware.frontRight.getCurrentPosition());
            telemetry.addData("RL encoder", hardware.rearLeft.getCurrentPosition());
            telemetry.addData("RR encoder", hardware.rearRight.getCurrentPosition());

            telemetry.addData("Arm encoder", hardware.arm.getCurrentPosition());
            telemetry.update();
        }

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
        driveEncoderCounts((int) (inches * COUNTS_PER_INCH), speed);

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

    public void resetDriveEncoders() {
        hardware.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hardware.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
