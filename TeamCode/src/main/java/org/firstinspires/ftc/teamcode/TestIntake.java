package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "Test Intake", group = "Testing")
public class TestIntake extends OpMode {

    public DcMotor intake1;
    public DcMotor intake2;

    public Servo deploy1;
    public Servo deploy2;

    public void init() {
        intake1 = hardwareMap.dcMotor.get("intake1");
        intake2 = hardwareMap.dcMotor.get("intake2");

        deploy1 = hardwareMap.servo.get("deploy1");
        deploy2 = hardwareMap.servo.get("deploy2");

        intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Ready");
        telemetry.update();
    }

    public void loop() {

        // Intake motor controls
        if(gamepad1.a) setIntakePower(1.0);
        else if(gamepad1.b) setIntakePower(-1.0);
        else setIntakePower(0.0);

        // Deploy servo controls
        deploy1.setPosition(1 - gamepad1.right_trigger);
        deploy2.setPosition(gamepad1.left_trigger);


        telemetry.addLine("Motor telemetry:");
        telemetry.addData("Intake1 power", intake1.getPower());
        telemetry.addData("Intake2 power", intake2.getPower());
        telemetry.addLine();
        telemetry.addLine("Servo telemetry:");
        telemetry.addData("Deploy1 position",deploy1.getPosition());
        telemetry.addData("Deploy2 position",deploy2.getPosition());
        telemetry.update();


    }



    public void setIntakePower(double power) {
        intake1.setPower(power);
        intake2.setPower(power);
    }
}

