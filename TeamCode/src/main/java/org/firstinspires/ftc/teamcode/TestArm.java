package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp (name = "Test: Arm & Lift Presets", group = "Testing")
public class TestArm extends OpMode {
    SLICBotHardware hardware = new SLICBotHardware();

    ButtonCooldown gp2_a    = new ButtonCooldown();
    ButtonCooldown gp2_lb   = new ButtonCooldown();
    ButtonCooldown gp2_rb   = new ButtonCooldown();
    ButtonCooldown gp2_y    = new ButtonCooldown();


    public final int ARM_STOWED         = 0;
    public final int ARM_GRABBING       = -1200;
    public final int ARM_PLACING_LOW    = -1200;
    public final int ARM_PLACING_HIGH   = -900;

    public final int LIFT_STOWED    = 0;
    public final int LIFT_1         = 3468;
    public final int LIFT_2         = 7951;
    public final int LIFT_3         = 5879;
    public final int LIFT_4         = 9642;

    public int liftStep = -1;   // Default state (cannot be reached during driver control)
    public int armStep  = -1;   // Default state (cannot be reached during driver control)


    public final double ANALOG_THRESHOLD = 0.05;



    @Override
    public void init() {
        hardware.init(hardwareMap);

        gp2_a   .setCooldown(1.000);
        gp2_lb  .setCooldown(1.000);
        gp2_rb  .setCooldown(1.000);
        gp2_y   .setCooldown(1.000);

        hardware.arm.setTargetPosition(hardware.arm.getCurrentPosition()); // Default target
        hardware.pulley.setTargetPosition(hardware.pulley.getCurrentPosition()); // Default target

        hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        telemetry.addLine("Ready");
        telemetry.update();
    }


    @Override
    public void loop() {
        double runtime = getRuntime();


        // Position cycling
        if(gamepad2.right_bumper && gp2_rb.ready(runtime) && liftStep < 4) {
            liftStep ++;
            gp2_rb.updateSnapshot(runtime);
        }
        if(gamepad2.left_bumper && gp2_lb.ready(runtime) && liftStep > 0) {
            liftStep --;
            gp2_lb.updateSnapshot(runtime);
        }

        if(gamepad2.y && gp2_y.ready(runtime)) {
            armStep ++;
            armStep %= 3;
            gp2_y.updateSnapshot(runtime);
        }

        int armPlacingVal = ( liftStep <= 2 ? ARM_PLACING_LOW : ARM_PLACING_HIGH);
        // End position cycling


        if(gamepad2.left_bumper || gamepad2.right_bumper) {
            hardware.pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            switch (liftStep) {
                case -1:
                    hardware.pulley.setTargetPosition(hardware.pulley.getCurrentPosition());
                    break;
                case 0:
                    hardware.pulley.setTargetPosition(LIFT_STOWED);
                    break;
                case 1:
                    hardware.pulley.setTargetPosition(LIFT_1);
                    break;
                case 2:
                    hardware.pulley.setTargetPosition(LIFT_2);
                    break;
                case 3:
                    hardware.pulley.setTargetPosition(LIFT_3);
                    break;
                case 4:
                    hardware.pulley.setTargetPosition(LIFT_4);
                    break;
                default:
                    break;
            }
            hardware.pulley.setPower(
                    (hardware.pulley.getTargetPosition() - hardware.pulley.getCurrentPosition()) > 0.0 ?
                    1.0 : 0.3);
        } else if(Math.abs(gamepad2.right_trigger - gamepad2.left_trigger) > ANALOG_THRESHOLD){
            hardware.pulley.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.pulley.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
            hardware.pulley.setTargetPosition(hardware.pulley.getCurrentPosition());
        } else {
            hardware.pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.pulley.setPower(
                    (hardware.pulley.getTargetPosition() - hardware.pulley.getCurrentPosition()) > 0.0 ?
                            1.0 : 0.3);
        }

        if(gamepad2.y) {
            hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            switch (armStep) {
                case -1:
                    hardware.arm.setTargetPosition(hardware.arm.getCurrentPosition());
                    break;
                case 0:
                    hardware.arm.setTargetPosition(ARM_STOWED);
                    break;
                case 1:
                    hardware.arm.setTargetPosition(ARM_GRABBING);
                    break;
                case 2:
                    hardware.arm.setTargetPosition(armPlacingVal);
                    break;
                default:
                    break;
            }
            hardware.arm.setPower(0.4);
        } else if(Math.abs(gamepad2.left_stick_y) > ANALOG_THRESHOLD) {
            hardware.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.arm.setPower(gamepad2.left_stick_y * 0.4);
            hardware.arm.setTargetPosition(hardware.arm.getCurrentPosition());
        } else {
            hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.arm.setPower(0.3);
        }




        if(gamepad1.x) {
            resetEncoder(hardware.arm);
        }
        if(gamepad1.y) {
            resetEncoder(hardware.pulley);
        }


        if (gamepad2.a && gp2_a.ready(runtime)) {
             hardware.clamp.setPosition(Math.abs(hardware.clamp.getPosition() - 1));
            gp2_a.updateSnapshot(runtime);
        }
//
//        hardware.arm.setPower(gamepad2.left_stick_y * 0.3);
//
//        double pulleySpeed = gamepad2.right_trigger - gamepad2.left_trigger;
//
//        hardware.pulley.setPower(pulleySpeed);

        telemetry.addData("Arm step", armStep);
        telemetry.addData("Lift step", liftStep);
        telemetry.addLine();
        telemetry.addData("Arm Position", hardware.arm.getCurrentPosition());
        telemetry.addData("Lift Position", hardware.pulley.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("Arm run mode", hardware.arm.getMode());
        telemetry.addData("Lift run mode", hardware.pulley.getMode());
        telemetry.update();


    }

    public void resetEncoder(DcMotor motor){
        DcMotor.RunMode runMode = motor.getMode();
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(runMode);
    }

}
