package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Driver Control", group = "TeleOp")
public class SLICBotTeleOp extends OpMode {

    SLICBotHardware hardware = new SLICBotHardware();

    // Button cooldowns
    ButtonCooldown gp2_a = new ButtonCooldown();
    ButtonCooldown gp2_x = new ButtonCooldown();
    ButtonCooldown gp2_lb   = new ButtonCooldown();
    ButtonCooldown gp2_rb   = new ButtonCooldown();
    ButtonCooldown gp2_y    = new ButtonCooldown();

    public double runtime;


//    public final int ARM_STOWED         = 0;
//    public final int ARM_GRABBING       = -1200;
//    public final int ARM_PLACING_LOW    = -1200;
//    public final int ARM_PLACING_HIGH   = -900;

    public final int LIFT_STOWED    = 0;
    public final int LIFT_1         = 3468;
    public final int LIFT_2         = 7951;
    public final int LIFT_3         = 5879;
    public final int LIFT_4         = 9642;

    public int liftStep = -1;   // Default state (cannot be reached during driver control)
//    public int armStep  = -1;   // Default state (cannot be reached during driver control)


    public final double ANALOG_THRESHOLD = 0.05;

    public void init() {

        hardware.init(hardwareMap);
        gp2_a.setCooldown(1.000);
        gp2_x.setCooldown(1.000);
        gp2_lb  .setCooldown(1.000);
        gp2_rb  .setCooldown(1.000);
        gp2_y   .setCooldown(1.000);

//        hardware.arm.setTargetPosition(hardware.arm.getCurrentPosition()); // Default target
        hardware.pulley.setTargetPosition(hardware.pulley.getCurrentPosition()); // Default target

//        hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.pulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardware.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready");
        telemetry.update();
    }

    public void loop() {

        runtime = getRuntime();



        // Speed Modifiers
        if(gamepad1.left_bumper) hardware.driverSpeedMod = hardware.SLOW;
        else if(gamepad1.right_bumper) hardware.driverSpeedMod = hardware.FAST;
        else hardware.driverSpeedMod = hardware.NORMAL;





//        // Arm and lift position cycling
//        if(gamepad2.right_bumper && gp2_rb.ready(runtime) && liftStep < 4) {
//            liftStep ++;
//            gp2_rb.updateSnapshot(runtime);
//        }
//        if(gamepad2.left_bumper && gp2_lb.ready(runtime) && liftStep > 0) {
//            liftStep --;
//            gp2_lb.updateSnapshot(runtime);
//        }

//        if(gamepad2.y && gp2_y.ready(runtime)) {
//            armStep ++;
//            armStep %= 3;
//            gp2_y.updateSnapshot(runtime);
//        }

//        int armPlacingVal = ( liftStep <= 2 ? ARM_PLACING_LOW : ARM_PLACING_HIGH);
        // End arm and lift position cycling





        double drive = -gamepad1.left_trigger + gamepad1.right_trigger;
        double turn = -gamepad1.left_stick_x * 3 / 4;

        if(drive < 0.0) {
            hardware.setLeftPower((drive + turn) * hardware.driverSpeedMod);
            hardware.setRightPower((drive - turn) * hardware.driverSpeedMod);
        } else {
            hardware.setLeftPower((drive - turn) * hardware.driverSpeedMod);
            hardware.setRightPower((drive + turn) * hardware.driverSpeedMod);
        }


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

//        if(gamepad2.y) {
//            hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            switch (armStep) {
//                case -1:
//                    hardware.arm.setTargetPosition(hardware.arm.getCurrentPosition());
//                    break;
//                case 0:
//                    hardware.arm.setTargetPosition(ARM_STOWED);
//                    break;
//                case 1:
//                    hardware.arm.setTargetPosition(ARM_GRABBING);
//                    break;
//                case 2:
//                    hardware.arm.setTargetPosition(armPlacingVal);
//                    break;
//                default:
//                    break;
//            }
//            hardware.arm.setPower(0.3);
//        } else if(Math.abs(gamepad2.left_stick_y) > ANALOG_THRESHOLD) {
//            hardware.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            hardware.arm.setPower(gamepad2.left_stick_y * 0.3);
//            hardware.arm.setTargetPosition(hardware.arm.getCurrentPosition());
//        } else {
//            hardware.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            hardware.arm.setPower(0.3);
//        }
        hardware.arm.setPower(gamepad2.left_stick_y * 0.3);


        if (gamepad2.a && gp2_a.ready(runtime)) {
            hardware.clamp.setPosition(Math.abs(hardware.clamp.getPosition() - 1));
            gp2_a.updateSnapshot(runtime);
        }

        if (gamepad2.x && gp2_x.ready(runtime)) {
            hardware.foundLeft.setPosition((Math.abs(hardware.foundLeft.getPosition() - 1)));
            hardware.foundRight.setPosition((Math.abs(hardware.foundRight.getPosition() - 1)));
            gp2_x.updateSnapshot(runtime);
        }

//        if(gamepad1.a || gamepad2.a) {
//            setIntakePower(1.0);
//        } else if(gamepad1.b || gamepad2.b) {
//            setIntakePower(-1.0);
//        } else {
//            setIntakePower(0.0);
//        }

        // Deploy servo controls
        if(gamepad2.dpad_down) {        // STOWED
            hardware.deploy2.setPosition(hardware.STOWED[0]);
            hardware.deploy1.setPosition(hardware.STOWED[1]);
        } else if(gamepad2.dpad_left) { // DEPLOYED
            hardware.deploy2.setPosition(hardware.DEPLOYED[0]);
            hardware.deploy1.setPosition(hardware.DEPLOYED[1]);
        } else if(gamepad2.dpad_up) {   // GRABBING
            hardware.deploy2.setPosition(hardware.GRABBING[0]);
            hardware.deploy1.setPosition(hardware.GRABBING[1]);
        }


        telemetry.addData("Drive power", drive);
        telemetry.addData("Turn power", turn);
        telemetry.addLine();
//        telemetry.addData("Intake1 power", hardware.intake1.getPower());
//        telemetry.addData("Intake2 power", hardware.intake2.getPower());
//        telemetry.addLine();
//        telemetry.addData("Deploy1 position", hardware.deploy1.getPosition());
////        telemetry.addData("Deploy2 position", hardware.deploy2.getPosition());
//        telemetry.addData("Clamp position", hardware.clamp.getPosition());
//        telemetry.addLine();
//        telemetry.addData("Pulley position", hardware.pulley.getCurrentPosition());
//        telemetry.addLine();
//        telemetry.addData("foundLeft pos", hardware.foundLeft.getPosition());
//        telemetry.addData("foundRight pos", hardware.foundRight.getPosition());
        telemetry.addData("arm power", hardware.arm.getPower());
        telemetry.update();
    }

    public void setIntakePower(double power) {
        hardware.intake1.setPower(power);
        hardware.intake2.setPower(power);
    }
}
