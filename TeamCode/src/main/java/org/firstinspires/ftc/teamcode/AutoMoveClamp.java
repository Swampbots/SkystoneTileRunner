package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "Move clamp", group = "Autonomous")
public class AutoMoveClamp extends LinearOpMode {

    // Hardware class
    SLICBotHardware hardware = new SLICBotHardware();



    @Override public void runOpMode() throws InterruptedException {
        telemetry.addLine("Move Clamp");
        telemetry.update();

        hardware.init(hardwareMap);

        waitForStart();



        hardware.clamp.setPosition(1.0);
        hardware.clamp.setPosition(0.0);

        telemetry.addLine("Finished");
        telemetry.update();
    }
}
