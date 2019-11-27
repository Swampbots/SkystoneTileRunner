package org.firstinspires.ftc.teamcode;

public class ButtonCooldown {

    private double cooldown = 0.075; // 75 milliseconds
    private double snapshot = 0.0;

    public void updateSnapshot(double snapshot) {
        this.snapshot = snapshot;
    }

    public boolean ready(double runtime) {
        return(runtime - snapshot) > cooldown;
    }

    public void setCooldown(double cooldown) {
        this.cooldown = cooldown;
    }
}