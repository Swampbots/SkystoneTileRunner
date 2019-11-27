package org.firstinspires.ftc.teamcode;

import java.util.LinkedList;
import java.util.List;

public class GamepadCooldowns {

    ButtonCooldown dpUp     = new ButtonCooldown();
    ButtonCooldown dpDown   = new ButtonCooldown();
    ButtonCooldown dpLeft   = new ButtonCooldown();
    ButtonCooldown dpRight  = new ButtonCooldown();

    ButtonCooldown a    = new ButtonCooldown();
    ButtonCooldown b    = new ButtonCooldown();
    ButtonCooldown x    = new ButtonCooldown();
    ButtonCooldown y    = new ButtonCooldown();

    ButtonCooldown lb   = new ButtonCooldown();
    ButtonCooldown rb   = new ButtonCooldown();
    ButtonCooldown lt   = new ButtonCooldown();
    ButtonCooldown rt   = new ButtonCooldown();

    ButtonCooldown lStickB = new ButtonCooldown();
    ButtonCooldown rStickB = new ButtonCooldown();

    List<ButtonCooldown> buttonCooldowns = new LinkedList<>();

    public GamepadCooldowns() {
        buttonCooldowns.add(dpUp);
        buttonCooldowns.add(dpDown);
        buttonCooldowns.add(dpLeft);
        buttonCooldowns.add(dpRight);

        buttonCooldowns.add(a);
        buttonCooldowns.add(b);
        buttonCooldowns.add(x);
        buttonCooldowns.add(y);

        buttonCooldowns.add(lb);
        buttonCooldowns.add(rb);
        buttonCooldowns.add(lt);
        buttonCooldowns.add(rt);

        buttonCooldowns.add(lStickB);
        buttonCooldowns.add(rStickB);
    }

    public void setCooldown(double cooldown) {
        for(ButtonCooldown button : buttonCooldowns) button.setCooldown(cooldown);
    }




}