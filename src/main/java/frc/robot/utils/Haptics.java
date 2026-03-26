// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class Haptics {
    private CommandXboxController driver;
    private CommandXboxController aux;
    private Notifier pulseNotifier;
    private int pulseCounter;
    private static Haptics instance;

    private Haptics(){
        pulseNotifier = new Notifier(null);
    }

    public static void initialize(CommandXboxController driver, CommandXboxController aux){
        getInstance().driver = driver;
        getInstance().aux = aux;
    }

    public static Haptics getInstance(){
        if(instance == null){
            instance = new Haptics();
        }
        return instance;
    }

    public void rumble(double intensity){
        driver.setRumble(RumbleType.kBothRumble, intensity);
        aux.setRumble(RumbleType.kBothRumble, intensity);
    }

    public void stopRumble(){
        driver.setRumble(RumbleType.kBothRumble, 0);
        aux.setRumble(RumbleType.kBothRumble, 0);
    }

    public void pulse(int pulseCount, double pulseTime, double intensity){
        pulseCounter = 0;

        pulseNotifier.setCallback(() -> {
            if(pulseCounter % 2 == 0){
                rumble(intensity);
            } else {
                stopRumble();
            }

            pulseCounter++;

            if(pulseCounter >= pulseCount * 2){ // One on toggle, one off toggle
                stopRumble();
                pulseNotifier.stop();
            }
        });

        // Stop and restart notifier
        pulseNotifier.stop();
        pulseNotifier.startPeriodic(pulseTime / 2);
    }
}
