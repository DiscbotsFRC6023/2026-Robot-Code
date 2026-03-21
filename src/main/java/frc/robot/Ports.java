package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
    // CAN Buses
    public static final CANBus kRoboRioCANBus = new CANBus("rio");
    public static final CANBus kCANivoreCANBus = new CANBus("main");

    // Talon FX IDs
    public static final int kIntakePivot = 22;
    public static final int kIntakeRollers = 21;
    public static final int kFloor1 = 31;
    public static final int kFloor2 = 32;
    public static final int kFeeder = 42;
    public static final int kShooterLeft = 51;
    public static final int kShooterMiddle = 52;
    public static final int kShooterRight = 53;
    public static final int kHanger = 41;

    // Hood Kraken X44 TalonFX ID (on CANivore "main" bus)
    public static final int kHoodKrakenId = 61;
}