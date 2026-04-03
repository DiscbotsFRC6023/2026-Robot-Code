package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
    // CAN Buses
    public static final CANBus kRoboRioCANBus = new CANBus("rio");
    public static final CANBus kCANivoreCANBus = new CANBus("main");

    // Talon FX IDs
    public static final int kIntakePivot = 22;
    public static final int kIntakeRollers = 21;
    public static final int kFloor1 = 41;
    public static final int kFloor2 = 42;
    public static final int kFeeder = 54;
    public static final int kShooterLeft = 51;
    public static final int kShooterRight = 52;
    public static final int kHanger = 61;

    // Hood Kraken X44 TalonFX ID (on CANivore "main" bus)
    public static final int kHoodKrakenId = 53;
}