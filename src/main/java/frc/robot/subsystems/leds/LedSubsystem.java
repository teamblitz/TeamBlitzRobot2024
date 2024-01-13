package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Here we can probably do some work, I am not entirely sure if this should follow the io layer
 * system or not. for consistency’s sake I say yes, but we should probably see what 6328's code base
 * does as we are using their logging framework.
 */
public class LedSubsystem extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    public LedSubsystem() {

        led = new AddressableLED(0);

        ledBuffer = new AddressableLEDBuffer(2);

        led.setLength(ledBuffer.getLength());

        led.start();

        ledBuffer.setRGB(0, 200, 200, 200);
        ledBuffer.setRGB(1, 200, 200, 200);
        led.setData(ledBuffer);

        //        new Trigger(() -> DriverStation.isDSAttached() || DriverStation.isFMSAttached())
        //                .onTrue(Commands.waitSeconds(.5).andThen(solidAllianceColorsCommand()));
    }

    public Command coneSolid() {
        return Commands.runOnce(
                () -> {
                    ledBuffer.setRGB(0, 255, 255, 0);
                    ledBuffer.setRGB(1, 255, 255, 0);

                    led.setData(ledBuffer);
                });
    }

    public Command cubeSolid() {
        return Commands.runOnce(
                () -> {
                    ledBuffer.setRGB(0, 255, 0, 255);
                    ledBuffer.setRGB(1, 255, 0, 255);

                    led.setData(ledBuffer);
                });
    }

    public Command coneSlideDrop() {
        return Commands.repeatingSequence(
                Commands.runOnce(
                                () -> {
                                    ledBuffer.setRGB(0, 255, 255, 0);
                                    ledBuffer.setRGB(1, 255, 255, 0);

                                    led.setData(ledBuffer);
                                },
                                this)
                        .andThen(Commands.waitSeconds(.25))
                        .andThen(
                                Commands.runOnce(
                                        () -> {
                                            ledBuffer.setRGB(0, 0, 0, 0);
                                            ledBuffer.setRGB(1, 0, 0, 0);

                                            led.setData(ledBuffer);
                                        },
                                        this))
                        .andThen(Commands.waitSeconds(.25)));
    }

    // public Command cubeSlideDrop() {
    //     return Commands.repeatingSequence(
    //             Commands.runOnce(
    //                             () -> {
    //                                 ledBuffer.setRGB(0, 255, 0, 255);
    //                                 ledBuffer.setRGB(1, 255, 0, 255);

    //                                 led.setData(ledBuffer);
    //                             },
    //                             this)
    //                     .andThen(Commands.waitSeconds(.25))
    //                     .andThen(
    //                             Commands.runOnce(
    //                                     () -> {
    //                                         ledBuffer.setRGB(0, 0, 0, 0);
    //                                         ledBuffer.setRGB(1, 0, 0, 0);

    //                                         led.setData(ledBuffer);
    //                                     },
    //                                     this))
    //                     .andThen(Commands.waitSeconds(.25)));
    // }

    public Command coneLeftShelf() {
        return Commands.runOnce(
                () -> {
                    ledBuffer.setRGB(1, 255, 255, 0);
                    ledBuffer.setRGB(1, 0, 0, 0);

                    led.setData(ledBuffer);
                });
    }

    public Command coneRightShelf() {
        return Commands.runOnce(
                () -> {
                    ledBuffer.setRGB(0, 255, 255, 0);
                    ledBuffer.setRGB(1, 0, 0, 0);

                    led.setData(ledBuffer);
                });
    }

    public Command cubeLeftShelf() {
        return Commands.runOnce(
                () -> {
                    ledBuffer.setRGB(1, 255, 0, 255);
                    ledBuffer.setRGB(1, 0, 0, 0);

                    led.setData(ledBuffer);
                });
    }

    public Command cubeRightShelf() {
        return Commands.runOnce(
                () -> {
                    ledBuffer.setRGB(0, 255, 0, 255);
                    ledBuffer.setRGB(1, 0, 0, 0);

                    led.setData(ledBuffer);
                });
    }

    public Command coneLeftShielfDrop() {
        return Commands.repeatingSequence(
                Commands.runOnce(
                                () -> {
                                    ledBuffer.setRGB(1, 255, 0, 255);

                                    led.setData(ledBuffer);
                                },
                                this)
                        .andThen(Commands.waitSeconds(.25))
                        .andThen(
                                Commands.runOnce(
                                        () -> {
                                            ledBuffer.setRGB(1, 0, 0, 0);

                                            led.setData(ledBuffer);
                                        },
                                        this))
                        .andThen(Commands.waitSeconds(.25)));
    }

    public Command coneRightShielfDrop() {
        return Commands.repeatingSequence(
                Commands.runOnce(
                                () -> {
                                    ledBuffer.setRGB(0, 255, 0, 255);

                                    led.setData(ledBuffer);
                                },
                                this)
                        .andThen(Commands.waitSeconds(.25))
                        .andThen(
                                Commands.runOnce(
                                        () -> {
                                            ledBuffer.setRGB(0, 0, 0, 0);

                                            led.setData(ledBuffer);
                                        },
                                        this))
                        .andThen(Commands.waitSeconds(.25)));
    }

    public Command cubeLeftShielfDrop() {
        return Commands.repeatingSequence(
                Commands.runOnce(
                                () -> {
                                    ledBuffer.setRGB(1, 255, 0, 255);

                                    led.setData(ledBuffer);
                                },
                                this)
                        .andThen(Commands.waitSeconds(.25))
                        .andThen(
                                Commands.runOnce(
                                        () -> {
                                            ledBuffer.setRGB(1, 0, 0, 0);

                                            led.setData(ledBuffer);
                                        },
                                        this))
                        .andThen(Commands.waitSeconds(.25)));
    }

    public Command cubeRightShielfDrop() {
        return Commands.repeatingSequence(
                Commands.runOnce(
                                () -> {
                                    ledBuffer.setRGB(0, 255, 0, 255);

                                    led.setData(ledBuffer);
                                },
                                this)
                        .andThen(Commands.waitSeconds(.25))
                        .andThen(
                                Commands.runOnce(
                                        () -> {
                                            ledBuffer.setRGB(0, 0, 0, 0);

                                            led.setData(ledBuffer);
                                        },
                                        this))
                        .andThen(Commands.waitSeconds(.25)));
    }

    public Command solidAllianceColorsCommand() {
        return Commands.runOnce(
                () -> {
                    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
                        ledBuffer.setRGB(0, 255, 0, 0);
                        ledBuffer.setRGB(1, 255, 0, 0);

                        led.setData(ledBuffer);
                    } else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
                        ledBuffer.setRGB(0, 0, 0, 255);
                        ledBuffer.setRGB(1, 0, 0, 255);
                    }
                });
    }

    public Command cubeSlideDrop() {
        return Commands.repeatingSequence(
                Commands.runOnce(
                                () -> {
                                    ledBuffer.setRGB(0, 255, 0, 255);
                                    ledBuffer.setRGB(1, 255, 0, 255);

                                    led.setData(ledBuffer);
                                },
                                this)
                        .andThen(Commands.waitSeconds(.25))
                        .andThen(
                                Commands.runOnce(
                                        () -> {
                                            ledBuffer.setRGB(0, 0, 0, 0);
                                            ledBuffer.setRGB(1, 0, 0, 0);

                                            led.setData(ledBuffer);
                                        })));
    }
}
