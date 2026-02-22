package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;


@Autonomous(name = "Auto", group = "1")
public class Auto extends BluLinearOpMode {
    public boolean selectedauto = false;
    public BaseAuto autoToRun;

    enum State {
        ALLIANCE_PICK,
        AUTO_PICK,
        INITAlIZE,
        INITIALIZED,
        RUNNING,
        RESETTING
    }

    enum AUTOSTARTINGPOS {
        FAR,
        CLOSE
    }
    Alliance CurrentSelectedAlliance = Alliance.BLUE;
    AUTOSTARTINGPOS CurrentSelectedAuto = AUTOSTARTINGPOS.CLOSE;
    public StateMachine sm;

    @Override
    public void initialize() {
        robot.clear();
        addSixWheel();
        addIntake();
        addElevator();
        addShooter();
        addTurret();
        addTransfer();
        addLLTagDetector();

        sm = new StateMachineBuilder()
                .state(State.ALLIANCE_PICK)
                .loop(() -> {
                    telemetry.addLine("Press Right Bumper to Confirm Selection! >.<");
                    if(CurrentSelectedAlliance == Alliance.BLUE) {
                        telemetry.addLine("BLUE <--");
                        telemetry.addLine("RED");
                    } else if (CurrentSelectedAlliance == Alliance.RED) {
                        telemetry.addLine("BLUE");
                        telemetry.addLine("RED <--");
                    }
                    if(driver1.pressedDpadDown()) {
                        if(CurrentSelectedAlliance == Alliance.BLUE) CurrentSelectedAlliance = Alliance.RED;
                    } else if (driver1.pressedDpadUp()) {
                        if (CurrentSelectedAlliance == Alliance.RED)
                            CurrentSelectedAlliance = Alliance.BLUE;
                    }
                    telemetry.update();
                })
                .transition(() -> driver1.pressedRightBumper(), State.AUTO_PICK)

                .state(State.AUTO_PICK)
                .loop(() -> {
                    telemetry.addData("Alliance", CurrentSelectedAlliance);
                    telemetry.addLine("Press Right Bumper to Confirm Selection! >.<");
                    if(CurrentSelectedAlliance == Alliance.BLUE) {
                        if(CurrentSelectedAuto == AUTOSTARTINGPOS.CLOSE) {
                            telemetry.addLine("Blue Close <--");
                            telemetry.addLine("Blue FAR");
                        } else if (CurrentSelectedAuto == AUTOSTARTINGPOS.FAR) {
                            telemetry.addLine("Blue Close");
                            telemetry.addLine("Blue Far <--");
                        }
                    } else if (CurrentSelectedAlliance == Alliance.RED) {
                        if(CurrentSelectedAuto == AUTOSTARTINGPOS.CLOSE) {
                            telemetry.addLine("RedClose <--");
                            telemetry.addLine("Red FAR");
                        } else if (CurrentSelectedAuto == AUTOSTARTINGPOS.FAR) {
                            telemetry.addLine("Red Close");
                            telemetry.addLine("Red Far <--");
                        }
                    }
                    if(driver1.pressedDpadDown()) {
                        if(CurrentSelectedAuto == AUTOSTARTINGPOS.CLOSE) CurrentSelectedAuto = AUTOSTARTINGPOS.FAR;
                    } else if (driver1.pressedDpadUp()) {
                        if (CurrentSelectedAuto == AUTOSTARTINGPOS.FAR) CurrentSelectedAuto = AUTOSTARTINGPOS.CLOSE;
                    }
                    telemetry.update();
                })
                .transition(() -> driver1.pressedRightBumper(), State.INITAlIZE)

                .state(State.INITAlIZE)
                .onEnter(() -> {
                    selectedauto = true;
                    telemetry.addLine("Building Paths . . .");
                    telemetry.update();
                    AutoConfig.AUTOS autoEnum = null;
                    if (CurrentSelectedAlliance == Alliance.BLUE) {
                        if (CurrentSelectedAuto == AUTOSTARTINGPOS.CLOSE) autoEnum = AutoConfig.AUTOS.BLUE_CLOSE;
                        else autoEnum = AutoConfig.AUTOS.BLUE_FAR;
                    } else {
                        if (CurrentSelectedAuto == AUTOSTARTINGPOS.CLOSE) autoEnum = AutoConfig.AUTOS.RED_CLOSE;
                        else autoEnum = AutoConfig.AUTOS.RED_FAR;
                    }

                    // Instantiate selected auto
                    autoToRun = AutoConfig.getAutoInstance(autoEnum);

                    if (autoToRun != null) {
                        // Inject hardware references
                        autoToRun.hardwareMap = Auto.this.hardwareMap;
                        autoToRun.telemetry = Auto.this.telemetry;
                        autoToRun.gamepad1 = Auto.this.gamepad1;
                        autoToRun.gamepad2 = Auto.this.gamepad2;
                        autoToRun.robot = Auto.this.robot;
                        autoToRun.sixWheel = Auto.this.sixWheel;
                        autoToRun.intake = Auto.this.intake;
                        autoToRun.elevator = Auto.this.elevator;
                        autoToRun.shooter = Auto.this.shooter;
                        autoToRun.turret = Auto.this.turret;
                        autoToRun.transfer = Auto.this.transfer;
                        autoToRun.obeliskTagDetector = Auto.this.obeliskTagDetector;
                        autoToRun.llTagDetector = Auto.this.llTagDetector;
                        autoToRun.driver1 = Auto.this.driver1;
                        autoToRun.driver2 = Auto.this.driver2;

                        autoToRun.initialize();
                    }
                })
                .transition(() -> !AutoConfig.InitBusy(), State.INITIALIZED)

                .state(State.INITIALIZED)
                .loop(() -> {
                    telemetry.addLine("Paths Built!");
                    telemetry.addLine("Congrats, do a dance!");
                    telemetry.update();
                })

                .state(State.RUNNING)
                .onEnter(() -> {
                    if (autoToRun != null) {
                        autoToRun.onStart();
                    }
                })
                .loop(() -> {
                    if (autoToRun != null) {
                        autoToRun.periodic();
                    }
                })
                .build();

        sm.setState(State.ALLIANCE_PICK);
        sm.start();
    }
    
    @Override
    public void initializePeriodic() {
        if (sm != null) sm.update();
    }

    public void onStart() {
        if(selectedauto) {
            sm.setState(State.RUNNING);
        } else {
            throw new RuntimeException("Auto not selected! You silly billy!");
        }
    }
}
