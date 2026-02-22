package org.firstinspires.ftc.teamcode.blucru.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.opmodes.Tele;
import org.firstinspires.ftc.teamcode.blucru.opmodes.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.blucru.opmodes.BluLinearOpMode;


@Autonomous(name = "Auto", group = "1")
public class Auto extends BluLinearOpMode {
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
    StateMachine sm = new StateMachineBuilder()
            .state(State.ALLIANCE_PICK,
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
            .transition(() -> driver1.pressedRightBumper()),
                    State.AUTO_PICK, () -> {
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
                    }
        .transition(() -> driver1.pressedRightBumper()),
    State.INITAlIZE, () -> {

        telemetry.addLine("Building Paths . . .");
        telemetry.update();
    }
    .transition(() -> (!AutoConfig.InitBusy)
    State.INITIALIZED, () -> {
        telemetry.addLine("Paths Built!");
        telemetry.addLine("Congrats, do a dance!");
        telemetry.update();
    }


    @Override
    public void initialize() {
        sm.setState(State.ALLIANCE_PICK);
        sm.start();
    }
    public void onStart() {
        if(selectedauto) {
            sm.setState(State.RUNNING);
        } else {
            throw new RuntimeException("Auto not selected! You silly billy!");
        }
    }

    public void initLoop() {
    }

    public void periodic() {
        AutoConfig.Periodic();
    }

    public void runOpMode() {
        AutoConfig.runOpMode();
    }

    @Override
    public void telemetry() {
    }

//    @Override
//    public void end() {
//        if(Globals.alliance == Alliance.RED) DriveBase.startPose = dt.pose;
//        else DriveBase.startPose = new Pose2d(dt.pose.vec(), Angle.norm(dt.pose.getHeading() + Math.PI));
//        Log.i("Auto", "start pose set to" + DriveBase.startPose);
//    }

    public void configTelemetry() {
    }
}
