import org.firstinspires.ftc.teamcode.blucru.common.hardware.servo.BluServo;
import org.firstinspires.ftc.teamcode.blucru.common.subsytems.BluSubsystem;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Transfer implements BluSubsystem {
    public BluServo servo1;
    public BluServo servo2;
    public BluServo servo3;

    public Transfer(String servo1Name, String servo2Name, String servo3Name) {
        servo1 = new BluServo(servo1Name);
        servo2 = new BluServo(servo2Name);
        servo3 = new BluServo(servo3Name);
    }

    public Transfer(String servo1Name, BluServo.Direction dir1,
                    String servo2Name, BluServo.Direction dir2,
                    String servo3Name, BluServo.Direction dir3) {
        servo1 = new BluServo(servo1Name, dir1);
        servo2 = new BluServo(servo2Name, dir2);
        servo3 = new BluServo(servo3Name, dir3);
    }

    @Override
    public void init() {
        servo1.init();
        servo2.init();
        servo3.init();
    }

    @Override
    public void read() {
        servo1.read();
        servo2.read();
        servo3.read();
    }

    @Override
    public void write() {
        servo1.write();
        servo2.write();
        servo3.write();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        servo1.telemetry();
        servo2.telemetry();
        servo3.telemetry();
    }

    @Override
    public void reset() {
        servo1.setPos(0);
        servo2.setPos(0);
        servo3.setPos(0);
        servo1.write();
        servo2.write();
        servo3.write();
    }
}
