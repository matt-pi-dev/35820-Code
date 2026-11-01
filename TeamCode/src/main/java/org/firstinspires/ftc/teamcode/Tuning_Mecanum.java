package org.firstinspires.ftc.teamcode;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
//import com.qualcomm.robotcore.hardware.HardwareMap;

public class Tuning_Mecanum {
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }
}