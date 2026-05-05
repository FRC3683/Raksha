package frc.robot.utils;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.nio.file.Path;
import java.util.Scanner;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;

public class heading_restorer {

    private static AtomicBoolean zerod = new AtomicBoolean(false);
    private static final String tmpdir = System.getProperty("java.io.tmpdir");
    private static File file = Path.of(tmpdir, "heading_rad.bin").toFile(); 

    private static Supplier<Rotation2d> heading_get;

    public static void init(TimedRobot robot, Consumer<Rotation2d> heading_reset, Supplier<Rotation2d> heading_get) {
        heading_restorer.heading_get = heading_get;
        new Thread(() -> {
            while(!zerod.get()) {
                try {
                    Thread.sleep(3000);
                    
                    if(!file.exists()) {
                        // first boot
                        heading_reset.accept( Rotation2d.kZero );
                    } else {
                        // code push, read file and reset to last known heading
                        // TODO: Zaeem said "mmap" (memory mapped file?)
                        var dis = new DataInputStream(new FileInputStream(file));
                        double heading = dis.readDouble();
                        heading_reset.accept( Rotation2d.fromRadians(heading) );
                        dis.close();
                    }
                    Thread.sleep(100);
                    zerod.set(true);
                } catch(Exception e) {}
            }
        }).start();

        robot.addPeriodic(heading_restorer::loop, 0.5);
    }

    public static boolean zerod() {
        return zerod.get();
    }

    private static void loop() {
        // dont store heading if we haven't initialized yet
        // don't override the file we're trying to read
        if(!zerod.get()) {
            return;
        }
        // dont waste time remembering heading if on a real field,
        // only care about maintaining heading when deploying code
        if(DriverStation.isFMSAttached()) {
            return;
        }

        try {
            if(file.canWrite()) {
                var dos = new DataOutputStream(new FileOutputStream(file));
                dos.writeDouble( heading_get.get().getRadians() );
                dos.close();
            }
        } catch(Exception e) {}

    }
}
