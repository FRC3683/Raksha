package frc.robot.utils;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;

public class crt_encoder_pair {

    public record enc_cfg( int can_id, Angle offset, int teeth ) {}

    private final CANcoder enc1, enc2;
    private final int turret_teeth, teeth1, teeth2;
    private final double ratio1, ratio2;
    private final Angle output_offset;

    private final StatusSignal<Angle> abs1_pos, abs2_pos;

    private Angle value = Rotations.of( 0 );
    private boolean enc1_connected = false;
    private boolean enc2_connected = false;

    public crt_encoder_pair( enc_cfg cfg1, enc_cfg cfg2, String canbus, int turret_teeth, Angle output_offset ) {
        enc1 = new CANcoder( cfg1.can_id, canbus );
        enc2 = new CANcoder( cfg2.can_id, canbus );

        enc1.getConfigurator().apply( new CANcoderConfiguration().withMagnetSensor( new MagnetSensorConfigs()
            .withMagnetOffset( cfg1.offset )
            .withAbsoluteSensorDiscontinuityPoint(1)
            .withSensorDirection( SensorDirectionValue.CounterClockwise_Positive )
        ) );
                
        enc2.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor( new MagnetSensorConfigs()
            .withMagnetOffset( cfg2.offset )
            .withAbsoluteSensorDiscontinuityPoint(1)
            .withSensorDirection( SensorDirectionValue.CounterClockwise_Positive )
        ) );

        abs1_pos = enc1.getAbsolutePosition();
        abs2_pos = enc2.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll( 100, abs1_pos, abs2_pos );

        teeth1 = cfg1.teeth;
        teeth2 = cfg2.teeth;
        this.turret_teeth = turret_teeth;
        this.output_offset = output_offset;
        ratio1 = (double) teeth1 / (double) turret_teeth;
        ratio2 = (double) teeth2 / (double) turret_teeth;
    }

    public crt_encoder_pair refresh() {
        enc1_connected = enc1.isConnected();
        enc2_connected = enc2.isConnected();

        BaseStatusSignal.refreshAll( enc1.getAbsolutePosition(), enc2.getAbsolutePosition() );

        double angle1_teeth = enc1_angle().in( Rotations ) * teeth1;
        double angle2_teeth = enc2_angle().in( Rotations ) * teeth2;

        double closest = 0;
        double closest_dist = Double.POSITIVE_INFINITY;
        for( int i = 0; i < teeth2; ++i ) {
            double possibility = ( angle1_teeth + (double)( i * teeth1 ) );
            double dist = MathUtil.inputModulus( angle2_teeth - possibility, -((double)teeth2/2.0), (double)teeth2/2.0 );
            if( Math.abs( dist ) < closest_dist ) {
                closest = possibility;
                closest_dist = Math.abs( dist );
            }
        }
        value = Rotations.of( closest / (double)turret_teeth );

        return this;
    }

    public Angle enc1_angle() {
        return abs1_pos.getValue();
    }

    public Angle enc2_angle() {
        return abs2_pos.getValue();
    }

    public boolean is_enc1_connected() {
        return enc1_connected;
    }

    public boolean is_enc2_connected() {
        return enc2_connected;
    }

    public boolean connected() {
        return enc1_connected && enc2_connected;
    }

    public Angle get_value( ) {
        return value.plus( output_offset );
    }

    
}
