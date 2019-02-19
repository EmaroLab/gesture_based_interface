package org.ros.android.android_wear_pub;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;

import java.net.URI;
import android.view.View;
import android.widget.TextView;

import org.ros.android.RosActivity;
import org.ros.android.android_wear_pub.ImuPublisher;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

public class SendingActivity extends RosActivity implements SensorEventListener {

    public SendingActivity() {
        super("IMU Wear", "IMU Wear", URI.create("http://192.168.43.164:11311"));
    }

    private ImuPublisher pub = new ImuPublisher("imu_data");

    private TextView Text;

    private SensorManager senSensorManager;
    private Sensor senAccelerometer, senGyroscope;

    private String deviceName;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_sending);

        setAmbientEnabled();

        Intent intent = getIntent();
        deviceName = intent.getStringExtra(MainActivity.deviceNamePath);

        TextView TextDevName;
        TextDevName = findViewById(R.id.deviceName);
        TextDevName.setText(deviceName);

        Text = findViewById(R.id.text);
    }

    /** Function triggered when new sensors data should be processed
     */
    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        Sensor mySensor = sensorEvent.sensor;

        if (mySensor.getType() == Sensor.TYPE_GYROSCOPE) {
                for(int i=0; i<3; i++)
                    pub.vel[i]=sensorEvent.values[i];

              Text.setText("Success");
            //dataGyro.setText("gyro: " + pub.r[0] + " " + pub.r[1] + " " + pub.r[2]);
        }

        if (mySensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            for(int i=0; i<3; i++)
                pub.acc[i]=sensorEvent.values[i];
            //dataAcc.setText("acc: " + pub.r1[0] + " " + pub.r1[1] + " " + pub.r[2]);
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {
    }

    public void stopStreaming(View view) {
        senSensorManager.unregisterListener(this);
        Intent intent = new Intent(this, MainActivity.class);
        finish();
        startActivity(intent);
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {

        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(getRosHostname());
        nodeConfiguration.setMasterUri(getMasterUri());
        nodeConfiguration.setNodeName("IMU");
        nodeMainExecutor.execute(pub, nodeConfiguration);

        senSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);

        senAccelerometer = senSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        senSensorManager.registerListener(this, senAccelerometer, SensorManager.SENSOR_DELAY_FASTEST);

        senGyroscope = senSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        senSensorManager.registerListener(this, senGyroscope, SensorManager.SENSOR_DELAY_FASTEST);

    }
}
