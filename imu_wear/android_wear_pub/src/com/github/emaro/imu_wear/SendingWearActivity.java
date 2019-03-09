package com.github.emaro.imu_wear;

import android.bluetooth.BluetoothAdapter;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;

import android.view.View;
import android.widget.EditText;
import android.widget.TextView;

import org.ros.android.RosWearActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

public class SendingWearActivity extends RosWearActivity implements SensorEventListener {

  BluetoothAdapter myDevice = BluetoothAdapter.getDefaultAdapter();
  String deviceName = myDevice.getName().replaceAll(" ", "_");
  private ImuPublisher pub = new ImuPublisher(deviceName+"/imu_data");
  private EditText frequency;
  private SensorManager senSensorManager;

  public SendingWearActivity() {
    super("IMU Wear", "IMU Wear");
  }

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_sending);

    setAmbientEnabled();
    frequency = findViewById(R.id.frequency);;
  }

  /**
   * Function triggered when new sensors data should be processed
   */
  @Override
  public void onSensorChanged(SensorEvent sensorEvent) {
    Sensor mySensor = sensorEvent.sensor;

    if (mySensor.getType() == Sensor.TYPE_GYROSCOPE) {
      System.arraycopy(sensorEvent.values, 0, pub.vel, 0, 3);
      pub.android_time = sensorEvent.timestamp;

      String frequency_string = frequency.getText().toString();
      if (!frequency_string.isEmpty()) {
        pub.frequency = Integer.parseInt(frequency_string);
      }
    }

    if (mySensor.getType() == Sensor.TYPE_ACCELEROMETER) {
      System.arraycopy(sensorEvent.values, 0, pub.acc, 0, 3);
      pub.android_time = sensorEvent.timestamp;
    }
  }

  @Override
  public void onAccuracyChanged(Sensor sensor, int i) {
  }

  @Override
  protected void init(NodeMainExecutor nodeMainExecutor) {

    NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(getRosHostname());
    nodeConfiguration.setMasterUri(getMasterUri());
    nodeConfiguration.setNodeName("IMU");
    nodeMainExecutor.execute(pub, nodeConfiguration);

    senSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);

    Sensor senAccelerometer = senSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
    senSensorManager.registerListener(this, senAccelerometer, SensorManager.SENSOR_DELAY_FASTEST);

    Sensor senGyroscope = senSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
    senSensorManager.registerListener(this, senGyroscope, SensorManager.SENSOR_DELAY_FASTEST);
  }

  public void stopStreaming(View view) {
    senSensorManager.unregisterListener(this);
    // Exit the app if Stop is pressed
    finish();
    System.exit(0);
  }
}
