package org.ros.android.android_wear_pub;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorManager;

import org.ros.concurrent.CancellableLoop;
import org.ros.internal.message.RawMessage;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import java.util.concurrent.Semaphore;

import geometry_msgs.Vector3;
import sensor_msgs.Imu;

public class ImuPublisher extends AbstractNodeMain {
    private String topic_name;

    public Semaphore semaphore = new Semaphore(1);
    public float[] acc = new float[3];
    public float[] vel = new float[3];
    private Publisher<sensor_msgs.Imu> publisher;
    private sensor_msgs.Imu msg;


    public ImuPublisher() {
        this.topic_name = "imu_data";
    }

    public ImuPublisher(String topic) {
        this.topic_name = topic;
    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("IMU/imu_data");
    }

    public void onStart(ConnectedNode connectedNode) {
        publisher = connectedNode.newPublisher(this.topic_name, sensor_msgs.Imu._TYPE);
        msg = publisher.newMessage();

        connectedNode.executeCancellableLoop(new CancellableLoop() {
            protected void setup() {

            }

            protected void loop() throws InterruptedException {
                msg.getLinearAcceleration().setX(acc[0]);
                msg.getLinearAcceleration().setY(acc[1]);
                msg.getLinearAcceleration().setZ(acc[2]);

                msg.getAngularVelocity().setX(vel[0]);
                msg.getAngularVelocity().setY(vel[1]);
                msg.getAngularVelocity().setZ(vel[2]);

                publisher.publish(msg);

                Thread.sleep(100L);
            }
        });
    }
}
