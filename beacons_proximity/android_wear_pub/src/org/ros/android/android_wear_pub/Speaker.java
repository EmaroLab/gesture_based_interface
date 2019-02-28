package org.ros.android.android_wear_pub;

import android.util.Log;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

public class Speaker extends AbstractNodeMain {
    private String topicName;
    public boolean presence = false;

    public Speaker() {
        this.topicName = "chatter";
        this.presence = false;
    }

    public Speaker(String topic) {
        this.topicName = topic;
        this.presence = false;
    }

    public GraphName getDefaultNodeName() {
        String nodeName = null;
        if(this.topicName == "/beacon/pink/presence")
            nodeName = "talkerPink";
        if(this.topicName == "/beacon/yellow/presence")
            nodeName = "talkerYellow";
        if(this.topicName == "/beacon/violet/presence")
            nodeName = "talkerViolet";

        return GraphName.of("beacons_proximity/" + nodeName);
    }


    public void onStart(ConnectedNode connectedNode) {
        final Publisher<std_msgs.String> publisher = connectedNode.newPublisher(this.topicName, "std_msgs/String");
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            private int sequenceNumber;

            protected void setup() {
                this.sequenceNumber = 0;
            }

            protected void loop() throws InterruptedException {
                std_msgs.String str = publisher.newMessage();
                if(presence){
                    str.setData("Presence in: " + topicName + this.sequenceNumber);
                    Log.d("Debug.. ", "Published presence message in: " + topicName + this.sequenceNumber);
                    publisher.publish(str);
                }
                ++this.sequenceNumber;
                Thread.sleep(1000L);
            }
        });
    }
}
