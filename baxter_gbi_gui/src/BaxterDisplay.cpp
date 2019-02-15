#include "BaxterDisplay.h"

BaxterDisplay::BaxterDisplay(std::string topic)
: topic(topic)
{
    publisher = n.advertise<sensor_msgs::Image>(topic, 1);
}

BaxterDisplay::BaxterDisplay(QString topic)
: BaxterDisplay(topic.toStdString())
{}

void BaxterDisplay::operator()(QPixmap &pixmap){
	auto img_rgb = pixmap.toImage().convertToFormat(QImage::Format_RGB888);
	auto img_bgr = img_rgb.rgbSwapped();
	//img_rgb.save("/home/lucrezia/file_rgb.png", "PNG", 100);
	//img_bgr.save("/home/lucrezia/file_bgr.png", "PNG", 100);
	
	message.height = 600;
	message.width = 1024;
	message.encoding = "bgr8";
	message.is_bigendian = 0;
	message.step = img_bgr.bytesPerLine();
	message.data.assign(img_bgr.constBits(), img_bgr.constBits() + (1024*600*3));
	publisher.publish(message);
	ros::spinOnce();
}
