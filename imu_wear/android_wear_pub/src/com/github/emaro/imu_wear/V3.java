package com.github.emaro.imu_wear;

import org.ros.internal.message.RawMessage;

import geometry_msgs.Vector3;

public class V3 implements Vector3 {

  private double vv[] = new double[3];

  public V3() {
  }

  @Override
  public double getX() {
    return vv[0];
  }

  @Override
  public void setX(double v) {
    vv[0] = v;
  }

  @Override
  public double getY() {
    return vv[1];
  }

  @Override
  public void setY(double v) {
    vv[1] = v;
  }

  @Override
  public double getZ() {
    return vv[2];
  }

  @Override
  public void setZ(double v) {
    vv[2] = v;
  }

  @Override
  public RawMessage toRawMessage() {
    return null;
  }
}
