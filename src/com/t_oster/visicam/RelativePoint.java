package com.t_oster.visicam;

import java.awt.geom.Point2D;
import java.awt.Point;

/**
 *
 * @author Thomas Oster <thomas.oster@rwth-aachen.de>
 */
public class RelativePoint 
{
  double x = 0;
  double y = 0;

  public RelativePoint(double x, double y)
  {
    this.x = check(x);
    this.y = check(y);
  }
  
  public RelativePoint(Point subject, int width, int height)
  {
    this.x = check(subject.x / (double) width);
    this.y = check(subject.y / (double) height);
  }
  
  public double getX() {
    return x;
  }

  public void setX(double x) {
    this.x = check(x);
  }

  public double getY() {
    return y;
  }

  public void setY(double y) {
    this.y = check(y);
  }

  public Point toAbsolutePoint(int width, int height)
  {
    return new Point((int) (x*width), (int) (y*height));
  }

  public Point2D.Double toPoint2D()
  {
    return new Point2D.Double(this.x, this.y);
  }

  protected final double check(double input)
  {
    if (input < 0 || input > 1)
    {
      throw new IllegalArgumentException("Only values between 0 and 1 are allowed");
    }
    return input;
  }
  
  @Override
  public String toString()
  {
    return "RelativePoint x="+(100*x)+"% y="+(100*y)+"%";
  }
}
