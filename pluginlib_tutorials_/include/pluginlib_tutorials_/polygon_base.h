#ifndef PLUGINLIB_TUTORIALS_POLYGON_BASE_H
#define PLUGINLIB_TUTORIALS_POLYGON_BASE_H

namespace polygon_base
{
  class RegularPolygon
  {
    public:
      // a virtual function is initialized to
      // 0 so that a derived class can overide
      // the implementation of this class
      virtual void initialize(double side_length) = 0;
      virtual double area() = 0;
      virtual ~RegularPolygon() {}
    protected:
      RegularPolygon() {}
  };
};

#endif
