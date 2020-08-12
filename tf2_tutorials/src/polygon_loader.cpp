#include <pluginlib/class_loader.h>
#include <pluginlib_tutorials_/polygon_base.h>

int main(int argc, char** argv)
{
  pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader("pluginlib_tutorials_", "polygon_base::RegularPolygon");
  try
  {
    boost::shared_ptr<polygon_base::RegularPolygon> triangle = poly_loader.createInstance("polygon_plugins::Triangle");
    triangle->initialize(5);
    boost::shared_ptr<polygon_base::RegularPolygon> rectangle = poly_loader.createInstance("polygon_plugins::Square");
    rectangle->initialize(10.0);

    ROS_INFO("Triangle area: %4f", triangle->area());
    ROS_INFO("Rectangle area: %4f", rectangle->area());
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load. Error: %s", ex.what());
  }
  
  return 0;
}
