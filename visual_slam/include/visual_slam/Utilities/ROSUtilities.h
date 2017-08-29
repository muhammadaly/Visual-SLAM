#ifndef ROSUTILITIES_H
#define ROSUTILITIES_H

namespace visual_slam {

class ROSUtilities
{
public:
  static ROSUtilities *instance()
  {
    if (!s_instance)
      s_instance = new ROSUtilities;
    return s_instance;
  }
bool asyncFrameDrop(ros::Time depth, ros::Time rgb);

private:
  static ROSUtilities * s_instance;
  ROSUtilities();
};

}


#endif // ROSUTILITIES_H
