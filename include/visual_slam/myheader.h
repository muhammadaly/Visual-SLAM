#ifndef MYHEADER_H
#define MYHEADER_H

namespace visual_slam {

class myHeader {
  public:
    myHeader();///<Allows same usage as ros/pcl header
    myHeader(pcl::PCLHeader h); ///<Allow implicit conversion from pcl header
    myHeader(std_msgs::Header rh);///<Allow implicit conversion from ros header
    myHeader(uint32_t aseq, ros::Time astamp, std::string aframe_id); ///<convenience

    std_msgs::Header toRosHeader();
    void fromRosHeader(std_msgs::Header rh);

    operator pcl::PCLHeader(); ///<Allow implicit conversion to pcl header
    operator std_msgs::Header(); ///<Allow implicit conversion to ros header

    uint32_t seq;///<Allows direct access as for pcl/ros header
    ros::Time stamp; ///<Allows direct access as for pcl/ros header
    std::string frame_id;///<Allows direct access as for pcl/ros header
};

}
#endif // MYHEADER_H
