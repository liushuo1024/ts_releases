
#include "vehicle_kinematics/vehicle_kinematics_node.h"

using namespace std;

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "vehicle_kinematics_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    std::string vehicle_type;
    ros::param::get("~vehicle_type", vehicle_type);
    std::cout << "vehicle_type: " << vehicle_type << std::endl;
    if(vehicle_type == "single_servo")
        SingleServo ss(nh, nh_priv);
    else if(vehicle_type == "omv")
        Omv omv(nh, nh_priv);
    else if(vehicle_type == "dual")
        Dual dual(nh, nh_priv);
    return 0;
}
