#include <gtest/gtest.h>
#include <ros/ros.h>

#include "track_path_client.h"
std::shared_ptr<decision_maker::TrackPathClient> track_path_ptr_;
void SendTrackPathClientGoal(const amr_msgs::track_pathGoal goal) {
  track_path_ptr_->SendTrackPathGoal(goal);
}
void StartPoint(amr_msgs::track_pathGoal &t_goal){
  t_goal.navi_type =1;
  t_goal.motion_type = 1; // 1:start_point, 2:直线, 3:b样条曲线, 4:圆弧
  t_goal.next_path_type = 0; // 目前不需要
  t_goal.max_speed = 1.0; // 不需要
  t_goal.target_speed = 1.0;
  t_goal.start_pose_x = 0.0;
  t_goal.start_pose_y = 0.0;
  t_goal.start_pose_yaw = 0.0;
  t_goal.end_x = 0;
  t_goal.end_y = 0;
  t_goal.end_yaw = 0;

  std::cout << " navi_type: " <<  t_goal.navi_type
            << ", motion_type: " <<  t_goal.motion_type
            << ", next_path_type: " <<  t_goal.next_path_type
            << ", max_speed: " <<  t_goal.max_speed
            << ", target_speed: " <<  t_goal.target_speed
            << ", adjust_speed_pose("
            <<  t_goal.adjust_speed_pose_x << ", "
            <<  t_goal.adjust_speed_pose_y << ", "
            <<  t_goal.adjust_speed_pose_yaw << "), start_pose("
            <<  t_goal.start_pose_x << ", "
            <<  t_goal.start_pose_y << ", "
            <<  t_goal.start_pose_yaw << "), high_precision_pose("
            <<  t_goal.high_precision_pose_x << ", "
            << "), c2 point: (" <<  t_goal.c2_x << ", "
            <<  t_goal.c2_y << ")"
            << ", next_end_id: " <<  t_goal.next_end_id
            << ", next end point(" <<  t_goal.next_end_x << ", "
            <<  t_goal.next_end_y << ", "
            <<  t_goal.next_end_yaw << "), next c1 point: ("
            <<  t_goal.next_c1_x << ", "
            <<  t_goal.next_c1_y << "), next c2 point: ("
            <<  t_goal.next_c2_x << ", "
            <<  t_goal.next_c2_y << ")"          <<  t_goal.high_precision_pose_y << ", "
            <<  t_goal.high_precision_pose_yaw
            << "), end_id: " <<  t_goal.end_id << ", end point("
            <<  t_goal.end_x << ", " <<  t_goal.end_y
            << ", " <<  t_goal.end_yaw << "), c1 point: ("
            <<  t_goal.c1_x << ", " <<  t_goal.c1_y << std::endl;
}
void StraightLine(amr_msgs::track_pathGoal &t_goal){
  t_goal.navi_type =1;
  t_goal.motion_type = 2; // 1:start_point, 2:直线, 3:b样条曲线, 4:圆弧
  t_goal.next_path_type = 0; // 目前不需要
  t_goal.max_speed = 1.0; // 不需要
  t_goal.target_speed = 1.0;
  t_goal.start_pose_x = 0.0;
  t_goal.start_pose_y = 0.0;
  t_goal.start_pose_yaw = 0.0;
  t_goal.end_x = 5.0;
  t_goal.end_y = 0;
  t_goal.end_yaw = 0;

  std::cout << " navi_type: " <<  t_goal.navi_type
            << ", motion_type: " <<  t_goal.motion_type
            << ", next_path_type: " <<  t_goal.next_path_type
            << ", max_speed: " <<  t_goal.max_speed
            << ", target_speed: " <<  t_goal.target_speed
            << ", adjust_speed_pose("
            <<  t_goal.adjust_speed_pose_x << ", "
            <<  t_goal.adjust_speed_pose_y << ", "
            <<  t_goal.adjust_speed_pose_yaw << "), start_pose("
            <<  t_goal.start_pose_x << ", "
            <<  t_goal.start_pose_y << ", "
            <<  t_goal.start_pose_yaw << "), high_precision_pose("
            <<  t_goal.high_precision_pose_x << ", "
            << "), c2 point: (" <<  t_goal.c2_x << ", "
            <<  t_goal.c2_y << ")"
            << ", next_end_id: " <<  t_goal.next_end_id
            << ", next end point(" <<  t_goal.next_end_x << ", "
            <<  t_goal.next_end_y << ", "
            <<  t_goal.next_end_yaw << "), next c1 point: ("
            <<  t_goal.next_c1_x << ", "
            <<  t_goal.next_c1_y << "), next c2 point: ("
            <<  t_goal.next_c2_x << ", "
            <<  t_goal.next_c2_y << ")"          <<  t_goal.high_precision_pose_y << ", "
            <<  t_goal.high_precision_pose_yaw
            << "), end_id: " <<  t_goal.end_id << ", end point("
            <<  t_goal.end_x << ", " <<  t_goal.end_y
            << ", " <<  t_goal.end_yaw << "), c1 point: ("
            <<  t_goal.c1_x << ", " <<  t_goal.c1_y << std::endl;

}

void BSplineCurve(amr_msgs::track_pathGoal &t_goal){
  t_goal.navi_type = 1;
  t_goal.motion_type = 3; // 3:b样条曲线
  t_goal.next_path_type = 0; // 目前不需要
  t_goal.max_speed = 1.0; // 不需要
  t_goal.target_speed = 0.5;
  t_goal.start_pose_x = 5.0;
  t_goal.start_pose_y = 0.0;
  t_goal.start_pose_yaw = 0.0;
  t_goal.end_x = 7;
  t_goal.end_y = 2;
  t_goal.end_yaw = 90;
  
  // B样条曲线控制点
  t_goal.c1_x = 6.0;  // 第一个控制点
  t_goal.c1_y = 0.0;
  t_goal.c2_x = 7.0;  // 第二个控制点
  t_goal.c2_y = 1.0;

  std::cout << " navi_type: " <<  t_goal.navi_type
            << ", motion_type: " <<  t_goal.motion_type
            << ", next_path_type: " <<  t_goal.next_path_type
            << ", max_speed: " <<  t_goal.max_speed
            << ", target_speed: " <<  t_goal.target_speed
            << ", adjust_speed_pose("
            <<  t_goal.adjust_speed_pose_x << ", "
            <<  t_goal.adjust_speed_pose_y << ", "
            <<  t_goal.adjust_speed_pose_yaw << "), start_pose("
            <<  t_goal.start_pose_x << ", "
            <<  t_goal.start_pose_y << ", "
            <<  t_goal.start_pose_yaw << "), high_precision_pose("
            <<  t_goal.high_precision_pose_x << ", "
            <<  t_goal.high_precision_pose_y << ", "
            <<  t_goal.high_precision_pose_yaw << "), end_id: " <<  t_goal.end_id 
            << ", end point(" <<  t_goal.end_x << ", " <<  t_goal.end_y
            << ", " <<  t_goal.end_yaw << "), c1 point: ("
            <<  t_goal.c1_x << ", " <<  t_goal.c1_y << "), c2 point: ("
            <<  t_goal.c2_x << ", " <<  t_goal.c2_y << ")" << std::endl;

}
void LatStraightLine(amr_msgs::track_pathGoal &t_goal){
  t_goal.navi_type =1;
  t_goal.motion_type = 32; // 1:start_point, 2:直线, 3:b样条曲线, 4:圆弧
  t_goal.next_path_type = 0; // 目前不需要
  t_goal.max_speed = 1.0; // 不需要
  t_goal.target_speed = 1.0;
  t_goal.start_pose_x = 0.0;
  t_goal.start_pose_y = 0.0;
  t_goal.start_pose_yaw = 0.0;
  t_goal.end_x = 0.0;
  t_goal.end_y = 2;
  t_goal.end_yaw = 0.0;

  std::cout << " navi_type: " <<  t_goal.navi_type
            << ", motion_type: " <<  t_goal.motion_type
            << ", next_path_type: " <<  t_goal.next_path_type
            << ", max_speed: " <<  t_goal.max_speed
            << ", target_speed: " <<  t_goal.target_speed
            << ", adjust_speed_pose("
            <<  t_goal.adjust_speed_pose_x << ", "
            <<  t_goal.adjust_speed_pose_y << ", "
            <<  t_goal.adjust_speed_pose_yaw << "), start_pose("
            <<  t_goal.start_pose_x << ", "
            <<  t_goal.start_pose_y << ", "
            <<  t_goal.start_pose_yaw << "), high_precision_pose("
            <<  t_goal.high_precision_pose_x << ", "
            << "), c2 point: (" <<  t_goal.c2_x << ", "
            <<  t_goal.c2_y << ")"
            << ", next_end_id: " <<  t_goal.next_end_id
            << ", next end point(" <<  t_goal.next_end_x << ", "
            <<  t_goal.next_end_y << ", "
            <<  t_goal.next_end_yaw << "), next c1 point: ("
            <<  t_goal.next_c1_x << ", "
            <<  t_goal.next_c1_y << "), next c2 point: ("
            <<  t_goal.next_c2_x << ", "
            <<  t_goal.next_c2_y << ")"          <<  t_goal.high_precision_pose_y << ", "
            <<  t_goal.high_precision_pose_yaw
            << "), end_id: " <<  t_goal.end_id << ", end point("
            <<  t_goal.end_x << ", " <<  t_goal.end_y
            << ", " <<  t_goal.end_yaw << "), c1 point: ("
            <<  t_goal.c1_x << ", " <<  t_goal.c1_y << std::endl;
}






void LatStraightLine2(amr_msgs::track_pathGoal &t_goal){
  t_goal.navi_type =1;
  t_goal.motion_type = 32; // 1:start_point, 2:直线, 3:b样条曲线, 4:圆弧
  t_goal.next_path_type = 0; // 目前不需要
  t_goal.max_speed = 1.0; // 不需要
  t_goal.target_speed = 1.0;
  t_goal.start_pose_x = 0.0;
  t_goal.start_pose_y = 0.0;
  t_goal.start_pose_yaw = 0.0;
  t_goal.end_x = 0.0;
  t_goal.end_y = -2.0;
  t_goal.end_yaw = 0.0;

  std::cout << " navi_type: " <<  t_goal.navi_type
            << ", motion_type: " <<  t_goal.motion_type
            << ", next_path_type: " <<  t_goal.next_path_type
            << ", max_speed: " <<  t_goal.max_speed
            << ", target_speed: " <<  t_goal.target_speed
            << ", adjust_speed_pose("
            <<  t_goal.adjust_speed_pose_x << ", "
            <<  t_goal.adjust_speed_pose_y << ", "
            <<  t_goal.adjust_speed_pose_yaw << "), start_pose("
            <<  t_goal.start_pose_x << ", "
            <<  t_goal.start_pose_y << ", "
            <<  t_goal.start_pose_yaw << "), high_precision_pose("
            <<  t_goal.high_precision_pose_x << ", "
            << "), c2 point: (" <<  t_goal.c2_x << ", "
            <<  t_goal.c2_y << ")"
            << ", next_end_id: " <<  t_goal.next_end_id
            << ", next end point(" <<  t_goal.next_end_x << ", "
            <<  t_goal.next_end_y << ", "
            <<  t_goal.next_end_yaw << "), next c1 point: ("
            <<  t_goal.next_c1_x << ", "
            <<  t_goal.next_c1_y << "), next c2 point: ("
            <<  t_goal.next_c2_x << ", "
            <<  t_goal.next_c2_y << ")"          <<  t_goal.high_precision_pose_y << ", "
            <<  t_goal.high_precision_pose_yaw
            << "), end_id: " <<  t_goal.end_id << ", end point("
            <<  t_goal.end_x << ", " <<  t_goal.end_y
            << ", " <<  t_goal.end_yaw << "), c1 point: ("
            <<  t_goal.c1_x << ", " <<  t_goal.c1_y << std::endl;

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "action_cilent_test");

  // Set ros log level:
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Info)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::NodeHandle nh;


  std::string trackPathName = "/track_path";
  track_path_ptr_ = std::make_shared<decision_maker::TrackPathClient>(trackPathName);

  amr_msgs::track_pathGoal t_goal;
  StartPoint(t_goal);
  SendTrackPathClientGoal(t_goal);
  while(ros::ok()){
    if(track_path_ptr_->FinishState() == decision_maker::GoalFinishState::SUCCEEDED){
      std::cout << "FinishState: SUCCEEDED" << std::endl;
      break;
    }
    ros::Duration(0.1).sleep();
  }

  // StraightLine(t_goal);
  // SendTrackPathClientGoal(t_goal);
  // while(ros::ok()){
  //   if(track_path_ptr_->FinishState() == decision_maker::GoalFinishState::SUCCEEDED){
  //     std::cout << "FinishState: SUCCEEDED" << std::endl;
  //     break;
  //   }
  //   ros::Duration(0.1).sleep();
  // }

  // // 测试B样条曲线
  // std::cout << "\n=== Testing BSpline Curve ===" << std::endl;
  // BSplineCurve(t_goal);
  // SendTrackPathClientGoal(t_goal);
  // while(ros::ok()){
  //   if(track_path_ptr_->FinishState() == decision_maker::GoalFinishState::SUCCEEDED){
  //     std::cout << "FinishState: SUCCEEDED" << std::endl;
  //     break;
  //   }
  //   ros::Duration(0.1).sleep();
  // }

  // 测试横移样条曲线
  // std::cout << "\n=== Testing Lat Line 1===" << std::endl;
  // LatStraightLine(t_goal);
  // SendTrackPathClientGoal(t_goal);
  // while(ros::ok()){
  //   if(track_path_ptr_->FinishState() == decision_maker::GoalFinishState::SUCCEEDED){
  //     std::cout << "FinishState: SUCCEEDED" << std::endl;
  //     break;
  //   }
  //   ros::Duration(0.1).sleep();
  // }

  std::cout << "\n=== Testing Lat Line 2===" << std::endl;
  LatStraightLine2(t_goal);
  SendTrackPathClientGoal(t_goal);
  while(ros::ok()){
    if(track_path_ptr_->FinishState() == decision_maker::GoalFinishState::SUCCEEDED){
      std::cout << "FinishState: SUCCEEDED" << std::endl;
      break;
    }
    ros::Duration(0.1).sleep();
  }
  ros::spin();
  return 0;
}