//
// Created by Stefan Fabian on 10.09.19.
//
#include <ros_babel_fish/babel_fish.h>
#include <ros/ros.h>

#include <chrono>

long long sum = 0;
ros_babel_fish::BabelFish fish;
std::chrono::time_point<std::chrono::high_resolution_clock> last_message;

void callback( const ros_babel_fish::BabelFishMessage::ConstPtr &msg )
{
  std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
  start = std::chrono::high_resolution_clock::now();
  ros_babel_fish::TranslatedMessage::Ptr message = fish.translateMessage( msg );
  sum += (*message->translated_message)["header"]["seq"].as<ros_babel_fish::ValueMessage<uint32_t >>().getValue();
  end = std::chrono::high_resolution_clock::now();
  std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>( end - start ).count() << std::endl;

  last_message = end;
}

int main( int argc, char **argv )
{
  ros::init( argc, argv, "ros_babel_fish_benchmark_image_rbf" );
  ros::NodeHandle nh;

  ros::Subscriber subscriber = nh.subscribe<ros_babel_fish::BabelFishMessage>( "/camera360/equirectangular_low_fov", 250,
                                                                               &callback );
  last_message = std::chrono::high_resolution_clock::now();
  std::chrono::time_point<std::chrono::high_resolution_clock> now = last_message;
  while ( sum == 0 || std::chrono::duration_cast<std::chrono::milliseconds>( now - last_message ).count() < 10000 )
  {
    ros::spinOnce();
    now = std::chrono::high_resolution_clock::now();
  }
  std::cout << sum << std::endl;
  return 0;
}
