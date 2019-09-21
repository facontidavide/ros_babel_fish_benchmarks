//
// Created by Stefan Fabian on 18.09.19.
//

#include <benchmark/benchmark.h>

#include <ros_babel_fish/babel_fish.h>
#include <ros_type_introspection/ros_introspection.hpp>
#include <ros_type_introspection/utils/shape_shifter.hpp>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>

#define ENABLE_RTI 1

void fillMessage( geometry_msgs::Pose& )
{
  // static size, doesn't matter
}

void fillMessage( sensor_msgs::JointState& joint )
{
  joint.header.frame_id = "base";
  joint.name.push_back("Joint1");
  joint.name.push_back("Joint2");
  joint.name.push_back("Joint3");
  joint.velocity.resize(3,0);
  joint.effort.resize(3,0);
  joint.position.push_back(0);
  joint.position.push_back(1);
  joint.position.push_back(2);
}

void fillMessage( sensor_msgs::PointCloud2 &pointcloud )
{
  for ( size_t i = 0; i < 100000; ++i )
  {
    pointcloud.data.emplace_back( 3 * i );
    pointcloud.data.emplace_back( 3 * i + 1 );
    pointcloud.data.emplace_back( 3 * i + 2 );
  }

  pointcloud.fields.emplace_back();
  pointcloud.fields[0].name = "Test";
}

void fillMessage( sensor_msgs::Image &image )
{
  for ( size_t i = 0; i < 1920 * 1080; ++i )
  {
    image.data.push_back( i % 255 );
  }
  image.width = 1920;
  image.height = 1080;
  image.encoding = "rgb8";
}

namespace mt = ros::message_traits;

#if ENABLE_RTI

template <typename MsgType>
void RTI_ParseMessageDefinition( benchmark::State &state )
{
  const std::string &datatype = mt::DataType<MsgType>::value();
  const std::string &definition = mt::Definition<MsgType>::value();
  const std::string &md5 = mt::MD5Sum<MsgType>::value();

  for ( auto _ : state )
  {
    state.PauseTiming();
    RosIntrospection::Parser parser;
    state.ResumeTiming();
    parser.registerMessageDefinition( datatype, RosIntrospection::ROSType( datatype ), definition );
  }
}

static void RTI_ParseMessageDefinitionPose( benchmark::State &state )
{
  RTI_ParseMessageDefinition<geometry_msgs::Pose>(state);
}

BENCHMARK( RTI_ParseMessageDefinitionPose )->Unit( benchmark::kMicrosecond );
#endif

template <typename MsgType>
void RBF_ParseMessageDefinition( benchmark::State &state )
{
  const std::string &datatype = mt::DataType<MsgType>::value();
  const std::string &definition = mt::Definition<MsgType>::value();
  const std::string &md5 = mt::MD5Sum<MsgType>::value();
  ros_babel_fish::BabelFishMessage msg;
  msg.morph( md5, datatype, definition );

  for ( auto _ : state )
  {
    state.PauseTiming();
    ros_babel_fish::BabelFish fish;
    state.ResumeTiming();
    fish.descriptionProvider()->getMessageDescription( msg );
  }
}

static void RBF_ParseMessageDefinitionPose(benchmark::State &state)
{
  RBF_ParseMessageDefinition<geometry_msgs::Pose>(state);
}

BENCHMARK(RBF_ParseMessageDefinitionPose)->Unit(benchmark::kMicrosecond);

#if ENABLE_RTI

template <typename MsgType>
void RTI_ParseMessage( benchmark::State &state )
{
  const std::string &datatype = mt::DataType<MsgType>::value();
  const std::string &definition = mt::Definition<MsgType>::value();
  const std::string &md5 = mt::MD5Sum<MsgType>::value();
  RosIntrospection::Parser parser;
  parser.registerMessageDefinition( datatype, RosIntrospection::ROSType( datatype ), definition );
  RosIntrospection::FlatMessage flat_container;
  RosIntrospection::RenamedValues renamed_value;

  MsgType msg;
  fillMessage(msg);

  ros::SerializedMessage serialized_msg = ros::serialization::serializeMessage( msg );
  RosIntrospection::ShapeShifter shape_shifter;
  shape_shifter.morph( md5, datatype, definition );
  ros::serialization::deserializeMessage( serialized_msg, shape_shifter );

  for ( auto _ : state )
  {
    auto buffer = absl::Span<uint8_t>( (uint8_t*)shape_shifter.raw_data(), shape_shifter.size() );
    parser.deserializeIntoFlatContainer( datatype, buffer, &flat_container, 10 );
    parser.applyNameTransform( datatype, flat_container, &renamed_value );
  }
}

static void RTI_ParseMessagePose( benchmark::State &state )
{
  RTI_ParseMessage<geometry_msgs::Pose>(state);
}

BENCHMARK( RTI_ParseMessagePose );
#endif

template <typename MsgType>
static void RBF_ParseMessage( benchmark::State &state )
{
  const std::string &datatype = mt::DataType<MsgType>::value();
  const std::string &definition = mt::Definition<MsgType>::value();
  const std::string &md5 = mt::MD5Sum<MsgType>::value();
  ros_babel_fish::BabelFish fish;

  MsgType msg;
  fillMessage(msg);

  ros::SerializedMessage serialized_msg = ros::serialization::serializeMessage( msg );
  ros_babel_fish::BabelFishMessage bf_msg;
  bf_msg.morph( md5, datatype, definition );
  fish.descriptionProvider()->getMessageDescription( bf_msg );
  ros::serialization::deserializeMessage( serialized_msg, bf_msg );

  for ( auto _ : state )
  {
    ros_babel_fish::Message::Ptr translated = fish.translateMessage( bf_msg );
  }
}

static void RBF_ParseMessagePose( benchmark::State &state )
{
  RBF_ParseMessage<geometry_msgs::Pose>(state);
}

BENCHMARK( RBF_ParseMessagePose );

#if ENABLE_RTI

static void RTI_ParseMessagePointcloud( benchmark::State &state )
{
  RTI_ParseMessage<sensor_msgs::PointCloud2>(state);
}

BENCHMARK( RTI_ParseMessagePointcloud );
#endif

static void RBF_ParseMessagePointcloud( benchmark::State &state )
{
  RBF_ParseMessage<sensor_msgs::PointCloud2>(state);
}

BENCHMARK( RBF_ParseMessagePointcloud );

#if ENABLE_RTI

static void RTI_ParseMessageFullHDImage( benchmark::State &state )
{
  RTI_ParseMessage<sensor_msgs::Image>(state);
}

BENCHMARK( RTI_ParseMessageFullHDImage );
#endif

static void RBF_ParseMessageFullHDImage( benchmark::State &state )
{
  RBF_ParseMessage<sensor_msgs::Image>(state);
}

BENCHMARK( RBF_ParseMessageFullHDImage );

#if ENABLE_RTI

static void RTI_ParseMessageJointState( benchmark::State &state )
{
  RTI_ParseMessage<sensor_msgs::JointState>(state);
}

BENCHMARK( RTI_ParseMessageJointState );
#endif

static void RBF_ParseMessageJointState( benchmark::State &state )
{
  RBF_ParseMessage<sensor_msgs::JointState>(state);
}

BENCHMARK( RBF_ParseMessageJointState );

BENCHMARK_MAIN();
