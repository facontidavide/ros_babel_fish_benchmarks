//
// Created by Stefan Fabian on 18.09.19.
//

#include <benchmark/benchmark.h>

#include <ros_babel_fish/babel_fish.h>
#include <ros_type_introspection/ros_introspection.hpp>
#include <topic_tools/shape_shifter.h>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>


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

static void RTI_ParseMessageDefinitionPose( benchmark::State &state )
{
  using MsgType = geometry_msgs::Pose;
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

BENCHMARK( RTI_ParseMessageDefinitionPose )->Unit( benchmark::kMicrosecond );
#endif

static void RBF_ParseMessageDefinitionPose( benchmark::State &state )
{
  using MsgType = geometry_msgs::Pose;
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

BENCHMARK( RBF_ParseMessageDefinitionPose )->Unit( benchmark::kMicrosecond );

#if ENABLE_RTI

static void RTI_ParseMessagePose( benchmark::State &state )
{
  using MsgType = geometry_msgs::Pose;
  const std::string &datatype = mt::DataType<MsgType>::value();
  const std::string &definition = mt::Definition<MsgType>::value();
  const std::string &md5 = mt::MD5Sum<MsgType>::value();
  RosIntrospection::Parser parser;
  parser.registerMessageDefinition( datatype, RosIntrospection::ROSType( datatype ), definition );
  RosIntrospection::FlatMessage flat_container;
  RosIntrospection::RenamedValues renamed_value;
  std::vector<uint8_t> buffer;

  geometry_msgs::Pose msg;
  ros::SerializedMessage serialized_msg = ros::serialization::serializeMessage( msg );
  topic_tools::ShapeShifter shape_shifter;
  shape_shifter.morph( md5, datatype, definition, "0" );
  ros::serialization::deserializeMessage( serialized_msg, shape_shifter );

  for ( auto _ : state )
  {
    buffer.resize( shape_shifter.size());
    ros::serialization::OStream stream( buffer.data(), buffer.size());
    shape_shifter.write( stream );

    parser.deserializeIntoFlatContainer( datatype, absl::Span<uint8_t>( buffer ), &flat_container, 10 );
    parser.applyNameTransform( datatype, flat_container, &renamed_value );
  }
}

BENCHMARK( RTI_ParseMessagePose );
#endif

static void RBF_ParseMessagePose( benchmark::State &state )
{
  using MsgType = geometry_msgs::Pose;
  const std::string &datatype = mt::DataType<MsgType>::value();
  const std::string &definition = mt::Definition<MsgType>::value();
  const std::string &md5 = mt::MD5Sum<MsgType>::value();
  ros_babel_fish::BabelFish fish;

  geometry_msgs::Pose msg;
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

BENCHMARK( RBF_ParseMessagePose );

#if ENABLE_RTI

static void RTI_ParseMessagePointcloud( benchmark::State &state )
{
  using MsgType = sensor_msgs::PointCloud2;
  const std::string &datatype = mt::DataType<MsgType>::value();
  const std::string &definition = mt::Definition<MsgType>::value();
  const std::string &md5 = mt::MD5Sum<MsgType>::value();
  RosIntrospection::Parser parser;
  parser.registerMessageDefinition( datatype, RosIntrospection::ROSType( datatype ), definition );
  RosIntrospection::FlatMessage flat_container;
  RosIntrospection::RenamedValues renamed_value;
  std::vector<uint8_t> buffer;

  MsgType msg;
  fillMessage( msg );
  ros::SerializedMessage serialized_msg = ros::serialization::serializeMessage( msg );
  topic_tools::ShapeShifter shape_shifter;
  shape_shifter.morph( md5, datatype, definition, "0" );
  ros::serialization::deserializeMessage( serialized_msg, shape_shifter );

  for ( auto _ : state )
  {
    buffer.resize( shape_shifter.size());
    ros::serialization::OStream stream( buffer.data(), buffer.size());
    shape_shifter.write( stream );

    parser.deserializeIntoFlatContainer( datatype, absl::Span<uint8_t>( buffer ), &flat_container, 10 );
    parser.applyNameTransform( datatype, flat_container, &renamed_value );
  }
}

BENCHMARK( RTI_ParseMessagePointcloud );
#endif

static void RBF_ParseMessagePointcloud( benchmark::State &state )
{
  using MsgType = sensor_msgs::PointCloud2;
  const std::string &datatype = mt::DataType<MsgType>::value();
  const std::string &definition = mt::Definition<MsgType>::value();
  const std::string &md5 = mt::MD5Sum<MsgType>::value();
  ros_babel_fish::BabelFish fish;

  MsgType msg;
  fillMessage( msg );
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

BENCHMARK( RBF_ParseMessagePointcloud );

#if ENABLE_RTI

static void RTI_ParseMessageFullHDImage( benchmark::State &state )
{
  using MsgType = sensor_msgs::Image;
  const std::string &datatype = mt::DataType<MsgType>::value();
  const std::string &definition = mt::Definition<MsgType>::value();
  const std::string &md5 = mt::MD5Sum<MsgType>::value();
  RosIntrospection::Parser parser;
  parser.registerMessageDefinition( datatype, RosIntrospection::ROSType( datatype ), definition );
  RosIntrospection::FlatMessage flat_container;
  RosIntrospection::RenamedValues renamed_value;
  std::vector<uint8_t> buffer;

  MsgType msg;
  fillMessage( msg );
  ros::SerializedMessage serialized_msg = ros::serialization::serializeMessage( msg );
  topic_tools::ShapeShifter shape_shifter;
  shape_shifter.morph( md5, datatype, definition, "0" );
  ros::serialization::deserializeMessage( serialized_msg, shape_shifter );

  for ( auto _ : state )
  {
    buffer.resize( shape_shifter.size());
    ros::serialization::OStream stream( buffer.data(), buffer.size());
    shape_shifter.write( stream );

    parser.deserializeIntoFlatContainer( datatype, absl::Span<uint8_t>( buffer ), &flat_container, 10 );
    parser.applyNameTransform( datatype, flat_container, &renamed_value );
  }
}

BENCHMARK( RTI_ParseMessageFullHDImage );
#endif

static void RBF_ParseMessageFullHDImage( benchmark::State &state )
{
  using MsgType = sensor_msgs::Image;
  const std::string &datatype = mt::DataType<MsgType>::value();
  const std::string &definition = mt::Definition<MsgType>::value();
  const std::string &md5 = mt::MD5Sum<MsgType>::value();
  ros_babel_fish::BabelFish fish;

  MsgType msg;
  fillMessage( msg );
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

BENCHMARK( RBF_ParseMessageFullHDImage );


BENCHMARK_MAIN();
