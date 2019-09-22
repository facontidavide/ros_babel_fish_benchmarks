//
// Created by Stefan Fabian on 18.09.19.
//

#include <benchmark/benchmark.h>

#include <ros_babel_fish/babel_fish.h>
#include <ros_type_introspection/ros_introspection.hpp>
#include <ros_type_introspection/utils/shape_shifter.hpp>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tfMessage.h>

inline void fillMessage( geometry_msgs::Pose& )
{
  // static size, doesn't matter
}

inline void fillMessage( sensor_msgs::JointState& joint )
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

inline void fillMessage( sensor_msgs::PointCloud2 &pointcloud )
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

inline void fillMessage( sensor_msgs::Image &image )
{
  for ( size_t i = 0; i < 1920 * 1080; ++i )
  {
    image.data.push_back( i % 255 );
  }
  image.width = 1920;
  image.height = 1080;
  image.encoding = "rgb8";
}

inline void fillMessage( nav_msgs::Odometry& odom )
{
  odom.header.frame_id ="odom";
  // I don't bother to fill the rest, because other fields have static size
}

inline void fillMessage( tf::tfMessage& msg )
{
  msg.transforms.resize(3);

  msg.transforms[0].header.frame_id = "this_one";
  msg.transforms[1].header.frame_id = "another_one";
  msg.transforms[2].header.frame_id = "last_one";
  // I don't bother to fill the rest, because other fields have static size
}



namespace mt = ros::message_traits;

template <typename MsgType> inline
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

template <typename MsgType> inline
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


template <typename MsgType> inline
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

  parser.setBlobPolicy( RosIntrospection::Parser::STORE_BLOB_AS_REFERENCE );

  for ( auto _ : state )
  {
    auto buffer = absl::Span<uint8_t>( (uint8_t*)shape_shifter.raw_data(), shape_shifter.size() );
    parser.deserializeIntoFlatContainer( datatype, buffer, &flat_container, 10 );
    parser.applyNameTransform( datatype, flat_container, &renamed_value );
  }
}

template <typename MsgType> inline
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

//------------------------------------------

static void RTI_ParseMessageDefinitionPose( benchmark::State &state )
{
  RTI_ParseMessageDefinition<geometry_msgs::Pose>(state);
}
BENCHMARK( RTI_ParseMessageDefinitionPose )->Unit( benchmark::kMicrosecond );

static void RBF_ParseMessageDefinitionPose(benchmark::State &state)
{
  RBF_ParseMessageDefinition<geometry_msgs::Pose>(state);
}
BENCHMARK(RBF_ParseMessageDefinitionPose)->Unit(benchmark::kMicrosecond);

//------------------------------------------

static void RTI_ParseMessagePose( benchmark::State &state )
{
  RTI_ParseMessage<geometry_msgs::Pose>(state);
}
BENCHMARK( RTI_ParseMessagePose );

static void RBF_ParseMessagePose( benchmark::State &state )
{
  RBF_ParseMessage<geometry_msgs::Pose>(state);
}
BENCHMARK( RBF_ParseMessagePose );

//------------------------------------------

static void RTI_ParseMessageJointState( benchmark::State &state )
{
  RTI_ParseMessage<sensor_msgs::JointState>(state);
}
BENCHMARK( RTI_ParseMessageJointState );


static void RBF_ParseMessageJointState( benchmark::State &state )
{
  RBF_ParseMessage<sensor_msgs::JointState>(state);
}
BENCHMARK( RBF_ParseMessageJointState );

//------------------------------------------

static void RTI_ParseMessageOdom( benchmark::State &state )
{
  RTI_ParseMessage<nav_msgs::Odometry>(state);
}
BENCHMARK( RTI_ParseMessageOdom );


static void RBF_ParseMessageOdom( benchmark::State &state )
{
  RBF_ParseMessage<nav_msgs::Odometry>(state);
}
BENCHMARK( RBF_ParseMessageOdom );

//------------------------------------------

static void RTI_ParseMessageTF( benchmark::State &state )
{
  RTI_ParseMessage<tf::tfMessage>(state);
}
BENCHMARK( RTI_ParseMessageTF );


static void RBF_ParseMessageTF( benchmark::State &state )
{
  RBF_ParseMessage<tf::tfMessage>(state);
}
BENCHMARK( RBF_ParseMessageTF );

//------------------------------------------

static void RTI_ParseMessagePointcloud( benchmark::State &state )
{
  RTI_ParseMessage<sensor_msgs::PointCloud2>(state);
}

BENCHMARK( RTI_ParseMessagePointcloud );

static void RBF_ParseMessagePointcloud( benchmark::State &state )
{
  RBF_ParseMessage<sensor_msgs::PointCloud2>(state);
}
BENCHMARK( RBF_ParseMessagePointcloud );

//------------------------------------------

static void RTI_ParseMessageFullHDImage( benchmark::State &state )
{
  RTI_ParseMessage<sensor_msgs::Image>(state);
}
BENCHMARK( RTI_ParseMessageFullHDImage );


static void RBF_ParseMessageFullHDImage( benchmark::State &state )
{
  RBF_ParseMessage<sensor_msgs::Image>(state);
}
BENCHMARK( RBF_ParseMessageFullHDImage );

//------------------------------------------

template <typename MsgType, typename SubMsgType> inline
    void RTI_ParseSubMessage( benchmark::State &state )
{
  const std::string &message_type = mt::DataType<MsgType>::value();
  const std::string& submessage_type = mt::DataType<SubMsgType>::value();

  const std::string& definition = mt::Definition<MsgType>::value();
  const std::string &md5 = mt::MD5Sum<MsgType>::value();

  RosIntrospection::Parser parser;
  parser.registerMessageDefinition( message_type, RosIntrospection::ROSType( message_type ), definition);

  parser.registerMessageDefinition( submessage_type, RosIntrospection::ROSType( submessage_type ),
                                    mt::Definition<SubMsgType>::value() );

  RosIntrospection::FlatMessage flat_container;
  RosIntrospection::RenamedValues renamed_value;

  MsgType msg;
  fillMessage(msg);

  ros::SerializedMessage serialized_msg = ros::serialization::serializeMessage( msg );
  RosIntrospection::ShapeShifter shape_shifter;
  shape_shifter.morph( md5, message_type, definition );
  ros::serialization::deserializeMessage( serialized_msg, shape_shifter );

  parser.setBlobPolicy( RosIntrospection::Parser::STORE_BLOB_AS_REFERENCE );

  for ( auto _ : state )
  {
    auto buffer = absl::Span<uint8_t>( (uint8_t*)shape_shifter.raw_data(), shape_shifter.size() );
    auto sub_buffer = parser.findSubField(message_type, RosIntrospection::ROSType(submessage_type), buffer);

    parser.deserializeIntoFlatContainer( submessage_type, sub_buffer, &flat_container, 10 );
    parser.applyNameTransform( submessage_type, flat_container, &renamed_value );
  }
}

static void RBF_ParseSubMessagePoseFromOdom( benchmark::State &state )
{
  RTI_ParseSubMessage<nav_msgs::Odometry, geometry_msgs::Pose>(state);
}
BENCHMARK( RBF_ParseSubMessagePoseFromOdom );

BENCHMARK_MAIN();
