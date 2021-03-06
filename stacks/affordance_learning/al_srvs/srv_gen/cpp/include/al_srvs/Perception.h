/* Auto-generated by genmsg_cpp for file /home/kadir/workspace/work/Dropbox/metu-ros-pkg/trunk/affordance_learning/al_srvs/srv/Perception.srv */
#ifndef AL_SRVS_SERVICE_PERCEPTION_H
#define AL_SRVS_SERVICE_PERCEPTION_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "ros/service_traits.h"




namespace al_srvs
{
template <class ContainerAllocator>
struct PerceptionRequest_ {
  typedef PerceptionRequest_<ContainerAllocator> Type;

  PerceptionRequest_()
  : task(0)
  , arg(0)
  , arg_effect(0)
  {
  }

  PerceptionRequest_(const ContainerAllocator& _alloc)
  : task(0)
  , arg(0)
  , arg_effect(0)
  {
  }

  typedef uint8_t _task_type;
  uint8_t task;

  typedef int8_t _arg_type;
  int8_t arg;

  typedef int8_t _arg_effect_type;
  int8_t arg_effect;

  enum { DO_PERCEPT = 0 };
  enum { EXTRACT_EFFECT = 1 };

  typedef boost::shared_ptr< ::al_srvs::PerceptionRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::al_srvs::PerceptionRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PerceptionRequest
typedef  ::al_srvs::PerceptionRequest_<std::allocator<void> > PerceptionRequest;

typedef boost::shared_ptr< ::al_srvs::PerceptionRequest> PerceptionRequestPtr;
typedef boost::shared_ptr< ::al_srvs::PerceptionRequest const> PerceptionRequestConstPtr;


template <class ContainerAllocator>
struct PerceptionResponse_ {
  typedef PerceptionResponse_<ContainerAllocator> Type;

  PerceptionResponse_()
  : feedback(0)
  , pushable_object_center()
  , pushable_object_size()
  , pushable_object_yaw(0.0)
  {
  }

  PerceptionResponse_(const ContainerAllocator& _alloc)
  : feedback(0)
  , pushable_object_center(_alloc)
  , pushable_object_size(_alloc)
  , pushable_object_yaw(0.0)
  {
  }

  typedef uint8_t _feedback_type;
  uint8_t feedback;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _pushable_object_center_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  pushable_object_center;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _pushable_object_size_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  pushable_object_size;

  typedef float _pushable_object_yaw_type;
  float pushable_object_yaw;

  enum { DONE = 1 };

  typedef boost::shared_ptr< ::al_srvs::PerceptionResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::al_srvs::PerceptionResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PerceptionResponse
typedef  ::al_srvs::PerceptionResponse_<std::allocator<void> > PerceptionResponse;

typedef boost::shared_ptr< ::al_srvs::PerceptionResponse> PerceptionResponsePtr;
typedef boost::shared_ptr< ::al_srvs::PerceptionResponse const> PerceptionResponseConstPtr;

struct Perception
{

typedef PerceptionRequest Request;
typedef PerceptionResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct Perception
} // namespace al_srvs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::al_srvs::PerceptionRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::al_srvs::PerceptionRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::al_srvs::PerceptionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d769a2df318cc1e0668e0936b22d5b19";
  }

  static const char* value(const  ::al_srvs::PerceptionRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd769a2df318cc1e0ULL;
  static const uint64_t static_value2 = 0x668e0936b22d5b19ULL;
};

template<class ContainerAllocator>
struct DataType< ::al_srvs::PerceptionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "al_srvs/PerceptionRequest";
  }

  static const char* value(const  ::al_srvs::PerceptionRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::al_srvs::PerceptionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
uint8 DO_PERCEPT = 0\n\
uint8 EXTRACT_EFFECT = 1\n\
uint8 task\n\
int8 arg\n\
int8 arg_effect\n\
\n\
";
  }

  static const char* value(const  ::al_srvs::PerceptionRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::al_srvs::PerceptionRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::al_srvs::PerceptionResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::al_srvs::PerceptionResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::al_srvs::PerceptionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c4c0a5df2fd44ed04c035679558f2b4f";
  }

  static const char* value(const  ::al_srvs::PerceptionResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xc4c0a5df2fd44ed0ULL;
  static const uint64_t static_value2 = 0x4c035679558f2b4fULL;
};

template<class ContainerAllocator>
struct DataType< ::al_srvs::PerceptionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "al_srvs/PerceptionResponse";
  }

  static const char* value(const  ::al_srvs::PerceptionResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::al_srvs::PerceptionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
uint8 DONE = 1\n\
uint8 feedback\n\
\n\
float32[] pushable_object_center\n\
float32[] pushable_object_size\n\
float32 pushable_object_yaw\n\
\n\
\n\
";
  }

  static const char* value(const  ::al_srvs::PerceptionResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::al_srvs::PerceptionRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.task);
    stream.next(m.arg);
    stream.next(m.arg_effect);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PerceptionRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::al_srvs::PerceptionResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.feedback);
    stream.next(m.pushable_object_center);
    stream.next(m.pushable_object_size);
    stream.next(m.pushable_object_yaw);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PerceptionResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<al_srvs::Perception> {
  static const char* value() 
  {
    return "62571064ec70ad507aaf517f70cf7e3d";
  }

  static const char* value(const al_srvs::Perception&) { return value(); } 
};

template<>
struct DataType<al_srvs::Perception> {
  static const char* value() 
  {
    return "al_srvs/Perception";
  }

  static const char* value(const al_srvs::Perception&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<al_srvs::PerceptionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "62571064ec70ad507aaf517f70cf7e3d";
  }

  static const char* value(const al_srvs::PerceptionRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<al_srvs::PerceptionRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "al_srvs/Perception";
  }

  static const char* value(const al_srvs::PerceptionRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<al_srvs::PerceptionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "62571064ec70ad507aaf517f70cf7e3d";
  }

  static const char* value(const al_srvs::PerceptionResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<al_srvs::PerceptionResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "al_srvs/Perception";
  }

  static const char* value(const al_srvs::PerceptionResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // AL_SRVS_SERVICE_PERCEPTION_H

