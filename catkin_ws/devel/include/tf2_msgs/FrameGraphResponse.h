// Generated by gencpp from file tf2_msgs/FrameGraphResponse.msg
// DO NOT EDIT!


#ifndef TF2_MSGS_MESSAGE_FRAMEGRAPHRESPONSE_H
#define TF2_MSGS_MESSAGE_FRAMEGRAPHRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace tf2_msgs
{
template <class ContainerAllocator>
struct FrameGraphResponse_
{
  typedef FrameGraphResponse_<ContainerAllocator> Type;

  FrameGraphResponse_()
    : frame_yaml()  {
    }
  FrameGraphResponse_(const ContainerAllocator& _alloc)
    : frame_yaml(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _frame_yaml_type;
  _frame_yaml_type frame_yaml;





  typedef boost::shared_ptr< ::tf2_msgs::FrameGraphResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tf2_msgs::FrameGraphResponse_<ContainerAllocator> const> ConstPtr;

}; // struct FrameGraphResponse_

typedef ::tf2_msgs::FrameGraphResponse_<std::allocator<void> > FrameGraphResponse;

typedef boost::shared_ptr< ::tf2_msgs::FrameGraphResponse > FrameGraphResponsePtr;
typedef boost::shared_ptr< ::tf2_msgs::FrameGraphResponse const> FrameGraphResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tf2_msgs::FrameGraphResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::tf2_msgs::FrameGraphResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::tf2_msgs::FrameGraphResponse_<ContainerAllocator1> & lhs, const ::tf2_msgs::FrameGraphResponse_<ContainerAllocator2> & rhs)
{
  return lhs.frame_yaml == rhs.frame_yaml;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::tf2_msgs::FrameGraphResponse_<ContainerAllocator1> & lhs, const ::tf2_msgs::FrameGraphResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace tf2_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::tf2_msgs::FrameGraphResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tf2_msgs::FrameGraphResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tf2_msgs::FrameGraphResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tf2_msgs::FrameGraphResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tf2_msgs::FrameGraphResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tf2_msgs::FrameGraphResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::tf2_msgs::FrameGraphResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "437ea58e9463815a0d511c7326b686b0";
  }

  static const char* value(const ::tf2_msgs::FrameGraphResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x437ea58e9463815aULL;
  static const uint64_t static_value2 = 0x0d511c7326b686b0ULL;
};

template<class ContainerAllocator>
struct DataType< ::tf2_msgs::FrameGraphResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tf2_msgs/FrameGraphResponse";
  }

  static const char* value(const ::tf2_msgs::FrameGraphResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::tf2_msgs::FrameGraphResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string frame_yaml\n"
"\n"
;
  }

  static const char* value(const ::tf2_msgs::FrameGraphResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::tf2_msgs::FrameGraphResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.frame_yaml);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FrameGraphResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tf2_msgs::FrameGraphResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::tf2_msgs::FrameGraphResponse_<ContainerAllocator>& v)
  {
    s << indent << "frame_yaml: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.frame_yaml);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TF2_MSGS_MESSAGE_FRAMEGRAPHRESPONSE_H