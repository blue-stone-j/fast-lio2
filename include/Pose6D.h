


#ifndef FAST_LIO_MESSAGE_POSE6D_H
#define FAST_LIO_MESSAGE_POSE6D_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace fast_lio
{
template <class ContainerAllocator>
struct Pose6D_
{
  typedef Pose6D_<ContainerAllocator> Type;

  Pose6D_()
    : offset_time(0.0)
    , acc()
    , gyr()
    , vel()
    , pos()
    , rot()  {
      acc.assign(0.0);

      gyr.assign(0.0);

      vel.assign(0.0);

      pos.assign(0.0);

      rot.assign(0.0);
  }
  Pose6D_(const ContainerAllocator& _alloc)
    : offset_time(0.0)
    , acc()
    , gyr()
    , vel()
    , pos()
    , rot()  {
  (void)_alloc;
      acc.assign(0.0);

      gyr.assign(0.0);

      vel.assign(0.0);

      pos.assign(0.0);

      rot.assign(0.0);
  }



   typedef double _offset_time_type;
  _offset_time_type offset_time;

   typedef boost::array<double, 3>  _acc_type;
  _acc_type acc;

   typedef boost::array<double, 3>  _gyr_type;
  _gyr_type gyr;

   typedef boost::array<double, 3>  _vel_type;
  _vel_type vel;

   typedef boost::array<double, 3>  _pos_type;
  _pos_type pos;

   typedef boost::array<double, 9>  _rot_type;
  _rot_type rot;





  typedef boost::shared_ptr< ::fast_lio::Pose6D_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fast_lio::Pose6D_<ContainerAllocator> const> ConstPtr;

}; // struct Pose6D_

typedef ::fast_lio::Pose6D_<std::allocator<void> > Pose6D;

typedef boost::shared_ptr< ::fast_lio::Pose6D > Pose6DPtr;
typedef boost::shared_ptr< ::fast_lio::Pose6D const> Pose6DConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::fast_lio::Pose6D_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::fast_lio::Pose6D_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::fast_lio::Pose6D_<ContainerAllocator1> & lhs, const ::fast_lio::Pose6D_<ContainerAllocator2> & rhs)
{
  return lhs.offset_time == rhs.offset_time &&
    lhs.acc == rhs.acc &&
    lhs.gyr == rhs.gyr &&
    lhs.vel == rhs.vel &&
    lhs.pos == rhs.pos &&
    lhs.rot == rhs.rot;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::fast_lio::Pose6D_<ContainerAllocator1> & lhs, const ::fast_lio::Pose6D_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace fast_lio

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::fast_lio::Pose6D_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::fast_lio::Pose6D_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::fast_lio::Pose6D_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::fast_lio::Pose6D_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fast_lio::Pose6D_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fast_lio::Pose6D_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::fast_lio::Pose6D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ab486e9c24704038320abf9ff59003d2";
  }

  static const char* value(const ::fast_lio::Pose6D_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xab486e9c24704038ULL;
  static const uint64_t static_value2 = 0x320abf9ff59003d2ULL;
};

template<class ContainerAllocator>
struct DataType< ::fast_lio::Pose6D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fast_lio/Pose6D";
  }

  static const char* value(const ::fast_lio::Pose6D_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::fast_lio::Pose6D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# the preintegrated Lidar states at the time of IMU measurements in a frame\n"
"float64  offset_time # the offset time of IMU measurement w.r.t the first lidar point\n"
"float64[3] acc       # the preintegrated total acceleration (global frame) at the Lidar origin\n"
"float64[3] gyr       # the unbiased angular velocity (body frame) at the Lidar origin\n"
"float64[3] vel       # the preintegrated velocity (global frame) at the Lidar origin\n"
"float64[3] pos       # the preintegrated position (global frame) at the Lidar origin\n"
"float64[9] rot       # the preintegrated rotation (global frame) at the Lidar origin\n"
;
  }

  static const char* value(const ::fast_lio::Pose6D_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::fast_lio::Pose6D_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.offset_time);
      stream.next(m.acc);
      stream.next(m.gyr);
      stream.next(m.vel);
      stream.next(m.pos);
      stream.next(m.rot);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Pose6D_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::fast_lio::Pose6D_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::fast_lio::Pose6D_<ContainerAllocator>& v)
  {
    s << indent << "offset_time: ";
    Printer<double>::stream(s, indent + "  ", v.offset_time);
    s << indent << "acc[]" << std::endl;
    for (size_t i = 0; i < v.acc.size(); ++i)
    {
      s << indent << "  acc[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.acc[i]);
    }
    s << indent << "gyr[]" << std::endl;
    for (size_t i = 0; i < v.gyr.size(); ++i)
    {
      s << indent << "  gyr[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.gyr[i]);
    }
    s << indent << "vel[]" << std::endl;
    for (size_t i = 0; i < v.vel.size(); ++i)
    {
      s << indent << "  vel[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.vel[i]);
    }
    s << indent << "pos[]" << std::endl;
    for (size_t i = 0; i < v.pos.size(); ++i)
    {
      s << indent << "  pos[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.pos[i]);
    }
    s << indent << "rot[]" << std::endl;
    for (size_t i = 0; i < v.rot.size(); ++i)
    {
      s << indent << "  rot[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.rot[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // fast_lio_MESSAGE_POSE6D_H