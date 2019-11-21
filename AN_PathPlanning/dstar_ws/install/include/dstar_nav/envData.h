// Generated by gencpp from file dstar_nav/envData.msg
// DO NOT EDIT!


#ifndef DSTAR_NAV_MESSAGE_ENVDATA_H
#define DSTAR_NAV_MESSAGE_ENVDATA_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace dstar_nav
{
template <class ContainerAllocator>
struct envData_
{
  typedef envData_<ContainerAllocator> Type;

  envData_()
    : setMap(false)
    , cliff(false)
    , x(0)
    , y(0)
    , z(0)
    , val(0.0)
    , vectors()  {
    }
  envData_(const ContainerAllocator& _alloc)
    : setMap(false)
    , cliff(false)
    , x(0)
    , y(0)
    , z(0)
    , val(0.0)
    , vectors(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _setMap_type;
  _setMap_type setMap;

   typedef uint8_t _cliff_type;
  _cliff_type cliff;

   typedef int16_t _x_type;
  _x_type x;

   typedef int16_t _y_type;
  _y_type y;

   typedef int16_t _z_type;
  _z_type z;

   typedef float _val_type;
  _val_type val;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _vectors_type;
  _vectors_type vectors;





  typedef boost::shared_ptr< ::dstar_nav::envData_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dstar_nav::envData_<ContainerAllocator> const> ConstPtr;

}; // struct envData_

typedef ::dstar_nav::envData_<std::allocator<void> > envData;

typedef boost::shared_ptr< ::dstar_nav::envData > envDataPtr;
typedef boost::shared_ptr< ::dstar_nav::envData const> envDataConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dstar_nav::envData_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dstar_nav::envData_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace dstar_nav

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'dstar_nav': ['/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_PathPlanning/dstar_ws/src/dstar_nav/msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::dstar_nav::envData_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dstar_nav::envData_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dstar_nav::envData_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dstar_nav::envData_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dstar_nav::envData_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dstar_nav::envData_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dstar_nav::envData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1e43a5c8c54c87cddb3fbc5f19377b83";
  }

  static const char* value(const ::dstar_nav::envData_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1e43a5c8c54c87cdULL;
  static const uint64_t static_value2 = 0xdb3fbc5f19377b83ULL;
};

template<class ContainerAllocator>
struct DataType< ::dstar_nav::envData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dstar_nav/envData";
  }

  static const char* value(const ::dstar_nav::envData_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dstar_nav::envData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool setMap\n"
"bool cliff\n"
"int16 x\n"
"int16 y\n"
"int16 z\n"
"float32 val\n"
"float32[] vectors\n"
;
  }

  static const char* value(const ::dstar_nav::envData_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dstar_nav::envData_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.setMap);
      stream.next(m.cliff);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.val);
      stream.next(m.vectors);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct envData_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dstar_nav::envData_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dstar_nav::envData_<ContainerAllocator>& v)
  {
    s << indent << "setMap: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.setMap);
    s << indent << "cliff: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cliff);
    s << indent << "x: ";
    Printer<int16_t>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<int16_t>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<int16_t>::stream(s, indent + "  ", v.z);
    s << indent << "val: ";
    Printer<float>::stream(s, indent + "  ", v.val);
    s << indent << "vectors[]" << std::endl;
    for (size_t i = 0; i < v.vectors.size(); ++i)
    {
      s << indent << "  vectors[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.vectors[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // DSTAR_NAV_MESSAGE_ENVDATA_H
