// Generated by gencpp from file dstar_nav/edge.msg
// DO NOT EDIT!


#ifndef DSTAR_NAV_MESSAGE_EDGE_H
#define DSTAR_NAV_MESSAGE_EDGE_H

#include <ros/service_traits.h>


#include <dstar_nav/edgeRequest.h>
#include <dstar_nav/edgeResponse.h>


namespace dstar_nav
{

struct edge
{

typedef edgeRequest Request;
typedef edgeResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct edge
} // namespace dstar_nav


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dstar_nav::edge > {
  static const char* value()
  {
    return "acc72f313e3078f8990785d2053a66d5";
  }

  static const char* value(const ::dstar_nav::edge&) { return value(); }
};

template<>
struct DataType< ::dstar_nav::edge > {
  static const char* value()
  {
    return "dstar_nav/edge";
  }

  static const char* value(const ::dstar_nav::edge&) { return value(); }
};


// service_traits::MD5Sum< ::dstar_nav::edgeRequest> should match 
// service_traits::MD5Sum< ::dstar_nav::edge > 
template<>
struct MD5Sum< ::dstar_nav::edgeRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dstar_nav::edge >::value();
  }
  static const char* value(const ::dstar_nav::edgeRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dstar_nav::edgeRequest> should match 
// service_traits::DataType< ::dstar_nav::edge > 
template<>
struct DataType< ::dstar_nav::edgeRequest>
{
  static const char* value()
  {
    return DataType< ::dstar_nav::edge >::value();
  }
  static const char* value(const ::dstar_nav::edgeRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dstar_nav::edgeResponse> should match 
// service_traits::MD5Sum< ::dstar_nav::edge > 
template<>
struct MD5Sum< ::dstar_nav::edgeResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dstar_nav::edge >::value();
  }
  static const char* value(const ::dstar_nav::edgeResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dstar_nav::edgeResponse> should match 
// service_traits::DataType< ::dstar_nav::edge > 
template<>
struct DataType< ::dstar_nav::edgeResponse>
{
  static const char* value()
  {
    return DataType< ::dstar_nav::edge >::value();
  }
  static const char* value(const ::dstar_nav::edgeResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DSTAR_NAV_MESSAGE_EDGE_H
