// This is an automatically generated file.
// Generated from this PointCloudYarp.msg definition:
//   uint32 w
//   uint32 h
//   float32[] x
//   float32[] y
//   float32[] z
//   
// Instances of this class can be read and written with YARP ports,
// using a ROS-compatible format.

#ifndef YARPMSG_TYPE_PointCloudYarp
#define YARPMSG_TYPE_PointCloudYarp

#include <string>
#include <vector>
#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class PointCloudYarp : public yarp::os::idl::WirePortable {
public:
  yarp::os::NetUint32 w;
  yarp::os::NetUint32 h;
  std::vector<yarp::os::NetFloat32> x;
  std::vector<yarp::os::NetFloat32> y;
  std::vector<yarp::os::NetFloat32> z;

  PointCloudYarp() {
  }

  bool readBare(yarp::os::ConnectionReader& connection) {
    // *** w ***
    w = connection.expectInt();

    // *** h ***
    h = connection.expectInt();

    // *** x ***
    int len = connection.expectInt();
    x.resize(len);
    if (!connection.expectBlock((char*)&x[0],sizeof(yarp::os::NetFloat32)*len)) return false;

    // *** y ***
    len = connection.expectInt();
    y.resize(len);
    if (!connection.expectBlock((char*)&y[0],sizeof(yarp::os::NetFloat32)*len)) return false;

    // *** z ***
    len = connection.expectInt();
    z.resize(len);
    if (!connection.expectBlock((char*)&z[0],sizeof(yarp::os::NetFloat32)*len)) return false;
    return !connection.isError();
  }

  bool readBottle(yarp::os::ConnectionReader& connection) {
    connection.convertTextMode();
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListHeader(5)) return false;

    // *** w ***
    w = reader.expectInt();

    // *** h ***
    h = reader.expectInt();

    // *** x ***
    if (connection.expectInt()!=(BOTTLE_TAG_LIST|BOTTLE_TAG_DOUBLE)) return false;
    int len = connection.expectInt();
    x.resize(len);
    for (size_t i=0; i<len; i++) {
      x[i] = (yarp::os::NetFloat32)connection.expectDouble();
    }

    // *** y ***
    if (connection.expectInt()!=(BOTTLE_TAG_LIST|BOTTLE_TAG_DOUBLE)) return false;
    len = connection.expectInt();
    y.resize(len);
    for (size_t i=0; i<len; i++) {
      y[i] = (yarp::os::NetFloat32)connection.expectDouble();
    }

    // *** z ***
    if (connection.expectInt()!=(BOTTLE_TAG_LIST|BOTTLE_TAG_DOUBLE)) return false;
    len = connection.expectInt();
    z.resize(len);
    for (size_t i=0; i<len; i++) {
      z[i] = (yarp::os::NetFloat32)connection.expectDouble();
    }
    return !connection.isError();
  }

  bool read(yarp::os::ConnectionReader& connection) {
    if (connection.isBareMode()) return readBare(connection);
    return readBottle(connection);
  }

  bool writeBare(yarp::os::ConnectionWriter& connection) {
    // *** w ***
    connection.appendInt(w);

    // *** h ***
    connection.appendInt(h);

    // *** x ***
    connection.appendInt(x.size());
    connection.appendExternalBlock((char*)&x[0],sizeof(yarp::os::NetFloat32)*x.size());

    // *** y ***
    connection.appendInt(y.size());
    connection.appendExternalBlock((char*)&y[0],sizeof(yarp::os::NetFloat32)*y.size());

    // *** z ***
    connection.appendInt(z.size());
    connection.appendExternalBlock((char*)&z[0],sizeof(yarp::os::NetFloat32)*z.size());
    return !connection.isError();
  }

  bool writeBottle(yarp::os::ConnectionWriter& connection) {
    connection.appendInt(BOTTLE_TAG_LIST);
    connection.appendInt(5);

    // *** w ***
    connection.appendInt(BOTTLE_TAG_INT);
    connection.appendInt((int)w);

    // *** h ***
    connection.appendInt(BOTTLE_TAG_INT);
    connection.appendInt((int)h);

    // *** x ***
    connection.appendInt(BOTTLE_TAG_LIST|BOTTLE_TAG_DOUBLE);
    connection.appendInt(x.size());
    for (size_t i=0; i<x.size(); i++) {
      connection.appendDouble((double)x[i]);
    }

    // *** y ***
    connection.appendInt(BOTTLE_TAG_LIST|BOTTLE_TAG_DOUBLE);
    connection.appendInt(y.size());
    for (size_t i=0; i<y.size(); i++) {
      connection.appendDouble((double)y[i]);
    }

    // *** z ***
    connection.appendInt(BOTTLE_TAG_LIST|BOTTLE_TAG_DOUBLE);
    connection.appendInt(z.size());
    for (size_t i=0; i<z.size(); i++) {
      connection.appendDouble((double)z[i]);
    }
    connection.convertTextMode();
    return !connection.isError();
  }

  bool write(yarp::os::ConnectionWriter& connection) {
    if (connection.isBareMode()) return writeBare(connection);
    return writeBottle(connection);
  }

  // This class will serialize ROS style or YARP style depending on protocol.
  // If you need to force a serialization style, use one of these classes:
  typedef yarp::os::idl::BareStyle<PointCloudYarp> rosStyle;
  typedef yarp::os::idl::BottleStyle<PointCloudYarp> bottleStyle;

  // Give source text for class, ROS will need this
  yarp::os::ConstString getTypeText() {
    return "uint32 w\n\
uint32 h\n\
float32[] x\n\
float32[] y\n\
float32[] z\n\
";
  }

  // Name the class, ROS will need this
  yarp::os::Type getType() {
    yarp::os::Type typ = yarp::os::Type::byName("PointCloudYarp","PointCloudYarp");
    typ.addProperty("md5sum",yarp::os::Value("e1bf94037ca25064d94d03ea280689ec"));
    typ.addProperty("message_definition",yarp::os::Value(getTypeText()));
    return typ;
  }
};

#endif
