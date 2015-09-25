#ifndef _UTIL_PROTO_UTIL_H__
#define _UTIL_PROTO_UTIL_H__

#include <glog/logging.h>
#include <google/protobuf/message.h>
#include <google/protobuf/text_format.h>

#include <fstream>
#include <string>

namespace mds_util {

inline bool ProtocolBuffersEqual(const google::protobuf::Message& msg1,
                                 const google::protobuf::Message& msg2) {
  std::string str1, str2;
  if (!google::protobuf::TextFormat::PrintToString(msg1, &str1) ||
      !google::protobuf::TextFormat::PrintToString(msg2, &str2)) {
    LOG(ERROR) << "Failed to print proto";
    return false;
  }

  return str1 == str2;
}

inline bool WriteASCIIProtoToFile(const std::string& filename,
                                  const google::protobuf::Message& msg) {
  std::string out_string;
  if (!google::protobuf::TextFormat::PrintToString(msg, &out_string)) {
    LOG(ERROR) << "Failed to print " << msg.DebugString();
    return false;
  }

  std::fstream output(filename.c_str(), std::fstream::out | std::fstream::trunc);
  output << out_string;
  output.close();
  return output.good();
}

inline bool SerializeProtoToFile(const std::string& filename,
                                 const google::protobuf::Message& msg) {
  std::fstream output(filename.c_str(),
                      std::fstream::out |
                      std::fstream::trunc |
                      std::fstream::binary);
  if (!msg.SerializeToOstream(&output)) {
    LOG(ERROR) << "Failed to print to " << filename << ": " << msg.DebugString();
    return false;
  }
  output.close();
  return output.good();
}

template <class P>
bool ReadProtoFromFile(const std::string& filename, P* msg) {
  std::ifstream ifs(filename.c_str());
  if (!ifs) {
    LOG(WARNING) << "Failed to read file " << filename;
  } else {
    std::string in_str((std::istreambuf_iterator<char>(ifs)),
                       (std::istreambuf_iterator<char>()));
    if (google::protobuf::TextFormat::ParseFromString(in_str, msg)) {
      return true;
    }
  }

  VLOG(1) << "Failed to parse " << filename << " as ascii; trying binary.";
  std::ifstream bin_ifs(filename.c_str(),
                        std::fstream::in | std::fstream::binary);
  if (!bin_ifs) {
    LOG(ERROR) << "Failed to read file " << filename << " as binary";
    return false;
  }
  return msg->ParseFromIstream(&bin_ifs);
}

}  // mds_util

#endif  // _UTIL_PROTO_UTIL_H__
