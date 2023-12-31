#include "web_video_server/ros_compressed_streamer.h"
#include "web_video_server/crc32.h"

namespace web_video_server
{

RosCompressedStreamer::RosCompressedStreamer(const async_web_server_cpp::HttpRequest &request,
                             async_web_server_cpp::HttpConnectionPtr connection, ros::NodeHandle& nh) :
  ImageStreamer(request, connection, nh), stream_(connection)
{
  stream_.sendInitialHeader();
}

RosCompressedStreamer::~RosCompressedStreamer()
{
  this->inactive_ = true;
  boost::mutex::scoped_lock lock(send_mutex_); // protects sendImage.
}

void RosCompressedStreamer::start() {
  std::string compressed_topic = topic_ + "/compressed";
  image_sub_ = nh_.subscribe(compressed_topic, 1, &RosCompressedStreamer::imageCallback, this);
}

void RosCompressedStreamer::restreamFrame(double max_age)
{
  if (inactive_ || (last_msg == 0))
    return;

  if ( last_frame + ros::Duration(max_age) < ros::Time::now() ) {
    boost::mutex::scoped_lock lock(send_mutex_);
    sendImage(last_msg, ros::Time::now() ); // don't update last_frame, it may remain an old value.
  }
}

void RosCompressedStreamer::sendImage(const sensor_msgs::CompressedImageConstPtr &msg,
                                      const ros::Time &time) {
  try {
    std::string content_type;
    if(msg->format.find("jpeg") != std::string::npos) {
      content_type = "image/jpeg";
    }
    else if(msg->format.find("png") != std::string::npos) {
      content_type = "image/png";
      convertGrayscale16toGrayscaleAlpha(msg);
    }
    else {
      ROS_WARN_STREAM("Unknown ROS compressed image format: " << msg->format);
      return;
    }

    stream_.sendPart(time, content_type, boost::asio::buffer(msg->data), msg);
  }
  catch (boost::system::system_error &e)
  {
    // happens when client disconnects
    ROS_DEBUG("system_error exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (std::exception &e)
  {
    ROS_ERROR_THROTTLE(30, "exception: %s", e.what());
    inactive_ = true;
    return;
  }
  catch (...)
  {
    ROS_ERROR_THROTTLE(30, "exception");
    inactive_ = true;
    return;
  }
}

void RosCompressedStreamer::convertGrayscale16toGrayscaleAlpha(const sensor_msgs::CompressedImageConstPtr &msg)
{
  static const uint8_t PNG_MAGIC_BYTES_AND_IHDR[16] = {
    0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n',
    '\0', '\0', '\0', '\r', 'I', 'H', 'D', 'R',
  };
  const std::vector<uint8_t>& data = msg->data;
  if (data.size() <= 33 || memcmp(&data[0], PNG_MAGIC_BYTES_AND_IHDR, 16))
  {
    return;
  }

  static const uint8_t GRAYSCALE16[2] = {16, 0};
  static const uint8_t GRAYSCALE_ALPHA[2] = {8, 4};

  // check if 16-bit grayscale
  if (memcmp(&data[0x18], GRAYSCALE16, 2) == 0)
  {
    uint32_t crc = 0xffffffff;
    crc = updateCrc32(crc, &data[0x0C], 12);
    crc = updateCrc32(crc, GRAYSCALE_ALPHA, 2);
    crc = updateCrc32(crc, &data[0x1A], 3);
    crc = ~crc;

    *reinterpret_cast<uint32_t*>(const_cast<uint8_t*>(&data[0x1D])) = __builtin_bswap32(crc);
    *reinterpret_cast<uint16_t*>(const_cast<uint8_t*>(&data[0x18])) = *reinterpret_cast<const uint16_t*>(GRAYSCALE_ALPHA);
  }
}

void RosCompressedStreamer::imageCallback(const sensor_msgs::CompressedImageConstPtr &msg) {
  boost::mutex::scoped_lock lock(send_mutex_); // protects last_msg and last_frame
  last_msg = msg;
  last_frame = ros::Time(msg->header.stamp.sec, msg->header.stamp.nsec);
  sendImage(last_msg, last_frame);
}


boost::shared_ptr<ImageStreamer> RosCompressedStreamerType::create_streamer(const async_web_server_cpp::HttpRequest &request,
										 async_web_server_cpp::HttpConnectionPtr connection,
										 ros::NodeHandle& nh)
{
  return boost::shared_ptr<ImageStreamer>(new RosCompressedStreamer(request, connection, nh));
}

std::string RosCompressedStreamerType::create_viewer(const async_web_server_cpp::HttpRequest &request)
{
  std::stringstream ss;
  ss << "<img src=\"/stream?";
  ss << request.query;
  ss << "\"></img>";
  return ss.str();
}


}
