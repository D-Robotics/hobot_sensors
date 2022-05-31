// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "hobot_usb_cam.hpp"

#include <rclcpp/rclcpp.hpp>

extern "C" {
#include <assert.h>
#include <errno.h>
#include <fcntl.h> /* low-level i/o */
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
}

namespace hobot_usb_cam {

#define CLEAR(x) memset(&(x), 0, sizeof(x))

HobotUSBCam::HobotUSBCam()
    : cam_state_(kSTATE_UNINITIALLED), cam_fd_(-1), buffer_numbers_(4) {
  buffers_ = new CamBuffer[buffer_numbers_];
}

HobotUSBCam::~HobotUSBCam() { delete[] buffers_; }

bool HobotUSBCam::Init(CamInformation &cam_information) {
  bool ret = true;
  std::lock_guard<std::mutex> lock(cam_mutex_);
  if (cam_state_ != kSTATE_UNINITIALLED) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_usb_cam"),
                 "Cmera state is not kSTATE_UNINITIALLED, current state:%d\n",
                 cam_state_);
    return false;
  }

  cam_information_ = cam_information;
  // memcpy(&cam_information_, cam_information, sizeof(CamInformation));

  if (OpenDevice() == false)
    return false;
  if (InitDevice() == false) {
    CloseDevice();
    return false;
  }
  cam_information = cam_information_;
  cam_state_ = kSTATE_INITIALLED;
  return ret;
}

bool HobotUSBCam::OpenDevice(void) {
  struct stat dev_stat;
  if (stat(cam_information_.dev.c_str(), &dev_stat) == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_usb_cam"),
                 "Cannot identify '%s': %d, %s\\n",
                 cam_information_.dev.c_str(), errno, strerror(errno));
    return false;
  }
  if (!S_ISCHR(dev_stat.st_mode)) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_usb_cam"),
                 "%s is not a character device\n", cam_information_.dev.c_str(),
                 errno, strerror(errno));
    return false;
  }
  cam_fd_ = open(cam_information_.dev.c_str(), O_RDWR | O_NONBLOCK, 0);
  if (cam_fd_ == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_usb_cam"),
                 "Cannot open '%s': %d, %s\\n", cam_information_.dev.c_str(),
                 errno, strerror(errno));
    return false;
  }
  return true;
}

bool HobotUSBCam::InitDevice() {
  struct v4l2_capability camera_capability;
  struct v4l2_cropcap camera_cropcap;
  struct v4l2_crop camera_crop;
  struct v4l2_format camera_format;
  unsigned int min;

  if (xioctl(cam_fd_, VIDIOC_QUERYCAP, &camera_capability) == -1) {
    if (EINVAL == errno) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_usb_cam"),
                   "%s is no V4L2 device\\n", cam_information_.dev.c_str());
      return false;
    } else {
      errno_exit("VIDIOC_QUERYCAP");
    }
  }

  if (!(camera_capability.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_usb_cam"),
                 "%s is no video capture device\\n",
                 cam_information_.dev.c_str());
    return false;
  }

  switch (cam_information_.io) {
    case kIO_METHOD_READ:
      if (!(camera_capability.capabilities & V4L2_CAP_READWRITE)) {
        RCLCPP_ERROR(rclcpp::get_logger("hobot_usb_cam"),
                     "%s does not support read i/o\n",
                     cam_information_.dev.c_str());
        return false;
      }
      break;

    case kIO_METHOD_MMAP:
    case kIO_METHOD_USERPTR:
      if (!(camera_capability.capabilities & V4L2_CAP_STREAMING)) {
        RCLCPP_ERROR(rclcpp::get_logger("hobot_usb_cam"),
                     "%s does not support streaming i/o\\n",
                     cam_information_.dev.c_str());
        return false;
      }
      break;
  }
  /* Select video input, video standard and tune here. */
  CLEAR(camera_cropcap);
  camera_cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (xioctl(cam_fd_, VIDIOC_CROPCAP, &camera_cropcap) == 0) {
    camera_crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    camera_crop.c = camera_cropcap.defrect; /* reset to default */

    if (-1 == xioctl(cam_fd_, VIDIOC_S_CROP, &camera_crop)) {
      switch (errno) {
        case EINVAL:
          /* Cropping not supported. */
          break;
        default:
          /* Errors ignored. */
          break;
      }
    }
  } else {
    /* Errors ignored. */
  }
  CLEAR(camera_format);
  camera_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  camera_format.fmt.pix.width = cam_information_.image_width;
  camera_format.fmt.pix.height = cam_information_.image_height;
  switch (cam_information_.pixel_format) {
    case kPIXEL_FORMAT_MJPEG:
      camera_format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
      break;
    case kPIXEL_FORMAT_YUYV:
      camera_format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
      break;
    case kPIXEL_FORMAT_UYVY:
      camera_format.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
      break;
    case kPIXEL_FORMAT_YUVMONO10:
      camera_format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
      break;
    case kPIXEL_FORMAT_RGB24:
      camera_format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
      break;
    case kPIXEL_FORMAT_GREY:
      camera_format.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
      break;
    default:
      camera_format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  }
  camera_format.fmt.pix.field = V4L2_FIELD_INTERLACED;
  if (xioctl(cam_fd_, VIDIOC_S_FMT, &camera_format) == -1)
    errno_exit("VIDIOC_S_FMT");
  if (camera_format.fmt.pix.width == (uint32_t)cam_information_.image_width &&
      camera_format.fmt.pix.height == (uint32_t)cam_information_.image_height) {
    RCLCPP_INFO(rclcpp::get_logger("hobot_usb_cam"),
      "Set resolution to %dx%d\n",
      cam_information_.image_width, cam_information_.image_height);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("hobot_usb_cam"),
      "Resolution %dx%d is not support\n",
      cam_information_.image_width,
      cam_information_.image_height);
    cam_information_.image_width = camera_format.fmt.pix.width;
    cam_information_.image_height = camera_format.fmt.pix.height;
    RCLCPP_WARN(rclcpp::get_logger("hobot_usb_cam"),
      "Usb resolution %dx%d instead\n",
      cam_information_.image_width,
      cam_information_.image_height);
  }
  /* Buggy driver paranoia. */
  min = camera_format.fmt.pix.width * 2;
  if (camera_format.fmt.pix.bytesperline < min)
    camera_format.fmt.pix.bytesperline = min;
  min = camera_format.fmt.pix.bytesperline * camera_format.fmt.pix.height;
  if (camera_format.fmt.pix.sizeimage < min)
    camera_format.fmt.pix.sizeimage = min;

  if (cam_information_.framerate > 0) {
    struct v4l2_streamparm stream_params;
    memset(&stream_params, 0, sizeof(stream_params));
    stream_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(cam_fd_, VIDIOC_G_PARM, &stream_params) < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_usb_cam"),
        "can't set stream params %d", errno);
      return false;
    }
    if (!(stream_params.parm.capture.capability & V4L2_CAP_TIMEPERFRAME)) {
      cam_information_.framerate =
        stream_params.parm.capture.timeperframe.denominator /
        stream_params.parm.capture.timeperframe.numerator;
      RCLCPP_ERROR(rclcpp::get_logger("hobot_usb_cam"),
        "V4L2_CAP_TIMEPERFRAME not supported."
        " Use camera default framerate:%d\n", cam_information_.framerate);
    } else {
      stream_params.parm.capture.timeperframe.numerator = 1;
      stream_params.parm.capture.timeperframe.denominator =
        cam_information_.framerate;
      if (xioctl(cam_fd_, VIDIOC_S_PARM, &stream_params) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("hobot_usb_cam"),
          "Set camera framerate failed\n");
      } else {
        if (stream_params.parm.capture.timeperframe.denominator ==
              (uint32_t)cam_information_.framerate) {
          RCLCPP_INFO(rclcpp::get_logger("hobot_usb_cam"),
            "Set framerate to be %d\n",
            cam_information_.framerate);
        } else {
          cam_information_.framerate =
            stream_params.parm.capture.timeperframe.denominator /
            stream_params.parm.capture.timeperframe.numerator;
          RCLCPP_WARN(rclcpp::get_logger("hobot_usb_cam"),
            "Camera not supported set frame. "
            " Use camera default framerate:%d\n", cam_information_.framerate);
        }
      }
    }
  }

  switch (cam_information_.io) {
    case kIO_METHOD_READ:
      InitRead(camera_format.fmt.pix.sizeimage);
      break;

    case kIO_METHOD_MMAP:
      InitMmap();
      break;

    case kIO_METHOD_USERPTR:
      InitUserspace(camera_format.fmt.pix.sizeimage);
      break;
  }
  return true;
}

int32_t HobotUSBCam::xioctl(int fh, uint32_t request, void *arg) {
  int32_t r;

  do {
    r = ioctl(fh, request, arg);
  } while (-1 == r && EINTR == errno);

  return r;
}

void HobotUSBCam::errno_exit(const char *s) {
  RCLCPP_ERROR(rclcpp::get_logger("hobot_usb_cam"), "%s error %d, %s\\n", s,
               errno, strerror(errno));
}

bool HobotUSBCam::InitRead(unsigned int buffer_size) {
  buffers_[0].length = buffer_size;
  buffers_[0].start = malloc(buffer_size);

  if (!buffers_[0].start) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_usb_cam"), "Out of memory\\n");
    return false;
  }
  return true;
}

bool HobotUSBCam::InitMmap(void) {
  struct v4l2_requestbuffers requset_buffer;
  unsigned int n_buffers;

  CLEAR(requset_buffer);

  requset_buffer.count = buffer_numbers_;
  requset_buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  requset_buffer.memory = V4L2_MEMORY_MMAP;

  if (xioctl(cam_fd_, VIDIOC_REQBUFS, &requset_buffer) == -1) {
    if (EINVAL == errno) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_usb_cam"),
                   "%s does not support memory mappingn\\n",
                   cam_information_.dev.c_str());
      return false;
    } else {
      errno_exit("VIDIOC_REQBUFS");
    }
  }

  for (n_buffers = 0; n_buffers < requset_buffer.count; ++n_buffers) {
    struct v4l2_buffer buffer;

    CLEAR(buffer);

    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = n_buffers;

    if (xioctl(cam_fd_, VIDIOC_QUERYBUF, &buffer) == -1)
      errno_exit("VIDIOC_QUERYBUF");
    buffers_[n_buffers].length = buffer.length;
    buffers_[n_buffers].start =
        mmap(NULL /* start anywhere */, buffer.length,
             PROT_READ | PROT_WRITE /* required */,
             MAP_SHARED /* recommended */, cam_fd_, buffer.m.offset);

    if (MAP_FAILED == buffers_[n_buffers].start) errno_exit("mmap");
  }
  return true;
}

bool HobotUSBCam::InitUserspace(unsigned int buffer_size) {
  struct v4l2_requestbuffers request_buffer;
  unsigned int n_buffers;

  CLEAR(request_buffer);

  request_buffer.count = buffer_numbers_;
  request_buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  request_buffer.memory = V4L2_MEMORY_USERPTR;

  if (xioctl(cam_fd_, VIDIOC_REQBUFS, &request_buffer) == -1) {
    if (EINVAL == errno) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_usb_cam"),
                   "%s does not support user pointer i/on\\n",
                   cam_information_.dev.c_str());
      return false;
    } else {
      errno_exit("VIDIOC_REQBUFS");
    }
  }

  for (n_buffers = 0; n_buffers < request_buffer.count; ++n_buffers) {
    buffers_[n_buffers].length = buffer_size;
    buffers_[n_buffers].start = malloc(buffer_size);
    if (!buffers_[n_buffers].start) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_usb_cam"), "Out of memory\\n");
    }
  }
  return true;
}

bool HobotUSBCam::CloseDevice(void) {
  if (close(cam_fd_) == -1) {
    errno_exit("close");
    return false;
  }
  cam_fd_ = -1;
  return true;
}

bool HobotUSBCam::Start(void) {
  int i;
  enum v4l2_buf_type type;

  std::lock_guard<std::mutex> lock(cam_mutex_);
  if (cam_state_ != kSTATE_INITIALLED) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_usb_cam"),
                 "Camera state is not kSTATE_INITIALLED, current state:%d\n",
                 cam_state_);
    return false;
  }
  switch (cam_information_.io) {
    case kIO_METHOD_READ:
      /* Nothing to do. */
      break;
    case kIO_METHOD_MMAP:
      for (i = 0; i < buffer_numbers_; ++i) {
        struct v4l2_buffer buffer;

        CLEAR(buffer);
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = i;

        if (xioctl(cam_fd_, VIDIOC_QBUF, &buffer) == -1)
          errno_exit("VIDIOC_QBUF");
      }
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (xioctl(cam_fd_, VIDIOC_STREAMON, &type) == -1)
        errno_exit("VIDIOC_STREAMON");
      break;
    case kIO_METHOD_USERPTR:
      for (i = 0; i < buffer_numbers_; ++i) {
        struct v4l2_buffer buffer;

        CLEAR(buffer);
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_USERPTR;
        buffer.index = i;
        buffer.m.userptr = (unsigned long)buffers_[i].start;
        buffer.length = buffers_[i].length;

        if (xioctl(cam_fd_, VIDIOC_QBUF, &buffer) == -1)
          errno_exit("VIDIOC_QBUF");
      }
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (xioctl(cam_fd_, VIDIOC_STREAMON, &type) == -1)
        errno_exit("VIDIOC_STREAMON");
      break;
  }
  cam_state_ = kSTATE_RUNING;
  return true;
}

bool HobotUSBCam::Stop(void) {
  enum v4l2_buf_type type;

  std::lock_guard<std::mutex> lock(cam_mutex_);
  if (cam_state_ != kSTATE_RUNING) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_usb_cam"),
                 "Cmera state is not kSTATE_RUNING, current state:%d\n",
                 cam_state_);
    return false;
  }
  switch (cam_information_.io) {
    case kIO_METHOD_READ:
      /* Nothing to do. */
      break;

    case kIO_METHOD_MMAP:
    case kIO_METHOD_USERPTR:
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (-1 == xioctl(cam_fd_, VIDIOC_STREAMOFF, &type)) {
        errno_exit("VIDIOC_STREAMOFF");
        return false;
      }
      break;
  }
  cam_state_ = kSTATE_STOP;
  return true;
}

bool HobotUSBCam::DeInit() {
  bool ret = true;
  int i;
  std::lock_guard<std::mutex> lock(cam_mutex_);
  if (cam_state_ != kSTATE_STOP) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_usb_cam"),
                 "Cmera state is not kSTATE_RUNING, current state:%d\n",
                 cam_state_);
    return false;
  }
  switch (cam_information_.io) {
  case kIO_METHOD_READ:
    free(buffers_[0].start);
    break;
  case kIO_METHOD_MMAP:
    for (i = 0; i < buffer_numbers_; ++i)
      if (-1 == munmap(buffers_[i].start, buffers_[i].length)) {
        errno_exit("munmap");
        ret = false;
      }
    break;
  case kIO_METHOD_USERPTR:
    for (i = 0; i < buffer_numbers_; ++i)
      free(buffers_[i].start);
    break;
  }
  ret = CloseDevice();
  cam_state_ = kSTATE_UNINITIALLED;
  return ret;
}

bool HobotUSBCam::GetFrame(CamBuffer &cam_buffer) {
  std::lock_guard<std::mutex> lock(cam_mutex_);
  if (cam_state_ != kSTATE_RUNING) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_usb_cam"),
                 "Cmera state is not kSTATE_RUNING, current state:%d\n",
                 cam_state_);
    return false;
  }
  for (;;) {
    fd_set fds;
    struct timeval tv;
    int r;

    FD_ZERO(&fds);
    FD_SET(cam_fd_, &fds);

    /* Timeout. */
    tv.tv_sec = 2;
    tv.tv_usec = 0;

    r = select(cam_fd_ + 1, &fds, NULL, NULL, &tv);

    if (-1 == r) {
      if (EINTR == errno) continue;
      errno_exit("select");
    }

    if (0 == r) {
      RCLCPP_ERROR(rclcpp::get_logger("hobot_usb_cam"), "select timeout\\n");
      return false;
    }

    if (ReadFrame(cam_buffer)) break;
    /* EAGAIN - continue select loop. */
  }
  return true;
}

bool HobotUSBCam::ReadFrame(CamBuffer &cam_buffer) {
  struct v4l2_buffer buffer;
  int i;

  switch (cam_information_.io) {
    case kIO_METHOD_READ:
      if (read(cam_fd_, buffers_[0].start, buffers_[0].length) == -1) {
        switch (errno) {
          case EAGAIN:
            return false;
          case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */
          default:
            errno_exit("read");
        }
      }
      cam_buffer.length = buffers_[0].length;
      cam_buffer.start = buffers_[0].start;
      cam_buffer.time_point = std::chrono::system_clock::now();
      break;

    case kIO_METHOD_MMAP:
      CLEAR(buffer);

      buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buffer.memory = V4L2_MEMORY_MMAP;

      if (xioctl(cam_fd_, VIDIOC_DQBUF, &buffer) == -1) {
        switch (errno) {
          case EAGAIN:
            return false;
          case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */
          default:
            errno_exit("VIDIOC_DQBUF");
            return false;
        }
      }
      cam_buffer.length = buffer.bytesused;
      cam_buffer.start = buffers_[buffer.index].start;
      cam_buffer.time_point = std::chrono::system_clock::time_point {
          std::chrono::seconds { buffer.timestamp.tv_sec } +
          std::chrono::microseconds { buffer.timestamp.tv_usec }
      };
      cam_buffer.reserved_buffer = buffer;
      break;

    case kIO_METHOD_USERPTR:
      CLEAR(buffer);

      buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buffer.memory = V4L2_MEMORY_USERPTR;

      if (xioctl(cam_fd_, VIDIOC_DQBUF, &buffer) == -1) {
        switch (errno) {
          case EAGAIN:
            return false;
          case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */
          default:
            errno_exit("VIDIOC_DQBUF");
            return false;
        }
      }

      for (i = 0; i < buffer_numbers_; ++i)
        if (buffer.m.userptr == (unsigned long)buffers_[i].start &&
            buffer.length == buffers_[i].length)
          break;

      cam_buffer.length = buffers_[i].length;
      cam_buffer.start = buffers_[i].start;
      cam_buffer.time_point = std::chrono::system_clock::time_point {
        std::chrono::seconds { buffer.timestamp.tv_sec } +
        std::chrono::microseconds { buffer.timestamp.tv_usec }
      };
      cam_buffer.reserved_buffer = buffer;
      break;
  }
  return true;
}

bool HobotUSBCam::ReleaseFrame(CamBuffer &cam_buffer) {
  bool ret = true;
  std::lock_guard<std::mutex> lock(cam_mutex_);
  if (cam_state_ != kSTATE_RUNING) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_usb_cam"),
                 "Cmera state is not kSTATE_RUNING, current state:%d\n",
                 cam_state_);
    return false;
  }
  switch (cam_information_.io) {
    case kIO_METHOD_READ:
      break;
    case kIO_METHOD_MMAP:
      if (xioctl(cam_fd_, VIDIOC_QBUF, &cam_buffer.reserved_buffer) == -1)
        errno_exit("kIO_METHOD_MMAP VIDIOC_QBUF");
      ret = false;
      break;
    case kIO_METHOD_USERPTR:
      if (xioctl(cam_fd_, VIDIOC_QBUF, &cam_buffer.reserved_buffer) == -1)
        errno_exit("kIO_METHOD_USERPTR VIDIOC_QBUF");
      ret = false;
      break;
  }
  return ret;
}
}  // namespace hobot_usb_cam