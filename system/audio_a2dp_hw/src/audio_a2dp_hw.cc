/******************************************************************************
 *
 *  Copyright 2009-2012 Broadcom Corporation
 *  Copyright (C) 2026 Mercedes-Benz Group AG
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

/*****************************************************************************
 *
 *  Filename:      audio_a2dp_hw.c
 *
 *  Description:   Implements hal for bluedroid a2dp audio device
 *
 *****************************************************************************/
#define LOG_TAG "bt_a2dp_hw"

#include <audio_utils/format.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <log/log.h>
#include <stdint.h>
#include <sys/errno.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/un.h>
#include <unistd.h>

#include <mutex>

#include <hardware/audio.h>
#include <hardware/hardware.h>
#include <system/audio.h>

#ifdef INSTALL_A2DP_HAL
#include <unordered_map>
#include <cutils/properties.h>
#else
#include "osi/include/hash_map_utils.h"
#include "osi/include/osi.h"
#include "osi/include/socket_utils/sockets.h"
#endif

#include <cutils/properties.h>
#include "audio_a2dp_hw.h"

/*****************************************************************************
 *  Constants & Macros
 *****************************************************************************/

#define CTRL_CHAN_RETRY_COUNT 3
#define USEC_PER_SEC 1000000L
#define SOCK_SEND_TIMEOUT_MS 2000 /* Timeout for sending */
#define SOCK_RECV_TIMEOUT_MS 5000 /* Timeout for receiving */
#define SEC_TO_MS 1000
#define SEC_TO_NS 1000000000
#define MS_TO_NS 1000000
#define DELAY_TO_NS 100000

#define MIN_DELAY_MS 100
#define MAX_DELAY_MS 1000

#define DEFAULT_BUF_SIZE 6144

// set WRITE_POLL_MS to 0 for blocking sockets, nonzero for polled non-blocking
// sockets
#define WRITE_POLL_MS 20
#define __FILE_A2DP__ "packages/modules/Bluetooth/system/audio_a2dp_hw/src/audio_a2dp_hw.cc"
#define FNLOG() ALOGV("%s:%d %s: ", __FILE_A2DP__, __LINE__, __func__)
#define DEBUG(fmt, args...) \
  ALOGD("%s:%d %s: " fmt, __FILE_A2DP__, __LINE__, __func__, ##args)
#define INFO(fmt, args...) \
  ALOGI("%s:%d %s: " fmt, __FILE_A2DP__, __LINE__, __func__, ##args)
#define WARN(fmt, args...) \
  ALOGW("%s:%d %s: " fmt, __FILE_A2DP__, __LINE__, __func__, ##args)
#define ERROR(fmt, args...) \
  ALOGE("%s:%d %s: " fmt, __FILE_A2DP__, __LINE__, __func__, ##args)
#define VERBOSE(fmt, args...) \
  ALOGV("%s:%d %s: " fmt, __FILE_A2DP__, __LINE__, __func__, ##args)

#define ASSERTC(cond, msg, val)                                           \
  if (!(cond)) {                                                          \
    ERROR("### ASSERT : %s line %d %s (%d) ###", __FILE_A2DP__, __LINE__, msg, \
          val);                                                           \
  }

#define IS_AUDIO_DEVICE_OUT_BUS(dev) ((dev) == AUDIO_DEVICE_OUT_BUS)

/* To remove libosi dependency */
/*
 * See also android.os.LocalSocketAddress.Namespace
 */
// Linux "abstract" (non-filesystem) namespace
#define ANDROID_SOCKET_NAMESPACE_ABSTRACT 0
// Android "reserved" (/dev/socket) namespace
#define ANDROID_SOCKET_NAMESPACE_RESERVED 1
// Normal filesystem namespace
#define ANDROID_SOCKET_NAMESPACE_FILESYSTEM 2
#define ANDROID_RESERVED_SOCKET_PREFIX "/dev/socket/"

#define OSI_NO_INTR(fn) \
  do {                  \
  } while ((fn) == -1 && errno == EINTR)

#define UNUSED_ATTR __attribute__((unused))

/*****************************************************************************
 *  Local type definitions
 *****************************************************************************/

typedef enum {
  AUDIO_A2DP_STATE_STARTING,
  AUDIO_A2DP_STATE_STARTED,
  AUDIO_A2DP_STATE_STOPPING,
  AUDIO_A2DP_STATE_STOPPED,
  /* need explicit set param call to resume (suspend=false) */
  AUDIO_A2DP_STATE_SUSPENDED,
  AUDIO_A2DP_STATE_STANDBY /* allows write to autoresume */
} a2dp_state_t;

struct a2dp_stream_in;
struct a2dp_stream_out;

struct a2dp_audio_device {
  // Important: device must be first as an audio_hw_device* may be cast to
  // a2dp_audio_device* when the type is implicitly known.
  struct audio_hw_device device;
  std::recursive_mutex* mutex;  // See note below on mutex acquisition order.
  struct a2dp_stream_in* input;
  struct a2dp_stream_out* output;
  std::string bdaddress;
};

struct a2dp_config {
  uint32_t rate;
  audio_channel_mask_t channel_mask;
  bool is_stereo_to_mono;  // True if fetching Stereo and mixing into Mono
  int format;
};

/* move ctrl_fd outside output stream and keep open until HAL unloaded ? */

struct a2dp_stream_common {
  std::recursive_mutex* mutex;  // See note below on mutex acquisition order.
  int ctrl_fd;
  int audio_fd;
  size_t buffer_sz;
  struct audio_config in_config;
  struct a2dp_config cfg;
  a2dp_state_t state;
  std::string bdaddress;
  audio_devices_t device_type;
  std::string bus_address;
  int volume;
  void * data;
  size_t data_sz;
  std::string displayId;
};

#define COMMON_ARRAY_SIZE 4
#define BUS_ADDRESS_BUS1 "BUS30_BT_HEADPHONE_1"
#define BUS_ADDRESS_BUS2 "BUS31_BT_HEADPHONE_2"
struct a2dp_stream_out {
  struct audio_stream_out stream;
  struct a2dp_stream_common commons[COMMON_ARRAY_SIZE];
  uint64_t frames_presented;  // frames written, never reset
  uint64_t frames_rendered;   // frames written, reset on standby
  struct timeval lastTime;
  uint64_t bytes_write;
};

struct a2dp_stream_in {
  struct audio_stream_in stream;
  struct a2dp_stream_common common;
};

/*
 * Mutex acquisition order:
 *
 * The a2dp_audio_device (adev) mutex must be acquired before
 * the a2dp_stream_common (out or in) mutex.
 *
 * This may differ from other audio HALs.
 */

/*****************************************************************************
 *  Static variables
 *****************************************************************************/

static bool enable_delay_reporting = false;

static std::unordered_map<std::string, struct a2dp_stream_out*> device_stream_map;
/* Get stream from bus address */
static std::unordered_map<std::string, struct a2dp_stream_out*> bus_stream_map;
/* Get group from address for same audio source */
static std::unordered_map<std::string, std::string> device_group_map;

/*****************************************************************************
 *  Static functions
 *****************************************************************************/

static size_t out_get_buffer_size(const struct audio_stream* stream);
static uint32_t out_get_latency(const struct audio_stream_out* stream);

/*****************************************************************************
 *  Externs
 *****************************************************************************/

/*****************************************************************************
 *  Functions
 *****************************************************************************/
static void a2dp_open_ctrl_path(struct a2dp_stream_common* common);

/*****************************************************************************
 *   Miscellaneous helper functions
 *****************************************************************************/
#ifdef INSTALL_A2DP_HAL
static std::unordered_map<std::string, std::string>
hash_map_utils_new_from_string_params(const char* params) {
  // CHECK(params != NULL);
  std::unordered_map<std::string, std::string> map;
  if(params == NULL) {
    return map;  
  }

  char* str = strdup(params);
  if (!str) return map;

  // Parse |str| and add extracted key-and-value pair(s) in |map|.
  int items = 0;
  char* tmpstr;
  char* kvpair = strtok_r(str, ";", &tmpstr);
  while (kvpair && *kvpair) {
    char* eq = strchr(kvpair, '=');

    if (eq == kvpair) goto next_pair;

    char* key;
    char* value;
    if (eq) {
      key = strndup(kvpair, eq - kvpair);

      // The increment of |eq| moves |eq| to the beginning of the value.
      ++eq;
      value = (*eq != '\0') ? strdup(eq) : strdup("");
    } else {
      key = strdup(kvpair);
      value = strdup("");
    }

    map[key] = value;

    free(key);
    free(value);

    items++;
  next_pair:
    kvpair = strtok_r(NULL, ";", &tmpstr);
  }

  free(str);
  return map;
}

/* Documented in header file. */
static int osi_socket_make_sockaddr_un(const char* name, int namespaceId,
                                struct sockaddr_un* p_addr, socklen_t* alen) {
  memset(p_addr, 0, sizeof(*p_addr));
  size_t namelen;

  switch (namespaceId) {
    case ANDROID_SOCKET_NAMESPACE_ABSTRACT:
#if defined(__linux__)
      namelen = strlen(name);

      // Test with length +1 for the *initial* '\0'.
      if ((namelen + 1) > sizeof(p_addr->sun_path)) {
        goto error;
      }

      /*
       * Note: The path in this case is *not* supposed to be
       * '\0'-terminated. ("man 7 unix" for the gory details.)
       */

      p_addr->sun_path[0] = 0;
      memcpy(p_addr->sun_path + 1, name, namelen);
#else
      /* this OS doesn't have the Linux abstract namespace */

      namelen = strlen(name) + strlen(FILESYSTEM_SOCKET_PREFIX);
      /* unix_path_max appears to be missing on linux */
      if (namelen >
          sizeof(*p_addr) - offsetof(struct sockaddr_un, sun_path) - 1) {
        goto error;
      }

      strlcpy(p_addr->sun_path, FILESYSTEM_SOCKET_PREFIX, sizeof(p_addr->sun_path));
      strlcat(p_addr->sun_path, name, sizeof(p_addr->sun_path));
#endif
      break;

    case ANDROID_SOCKET_NAMESPACE_RESERVED:
      namelen = strlen(name) + strlen(ANDROID_RESERVED_SOCKET_PREFIX);
      /* unix_path_max appears to be missing on linux */
      if (namelen >
          sizeof(*p_addr) - offsetof(struct sockaddr_un, sun_path) - 1) {
        goto error;
      }

      strlcpy(p_addr->sun_path, ANDROID_RESERVED_SOCKET_PREFIX, sizeof(p_addr->sun_path));
      strlcat(p_addr->sun_path, name, sizeof(p_addr->sun_path));
      break;

    case ANDROID_SOCKET_NAMESPACE_FILESYSTEM:
      namelen = strlen(name);
      /* unix_path_max appears to be missing on linux */
      if (namelen >
          sizeof(*p_addr) - offsetof(struct sockaddr_un, sun_path) - 1) {
        goto error;
      }

      strlcpy(p_addr->sun_path, name, sizeof(p_addr->sun_path));
      break;
    default:
      // invalid namespace id
      return -1;
  }

  p_addr->sun_family = AF_LOCAL;
  *alen = namelen + offsetof(struct sockaddr_un, sun_path) + 1;
  return 0;
error:
  return -1;
}


/**
 * connect to peer named "name" on fd
 * returns same fd or -1 on error.
 * fd is not closed on error. that's your job.
 *
 * Used by AndroidSocketImpl
 */
static int osi_socket_local_client_connect(int fd, const char* name, int namespaceId,
                                    int type UNUSED_ATTR) {
  struct sockaddr_un addr;
  socklen_t alen;
  int err;

  err = osi_socket_make_sockaddr_un(name, namespaceId, &addr, &alen);

  if (err < 0) {
    goto error;
  }

  OSI_NO_INTR(err = connect(fd, (struct sockaddr*)&addr, alen));
  if (err < 0) {
    goto error;
  }

  return fd;

error:
  return -1;
}

bool osi_property_get_bool(const char* key, bool default_value) {
#if defined(OS_GENERIC)
  return default_value;
#else
  return property_get_bool(key, default_value);
#endif  // defined(OS_GENERIC)
}
#endif

static UNUSED_ATTR void hash_map_utils_dump_string_keys_string_values(
    std::unordered_map<std::string, std::string>& map) {
  for (const auto& ptr : map) {
    INFO("key: '%s' value: '%s'\n", ptr.first.c_str(), ptr.second.c_str());
  }
}

/* logs timestamp with microsec precision
   pprev is optional in case a dedicated diff is required */
static void ts_log(UNUSED_ATTR const char* tag, UNUSED_ATTR int val,
                   struct timespec* pprev_opt) {
  struct timespec now;
  static struct timespec prev = {0, 0};
  unsigned long long now_us;
  unsigned long long diff_us;

  clock_gettime(CLOCK_MONOTONIC, &now);

  now_us = now.tv_sec * USEC_PER_SEC + now.tv_nsec / 1000;

  if (pprev_opt) {
    diff_us = (now.tv_sec - prev.tv_sec) * USEC_PER_SEC +
              (now.tv_nsec - prev.tv_nsec) / 1000;
    *pprev_opt = now;
    VERBOSE("[%s] ts %08lld, *diff %08lld, val %d", tag, now_us, diff_us, val);
  } else {
    diff_us = (now.tv_sec - prev.tv_sec) * USEC_PER_SEC +
              (now.tv_nsec - prev.tv_nsec) / 1000;
    prev = now;
    VERBOSE("[%s] ts %08lld, diff %08lld, val %d", tag, now_us, diff_us, val);
  }
}

static int calc_audiotime_usec(struct a2dp_config cfg, int bytes) {
  int chan_count = audio_channel_count_from_out_mask(cfg.channel_mask);
  int bytes_per_sample;

  switch (cfg.format) {
    case AUDIO_FORMAT_PCM_8_BIT:
      bytes_per_sample = 1;
      break;
    case AUDIO_FORMAT_PCM_16_BIT:
      bytes_per_sample = 2;
      break;
    case AUDIO_FORMAT_PCM_24_BIT_PACKED:
      bytes_per_sample = 3;
      break;
    case AUDIO_FORMAT_PCM_8_24_BIT:
      bytes_per_sample = 4;
      break;
    case AUDIO_FORMAT_PCM_32_BIT:
      bytes_per_sample = 4;
      break;
    default:
      ASSERTC(false, "unsupported sample format", cfg.format);
      bytes_per_sample = 2;
      break;
  }

  return (
      int)(((int64_t)bytes * (USEC_PER_SEC / (chan_count * bytes_per_sample))) /
           cfg.rate);
}

/*****************************************************************************
 *
 *   bluedroid stack adaptation
 *
 ****************************************************************************/

static int skt_connect(const char* path, size_t buffer_sz) {
  int ret;
  int skt_fd;
  int len;

  INFO("connect to %s (sz %zu)", path, buffer_sz);

  skt_fd = socket(AF_LOCAL, SOCK_STREAM, 0);

  if (osi_socket_local_client_connect(
          skt_fd, path, ANDROID_SOCKET_NAMESPACE_ABSTRACT, SOCK_STREAM) < 0) {
    ERROR("failed to connect (%s)", strerror(errno));
    close(skt_fd);
    return -1;
  }

  len = buffer_sz;
  ret =
      setsockopt(skt_fd, SOL_SOCKET, SO_SNDBUF, (char*)&len, (int)sizeof(len));
  if (ret < 0) ERROR("setsockopt failed (%s)", strerror(errno));

  ret =
      setsockopt(skt_fd, SOL_SOCKET, SO_RCVBUF, (char*)&len, (int)sizeof(len));
  if (ret < 0) ERROR("setsockopt failed (%s)", strerror(errno));

  /* Socket send/receive timeout value */
  struct timeval tv;
  tv.tv_sec = SOCK_SEND_TIMEOUT_MS / 1000;
  tv.tv_usec = (SOCK_SEND_TIMEOUT_MS % 1000) * 1000;

  ret = setsockopt(skt_fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  if (ret < 0) ERROR("setsockopt failed (%s)", strerror(errno));

  tv.tv_sec = SOCK_RECV_TIMEOUT_MS / 1000;
  tv.tv_usec = (SOCK_RECV_TIMEOUT_MS % 1000) * 1000;

  ret = setsockopt(skt_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  if (ret < 0) ERROR("setsockopt failed (%s)", strerror(errno));

  INFO("connected to stack fd = %d", skt_fd);

  return skt_fd;
}

static int skt_read(int fd, void* p, size_t len) {
  ssize_t read;

  FNLOG();

  ts_log("skt_read recv", len, NULL);

  OSI_NO_INTR(read = recv(fd, p, len, MSG_NOSIGNAL));
  if (read == -1) ERROR("read failed with errno=%d\n", errno);

  return (int)read;
}

static int skt_write(int fd, const void* p, size_t len) {
  ssize_t sent;
  FNLOG();

  ts_log("skt_write", len, NULL);

  if (WRITE_POLL_MS == 0) {
    // do not poll, use blocking send
    OSI_NO_INTR(sent = send(fd, p, len, MSG_NOSIGNAL));
    if (sent == -1) ERROR("write failed with error(%s)", strerror(errno));

    return (int)sent;
  }

  // use non-blocking send, poll
  int ms_timeout = SOCK_SEND_TIMEOUT_MS;
  size_t count = 0;
  while (count < len) {
    OSI_NO_INTR(sent = send(fd, p, len - count, MSG_NOSIGNAL | MSG_DONTWAIT));
    if (sent == -1) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        ERROR("write failed with error(%s)", strerror(errno));
        return -1;
      }
      if (ms_timeout >= WRITE_POLL_MS) {
        usleep(WRITE_POLL_MS * 1000);
        ms_timeout -= WRITE_POLL_MS;
        continue;
      }
      WARN("write timeout exceeded, sent %zu bytes", count);
      return -1;
    }
    count += sent;
    p = (const uint8_t*)p + sent;
  }
  return (int)count;
}

static int skt_disconnect(int fd) {
  INFO("fd %d", fd);

  if (fd != AUDIO_SKT_DISCONNECTED) {
    shutdown(fd, SHUT_RDWR);
    close(fd);
  }
  return 0;
}

/*****************************************************************************
 *
 *  AUDIO CONTROL PATH
 *
 ****************************************************************************/

static int a2dp_ctrl_receive(struct a2dp_stream_common* common, void* buffer,
                             size_t length) {
  ssize_t ret;
  int i;

  for (i = 0;; i++) {
    OSI_NO_INTR(ret = recv(common->ctrl_fd, buffer, length, MSG_NOSIGNAL));
    if (ret > 0) {
      break;
    }
    if (ret == 0) {
      ERROR("receive control data failed: peer closed");
      break;
    }
    if (errno != EWOULDBLOCK && errno != EAGAIN) {
      ERROR("receive control data failed: error(%s)", strerror(errno));
      break;
    }
    if (i == (CTRL_CHAN_RETRY_COUNT - 1)) {
      ERROR("receive control data failed: max retry count");
      break;
    }
    INFO("receive control data failed (%s), retrying", strerror(errno));
  }
  if (ret <= 0) {
    skt_disconnect(common->ctrl_fd);
    common->ctrl_fd = AUDIO_SKT_DISCONNECTED;
  }
  return ret;
}

// Sends control info for stream |common|. The data to send is stored in
// |buffer| and has size |length|.
// On success, returns the number of octets sent, otherwise -1.
static int a2dp_ctrl_send(struct a2dp_stream_common* common, const void* buffer,
                          size_t length) {
  ssize_t sent;
  size_t remaining = length;
  int i;

  if (length == 0) return 0;  // Nothing to do

  for (i = 0;; i++) {
    OSI_NO_INTR(sent = send(common->ctrl_fd, buffer, remaining, MSG_NOSIGNAL));
    if (sent == static_cast<ssize_t>(remaining)) {
      remaining = 0;
      break;
    }
    if (sent > 0) {
      buffer = (static_cast<const char*>(buffer) + sent);
      remaining -= sent;
      continue;
    }
    if (sent < 0) {
      if (errno != EWOULDBLOCK && errno != EAGAIN) {
        ERROR("send control data failed: error(%s)", strerror(errno));
        break;
      }
      INFO("send control data failed (%s), retrying", strerror(errno));
    }
    if (i >= (CTRL_CHAN_RETRY_COUNT - 1)) {
      ERROR("send control data failed: max retry count");
      break;
    }
  }
  if (remaining > 0) {
    skt_disconnect(common->ctrl_fd);
    common->ctrl_fd = AUDIO_SKT_DISCONNECTED;
    return -1;
  }
  return length;
}

static int a2dp_command(struct a2dp_stream_common* common, tA2DP_CTRL_CMD cmd) {
  char ack;

  VERBOSE("A2DP COMMAND %s", audio_a2dp_hw_dump_ctrl_event(cmd));

  if (common->ctrl_fd == AUDIO_SKT_DISCONNECTED) {
    INFO("starting up or recovering from previous error: command=%s",
         audio_a2dp_hw_dump_ctrl_event(cmd));
    a2dp_open_ctrl_path(common);
    if (common->ctrl_fd == AUDIO_SKT_DISCONNECTED) {
      ERROR("failure to open ctrl path: command=%s",
            audio_a2dp_hw_dump_ctrl_event(cmd));
      return -1;
    }
  }

  /* send command */
  ssize_t sent;
  OSI_NO_INTR(sent = send(common->ctrl_fd, &cmd, 1, MSG_NOSIGNAL));
  if (sent == -1) {
    ERROR("cmd failed (%s): command=%s", strerror(errno),
          audio_a2dp_hw_dump_ctrl_event(cmd));
    skt_disconnect(common->ctrl_fd);
    common->ctrl_fd = AUDIO_SKT_DISCONNECTED;
    return -1;
  }

  /* wait for ack byte */
  if (a2dp_ctrl_receive(common, &ack, 1) < 0) {
    ERROR("A2DP COMMAND %s: no ACK", audio_a2dp_hw_dump_ctrl_event(cmd));
    return -1;
  }

  VERBOSE("A2DP COMMAND %s DONE STATUS %d", audio_a2dp_hw_dump_ctrl_event(cmd),
        ack);

  if (ack == A2DP_CTRL_ACK_INCALL_FAILURE) {
    ERROR("A2DP COMMAND %s error %d", audio_a2dp_hw_dump_ctrl_event(cmd), ack);
    return ack;
  }
  if (ack != A2DP_CTRL_ACK_SUCCESS) {
    ERROR("A2DP COMMAND %s error %d", audio_a2dp_hw_dump_ctrl_event(cmd), ack);
    return -1;
  }

  return 0;
}

static int check_a2dp_ready(struct a2dp_stream_common* common) {
  if (a2dp_command(common, A2DP_CTRL_CMD_CHECK_READY) < 0) {
    ERROR("check a2dp ready failed");
    return -1;
  }
  return 0;
}

static int a2dp_read_input_audio_config(struct a2dp_stream_common* common) {
  tA2DP_SAMPLE_RATE sample_rate;
  tA2DP_CHANNEL_COUNT channel_count;

  if (a2dp_command(common, A2DP_CTRL_GET_INPUT_AUDIO_CONFIG) < 0) {
    ERROR("get a2dp input audio config failed");
    return -1;
  }

  if (a2dp_ctrl_receive(common, &sample_rate, sizeof(tA2DP_SAMPLE_RATE)) < 0)
    return -1;
  if (a2dp_ctrl_receive(common, &channel_count, sizeof(tA2DP_CHANNEL_COUNT)) <
      0) {
    return -1;
  }

  switch (sample_rate) {
    case 44100:
    case 48000:
      common->cfg.rate = sample_rate;
      break;
    default:
      ERROR("Invalid sample rate: %" PRIu32, sample_rate);
      return -1;
  }

  switch (channel_count) {
    case 1:
      common->cfg.channel_mask = AUDIO_CHANNEL_IN_MONO;
      break;
    case 2:
      common->cfg.channel_mask = AUDIO_CHANNEL_IN_STEREO;
      break;
    default:
      ERROR("Invalid channel count: %" PRIu32, channel_count);
      return -1;
  }

  // TODO: For now input audio format is always hard-coded as PCM 16-bit
  common->cfg.format = AUDIO_FORMAT_PCM_16_BIT;

  INFO("got input audio config %d %d", common->cfg.format, common->cfg.rate);

  return 0;
}

static int a2dp_read_output_audio_config(
    struct a2dp_stream_common* common, btav_a2dp_codec_config_t* codec_config,
    btav_a2dp_codec_config_t* codec_capability, bool update_stream_config) {
  struct a2dp_config stream_config;

  if (a2dp_command(common, A2DP_CTRL_GET_OUTPUT_AUDIO_CONFIG) < 0) {
    ERROR("get a2dp output audio config failed");
    return -1;
  }

  // Receive the current codec config
  if (a2dp_ctrl_receive(common, &codec_config->sample_rate,
                        sizeof(btav_a2dp_codec_sample_rate_t)) < 0) {
    return -1;
  }
  if (a2dp_ctrl_receive(common, &codec_config->bits_per_sample,
                        sizeof(btav_a2dp_codec_bits_per_sample_t)) < 0) {
    return -1;
  }
  if (a2dp_ctrl_receive(common, &codec_config->channel_mode,
                        sizeof(btav_a2dp_codec_channel_mode_t)) < 0) {
    return -1;
  }

  // Receive the current codec capability
  if (a2dp_ctrl_receive(common, &codec_capability->sample_rate,
                        sizeof(btav_a2dp_codec_sample_rate_t)) < 0) {
    return -1;
  }
  if (a2dp_ctrl_receive(common, &codec_capability->bits_per_sample,
                        sizeof(btav_a2dp_codec_bits_per_sample_t)) < 0) {
    return -1;
  }
  if (a2dp_ctrl_receive(common, &codec_capability->channel_mode,
                        sizeof(btav_a2dp_codec_channel_mode_t)) < 0) {
    return -1;
  }

  // Check the codec config sample rate
  switch (codec_config->sample_rate) {
    case BTAV_A2DP_CODEC_SAMPLE_RATE_44100:
      stream_config.rate = 44100;
      break;
    case BTAV_A2DP_CODEC_SAMPLE_RATE_48000:
      stream_config.rate = 48000;
      break;
    case BTAV_A2DP_CODEC_SAMPLE_RATE_88200:
      stream_config.rate = 88200;
      break;
    case BTAV_A2DP_CODEC_SAMPLE_RATE_96000:
      stream_config.rate = 96000;
      break;
    case BTAV_A2DP_CODEC_SAMPLE_RATE_176400:
      stream_config.rate = 176400;
      break;
    case BTAV_A2DP_CODEC_SAMPLE_RATE_192000:
      stream_config.rate = 192000;
      break;
    case BTAV_A2DP_CODEC_SAMPLE_RATE_NONE:
    default:
      ERROR("Invalid sample rate: 0x%x", codec_config->sample_rate);
      return -1;
  }

  // Check the codec config bits per sample
  switch (codec_config->bits_per_sample) {
    case BTAV_A2DP_CODEC_BITS_PER_SAMPLE_16:
      stream_config.format = AUDIO_FORMAT_PCM_16_BIT;
      break;
    case BTAV_A2DP_CODEC_BITS_PER_SAMPLE_24:
      stream_config.format = AUDIO_FORMAT_PCM_24_BIT_PACKED;
      break;
    case BTAV_A2DP_CODEC_BITS_PER_SAMPLE_32:
      stream_config.format = AUDIO_FORMAT_PCM_32_BIT;
      break;
    case BTAV_A2DP_CODEC_BITS_PER_SAMPLE_NONE:
    default:
      ERROR("Invalid bits per sample: 0x%x", codec_config->bits_per_sample);
      return -1;
  }

  // Check the codec config channel mode
  switch (codec_config->channel_mode) {
    case BTAV_A2DP_CODEC_CHANNEL_MODE_MONO:
      stream_config.channel_mask = AUDIO_CHANNEL_OUT_MONO;
      stream_config.is_stereo_to_mono = true;
      break;
    case BTAV_A2DP_CODEC_CHANNEL_MODE_STEREO:
      stream_config.channel_mask = AUDIO_CHANNEL_OUT_STEREO;
      stream_config.is_stereo_to_mono = false;
      break;
    case BTAV_A2DP_CODEC_CHANNEL_MODE_NONE:
    default:
      ERROR("Invalid channel mode: 0x%x", codec_config->channel_mode);
      return -1;
  }
  if (stream_config.is_stereo_to_mono) {
    stream_config.channel_mask = AUDIO_CHANNEL_OUT_STEREO;
  }

  // Update the output stream configuration
  if (update_stream_config) {
    common->cfg.rate = stream_config.rate;
    common->cfg.channel_mask = stream_config.channel_mask;
    common->cfg.is_stereo_to_mono = stream_config.is_stereo_to_mono;
    common->cfg.format = stream_config.format;
    common->buffer_sz = audio_a2dp_hw_stream_compute_buffer_size(
        codec_config->sample_rate, codec_config->bits_per_sample,
        codec_config->channel_mode);
    if (common->cfg.is_stereo_to_mono) {
      // We need to fetch twice as much data from the Audio framework
      common->buffer_sz *= 2;
    }
  }

  INFO(
      "got output codec config (update_stream_config=%s): "
      "sample_rate=0x%x bits_per_sample=0x%x channel_mode=0x%x",
      update_stream_config ? "true" : "false", codec_config->sample_rate,
      codec_config->bits_per_sample, codec_config->channel_mode);

  INFO(
      "got output codec capability: sample_rate=0x%x bits_per_sample=0x%x "
      "channel_mode=0x%x",
      codec_capability->sample_rate, codec_capability->bits_per_sample,
      codec_capability->channel_mode);

  return 0;
}

static int a2dp_write_output_audio_config(struct a2dp_stream_common* common) {
  btav_a2dp_codec_config_t codec_config;

  if (a2dp_command(common, A2DP_CTRL_SET_OUTPUT_AUDIO_CONFIG) < 0) {
    ERROR("set a2dp output audio config failed");
    return -1;
  }

  codec_config.sample_rate = BTAV_A2DP_CODEC_SAMPLE_RATE_NONE;
  codec_config.bits_per_sample = BTAV_A2DP_CODEC_BITS_PER_SAMPLE_NONE;
  codec_config.channel_mode = BTAV_A2DP_CODEC_CHANNEL_MODE_NONE;

  switch (common->cfg.rate) {
    case 44100:
      codec_config.sample_rate = BTAV_A2DP_CODEC_SAMPLE_RATE_44100;
      break;
    case 48000:
      codec_config.sample_rate = BTAV_A2DP_CODEC_SAMPLE_RATE_48000;
      break;
    case 88200:
      codec_config.sample_rate = BTAV_A2DP_CODEC_SAMPLE_RATE_88200;
      break;
    case 96000:
      codec_config.sample_rate = BTAV_A2DP_CODEC_SAMPLE_RATE_96000;
      break;
    case 176400:
      codec_config.sample_rate = BTAV_A2DP_CODEC_SAMPLE_RATE_176400;
      break;
    case 192000:
      codec_config.sample_rate = BTAV_A2DP_CODEC_SAMPLE_RATE_192000;
      break;
    default:
      ERROR("Invalid sample rate: %" PRIu32, common->cfg.rate);
      return -1;
  }

  switch (common->cfg.format) {
    case AUDIO_FORMAT_PCM_16_BIT:
      codec_config.bits_per_sample = BTAV_A2DP_CODEC_BITS_PER_SAMPLE_16;
      break;
    case AUDIO_FORMAT_PCM_24_BIT_PACKED:
      codec_config.bits_per_sample = BTAV_A2DP_CODEC_BITS_PER_SAMPLE_24;
      break;
    case AUDIO_FORMAT_PCM_32_BIT:
      codec_config.bits_per_sample = BTAV_A2DP_CODEC_BITS_PER_SAMPLE_32;
      break;
    case AUDIO_FORMAT_PCM_8_24_BIT:
      // All 24-bit audio is expected in AUDIO_FORMAT_PCM_24_BIT_PACKED format
      FALLTHROUGH_INTENDED; /* FALLTHROUGH */
    default:
      ERROR("Invalid audio format: 0x%x", common->cfg.format);
      return -1;
  }

  switch (common->cfg.channel_mask) {
    case AUDIO_CHANNEL_OUT_MONO:
      codec_config.channel_mode = BTAV_A2DP_CODEC_CHANNEL_MODE_MONO;
      break;
    case AUDIO_CHANNEL_OUT_STEREO:
      if (common->cfg.is_stereo_to_mono) {
        codec_config.channel_mode = BTAV_A2DP_CODEC_CHANNEL_MODE_MONO;
      } else {
        codec_config.channel_mode = BTAV_A2DP_CODEC_CHANNEL_MODE_STEREO;
      }
      break;
    default:
      ERROR("Invalid channel mask: 0x%x", common->cfg.channel_mask);
      return -1;
  }

  // Send the current codec config that has been selected by us
  if (a2dp_ctrl_send(common, &codec_config.sample_rate,
                     sizeof(btav_a2dp_codec_sample_rate_t)) < 0)
    return -1;
  if (a2dp_ctrl_send(common, &codec_config.bits_per_sample,
                     sizeof(btav_a2dp_codec_bits_per_sample_t)) < 0) {
    return -1;
  }
  if (a2dp_ctrl_send(common, &codec_config.channel_mode,
                     sizeof(btav_a2dp_codec_channel_mode_t)) < 0) {
    return -1;
  }

  INFO(
      "sent output codec config: sample_rate=0x%x bits_per_sample=0x%x "
      "channel_mode=0x%x",
      codec_config.sample_rate, codec_config.bits_per_sample,
      codec_config.channel_mode);

  return 0;
}

static int UNUSED_ATTR a2dp_get_presentation_position_cmd(struct a2dp_stream_common* common,
                                              uint64_t* bytes, uint16_t* delay,
                                              struct timespec* timestamp) {
  if ((common->ctrl_fd == AUDIO_SKT_DISCONNECTED) ||
      (common->state != AUDIO_A2DP_STATE_STARTED)) {  // Audio is not streaming
    return -1;
  }

  if (a2dp_command(common, A2DP_CTRL_GET_PRESENTATION_POSITION) < 0) {
    return -1;
  }

  if (a2dp_ctrl_receive(common, bytes, sizeof(*bytes)) < 0) {
    return -1;
  }

  if (a2dp_ctrl_receive(common, delay, sizeof(*delay)) < 0) {
    return -1;
  }

  uint32_t seconds;
  if (a2dp_ctrl_receive(common, &seconds, sizeof(seconds)) < 0) {
    return -1;
  }

  uint32_t nsec;
  if (a2dp_ctrl_receive(common, &nsec, sizeof(nsec)) < 0) {
    return -1;
  }

  timestamp->tv_sec = seconds;
  timestamp->tv_nsec = nsec;
  return 0;
}

static std::string get_a2dp_socket_path(struct a2dp_stream_common* common, const char* path) {
  INFO("get_a2dp_socket_path: path:%s, address:%s, bus:%s", path, common->bdaddress.c_str(),
        common->bus_address.c_str());
  std::string socket = path;
  // Append the address as socket name to connect socket for specified device
  if (common->bdaddress.compare("") != 0) {
    /* The control server socket named with Bluetooth address
    * for example when adddress is 11:22:33:44:AA:BB, the ctrl server socket is
    * named like /data/misc/bluedroid/.a2dp_ctrl_11_22_33_44_AA_BB
    * The data server socket is named like
    * /data/misc/bluedroid/.a2dp_data_11_22_33_44_AA_BB
    */
    std::transform(common->bdaddress.begin(), common->bdaddress.end(),
          common->bdaddress.begin(), ::toupper);
    socket.append("_" + common->bdaddress);
    std::replace(socket.begin(), socket.end(), ':', '_');
  }
  return socket;
}

static void a2dp_open_ctrl_path(struct a2dp_stream_common* common) {
  int i;
  if (common->bdaddress.empty()) return;  // empty bdaddress

  if (common->ctrl_fd != AUDIO_SKT_DISCONNECTED) return;  // already connected

  /* retry logic to catch any timing variations on control channel */
  for (i = 0; i < CTRL_CHAN_RETRY_COUNT; i++) {
    /* connect control channel if not already connected */
    if ((common->ctrl_fd = skt_connect(
             get_a2dp_socket_path(common, A2DP_CTRL_PATH).c_str(),
             AUDIO_STREAM_CONTROL_OUTPUT_BUFFER_SZ)) >= 0) {
      /* success, now check if stack is ready */
      if (check_a2dp_ready(common) == 0) break;

      ERROR("error : a2dp not ready, wait 250 ms and retry");
      usleep(250000);
      skt_disconnect(common->ctrl_fd);
      common->ctrl_fd = AUDIO_SKT_DISCONNECTED;
    }

    /* ctrl channel not ready, wait a bit */
    usleep(250000);
  }
}

/*****************************************************************************
 *
 * AUDIO DATA PATH
 *
 ****************************************************************************/

static void a2dp_stream_common_init(struct a2dp_stream_common* common) {
  FNLOG();

  common->mutex = new std::recursive_mutex;

  common->ctrl_fd = AUDIO_SKT_DISCONNECTED;
  common->audio_fd = AUDIO_SKT_DISCONNECTED;
  common->state = AUDIO_A2DP_STATE_STOPPED;

  /* manages max capacity of socket pipe */
  common->buffer_sz = AUDIO_STREAM_OUTPUT_BUFFER_SZ;

  common->device_type = AUDIO_DEVICE_NONE;
  common->volume = 32;
}

static void a2dp_stream_common_destroy(struct a2dp_stream_common* common) {
  FNLOG();

  delete common->mutex;
  common->mutex = NULL;
  common->data_sz = 0;
  if (common->data != NULL) {
    free(common->data);
    common->data = NULL;
  }
}

static int start_audio_datapath(struct a2dp_stream_common* common) {
  INFO("state %d", common->state);

  int oldstate = common->state;
  common->state = AUDIO_A2DP_STATE_STARTING;

  int a2dp_status = a2dp_command(common, A2DP_CTRL_CMD_START);
  if (a2dp_status < 0) {
    ERROR("Audiopath start failed (status %d)", a2dp_status);
    goto error;
  } else if (a2dp_status == A2DP_CTRL_ACK_INCALL_FAILURE) {
    ERROR("Audiopath start failed - in call, move to suspended");
    goto error;
  }

  /* connect socket if not yet connected */
  if (common->audio_fd == AUDIO_SKT_DISCONNECTED) {
    common->audio_fd = skt_connect(get_a2dp_socket_path(common, A2DP_DATA_PATH).c_str(), common->buffer_sz);
    if (common->audio_fd < 0) {
      ERROR("Audiopath start failed - error opening data socket");
      goto error;
    }
  }
  common->state = (a2dp_state_t)AUDIO_A2DP_STATE_STARTED;

  /* check to see if delay reporting is enabled */
  enable_delay_reporting = delay_reporting_enabled();

  return 0;

error:
  common->state = (a2dp_state_t)oldstate;
  return -1;
}

static int stop_audio_datapath(struct a2dp_stream_common* common) {
  int oldstate = common->state;

  INFO("state %d", common->state);

  /* prevent any stray output writes from autostarting the stream
     while stopping audiopath */
  common->state = AUDIO_A2DP_STATE_STOPPING;

  if (a2dp_command(common, A2DP_CTRL_CMD_STOP) < 0) {
    ERROR("audiopath stop failed");
    common->state = (a2dp_state_t)oldstate;
    return -1;
  }

  common->state = (a2dp_state_t)AUDIO_A2DP_STATE_STOPPED;

  /* disconnect audio path */
  skt_disconnect(common->audio_fd);
  common->audio_fd = AUDIO_SKT_DISCONNECTED;

  return 0;
}

static int suspend_audio_datapath(struct a2dp_stream_common* common,
                                  bool standby) {
  INFO("state %d", common->state);

  if (common->state == AUDIO_A2DP_STATE_STOPPING) return -1;

  if (a2dp_command(common, A2DP_CTRL_CMD_SUSPEND) < 0) return -1;

  if (standby)
    common->state = AUDIO_A2DP_STATE_STANDBY;
  else
    common->state = AUDIO_A2DP_STATE_SUSPENDED;

  /* disconnect audio path */
  skt_disconnect(common->audio_fd);

  common->audio_fd = AUDIO_SKT_DISCONNECTED;

  return 0;
}
#if 0
static int bt_format_from_24_to_16(a2dp_stream_out *out, const void *buffer,
                                   size_t in_size, size_t *out_size)
{
  /* The value of src_count and des_size refer to
   * memcpy_to_i16_from_p24 API
   */
  size_t src_count = in_size/3;
  size_t des_size = src_count*2;
  *out_size = des_size;
  DEBUG("src_count = %zu, des_size = %zu, data_sz = %zu",
        src_count, des_size, out->common.data_sz);
  if (des_size > out->common.data_sz) {
    out->common.data = realloc(out->common.data, des_size);
    out->common.data_sz = des_size;
    if (out->common.data == NULL) {
      ERROR("%s: Memory Allocation failed!", __func__);
      out->common.data_sz = 0;
      return -1;
    }
  }
  memcpy_by_audio_format((int16_t*)out->common.data,
                         (audio_format_t)out->common.cfg.format,
                         (uint8_t*)buffer,
                         out->common.in_config.format,
                         src_count);
  return 0;
}

static int bt_format_from_16_to_24(a2dp_stream_out *out, const void *buffer,
                                   size_t in_size, size_t *out_size)
{
  /* The value of src_count and des_size refer to
   * memcpy_to_p24_from_i16 API
   */
  size_t src_count = in_size/2;
  size_t des_size = src_count*3;
  *out_size = des_size;
  DEBUG("src_count = %zu, des_size = %zu, data_sz = %zu",
        src_count, des_size, out->common.data_sz);
  if (des_size > out->common.data_sz) {
    out->common.data = realloc(out->common.data, des_size);
    out->common.data_sz = des_size;
    if (out->common.data == NULL) {
      ERROR("%s: Memory Allocation failed!", __func__);
      out->common.data_sz = 0;
      return -1;
    }
  }
  memcpy_by_audio_format((uint8_t*)out->common.data,
                         (audio_format_t)out->common.cfg.format,
                         (int16_t*)buffer,
                         out->common.in_config.format,
                         src_count);
  return 0;
}
#endif
/*****************************************************************************
 *
 *  audio output callbacks
 *
 ****************************************************************************/

static ssize_t out_write_ext(struct audio_stream_out* stream, const int index, const void* buffer,
                         size_t bytes, bool printLog) {
  struct a2dp_stream_out* out = (struct a2dp_stream_out*)stream;
  int sent = -1;
  size_t write_bytes = bytes;
#if 0
  size_t write_reformat_bytes = 0;
#endif
  if (printLog) {
    DEBUG("bdaddress:%s, bus:%s", out->commons[index].bdaddress.c_str(), out->commons[index].bus_address.c_str());

    DEBUG("write %zu bytes (fd %d)", bytes, out->commons[index].audio_fd);
  }

  std::unique_lock<std::recursive_mutex> lock(*out->commons[index].mutex);
  if (out->commons[index].state == AUDIO_A2DP_STATE_SUSPENDED ||
      out->commons[index].state == AUDIO_A2DP_STATE_STOPPING) {
    DEBUG("stream suspended or closing");
    goto finish;
  }

  /* only allow autostarting if we are in stopped or standby */
  if ((out->commons[index].state == AUDIO_A2DP_STATE_STOPPED) ||
      (out->commons[index].state == AUDIO_A2DP_STATE_STANDBY)) {
    if (start_audio_datapath(&out->commons[index]) < 0) {
      goto finish;
    }
  } else if (out->commons[index].state != AUDIO_A2DP_STATE_STARTED) {
    ERROR("stream not in stopped or standby");
    goto finish;
  }

  // Mix the stereo into mono if necessary
  if (out->commons[index].cfg.is_stereo_to_mono) {
    const size_t frames = bytes / audio_stream_out_frame_size(stream);
    int16_t* src = (int16_t*)buffer;
    int16_t* dst = (int16_t*)buffer;
    for (size_t i = 0; i < frames; i++, dst++, src += 2) {
      *dst = (int16_t)(((int32_t)src[0] + (int32_t)src[1]) >> 1);
    }
    write_bytes /= 2;
    DEBUG("stereo-to-mono mixing: write %zu bytes (fd %d)", write_bytes,
          out->commons[index].audio_fd);
  }

  // Adjust volume by property
  {
    // default volume is 32
    int volume = out->commons[index].volume;
#if 0
#ifndef BLUETOOTH_A2DP_EXT
    int value = volume;
    if (out->commons[index].bus_address.compare(BUS_ADDRESS_BUS1) == 0) {
        value = property_get_int32("persist.vendor.mercedes.audio.left.volume.value", 32);
    } else if (out->commons[index].bus_address.compare(BUS_ADDRESS_BUS2) == 0) {
        value = property_get_int32("persist.vendor.mercedes.audio.right.volume.value", 32);
    }
#else
    int value = property_get_int32("persist.vendor.mercedes.audio.right.volume.value", 32);
#endif
    if(value < 0 || value > 32) {
      value = 32;
    }
    if(value != volume) {
      WARN("volume changed from %d to %d", volume, value);
      volume = value;
    }
#endif
    if (printLog) {
      WARN("%s: volume:%d", __func__, volume);
    }
    size_t cnt = write_bytes / audio_stream_out_frame_size(stream);
    int16_t *data = (int16_t *)buffer;
    for (size_t i = 0; i < cnt; i++, data += 2) {
      data[0] = (int16_t)((int32_t)data[0] * volume >> 5);
      data[1] = (int16_t)((int32_t)data[1] * volume >> 5);
    }
  }

  lock.unlock();
#if 0
  if ((audio_format_t)out->common.cfg.format != out->common.in_config.format) {
    if (((audio_format_t)out->common.cfg.format == AUDIO_FORMAT_PCM_16_BIT) &&
        (out->common.in_config.format == AUDIO_FORMAT_PCM_24_BIT_PACKED)) {
      /* Do re-format, Source bitrate is AUDIO_FORMAT_PCM_24_BIT_PACKED,
       * Destination bitrate is AUDIO_FORMAT_PCM_16_BIT
       */
      bt_format_from_24_to_16(out, buffer, write_bytes,
                              &write_reformat_bytes);
      sent = skt_write(out->common.audio_fd,
                       (int16_t*)out->common.data,
                       write_reformat_bytes);
    } else if (((audio_format_t)out->common.cfg.format == AUDIO_FORMAT_PCM_24_BIT_PACKED) &&
               (out->common.in_config.format == AUDIO_FORMAT_PCM_16_BIT)) {
      /* Do re-format, Source bitrate is AUDIO_FORMAT_PCM_16_BIT,
       * Destination bitrate is AUDIO_FORMAT_PCM_24_BIT_PACKED
       */
      bt_format_from_16_to_24(out, buffer, write_bytes,
                              &write_reformat_bytes);
      sent = skt_write(out->common.audio_fd,
                       (int16_t*)out->common.data,
                       write_reformat_bytes);
    } else {
      ERROR("%s: Not support re-format for bitrate", __func__);
      goto finish;
    }
  } else {
#endif
  sent = skt_write(out->commons[index].audio_fd, buffer, write_bytes);
#if 0
  }
#endif
  lock.lock();

  if (sent == -1) {
    skt_disconnect(out->commons[index].audio_fd);
    out->commons[index].audio_fd = AUDIO_SKT_DISCONNECTED;
    if ((out->commons[index].state != AUDIO_A2DP_STATE_SUSPENDED) &&
        (out->commons[index].state != AUDIO_A2DP_STATE_STOPPING)) {
      out->commons[index].state = AUDIO_A2DP_STATE_STOPPED;
    } else {
      ERROR("write failed : stream suspended, avoid resetting state");
    }
    goto finish;
  }

finish:;
  lock.unlock();

  // If send didn't work out, sleep to emulate write delay.
  if (sent == -1 && device_group_map.size() <= 1) {
    const int us_delay = calc_audiotime_usec(out->commons[index].cfg, bytes);
    DEBUG("emulate a2dp write delay (%d us)", us_delay);
    usleep(us_delay);
  }
  return bytes;
}

struct timeval getCurrentTime() {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return tv;
}

double getTimeDifferenceInSeconds(const struct timeval& startTime, const struct timeval& currentTime) {
    return (currentTime.tv_sec - startTime.tv_sec) + (currentTime.tv_usec - startTime.tv_usec) / 1000000.0;
}

static ssize_t out_write(struct audio_stream_out* stream, const void* buffer,
                         size_t bytes) {
// INFO("out_write 21:16 : bytes=%zu", bytes);
  ssize_t ret = bytes;
  struct a2dp_stream_out* out = (struct a2dp_stream_out*)stream;
  

  //   do {
  //   static FILE* dump_file = nullptr;
  //   if (dump_file == nullptr) {
  //     dump_file = fopen("/data/misc/audioserver/a2dp_out-audioserver.pcm", "wb");
  //     if (dump_file == nullptr) {
  //       ERROR("out_write: failed to open dump file: %s", strerror(errno));
  //     }
  //   }
  //   if (dump_file != nullptr) {
  //     fwrite(buffer, 1, bytes, dump_file);
  //   }
  // } while (0);
  // bool debug = false;
  // if (debug) {
  //   INFO("out_write 21:16 : bytes=%zu, return", bytes);
  //   return bytes;
  // }

  bool printLog = false;
  struct timeval currentTime = getCurrentTime();

  if (getTimeDifferenceInSeconds(out->lastTime, currentTime) > 1) {
    printLog = true;
    out->lastTime = currentTime;
  } else {
    out->bytes_write += bytes;
  }

  for (auto it : bus_stream_map) {
    //it.first��bus��ַ��it.second��streamʵ��
    if ((a2dp_stream_out*)stream == it.second){
      if (printLog) {
        INFO("%s find bus %s bytes_write=%" PRIu64, __func__, it.first.c_str(), out->bytes_write);
        out->bytes_write = 0;
      }
      uint8_t* tempBuffer = NULL;
      for (int i = 0; i < COMMON_ARRAY_SIZE; i++) {
        if (!out->commons[i].bdaddress.empty()) {
          if (printLog) {
            INFO("%s, common[%d] bdaddress is %s", __func__, i,
                out->commons[i].bdaddress.c_str());
          }
          if (!tempBuffer) {
            tempBuffer = (uint8_t*)malloc(bytes);
          }
          memcpy(tempBuffer, buffer, bytes);
          ret = out_write_ext(stream, i, tempBuffer, bytes, printLog);
        }
      }

      if (tempBuffer) {
        free(tempBuffer);
        tempBuffer = NULL;
      }
    }
  }

#if 0
  for (auto it : device_group_map) {
    // Write data to all stream in this bus group
    if (it.second.compare(out->common.bus_address) == 0) {
      ret = out_write_ext((struct audio_stream_out*)device_stream_map.find(it.first)->second, buffer, bytes);
    }
  }
#endif
  const size_t frames = bytes / audio_stream_out_frame_size(stream);
  out->frames_rendered += frames;
  out->frames_presented += frames;
  return ret;
}

static uint32_t out_get_sample_rate(const struct audio_stream* stream) {
  struct a2dp_stream_out* out = (struct a2dp_stream_out*)stream;

  DEBUG("rate %" PRIu32, out->commons[0].cfg.rate);

  return out->commons[0].cfg.rate;
}

static int out_set_sample_rate(struct audio_stream* stream, uint32_t rate) {
  struct a2dp_stream_out* out = (struct a2dp_stream_out*)stream;

  DEBUG("out_set_sample_rate : %" PRIu32, rate);

  out->commons[0].cfg.rate = rate;

  return 0;
}

static size_t out_get_buffer_size(const struct audio_stream* stream) {
  struct a2dp_stream_out* out = (struct a2dp_stream_out*)stream;
  // period_size is the AudioFlinger mixer buffer size.
  const size_t period_size =
      out->commons[0].buffer_sz / AUDIO_STREAM_OUTPUT_BUFFER_PERIODS;

  DEBUG("socket buffer size: %zu  period size: %zu", out->commons[0].buffer_sz,
        period_size);

  return period_size;
}

size_t audio_a2dp_hw_stream_compute_buffer_size(
    btav_a2dp_codec_sample_rate_t codec_sample_rate,
    btav_a2dp_codec_bits_per_sample_t codec_bits_per_sample,
    btav_a2dp_codec_channel_mode_t codec_channel_mode) {
  size_t buffer_sz = AUDIO_STREAM_OUTPUT_BUFFER_SZ;  // Default value
  const uint64_t time_period_ms = 20;                // Conservative 20ms
  uint32_t sample_rate;
  uint32_t bits_per_sample;
  uint32_t number_of_channels;

  // Check the codec config sample rate
  switch (codec_sample_rate) {
    case BTAV_A2DP_CODEC_SAMPLE_RATE_44100:
      sample_rate = 44100;
      break;
    case BTAV_A2DP_CODEC_SAMPLE_RATE_48000:
      sample_rate = 48000;
      break;
    case BTAV_A2DP_CODEC_SAMPLE_RATE_88200:
      sample_rate = 88200;
      break;
    case BTAV_A2DP_CODEC_SAMPLE_RATE_96000:
      sample_rate = 96000;
      break;
    case BTAV_A2DP_CODEC_SAMPLE_RATE_176400:
      sample_rate = 176400;
      break;
    case BTAV_A2DP_CODEC_SAMPLE_RATE_192000:
      sample_rate = 192000;
      break;
    case BTAV_A2DP_CODEC_SAMPLE_RATE_NONE:
    default:
      ERROR("Invalid sample rate: 0x%x", codec_sample_rate);
      return buffer_sz;
  }

  // Check the codec config bits per sample
  switch (codec_bits_per_sample) {
    case BTAV_A2DP_CODEC_BITS_PER_SAMPLE_16:
      bits_per_sample = 16;
      break;
    case BTAV_A2DP_CODEC_BITS_PER_SAMPLE_24:
      bits_per_sample = 24;
      break;
    case BTAV_A2DP_CODEC_BITS_PER_SAMPLE_32:
      bits_per_sample = 32;
      break;
    case BTAV_A2DP_CODEC_BITS_PER_SAMPLE_NONE:
    default:
      ERROR("Invalid bits per sample: 0x%x", codec_bits_per_sample);
      return buffer_sz;
  }

  // Check the codec config channel mode
  switch (codec_channel_mode) {
    case BTAV_A2DP_CODEC_CHANNEL_MODE_MONO:
      number_of_channels = 1;
      break;
    case BTAV_A2DP_CODEC_CHANNEL_MODE_STEREO:
      number_of_channels = 2;
      break;
    case BTAV_A2DP_CODEC_CHANNEL_MODE_NONE:
    default:
      ERROR("Invalid channel mode: 0x%x", codec_channel_mode);
      return buffer_sz;
  }

  //
  // The buffer size is computed by using the following formula:
  //
  // AUDIO_STREAM_OUTPUT_BUFFER_SIZE =
  //    (TIME_PERIOD_MS * AUDIO_STREAM_OUTPUT_BUFFER_PERIODS *
  //     SAMPLE_RATE_HZ * NUMBER_OF_CHANNELS * (BITS_PER_SAMPLE / 8)) / 1000
  //
  // AUDIO_STREAM_OUTPUT_BUFFER_PERIODS controls how the socket buffer is
  // divided for AudioFlinger data delivery. The AudioFlinger mixer delivers
  // data in chunks of
  // (AUDIO_STREAM_OUTPUT_BUFFER_SIZE / AUDIO_STREAM_OUTPUT_BUFFER_PERIODS) .
  // If the number of periods is 2, the socket buffer represents "double
  // buffering" of the AudioFlinger mixer buffer.
  //
  // Furthermore, the AudioFlinger expects the buffer size to be a multiple
  // of 16 frames.
  const size_t divisor = (AUDIO_STREAM_OUTPUT_BUFFER_PERIODS * 16 *
                          number_of_channels * bits_per_sample) /
                         8;

  buffer_sz = (time_period_ms * AUDIO_STREAM_OUTPUT_BUFFER_PERIODS *
               sample_rate * number_of_channels * (bits_per_sample / 8)) /
              1000;

  // Adjust the buffer size so it can be divided by the divisor
  const size_t remainder = buffer_sz % divisor;
  if (remainder != 0) {
    buffer_sz += divisor - remainder;
  }

  return buffer_sz;
}

static audio_channel_mask_t out_get_channels(
    const struct audio_stream* stream) {
  struct a2dp_stream_out* out = (struct a2dp_stream_out*)stream;

  VERBOSE("channels 0x%" PRIx32, out->commons[0].cfg.channel_mask);

  return (audio_channel_mask_t)out->commons[0].cfg.channel_mask;
}

static audio_format_t out_get_format(const struct audio_stream* stream) {
  struct a2dp_stream_out* out = (struct a2dp_stream_out*)stream;
  VERBOSE("format 0x%x", out->commons[0].cfg.format);
  return (audio_format_t)out->commons[0].cfg.format;
}

static int out_set_format(UNUSED_ATTR struct audio_stream* stream,
                          UNUSED_ATTR audio_format_t format) {
  DEBUG("setting format not yet supported (0x%x)", format);
  return -ENOSYS;
}

static int out_standby(struct audio_stream* stream) {
  struct a2dp_stream_out* out = (struct a2dp_stream_out*)stream;
  int retVal = 0;

  FNLOG();
  // prevent interference with adev_set_parameters.
  for (int i = 0; i < COMMON_ARRAY_SIZE; i++) {
      std::lock_guard<std::recursive_mutex> lock(*out->commons[i].mutex);
      if (out->commons[i].device_type != AUDIO_DEVICE_OUT_BUS) {
        // Do nothing in SUSPENDED state.
        if (out->commons[i].state != AUDIO_A2DP_STATE_SUSPENDED)
          retVal = suspend_audio_datapath(&out->commons[i], true);
      } else {
        const a2dp_state_t state = out->commons[i].state;
        INFO("closing output (state %d)", (int)state);
        if ((state == AUDIO_A2DP_STATE_STARTED) ||
            (state == AUDIO_A2DP_STATE_STOPPING)) {
          retVal = stop_audio_datapath(&out->commons[i]);
        }

        skt_disconnect(out->commons[i].ctrl_fd);
        out->commons[i].ctrl_fd = AUDIO_SKT_DISCONNECTED;
      }
  }

  out->frames_rendered = 0;  // rendered is reset, presented is not

  return retVal;
}

static int out_dump(UNUSED_ATTR const struct audio_stream* stream,
                    UNUSED_ATTR int fd) {
  FNLOG();
  return 0;
}

static int out_set_parameters(struct audio_stream* stream,
                              const char* kvpairs) {
  struct a2dp_stream_out* out = (struct a2dp_stream_out*)stream;

  for (int i = 0; i < COMMON_ARRAY_SIZE; i++) {
    VERBOSE("stream %p bus %s", stream, out->commons[i].bus_address.c_str());

    VERBOSE("state %d kvpairs %s", out->commons[i].state, kvpairs);
  }

  std::unordered_map<std::string, std::string> params =
      hash_map_utils_new_from_string_params(kvpairs);
  int status = 0;

  if (params.empty()) return status;

  /* dump params */
  // hash_map_utils_dump_string_keys_string_values(params);

  if (params.find("displayId") != params.end() && params["displayId"].compare("0") != 0) {
    params["displayId"] = "2";
  }

  for (int i = 0; i < COMMON_ARRAY_SIZE; i++) {
      std::lock_guard<std::recursive_mutex> lock(*out->commons[i].mutex);

      if (params["closing"].compare("true") == 0) {
        DEBUG("stream closing, disallow any writes");
        out->commons[i].state = AUDIO_A2DP_STATE_STOPPING;
      }
    #ifndef BLUETOOTH_A2DP_EXT
      if (params["A2dpSuspended"].compare("true") == 0 && params["displayId"].compare(out->commons[i].displayId) == 0 ) {
    #else
      if (params["A2dpSuspended"].compare("true") == 0 && params["displayId"].compare("2") == 0 ) {
    #endif
        DEBUG("A2dpSuspended true");
        if (out->commons[i].state == AUDIO_A2DP_STATE_STARTED) {
            status = suspend_audio_datapath(&out->commons[i], false);
        }
    #ifndef BLUETOOTH_A2DP_EXT
      } else if (params["A2dpSuspended"].compare("false") == 0 && params["displayId"].compare(out->commons[i].displayId) == 0 ) {
    #else
      } else if (params["A2dpSuspended"].compare("false") == 0 && params["displayId"].compare("2") == 0 ) {
    #endif
        /* Do not start the streaming automatically. If the phone was streaming
         * prior to being suspended, the next out_write shall trigger the
         * AVDTP start procedure */
        DEBUG("A2dpSuspended false");
        if (out->commons[i].state == AUDIO_A2DP_STATE_SUSPENDED)
          out->commons[i].state = AUDIO_A2DP_STATE_STANDBY;
        /* Irrespective of the state, return 0 */
      }

      std::string address = params["bdaddress"];

      VERBOSE("bdaddress %s", address.c_str());
      if (address.compare("") != 0) {
        // for BUS device
        if (IS_AUDIO_DEVICE_OUT_BUS(out->commons[i].device_type)) {
          if (out->commons[i].bus_address.compare(params["zone_bus"]) == 0) {
            if (params["connected"].compare("1") == 0) {
              INFO("%s common[%d], address=%s", __func__, i, out->commons[i].bdaddress.c_str());
              if (!out->commons[i].bdaddress.empty()) {
                if (out->commons[i].bdaddress == address) {
                  INFO("%s address already be set. %s", __func__, out->commons[i].bdaddress.c_str());
                  return 0;
                }
                INFO("%s address is NOT empty[%s], continue...", __func__, out->commons[i].bdaddress.c_str());
                continue;
              }
              INFO("save bdaddress %s for bus device %s with displayId %s", address.c_str(),
                out->commons[i].bus_address.c_str(), params["displayId"].c_str());
              out->commons[i].displayId = params["displayId"];
              out->commons[i].bdaddress = address;
              device_stream_map.erase(address);
              device_stream_map.emplace(address, out);
              device_group_map.erase(address);
              device_group_map.emplace(address, params["zone_bus"]);
              btav_a2dp_codec_config_t codec_config;
              btav_a2dp_codec_config_t codec_capability;
              status = a2dp_read_output_audio_config(&out->commons[i], &codec_config,
                                                &codec_capability,
                                                true /* update_stream_config */);
              if (status < 0) {
                ERROR("a2dp_read_output_audio_config failed");
                out->commons[i].bdaddress = "";
                device_stream_map.erase(address);
                device_group_map.erase(address);
                skt_disconnect(out->commons[i].ctrl_fd);
                out->commons[i].ctrl_fd = AUDIO_SKT_DISCONNECTED;
              }
              return status;
            } else {
              INFO("bdaddress %s disconnected", address.c_str());
              device_stream_map.erase(address);
              device_group_map.erase(address);
              if (!out->commons[i].bdaddress.empty()) {
                if (out->commons[i].bdaddress == address) {
                    out->commons[i].displayId = "";
                    out->commons[i].bdaddress = "";
                    const a2dp_state_t state = out->commons[i].state;
                    INFO("closing output (state %d)", (int)state);
                    if ((state == AUDIO_A2DP_STATE_STARTED) ||
                        (state == AUDIO_A2DP_STATE_STOPPING)) {
                        stop_audio_datapath(&out->commons[i]);
                    }

                    skt_disconnect(out->commons[i].ctrl_fd);
                    out->commons[i].ctrl_fd = AUDIO_SKT_DISCONNECTED;
                    return 0;
                }
              }
            }
          }
        } else {
          // for A2DP device
          INFO("save bdaddress %s for A2DP OUT DEVICE", address.c_str());
          out->commons[i].bdaddress = address;
        }
      }

      if (params["bt.volume"].compare("") != 0) {
        if (IS_AUDIO_DEVICE_OUT_BUS(out->commons[i].device_type)) {
          if (out->commons[i].bus_address.compare(params["bt.bus"]) == 0) {
            int value = atoi(params["bt.volume"].c_str());

            if (value < 0 || value > 32) {
              value = 32;
            }

            if (value != out->commons[i].volume) {
              WARN("volume changed from %d to %d", out->commons[i].volume,
                   value);
              out->commons[i].volume = value;
            }
          }
        }
      }
  }

  return status;
}

static char* out_get_parameters(const struct audio_stream* stream,
                                const char* keys) {
  FNLOG();

  INFO("stream %p", stream);

  btav_a2dp_codec_config_t codec_config;
  btav_a2dp_codec_config_t codec_capability;

  struct a2dp_stream_out* out = (struct a2dp_stream_out*)stream;

  std::unordered_map<std::string, std::string> params =
      hash_map_utils_new_from_string_params(keys);
  std::unordered_map<std::string, std::string> return_params;

  if (params.empty()) return strdup("");

  for (int i = 0; i < COMMON_ARRAY_SIZE; i++){
    std::lock_guard<std::recursive_mutex> lock(*out->commons[i].mutex);
    if (!out->commons[i].bdaddress.empty()) {
      if (a2dp_read_output_audio_config(&out->commons[i], &codec_config,
                                        &codec_capability,
                                        false /* update_stream_config */) < 0) {
          ERROR("a2dp_read_output_audio_config failed");
          goto done;
      } else {
          break;
      }
    }
  }

  // Add the format
  if (params.find(AUDIO_PARAMETER_STREAM_SUP_FORMATS) != params.end()) {
    std::string param;
    if (codec_capability.bits_per_sample & BTAV_A2DP_CODEC_BITS_PER_SAMPLE_16) {
      if (!param.empty()) param += "|";
      param += "AUDIO_FORMAT_PCM_16_BIT";
    }
    if (codec_capability.bits_per_sample & BTAV_A2DP_CODEC_BITS_PER_SAMPLE_24) {
      if (!param.empty()) param += "|";
      param += "AUDIO_FORMAT_PCM_24_BIT_PACKED";
    }
    if (codec_capability.bits_per_sample & BTAV_A2DP_CODEC_BITS_PER_SAMPLE_32) {
      if (!param.empty()) param += "|";
      param += "AUDIO_FORMAT_PCM_32_BIT";
    }
    if (param.empty()) {
      ERROR("Invalid codec capability bits_per_sample=0x%x",
            codec_capability.bits_per_sample);
      goto done;
    } else {
      return_params[AUDIO_PARAMETER_STREAM_SUP_FORMATS] = param;
    }
  }

  // Add the sample rate
  if (params.find(AUDIO_PARAMETER_STREAM_SUP_SAMPLING_RATES) != params.end()) {
    std::string param;
    if (codec_capability.sample_rate & BTAV_A2DP_CODEC_SAMPLE_RATE_44100) {
      if (!param.empty()) param += "|";
      param += "44100";
    }
    if (codec_capability.sample_rate & BTAV_A2DP_CODEC_SAMPLE_RATE_48000) {
      if (!param.empty()) param += "|";
      param += "48000";
    }
    if (codec_capability.sample_rate & BTAV_A2DP_CODEC_SAMPLE_RATE_88200) {
      if (!param.empty()) param += "|";
      param += "88200";
    }
    if (codec_capability.sample_rate & BTAV_A2DP_CODEC_SAMPLE_RATE_96000) {
      if (!param.empty()) param += "|";
      param += "96000";
    }
    if (codec_capability.sample_rate & BTAV_A2DP_CODEC_SAMPLE_RATE_176400) {
      if (!param.empty()) param += "|";
      param += "176400";
    }
    if (codec_capability.sample_rate & BTAV_A2DP_CODEC_SAMPLE_RATE_192000) {
      if (!param.empty()) param += "|";
      param += "192000";
    }
    if (param.empty()) {
      ERROR("Invalid codec capability sample_rate=0x%x",
            codec_capability.sample_rate);
      goto done;
    } else {
      return_params[AUDIO_PARAMETER_STREAM_SUP_SAMPLING_RATES] = param;
    }
  }

  // Add the channel mask
  if (params.find(AUDIO_PARAMETER_STREAM_SUP_CHANNELS) != params.end()) {
    std::string param;
    if (codec_capability.channel_mode & BTAV_A2DP_CODEC_CHANNEL_MODE_MONO) {
      if (!param.empty()) param += "|";
      param += "AUDIO_CHANNEL_OUT_MONO";
    }
    if (codec_capability.channel_mode & BTAV_A2DP_CODEC_CHANNEL_MODE_STEREO) {
      if (!param.empty()) param += "|";
      param += "AUDIO_CHANNEL_OUT_STEREO";
    }
    if (param.empty()) {
      ERROR("Invalid codec capability channel_mode=0x%x",
            codec_capability.channel_mode);
      goto done;
    } else {
      return_params[AUDIO_PARAMETER_STREAM_SUP_CHANNELS] = param;
    }
  }

done:
  std::string result;
  for (const auto& ptr : return_params) {
    result += ptr.first + "=" + ptr.second + ";";
  }

  INFO("get parameters result = %s", result.c_str());

  return strdup(result.c_str());
}

static uint32_t out_get_latency(const struct audio_stream_out* stream) {
  int latency_us;

  struct a2dp_stream_out* out = (struct a2dp_stream_out*)stream;

  FNLOG();

  latency_us =
      ((out->commons[0].buffer_sz * 1000) /
       audio_stream_out_frame_size(&out->stream) / out->commons[0].cfg.rate) *
      1000;

  return (latency_us / 1000) + 200;
}

static int out_set_volume(UNUSED_ATTR struct audio_stream_out* stream,
                          UNUSED_ATTR float left, UNUSED_ATTR float right) {
  FNLOG();

  /* volume controlled in audioflinger mixer (digital) */

  return -ENOSYS;
}

static int out_get_presentation_position(const struct audio_stream_out* stream,
                                         uint64_t* frames,
                                         struct timespec* timestamp) {
  struct a2dp_stream_out* out = (struct a2dp_stream_out*)stream;

  FNLOG();
  if (stream == NULL || frames == NULL || timestamp == NULL) return -EINVAL;

  std::lock_guard<std::recursive_mutex> lock(*out->commons[0].mutex);

  // bytes is the total number of bytes sent by the Bluetooth stack to a
  // remote headset
  // uint64_t bytes = 0;

  // delay_report is the audio delay from the remote headset receiving data to
  // the headset playing sound in units of 1/10ms
  // uint16_t delay_report = 0;

  // If for some reason getting a delay fails or delay reports are disabled,
  // default to old delay
  // if (enable_delay_reporting &&
  //     a2dp_get_presentation_position_cmd(&out->commons[0], &bytes, &delay_report,
  //                                        timestamp) == 0) {
  //   uint64_t delay_ns = delay_report * DELAY_TO_NS;
  //   if (delay_ns > MIN_DELAY_MS * MS_TO_NS &&
  //       delay_ns < MAX_DELAY_MS * MS_TO_NS) {
  //     *frames = bytes / audio_stream_out_frame_size(stream);

  //     timestamp->tv_nsec += delay_ns;
  //     if (timestamp->tv_nsec > 1 * SEC_TO_NS) {
  //       timestamp->tv_sec++;
  //       timestamp->tv_nsec -= SEC_TO_NS;
  //     }
  //     return 0;
  //   }
  // }

  uint64_t latency_frames =
      (uint64_t)out_get_latency(stream) * out->commons[0].cfg.rate / 1000;
  // ERROR("out_get_presentation_position out_get_latency=%lu rate=%lu", (uint64_t)out_get_latency(stream), out->commons[0].cfg.rate);
  // ERROR("out_get_presentation_position latency_frames=%lu frames_presented=%lu", latency_frames, out->frames_presented);
  if (out->frames_presented >= latency_frames) {
    clock_gettime(CLOCK_MONOTONIC, timestamp);
    *frames = out->frames_presented - latency_frames;
    // ERROR("out_get_presentation_position frames=%lu tv_sec=%lu tv_nsec=%lu", *frames, timestamp->tv_sec, timestamp->tv_nsec);
    return 0;
  }

  return -EWOULDBLOCK;
}

static int out_get_render_position(const struct audio_stream_out* stream,
                                   uint32_t* dsp_frames) {
  struct a2dp_stream_out* out = (struct a2dp_stream_out*)stream;

  FNLOG();
  if (stream == NULL || dsp_frames == NULL) return -EINVAL;

  std::lock_guard<std::recursive_mutex> lock(*out->commons[0].mutex);
  uint64_t latency_frames =
      (uint64_t)out_get_latency(stream) * out->commons[0].cfg.rate / 1000;
  if (out->frames_rendered >= latency_frames) {
    *dsp_frames = (uint32_t)(out->frames_rendered - latency_frames);
    // ERROR("out_get_render_position dsp_frames=%lu", *dsp_frames);
  } else {
    *dsp_frames = 0;
    // ERROR("out_get_render_position dsp_frames=%d", *dsp_frames);
  }
  return 0;
}

static int out_add_audio_effect(UNUSED_ATTR const struct audio_stream* stream,
                                UNUSED_ATTR effect_handle_t effect) {
  FNLOG();
  return 0;
}

static int out_remove_audio_effect(
    UNUSED_ATTR const struct audio_stream* stream,
    UNUSED_ATTR effect_handle_t effect) {
  FNLOG();
  return 0;
}

/*
 * AUDIO INPUT STREAM
 */

static uint32_t in_get_sample_rate(const struct audio_stream* stream) {
  struct a2dp_stream_in* in = (struct a2dp_stream_in*)stream;

  FNLOG();
  return in->common.cfg.rate;
}

static int in_set_sample_rate(struct audio_stream* stream, uint32_t rate) {
  struct a2dp_stream_in* in = (struct a2dp_stream_in*)stream;

  FNLOG();

  if (in->common.cfg.rate > 0 && in->common.cfg.rate == rate)
    return 0;
  else
    return -1;
}

static size_t in_get_buffer_size(
    UNUSED_ATTR const struct audio_stream* stream) {
  FNLOG();
  return 320;
}

static audio_channel_mask_t in_get_channels(const struct audio_stream* stream) {
  struct a2dp_stream_in* in = (struct a2dp_stream_in*)stream;

  FNLOG();
  return (audio_channel_mask_t)in->common.cfg.channel_mask;
}

static audio_format_t in_get_format(
    UNUSED_ATTR const struct audio_stream* stream) {
  FNLOG();
  return AUDIO_FORMAT_PCM_16_BIT;
}

static int in_set_format(UNUSED_ATTR struct audio_stream* stream,
                         UNUSED_ATTR audio_format_t format) {
  FNLOG();
  if (format == AUDIO_FORMAT_PCM_16_BIT)
    return 0;
  else
    return -1;
}

static int in_standby(UNUSED_ATTR struct audio_stream* stream) {
  FNLOG();
  return 0;
}

static int in_dump(UNUSED_ATTR const struct audio_stream* stream,
                   UNUSED_ATTR int fd) {
  FNLOG();
  return 0;
}

static int in_set_parameters(UNUSED_ATTR struct audio_stream* stream,
                             UNUSED_ATTR const char* kvpairs) {
  FNLOG();
  return 0;
}

static char* in_get_parameters(UNUSED_ATTR const struct audio_stream* stream,
                               UNUSED_ATTR const char* keys) {
  FNLOG();
  return strdup("");
}

static int in_set_gain(UNUSED_ATTR struct audio_stream_in* stream,
                       UNUSED_ATTR float gain) {
  FNLOG();
  return 0;
}

static ssize_t in_read(struct audio_stream_in* stream, void* buffer,
                       size_t bytes) {
  struct a2dp_stream_in* in = (struct a2dp_stream_in*)stream;
  int read;
  int us_delay;

  DEBUG("read %zu bytes, state: %d", bytes, in->common.state);

  std::unique_lock<std::recursive_mutex> lock(*in->common.mutex);
  if (in->common.state == AUDIO_A2DP_STATE_SUSPENDED ||
      in->common.state == AUDIO_A2DP_STATE_STOPPING) {
    DEBUG("stream suspended");
    goto error;
  }

  /* only allow autostarting if we are in stopped or standby */
  if ((in->common.state == AUDIO_A2DP_STATE_STOPPED) ||
      (in->common.state == AUDIO_A2DP_STATE_STANDBY)) {
    if (start_audio_datapath(&in->common) < 0) {
      goto error;
    }
  } else if (in->common.state != AUDIO_A2DP_STATE_STARTED) {
    ERROR("stream not in stopped or standby");
    goto error;
  }

  lock.unlock();
  read = skt_read(in->common.audio_fd, buffer, bytes);
  lock.lock();
  if (read == -1) {
    skt_disconnect(in->common.audio_fd);
    in->common.audio_fd = AUDIO_SKT_DISCONNECTED;
    if ((in->common.state != AUDIO_A2DP_STATE_SUSPENDED) &&
        (in->common.state != AUDIO_A2DP_STATE_STOPPING)) {
      in->common.state = AUDIO_A2DP_STATE_STOPPED;
    } else {
      ERROR("read failed : stream suspended, avoid resetting state");
    }
    goto error;
  } else if (read == 0) {
    DEBUG("read time out - return zeros");
    memset(buffer, 0, bytes);
    read = bytes;
  }
  lock.unlock();

  DEBUG("read %d bytes out of %zu bytes", read, bytes);
  return read;

error:
  memset(buffer, 0, bytes);
  us_delay = calc_audiotime_usec(in->common.cfg, bytes);
  DEBUG("emulate a2dp read delay (%d us)", us_delay);

  usleep(us_delay);
  return bytes;
}

static uint32_t in_get_input_frames_lost(
    UNUSED_ATTR struct audio_stream_in* stream) {
  FNLOG();
  return 0;
}

static int in_add_audio_effect(UNUSED_ATTR const struct audio_stream* stream,
                               UNUSED_ATTR effect_handle_t effect) {
  FNLOG();
  return 0;
}

static int in_remove_audio_effect(UNUSED_ATTR const struct audio_stream* stream,
                                  UNUSED_ATTR effect_handle_t effect) {
  FNLOG();

  return 0;
}

static int adev_open_output_stream(struct audio_hw_device* dev,
                                   UNUSED_ATTR audio_io_handle_t handle,
                                   audio_devices_t devices,
                                   UNUSED_ATTR audio_output_flags_t flags,
                                   struct audio_config* config,
                                   struct audio_stream_out** stream_out,
                                   UNUSED_ATTR const char* address)

{
  struct a2dp_audio_device* a2dp_dev = (struct a2dp_audio_device*)dev;
  struct a2dp_stream_out* out;
  int ret = 0;
  // Make sure we always have the feeding parameters configured
  btav_a2dp_codec_config_t codec_config;
  btav_a2dp_codec_config_t codec_capability;

  INFO("dev %p", dev);

  INFO("opening output");
  INFO("address %s, a2dp_dev->bdaddress %s", address, a2dp_dev->bdaddress.c_str());
  // protect against adev->output and stream_out from being inconsistent
  std::lock_guard<std::recursive_mutex> lock(*a2dp_dev->mutex);
  out = (struct a2dp_stream_out*)calloc(1, sizeof(struct a2dp_stream_out));

  if (!out) return -ENOMEM;
  out->lastTime = getCurrentTime();
  out->bytes_write = 0;
  out->stream.common.get_sample_rate = out_get_sample_rate;
  out->stream.common.set_sample_rate = out_set_sample_rate;
  out->stream.common.get_buffer_size = out_get_buffer_size;
  out->stream.common.get_channels = out_get_channels;
  out->stream.common.get_format = out_get_format;
  out->stream.common.set_format = out_set_format;
  out->stream.common.standby = out_standby;
  out->stream.common.dump = out_dump;
  out->stream.common.set_parameters = out_set_parameters;
  out->stream.common.get_parameters = out_get_parameters;
  out->stream.common.add_audio_effect = out_add_audio_effect;
  out->stream.common.remove_audio_effect = out_remove_audio_effect;
  out->stream.get_latency = out_get_latency;
  out->stream.set_volume = out_set_volume;
  out->stream.write = out_write;
  out->stream.get_render_position = out_get_render_position;
  out->stream.get_presentation_position = out_get_presentation_position;

  for(int i = 0; i < COMMON_ARRAY_SIZE; i++) {
      /* initialize a2dp specifics */
      a2dp_stream_common_init(&out->commons[i]);

      if (strcmp(a2dp_dev->bdaddress.c_str(), address) == 0) {
        INFO("Save bdaddress to out->common.bdaddress");
        out->commons[i].bdaddress = address;
      }

      // Make sure we always have the feeding parameters configured
      out->commons[i].device_type = devices;
      if (IS_AUDIO_DEVICE_OUT_BUS(devices)) {
        out->commons[i].bus_address = address;
        INFO("bus %s", out->commons[i].bus_address.c_str());
      } else {
        if (a2dp_read_output_audio_config(&out->commons[i], &codec_config,
                                          &codec_capability,
                                          true /* update_stream_config */) < 0) {
          ERROR("a2dp_read_output_audio_config failed");
          ret = -1;
          goto err_open;
        }
      }
      // a2dp_read_output_audio_config() opens the socket control path (or fails)

      /* set output config values */
      if (config != nullptr) {
        // Try to use the config parameters and send it to the remote side
        // TODO: Shall we use out_set_format() and similar?
        if (config->format != 0) out->commons[i].cfg.format = config->format;
        if (config->sample_rate != 0) out->commons[i].cfg.rate = config->sample_rate;
        if (config->channel_mask != 0)
          out->commons[i].cfg.channel_mask = config->channel_mask;
        if ((out->commons[i].cfg.format != 0) || (out->commons[i].cfg.rate != 0) ||
            (out->commons[i].cfg.channel_mask != 0)) {
          if (!IS_AUDIO_DEVICE_OUT_BUS(devices)) {
            if (a2dp_write_output_audio_config(&out->commons[i]) < 0) {
              ERROR("a2dp_write_output_audio_config failed");
              ret = -1;
              goto err_open;
            }
            // Read again and make sure we use the same parameters as the remote side
            if (a2dp_read_output_audio_config(&out->commons[i], &codec_config,
                                              &codec_capability,
                                              true /* update_stream_config */) < 0) {
              ERROR("a2dp_read_output_audio_config failed");
              ret = -1;
              goto err_open;
            }
          }
        }
        config->format = out_get_format((const struct audio_stream*)&out->stream);
        config->sample_rate =
            out_get_sample_rate((const struct audio_stream*)&out->stream);
        config->channel_mask =
            out_get_channels((const struct audio_stream*)&out->stream);

        out->commons[i].in_config = *config;

        INFO(
            "Output stream config: format=0x%x sample_rate=%d channel_mask=0x%x "
            "buffer_sz=%zu",
            config->format, config->sample_rate, config->channel_mask,
            out->commons[i].buffer_sz);
      }
      *stream_out = &out->stream;
      a2dp_dev->output = out;
      INFO("stream %p", out);
      if (devices == AUDIO_DEVICE_OUT_BUS) {
        INFO("stream %p, busaddress= %s", out,address);
        bus_stream_map.emplace(address, out);
        out->commons[i].data = malloc(DEFAULT_BUF_SIZE);
        out->commons[i].data_sz = DEFAULT_BUF_SIZE;
        if (out->commons[i].data == NULL) {
          ERROR("%s: Memory Allocation failed!", __func__);
          out->commons[i].data_sz = 0;
          goto err_open;
        }
      }
  }

  DEBUG("success");
  /* Delay to ensure Headset is in proper state when START is initiated from
   * DUT immediately after the connection due to ongoing music playback. */
  if (!IS_AUDIO_DEVICE_OUT_BUS(devices)) {
    usleep(250000);
  }
  return 0;

err_open:
  for (int i = 0; i < COMMON_ARRAY_SIZE; i++) {
    a2dp_stream_common_destroy(&out->commons[i]);
  }
  free(out);
  *stream_out = NULL;
  a2dp_dev->output = NULL;
  ERROR("failed");
  return ret;
}

static void adev_close_output_stream(struct audio_hw_device* dev,
                                     struct audio_stream_out* stream) {
  struct a2dp_audio_device* a2dp_dev = (struct a2dp_audio_device*)dev;
  struct a2dp_stream_out* out = (struct a2dp_stream_out*)stream;

  INFO("stream %p", stream);
  for (int i = 0; i < COMMON_ARRAY_SIZE; i++) {
      INFO("%s: close all commons", __func__);
      INFO("%s: state %d", __func__, out->commons[i].state);

      // prevent interference with adev_set_parameters.
      std::lock_guard<std::recursive_mutex> lock(*a2dp_dev->mutex);
      {
        std::lock_guard<std::recursive_mutex> lock(*out->commons[i].mutex);
        const a2dp_state_t state = out->commons[i].state;
        INFO("closing output (state %d)", (int)state);
        if ((state == AUDIO_A2DP_STATE_STARTED) ||
            (state == AUDIO_A2DP_STATE_STOPPING)) {
          stop_audio_datapath(&out->commons[i]);
        }

        skt_disconnect(out->commons[i].ctrl_fd);
        out->commons[i].ctrl_fd = AUDIO_SKT_DISCONNECTED;
      }

      a2dp_stream_common_destroy(&out->commons[i]);
  }
  free(stream);
  a2dp_dev->output = NULL;

  DEBUG("done");
}

static int adev_set_parameters(struct audio_hw_device* dev,
                               const char* kvpairs) {
  struct a2dp_audio_device* a2dp_dev = (struct a2dp_audio_device*)dev;
  int retval = 0;

  // prevent interference with adev_close_output_stream
  std::lock_guard<std::recursive_mutex> lock(*a2dp_dev->mutex);
  struct a2dp_stream_out* out = a2dp_dev->output;

  std::unordered_map<std::string, std::string> params =
    hash_map_utils_new_from_string_params(kvpairs);
  if (!params.empty()) {
    for (auto it : bus_stream_map) {
        out = it.second;
        if (out) {
            for (int i = 0; i < COMMON_ARRAY_SIZE; i++) {
            VERBOSE("state %d", out->commons[i].state);
            }
            retval = out->stream.common.set_parameters((struct audio_stream*)out,
                                                        kvpairs);
        }
    }
  }

  return retval;
}

static char* adev_get_parameters(UNUSED_ATTR const struct audio_hw_device* dev,
                                 const char* keys) {
  FNLOG();

  std::unordered_map<std::string, std::string> params =
      hash_map_utils_new_from_string_params(keys);
  // hash_map_utils_dump_string_keys_string_values(params);

  if (!params.empty()) {
    std::string address = params["bdaddress"];
    INFO("address %s", address.c_str());
    if (address != "") {
      if (device_stream_map.find(address) != device_stream_map.end()) {
        struct a2dp_stream_out* out = device_stream_map.find(address)->second;
        if (out != nullptr) {
          INFO("found bus: %s",  out->commons[0].bus_address.c_str());
          return strdup(out->commons[0].bus_address.c_str());
        }
      }
    }
  }
  return strdup("");
}

static int adev_init_check(UNUSED_ATTR const struct audio_hw_device* dev) {
  FNLOG();

  return 0;
}

static int adev_set_voice_volume(UNUSED_ATTR struct audio_hw_device* dev,
                                 UNUSED_ATTR float volume) {
  FNLOG();

  return -ENOSYS;
}

static int adev_set_master_volume(UNUSED_ATTR struct audio_hw_device* dev,
                                  UNUSED_ATTR float volume) {
  FNLOG();

  return -ENOSYS;
}

static int adev_set_mode(UNUSED_ATTR struct audio_hw_device* dev,
                         UNUSED_ATTR audio_mode_t mode) {
  FNLOG();

  return 0;
}

static int adev_set_mic_mute(UNUSED_ATTR struct audio_hw_device* dev,
                             UNUSED_ATTR bool state) {
  FNLOG();

  return -ENOSYS;
}

static int adev_get_mic_mute(UNUSED_ATTR const struct audio_hw_device* dev,
                             UNUSED_ATTR bool* state) {
  FNLOG();

  return -ENOSYS;
}

static size_t adev_get_input_buffer_size(
    UNUSED_ATTR const struct audio_hw_device* dev,
    UNUSED_ATTR const struct audio_config* config) {
  FNLOG();

  return 320;
}

static int adev_open_input_stream(struct audio_hw_device* dev,
                                  UNUSED_ATTR audio_io_handle_t handle,
                                  UNUSED_ATTR audio_devices_t devices,
                                  UNUSED_ATTR struct audio_config* config,
                                  struct audio_stream_in** stream_in,
                                  UNUSED_ATTR audio_input_flags_t flags,
                                  UNUSED_ATTR const char* address,
                                  UNUSED_ATTR audio_source_t source) {
  struct a2dp_audio_device* a2dp_dev = (struct a2dp_audio_device*)dev;
  struct a2dp_stream_in* in;
  int ret;

  FNLOG();

  // protect against adev->input and stream_in from being inconsistent
  std::lock_guard<std::recursive_mutex> lock(*a2dp_dev->mutex);
  in = (struct a2dp_stream_in*)calloc(1, sizeof(struct a2dp_stream_in));

  if (!in) return -ENOMEM;

  in->stream.common.get_sample_rate = in_get_sample_rate;
  in->stream.common.set_sample_rate = in_set_sample_rate;
  in->stream.common.get_buffer_size = in_get_buffer_size;
  in->stream.common.get_channels = in_get_channels;
  in->stream.common.get_format = in_get_format;
  in->stream.common.set_format = in_set_format;
  in->stream.common.standby = in_standby;
  in->stream.common.dump = in_dump;
  in->stream.common.set_parameters = in_set_parameters;
  in->stream.common.get_parameters = in_get_parameters;
  in->stream.common.add_audio_effect = in_add_audio_effect;
  in->stream.common.remove_audio_effect = in_remove_audio_effect;
  in->stream.set_gain = in_set_gain;
  in->stream.read = in_read;
  in->stream.get_input_frames_lost = in_get_input_frames_lost;

  /* initialize a2dp specifics */
  a2dp_stream_common_init(&in->common);

  *stream_in = &in->stream;
  a2dp_dev->input = in;

  if (a2dp_read_input_audio_config(&in->common) < 0) {
    ERROR("a2dp_read_input_audio_config failed (%s)", strerror(errno));
    ret = -1;
    goto err_open;
  }
  // a2dp_read_input_audio_config() opens socket control path (or fails)

  DEBUG("success");
  return 0;

err_open:
  a2dp_stream_common_destroy(&in->common);
  free(in);
  *stream_in = NULL;
  a2dp_dev->input = NULL;
  ERROR("failed");
  return ret;
}

static void adev_close_input_stream(struct audio_hw_device* dev,
                                    struct audio_stream_in* stream) {
  struct a2dp_audio_device* a2dp_dev = (struct a2dp_audio_device*)dev;
  struct a2dp_stream_in* in = (struct a2dp_stream_in*)stream;

  std::lock_guard<std::recursive_mutex> lock(*a2dp_dev->mutex);
  {
    std::lock_guard<std::recursive_mutex> lock(*in->common.mutex);
    const a2dp_state_t state = in->common.state;
    INFO("closing input (state %d)", (int)state);

    if ((state == AUDIO_A2DP_STATE_STARTED) ||
        (state == AUDIO_A2DP_STATE_STOPPING))
      stop_audio_datapath(&in->common);

    skt_disconnect(in->common.ctrl_fd);
    in->common.ctrl_fd = AUDIO_SKT_DISCONNECTED;
  }
  a2dp_stream_common_destroy(&in->common);
  free(stream);
  a2dp_dev->input = NULL;

  DEBUG("done");
}

static int adev_dump(UNUSED_ATTR const audio_hw_device_t* device,
                     UNUSED_ATTR int fd) {
  FNLOG();

  return 0;
}

static int adev_close(hw_device_t* device) {
  struct a2dp_audio_device* a2dp_dev = (struct a2dp_audio_device*)device;
  FNLOG();

  delete a2dp_dev->mutex;
  a2dp_dev->mutex = nullptr;
  free(device);
  return 0;
}

static int adev_open(const hw_module_t* module, const char* name,
                     hw_device_t** device) {
  struct a2dp_audio_device* adev;

  INFO(" adev_open in a2dp_audio_hw module.");
  FNLOG();

  if (strcmp(name, AUDIO_HARDWARE_INTERFACE) != 0) {
    ERROR("interface %s not matching [%s]", name, AUDIO_HARDWARE_INTERFACE);
    return -EINVAL;
  }

  adev = (struct a2dp_audio_device*)calloc(1, sizeof(struct a2dp_audio_device));

  if (!adev) return -ENOMEM;

  adev->mutex = new std::recursive_mutex;

  adev->device.common.tag = HARDWARE_DEVICE_TAG;
  adev->device.common.version = AUDIO_DEVICE_API_VERSION_2_0;
  adev->device.common.module = (struct hw_module_t*)module;
  adev->device.common.close = adev_close;

  adev->device.init_check = adev_init_check;
  adev->device.set_voice_volume = adev_set_voice_volume;
  adev->device.set_master_volume = adev_set_master_volume;
  adev->device.set_mode = adev_set_mode;
  adev->device.set_mic_mute = adev_set_mic_mute;
  adev->device.get_mic_mute = adev_get_mic_mute;
  adev->device.set_parameters = adev_set_parameters;
  adev->device.get_parameters = adev_get_parameters;
  adev->device.get_input_buffer_size = adev_get_input_buffer_size;
  adev->device.open_output_stream = adev_open_output_stream;
  adev->device.close_output_stream = adev_close_output_stream;
  adev->device.open_input_stream = adev_open_input_stream;
  adev->device.close_input_stream = adev_close_input_stream;
  adev->device.dump = adev_dump;

  adev->output = NULL;

  *device = &adev->device.common;

  return 0;
}

static struct hw_module_methods_t hal_module_methods = {
    .open = adev_open,
};

__attribute__((
    visibility("default"))) struct audio_module HAL_MODULE_INFO_SYM = {
    .common =
        {
            .tag = HARDWARE_MODULE_TAG,
            .version_major = 1,
            .version_minor = 0,
            .id = AUDIO_HARDWARE_MODULE_ID,
            .name = "A2DP Audio HW HAL",
            .author = "The Android Open Source Project",
            .methods = &hal_module_methods,
        },
};
