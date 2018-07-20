/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

// An parser for decoding binary messages from a Nmea receiver. The following
// messages must be
// logged in order for this parser to work properly.
//
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "ros/include/ros/ros.h"

#include "modules/common/log.h"
#include "modules/drivers/gnss/parser/parser.h"
#include "modules/drivers/gnss/parser/rtcm_decode.h"
#include "modules/drivers/gnss/proto/gnss.pb.h"
#include "modules/drivers/gnss/proto/heading.pb.h"
#include "modules/drivers/gnss/util/time_conversion.h"

namespace apollo {
namespace drivers {
namespace gnss {

// Anonymous namespace that contains helper constants and functions.
namespace {

constexpr size_t BUFFER_SIZE = 256;

constexpr int SECONDS_PER_WEEK = 60 * 60 * 24 * 7;

constexpr double DEG_TO_RAD = M_PI / 180.0;

constexpr float FLOAT_NAN = std::numeric_limits<float>::quiet_NaN();


template <typename T>
constexpr bool is_zero(T value) {
  return value == static_cast<T>(0);
}

// Converts Nmea's azimuth (north = 0, east = 90) to FLU yaw (east = 0, north
// = pi/2).
constexpr double azimuth_deg_to_yaw_rad(double azimuth) {
  return (90.0 - azimuth) * DEG_TO_RAD;
}

// A helper that fills an Point3D object (which uses the FLU frame) using RFU
// measurements.
inline void rfu_to_flu(double r, double f, double u,
                       ::apollo::common::Point3D* flu) {
  flu->set_x(f);
  flu->set_y(-r);
  flu->set_z(u);
}l;

}  // namespace

class NmeaParser : public Parser {
 public:
  NmeaParser();
  explicit NmeaParser(const config::Config& config);

  virtual MessageType GetMessage(MessagePtr* message_ptr);

 private:

  Parser::MessageType PrepareMessage(MessagePtr* message_ptr);

  std::vector<uint8_t> buffer_;

  size_t header_length_ = 0;

  size_t total_length_ = 0;

  ::apollo::drivers::gnss::GnssBestPose bestpos_;
  ::apollo::drivers::gnss::Heading heading_;
};

Parser* Parser::CreateNmea(const config::Config& config) {
  return new NmeaParser(config);
}

NmeaParser::NmeaParser() {
  buffer_.reserve(BUFFER_SIZE);
}

NmeaParser::NmeaParser(const config::Config& config) {
  buffer_.reserve(BUFFER_SIZE);
}

Parser::MessageType NmeaParser::GetMessage(MessagePtr* message_ptr) {
  if (data_ == nullptr) {
    return MessageType::NONE;
  }

  MessageType type = PrepareMessage(message_ptr);
  if (type != MessageType::NONE) {
    return type;
  }

  return MessageType::NONE;
}

Parser::MessageType NmeaParser::PrepareMessage(MessagePtr* message_ptr) {

  uint8_t* message = nullptr;
  Nmea::MessageId message_id; //нужно определить свои типы сообщений

  //Получить из data_ тип сообщения

  switch (message_id) {
    case Nmea::GPGGA:

        //получить данные из data_

        bestpos_.set_latitude(latitude);
        bestpos_.set_longitude(longitude);
        bestpos_.set_height_msl(height_msl);

        *message_ptr = &bestpos_;
        return MessageType::BEST_GNSS_POS;

      break;

   
    case Nmea::HEADING:

        //получить данные из data_

        heading_.set_baseline_length(heading->length);
        heading_.set_heading(heading->heading);
        heading_.set_pitch(heading->pitch);

        *message_ptr = &heading_;
        return MessageType::HEADING;

      break;

    default:
      break;
  }
  return MessageType::NONE;
}


}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
