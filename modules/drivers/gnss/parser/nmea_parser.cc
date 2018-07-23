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
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/drivers/gnss/proto/heading.pb.h"
#include "modules/drivers/gnss/util/time_conversion.h"

namespace apollo {
namespace drivers {
namespace gnss {

namespace nmea {

  enum MessageId : uint16_t {
    NONE = 0,
    GPGGA = 1,
    HEADING = 2
  };
}

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

}  // namespace

#define MINUTE_LENGTH 10 // Length of minutes data in NMEA

class NmeaParser : public Parser {
 public:
  NmeaParser();
  explicit NmeaParser(const config::Config& config);

  virtual MessageType GetMessage(MessagePtr* message_ptr);

 private:

  Parser::MessageType PrepareMessage(MessagePtr* message_ptr);
  std::vector<std::string> splitStringByComma(std::string input);
  double getCoordinates(std::string array);
  double stringToDouble(std::string inputString);
  double degreesToDecimal(int degrees, double minutes, int seconds );

  std::string buffer_;

  double last_heading_;

  ::apollo::drivers::gnss::Ins ins_;
  ::apollo::drivers::gnss::Heading heading_;
};

Parser* Parser::CreateNmea(const config::Config& config) {
  return new NmeaParser(config);
}

NmeaParser::NmeaParser() {

}

NmeaParser::NmeaParser(const config::Config& config) {

}

Parser::MessageType NmeaParser::GetMessage(MessagePtr* message_ptr) {
  if (data_ == nullptr) {
    return MessageType::NONE;
  }

  while (data_ < data_end_) {

    if (*data_ == '\n')
    {
      ++data_;
      AINFO << "Buffer: " << buffer_;
      MessageType type = PrepareMessage(message_ptr);
      if (type != MessageType::NONE) {
        return type;
      }
    }
    if (*data_ == '$') {
      buffer_.clear();
    }
    ++data_;
    buffer_ +=*data_;

  }
  return MessageType::NONE;
}

Parser::MessageType NmeaParser::PrepareMessage(MessagePtr* message_ptr) {

  uint8_t* message = nullptr;
  nmea::MessageId message_id = nmea::NONE; //нужно определить свои типы сообщений

  std::vector<std::string> elementVector = splitStringByComma(buffer_);
  AINFO << "Message type: " << elementVector[0];
  if (elementVector[0] == "GPGGA"){
    message_id = nmea::GPGGA;
  } else if (elementVector[0] == "PNVGBLS"){
    message_id = nmea::HEADING;
  }

  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  double seconds = 0.0;

  double baseline = 0.0;
  double pitch = 0.0;

  AINFO << "Message ID: " << message_id;

  switch (message_id) {
    case nmea::GPGGA:

        seconds = ros::Time::now().toSec();
        ins_.set_measurement_time(seconds);
        ins_.mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
        
        latitude            = getCoordinates(elementVector[2]);
        if (elementVector[3] == "S") latitude  = -latitude;
        longitude           = getCoordinates(elementVector[4]);
        if (elementVector[5] == "W") longitude  = -longitude;
        altitude            = stringToDouble(elementVector[9]);

        ins_.mutable_position()->set_lon(latitude);
        ins_.mutable_position()->set_lat(longitude);
        ins_.mutable_position()->set_height(altitude);
        ins_.mutable_euler_angles()->set_x(0.0);
        ins_.mutable_euler_angles()->set_y(0.0);
        ins_.mutable_euler_angles()->set_z(last_heading_ * M_PI / 180.0);
        ins_.mutable_linear_velocity()->set_x(0.0);
        ins_.mutable_linear_velocity()->set_y(0.0);
        ins_.mutable_linear_velocity()->set_z(0.0);

        AINFO << "Lat: " << latitude << " Long: " << longitude << " Alt: " << altitude;

        *message_ptr = &ins_;
        return MessageType::INS;

      break;

   
    case nmea::HEADING:

        //получить данные из buffer_
        baseline            = stringToDouble(elementVector[5]);
        last_heading_            = stringToDouble(elementVector[6]);
        pitch            = stringToDouble(elementVector[7]);

        heading_.set_baseline_length(baseline);
        heading_.set_heading(last_heading_);
        heading_.set_pitch(pitch);

        AINFO << "Baseline: " << baseline << " Heading: " << last_heading_ << " Pitch: " << pitch;

        *message_ptr = &heading_;
        return MessageType::HEADING;

      break;

    default:
      break;
  }
  return MessageType::UNKNOWN;
}

std::vector<std::string> NmeaParser::splitStringByComma(std::string input){

    std::vector<std::string>  returnVector;
    std::stringstream    ss(input);
    std::string          element;

    while(std::getline(ss, element, ',')) {
        returnVector.push_back(element);
    }


    return returnVector;
}

double NmeaParser::degreesToDecimal(int degrees, double minutes, int seconds )
{
    double returnDouble = 0;

    returnDouble = degrees + minutes/60 + seconds/3600.0f;

    return returnDouble;
}

double NmeaParser::stringToDouble(std::string inputString){

    //If string empty, return 0.
    double returnValue = 0;
    std::istringstream istr(inputString);

    istr >> returnValue;

    return (returnValue);

}
double NmeaParser::getCoordinates(std::string array){

    double decimalDegrees = 0;
    std::string degreeArray;
    std::string minuteArray;

    // Convert input array into two sub arrays containing the degrees and the minutes
    // Check for correct array length
    if (array.length() > MINUTE_LENGTH){

        degreeArray.assign(array.begin(), array.end() - MINUTE_LENGTH);
        minuteArray.assign(array.end() - MINUTE_LENGTH, array.end());

        // Convert strings into numbers
        int degrees;
        double minutes;
        degrees = std::atoi(degreeArray.c_str());
        minutes = stringToDouble(minuteArray);

        // Convert degrees and mintues into decimal
        decimalDegrees = degreesToDecimal(degrees,minutes, 0);

    }
    return decimalDegrees;

}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
