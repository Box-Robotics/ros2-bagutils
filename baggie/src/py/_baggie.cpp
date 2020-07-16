/*
 * Copyright (C) 2020 Box Robotics, Inc
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
 */

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/chrono.h>

#include <rcutils/types.h>
#include <rcutils/time.h>
#include <rosbag2_compression/compression_options.hpp>
#include <rosbag2_compression/sequential_compression_reader.hpp>
#include <rosbag2_compression/sequential_compression_writer.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/info.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/storage_options.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/bag_metadata.hpp>
#include <rosbag2_storage/ros_helper.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

namespace bag = rosbag2_cpp;
namespace bagcomp = rosbag2_compression;
namespace py = pybind11;

namespace baggie
{
  class Reader
  {
  public:
    explicit Reader(const bagcomp::CompressionOptions & c_opt)
    {
      if (c_opt.compression_mode == bagcomp::CompressionMode::NONE)
        {
          this->reader_ =
            std::make_unique<bag::Reader>(
              std::make_unique<bag::readers::SequentialReader>());
        }
      else
        {
          this->reader_ =
            std::make_unique<bag::Reader>(
              std::make_unique<bagcomp::SequentialCompressionReader>());
        }
    }

    void open(bag::StorageOptions & storage_opts,
              bag::ConverterOptions & converter_opts)
    {
      this->reader_->open(storage_opts, converter_opts);
    }

    bool has_next()
    {
      return this->reader_->has_next();
    }

    py::tuple read_next()
    {
      const auto next = this->reader_->read_next();
      rcutils_uint8_array_t rc_data = *next->serialized_data.get();
      std::string ser_data(
        rc_data.buffer, rc_data.buffer + rc_data.buffer_length);
      return py::make_tuple(
        next->topic_name, py::bytes(ser_data), next->time_stamp);
    }

    std::vector<rosbag2_storage::TopicMetadata>
    get_all_topics_and_types()
    {
      return this->reader_->get_all_topics_and_types();
    }

    void set_filter(const rosbag2_storage::StorageFilter & filt)
    {
      this->reader_->set_filter(filt);
    }

    void reset_filter()
    {
      this->reader_->reset_filter();
    }

  private:
    std::unique_ptr<bag::Reader> reader_;

  }; // end: class Reader


  class Writer
  {
  public:
    explicit Writer(const bagcomp::CompressionOptions & c_opt)
    {
      if (c_opt.compression_mode == bagcomp::CompressionMode::NONE)
        {
          this->writer_ =
            std::make_unique<bag::Writer>(
              std::make_unique<bag::writers::SequentialWriter>());
        }
      else
        {
          this->writer_ =
            std::make_unique<bag::Writer>(
              std::make_unique<bagcomp::SequentialCompressionWriter>(c_opt));
        }
    }

    void open(bag::StorageOptions & storage_opts,
              bag::ConverterOptions & converter_opts)
    {
      this->writer_->open(storage_opts, converter_opts);
    }

    void create_topic(const rosbag2_storage::TopicMetadata & meta)
    {
      this->writer_->create_topic(meta);
    }

    void remove_topic(const rosbag2_storage::TopicMetadata & meta)
    {
      this->writer_->remove_topic(meta);
    }

    void write(const std::string & topic_name,
               const std::string & message,
               const rcutils_time_point_value_t & time_stamp)
    {
      auto bag_message =
        std::make_shared<rosbag2_storage::SerializedBagMessage>();

      bag_message->topic_name = topic_name;
      bag_message->serialized_data =
        rosbag2_storage::make_serialized_message(
          message.c_str(), message.length());
      bag_message->time_stamp = time_stamp;

      this->writer_->write(bag_message);
    }

  private:
    std::unique_ptr<bag::Writer> writer_;

  }; // end: class Writer

} // end: namespace baggie

PYBIND11_MODULE(_baggie, m)
{
  m.doc() = "Python wrapper around the ROS2 C++ bag API";

  py::enum_<bagcomp::CompressionMode>(m, "CompressionMode")
    .value("NONE", bagcomp::CompressionMode::NONE)
    .value("FILE", bagcomp::CompressionMode::FILE)
    // @todo As of this writing, only "FILE" compression is supported in C++
    //.value("MESSAGE", bagcomp::CompressionMode::MESSAGE)
    //.value("LAST_MODE", bagcomp::CompressionMode::LAST_MODE)
    .export_values();

  py::class_<bagcomp::CompressionOptions>(m, "CompressionOptions")
    .def(py::init())
    .def("mode_to_string",
         [](const bagcomp::CompressionOptions & c) -> std::string
         {
           return bagcomp::compression_mode_to_string(c.compression_mode);
         })
    .def_readwrite(
      "compression_format", &bagcomp::CompressionOptions::compression_format)
    .def_readwrite(
      "compression_mode", &bagcomp::CompressionOptions::compression_mode);

  py::class_<baggie::Reader>(m, "Reader")
    .def(py::init<const bagcomp::CompressionOptions &>())
    .def("open", &baggie::Reader::open)
    .def("has_next", &baggie::Reader::has_next)
    .def("read_next", &baggie::Reader::read_next)
    .def("get_all_topics_and_types", &baggie::Reader::get_all_topics_and_types)
    .def("set_filter", &baggie::Reader::set_filter)
    .def("reset_filter", &baggie::Reader::reset_filter);

   py::class_<baggie::Writer>(m, "Writer")
     .def(py::init<const bagcomp::CompressionOptions &>())
     .def("open", &baggie::Writer::open)
     .def("create_topic", &baggie::Writer::create_topic)
     .def("remove_topic", &baggie::Writer::remove_topic)
     .def("write", &baggie::Writer::write);

  py::class_<bag::ConverterOptions>(m, "ConverterOptions")
     .def(py::init())
     .def_readwrite(
       "input_serialization_format",
       &bag::ConverterOptions::input_serialization_format)
     .def_readwrite(
       "output_serialization_format",
       &bag::ConverterOptions::output_serialization_format);

  py::class_<bag::StorageOptions>(m, "StorageOptions")
    .def(py::init())
    .def_readwrite("uri", &bag::StorageOptions::uri)
    .def_readwrite("storage_id", &bag::StorageOptions::storage_id)
    .def_readwrite("max_bagfile_size", &bag::StorageOptions::max_bagfile_size);

  py::class_<rosbag2_storage::StorageFilter>(m, "StorageFilter")
    .def(py::init())
    .def_readwrite("topics", &rosbag2_storage::StorageFilter::topics);

  py::class_<rosbag2_storage::TopicMetadata>(m, "TopicMetadata")
    .def(py::init())
    .def_readwrite("name", &rosbag2_storage::TopicMetadata::name)
    .def_readwrite("type", &rosbag2_storage::TopicMetadata::type)
    .def_readwrite(
      "serialization_format",
      &rosbag2_storage::TopicMetadata::serialization_format)
    .def_readwrite(
      "offered_qos_profiles",
      &rosbag2_storage::TopicMetadata::offered_qos_profiles)
    .def("equals", &rosbag2_storage::TopicMetadata::operator==);

  py::class_<rosbag2_storage::TopicInformation>(m, "TopicInformation")
    .def(py::init())
    .def_readwrite(
      "topic_metadata",
      &rosbag2_storage::TopicInformation::topic_metadata)
    .def_readwrite(
      "message_count",
      &rosbag2_storage::TopicInformation::message_count);

  //
  // NOTE: Pybind11 converts the chrono types to datetime.*
  // which only have usec resolution. So, we bind the lambdas below
  // so we get full nano resolution and convert to rclpy.time.* types in the
  // Python layer of baggie.
  //
  py::class_<rosbag2_storage::BagMetadata>(m, "BagMetadata")
    .def(py::init())
    .def_readwrite("version", &rosbag2_storage::BagMetadata::version)
    .def_readwrite(
      "storage_identifier", &rosbag2_storage::BagMetadata::storage_identifier)
    .def_readwrite(
      "relative_file_paths", &rosbag2_storage::BagMetadata::relative_file_paths)
    .def_readwrite("duration", &rosbag2_storage::BagMetadata::duration)
    .def("duration_as_nanos", [](const rosbag2_storage::BagMetadata & meta)
                            {
                              return meta.duration.count();
                            })
    .def_readwrite(
      "starting_time", &rosbag2_storage::BagMetadata::starting_time)
    .def("starting_time_as_nanos",
         [](const rosbag2_storage::BagMetadata & meta)
         {
           return meta.starting_time.time_since_epoch().count();
         })
    .def_readwrite(
       "message_count", &rosbag2_storage::BagMetadata::message_count)
    .def_readwrite(
       "topics_with_message_count",
       &rosbag2_storage::BagMetadata::topics_with_message_count)
    .def_readwrite(
       "compression_format", &rosbag2_storage::BagMetadata::compression_format)
    .def_readwrite(
       "compression_mode", &rosbag2_storage::BagMetadata::compression_mode);

  py::class_<bag::Info>(m, "BagInfo")
    .def(py::init())
    .def("read_metadata", &bag::Info::read_metadata);
}
