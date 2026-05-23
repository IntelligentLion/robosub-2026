// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from auv_msgs:msg/ObjectDetection.idl
// generated code does not contain a copyright notice

#include "auv_msgs/msg/detail/object_detection__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_auv_msgs
const rosidl_type_hash_t *
auv_msgs__msg__ObjectDetection__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x2e, 0x2f, 0x55, 0xb0, 0x30, 0x15, 0x0d, 0x52,
      0xd4, 0x6d, 0x51, 0x2a, 0xf4, 0x5a, 0x79, 0x32,
      0xe5, 0x28, 0x95, 0xd5, 0x34, 0x24, 0xee, 0x67,
      0x0c, 0xf7, 0x12, 0x7b, 0x57, 0x38, 0xce, 0xcb,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "geometry_msgs/msg/detail/point__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t geometry_msgs__msg__Point__EXPECTED_HASH = {1, {
    0x69, 0x63, 0x08, 0x48, 0x42, 0xa9, 0xb0, 0x44,
    0x94, 0xd6, 0xb2, 0x94, 0x1d, 0x11, 0x44, 0x47,
    0x08, 0xd8, 0x92, 0xda, 0x2f, 0x4b, 0x09, 0x84,
    0x3b, 0x9c, 0x43, 0xf4, 0x2a, 0x7f, 0x68, 0x81,
  }};
#endif

static char auv_msgs__msg__ObjectDetection__TYPE_NAME[] = "auv_msgs/msg/ObjectDetection";
static char geometry_msgs__msg__Point__TYPE_NAME[] = "geometry_msgs/msg/Point";

// Define type names, field names, and default values
static char auv_msgs__msg__ObjectDetection__FIELD_NAME__label[] = "label";
static char auv_msgs__msg__ObjectDetection__FIELD_NAME__confidence[] = "confidence";
static char auv_msgs__msg__ObjectDetection__FIELD_NAME__position[] = "position";
static char auv_msgs__msg__ObjectDetection__FIELD_NAME__bbox_width[] = "bbox_width";
static char auv_msgs__msg__ObjectDetection__FIELD_NAME__bbox_height[] = "bbox_height";

static rosidl_runtime_c__type_description__Field auv_msgs__msg__ObjectDetection__FIELDS[] = {
  {
    {auv_msgs__msg__ObjectDetection__FIELD_NAME__label, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {auv_msgs__msg__ObjectDetection__FIELD_NAME__confidence, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {auv_msgs__msg__ObjectDetection__FIELD_NAME__position, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    },
    {NULL, 0, 0},
  },
  {
    {auv_msgs__msg__ObjectDetection__FIELD_NAME__bbox_width, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {auv_msgs__msg__ObjectDetection__FIELD_NAME__bbox_height, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription auv_msgs__msg__ObjectDetection__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
auv_msgs__msg__ObjectDetection__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {auv_msgs__msg__ObjectDetection__TYPE_NAME, 28, 28},
      {auv_msgs__msg__ObjectDetection__FIELDS, 5, 5},
    },
    {auv_msgs__msg__ObjectDetection__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string label\n"
  "float32 confidence\n"
  "geometry_msgs/Point position\n"
  "float32 bbox_width\n"
  "float32 bbox_height";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
auv_msgs__msg__ObjectDetection__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {auv_msgs__msg__ObjectDetection__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 100, 100},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
auv_msgs__msg__ObjectDetection__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *auv_msgs__msg__ObjectDetection__get_individual_type_description_source(NULL),
    sources[1] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
