// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from auv_msgs:msg/BehaviorStatus.idl
// generated code does not contain a copyright notice

#include "auv_msgs/msg/detail/behavior_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_auv_msgs
const rosidl_type_hash_t *
auv_msgs__msg__BehaviorStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x26, 0x96, 0x47, 0x7d, 0x52, 0xcc, 0x65, 0xd4,
      0x6b, 0xfe, 0x1a, 0x66, 0xc8, 0xd9, 0xb1, 0x50,
      0x18, 0xc0, 0x0f, 0x91, 0xf4, 0x48, 0xb4, 0x77,
      0xec, 0x36, 0x1f, 0xae, 0xfa, 0xb9, 0xbf, 0x77,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
#endif

static char auv_msgs__msg__BehaviorStatus__TYPE_NAME[] = "auv_msgs/msg/BehaviorStatus";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";

// Define type names, field names, and default values
static char auv_msgs__msg__BehaviorStatus__FIELD_NAME__stamp[] = "stamp";
static char auv_msgs__msg__BehaviorStatus__FIELD_NAME__action_name[] = "action_name";
static char auv_msgs__msg__BehaviorStatus__FIELD_NAME__status[] = "status";
static char auv_msgs__msg__BehaviorStatus__FIELD_NAME__reason[] = "reason";

static rosidl_runtime_c__type_description__Field auv_msgs__msg__BehaviorStatus__FIELDS[] = {
  {
    {auv_msgs__msg__BehaviorStatus__FIELD_NAME__stamp, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {auv_msgs__msg__BehaviorStatus__FIELD_NAME__action_name, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {auv_msgs__msg__BehaviorStatus__FIELD_NAME__status, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {auv_msgs__msg__BehaviorStatus__FIELD_NAME__reason, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription auv_msgs__msg__BehaviorStatus__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
auv_msgs__msg__BehaviorStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {auv_msgs__msg__BehaviorStatus__TYPE_NAME, 27, 27},
      {auv_msgs__msg__BehaviorStatus__FIELDS, 4, 4},
    },
    {auv_msgs__msg__BehaviorStatus__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "builtin_interfaces/Time stamp\n"
  "string action_name\n"
  "string status\n"
  "string reason";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
auv_msgs__msg__BehaviorStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {auv_msgs__msg__BehaviorStatus__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 77, 77},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
auv_msgs__msg__BehaviorStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *auv_msgs__msg__BehaviorStatus__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
