#ifndef SPLSTANDARDMESSAGE_H
#define SPLSTANDARDMESSAGE_H

#include <stdint.h>

#define SPL_STANDARD_MESSAGE_STRUCT_HEADER  "SPL "
#define SPL_STANDARD_MESSAGE_STRUCT_VERSION 6
#define SPL_STANDARD_MESSAGE_DATA_SIZE      780
#define SPL_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS 5

/*
 Important remarks about units:

 For each parameter, the respective comments describe its unit.
 The following units are used:

   - Distances:  Millimeters (mm)
   - Angles:     Radian
   - Time:       Seconds (s)
   - Speed:      Millimeters per second (mm/s)
*/


struct spl_standard_message
{
  char header[4];        // "SPL "
  uint8_t version;       // has to be set to SPL_STANDARD_MESSAGE_STRUCT_VERSION
  int8_t player_num;      // [MANDATORY FIELD] 1-5 in drop-in games, 1-6 in normal games
  int8_t team_num;        // [MANDATORY FIELD] the number of the team (as provided by the organizers)
  int8_t fallen;         // [MANDATORY FIELD] 1 means that the robot is fallen, 0 means that the robot can play

  // [MANDATORY FIELD]
  // position and orientation of robot
  // coordinates in millimeters
  // 0,0 is in center of field
  // +ve x-axis points towards the goal we are attempting to score on
  // +ve y-axis is 90 degrees counter clockwise from the +ve x-axis
  // angle in radians, 0 along the +x axis, increasing counter clockwise
  float pose[3];      // x,y,theta

  // [MANDATORY FIELD]
  // the robot's target position on the field
  // the coordinate system is the same as for the pose
  // if the robot does not have any target, this attribute should be set to the robot's position
  float walking_to[2];

  // [MANDATORY FIELD]
  // the target position of the next shot (either pass or goal shot)
  // the coordinate system is the same as for the pose
  // if the robot does not intend to shoot, this attribute should be set to the robot's position
  float shooting_to[2];

  // ball information
  float ball_age;        // seconds since this robot last saw the ball. -1.f if we haven't seen it

  // position of ball relative to the robot
  // coordinates in millimeters
  // 0,0 is in center of the robot
  // +ve x-axis points forward from the robot
  // +ve y-axis is 90 degrees counter clockwise from the +ve x-axis
  float ball[2];

  // velocity of the ball (same coordinate system as above)
  // the unit is millimeters per second
  float ball_vel[2];

  // describes what - in the robot's opinion - the teammates should do:
  // 0 - nothing particular (default)
  // 1 - play keeper
  // 2 - support defense
  // 3 - support offense
  // 4 - play the ball
  // For each teammate, the corresponding suggestion is put in the element
  // playerNumber(teammate) -1.
  // Example: To suggest a teammate, which has the number 5, to play in defense:
  //          suggestion[4] = 2;
  int8_t suggestion[SPL_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS];

  // [MANDATORY FIELD]
  // describes what the robot intends to do:
  // 0 - nothing particular (default)
  // 1 - wants to be keeper
  // 2 - wants to play defense
  // 3 - wants to play the ball
  // 4 - robot is lost (i.e. cannot decide what to do now, maybe because of disorientation [see confidence fields])
  int8_t intention;

  // [MANDATORY]
  // the average speed that the robot has, for instance, when walking towards the ball
  // the unit is mm/s
  // the idea of this value is to roughly represent the robot's walking skill
  // it has to be set once at the beginning of the game and remains fixed
  int16_t average_walk_speed;

  // [MANDATORY]
  // the maximum distance that the ball rolls after a strong kick by the robot
  // the unit is mm
  // the idea of this value is to roughly represent the robot's kicking skill
  // it has to be set once at the beginning of the game and remains fixed
  int16_t max_kick_distance;

  // [MANDATORY]
  // describes the current confidence of a robot about its self-location,
  // the unit is percent [0,..100]
  // the value should be updated in the course of the game
  int8_t current_position_confidence;

  // [MANDATORY]
  // describes the current confidence of a robot about playing in the right direction,
  // the unit is percent [0,..100]
  // the value should be updated in the course of the game
  int8_t current_side_confidence;

  // number of bytes that is actually used by the data array
  uint16_t num_of_data_bytes;

  // buffer for arbitrary data, teams do not need to send more than specified in num_of_data_bytes
  uint8_t data[SPL_STANDARD_MESSAGE_DATA_SIZE];

#ifdef __cplusplus
  // constructor
  spl_standard_message() :
    version(SPL_STANDARD_MESSAGE_STRUCT_VERSION),
    player_num(-1),
    team_num(-1),
    fallen(-1),
    ball_age(-1.f),
    intention(-1),
    average_walk_speed(-1),
    max_kick_distance(-1),
    current_position_confidence(-1),
    current_side_confidence(-1),
    num_of_data_bytes(0)
  {
    const char* init = SPL_STANDARD_MESSAGE_STRUCT_HEADER;
    for(unsigned int i = 0; i < sizeof(header); ++i)
      header[i] = init[i];
    pose[0] = 0.f;
    pose[1] = 0.f;
    pose[2] = 0.f;
    walking_to[0] = 0.f;
    walking_to[1] = 0.f;
    shooting_to[0] = 0.f;
    shooting_to[1] = 0.f;
    ball[0] = 0.f;
    ball[1] = 0.f;
    ball_vel[0] = 0.f;
    ball_vel[1] = 0.f;
    for(int i = 0; i < SPL_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS; ++i)
      suggestion[i] = -1;
  }
#endif
};

#endif // SPLSTANDARDMESSAGE_H
