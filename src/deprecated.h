#pragma once

#include <gantry_robot/Position.h>
#include <gantry_robot/Homing.h>

bool servicePositionCallback(gantry_robot::Position::Request &req, gantry_robot::Position::Response &res);
bool serviceHomingCallback(gantry_robot::Homing::Request &req, gantry_robot::Homing::Response &res);