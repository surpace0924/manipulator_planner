#pragma once

template <typename T>
struct BackhoeConfig
{
  T boom_offset_x;
  T boom_offset_y;
  T boom_offset_z;
  T boom_length;
  T arm_length;
  T bucket_length;
  T tip_width;
};
