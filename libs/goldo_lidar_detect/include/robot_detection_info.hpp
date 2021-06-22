#pragma once

namespace goldobot
{

  typedef struct _lidar_plot_msg {
    unsigned int timestamp_ms;
    short int x_mm;
    short int y_mm;
  } lidar_plot_msg_t;

  typedef struct _robot_detection_msg {
    unsigned int timestamp_ms;
    unsigned int id;
    short int x_mm_X4;
    short int y_mm_X4;
    short int vx_mm_sec;
    short int vy_mm_sec;
    short int ax_mm_sec_2;
    short int ay_mm_sec_2;
    unsigned int detect_quality;
  } robot_detection_msg_t;

  typedef struct _detected_robot_info {
    unsigned int timestamp_ms;
    unsigned int id;
    double x_mm;
    double y_mm;
    double vx_mm_sec;
    double vy_mm_sec;
    double ax_mm_sec_2;
    double ay_mm_sec_2;
    unsigned int detect_quality;
  } detected_robot_info_t;

}
