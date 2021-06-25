#ifdef WIN32
#include <winsock2.h>
#include <windows.h>
#endif

#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <unistd.h>
#include <zmq.h>
#include <math.h>
#include <pthread.h>
#include <iostream>

#include "lidar_detect.hpp"

using namespace goldobot;

#ifndef M_PI
#define M_PI 3.141592653589793
#endif

LidarDetect LidarDetect::s_instance;

LidarDetect& LidarDetect::instance()
{
	return s_instance;
}

LidarDetect::LidarDetect()
{
  m_cur_ts_ms = 0;

  m_quality_threshold = 1;

  m_nb_of_send_detect = MAX_NB_OF_DETECTED_ROBOTS;

  m_detect_lock = false;

  m_beacon_zone_cnt = 0;

  m_rplidar_plot_lifetime_ms = 300;
}

int LidarDetect::init()
{
  m_cur_ts_ms = 0;

  m_quality_threshold = 1;

  m_nb_of_send_detect = MAX_NB_OF_DETECTED_ROBOTS;

  for (int i=0; i<MAX_NB_OF_DETECTED_ROBOTS; i++)
  {
    m_detect_t_0[i].detect_quality     = 0;
    m_detect_t_0[i].timestamp_ms       = 0;
    m_detect_t_0[i].id                 = i;
    m_detect_t_0[i].x_mm               = 0.0;
    m_detect_t_0[i].y_mm               = 0.0;
    m_detect_t_0[i].vx_mm_sec          = 0.0;
    m_detect_t_0[i].vy_mm_sec          = 0.0;
    m_detect_t_0[i].ax_mm_sec_2        = 0.0;
    m_detect_t_0[i].ay_mm_sec_2        = 0.0;

    m_detect_t_1[i].detect_quality     = 0;
    m_detect_t_1[i].timestamp_ms       = 0;
    m_detect_t_1[i].id                 = i;
    m_detect_t_1[i].x_mm               = 0.0;
    m_detect_t_1[i].y_mm               = 0.0;
    m_detect_t_1[i].vx_mm_sec          = 0.0;
    m_detect_t_1[i].vy_mm_sec          = 0.0;
    m_detect_t_1[i].ax_mm_sec_2        = 0.0;
    m_detect_t_1[i].ay_mm_sec_2        = 0.0;

    m_detect_t_2[i].detect_quality     = 0;
    m_detect_t_2[i].timestamp_ms       = 0;
    m_detect_t_2[i].id                 = i;
    m_detect_t_2[i].x_mm               = 0.0;
    m_detect_t_2[i].y_mm               = 0.0;
    m_detect_t_2[i].vx_mm_sec          = 0.0;
    m_detect_t_2[i].vy_mm_sec          = 0.0;
    m_detect_t_2[i].ax_mm_sec_2        = 0.0;
    m_detect_t_2[i].ay_mm_sec_2        = 0.0;
  }

  m_detect_lock = false;

  memcpy (m_detect_export, m_detect_t_0, sizeof(m_detect_t_0));

  memset (m_sample_cache, 0, sizeof(m_sample_cache));

  m_last_free_cache = 0;

  /* FIXME : TODO : add beacon zones in conf! */
  m_beacon_zone_cnt = 4;

  m_beacon_zone[0].x_min_mm =  1000.0 - 100.0;
  m_beacon_zone[0].y_min_mm = -1500.0 - 100.0;
  m_beacon_zone[0].x_max_mm =  1000.0 + 100.0;
  m_beacon_zone[0].y_max_mm = -1500.0 +  20.0;

  m_beacon_zone[1].x_min_mm =    50.0 - 100.0;
  m_beacon_zone[1].y_min_mm =  1500.0 -  20.0;
  m_beacon_zone[1].x_max_mm =    50.0 + 100.0;
  m_beacon_zone[1].y_max_mm =  1500.0 + 100.0;

  m_beacon_zone[2].x_min_mm =  1950.0 - 100.0;
  m_beacon_zone[2].y_min_mm =  1500.0 -  20.0;
  m_beacon_zone[2].x_max_mm =  1950.0 + 100.0;
  m_beacon_zone[2].y_max_mm =  1500.0 + 100.0;

#if 1
/* FIXME : EXPERIMENTAL : girouette */
  m_beacon_zone[3].x_min_mm =  -200.0;
  m_beacon_zone[3].y_min_mm =  -200.0;
  m_beacon_zone[3].x_max_mm =   200.0;
  m_beacon_zone[3].y_max_mm =   200.0;
#else
/* FIXME : DEBUG : CALIB */
  m_beacon_zone[3].x_min_mm =  1500.0;
  m_beacon_zone[3].y_min_mm = -1300.0;
  m_beacon_zone[3].x_max_mm =  1800.0;
  m_beacon_zone[3].y_max_mm =  1500.0;
#endif

  return 0;
}


void LidarDetect::clearSlots()
{
  for (int i=0; i<MAX_NB_OF_DETECTION_SLOTS; i++)
  {
    m_detect_slot[i].nb_rplidar_samples = 0;
    m_detect_slot[i].timestamp_ms = 0;
    m_detect_slot[i].x_mm = 0.0;
    m_detect_slot[i].y_mm = 0.0;
  }
}


bool LidarDetect::sampleInBeaconZone(double x_mm, double y_mm)
{
  for (int i=0; i<m_beacon_zone_cnt; i++)
  {
    if (
      (x_mm>=m_beacon_zone[i].x_min_mm) && 
      (y_mm>=m_beacon_zone[i].y_min_mm) && 
      (x_mm<=m_beacon_zone[i].x_max_mm) && 
      (y_mm<=m_beacon_zone[i].y_max_mm)
      )
    {
      return true;
    }
  }
  return false;
}


void LidarDetect::recordNewLidarSample(unsigned int ts_ms, double x_mm, double y_mm)
{
  m_cur_ts_ms = ts_ms;

  for (int i=0; i<MAX_NB_OF_CACHED_SAMPLES; i++)
  {
    if (m_sample_cache[i].timestamp_ms == 0)
    {
      m_sample_cache[i].timestamp_ms = ts_ms;
      m_sample_cache[i].x_mm = x_mm;
      m_sample_cache[i].y_mm = y_mm;
      return;
    }
  }
}


void LidarDetect::updateDetection()
{
  int best_samples=0;
  int best_pos=0;
  int second_samples=0;
  int second_pos=0;
  int third_samples=0;
  int third_pos=0;


  unsigned int lifetime_ms = m_rplidar_plot_lifetime_ms;

  for (int j=0; j<MAX_NB_OF_CACHED_SAMPLES; j++)
  {
    if ((m_cur_ts_ms - m_sample_cache[j].timestamp_ms) > lifetime_ms)
    {
      m_sample_cache[j].timestamp_ms = 0;
      m_sample_cache[j].x_mm = 0.0;
      m_sample_cache[j].y_mm = 0.0;
    }
    else
    {
      unsigned int ts_ms = m_sample_cache[j].timestamp_ms;
      double x_mm = m_sample_cache[j].x_mm;
      double y_mm = m_sample_cache[j].y_mm;

      for (int i=0; i<MAX_NB_OF_DETECTION_SLOTS; i++)
      {
        if (m_detect_slot[i].nb_rplidar_samples == 0)
        {
          m_detect_slot[i].nb_rplidar_samples = 1;
          m_detect_slot[i].timestamp_ms = ts_ms;
          m_detect_slot[i].x_mm = x_mm;
          m_detect_slot[i].y_mm = y_mm;
          break;
        }
        else if (dist(m_detect_slot[i].x_mm,m_detect_slot[i].y_mm,x_mm,y_mm) 
                 < OBSTACLE_SIZE_MM)
        {
          int n = m_detect_slot[i].nb_rplidar_samples;
          m_detect_slot[i].nb_rplidar_samples++;
          if (m_detect_slot[i].timestamp_ms < ts_ms) 
            m_detect_slot[i].timestamp_ms = ts_ms;
          m_detect_slot[i].x_mm = (x_mm + n*m_detect_slot[i].x_mm)/(n+1);
          m_detect_slot[i].y_mm = (y_mm + n*m_detect_slot[i].y_mm)/(n+1);
          break;
        }
      }
    }

  } /* for (int j=0; j<MAX_NB_OF_CACHED_SAMPLES; j++) */


  for (int i=0; i<MAX_NB_OF_DETECTION_SLOTS; i++)
  {
    if (m_detect_slot[i].nb_rplidar_samples > best_samples)
    {
      third_samples = second_samples;
      third_pos = second_pos;

      second_samples = best_samples;
      second_pos = best_pos;

      best_samples = m_detect_slot[i].nb_rplidar_samples;
      best_pos = i;
    }
    else if (m_detect_slot[i].nb_rplidar_samples > second_samples)
    {
      third_samples = second_samples;
      third_pos = second_pos;

      second_samples = m_detect_slot[i].nb_rplidar_samples;
      second_pos = i;
    }
    else if (m_detect_slot[i].nb_rplidar_samples > third_samples)
    {
      third_samples = m_detect_slot[i].nb_rplidar_samples;
      third_pos = i;
    }
  }

  m_detect_candidate[0].detect_quality     = m_detect_slot[best_pos].nb_rplidar_samples;
  m_detect_candidate[0].timestamp_ms       = m_detect_slot[best_pos].timestamp_ms;
  m_detect_candidate[0].id                 = 0xffffffff;
  m_detect_candidate[0].x_mm               = m_detect_slot[best_pos].x_mm;
  m_detect_candidate[0].y_mm               = m_detect_slot[best_pos].y_mm;

  m_detect_candidate[1].detect_quality     = m_detect_slot[second_pos].nb_rplidar_samples;
  m_detect_candidate[1].timestamp_ms       = m_detect_slot[second_pos].timestamp_ms;
  m_detect_candidate[1].id                 = 0xffffffff;
  m_detect_candidate[1].x_mm               = m_detect_slot[second_pos].x_mm;
  m_detect_candidate[1].y_mm               = m_detect_slot[second_pos].y_mm;

  m_detect_candidate[2].detect_quality     = m_detect_slot[third_pos].nb_rplidar_samples;
  m_detect_candidate[2].timestamp_ms       = m_detect_slot[third_pos].timestamp_ms;
  m_detect_candidate[2].id                 = 0xffffffff;
  m_detect_candidate[2].x_mm               = m_detect_slot[third_pos].x_mm;
  m_detect_candidate[2].y_mm               = m_detect_slot[third_pos].y_mm;

#ifdef MULTITRACKING
  for (int i=0; i<MAX_NB_OF_DETECTED_ROBOTS; i++)
  {
    m_detect_t_2[i].detect_quality     = m_detect_t_1[i].detect_quality;
    m_detect_t_2[i].timestamp_ms       = m_detect_t_1[i].timestamp_ms;
    m_detect_t_2[i].id                 = m_detect_t_1[i].id;
    m_detect_t_2[i].x_mm               = m_detect_t_1[i].x_mm;
    m_detect_t_2[i].y_mm               = m_detect_t_1[i].y_mm;
    m_detect_t_2[i].vx_mm_sec          = m_detect_t_1[i].vx_mm_sec;
    m_detect_t_2[i].vy_mm_sec          = m_detect_t_1[i].vy_mm_sec;
    m_detect_t_2[i].ax_mm_sec_2        = m_detect_t_1[i].ax_mm_sec_2;
    m_detect_t_2[i].ay_mm_sec_2        = m_detect_t_1[i].ay_mm_sec_2;

    m_detect_t_1[i].detect_quality     = m_detect_t_0[i].detect_quality;
    m_detect_t_1[i].timestamp_ms       = m_detect_t_0[i].timestamp_ms;
    m_detect_t_1[i].id                 = m_detect_t_0[i].id;
    m_detect_t_1[i].x_mm               = m_detect_t_0[i].x_mm;
    m_detect_t_1[i].y_mm               = m_detect_t_0[i].y_mm;
    m_detect_t_1[i].vx_mm_sec          = m_detect_t_0[i].vx_mm_sec;
    m_detect_t_1[i].vy_mm_sec          = m_detect_t_0[i].vy_mm_sec;
    m_detect_t_1[i].ax_mm_sec_2        = m_detect_t_0[i].ax_mm_sec_2;
    m_detect_t_1[i].ay_mm_sec_2        = m_detect_t_0[i].ay_mm_sec_2;
  }

  for (int i=0; i<MAX_NB_OF_DETECTED_ROBOTS; i++)
  {
    m_detect_t_0[i].id                 = 0xffffffff;
    if ((m_detect_t_0[i].detect_quality>0))
    {
      for (int j=0; j<MAX_NB_OF_DETECTED_ROBOTS; j++)
      {
        if ((m_detect_candidate[j].detect_quality>0) && dist(m_detect_t_0[i],m_detect_candidate[j])<100.0)
        {
          m_detect_t_0[i].detect_quality = m_detect_candidate[j].detect_quality;
          m_detect_t_0[i].timestamp_ms   = m_detect_candidate[j].timestamp_ms;
          m_detect_t_0[i].id             = i;
          m_detect_t_0[i].x_mm           = m_detect_candidate[j].x_mm;
          m_detect_t_0[i].y_mm           = m_detect_candidate[j].y_mm;

          m_detect_candidate[j].id       = i;
        }
      }
    }
    else
    {
      for (int j=0; j<MAX_NB_OF_DETECTED_ROBOTS; j++)
      {
        if ((m_detect_candidate[j].detect_quality>0) && (m_detect_candidate[j].id==0xffffffff))
        {
          m_detect_t_0[i].detect_quality = m_detect_candidate[j].detect_quality;
          m_detect_t_0[i].timestamp_ms   = m_detect_candidate[j].timestamp_ms;
          m_detect_t_0[i].id             = i;
          m_detect_t_0[i].x_mm           = m_detect_candidate[j].x_mm;
          m_detect_t_0[i].y_mm           = m_detect_candidate[j].y_mm;

          m_detect_candidate[j].id       = i;
        }
      }
    }
  }

  for (int i=0; i<MAX_NB_OF_DETECTED_ROBOTS; i++)
  {
    double dt_sec = (m_detect_t_0[i].timestamp_ms - m_detect_t_1[i].timestamp_ms)/1000.0;

    m_detect_t_0[i].id                 = i;
    m_detect_t_0[i].vx_mm_sec          = (m_detect_t_0[i].x_mm - m_detect_t_1[i].x_mm)/dt_sec;
    m_detect_t_0[i].vy_mm_sec          = (m_detect_t_0[i].y_mm - m_detect_t_1[i].y_mm)/dt_sec;
    m_detect_t_0[i].ax_mm_sec_2        = (m_detect_t_0[i].vx_mm_sec - m_detect_t_1[i].vx_mm_sec)/dt_sec;
    m_detect_t_0[i].ay_mm_sec_2        = (m_detect_t_0[i].vy_mm_sec - m_detect_t_1[i].vy_mm_sec)/dt_sec;
  }

#else

  m_detect_t_0[0].detect_quality     = m_detect_slot[best_pos].nb_rplidar_samples;
  m_detect_t_0[0].timestamp_ms       = m_detect_slot[best_pos].timestamp_ms;
  m_detect_t_0[0].id                 = 0;
  m_detect_t_0[0].x_mm               = m_detect_slot[best_pos].x_mm;
  m_detect_t_0[0].y_mm               = m_detect_slot[best_pos].y_mm;

  m_detect_t_0[1].detect_quality     = m_detect_slot[second_pos].nb_rplidar_samples;
  m_detect_t_0[1].timestamp_ms       = m_detect_slot[second_pos].timestamp_ms;
  m_detect_t_0[1].id                 = 1;
  m_detect_t_0[1].x_mm               = m_detect_slot[second_pos].x_mm;
  m_detect_t_0[1].y_mm               = m_detect_slot[second_pos].y_mm;

  m_detect_t_0[2].detect_quality     = m_detect_slot[third_pos].nb_rplidar_samples;
  m_detect_t_0[2].timestamp_ms       = m_detect_slot[third_pos].timestamp_ms;
  m_detect_t_0[2].id                 = 2;
  m_detect_t_0[2].x_mm               = m_detect_slot[third_pos].x_mm;
  m_detect_t_0[2].y_mm               = m_detect_slot[third_pos].y_mm;

#endif

  /* FIXME : TODO : improve (quick hack) */
  if (m_nb_of_send_detect<3)
    m_detect_t_0[2].detect_quality = 0;
  if (m_nb_of_send_detect<2)
    m_detect_t_0[1].detect_quality = 0;

  m_detect_lock = true;
  memcpy (m_detect_export, m_detect_t_0, sizeof(m_detect_t_0));
  m_detect_lock = false;
}


detected_robot_info_t& LidarDetect::detected_robot(int _obst_idx)
{
  /* FIXME : TODO : improve synchronisation */
  while(m_detect_lock); /* Warning : dangerous! */
  return m_detect_export[_obst_idx];
}


double LidarDetect::dist(double x0, double y0, double x1, double y1)
{
  return sqrt((x0-x1)*(x0-x1) + (y0-y1)*(y0-y1));
}


double LidarDetect::dist(detected_robot_info_t &R0, detected_robot_info_t &R1)
{
  return sqrt((R0.x_mm-R1.x_mm)*(R0.x_mm-R1.x_mm) + (R0.y_mm-R1.y_mm)*(R0.y_mm-R1.y_mm));
}


