/******************************************************************************
 * Copyright (c) The Conch Global Authors. 2021 .All Rights Reserved.
 * Description: main function
 *
 * Author: xiongyanfei
 * Create: 2021-04-22
 *****************************************************************************/

#pragma once

#include <memory>
#include <list>
#include <string>

namespace Decision {


/*plan to Dwa data*/
struct RP_tstDestinationStatus {
    float fx;
    float fy;
    float fAngleHeading;
};
struct RoutePoint
{
    float x;
    float y;
    float heading;
};
typedef struct RP_tstCalDistance
{
    float fX = 0.0;  // x轴
    float fY = 0.0;  // y轴
} RP_tstCalDistance;

typedef struct RP_tstPlanToAlp
{
    int i_nearest_index;  // 距离路线最近的点位置
    int i_last_handover_index;  // 计算上山最终交接点
    int i_plan_to_alp_index;  // 交点位置index
    float f_plan_to_alp_position_x;  // 交点x坐标
    float f_plan_to_alp_position_y;  // 交点y坐标
    float f_nearest_position_x;  // 最近点x坐标
    float f_nearest_position_y;  // 最近点y坐标
    float f_last_handover_position_x;  // 最终交接点x坐标
    float f_last_handover_position_y;  // 最终交接点y坐标

}RP_tstPlanToAlp;

struct RTKData {
  int fp_nPointMax;
  int fp_nHandover_backward_dis;
  int fp_nHandover_backward_diff;
};

struct StopPoint {
  float x;
  float y;
  float heading;
};

}
