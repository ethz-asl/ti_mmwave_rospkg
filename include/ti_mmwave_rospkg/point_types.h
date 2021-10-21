/*
 * \file
 *
 *  Customized Point Cloud Library point structures for mmWave1843Boost data.
 *
 *
 * Copyright? (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  @author Claude, https://github.com/Claud1234/ti_mmwave_rospkg/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef TI_MMWAVE_ROSPKG_POINT_TYPES_H
#define TI_MMWAVE_ROSPKG_POINT_TYPES_H

#include <pcl/point_types.h>

/* Customized structure x, y, z, intensity, velocity, range*/
namespace radar_pcl
{
struct PointXYZIVR
{
  PCL_ADD_POINT4D;
  float intensity;

  // Velocity in cartesian
  float velocity;

  // range in cartesian
  float range;

  // ensure proper alignment
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}
EIGEN_ALIGN16;

}  // namespace radar_pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(radar_pcl::PointXYZIVR,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, intensity, intensity)
                                  (float, velocity, velocity)
                                  (float, range, range))

#endif  // TI_MMWAVE_ROSPKG_POINT_TYPES_H
