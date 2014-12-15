/*
   This file is part of the Calibu Project.
   https://github.com/gwu-robotics/Calibu

   Copyright (C) 2013 George Washington University,
                      Steven Lovegrove,
                      Gabe Sibley

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 */

#pragma once

#include <algorithm>
#include <vector>
#include <sophus/se3.hpp>

#include <calibu/Platform.h>
#include <calibu/cam/CameraModel.h>
#include <calibu/utils/Range.h>
#include <calibu/cam/LookupTable.h>

namespace calibu
{
    /// Create lookup table which can be used to remap a general camera model
    /// 'cam_from' to a linear and potentially rotated model, 'R_onK'.
    /// R_onK is formed from the multiplication R_on (old form new) and the new
    /// camera intrinsics K.
    CALIBU_EXPORT void CreateLookupTable(
            const calibu::CameraModelInterface& cam_from,
            const Eigen::Matrix3d& R_onKinv,
            LookupTable& lut
            );

    /// Rectify image pInputImageData using lookup table generated by
    /// 'CreateLookupTable' to output image pOutputRectImageData.
    CALIBU_EXPORT void Rectify(
            const LookupTable& lut,
            const unsigned char* pInputImageData,
            unsigned char* pOutputRectImageData,
            int w, int h
            );


    ///
    inline Range MinMaxRotatedCol( const calibu::CameraModelInterface& cam, const Eigen::Matrix3d& Rnl_l )
    {
        Range range = Range::Open();
        for(size_t row = 0; row < cam.Height(); ++row) {
            const Eigen::Vector2d ln = Project(Eigen::Vector3d(Rnl_l* cam.Unproject(Eigen::Vector2d(0,row)) ));
            const Eigen::Vector2d rn = Project(Eigen::Vector3d(Rnl_l* cam.Unproject(Eigen::Vector2d(cam.Width()-1,row)) ));
            range.ExcludeLessThan(ln[0]);
            range.ExcludeGreaterThan(rn[0]);
        }

        return range;
    }

    ///
    inline Range MinMaxRotatedRow( const calibu::CameraModelInterface& cam, const Eigen::Matrix3d& Rnl_l )
    {
        Range range = Range::Open();
        for(size_t col = 0; col < cam.Width(); ++col) {
            const Eigen::Vector2d tn = Project(Eigen::Vector3d(Rnl_l*cam.Unproject(Eigen::Vector2d(col,0)) ));
            const Eigen::Vector2d bn = Project(Eigen::Vector3d(Rnl_l*cam.Unproject(Eigen::Vector2d(col,cam.Height()-1)) ));
            range.ExcludeLessThan(tn[1]);
            range.ExcludeGreaterThan(bn[1]);
        }
        return range;
    }
}
