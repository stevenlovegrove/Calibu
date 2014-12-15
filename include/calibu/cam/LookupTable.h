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

#include <calibu/Platform.h>
#include <vector>

namespace calibu
{

///////////////////////////////////////////////////////////////////////////////
/// This structure is used to build a LUT without requiring branching
//  when rectifying images. The values out of the image to the top left
//  pixels instead of having a test for out of bound access, the aim is
//  to avoid a branch in the code.
//
//    int xt = (int) x;  /* top-left corner */
//    int yt = (int) y;
//    float ax = x - xt;
//    float ay = y - yt;
//    float *ptr = image + (width*yt) + xt;
//
//    return( (1-ax) * (1-ay) * *ptr +
//            ( ax ) * (1-ay) * *(ptr+1) +
//            (1-ax) * ( ay ) * *(ptr+(width)) +
//            ( ax ) * ( ay ) * *(ptr+(width)+1) );
struct BilinearLutPoint
{
    int idx0; // index to pixel in src image
    int idx1; // index to pixel + one row in src image
    float w00; // top left weight for bilinear interpolation
    float w01; // top right weight for bilinear interpolation
    float w10; // bottom left weight ...
    float w11; // bottom right weight ...
};

///////////////////////////////////////////////////////////////////////////////
CALIBU_EXPORT
struct LookupTable
{
    inline LookupTable(){};
    inline LookupTable( int nWidth, int nHeight )
    {
        m_vLutPixels.resize( nWidth*nHeight );
        m_nWidth = nWidth;
    }

    inline LookupTable( const LookupTable& rhs )
    {
        m_vLutPixels.resize( rhs.m_vLutPixels.size() );
        m_nWidth = rhs.m_nWidth;
        m_vLutPixels = rhs.m_vLutPixels;
    }

    inline unsigned int Width() const
    {
        return m_nWidth;
    }

    inline unsigned int Height() const
    {
        return m_vLutPixels.size() / m_nWidth;
    }

    inline void SetPoint( unsigned int nRow, unsigned int nCol, const BilinearLutPoint& p )
    {
        m_vLutPixels[ nRow*m_nWidth + nCol ] = p;
    }

    std::vector<BilinearLutPoint> m_vLutPixels;
    int m_nWidth; // so m_nHeight = m_vPixels.size()/m_nWidth
};

}
