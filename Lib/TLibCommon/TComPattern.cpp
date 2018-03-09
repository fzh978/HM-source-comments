/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2015, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     TComPattern.cpp
    \brief    neighbouring pixel access classes
*/

#include "TComPic.h"
#include "TComPattern.h"
#include "TComDataCU.h"
#include "TComTU.h"
#include "Debug.h"
#include "TComPrediction.h"

//! \ingroup TLibCommon
//! \{

// Forward declarations

/// padding of unavailable reference samples for intra prediction
Void fillReferenceSamples( const Int bitDepth, 
#if O0043_BEST_EFFORT_DECODING
                           const Int bitDepthDelta, 
#endif
                           const Pel* piRoiOrigin, 
                                 Pel* piIntraTemp,
                           const Bool* bNeighborFlags,
                           const Int iNumIntraNeighbor, 
                           const Int unitWidth, 
                           const Int unitHeight, 
                           const Int iAboveUnits, 
                           const Int iLeftUnits,
                           const UInt uiWidth, 
                           const UInt uiHeight, 
                           const Int iPicStride );

/// constrained intra prediction
Bool  isAboveLeftAvailable  ( TComDataCU* pcCU, UInt uiPartIdxLT );
Int   isAboveAvailable      ( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxRT, Bool* bValidFlags );
Int   isLeftAvailable       ( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxLB, Bool* bValidFlags );
Int   isAboveRightAvailable ( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxRT, Bool* bValidFlags );
Int   isBelowLeftAvailable  ( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxLB, Bool* bValidFlags );

<<<<<<< HEAD
//该类的功能是求得用于帧内预测的参考像素值　包括符合条件（Tu块大小和帧内预测模式）的参考像素滤波　参考像素的获取（根据情况分为直接获取和用最邻近像素填充）
=======

>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
// ====================================================================================================================
// Public member functions (TComPatternParam)
// ====================================================================================================================

/** 
 \param  piTexture     pixel data
 \param  iRoiWidth     pattern width
 \param  iRoiHeight    pattern height
 \param  iStride       buffer stride
 \param  bitDepth      bit depth
 */
Void TComPatternParam::setPatternParamPel ( Pel* piTexture,
                                           Int iRoiWidth,
                                           Int iRoiHeight,
                                           Int iStride,
                                           Int bitDepth
                                           )
{
  m_piROIOrigin    = piTexture;
  m_iROIWidth      = iRoiWidth;
  m_iROIHeight     = iRoiHeight;
  m_iPatternStride = iStride;
  m_bitDepth       = bitDepth;
}

// ====================================================================================================================
// Public member functions (TComPattern)
// ====================================================================================================================

Void TComPattern::initPattern (Pel* piY,
                               Int iRoiWidth,
                               Int iRoiHeight,
                               Int iStride,
                               Int bitDepthLuma)
{
  m_cPatternY. setPatternParamPel( piY,  iRoiWidth, iRoiHeight, iStride, bitDepthLuma);
}


// TODO: move this function to TComPrediction.cpp.
Void TComPrediction::initIntraPatternChType( TComTU &rTu, const ComponentID compID, const Bool bFilterRefSamples DEBUG_STRING_FN_DECLARE(sDebug))
{
<<<<<<< HEAD
  const ChannelType chType    = toChannelType(compID);//通道类型

  TComDataCU *pcCU=rTu.getCU();//Tu块所在Cu块信息
  const TComSPS &sps = *(pcCU->getSlice()->getSPS());
  const UInt uiZorderIdxInPart=rTu.GetAbsPartIdxTU();//Tu在Ctu中的位置(Z形扫描）　＝GetAbsPartIdxCU() + GetRelPartIdxTU(compID)
  const UInt uiTuWidth        = rTu.getRect(compID).width;//Tu块的宽
  const UInt uiTuHeight       = rTu.getRect(compID).height;//Tu块的高
  const UInt uiTuWidth2       = uiTuWidth  << 1;//2倍Tu块的宽
  const UInt uiTuHeight2      = uiTuHeight << 1;//2倍Tu块的高

  const Int  iBaseUnitSize    = sps.getMaxCUWidth() >> sps.getMaxTotalCUDepth();//基本块大小默认参数下为4*4
  const Int  iUnitWidth       = iBaseUnitSize  >> pcCU->getPic()->getPicYuvRec()->getComponentScaleX(compID);//不同分量下单位块的宽
  const Int  iUnitHeight      = iBaseUnitSize  >> pcCU->getPic()->getPicYuvRec()->getComponentScaleY(compID);//不同分量下单位块的高
  const Int  iTUWidthInUnits  = uiTuWidth  / iUnitWidth;//Tu以单位块为单位的宽
  const Int  iTUHeightInUnits = uiTuHeight / iUnitHeight;//Tu以单位块为单位的高
  const Int  iAboveUnits      = iTUWidthInUnits  << 1;//上方参考像素的个数（以单位块为单位）
  const Int  iLeftUnits       = iTUHeightInUnits << 1;//左侧参考像素的个数（以单位块为单位）
  const Int  bitDepthForChannel = sps.getBitDepth(chType);//通道对应位深

  assert(iTUHeightInUnits > 0 && iTUWidthInUnits > 0);

  const Int  iPartIdxStride   = pcCU->getPic()->getNumPartInCtuWidth();//步长为Ctu以4*4单位块为单位的宽　默认参数下为16
  const UInt uiPartIdxLT      = pcCU->getZorderIdxInCtu() + uiZorderIdxInPart;//Tu中左上4*4小块的位置
  const UInt uiPartIdxRT      = g_auiRasterToZscan[ g_auiZscanToRaster[ uiPartIdxLT ] +   iTUWidthInUnits  - 1                   ];//Tu中右上4*4小块的位置
  const UInt uiPartIdxLB      = g_auiRasterToZscan[ g_auiZscanToRaster[ uiPartIdxLT ] + ((iTUHeightInUnits - 1) * iPartIdxStride)];//Tu中左下4*4小块的位置

  Int   iPicStride = pcCU->getPic()->getStride(compID);
  Bool  bNeighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];//Tu中各小块是否存在相邻d的小块
  Int   iNumIntraNeighbor = 0;//相邻小块的个数

  bNeighborFlags[iLeftUnits] = isAboveLeftAvailable( pcCU, uiPartIdxLT );//左上参考像素是否可获得
  iNumIntraNeighbor += bNeighborFlags[iLeftUnits] ? 1 : 0;
  iNumIntraNeighbor  += isAboveAvailable     ( pcCU, uiPartIdxLT, uiPartIdxRT, (bNeighborFlags + iLeftUnits + 1)                    );//上方参考像素是否可获得
  iNumIntraNeighbor  += isAboveRightAvailable( pcCU, uiPartIdxLT, uiPartIdxRT, (bNeighborFlags + iLeftUnits + 1 + iTUWidthInUnits ) );//右上参考像素是否可获得
  iNumIntraNeighbor  += isLeftAvailable      ( pcCU, uiPartIdxLT, uiPartIdxLB, (bNeighborFlags + iLeftUnits - 1)                    );//左侧参考像素是否可获得
  iNumIntraNeighbor  += isBelowLeftAvailable ( pcCU, uiPartIdxLT, uiPartIdxLB, (bNeighborFlags + iLeftUnits - 1 - iTUHeightInUnits) );//左下参考像素是否可获得　并统计可获得的相邻块的个数

  const UInt         uiROIWidth  = uiTuWidth2+1;//覆盖参考像素的最小宽度
  const UInt         uiROIHeight = uiTuHeight2+1;//覆盖参考像素的最小高度

  assert(uiROIWidth*uiROIHeight <= m_iYuvExtSize);//覆盖参考像素的范围一定小于图像范围
=======
  const ChannelType chType    = toChannelType(compID);

  TComDataCU *pcCU=rTu.getCU();
  const TComSPS &sps = *(pcCU->getSlice()->getSPS());
  const UInt uiZorderIdxInPart=rTu.GetAbsPartIdxTU();
  const UInt uiTuWidth        = rTu.getRect(compID).width;
  const UInt uiTuHeight       = rTu.getRect(compID).height;
  const UInt uiTuWidth2       = uiTuWidth  << 1;
  const UInt uiTuHeight2      = uiTuHeight << 1;

  const Int  iBaseUnitSize    = sps.getMaxCUWidth() >> sps.getMaxTotalCUDepth();
  const Int  iUnitWidth       = iBaseUnitSize  >> pcCU->getPic()->getPicYuvRec()->getComponentScaleX(compID);
  const Int  iUnitHeight      = iBaseUnitSize  >> pcCU->getPic()->getPicYuvRec()->getComponentScaleY(compID);
  const Int  iTUWidthInUnits  = uiTuWidth  / iUnitWidth;
  const Int  iTUHeightInUnits = uiTuHeight / iUnitHeight;
  const Int  iAboveUnits      = iTUWidthInUnits  << 1;
  const Int  iLeftUnits       = iTUHeightInUnits << 1;
  const Int  bitDepthForChannel = sps.getBitDepth(chType);

  assert(iTUHeightInUnits > 0 && iTUWidthInUnits > 0);

  const Int  iPartIdxStride   = pcCU->getPic()->getNumPartInCtuWidth();
  const UInt uiPartIdxLT      = pcCU->getZorderIdxInCtu() + uiZorderIdxInPart;
  const UInt uiPartIdxRT      = g_auiRasterToZscan[ g_auiZscanToRaster[ uiPartIdxLT ] +   iTUWidthInUnits  - 1                   ];
  const UInt uiPartIdxLB      = g_auiRasterToZscan[ g_auiZscanToRaster[ uiPartIdxLT ] + ((iTUHeightInUnits - 1) * iPartIdxStride)];

  Int   iPicStride = pcCU->getPic()->getStride(compID);
  Bool  bNeighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  Int   iNumIntraNeighbor = 0;

  bNeighborFlags[iLeftUnits] = isAboveLeftAvailable( pcCU, uiPartIdxLT );
  iNumIntraNeighbor += bNeighborFlags[iLeftUnits] ? 1 : 0;
  iNumIntraNeighbor  += isAboveAvailable     ( pcCU, uiPartIdxLT, uiPartIdxRT, (bNeighborFlags + iLeftUnits + 1)                    );
  iNumIntraNeighbor  += isAboveRightAvailable( pcCU, uiPartIdxLT, uiPartIdxRT, (bNeighborFlags + iLeftUnits + 1 + iTUWidthInUnits ) );
  iNumIntraNeighbor  += isLeftAvailable      ( pcCU, uiPartIdxLT, uiPartIdxLB, (bNeighborFlags + iLeftUnits - 1)                    );
  iNumIntraNeighbor  += isBelowLeftAvailable ( pcCU, uiPartIdxLT, uiPartIdxLB, (bNeighborFlags + iLeftUnits - 1 - iTUHeightInUnits) );

  const UInt         uiROIWidth  = uiTuWidth2+1;
  const UInt         uiROIHeight = uiTuHeight2+1;

  assert(uiROIWidth*uiROIHeight <= m_iYuvExtSize);
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2

#if DEBUG_STRING
  std::stringstream ss(stringstream::out);
#endif

  {
    Pel *piIntraTemp   = m_piYuvExt[compID][PRED_BUF_UNFILTERED];
<<<<<<< HEAD
    Pel *piRoiOrigin = pcCU->getPic()->getPicYuvRec()->getAddr(compID, pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu()+uiZorderIdxInPart);//Tu起始像素值在图像中的位置
=======
    Pel *piRoiOrigin = pcCU->getPic()->getPicYuvRec()->getAddr(compID, pcCU->getCtuRsAddr(), pcCU->getZorderIdxInCtu()+uiZorderIdxInPart);
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
#if O0043_BEST_EFFORT_DECODING
    const Int  bitDepthForChannelInStream = sps.getStreamBitDepth(chType);
    fillReferenceSamples (bitDepthForChannelInStream, bitDepthForChannelInStream - bitDepthForChannel,
#else
    fillReferenceSamples (bitDepthForChannel,
#endif
                          piRoiOrigin, piIntraTemp, bNeighborFlags, iNumIntraNeighbor,  iUnitWidth, iUnitHeight, iAboveUnits, iLeftUnits,
<<<<<<< HEAD
                          uiROIWidth, uiROIHeight, iPicStride);//得到帧内预测所需的参考像素值
=======
                          uiROIWidth, uiROIHeight, iPicStride);
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2


#if DEBUG_STRING
    if (DebugOptionList::DebugString_Pred.getInt()&DebugStringGetPredModeMask(MODE_INTRA))
    {
      ss << "###: generating Ref Samples for channel " << compID << " and " << rTu.getRect(compID).width << " x " << rTu.getRect(compID).height << "\n";
      for (UInt y=0; y<uiROIHeight; y++)
      {
        ss << "###: - ";
        for (UInt x=0; x<uiROIWidth; x++)
        {
          if (x==0 || y==0)
          {
            ss << piIntraTemp[y*uiROIWidth + x] << ", ";
//          if (x%16==15) ss << "\nPart size: ~ ";
          }
        }
        ss << "\n";
      }
    }
#endif

<<<<<<< HEAD
    if (bFilterRefSamples)//如果参考像素需要滤波
=======
    if (bFilterRefSamples)
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
    {
      // generate filtered intra prediction samples

            Int          stride    = uiROIWidth;
      const Pel         *piSrcPtr  = piIntraTemp                           + (stride * uiTuHeight2); // bottom left
            Pel         *piDestPtr = m_piYuvExt[compID][PRED_BUF_FILTERED] + (stride * uiTuHeight2); // bottom left

      //------------------------------------------------

<<<<<<< HEAD
      Bool useStrongIntraSmoothing = isLuma(chType) && sps.getUseStrongIntraSmoothing();//参考像素强滤波的前提条件是　亮度分量且允许强滤波

      const Pel bottomLeft = piIntraTemp[stride * uiTuHeight2];//左下参考像素值
      const Pel topLeft    = piIntraTemp[0];//左上参考像素值
      const Pel topRight   = piIntraTemp[uiTuWidth2];//右上参考像素值

      if (useStrongIntraSmoothing)//若满足强滤波的前提条件　则判断参考像素是否需要强滤波
=======
      Bool useStrongIntraSmoothing = isLuma(chType) && sps.getUseStrongIntraSmoothing();

      const Pel bottomLeft = piIntraTemp[stride * uiTuHeight2];
      const Pel topLeft    = piIntraTemp[0];
      const Pel topRight   = piIntraTemp[uiTuWidth2];

      if (useStrongIntraSmoothing)
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
      {
#if O0043_BEST_EFFORT_DECODING
        const Int  threshold     = 1 << (bitDepthForChannelInStream - 5);
#else
<<<<<<< HEAD
        const Int  threshold     = 1 << (bitDepthForChannel - 5);//用于判断的阈值
#endif
        const Bool bilinearLeft  = abs((bottomLeft + topLeft ) - (2 * piIntraTemp[stride * uiTuHeight])) < threshold; //difference between the　abs(A+C-2B)<threshold
        const Bool bilinearAbove = abs((topLeft    + topRight) - (2 * piIntraTemp[         uiTuWidth ])) < threshold; //ends and the middle     abs(A+C-2B)<threshold
        if ((uiTuWidth < 32) || (!bilinearLeft) || (!bilinearAbove))//Tu大小为32*32并且参考像素满足阈值判断条件　才进行强滤波
=======
        const Int  threshold     = 1 << (bitDepthForChannel - 5);
#endif
        const Bool bilinearLeft  = abs((bottomLeft + topLeft ) - (2 * piIntraTemp[stride * uiTuHeight])) < threshold; //difference between the
        const Bool bilinearAbove = abs((topLeft    + topRight) - (2 * piIntraTemp[         uiTuWidth ])) < threshold; //ends and the middle
        if ((uiTuWidth < 32) || (!bilinearLeft) || (!bilinearAbove))
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
        {
          useStrongIntraSmoothing = false;
        }
      }

<<<<<<< HEAD
      *piDestPtr = *piSrcPtr; // bottom left is not filtered　//左下像素不需要滤波　跳过该点
=======
      *piDestPtr = *piSrcPtr; // bottom left is not filtered
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
      piDestPtr -= stride;
      piSrcPtr  -= stride;

      //------------------------------------------------

      //left column (bottom to top)

<<<<<<< HEAD
      if (useStrongIntraSmoothing)//如果对参考像素进行强滤波
      {
        const Int shift = g_aucConvertToBit[uiTuHeight] + 3; //log2(uiTuHeight2)

        for(UInt i=1; i<uiTuHeight2; i++, piDestPtr-=stride)//遍历左侧参考像素（从bottom到top）
        {
          *piDestPtr = (((uiTuHeight2 - i) * bottomLeft) + (i * topLeft) + uiTuHeight) >> shift;//到左下参考像素和左上参考像素的距离的加权平均　RF(0,y)=((64-y)+R(0,0)+y*R(0,2N)+32)>>6
        }

        piSrcPtr -= stride * (uiTuHeight2 - 1);//参考像素位置从左下移到左上参考像素位置
      }
      else//对参考像素常规滤波
      {
        for(UInt i=1; i<uiTuHeight2; i++, piDestPtr-=stride, piSrcPtr-=stride)//遍历左侧参考像素（从bottom到top）
        {
          *piDestPtr = ( piSrcPtr[stride] + 2*piSrcPtr[0] + piSrcPtr[-stride] + 2 ) >> 2;//RF(0,y)=(R(0,y-1)+2R(0,y)+R(0,y+1)+2)>>2
=======
      if (useStrongIntraSmoothing)
      {
        const Int shift = g_aucConvertToBit[uiTuHeight] + 3; //log2(uiTuHeight2)

        for(UInt i=1; i<uiTuHeight2; i++, piDestPtr-=stride)
        {
          *piDestPtr = (((uiTuHeight2 - i) * bottomLeft) + (i * topLeft) + uiTuHeight) >> shift;
        }

        piSrcPtr -= stride * (uiTuHeight2 - 1);
      }
      else
      {
        for(UInt i=1; i<uiTuHeight2; i++, piDestPtr-=stride, piSrcPtr-=stride)
        {
          *piDestPtr = ( piSrcPtr[stride] + 2*piSrcPtr[0] + piSrcPtr[-stride] + 2 ) >> 2;
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
        }
      }

      //------------------------------------------------

      //top-left

<<<<<<< HEAD
      if (useStrongIntraSmoothing)//强滤波下不需要对左上参考像素滤波
      {
        *piDestPtr = piSrcPtr[0];
      }
      else//常规滤波下对左上参考像素滤波　RF(0,0)=(R(0,1)+2R(0,0)+R(1,0)+2)>>2
      {
        *piDestPtr = ( piSrcPtr[stride] + 2*piSrcPtr[0] + piSrcPtr[1] + 2 ) >> 2;
      }
      piDestPtr += 1;//移动到下一点
=======
      if (useStrongIntraSmoothing)
      {
        *piDestPtr = piSrcPtr[0];
      }
      else
      {
        *piDestPtr = ( piSrcPtr[stride] + 2*piSrcPtr[0] + piSrcPtr[1] + 2 ) >> 2;
      }
      piDestPtr += 1;
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
      piSrcPtr  += 1;

      //------------------------------------------------

      //top row (left-to-right)
<<<<<<< HEAD
      //对上方参考像素滤波（从left-to-right）　同上　不在叙述
=======

>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
      if (useStrongIntraSmoothing)
      {
        const Int shift = g_aucConvertToBit[uiTuWidth] + 3; //log2(uiTuWidth2)

        for(UInt i=1; i<uiTuWidth2; i++, piDestPtr++)
        {
          *piDestPtr = (((uiTuWidth2 - i) * topLeft) + (i * topRight) + uiTuWidth) >> shift;
        }

        piSrcPtr += uiTuWidth2 - 1;
      }
      else
      {
        for(UInt i=1; i<uiTuWidth2; i++, piDestPtr++, piSrcPtr++)
        {
          *piDestPtr = ( piSrcPtr[1] + 2*piSrcPtr[0] + piSrcPtr[-1] + 2 ) >> 2;
        }
      }

      //------------------------------------------------

<<<<<<< HEAD
      *piDestPtr=*piSrcPtr; // far right is not filtered//右上参考像素不需要滤波
=======
      *piDestPtr=*piSrcPtr; // far right is not filtered
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2

#if DEBUG_STRING
    if (DebugOptionList::DebugString_Pred.getInt()&DebugStringGetPredModeMask(MODE_INTRA))
    {
      ss << "###: filtered result for channel " << compID <<"\n";
      for (UInt y=0; y<uiROIHeight; y++)
      {
        ss << "###: - ";
        for (UInt x=0; x<uiROIWidth; x++)
        {
          if (x==0 || y==0)
          {
            ss << m_piYuvExt[compID][PRED_BUF_FILTERED][y*uiROIWidth + x] << ", ";
//          if (x%16==15) ss << "\nPart size: ~ ";
          }
        }
        ss << "\n";
      }
    }
#endif


    }
  }
  DEBUG_STRING_APPEND(sDebug, ss.str())
}

Void fillReferenceSamples( const Int bitDepth, 
#if O0043_BEST_EFFORT_DECODING
                           const Int bitDepthDelta, 
#endif
                           const Pel* piRoiOrigin, 
                                 Pel* piIntraTemp,
                           const Bool* bNeighborFlags,
                           const Int iNumIntraNeighbor, 
                           const Int unitWidth, 
                           const Int unitHeight, 
                           const Int iAboveUnits, 
                           const Int iLeftUnits,
                           const UInt uiWidth, 
                           const UInt uiHeight, 
                           const Int iPicStride )
<<<<<<< HEAD
{//得到帧间预测参考像素
=======
{
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
  const Pel* piRoiTemp;
  Int  i, j;
  Int  iDCValue = 1 << (bitDepth - 1);
  const Int iTotalUnits = iAboveUnits + iLeftUnits + 1; //+1 for top-left

<<<<<<< HEAD
  if (iNumIntraNeighbor == 0)//若所有区域的参考像素都不可得到　则参考像素都用DC值填充
  {
    // Fill border with DC value
    for (i=0; i<uiWidth; i++)//上方参考像素
    {
      piIntraTemp[i] = iDCValue;
    }
    for (i=1; i<uiHeight; i++)//左侧参考像素
=======
  if (iNumIntraNeighbor == 0)
  {
    // Fill border with DC value
    for (i=0; i<uiWidth; i++)
    {
      piIntraTemp[i] = iDCValue;
    }
    for (i=1; i<uiHeight; i++)
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
    {
      piIntraTemp[i*uiWidth] = iDCValue;
    }
  }
<<<<<<< HEAD
  else if (iNumIntraNeighbor == iTotalUnits)//所有区域参考像素都可获得　则用图中对应位置的像素值填充
=======
  else if (iNumIntraNeighbor == iTotalUnits)
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
  {
    // Fill top-left border and top and top right with rec. samples
    piRoiTemp = piRoiOrigin - iPicStride - 1;

<<<<<<< HEAD
    for (i=0; i<uiWidth; i++)//上方参考像素
=======
    for (i=0; i<uiWidth; i++)
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
    {
#if O0043_BEST_EFFORT_DECODING
      piIntraTemp[i] = piRoiTemp[i] << bitDepthDelta;
#else
      piIntraTemp[i] = piRoiTemp[i];
#endif
    }

    // Fill left and below left border with rec. samples
    piRoiTemp = piRoiOrigin - 1;

<<<<<<< HEAD
    for (i=1; i<uiHeight; i++)//左侧参考像素
=======
    for (i=1; i<uiHeight; i++)
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
    {
#if O0043_BEST_EFFORT_DECODING
      piIntraTemp[i*uiWidth] = (*(piRoiTemp)) << bitDepthDelta;
#else
      piIntraTemp[i*uiWidth] = *(piRoiTemp);
#endif
      piRoiTemp += iPicStride;
    }
  }
<<<<<<< HEAD
  else // reference samples are partially available//部分区域参考像素可以获得
  {
    // all above units have "unitWidth" samples each, all left/below-left units have "unitHeight" samples each
    const Int  iTotalSamples = (iLeftUnits * unitHeight) + ((iAboveUnits + 1) * unitWidth);
    Pel  piIntraLine[5 * MAX_CU_SIZE];//5 * MAX_CU_SIZE为可能的最大的iTotalSamples
=======
  else // reference samples are partially available
  {
    // all above units have "unitWidth" samples each, all left/below-left units have "unitHeight" samples each
    const Int  iTotalSamples = (iLeftUnits * unitHeight) + ((iAboveUnits + 1) * unitWidth);
    Pel  piIntraLine[5 * MAX_CU_SIZE];
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
    Pel  *piIntraLineTemp;
    const Bool *pbNeighborFlags;


    // Initialize
    for (i=0; i<iTotalSamples; i++)
    {
      piIntraLine[i] = iDCValue;
    }

    // Fill top-left sample
    piRoiTemp = piRoiOrigin - iPicStride - 1;
<<<<<<< HEAD
    piIntraLineTemp = piIntraLine + (iLeftUnits * unitHeight);//储存左上参考像素位置
    pbNeighborFlags = bNeighborFlags + iLeftUnits;//左上参考像素可否获得
    if (*pbNeighborFlags)//左上参考像素可以获得
=======
    piIntraLineTemp = piIntraLine + (iLeftUnits * unitHeight);
    pbNeighborFlags = bNeighborFlags + iLeftUnits;
    if (*pbNeighborFlags)
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
    {
#if O0043_BEST_EFFORT_DECODING
      Pel topLeftVal=piRoiTemp[0] << bitDepthDelta;
#else
      Pel topLeftVal=piRoiTemp[0];
#endif
      for (i=0; i<unitWidth; i++)
      {
<<<<<<< HEAD
        piIntraLineTemp[i] = topLeftVal;//获取左上参考像素值　
=======
        piIntraLineTemp[i] = topLeftVal;
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
      }
    }

    // Fill left & below-left samples (downwards)
<<<<<<< HEAD
    piRoiTemp += iPicStride;//从左上移动到左侧位置
    piIntraLineTemp--;
    pbNeighborFlags--;

    for (j=0; j<iLeftUnits; j++)//遍历左侧参考像素（以单位块为单位）
    {
      if (*pbNeighborFlags)//TU当前左侧单位块存在相邻单位块
      {
        for (i=0; i<unitHeight; i++)//将相邻的像素　赋值给参考像素
=======
    piRoiTemp += iPicStride;
    piIntraLineTemp--;
    pbNeighborFlags--;

    for (j=0; j<iLeftUnits; j++)
    {
      if (*pbNeighborFlags)
      {
        for (i=0; i<unitHeight; i++)
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
        {
#if O0043_BEST_EFFORT_DECODING
          piIntraLineTemp[-i] = piRoiTemp[i*iPicStride] << bitDepthDelta;
#else
          piIntraLineTemp[-i] = piRoiTemp[i*iPicStride];
#endif
        }
      }
<<<<<<< HEAD
      piRoiTemp += unitHeight*iPicStride;//处理下一个单位块的参考像素
=======
      piRoiTemp += unitHeight*iPicStride;
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
      piIntraLineTemp -= unitHeight;
      pbNeighborFlags--;
    }

    // Fill above & above-right samples (left-to-right) (each unit has "unitWidth" samples)
<<<<<<< HEAD
    piRoiTemp = piRoiOrigin - iPicStride;//从左上移动到上方位置
    // offset line buffer by iNumUints2*unitHeight (for left/below-left) + unitWidth (for above-left)
    piIntraLineTemp = piIntraLine + (iLeftUnits * unitHeight) + unitWidth;//上方第一个参考像素位置（从left-to-right）
    pbNeighborFlags = bNeighborFlags + iLeftUnits + 1;//TU上方第一个小块是否存在相邻小块
    for (j=0; j<iAboveUnits; j++)//遍历上方参考像素（以单位块为单位）
    {
      if (*pbNeighborFlags)//TU当前上方单位块存在相邻单位块
      {
        for (i=0; i<unitWidth; i++)//将相邻的像素　赋值给参考像素
=======
    piRoiTemp = piRoiOrigin - iPicStride;
    // offset line buffer by iNumUints2*unitHeight (for left/below-left) + unitWidth (for above-left)
    piIntraLineTemp = piIntraLine + (iLeftUnits * unitHeight) + unitWidth;
    pbNeighborFlags = bNeighborFlags + iLeftUnits + 1;
    for (j=0; j<iAboveUnits; j++)
    {
      if (*pbNeighborFlags)
      {
        for (i=0; i<unitWidth; i++)
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
        {
#if O0043_BEST_EFFORT_DECODING
          piIntraLineTemp[i] = piRoiTemp[i] << bitDepthDelta;
#else
          piIntraLineTemp[i] = piRoiTemp[i];
#endif
        }
      }
<<<<<<< HEAD
      piRoiTemp += unitWidth;//处理下一个单位块的参考像素
=======
      piRoiTemp += unitWidth;
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
      piIntraLineTemp += unitWidth;
      pbNeighborFlags++;
    }

    // Pad reference samples when necessary
<<<<<<< HEAD
    Int iCurrJnit = 0;//当前已填充到的参考像素位置（以单位块为单位）
    Pel  *piIntraLineCur   = piIntraLine;
    const UInt piIntraLineTopRowOffset = iLeftUnits * (unitHeight - unitWidth);//用于计算piIntraLineNext的位置　左侧参考像素产生的补偿量

    if (!bNeighborFlags[0])//不可获得参考像素左下小块
=======
    Int iCurrJnit = 0;
    Pel  *piIntraLineCur   = piIntraLine;
    const UInt piIntraLineTopRowOffset = iLeftUnits * (unitHeight - unitWidth);

    if (!bNeighborFlags[0])
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
    {
      // very bottom unit of bottom-left; at least one unit will be valid.
      {
        Int   iNext = 1;
<<<<<<< HEAD
        while (iNext < iTotalUnits && !bNeighborFlags[iNext])//从左下到左上到右上找到第一个可获得的参考像素小块
        {
          iNext++;
        }
        Pel *piIntraLineNext = piIntraLine + ((iNext < iLeftUnits) ? (iNext * unitHeight) : (piIntraLineTopRowOffset + (iNext * unitWidth)));//第一个可获得的参考像素小块的起始位置
        const Pel refSample = *piIntraLineNext;//第一个可直接获得的参考像素值
        // Pad unavailable samples with new value
        Int iNextOrTop = std::min<Int>(iNext, iLeftUnits);
        // fill left column
        while (iCurrJnit < iNextOrTop)//从左下用第一个可直接获得的参考像素值填充左侧参考像素值（直到第一个可直接获得的参考像素值）
=======
        while (iNext < iTotalUnits && !bNeighborFlags[iNext])
        {
          iNext++;
        }
        Pel *piIntraLineNext = piIntraLine + ((iNext < iLeftUnits) ? (iNext * unitHeight) : (piIntraLineTopRowOffset + (iNext * unitWidth)));
        const Pel refSample = *piIntraLineNext;
        // Pad unavailable samples with new value
        Int iNextOrTop = std::min<Int>(iNext, iLeftUnits);
        // fill left column
        while (iCurrJnit < iNextOrTop)
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
        {
          for (i=0; i<unitHeight; i++)
          {
            piIntraLineCur[i] = refSample;
          }
          piIntraLineCur += unitHeight;
          iCurrJnit++;
        }
        // fill top row
<<<<<<< HEAD
        while (iCurrJnit < iNext)//从左上用第一个可直接获得的参考像素值填充上方参考像素值（直到第一个可直接获得的参考像素值）
=======
        while (iCurrJnit < iNext)
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
        {
          for (i=0; i<unitWidth; i++)
          {
            piIntraLineCur[i] = refSample;
          }
          piIntraLineCur += unitWidth;
          iCurrJnit++;
        }
      }
    }

    // pad all other reference samples.
<<<<<<< HEAD
    while (iCurrJnit < iTotalUnits)//填充所有参考像素值（以单位块为单位）
    {
      if (!bNeighborFlags[iCurrJnit]) // samples not available//该小块不存在相邻小块
      {
        {
          const Int numSamplesInCurrUnit = (iCurrJnit >= iLeftUnits) ? unitWidth : unitHeight;
          const Pel refSample = *(piIntraLineCur-1);//用邻近的可获得的参考像素 (-1方向)
          for (i=0; i<numSamplesInCurrUnit; i++)//用邻近的可获得的参考像素填充
=======
    while (iCurrJnit < iTotalUnits)
    {
      if (!bNeighborFlags[iCurrJnit]) // samples not available
      {
        {
          const Int numSamplesInCurrUnit = (iCurrJnit >= iLeftUnits) ? unitWidth : unitHeight;
          const Pel refSample = *(piIntraLineCur-1);
          for (i=0; i<numSamplesInCurrUnit; i++)
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
          {
            piIntraLineCur[i] = refSample;
          }
          piIntraLineCur += numSamplesInCurrUnit;
          iCurrJnit++;
        }
      }
<<<<<<< HEAD
      else//否则直接处理下一小块
=======
      else
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
      {
        piIntraLineCur += (iCurrJnit >= iLeftUnits) ? unitWidth : unitHeight;
        iCurrJnit++;
      }
    }

    // Copy processed samples
<<<<<<< HEAD
    //将处理好的参考像素值赋给 piIntraTemp
=======

>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
    piIntraLineTemp = piIntraLine + uiHeight + unitWidth - 2;
    // top left, top and top right samples
    for (i=0; i<uiWidth; i++)
    {
      piIntraTemp[i] = piIntraLineTemp[i];
    }

    piIntraLineTemp = piIntraLine + uiHeight - 1;
    for (i=1; i<uiHeight; i++)
    {
      piIntraTemp[i*uiWidth] = piIntraLineTemp[-i];
    }
  }
}

Bool TComPrediction::filteringIntraReferenceSamples(const ComponentID compID, UInt uiDirMode, UInt uiTuChWidth, UInt uiTuChHeight, const ChromaFormat chFmt, const Bool intraReferenceSmoothingDisabled)
<<<<<<< HEAD
{//根据帧内预测模式和Tu大小判断是否需要对参考像素滤波
  Bool bFilter;

  if (!filterIntraReferenceSamples(toChannelType(compID), chFmt, intraReferenceSmoothingDisabled))//为亮度分量或色度格式为４４４　且intraReferenceSmoothingDisabled为０才进一步判断
=======
{
  Bool bFilter;

  if (!filterIntraReferenceSamples(toChannelType(compID), chFmt, intraReferenceSmoothingDisabled))
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
  {
    bFilter=false;
  }
  else
  {
    assert(uiTuChWidth>=4 && uiTuChHeight>=4 && uiTuChWidth<128 && uiTuChHeight<128);

<<<<<<< HEAD
    if (uiDirMode == DC_IDX)//DC模式不需要对参考像素滤波
    {
      bFilter=false; //no smoothing for DC or LM chroma
    }
    else//根据Tu块大小　选取符合条件的帧内预测模式对参考像素滤波　（32*32除去10,26 16*16除去９,11,25,27 8*8仅对2,18,34及planar)
=======
    if (uiDirMode == DC_IDX)
    {
      bFilter=false; //no smoothing for DC or LM chroma
    }
    else
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
    {
      Int diff = min<Int>(abs((Int) uiDirMode - HOR_IDX), abs((Int)uiDirMode - VER_IDX));
      UInt sizeIndex=g_aucConvertToBit[uiTuChWidth];
      assert(sizeIndex < MAX_INTRA_FILTER_DEPTHS);
      bFilter = diff > m_aucIntraFilter[toChannelType(compID)][sizeIndex];
    }
  }
  return bFilter;
}

<<<<<<< HEAD
Bool isAboveLeftAvailable( TComDataCU* pcCU, UInt uiPartIdxLT )//位置为uiPartIdxLT的小块左上小块是否可获得
=======
Bool isAboveLeftAvailable( TComDataCU* pcCU, UInt uiPartIdxLT )
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
{
  Bool bAboveLeftFlag;
  UInt uiPartAboveLeft;
  TComDataCU* pcCUAboveLeft = pcCU->getPUAboveLeft( uiPartAboveLeft, uiPartIdxLT );
<<<<<<< HEAD
  if(pcCU->getSlice()->getPPS()->getConstrainedIntraPred())//约束为帧内
=======
  if(pcCU->getSlice()->getPPS()->getConstrainedIntraPred())
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
  {
    bAboveLeftFlag = ( pcCUAboveLeft && pcCUAboveLeft->isIntra( uiPartAboveLeft ) );
  }
  else
  {
    bAboveLeftFlag = (pcCUAboveLeft ? true : false);
  }
  return bAboveLeftFlag;
}

Int isAboveAvailable( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxRT, Bool *bValidFlags )
<<<<<<< HEAD
{//判断从uiPartIdxLT到uiPartIdxRT位置的小块上方是否存在相邻小块并标识　并返回相邻小块个数
  const UInt uiRasterPartBegin = g_auiZscanToRaster[uiPartIdxLT];//Pu左上位置
  const UInt uiRasterPartEnd = g_auiZscanToRaster[uiPartIdxRT]+1;// Pu右上位置
=======
{
  const UInt uiRasterPartBegin = g_auiZscanToRaster[uiPartIdxLT];
  const UInt uiRasterPartEnd = g_auiZscanToRaster[uiPartIdxRT]+1;
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
  const UInt uiIdxStep = 1;
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;

<<<<<<< HEAD
  for ( UInt uiRasterPart = uiRasterPartBegin; uiRasterPart < uiRasterPartEnd; uiRasterPart += uiIdxStep )//遍历Pu内上方所有小块
  {
    UInt uiPartAbove;
    TComDataCU* pcCUAbove = pcCU->getPUAbove( uiPartAbove, g_auiRasterToZscan[uiRasterPart] );
    if(pcCU->getSlice()->getPPS()->getConstrainedIntraPred())//要求参考像素也为帧内预测得到
    {
      if ( pcCUAbove && pcCUAbove->isIntra( uiPartAbove ) )//PU内该小块存在相邻小块
      {
        iNumIntra++;//相邻小块数+1
        *pbValidFlags = true;//该位置是否相邻小块标示置为真
      }
      else//反之　置为加
=======
  for ( UInt uiRasterPart = uiRasterPartBegin; uiRasterPart < uiRasterPartEnd; uiRasterPart += uiIdxStep )
  {
    UInt uiPartAbove;
    TComDataCU* pcCUAbove = pcCU->getPUAbove( uiPartAbove, g_auiRasterToZscan[uiRasterPart] );
    if(pcCU->getSlice()->getPPS()->getConstrainedIntraPred())
    {
      if ( pcCUAbove && pcCUAbove->isIntra( uiPartAbove ) )
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
      {
        *pbValidFlags = false;
      }
    }
<<<<<<< HEAD
    else//不要求为帧内
=======
    else
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
    {
      if (pcCUAbove)
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
<<<<<<< HEAD
    pbValidFlags++;//若以左上参考像素为０位置点　则左侧参考像素为负　上方参考像素为正
=======
    pbValidFlags++;
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
  }
  return iNumIntra;
}

Int isLeftAvailable( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxLB, Bool *bValidFlags )
<<<<<<< HEAD
{//同上　不在叙述
=======
{
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
  const UInt uiRasterPartBegin = g_auiZscanToRaster[uiPartIdxLT];
  const UInt uiRasterPartEnd = g_auiZscanToRaster[uiPartIdxLB]+1;
  const UInt uiIdxStep = pcCU->getPic()->getNumPartInCtuWidth();
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;

  for ( UInt uiRasterPart = uiRasterPartBegin; uiRasterPart < uiRasterPartEnd; uiRasterPart += uiIdxStep )
  {
    UInt uiPartLeft;
    TComDataCU* pcCULeft = pcCU->getPULeft( uiPartLeft, g_auiRasterToZscan[uiRasterPart] );
    if(pcCU->getSlice()->getPPS()->getConstrainedIntraPred())
    {
      if ( pcCULeft && pcCULeft->isIntra( uiPartLeft ) )
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    else
    {
      if ( pcCULeft )
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    pbValidFlags--; // opposite direction
  }

  return iNumIntra;
}

Int isAboveRightAvailable( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxRT, Bool *bValidFlags )
<<<<<<< HEAD
{//基本同上　不在详述
  const UInt uiNumUnitsInPU = g_auiZscanToRaster[uiPartIdxRT] - g_auiZscanToRaster[uiPartIdxLT] + 1;//该PU一行的宽（以单位块为单位）
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;

  for ( UInt uiOffset = 1; uiOffset <= uiNumUnitsInPU; uiOffset++ )//该PU块外右上块距离PU块内右上块的水平距离（以单位块为单位）
=======
{
  const UInt uiNumUnitsInPU = g_auiZscanToRaster[uiPartIdxRT] - g_auiZscanToRaster[uiPartIdxLT] + 1;
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;

  for ( UInt uiOffset = 1; uiOffset <= uiNumUnitsInPU; uiOffset++ )
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
  {
    UInt uiPartAboveRight;
    TComDataCU* pcCUAboveRight = pcCU->getPUAboveRight( uiPartAboveRight, uiPartIdxRT, uiOffset );
    if(pcCU->getSlice()->getPPS()->getConstrainedIntraPred())
    {
      if ( pcCUAboveRight && pcCUAboveRight->isIntra( uiPartAboveRight ) )
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    else
    {
      if ( pcCUAboveRight )
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    pbValidFlags++;
  }

  return iNumIntra;
}

Int isBelowLeftAvailable( TComDataCU* pcCU, UInt uiPartIdxLT, UInt uiPartIdxLB, Bool *bValidFlags )
<<<<<<< HEAD
{//同上　不在叙述
=======
{
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
  const UInt uiNumUnitsInPU = (g_auiZscanToRaster[uiPartIdxLB] - g_auiZscanToRaster[uiPartIdxLT]) / pcCU->getPic()->getNumPartInCtuWidth() + 1;
  Bool *pbValidFlags = bValidFlags;
  Int iNumIntra = 0;

  for ( UInt uiOffset = 1; uiOffset <= uiNumUnitsInPU; uiOffset++ )
  {
    UInt uiPartBelowLeft;
    TComDataCU* pcCUBelowLeft = pcCU->getPUBelowLeft( uiPartBelowLeft, uiPartIdxLB, uiOffset );
    if(pcCU->getSlice()->getPPS()->getConstrainedIntraPred())
    {
      if ( pcCUBelowLeft && pcCUBelowLeft->isIntra( uiPartBelowLeft ) )
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    else
    {
      if ( pcCUBelowLeft )
      {
        iNumIntra++;
        *pbValidFlags = true;
      }
      else
      {
        *pbValidFlags = false;
      }
    }
    pbValidFlags--; // opposite direction
  }

  return iNumIntra;
}
//! \}
