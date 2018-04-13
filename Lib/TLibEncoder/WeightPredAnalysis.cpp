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

/** \file     WeightPredAnalysis.cpp
    \brief    weighted prediction encoder class
*/

#include "../TLibCommon/CommonDef.h"
#include "../TLibCommon/TComSlice.h"
#include "../TLibCommon/TComPic.h"
#include "../TLibCommon/TComPicYuv.h"
#include "WeightPredAnalysis.h"

#define ABS(a)    ((a) < 0 ? - (a) : (a))
#define DTHRESH (0.99)

WeightPredAnalysis::WeightPredAnalysis()//构造函数 初始化赋值
{
  for ( UInt lst =0 ; lst<NUM_REF_PIC_LIST_01 ; lst++ )//遍历list0 lsit1参考图像列表
  {
    for ( Int iRefIdx=0 ; iRefIdx<MAX_NUM_REF ; iRefIdx++ )//遍历参考图像列表中最大参考图像数
    {
      for ( Int comp=0 ; comp<MAX_NUM_COMPONENT ;comp++ )//遍历所有分量
      {
        WPScalingParam  *pwp   = &(m_wp[lst][iRefIdx][comp]);
        pwp->bPresentFlag      = false;
        pwp->uiLog2WeightDenom = 0;
        pwp->iWeight           = 1;
        pwp->iOffset           = 0;//初始化赋值
      }
    }
  }
}


//! calculate AC and DC values for current original image
Void WeightPredAnalysis::xCalcACDCParamSlice(TComSlice *const slice)//计算当前原始图像的AC DC值
{
  //===== calculate AC/DC value =====
  TComPicYuv*   pPic = slice->getPic()->getPicYuvOrg();//原始图像

  WPACDCParam weightACDCParam[MAX_NUM_COMPONENT];

  for(Int componentIndex = 0; componentIndex < pPic->getNumberValidComponents(); componentIndex++)//遍历所有分量
  {
    const ComponentID compID = ComponentID(componentIndex);

    // calculate DC/AC value for channel

    const Int iStride = pPic->getStride(compID);
    const Int iWidth  = pPic->getWidth(compID);
    const Int iHeight = pPic->getHeight(compID);

    const Int iSample = iWidth*iHeight;//总像素数

    Int64 iOrgDC = 0;
    {
      const Pel *pPel = pPic->getAddr(compID);//原始图像的起始位置

      for(Int y = 0; y < iHeight; y++, pPel+=iStride )
      {
        for(Int x = 0; x < iWidth; x++ )
        {
          iOrgDC += (Int)( pPel[x] );//所有像素值之和
        }
      }
    }

    const Int64 iOrgNormDC = ((iOrgDC+(iSample>>1)) / iSample);//直流分量(像素平均值)

    Int64 iOrgAC = 0;
    {
      const Pel *pPel = pPic->getAddr(compID);

      for(Int y = 0; y < iHeight; y++, pPel += iStride )
      {
        for(Int x = 0; x < iWidth; x++ )
        {
          iOrgAC += abs( (Int)pPel[x] - (Int)iOrgNormDC );//交流分量(所有像素与平均像素值之差绝对值的和)
        }
      }
    }

    const Int fixedBitShift = (slice->getSPS()->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag())?RExt__PREDICTION_WEIGHTING_ANALYSIS_DC_PRECISION:0;
    weightACDCParam[compID].iDC = (((iOrgDC<<fixedBitShift)+(iSample>>1)) / iSample);
    weightACDCParam[compID].iAC = iOrgAC;//最终的DC AC值
  }

  slice->setWpAcDcParam(weightACDCParam);//设置slice加权预测的AC DC值
}


//! check weighted pred or non-weighted pred
Void  WeightPredAnalysis::xCheckWPEnable(TComSlice *const slice)
{
  const TComPicYuv *pPic = slice->getPic()->getPicYuvOrg();

  Int iPresentCnt = 0;
  for ( UInt lst=0 ; lst<NUM_REF_PIC_LIST_01 ; lst++ )//遍历list0 list0参考图像列表
  {
    for ( Int iRefIdx=0 ; iRefIdx<MAX_NUM_REF ; iRefIdx++ )//遍历参考图像列表中所有参考图像
    {
      for(Int componentIndex = 0; componentIndex < pPic->getNumberValidComponents(); componentIndex++)//遍历所有分量
      {
        WPScalingParam  *pwp = &(m_wp[lst][iRefIdx][componentIndex]);//加权参数
        iPresentCnt += (Int)pwp->bPresentFlag;//使用权重的参考图像(分量)数
      }
    }
  }

  if(iPresentCnt==0)//若没有参考图像(分量)使用权重
  {
    slice->setTestWeightPred(false);
    slice->setTestWeightBiPred(false);//则不使用加权预测

    for ( UInt lst=0 ; lst<NUM_REF_PIC_LIST_01 ; lst++ )
    {
      for ( Int iRefIdx=0 ; iRefIdx<MAX_NUM_REF ; iRefIdx++ )
      {
        for(Int componentIndex = 0; componentIndex < pPic->getNumberValidComponents(); componentIndex++)
        {
          WPScalingParam  *pwp = &(m_wp[lst][iRefIdx][componentIndex]);

          pwp->bPresentFlag      = false;
          pwp->uiLog2WeightDenom = 0;
          pwp->iWeight           = 1;
          pwp->iOffset           = 0;//设置默认的权重参数
        }
      }
    }
    slice->setWpScaling( m_wp );//设置slice为默认的权重参数
  }
  else//若存在参考图像(分量)使用权重
  {
    slice->setTestWeightPred(slice->getPPS()->getUseWP());
    slice->setTestWeightBiPred(slice->getPPS()->getWPBiPred());//则由PPS信息得到是否使用加权预测
  }
}


//! estimate wp tables for explicit wp
Void WeightPredAnalysis::xEstimateWPParamSlice(TComSlice *const slice)
{//iDenom为运算前的左移移位量 可以提高加权预测的运算精度
  Int  iDenom         = 6;
  Bool validRangeFlag = false;

  if(slice->getNumRefIdx(REF_PIC_LIST_0)>3)
  {
    iDenom = 7;
  }//确定初始iDenom值

  do
  {
    validRangeFlag = xUpdatingWPParameters(slice, iDenom);//用给定的Denom计算加权预测参数
    if (!validRangeFlag)
    {
      iDenom--; // decrement to satisfy the range limitation
    }
  } while (validRangeFlag == false);//依次递减遍历不同的Denom值 直至加权权重在允许的范围内

  // selecting whether WP is used, or not (fast search)
  // NOTE: This is not operating on a slice, but the entire picture.
  xSelectWP(slice, iDenom);//判断是否选择加权预测

  slice->setWpScaling( m_wp );//设置slice的参考图像的加权参数
}


//! update wp tables for explicit wp w.r.t range limitation
Bool WeightPredAnalysis::xUpdatingWPParameters(TComSlice *const slice, const Int log2Denom)//计算加权预测中的加权参数
{
  const Int  numComp                    = slice->getPic()->getPicYuvOrg()->getNumberValidComponents();
  const Bool bUseHighPrecisionWeighting = slice->getSPS()->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag();
  const Int numPredDir                  = slice->isInterP() ? 1 : 2;//P帧只有一个预测方向 B帧两个预测方向 I帧无法使用加权预测

  assert (numPredDir <= Int(NUM_REF_PIC_LIST_01));

  for ( Int refList = 0; refList < numPredDir; refList++ )//遍历所有预测方向
  {
    const RefPicList eRefPicList = ( refList ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );//遍历对应预测方向的参考图像列表

    for ( Int refIdxTemp = 0; refIdxTemp < slice->getNumRefIdx(eRefPicList); refIdxTemp++ )//遍历参考图像列表中的参考图像
    {
      WPACDCParam *currWeightACDCParam, *refWeightACDCParam;
      slice->getWpAcDcParam(currWeightACDCParam);
      slice->getRefPic(eRefPicList, refIdxTemp)->getSlice(0)->getWpAcDcParam(refWeightACDCParam);

      for ( Int comp = 0; comp < numComp; comp++ )
      {
        const ComponentID compID        = ComponentID(comp);
        const Int         bitDepth      = slice->getSPS()->getBitDepth(toChannelType(compID));
        const Int         range         = bUseHighPrecisionWeighting ? (1<<bitDepth)/2 : 128;
        const Int         realLog2Denom = log2Denom + (bUseHighPrecisionWeighting ? RExt__PREDICTION_WEIGHTING_ANALYSIS_DC_PRECISION : (bitDepth - 8));
        const Int         realOffset    = ((Int)1<<(realLog2Denom-1));//取整偏移量

        // current frame
        const Int64 currDC = currWeightACDCParam[comp].iDC;
        const Int64 currAC = currWeightACDCParam[comp].iAC;//当前帧的DC AC值
        // reference frame
        const Int64 refDC  = refWeightACDCParam[comp].iDC;
        const Int64 refAC  = refWeightACDCParam[comp].iAC;//参考帧的DC AC值

        // calculating iWeight and iOffset params//用图像的AC DC值计算该参考图像该分量的权重和偏移量
        const Double dWeight = (refAC==0) ? (Double)1.0 : Clip3( -16.0, 15.0, ((Double)currAC / (Double)refAC) );
        const Int weight     = (Int)( 0.5 + dWeight * (Double)(1<<log2Denom) );
        const Int offset     = (Int)( ((currDC<<log2Denom) - ((Int64)weight * refDC) + (Int64)realOffset) >> realLog2Denom );

        Int clippedOffset;
        if(isChroma(compID)) // Chroma offset range limination//限制色度偏移量的范围
        {
          const Int pred        = ( range - ( ( range*weight)>>(log2Denom) ) );//预测offset值
          const Int deltaOffset = Clip3( -4*range, 4*range-1, (offset - pred) ); // signed 10bit //色度分量偏移量的表示是使用实际offset值与预测offset值的差值来表示 所以要限制deltaoffset值的大小 *4是因为(8bit->10bit)

          clippedOffset = Clip3( -range, range-1, (deltaOffset + pred) );  // signed 8bit//限制实际的offset值的大小
        }
        else // Luma offset range limitation//限制亮度偏移量的范围
        {
          clippedOffset = Clip3( -range, range-1, offset);
        }

        // Weighting factor limitation//限制权重的范围
        const Int defaultWeight = (1<<log2Denom);//默认权重为1
        const Int deltaWeight   = (defaultWeight - weight);//同样weight值也用与默认权重的deltaWeight值来表示

        if(deltaWeight >= range || deltaWeight < -range)//超出范围 则直接返回false
        {
          return false;
        }

        m_wp[refList][refIdxTemp][comp].bPresentFlag      = true;
        m_wp[refList][refIdxTemp][comp].iWeight           = weight;
        m_wp[refList][refIdxTemp][comp].iOffset           = clippedOffset;
        m_wp[refList][refIdxTemp][comp].uiLog2WeightDenom = log2Denom;//设置加权预测参数
      }
    }
  }
  return true;//满足范围要求返回真
}


//! select whether weighted pred enables or not.
Bool WeightPredAnalysis::xSelectWP(TComSlice *const slice, const Int log2Denom)
{
        TComPicYuv *const pPic                                = slice->getPic()->getPicYuvOrg();
  const Int               iDefaultWeight                      = ((Int)1<<log2Denom);
  const Int               iNumPredDir                         = slice->isInterP() ? 1 : 2;
  const Bool              useHighPrecisionPredictionWeighting = slice->getSPS()->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag();

  assert (iNumPredDir <= Int(NUM_REF_PIC_LIST_01));

  for ( Int iRefList = 0; iRefList < iNumPredDir; iRefList++ )//遍历所有预测方向
  {
    const RefPicList eRefPicList = ( iRefList ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );//遍历对应预测方向的参考图像列表

    for ( Int iRefIdxTemp = 0; iRefIdxTemp < slice->getNumRefIdx(eRefPicList); iRefIdxTemp++ )//遍历参考图像列表中的参考图像
    {
      Int64 iSADWP = 0, iSADnoWP = 0;

      for(Int comp=0; comp<pPic->getNumberValidComponents(); comp++)//遍历所有分量
      {
        const ComponentID  compID     = ComponentID(comp);
              Pel         *pOrg       = pPic->getAddr(compID);
              Pel         *pRef       = slice->getRefPic(eRefPicList, iRefIdxTemp)->getPicYuvRec()->getAddr(compID);
        const Int          iOrgStride = pPic->getStride(compID);
        const Int          iRefStride = slice->getRefPic(eRefPicList, iRefIdxTemp)->getPicYuvRec()->getStride(compID);
        const Int          iWidth     = pPic->getWidth(compID);
        const Int          iHeight    = pPic->getHeight(compID);
        const Int          bitDepth   = slice->getSPS()->getBitDepth(toChannelType(compID));

        // calculate SAD costs with/without wp for luma//计算使用加权预测和不使用加权预测的SAD误差
        iSADWP   += xCalcSADvalueWP(bitDepth, pOrg, pRef, iWidth, iHeight, iOrgStride, iRefStride, log2Denom, m_wp[iRefList][iRefIdxTemp][compID].iWeight, m_wp[iRefList][iRefIdxTemp][compID].iOffset, useHighPrecisionPredictionWeighting);
        iSADnoWP += xCalcSADvalueWP(bitDepth, pOrg, pRef, iWidth, iHeight, iOrgStride, iRefStride, log2Denom, iDefaultWeight, 0, useHighPrecisionPredictionWeighting);
      }

      const Double dRatio = ((Double)iSADWP / (Double)iSADnoWP);//使用加权预测和不使用加权预测的SAD比值 (SAD值越小说明误差越小)
      if(dRatio >= (Double)DTHRESH)//比值大于阈值(0.99)说明不使用加权预测时误差更小
      {
        for(Int comp=0; comp<pPic->getNumberValidComponents(); comp++)
        {
          m_wp[iRefList][iRefIdxTemp][comp].bPresentFlag      = false;
          m_wp[iRefList][iRefIdxTemp][comp].iOffset           = 0;
          m_wp[iRefList][iRefIdxTemp][comp].iWeight           = iDefaultWeight;
          m_wp[iRefList][iRefIdxTemp][comp].uiLog2WeightDenom = log2Denom;//设置该参考图像不使用加权预测
        }
      }
    }
  }

  return true;
}


//! calculate SAD values for both WP version and non-WP version.
Int64 WeightPredAnalysis::xCalcSADvalueWP(const Int   bitDepth,
                                          const Pel  *pOrgPel,
                                          const Pel  *pRefPel,
                                          const Int   iWidth,
                                          const Int   iHeight,
                                          const Int   iOrgStride,
                                          const Int   iRefStride,
                                          const Int   iLog2Denom,
                                          const Int   iWeight,
                                          const Int   iOffset,
                                          const Bool  useHighPrecisionPredictionWeighting)//计算原图像与加权后的参考图像间的平均SAD误差
{
  const Int64 iSize          = iWidth*iHeight;//像素数
  const Int64 iRealLog2Denom = useHighPrecisionPredictionWeighting ? iLog2Denom : (iLog2Denom + (bitDepth - 8));

  Int64 iSAD = 0;
  for( Int y = 0; y < iHeight; y++ )//遍历图像中所有像素
  {
    for( Int x = 0; x < iWidth; x++ )
    {
      iSAD += ABS(( ((Int64)pOrgPel[x]<<(Int64)iLog2Denom) - ( (Int64)pRefPel[x] * (Int64)iWeight + ((Int64)iOffset<<iRealLog2Denom) ) ) );//原图像与加权后的参考图像间的DSAD之和
    }
    pOrgPel += iOrgStride;//下一行
    pRefPel += iRefStride;
  }

  return (iSAD/iSize);//平均SAD
}
