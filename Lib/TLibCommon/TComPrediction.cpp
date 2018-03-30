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

/** \file     TComPrediction.cpp
    \brief    prediction class
*/

#include <memory.h>
#include "TComPrediction.h"
#include "TComPic.h"
#include "TComTU.h"

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Tables
// ====================================================================================================================

const UChar TComPrediction::m_aucIntraFilter[MAX_NUM_CHANNEL_TYPE][MAX_INTRA_FILTER_DEPTHS] =
{
  { // Luma
    10, //4x4
    7, //8x8
    1, //16x16
    0, //32x32
    10, //64x64
  },
  { // Chroma
    10, //4xn
    7, //8xn
    1, //16xn
    0, //32xn
    10, //64xn
  }

};

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

TComPrediction::TComPrediction()//构造函数
: m_pLumaRecBuffer(0)
, m_iLumaRecStride(0)
{
  for(UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    for(UInt buf=0; buf<2; buf++)
    {
      m_piYuvExt[ch][buf] = NULL;
    }
  }
}

TComPrediction::~TComPrediction()
{
  destroy();
}

Void TComPrediction::destroy()//销毁对象
{
  for(UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    for(UInt buf=0; buf<NUM_PRED_BUF; buf++)
    {
      delete [] m_piYuvExt[ch][buf];
      m_piYuvExt[ch][buf] = NULL;
    }
  }

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_acYuvPred[i].destroy();
  }

  m_cYuvPredTemp.destroy();

  if( m_pLumaRecBuffer )
  {
    delete [] m_pLumaRecBuffer;
    m_pLumaRecBuffer = 0;
  }
  m_iLumaRecStride = 0;

  for (UInt i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; i++)
  {
    for (UInt j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; j++)
    {
      m_filteredBlock[i][j].destroy();
    }
    m_filteredBlockTmp[i].destroy();
  }
}

Void TComPrediction::initTempBuff(ChromaFormat chromaFormatIDC)
{//初始化ｂｕｆｆ
  // if it has been initialised before, but the chroma format has changed, release the memory and start again.
  if( m_piYuvExt[COMPONENT_Y][PRED_BUF_UNFILTERED] != NULL && m_cYuvPredTemp.getChromaFormat()!=chromaFormatIDC)
  {
    destroy();
  }

  if( m_piYuvExt[COMPONENT_Y][PRED_BUF_UNFILTERED] == NULL ) // check if first is null (in which case, nothing initialised yet)
  {
    Int extWidth  = MAX_CU_SIZE + 16;
    Int extHeight = MAX_CU_SIZE + 1;

    for (UInt i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; i++)
    {
      m_filteredBlockTmp[i].create(extWidth, extHeight + 7, chromaFormatIDC);
      for (UInt j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS; j++)
      {
        m_filteredBlock[i][j].create(extWidth, extHeight, chromaFormatIDC);
      }
    }

    m_iYuvExtSize = (MAX_CU_SIZE*2+1) * (MAX_CU_SIZE*2+1);
    for(UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
    {
      for(UInt buf=0; buf<NUM_PRED_BUF; buf++)
      {
        m_piYuvExt[ch][buf] = new Pel[ m_iYuvExtSize ];
      }
    }

    // new structure
    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      m_acYuvPred[i] .create( MAX_CU_SIZE, MAX_CU_SIZE, chromaFormatIDC );
    }

    m_cYuvPredTemp.create( MAX_CU_SIZE, MAX_CU_SIZE, chromaFormatIDC );
  }


  if (m_iLumaRecStride != (MAX_CU_SIZE>>1) + 1)
  {
    m_iLumaRecStride =  (MAX_CU_SIZE>>1) + 1;
    if (!m_pLumaRecBuffer)
    {
      m_pLumaRecBuffer = new Pel[ m_iLumaRecStride * m_iLumaRecStride ];
    }
  }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// Function for calculating DC value of the reference samples used in Intra prediction
//NOTE: Bit-Limit - 25-bit source
//帧内预测DC　　模式计算参考像素平均值
Pel TComPrediction::predIntraGetPredValDC( const Pel* pSrc, Int iSrcStride, UInt iWidth, UInt iHeight)
{//pSrc为将行参考像素和列参考像素合并在一起的一位数组；iSrcStride为步长
  assert(iWidth > 0 && iHeight > 0);　// 宽高一定大于零
  Int iInd, iSum = 0;　　　　
  Pel pDcVal;             

  for (iInd = 0;iInd < iWidth;iInd++)　　//遍历行参考像素，计算总和
  {
    iSum += pSrc[iInd-iSrcStride];
  }
  for (iInd = 0;iInd < iHeight;iInd++)　　//遍历列参考像素，计算总和
  {
    iSum += pSrc[iInd*iSrcStride-1];
  }

  pDcVal = (iSum + iWidth) / (iWidth + iHeight);　//计算参考像素值，iWidth的作用为向上取整

  return pDcVal;
}

// Function for deriving the angular Intra predictions

/** Function for deriving the simplified angular intra predictions.
 * \param bitDepth           bit depth
 * \param pSrc               pointer to reconstructed sample array
 * \param srcStride          the stride of the reconstructed sample array
 * \param pTrueDst           reference to pointer for the prediction sample array
 * \param dstStrideTrue      the stride of the prediction sample array
 * \param uiWidth            the width of the block
 * \param uiHeight           the height of the block
 * \param channelType        type of pel array (luma/chroma)
 * \param format             chroma format
 * \param dirMode            the intra prediction mode index
 * \param blkAboveAvailable  boolean indication if the block above is available
 * \param blkLeftAvailable   boolean indication if the block to the left is available
 * \param bEnableEdgeFilters indication whether to enable edge filters
 *
 * This function derives the prediction samples for the angular mode based on the prediction direction indicated by
 * the prediction mode index. The prediction direction is given by the displacement of the bottom row of the block and
 * the reference row above the block in the case of vertical prediction or displacement of the rightmost column
 * of the block and reference column left from the block in the case of the horizontal prediction. The displacement
 * is signalled at 1/32 pixel accuracy. When projection of the predicted pixel falls inbetween reference samples,
 * the predicted value for the pixel is linearly interpolated from the reference samples. All reference samples are taken
 * from the extended main reference.
 */
//NOTE: Bit-Limit - 25-bit source
//计算不同帧内角度预测模式中的像素预测值
Void TComPrediction::xPredIntraAng(       Int bitDepth,
                                    const Pel* pSrc,     Int srcStride,
                                          Pel* pTrueDst, Int dstStrideTrue,
                                          UInt uiWidth, UInt uiHeight, ChannelType channelType,
                                          UInt dirMode, const Bool bEnableEdgeFilters
                                  )
{
  Int width=Int(uiWidth);
  Int height=Int(uiHeight);

  // Map the mode index to main prediction direction and angle
  assert( dirMode != PLANAR_IDX ); //no planar　//帧内模式一定不为planar模式
  const Bool modeDC        = dirMode==DC_IDX;   //帧内模式是否为ＤＣ模式

  // Do the DC prediction
  if (modeDC)　//如果为ＤＣ模式
  {
    const Pel dcval = predIntraGetPredValDC(pSrc, srcStride, width, height);　//根据参考像素计算ＤＣ值

    for (Int y=height;y>0;y--, pTrueDst+=dstStrideTrue)　//将DC值赋给所有待预测像素
    {
      for (Int x=0; x<width;) // width is always a multiple of 4.
      {
        pTrueDst[x++] = dcval;
      }
    }
  }
  else // Do angular predictions//计算角度预测模式预测像素值
  {
    const Bool       bIsModeVer         = (dirMode >= 18);   //是否为垂直模式（18-34）
    const Int        intraPredAngleMode = (bIsModeVer) ? (Int)dirMode - VER_IDX :  -((Int)dirMode - HOR_IDX);//角度模式索引
    const Int        absAngMode         = abs(intraPredAngleMode);　　//索引绝对值
    const Int        signAng            = intraPredAngleMode < 0 ? -1 : 1;　//索引符号
    const Bool       edgeFilter         = bEnableEdgeFilters && isLuma(channelType) && (width <= MAXIMUM_INTRA_FILTERED_WIDTH) && (height <= MAXIMUM_INTRA_FILTERED_HEIGHT);

    // Set bitshifts and scale the angle parameter to block size
    static const Int angTable[9]    = {0,    2,    5,   9,  13,  17,  21,  26,  32};　　//不同模式的像素偏移量
    static const Int invAngTable[9] = {0, 4096, 1638, 910, 630, 482, 390, 315, 256}; // (256 * 32) / Angle　//方便后面使用节省计算量
    Int invAngle                    = invAngTable[absAngMode];　　　
    Int absAng                      = angTable[absAngMode];
    Int intraPredAngle              = signAng * absAng;　　//模式对应的像素偏移位置

    Pel* refMain;
    Pel* refSide;

    Pel  refAbove[2*MAX_CU_SIZE+1];
    Pel  refLeft[2*MAX_CU_SIZE+1];

    // Initialize the Main and Left reference array.
    if (intraPredAngle < 0)
    {
      const Int refMainOffsetPreScale = (bIsModeVer ? height : width ) - 1;　//辅参考像素投影至主参考像素时计算用　为辅参考像素长度
      const Int refMainOffset         = height - 1;  //参考像素数组前预留height个位置以备后用
      for (Int x=0;x<width+1;x++)　//初始化上方参考像素
      {
        refAbove[x+refMainOffset] = pSrc[x-srcStride-1];　
      }
      for (Int y=0;y<height+1;y++)　//初始化左侧参考像素
      {
        refLeft[y+refMainOffset] = pSrc[(y-1)*srcStride-1];
      }
      refMain = (bIsModeVer ? refAbove : refLeft)  + refMainOffset;　//若为垂直模式　则主参考像素为上方参考像素，辅参考像素为左侧参考像素；若为水平模式，则反之．
      refSide = (bIsModeVer ? refLeft  : refAbove) + refMainOffset;

      // Extend the Main reference to the left.
      Int invAngleSum    = 128;       // rounding for (shift by 8)//向上取整用　２＾（８－１）
      for (Int k=-1; k>(refMainOffsetPreScale+1)*intraPredAngle>>5; k--)  //在负角度模式中　按如下方法将辅参考像素选择性投影至主参考像素　负角度模式上方和左侧的参考像素都会用到
      {
        invAngleSum += invAngle;
        refMain[k] = refSide[invAngleSum>>8];
      }
    }
    else　//若为非负模式　则不需要将辅参考像素选择性投影至主参考像素　其他同上
    {
      for (Int x=0;x<2*width+1;x++)
      {
        refAbove[x] = pSrc[x-srcStride-1];
      }
      for (Int y=0;y<2*height+1;y++)
      {
        refLeft[y] = pSrc[(y-1)*srcStride-1];
      }
      refMain = bIsModeVer ? refAbove : refLeft ;　
      refSide = bIsModeVer ? refLeft  : refAbove;
    }

    // swap width/height if we are doing a horizontal mode:
    Pel tempArray[MAX_CU_SIZE*MAX_CU_SIZE];
    const Int dstStride = bIsModeVer ? dstStrideTrue : MAX_CU_SIZE;
    Pel *pDst = bIsModeVer ? pTrueDst : tempArray;
    if (!bIsModeVer)
    {
      std::swap(width, height);
    }

    if (intraPredAngle == 0)  // pure vertical or pure horizontal//模式10 26　由主参考像素得到预测像素值
    {
      for (Int y=0;y<height;y++)
      {
        for (Int x=0;x<width;x++)
        {
          pDst[y*dstStride+x] = refMain[x+1];
        }
      }

      if (edgeFilter)//当预测块小于32*32的luma块　标准水平和垂直模式（10 26）靠近辅参考像素处的预测像素需要滤波
      {
        for (Int y=0;y<height;y++)
        {
          pDst[y*dstStride] = Clip3 (0, ((1 << bitDepth) - 1), pDst[y*dstStride] + (( refSide[y+1] - refSide[0] ) >> 1) );
        }
      }
    }
    else
    {
      Pel *pDsty=pDst;

      for (Int y=0, deltaPos=intraPredAngle; y<height; y++, deltaPos+=intraPredAngle, pDsty+=dstStride)　//计算每一行或列预测像素对应到主参考像素的位置　精度为1/32像素
      {
        const Int deltaInt   = deltaPos >> 5;　　　　　　　//除以３２后整数部分
        const Int deltaFract = deltaPos & (32 - 1);　　　//除以３２后小数部分　（用与运算得到小数部分　）

        if (deltaFract)　//若对应主参考像素位置为小数
        {
          // Do linear filtering
          const Pel *pRM=refMain+deltaInt+1;
          Int lastRefMainPel=*pRM++;
          for (Int x=0;x<width;pRM++,x++)　//由主参考像素得到预测像素值
          {
            Int thisRefMainPel=*pRM;
            pDsty[x+0] = (Pel) ( ((32-deltaFract)*lastRefMainPel + deltaFract*thisRefMainPel +16) >> 5 );//1/32像素位置精度　线性加权得到预测像素值
            lastRefMainPel=thisRefMainPel;
          }
        }
        else　//若对应主参考像素正好为整数　则直接复制
        {
          // Just copy the integer samples
          for (Int x=0;x<width; x++)
          {
            pDsty[x] = refMain[x+deltaInt+1];
          }
        }
      }
    }

    // Flip the block if this is the horizontal mode
    if (!bIsModeVer) //若为水平模式　则翻转预测块　（pDst[]由假设为垂直模式得到）
    {
      for (Int y=0; y<height; y++)
      {
        for (Int x=0; x<width; x++)
        {
          pTrueDst[x*dstStrideTrue] = pDst[x];
        }
        pTrueDst++;
        pDst+=dstStride;
      }
    }
  }
}

//所有帧内模式中用给定的uiDirMode得到最后预测值
Void TComPrediction::predIntraAng( const ComponentID compID, UInt uiDirMode, Pel* piOrg /* Will be null for decoding */, UInt uiOrgStride, Pel* piPred, UInt uiStride, TComTU &rTu, const Bool bUseFilteredPredSamples, const Bool bUseLosslessDPCM )
{
  const ChannelType    channelType = toChannelType(compID);
  const TComRectangle &rect        = rTu.getRect(isLuma(compID) ? COMPONENT_Y : COMPONENT_Cb);
  const Int            iWidth      = rect.width;
  const Int            iHeight     = rect.height;

  assert( g_aucConvertToBit[ iWidth ] >= 0 ); //   4x  4
  assert( g_aucConvertToBit[ iWidth ] <= 5 ); // 128x128
  //assert( iWidth == iHeight  );

        Pel *pDst = piPred;//预测值首地址

  // get starting pixel in block
  const Int sw = (2 * iWidth + 1);

  if ( bUseLosslessDPCM ) //若使用DPCM模式 则只有垂直DPCM和水平DPCM两种情况
  {
    const Pel *ptrSrc = getPredictorPtr( compID, false );//得到参考像素值
    // Sample Adaptive intra-Prediction (SAP)
    if (uiDirMode==HOR_IDX)　//用水平方式进行预测，预测值的第一列用参考像素填充，如果piOrg(原始像素值)可以获得的话则将预测像素块的其他列用piOrg填充
    {
      // left column filled with reference samples
      // remaining columns filled with piOrg data (if available).
      for(Int y=0; y<iHeight; y++)
      {
        piPred[y*uiStride+0] = ptrSrc[(y+1)*sw];
      }
      if (piOrg!=0)
      {
        piPred+=1; // miss off first column
        for(Int y=0; y<iHeight; y++, piPred+=uiStride, piOrg+=uiOrgStride)
        {
          memcpy(piPred, piOrg, (iWidth-1)*sizeof(Pel));
        }
      }
    }
    else // VER_IDX　同上
    {
      // top row filled with reference samples
      // remaining rows filled with piOrd data (if available)
      for(Int x=0; x<iWidth; x++)
      {
        piPred[x] = ptrSrc[x+1];
      }
      if (piOrg!=0)
      {
        piPred+=uiStride; // miss off the first row
        for(Int y=1; y<iHeight; y++, piPred+=uiStride, piOrg+=uiOrgStride)
        {
          memcpy(piPred, piOrg, iWidth*sizeof(Pel));
        }
      }
    }
  }
  else
  {
    const Pel *ptrSrc = getPredictorPtr( compID, bUseFilteredPredSamples );//得到参考像素值

    if ( uiDirMode == PLANAR_IDX )　//计算planar模式预测值
    {
      xPredIntraPlanar( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight );
    }
    else
    {
      // Create the prediction
            TComDataCU *const pcCU              = rTu.getCU();
      const UInt              uiAbsPartIdx      = rTu.GetAbsPartIdxTU();
      const Bool              enableEdgeFilters = !(pcCU->isRDPCMEnabled(uiAbsPartIdx) && pcCU->getCUTransquantBypass(uiAbsPartIdx));　//从当前CU块信息判断是否边界滤波
#if O0043_BEST_EFFORT_DECODING
      const Int channelsBitDepthForPrediction = rTu.getCU()->getSlice()->getSPS()->getStreamBitDepth(channelType);
#else
      const Int channelsBitDepthForPrediction = rTu.getCU()->getSlice()->getSPS()->getBitDepth(channelType);　//位深信息
#endif
      xPredIntraAng( channelsBitDepthForPrediction, ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight, channelType, uiDirMode, enableEdgeFilters ); //角度模式计算预测值

      if( uiDirMode == DC_IDX )
      {
        xDCPredFiltering( ptrSrc+sw+1, sw, pDst, uiStride, iWidth, iHeight, channelType );　　//DC模式预测值还需滤波
      }
    }
  }

}

/** Check for identical motion in both motion vector direction of a bi-directional predicted CU
  * \returns true, if motion vectors and reference pictures match
 */
Bool TComPrediction::xCheckIdenticalMotion ( TComDataCU* pcCU, UInt PartAddr )　//检查是否存在一样的运动矢量
{
  if( pcCU->getSlice()->isInterB() && !pcCU->getSlice()->getPPS()->getWPBiPred() )　//该slice为Ｂslice 且不使用权重
  {
    if( pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr) >= 0 && pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr) >= 0)//list0和list1中均存在参考图像
    {
      Int RefPOCL0 = pcCU->getSlice()->getRefPic(REF_PIC_LIST_0, pcCU->getCUMvField(REF_PIC_LIST_0)->getRefIdx(PartAddr))->getPOC();
      Int RefPOCL1 = pcCU->getSlice()->getRefPic(REF_PIC_LIST_1, pcCU->getCUMvField(REF_PIC_LIST_1)->getRefIdx(PartAddr))->getPOC();
      if(RefPOCL0 == RefPOCL1 && pcCU->getCUMvField(REF_PIC_LIST_0)->getMv(PartAddr) == pcCU->getCUMvField(REF_PIC_LIST_1)->getMv(PartAddr))//若参考图像和运动矢量均相同
      {
        return true;
      }
    }
  }
  return false;
}
//! 运动补偿用到的是Cu中传递的CUMvField信息 m_acCUMvField[e] !!!这点非常重要 帧间预测时 经常用到这点特性
Void TComPrediction::motionCompensation ( TComDataCU* pcCU, TComYuv* pcYuvPred, RefPicList eRefPicList, Int iPartIdx )//PU块运动补偿　由参考图像和运动矢量计算得到预测值
{
  Int         iWidth;
  Int         iHeight;
  UInt        uiPartAddr;

  if ( iPartIdx >= 0 )//索引大于等于零表PU块有效
  {
    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iWidth, iHeight );//得到PU块的大小　和　地址
    if ( eRefPicList != REF_PIC_LIST_X )//指明（一个）具体参考列表
    {
      if( pcCU->getSlice()->getPPS()->getUseWP())//是否加权预测
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred, true );//中间结果　像素值不进行缩放还原　pcYuvPred为中间结果
      }
      else
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );//最终结果　像素值缩放还原
      }
      if ( pcCU->getSlice()->getPPS()->getUseWP() )//如果加权预测
      {
        xWeightedPredictionUni( pcCU, pcYuvPred, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );//预测值进行加权　输入为中间值pcYuvPred　输出为pcYuvPred
      }
    }
    else//未指明具体参考列表（两个列表都参考）
    {
      if ( xCheckIdenticalMotion( pcCU, uiPartAddr ) )　//判断PU两个方向上（list0 list1）上的参考帧和运动矢量是否相同
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred );//若相同　则取list0（或list1）预测
      }
      else
      {
        xPredInterBi  (pcCU, uiPartAddr, iWidth, iHeight, pcYuvPred );//若不同　Ｂslice预测（list0 list1 全需用到）
      }
    }
    return;　//方法结束　不在执行该方法后续程序
  }

  for ( iPartIdx = 0; iPartIdx < pcCU->getNumPartitions(); iPartIdx++ )//若索引无效　则处理所有当前CU中的所有PU 处理同上
  {
    pcCU->getPartIndexAndSize( iPartIdx, uiPartAddr, iWidth, iHeight );

    if ( eRefPicList != REF_PIC_LIST_X )
    {
      if( pcCU->getSlice()->getPPS()->getUseWP())
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred, true );
      }
      else
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
      if ( pcCU->getSlice()->getPPS()->getUseWP() )
      {
        xWeightedPredictionUni( pcCU, pcYuvPred, uiPartAddr, iWidth, iHeight, eRefPicList, pcYuvPred );
      }
    }
    else
    {
      if ( xCheckIdenticalMotion( pcCU, uiPartAddr ) )
      {
        xPredInterUni (pcCU, uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred );
      }
      else
      {
        xPredInterBi  (pcCU, uiPartAddr, iWidth, iHeight, pcYuvPred );
      }
    }
  }
  return;
}

Void TComPrediction::xPredInterUni ( TComDataCU* pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, RefPicList eRefPicList, TComYuv* pcYuvPred, Bool bi )//单向帧间预测得到预测像素值
{//由xPredInterBlk实现　分别计算（Y cr cb）各个组成的预测像素值
  Int         iRefIdx     = pcCU->getCUMvField( eRefPicList )->getRefIdx( uiPartAddr );           assert (iRefIdx >= 0);//参考图像索引一定不小于零
  TComMv      cMv         = pcCU->getCUMvField( eRefPicList )->getMv( uiPartAddr );　//得到运动矢量
  pcCU->clipMv(cMv);//限制运动矢量范围

  for (UInt comp=COMPONENT_Y; comp<pcYuvPred->getNumberValidComponents(); comp++)//为Ｙ cr cb　分别计算预测值
  {
    const ComponentID compID=ComponentID(comp);
    //亚像素补偿　提高预测精度
    xPredInterBlk  (compID,  pcCU, pcCU->getSlice()->getRefPic( eRefPicList, iRefIdx )->getPicYuvRec(), uiPartAddr, &cMv, iWidth, iHeight, pcYuvPred, bi, pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID)) );
  }
}

Void TComPrediction::xPredInterBi ( TComDataCU* pcCU, UInt uiPartAddr, Int iWidth, Int iHeight, TComYuv* pcYuvPred )//B slice 双向帧间预测计算预测值　pcYuvPred保存预测结果
{//由xPredInterUni实现　判断是否加权
  TComYuv* pcMbYuv;
  Int      iRefIdx[NUM_REF_PIC_LIST_01] = {-1, -1};

  for ( UInt refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
  {
    RefPicList eRefPicList = (refList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
    iRefIdx[refList] = pcCU->getCUMvField( eRefPicList )->getRefIdx( uiPartAddr );//得到list0（list1）中参考图像索引

    if ( iRefIdx[refList] < 0 )　//索引无效　则继续下个list
    {
      continue;
    }

    assert( iRefIdx[refList] < pcCU->getSlice()->getNumRefIdx(eRefPicList) );//参考图像索引一定小于总参考图像数

    pcMbYuv = &m_acYuvPred[refList];//对应reflist的预测值
    if( pcCU->getCUMvField( REF_PIC_LIST_0 )->getRefIdx( uiPartAddr ) >= 0 && pcCU->getCUMvField( REF_PIC_LIST_1 )->getRefIdx( uiPartAddr ) >= 0 )//list0 list1中均有有效参考图像
    {
      xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv, true );//list0 list1中均有有效参考图像 则一定会被加权（作为中间结果参与后续计算）
    }
    else
    {
      if ( ( pcCU->getSlice()->getPPS()->getUseWP()       && pcCU->getSlice()->getSliceType() == P_SLICE ) || //　p slice 或 b slice　加权预测
           ( pcCU->getSlice()->getPPS()->getWPBiPred()    && pcCU->getSlice()->getSliceType() == B_SLICE ) )
      {
        xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv, true );//中间结果
      }
      else//直接预测
      {
        xPredInterUni ( pcCU, uiPartAddr, iWidth, iHeight, eRefPicList, pcMbYuv );//最终结果
      }
    }
  }

  if ( pcCU->getSlice()->getPPS()->getWPBiPred()    && pcCU->getSlice()->getSliceType() == B_SLICE  )//B slice 加权双向预测
  {
    xWeightedPredictionBi( pcCU, &m_acYuvPred[REF_PIC_LIST_0], &m_acYuvPred[REF_PIC_LIST_1], iRefIdx[REF_PIC_LIST_0], iRefIdx[REF_PIC_LIST_1], uiPartAddr, iWidth, iHeight, pcYuvPred );
  }
  else if ( pcCU->getSlice()->getPPS()->getUseWP() && pcCU->getSlice()->getSliceType() == P_SLICE )　//P slice 加权预测
  {
    xWeightedPredictionUni( pcCU, &m_acYuvPred[REF_PIC_LIST_0], uiPartAddr, iWidth, iHeight, REF_PIC_LIST_0, pcYuvPred );
  }
  else　//未加权　若list0和list1中均存在有效索引则取二预测值平均　若只有一个list中存在有效索引则直接复制预测值
  {
    xWeightedAverage( &m_acYuvPred[REF_PIC_LIST_0], &m_acYuvPred[REF_PIC_LIST_1], iRefIdx[REF_PIC_LIST_0], iRefIdx[REF_PIC_LIST_1], uiPartAddr, iWidth, iHeight, pcYuvPred, pcCU->getSlice()->getSPS()->getBitDepths() );
  }
}

/**
 * \brief Generate motion-compensated block
 *
 * \param compID     Colour component ID
 * \param cu         Pointer to current CU
 * \param refPic     Pointer to reference picture
 * \param partAddr   Address of block within CU
 * \param mv         Motion vector
 * \param width      Width of block
 * \param height     Height of block
 * \param dstPic     Pointer to destination picture
 * \param bi         Flag indicating whether bipred is used
 * \param  bitDepth  Bit depth
 */


Void TComPrediction::xPredInterBlk(const ComponentID compID, TComDataCU *cu, TComPicYuv *refPic, UInt partAddr, TComMv *mv, Int width, Int height, TComYuv *dstPic, Bool bi, const Int bitDepth )
{//根据运动矢量计算预测像素值
  Int     refStride  = refPic->getStride(compID);//参考图像步长
  Int     dstStride  = dstPic->getStride(compID);//目标图像步长
  Int shiftHor=(2+refPic->getComponentScaleX(compID));//水平方向运动矢量像素精度(luma为1/4像素精度)
  Int shiftVer=(2+refPic->getComponentScaleY(compID));//垂直方向同上

  Int     refOffset  = (mv->getHor() >> shiftHor) + (mv->getVer() >> shiftVer) * refStride;//由运动矢量确定的参考图像位置与目标图像的偏移量

  Pel*    ref     = refPic->getAddr(compID, cu->getCtuRsAddr(), cu->getZorderIdxInCtu() + partAddr ) + refOffset;//参考像素起始位置

  Pel*    dst = dstPic->getAddr( compID, partAddr );　　//目标图像起始位置

  Int     xFrac  = mv->getHor() & ((1<<shiftHor)-1);//运动向量　小数部分（1/4精度）
  Int     yFrac  = mv->getVer() & ((1<<shiftVer)-1);
  UInt    cxWidth  = width  >> refPic->getComponentScaleX(compID);　//参考图像大小
  UInt    cxHeight = height >> refPic->getComponentScaleY(compID);

  const ChromaFormat chFmt = cu->getPic()->getChromaFormat();//色度格式

  if ( yFrac == 0 ) //垂直方向运动矢量正好为整数，则进行水平1/4像素精度插值
  {
    m_if.filterHor(compID, ref, refStride, dst,  dstStride, cxWidth, cxHeight, xFrac, !bi, chFmt, bitDepth);
  }
  else if ( xFrac == 0 )//水平方向运动矢量正好为整数，则进行垂直1/4像素精度插值
  {
    m_if.filterVer(compID, ref, refStride, dst, dstStride, cxWidth, cxHeight, yFrac, true, !bi, chFmt, bitDepth);
  }
  else//否则水平垂直都进行插值
  {
    Int   tmpStride = m_filteredBlockTmp[0].getStride(compID);
    Pel*  tmp       = m_filteredBlockTmp[0].getAddr(compID);

    const Int vFilterSize = isLuma(compID) ? NTAPS_LUMA : NTAPS_CHROMA;//抽头系数个数
　　//运动矢量垂直水平分量均位于亚像素位置时　求最终结果需缓存水平滤波值用于最终滤波值的计算
    m_if.filterHor(compID, ref - ((vFilterSize>>1) -1)*refStride, refStride, tmp, tmpStride, cxWidth, cxHeight+vFilterSize-1, xFrac, false,      chFmt, bitDepth);
    m_if.filterVer(compID, tmp + ((vFilterSize>>1) -1)*tmpStride, tmpStride, dst, dstStride, cxWidth, cxHeight,               yFrac, false, !bi, chFmt, bitDepth);
  }
}

Void TComPrediction::xWeightedAverage( TComYuv* pcYuvSrc0, TComYuv* pcYuvSrc1, Int iRefIdx0, Int iRefIdx1, UInt uiPartIdx, Int iWidth, Int iHeight, TComYuv* pcYuvDst, const BitDepths &clipBitDepths )
{
  if( iRefIdx0 >= 0 && iRefIdx1 >= 0 )//list0 list1中均存在有效参考图像
  {
    pcYuvDst->addAvg( pcYuvSrc0, pcYuvSrc1, uiPartIdx, iWidth, iHeight, clipBitDepths );// pcYuvSrc0, pcYuvSrc1二者取平均
  }
  else if ( iRefIdx0 >= 0 && iRefIdx1 <  0 )　//复制pcYuvSrc0至pcYuvDst
  {
    pcYuvSrc0->copyPartToPartYuv( pcYuvDst, uiPartIdx, iWidth, iHeight );
  }
  else if ( iRefIdx0 <  0 && iRefIdx1 >= 0 )　//复制pcYuvSrc１至pcYuvDst
  {
    pcYuvSrc1->copyPartToPartYuv( pcYuvDst, uiPartIdx, iWidth, iHeight );
  }
}

// AMVP
Void TComPrediction::getMvPredAMVP( TComDataCU* pcCU, UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, TComMv& rcMvPred )//由AMVP MV候选列表得到预测运动矢量
{
  AMVPInfo* pcAMVPInfo = pcCU->getCUMvField(eRefPicList)->getAMVPInfo();

  if( pcAMVPInfo->iN <= 1 )//候选运动矢量数小于或等于１
  {
    rcMvPred = pcAMVPInfo->m_acMvCand[0];//则直接先第一个矢量作为预测矢量

    pcCU->setMVPIdxSubParts( 0, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
    pcCU->setMVPNumSubParts( pcAMVPInfo->iN, eRefPicList, uiPartAddr, uiPartIdx, pcCU->getDepth(uiPartAddr));
    return;
  }

  assert(pcCU->getMVPIdx(eRefPicList,uiPartAddr) >= 0);
  rcMvPred = pcAMVPInfo->m_acMvCand[pcCU->getMVPIdx(eRefPicList,uiPartAddr)];//由参考图像列表和块地址得到ＭＶ在候选列表中的索引
  return;
}

/** Function for deriving planar intra prediction.
 * \param pSrc        pointer to reconstructed sample array
 * \param srcStride   the stride of the reconstructed sample array
 * \param rpDst       reference to pointer for the prediction sample array
 * \param dstStride   the stride of the prediction sample array
 * \param width       the width of the block
 * \param height      the height of the block
 * \param channelType type of pel array (luma, chroma)
 * \param format      chroma format
 *
 * This function derives the prediction samples for planar mode (intra coding).
 */
//NOTE: Bit-Limit - 24-bit source
//planar模式计算预测像素值
Void TComPrediction::xPredIntraPlanar( const Pel* pSrc, Int srcStride, Pel* rpDst, Int dstStride, UInt width, UInt height )
{
  assert(width <= height);

  Int leftColumn[MAX_CU_SIZE+1], topRow[MAX_CU_SIZE+1], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
  UInt shift1Dhor = g_aucConvertToBit[ width ] + 2;
  UInt shift1Dver = g_aucConvertToBit[ height ] + 2;

  // Get left and above reference column and row
  for(Int k=0;k<width+1;k++)//得到上方参考像素值
  {
    topRow[k] = pSrc[k-srcStride];
  }

  for (Int k=0; k < height+1; k++)//得到左侧参考像素值
  {
    leftColumn[k] = pSrc[k*srcStride-1];
  }

  // Prepare intermediate variables used in interpolation
  Int bottomLeft = leftColumn[height];//p[-1][N]
  Int topRight   = topRow[width];//p[N][-1]

  for(Int k=0;k<width;k++)//计算p[-1][N]-p[x][-1] 和　N*p[x][-1]
  {
    bottomRow[k]  = bottomLeft - topRow[k];
    topRow[k]     <<= shift1Dver;
  }

  for(Int k=0;k<height;k++)//计算p[N][-1]-p[-1][y] 和 N*p[-1][y]
  {
    rightColumn[k]  = topRight - leftColumn[k];
    leftColumn[k]   <<= shift1Dhor;
  }

  const UInt topRowShift = 0;

  // Generate prediction signal
  for (Int y=0;y<height;y++)//遍历计算Ph[x][y]=N*p[-1][y]+(x+1)(p[N][-1]-p[-1][y]) x为遍历次数
  {
    Int horPred = leftColumn[y] + width;
    for (Int x=0;x<width;x++)
    {
      horPred += rightColumn[y];//循环加法计算乘法　减少计算量
      topRow[x] += bottomRow[x];//循环加法计算乘法　减少计算量

      Int vertPred = ((topRow[x] + topRowShift)>>topRowShift);//遍历计算Pv[x][y]=N*p[N][-1]+(y+1)(p[-1][N]-p[x][-1]) y为遍历次数
      rpDst[y*dstStride+x] = ( horPred + vertPred ) >> (shift1Dhor+1);//二者平均得到最终预测像素值
    }
  }
}

/** Function for filtering intra DC predictor.
 * \param pSrc pointer to reconstructed sample array
 * \param iSrcStride the stride of the reconstructed sample array
 * \param pDst reference to pointer for the prediction sample array
 * \param iDstStride the stride of the prediction sample array
 * \param iWidth the width of the block
 * \param iHeight the height of the block
 * \param channelType type of pel array (luma, chroma)
 *
 * This function performs filtering left and top edges of the prediction samples for DC mode (intra coding).
 */
//DC模式边界像素滤波
Void TComPrediction::xDCPredFiltering( const Pel* pSrc, Int iSrcStride, Pel* pDst, Int iDstStride, Int iWidth, Int iHeight, ChannelType channelType )
{
  Int x, y, iDstStride2, iSrcStride2;

  if (isLuma(channelType) && (iWidth <= MAXIMUM_INTRA_FILTERED_WIDTH) && (iHeight <= MAXIMUM_INTRA_FILTERED_HEIGHT))//luma 块大小小于３２＊３２
  {
    //top-left
    pDst[0] = (Pel)((pSrc[-iSrcStride] + pSrc[-1] + 2 * pDst[0] + 2) >> 2);　//左上侧一点滤波

    //top row (vertical filter)
    for ( x = 1; x < iWidth; x++ )　//最上方一行预测像素滤波
    {
      pDst[x] = (Pel)((pSrc[x - iSrcStride] +  3 * pDst[x] + 2) >> 2);
    }

    //left column (horizontal filter)
    for ( y = 1, iDstStride2 = iDstStride, iSrcStride2 = iSrcStride-1; y < iHeight; y++, iDstStride2+=iDstStride, iSrcStride2+=iSrcStride )//最左侧一列像素滤波
    {
      pDst[iDstStride2] = (Pel)((pSrc[iSrcStride2] + 3 * pDst[iDstStride2] + 2) >> 2);
    }
  }

  return;
}

/* Static member function */
Bool TComPrediction::UseDPCMForFirstPassIntraEstimation(TComTU &rTu, const UInt uiDirMode)
{//为标准垂直模式和标准水平时可用DPCM
  return (rTu.getCU()->isRDPCMEnabled(rTu.GetAbsPartIdxTU()) ) &&
          rTu.getCU()->getCUTransquantBypass(rTu.GetAbsPartIdxTU()) &&
          (uiDirMode==HOR_IDX || uiDirMode==VER_IDX);
}

//! \}
