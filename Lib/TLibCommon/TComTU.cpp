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

/*
*重载多种不同的TcomTU构造方法
*TComTURecurse继承自TcomTU
一个父TU块根据不同的分割方法其子TU块信息不同
mRect表示同一父Tu块中子Tu块的位置信息
*/

#include "TComTU.h"
#include "TComRom.h"
#include "TComDataCU.h"
#include "TComPic.h"

//----------------------------------------------------------------------------------------------------------------------

/*static*/ const UInt TComTU::NUMBER_OF_SECTIONS[TComTU::NUMBER_OF_SPLIT_MODES] = { 1, 2, 4 };//三种分割方法对应子块个数

static     const UInt         partIdxStepShift  [TComTU::NUMBER_OF_SPLIT_MODES] = { 0, 1, 2 };//log2(子块数)

//----------------------------------------------------------------------------------------------------------------------

TComTU::TComTU(TComDataCU *pcCU, const UInt absPartIdxCU, const UInt cuDepth, const UInt initTrDepthRelCU)//当前CU作为TU树的根节点 initTrDepthRelCU TU相对CU深度
  : mChromaFormat(pcCU->getSlice()->getSPS()->getChromaFormatIdc()),
    mbProcessLastOfLevel(true), // does not matter. the top level is not 4 quadrants.
    mCuDepth(cuDepth),
    mSection(0),
    mSplitMode(DONT_SPLIT),
    mAbsPartIdxCU(absPartIdxCU),
    mAbsPartIdxTURelCU(0),//TU块相对其所在CU块的位置索引
    mAbsPartIdxStep(pcCU->getPic()->getNumPartitionsInCtu() >> (pcCU->getDepth(absPartIdxCU)<<1)),//地址为absPartIdxCU的CU块含有最小块个数
    mpcCU(pcCU),
    mLog2TrLumaSize(0),
    mpParent(NULL)
{//该构造函数声明相对CU深度为initTrDepthRelCU的TU块
  const TComSPS *pSPS=pcCU->getSlice()->getSPS();
  mLog2TrLumaSize = g_aucConvertToBit[pSPS->getMaxCUWidth() >> (mCuDepth+initTrDepthRelCU)]+2;//g_aucConvertToBit[ x ]: log2(x/4), if x=4 -> 0, x=8 -> 1, x=16 -> 2, ...

  const UInt baseOffset444=pcCU->getPic()->getMinCUWidth()*pcCU->getPic()->getMinCUHeight()*absPartIdxCU;//该CU总像素偏移量（CU的位置）

  for(UInt i=0; i<MAX_NUM_COMPONENT; i++)//为Y Cr Cb赋值
  {
    mTrDepthRelCU[i] = initTrDepthRelCU;
    const UInt csx=getComponentScaleX(ComponentID(i), mChromaFormat);//return (isLuma(id) || (fmt==CHROMA_444)) ? 0 : 1;
    const UInt csy=getComponentScaleY(ComponentID(i), mChromaFormat);//return (isLuma(id) || (fmt!=CHROMA_420)) ? 0 : 1; 
    mOrigWidth[i]=mRect[i].width = (i < getNumberValidComponents(mChromaFormat)) ? (pcCU->getWidth( absPartIdxCU) >> csx) : 0;//该TU块的宽
    mRect[i].height              = (i < getNumberValidComponents(mChromaFormat)) ? (pcCU->getHeight(absPartIdxCU) >> csy) : 0;//该TU块的高
    mRect[i].x0=0;//定义TU块左上像素坐标为（0，0）
    mRect[i].y0=0;
    mCodeAll[i]=true;
    mOffsets[i]=baseOffset444>>(csx+csy);
  }
}



TComTURecurse::TComTURecurse(      TComDataCU *pcCU,
                             const UInt        absPartIdxCU)
  : TComTU(pcCU, absPartIdxCU, pcCU->getDepth(absPartIdxCU), 0)////当前CU最上层TU块（即与CU等大小TU块）
{ }



TComTU::TComTU(TComTU &parent, const Bool bProcessLastOfLevel, const TU_SPLIT_MODE splitMode, const Bool splitAtCurrentDepth, const ComponentID absPartIdxSourceComponent)//以父TU声明子TU信息
  : mChromaFormat(parent.mChromaFormat),//初始化赋值
    mbProcessLastOfLevel(bProcessLastOfLevel),
    mCuDepth(parent.mCuDepth),
    mSection(0),
    mSplitMode(splitMode),
    mAbsPartIdxCU(parent.mAbsPartIdxCU),
    mAbsPartIdxTURelCU(parent.GetRelPartIdxTU(absPartIdxSourceComponent)),
    mAbsPartIdxStep(std::max<UInt>(1, (parent.GetAbsPartIdxNumParts(absPartIdxSourceComponent) >> partIdxStepShift[splitMode]))),
    mpcCU(parent.mpcCU),
    mLog2TrLumaSize(parent.mLog2TrLumaSize - ((splitMode != QUAD_SPLIT) ? 0 : 1)), //no change in width for vertical split
    mpParent(&parent)
{
  for(UInt i=0; i<MAX_NUM_COMPONENT; i++)//遍历Y Cr Cb 所有组成成分
  {
    mTrDepthRelCU[i] = parent.mTrDepthRelCU[i] + ((splitAtCurrentDepth || (splitMode == DONT_SPLIT)) ? 0 : 1);//若(父)Tu分割则(子)Tu相对深度加一
  }

  if (mSplitMode==DONT_SPLIT)
  {//不分割 子Tu同父Tu
    for(UInt i=0; i<MAX_NUM_COMPONENT; i++)
    {
      mRect[i] = (parent.mRect[i]);
      mOffsets[i]=parent.mOffsets[i];
      mCodeAll[i]=true; // The 1 TU at this level is coded.
      mOrigWidth[i]=mRect[i].width;
    }
    return;//方法结束
  }
  else if (mSplitMode==VERTICAL_SPLIT)//分割成上下半部分
  {
    for(UInt i=0; i<MAX_NUM_COMPONENT; i++)
    {
      mRect[i].x0 = (parent.mRect[i].x0);//第一块子TU起始位置同父Tu起始位置
      mRect[i].y0 = (parent.mRect[i].y0);
      mRect[i].width  = (parent.mRect[i].width);//宽度不变
      mRect[i].height = (parent.mRect[i].height)>>1;//高度减半
      mOffsets[i]=parent.mOffsets[i];
      mCodeAll[i]=true; // The 2 TUs at this level is coded.
      mOrigWidth[i]=mRect[i].width;
    }
    return;
  }

  for(UInt i=0; i<MAX_NUM_COMPONENT; i++)
  {//分割成等大四块
    mRect[i].width = (parent.mRect[i].width >> 1);//子Tu宽减半
    mRect[i].height= (parent.mRect[i].height>> 1);////子Tu高减半
    mRect[i].x0=parent.mRect[i].x0;//第一块子TU起始位置同父Tu起始位置
    mRect[i].y0=parent.mRect[i].y0;
    mOffsets[i]=parent.mOffsets[i];

    if ((mRect[i].width < MIN_TU_SIZE || mRect[i].height < MIN_TU_SIZE) && mRect[i].width!=0)//子TU小于最小TU块
    {
      const UInt numPels=mRect[i].width * mRect[i].height;//子TU块像素数
      if (numPels < (MIN_TU_SIZE*MIN_TU_SIZE))
      {//宽和高无法同时不小于4 则无法往下分割 返回上一层即父Tu
        // this level doesn't have enough pixels to have 4 blocks of any relative dimension
        mRect[i].width = parent.mRect[i].width;
        mRect[i].height= parent.mRect[i].height;
        mCodeAll[i]=false; // go up a level, so only process one entry of a quadrant
        mTrDepthRelCU[i]--;
      }
      else if (mRect[i].width < mRect[i].height)
      {//转化成所能处理的块大小
        mRect[i].width=MIN_TU_SIZE;
        mRect[i].height=numPels/MIN_TU_SIZE;
        mCodeAll[i]=true;
      }
      else
      {//转化成所能处理的块大小
        mRect[i].height=MIN_TU_SIZE;
        mRect[i].width=numPels/MIN_TU_SIZE;
        mCodeAll[i]=true;
      }
    }
    else
    {
      mCodeAll[i]=true;
    }

    mOrigWidth[i]=mRect[i].width;
    if (!mCodeAll[i] && mbProcessLastOfLevel)//不需要往下分割
    {
      mRect[i].width=0;
    }
  }
}

Bool TComTURecurse::nextSection(const TComTU &parent)//以当前TU块得到子TU块信息 若存在下一子块则返回真 
{
  if (mSplitMode==DONT_SPLIT)
  {//不存在子TU块 返回false
    mSection++;
    return false;
  }
  else//存在子块
  {
    for(UInt i=0; i<MAX_NUM_COMPONENT; i++)//遍历Y Cr Cb分量                    
    {																		
      mOffsets[i]+=mRect[i].width*mRect[i].height;
      if (mbProcessLastOfLevel)//当前层作为最后操作层
      {
        mRect[i].width=mOrigWidth[i];//子块宽度等于父块
      }
      mRect[i].x0+=mRect[i].width;//下一子TU块 起始位置
      const TComRectangle &parentRect=parent.getRect(ComponentID(i));
      if (mRect[i].x0 >= parentRect.x0+parentRect.width)//若下一子TU位置超出父TU所在第一行子TU块位置 则转到下一行 （例如section=2时）
      {
        mRect[i].x0=parentRect.x0;
        mRect[i].y0+=mRect[i].height;//转到下一行
      }
      if (!mCodeAll[i])//
      {
        if (!mbProcessLastOfLevel || mSection!=2)
        {
          mRect[i].width=0;
        }
      }
    }
    assert(mRect[COMPONENT_Cb].x0==mRect[COMPONENT_Cr].x0);//断言Cr Cb 位置大小相同
    assert(mRect[COMPONENT_Cb].y0==mRect[COMPONENT_Cr].y0);
    assert(mRect[COMPONENT_Cb].width==mRect[COMPONENT_Cr].width);
    assert(mRect[COMPONENT_Cb].height==mRect[COMPONENT_Cr].height);

    mAbsPartIdxTURelCU+=mAbsPartIdxStep;//该子TU块相对CU块位置索引
    mSection++;
    return mSection< (1<<mSplitMode);//是否小于总section数 即section是否遍历完
  }
}


UInt TComTU::GetEquivalentLog2TrSize(const ComponentID compID)     const
{//log2(TUsize)
  return g_aucConvertToBit[ getRect(compID).height ] + 2;
}


Bool TComTU::useDST(const ComponentID compID)
{//TU是否使用DST变换
        TComDataCU *const pcCU       = getCU();
  const UInt              absPartIdx = GetAbsPartIdxTU(compID);

  return isLuma(compID) && pcCU->isIntra(absPartIdx);
}


Bool TComTU::isNonTransformedResidualRotated(const ComponentID compID)
{
  // rotation only for 4x4 intra, and is only used for non-transformed blocks (the latter is not checked here)
  return    getCU()->getSlice()->getSPS()->getSpsRangeExtension().getTransformSkipRotationEnabledFlag()
         && mRect[compID].width == 4
         && getCU()->isIntra(GetAbsPartIdxTU());
}


UInt TComTU::getGolombRiceStatisticsIndex(const ComponentID compID)
{
        TComDataCU *const pcCU             = getCU();
  const UInt              absPartIdx       = GetAbsPartIdxTU(compID);
  const Bool              transformSkip    = pcCU->getTransformSkip(absPartIdx, compID);
  const Bool              transquantBypass = pcCU->getCUTransquantBypass(absPartIdx);

  //--------

  const UInt channelTypeOffset    =  isChroma(compID)                   ? 2 : 0;
  const UInt nonTransformedOffset = (transformSkip || transquantBypass) ? 1 : 0;

  //--------

  const UInt selectedIndex = channelTypeOffset + nonTransformedOffset;
  assert(selectedIndex < RExt__GOLOMB_RICE_ADAPTATION_STATISTICS_SETS);

  return selectedIndex;
}
