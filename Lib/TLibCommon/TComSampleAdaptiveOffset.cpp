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

/** \file     TComSampleAdaptiveOffset.cpp
    \brief    sample adaptive offset class
*/

#include "TComSampleAdaptiveOffset.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

//! \ingroup TLibCommon
//! \{

SAOOffset::SAOOffset()//构造函数
{
  reset();
}

SAOOffset::~SAOOffset()//析构函数
{

}

Void SAOOffset::reset()
{//SAOOffset复位 赋予初始化无用信息
  modeIdc = SAO_MODE_OFF;
  typeIdc = -1;
  typeAuxInfo = -1;
  ::memset(offset, 0, sizeof(Int)* MAX_NUM_SAO_CLASSES);
}

const SAOOffset& SAOOffset::operator= (const SAOOffset& src)//赋值 运算符重载
{
  modeIdc = src.modeIdc;
  typeIdc = src.typeIdc;
  typeAuxInfo = src.typeAuxInfo;
  ::memcpy(offset, src.offset, sizeof(Int)* MAX_NUM_SAO_CLASSES);

  return *this;
}
/*
*SAOOffset结构如下：
struct SAOOffset
{
  SAOMode modeIdc; // NEW, MERGE, OFF
  Int typeIdc;     // union of SAOModeMergeTypes and SAOModeNewTypes, depending on modeIdc.
  Int typeAuxInfo; // BO: starting band index
  Int offset[MAX_NUM_SAO_CLASSES];

  SAOOffset();
  ~SAOOffset();
  Void reset();

  const SAOOffset& operator= (const SAOOffset& src);
};
*/

SAOBlkParam::SAOBlkParam()//构造函数
{
  reset();
}

SAOBlkParam::~SAOBlkParam()//析构函数
{

}

Void SAOBlkParam::reset()
{//SAOBlkParam初始化 包含3个分量的SAOOffset 为各个分量的SAOOffset赋值
  for(Int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    offsetParam[compIdx].reset();
  }
}

const SAOBlkParam& SAOBlkParam::operator= (const SAOBlkParam& src)//运算符重载
{
  for(Int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    offsetParam[compIdx] = src.offsetParam[compIdx];
  }
  return *this;

}

TComSampleAdaptiveOffset::TComSampleAdaptiveOffset()
{
  m_tempPicYuv = NULL;
  m_lineBufWidth = 0;
  m_signLineBuf1 = NULL;
  m_signLineBuf2 = NULL;
}


TComSampleAdaptiveOffset::~TComSampleAdaptiveOffset()//析构函数  释放资源
{
  destroy();

  if (m_signLineBuf1)
  {
    delete[] m_signLineBuf1;
    m_signLineBuf1 = NULL;
  }
  if (m_signLineBuf2)
  {
    delete[] m_signLineBuf2;
    m_signLineBuf2 = NULL;
  }
}

Void TComSampleAdaptiveOffset::create( Int picWidth, Int picHeight, ChromaFormat format, UInt maxCUWidth, UInt maxCUHeight, UInt maxCUDepth, UInt lumaBitShift, UInt chromaBitShift )
{//创建对象 分配内存
  destroy();

  m_picWidth        = picWidth;
  m_picHeight       = picHeight;
  m_chromaFormatIDC = format;
  m_maxCUWidth      = maxCUWidth;
  m_maxCUHeight     = maxCUHeight;

  m_numCTUInWidth   = (m_picWidth/m_maxCUWidth) + ((m_picWidth % m_maxCUWidth)?1:0);//图像宽CTU个数
  m_numCTUInHeight  = (m_picHeight/m_maxCUHeight) + ((m_picHeight % m_maxCUHeight)?1:0);//图像列CTU个数
  m_numCTUsPic      = m_numCTUInHeight*m_numCTUInWidth;//图像CTU'总个数

  //temporary picture buffer
  if ( !m_tempPicYuv )//暂存图像
  {
    m_tempPicYuv = new TComPicYuv;
    m_tempPicYuv->create( m_picWidth, m_picHeight, m_chromaFormatIDC, m_maxCUWidth, m_maxCUHeight, maxCUDepth, true );
  }

  //bit-depth related
  for(Int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)//分别为亮度 色度分量的offset移位量赋值
  {
    m_offsetStepLog2  [compIdx] = isLuma(ComponentID(compIdx))? lumaBitShift : chromaBitShift;
  }
}

Void TComSampleAdaptiveOffset::destroy()
{//销毁对象
  if ( m_tempPicYuv )
  {
    m_tempPicYuv->destroy();
    delete m_tempPicYuv;
    m_tempPicYuv = NULL;
  }
}

Void TComSampleAdaptiveOffset::invertQuantOffsets(ComponentID compIdx, Int typeIdc, Int typeAuxInfo, Int* dstOffsets, Int* srcOffsets)
{
  Int codedOffset[MAX_NUM_SAO_CLASSES];

  ::memcpy(codedOffset, srcOffsets, sizeof(Int)*MAX_NUM_SAO_CLASSES);
  ::memset(dstOffsets, 0, sizeof(Int)*MAX_NUM_SAO_CLASSES);

  if(typeIdc == SAO_TYPE_START_BO)//边带模式
  {
    for(Int i=0; i< 4; i++)//CTU只能选择四条连续的边带
    {
      dstOffsets[(typeAuxInfo+ i)%NUM_SAO_BO_CLASSES] = codedOffset[(typeAuxInfo+ i)%NUM_SAO_BO_CLASSES]*(1<<m_offsetStepLog2[compIdx]);
    }
  }
  else //EO 边界模式
  {
    for(Int i=0; i< NUM_SAO_EO_CLASSES; i++)//依次为5种边界模式dstOffsets赋值
    {
      dstOffsets[i] = codedOffset[i] *(1<<m_offsetStepLog2[compIdx]);
    }
    assert(dstOffsets[SAO_CLASS_EO_PLAIN] == 0); //keep EO plain offset as zero
  }

}

Int TComSampleAdaptiveOffset::getMergeList(TComPic* pic, Int ctuRsAddr, SAOBlkParam* blkParams, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES])
{//SAO参数融合 由MergeList得到SAO参数
  Int ctuX = ctuRsAddr % m_numCTUInWidth;//当前CTU在图像中的X坐标（以CTU为单位）
  Int ctuY = ctuRsAddr / m_numCTUInWidth;//当前CTU在图像中的Y坐标（以CTU为单位）
  Int mergedCTUPos;
  Int numValidMergeCandidates = 0;

  for(Int mergeType=0; mergeType< NUM_SAO_MERGE_TYPES; mergeType++)//遍历两种融合模式
  {
    SAOBlkParam* mergeCandidate = NULL;

    switch(mergeType)
    {
    case SAO_MERGE_ABOVE: //融合上方块参数
      {
        if(ctuY > 0)//不为第一行 即存在上方块
        {
          mergedCTUPos = ctuRsAddr- m_numCTUInWidth;//当前CTU上方CTU的位置
          if( pic->getSAOMergeAvailability(ctuRsAddr, mergedCTUPos) )//若上方CTU SAO参数可获得
          {
            mergeCandidate = &(blkParams[mergedCTUPos]);//得到候选参数
          }
        }
      }
      break;
    case SAO_MERGE_LEFT://融合左侧块参数
      {
        if(ctuX > 0)//不为第一列 即存在左侧块
        {
          mergedCTUPos = ctuRsAddr- 1;//当前CTU左侧CTU的位置
          if( pic->getSAOMergeAvailability(ctuRsAddr, mergedCTUPos) )//若左侧CTU SAO参数可获得
          {
            mergeCandidate = &(blkParams[mergedCTUPos]);//得到候选参数
          }
        }
      }
      break;
    default:
      {
        printf("not a supported merge type");
        assert(0);
        exit(-1);
      }
    }

    mergeList[mergeType]=mergeCandidate;//当前CTU块融合参数列表（左侧和上方）
    if (mergeCandidate != NULL)
    {
      numValidMergeCandidates++;
    }
  }

  return numValidMergeCandidates;//返回有效后效融合候选参数的个数
}


Void TComSampleAdaptiveOffset::reconstructBlkSAOParam(SAOBlkParam& recParam, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES])
{//重建该CTU块所有分量的SAO参数信息 SAOBlkParam& recParam保存输出信息
  const Int numberOfComponents = getNumberValidComponents(m_chromaFormatIDC);
  for(Int compIdx = 0; compIdx < numberOfComponents; compIdx++)//遍历各个分量
  {
    const ComponentID component = ComponentID(compIdx);
    SAOOffset& offsetParam = recParam[component];//各个分量对应的SAOOffset参数

    if(offsetParam.modeIdc == SAO_MODE_OFF)//若该分量SAO模式未开  则继续执行下一分量
    {
      continue;
    }

    switch(offsetParam.modeIdc)
    {
    case SAO_MODE_NEW://边带或边界模式
      {
        invertQuantOffsets(component, offsetParam.typeIdc, offsetParam.typeAuxInfo, offsetParam.offset, offsetParam.offset);
      }
      break;
    case SAO_MODE_MERGE://参数融合模式 由左侧或上方块参数直接得到该块的参数
      {
        SAOBlkParam* mergeTarget = mergeList[offsetParam.typeIdc];
        assert(mergeTarget != NULL);

        offsetParam = (*mergeTarget)[component];
      }
      break;
    default:
      {
        printf("Not a supported mode");
        assert(0);
        exit(-1);
      }
    }
  }
}

Void TComSampleAdaptiveOffset::reconstructBlkSAOParams(TComPic* pic, SAOBlkParam* saoBlkParams)
{//得到该图像所有CTU块的SAO参数信息
  for(Int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    m_picSAOEnabled[compIdx] = false;//各个分量SAO使能初始化赋false
  }

  const Int numberOfComponents = getNumberValidComponents(m_chromaFormatIDC);

  for(Int ctuRsAddr=0; ctuRsAddr< m_numCTUsPic; ctuRsAddr++)//遍历图像所有CTU块
  {
    SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES] = { NULL };
    getMergeList(pic, ctuRsAddr, saoBlkParams, mergeList);

    reconstructBlkSAOParam(saoBlkParams[ctuRsAddr], mergeList);//重建该CTU块SAO参数

    for(Int compIdx = 0; compIdx < numberOfComponents; compIdx++)//若该分量中存在开启SAO模式的CTU块 则该分量SAO使能
    {
      if(saoBlkParams[ctuRsAddr][compIdx].modeIdc != SAO_MODE_OFF)
      {
        m_picSAOEnabled[compIdx] = true;
      }
    }
  }
}


Void TComSampleAdaptiveOffset::offsetBlock(const Int channelBitDepth, Int typeIdx, Int* offset
                                          , Pel* srcBlk, Pel* resBlk, Int srcStride, Int resStride,  Int width, Int height
                                          , Bool isLeftAvail,  Bool isRightAvail, Bool isAboveAvail, Bool isBelowAvail, Bool isAboveLeftAvail, Bool isAboveRightAvail, Bool isBelowLeftAvail, Bool isBelowRightAvail)
{
  if(m_lineBufWidth != m_maxCUWidth)
  {
    m_lineBufWidth = m_maxCUWidth;

    if (m_signLineBuf1)
    {
      delete[] m_signLineBuf1;
      m_signLineBuf1 = NULL;
    }
    m_signLineBuf1 = new Char[m_lineBufWidth+1];

    if (m_signLineBuf2)
    {
      delete[] m_signLineBuf2;
      m_signLineBuf2 = NULL;
    }
    m_signLineBuf2 = new Char[m_lineBufWidth+1];
  }

  const Int maxSampleValueIncl = (1<< channelBitDepth )-1;//最大像素值

  Int x,y, startX, startY, endX, endY, edgeType;
  Int firstLineStartX, firstLineEndX, lastLineStartX, lastLineEndX;
  Char signLeft, signRight, signDown;

  Pel* srcLine = srcBlk;//SAO补偿前的像素值
  Pel* resLine = resBlk;//SAO补偿后的像素值

  switch(typeIdx)//SAO补偿类型
  {
  case SAO_TYPE_EO_0://水平方向边界补偿
    {
      offset += 2;
      startX = isLeftAvail ? 0 : 1;//当前像素左侧必须有像素
      endX   = isRightAvail ? width : (width -1);////当前像素右侧必须有像素
      for (y=0; y< height; y++)//遍历列
      {
        signLeft = (Char)sgn(srcLine[startX] - srcLine[startX-1]);//当前像素于左侧像素大小
        for (x=startX; x< endX; x++)//遍历行
        {
          signRight = (Char)sgn(srcLine[x] - srcLine[x+1]); //当前像素与右侧像素大小
          edgeType =  signRight + signLeft;//边界补偿分类(共5类 -2 -1 0 1 2)
          signLeft  = -signRight;//像素向右移动一个 当前像素的signRight是下一像素signLeft的相反值

          resLine[x] = Clip3<Int>(0, maxSampleValueIncl, srcLine[x] + offset[edgeType]);//不同分类加上对应补偿值
        }
        srcLine  += srcStride;//下一行
        resLine += resStride;
      }

    }
    break;
  case SAO_TYPE_EO_90://垂直方向边界补偿 与水平方向基本相同 不再叙述
    {
      offset += 2;
      Char *signUpLine = m_signLineBuf1;

      startY = isAboveAvail ? 0 : 1;
      endY   = isBelowAvail ? height : height-1;
      if (!isAboveAvail)
      {
        srcLine += srcStride;
        resLine += resStride;
      }

      Pel* srcLineAbove= srcLine- srcStride;
      for (x=0; x< width; x++)
      {
        signUpLine[x] = (Char)sgn(srcLine[x] - srcLineAbove[x]);
      }

      Pel* srcLineBelow;
      for (y=startY; y<endY; y++)
      {
        srcLineBelow= srcLine+ srcStride;

        for (x=0; x< width; x++)
        {
          signDown  = (Char)sgn(srcLine[x] - srcLineBelow[x]);
          edgeType = signDown + signUpLine[x];
          signUpLine[x]= -signDown;

          resLine[x] = Clip3<Int>(0, maxSampleValueIncl, srcLine[x] + offset[edgeType]);
        }
        srcLine += srcStride;
        resLine += resStride;
      }

    }
    break;
  case SAO_TYPE_EO_135://135度方向 边界补偿
    {
      offset += 2;
      Char *signUpLine, *signDownLine, *signTmpLine;

      signUpLine  = m_signLineBuf1;
      signDownLine= m_signLineBuf2;

      startX = isLeftAvail ? 0 : 1 ;//保证当前像素左侧有像素
      endX   = isRightAvail ? width : (width-1);//保证当前像素右侧有像素

      //prepare 2nd line's upper sign
      Pel* srcLineBelow= srcLine+ srcStride;//第二行像素
      for (x=startX; x< endX+1; x++)//比价第二行像素与其左上角像素大小
      {
        signUpLine[x] = (Char)sgn(srcLineBelow[x] - srcLine[x- 1]);
      }

      //1st line
      Pel* srcLineAbove= srcLine- srcStride;
      firstLineStartX = isAboveLeftAvail ? 0 : 1;//该行像素左上像素是否可得到
      firstLineEndX   = isAboveAvail? endX: 1;
      for(x= firstLineStartX; x< firstLineEndX; x++)//第一行像素单独判断边界补偿分类 因为不确定第一行像素左上方像素是否可以获得 若左上像素无法获得 则无需判断该行像素
      {
        edgeType  =  sgn(srcLine[x] - srcLineAbove[x- 1]) - signUpLine[x+1];//与左上 右下像素比较大小 得到边界补偿分类

        resLine[x] = Clip3<Int>(0, maxSampleValueIncl, srcLine[x] + offset[edgeType]);//补偿后像素值
      }
      srcLine  += srcStride;//下一行
      resLine  += resStride;


      //middle lines
      for (y= 1; y< height-1; y++)//所有中间行计算边界补偿类型 中间行的上方和下方像素一定可得到
      {
        srcLineBelow= srcLine+ srcStride;//当前像素下方像素

        for (x=startX; x<endX; x++)//遍历该行
        {
          signDown =  (Char)sgn(srcLine[x] - srcLineBelow[x+ 1]);//该像素与右下像素大小
          edgeType =  signDown + signUpLine[x];//由左上 右下像素计算边界补偿分类
          resLine[x] = Clip3<Int>(0, maxSampleValueIncl, srcLine[x] + offset[edgeType]);//补偿后像素值

          signDownLine[x+1] = -signDown;//保存该行像素的signDown相反值
        }
        signDownLine[startX] = (Char)sgn(srcLineBelow[startX] - srcLine[startX-1]);//下行的第一个像素的signDownLine无法由上行像素得到  得由计算得到

        signTmpLine  = signUpLine;
        signUpLine   = signDownLine;//下行像素的signUp是上行像素的signDown 即下行像素与左上像素大小的sign是上行像素与右下像素大小的sign的相反值
        signDownLine = signTmpLine;

        srcLine += srcStride;//下一行
        resLine += resStride;
      }

      //last line
      srcLineBelow= srcLine+ srcStride;
      lastLineStartX = isBelowAvail ? startX : (width -1);
      lastLineEndX   = isBelowRightAvail ? width : (width -1);
      for(x= lastLineStartX; x< lastLineEndX; x++)//最后一行像素单独判断边界补偿分类 因为不确定最后行像素右下方像素是否可以获得 若右下像素无法获得 则无需判断该行像素
      {
        edgeType =  sgn(srcLine[x] - srcLineBelow[x+ 1]) + signUpLine[x];
        resLine[x] = Clip3<Int>(0, maxSampleValueIncl, srcLine[x] + offset[edgeType]);

      }
    }
    break;
  case SAO_TYPE_EO_45://45度方向同135度方向 故不在叙述
    {
      offset += 2;
      Char *signUpLine = m_signLineBuf1+1;

      startX = isLeftAvail ? 0 : 1;
      endX   = isRightAvail ? width : (width -1);

      //prepare 2nd line upper sign
      Pel* srcLineBelow= srcLine+ srcStride;
      for (x=startX-1; x< endX; x++)
      {
        signUpLine[x] = (Char)sgn(srcLineBelow[x] - srcLine[x+1]);
      }


      //first line
      Pel* srcLineAbove= srcLine- srcStride;
      firstLineStartX = isAboveAvail ? startX : (width -1 );
      firstLineEndX   = isAboveRightAvail ? width : (width-1);
      for(x= firstLineStartX; x< firstLineEndX; x++)
      {
        edgeType = sgn(srcLine[x] - srcLineAbove[x+1]) -signUpLine[x-1];
        resLine[x] = Clip3<Int>(0, maxSampleValueIncl, srcLine[x] + offset[edgeType]);
      }
      srcLine += srcStride;
      resLine += resStride;

      //middle lines
      for (y= 1; y< height-1; y++)
      {
        srcLineBelow= srcLine+ srcStride;

        for(x= startX; x< endX; x++)
        {
          signDown =  (Char)sgn(srcLine[x] - srcLineBelow[x-1]);
          edgeType =  signDown + signUpLine[x];
          resLine[x] = Clip3<Int>(0, maxSampleValueIncl, srcLine[x] + offset[edgeType]);
          signUpLine[x-1] = -signDown;
        }
        signUpLine[endX-1] = (Char)sgn(srcLineBelow[endX-1] - srcLine[endX]);
        srcLine  += srcStride;
        resLine += resStride;
      }

      //last line
      srcLineBelow= srcLine+ srcStride;
      lastLineStartX = isBelowLeftAvail ? 0 : 1;
      lastLineEndX   = isBelowAvail ? endX : 1;
      for(x= lastLineStartX; x< lastLineEndX; x++)
      {
        edgeType = sgn(srcLine[x] - srcLineBelow[x-1]) + signUpLine[x];
        resLine[x] = Clip3<Int>(0, maxSampleValueIncl, srcLine[x] + offset[edgeType]);

      }
    }
    break;
  case SAO_TYPE_BO://边带补偿
    {
      const Int shiftBits = channelBitDepth - NUM_SAO_BO_CLASSES_LOG2;
      for (y=0; y< height; y++)//为每个像素计算补偿后的值
      {
        for (x=0; x< width; x++)
        {
          resLine[x] = Clip3<Int>(0, maxSampleValueIncl, srcLine[x] + offset[srcLine[x] >> shiftBits] );//srcLine[x] >> shiftBits得到该像素为32条边带中那一条
        }
        srcLine += srcStride;//下一行
        resLine += resStride;
      }
    }
    break;
  default:
    {
      printf("Not a supported SAO types\n");
      assert(0);
      exit(-1);
    }
  }
}

Void TComSampleAdaptiveOffset::offsetCTU(Int ctuRsAddr, TComPicYuv* srcYuv, TComPicYuv* resYuv, SAOBlkParam& saoblkParam, TComPic* pPic)
{//为CTU中各个分量SAO补偿
  Bool isLeftAvail,isRightAvail,isAboveAvail,isBelowAvail,isAboveLeftAvail,isAboveRightAvail,isBelowLeftAvail,isBelowRightAvail;

  const Int numberOfComponents = getNumberValidComponents(m_chromaFormatIDC);
  Bool bAllOff=true;
  for(Int compIdx = 0; compIdx < numberOfComponents; compIdx++)//若所有分量均不需要补偿 则方法直接结束
  {
    if (saoblkParam[compIdx].modeIdc != SAO_MODE_OFF)
    {
      bAllOff=false;
    }
  }
  if (bAllOff)
  {
    return;
  }

  //block boundary availability
  pPic->getPicSym()->deriveLoopFilterBoundaryAvailibility(ctuRsAddr, isLeftAvail,isRightAvail,isAboveAvail,isBelowAvail,isAboveLeftAvail,isAboveRightAvail,isBelowLeftAvail,isBelowRightAvail);

  Int yPos   = (ctuRsAddr / m_numCTUInWidth)*m_maxCUHeight;//该CTU在图像中的位置(CTU为单位)
  Int xPos   = (ctuRsAddr % m_numCTUInWidth)*m_maxCUWidth;
  Int height = (yPos + m_maxCUHeight > m_picHeight)?(m_picHeight- yPos):m_maxCUHeight;
  Int width  = (xPos + m_maxCUWidth  > m_picWidth )?(m_picWidth - xPos):m_maxCUWidth;

  for(Int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    const ComponentID component = ComponentID(compIdx);
    SAOOffset& ctbOffset = saoblkParam[compIdx];

    if(ctbOffset.modeIdc != SAO_MODE_OFF)
    {
      const UInt componentScaleX = getComponentScaleX(component, pPic->getChromaFormat());
      const UInt componentScaleY = getComponentScaleY(component, pPic->getChromaFormat());

      Int  blkWidth   = (width  >> componentScaleX);
      Int  blkHeight  = (height >> componentScaleY);
      Int  blkXPos    = (xPos   >> componentScaleX);
      Int  blkYPos    = (yPos   >> componentScaleY);

      Int  srcStride  = srcYuv->getStride(component);//该分量图像中步长大小
      Pel* srcBlk     = srcYuv->getAddr(component) + blkYPos*srcStride + blkXPos;//该分量中当前CTU的首像素在图像中的位置（像素为单位）

      Int  resStride  = resYuv->getStride(component);
      Pel* resBlk     = resYuv->getAddr(component) + blkYPos*resStride + blkXPos;

      offsetBlock( pPic->getPicSym()->getSPS().getBitDepth(toChannelType(component)), ctbOffset.typeIdc, ctbOffset.offset
                  , srcBlk, resBlk, srcStride, resStride, blkWidth, blkHeight
                  , isLeftAvail, isRightAvail
                  , isAboveAvail, isBelowAvail
                  , isAboveLeftAvail, isAboveRightAvail
                  , isBelowLeftAvail, isBelowRightAvail
                  );//对该分量SAO补偿
    }
  } //compIdx

}


Void TComSampleAdaptiveOffset::SAOProcess(TComPic* pDecPic)
{//以CTU为单位 对整幅图像SAO补偿
  const Int numberOfComponents = getNumberValidComponents(m_chromaFormatIDC);
  Bool bAllDisabled=true;
  for(Int compIdx = 0; compIdx < numberOfComponents; compIdx++)//若所有分量均不需要补偿 则方法直接结束
  {
    if (m_picSAOEnabled[compIdx])
    {
      bAllDisabled=false;
    }
  }
  if (bAllDisabled)
  {
    return;
  }

  TComPicYuv* resYuv = pDecPic->getPicYuvRec();
  TComPicYuv* srcYuv = m_tempPicYuv;
  resYuv->copyToPic(srcYuv);
  for(Int ctuRsAddr= 0; ctuRsAddr < m_numCTUsPic; ctuRsAddr++)
  {
    offsetCTU(ctuRsAddr, srcYuv, resYuv, (pDecPic->getPicSym()->getSAOBlkParam())[ctuRsAddr], pDecPic);
  } //ctu
}


/** PCM LF disable process.
 * \param pcPic picture (TComPic) pointer
 *
 * \note Replace filtered sample values of PCM mode blocks with the transmitted and reconstructed ones.
 */
Void TComSampleAdaptiveOffset::PCMLFDisableProcess (TComPic* pcPic)
{
  xPCMRestoration(pcPic);
}

/** Picture-level PCM restoration.
 * \param pcPic picture (TComPic) pointer
 */
Void TComSampleAdaptiveOffset::xPCMRestoration(TComPic* pcPic)
{
  Bool  bPCMFilter = (pcPic->getSlice(0)->getSPS()->getUsePCM() && pcPic->getSlice(0)->getSPS()->getPCMFilterDisableFlag())? true : false;

  if(bPCMFilter || pcPic->getSlice(0)->getPPS()->getTransquantBypassEnableFlag())
  {
    for( UInt ctuRsAddr = 0; ctuRsAddr < pcPic->getNumberOfCtusInFrame() ; ctuRsAddr++ )
    {
      TComDataCU* pcCU = pcPic->getCtu(ctuRsAddr);

      xPCMCURestoration(pcCU, 0, 0);
    }
  }
}

/** PCM CU restoration.
 * \param pcCU            pointer to current CU
 * \param uiAbsZorderIdx  part index
 * \param uiDepth         CU depth
 */
Void TComSampleAdaptiveOffset::xPCMCURestoration ( TComDataCU* pcCU, UInt uiAbsZorderIdx, UInt uiDepth )
{
  TComPic* pcPic     = pcCU->getPic();
  UInt uiCurNumParts = pcPic->getNumPartitionsInCtu() >> (uiDepth<<1);
  UInt uiQNumParts   = uiCurNumParts>>2;

  // go to sub-CU
  if( pcCU->getDepth(uiAbsZorderIdx) > uiDepth )
  {
    for ( UInt uiPartIdx = 0; uiPartIdx < 4; uiPartIdx++, uiAbsZorderIdx+=uiQNumParts )
    {
      UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsZorderIdx] ];
      UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsZorderIdx] ];
      if( ( uiLPelX < pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ) && ( uiTPelY < pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples() ) )
      {
        xPCMCURestoration( pcCU, uiAbsZorderIdx, uiDepth+1 );
      }
    }
    return;
  }

  // restore PCM samples
  if ((pcCU->getIPCMFlag(uiAbsZorderIdx)&& pcPic->getSlice(0)->getSPS()->getPCMFilterDisableFlag()) || pcCU->isLosslessCoded( uiAbsZorderIdx))
  {
    const UInt numComponents=pcPic->getNumberValidComponents();
    for(UInt comp=0; comp<numComponents; comp++)
    {
      xPCMSampleRestoration (pcCU, uiAbsZorderIdx, uiDepth, ComponentID(comp));
    }
  }
}

/** PCM sample restoration.
 * \param pcCU           pointer to current CU
 * \param uiAbsZorderIdx part index
 * \param uiDepth        CU depth
 * \param compID         texture component type
 */
Void TComSampleAdaptiveOffset::xPCMSampleRestoration (TComDataCU* pcCU, UInt uiAbsZorderIdx, UInt uiDepth, const ComponentID compID)
{
        TComPicYuv* pcPicYuvRec = pcCU->getPic()->getPicYuvRec();
        UInt uiPcmLeftShiftBit;
  const UInt uiMinCoeffSize = pcCU->getPic()->getMinCUWidth()*pcCU->getPic()->getMinCUHeight();
  const UInt csx=pcPicYuvRec->getComponentScaleX(compID);
  const UInt csy=pcPicYuvRec->getComponentScaleY(compID);
  const UInt uiOffset   = (uiMinCoeffSize*uiAbsZorderIdx)>>(csx+csy);

        Pel *piSrc = pcPicYuvRec->getAddr(compID, pcCU->getCtuRsAddr(), uiAbsZorderIdx);
  const Pel *piPcm = pcCU->getPCMSample(compID) + uiOffset;
  const UInt uiStride  = pcPicYuvRec->getStride(compID);
  const TComSPS &sps = *(pcCU->getSlice()->getSPS());
  const UInt uiWidth  = ((sps.getMaxCUWidth()  >> uiDepth) >> csx);
  const UInt uiHeight = ((sps.getMaxCUHeight() >> uiDepth) >> csy);

  if ( pcCU->isLosslessCoded(uiAbsZorderIdx) && !pcCU->getIPCMFlag(uiAbsZorderIdx) )
  {
    uiPcmLeftShiftBit = 0;
  }
  else
  {
    uiPcmLeftShiftBit = sps.getBitDepth(toChannelType(compID)) - sps.getPCMBitDepth(toChannelType(compID));
  }

  for(UInt uiY = 0; uiY < uiHeight; uiY++ )
  {
    for(UInt uiX = 0; uiX < uiWidth; uiX++ )
    {
      piSrc[uiX] = (piPcm[uiX] << uiPcmLeftShiftBit);
    }
    piPcm += uiWidth;
    piSrc += uiStride;
  }
}

//! \}
