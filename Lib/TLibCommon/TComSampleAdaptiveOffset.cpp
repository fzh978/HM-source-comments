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

SAOOffset::SAOOffset()//���캯��
{
  reset();
}

SAOOffset::~SAOOffset()//��������
{

}

Void SAOOffset::reset()
{//SAOOffset��λ �����ʼ��������Ϣ
  modeIdc = SAO_MODE_OFF;
  typeIdc = -1;
  typeAuxInfo = -1;
  ::memset(offset, 0, sizeof(Int)* MAX_NUM_SAO_CLASSES);
}

const SAOOffset& SAOOffset::operator= (const SAOOffset& src)//��ֵ ���������
{
  modeIdc = src.modeIdc;
  typeIdc = src.typeIdc;
  typeAuxInfo = src.typeAuxInfo;
  ::memcpy(offset, src.offset, sizeof(Int)* MAX_NUM_SAO_CLASSES);

  return *this;
}
/*
*SAOOffset�ṹ���£�
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

SAOBlkParam::SAOBlkParam()//���캯��
{
  reset();
}

SAOBlkParam::~SAOBlkParam()//��������
{

}

Void SAOBlkParam::reset()
{//SAOBlkParam��ʼ�� ����3��������SAOOffset Ϊ����������SAOOffset��ֵ
  for(Int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    offsetParam[compIdx].reset();
  }
}

const SAOBlkParam& SAOBlkParam::operator= (const SAOBlkParam& src)//���������
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


TComSampleAdaptiveOffset::~TComSampleAdaptiveOffset()//��������  �ͷ���Դ
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
{//�������� �����ڴ�
  destroy();

  m_picWidth        = picWidth;
  m_picHeight       = picHeight;
  m_chromaFormatIDC = format;
  m_maxCUWidth      = maxCUWidth;
  m_maxCUHeight     = maxCUHeight;

  m_numCTUInWidth   = (m_picWidth/m_maxCUWidth) + ((m_picWidth % m_maxCUWidth)?1:0);//ͼ���CTU����
  m_numCTUInHeight  = (m_picHeight/m_maxCUHeight) + ((m_picHeight % m_maxCUHeight)?1:0);//ͼ����CTU����
  m_numCTUsPic      = m_numCTUInHeight*m_numCTUInWidth;//ͼ��CTU'�ܸ���

  //temporary picture buffer
  if ( !m_tempPicYuv )//�ݴ�ͼ��
  {
    m_tempPicYuv = new TComPicYuv;
    m_tempPicYuv->create( m_picWidth, m_picHeight, m_chromaFormatIDC, m_maxCUWidth, m_maxCUHeight, maxCUDepth, true );
  }

  //bit-depth related
  for(Int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)//�ֱ�Ϊ���� ɫ�ȷ�����offset��λ����ֵ
  {
    m_offsetStepLog2  [compIdx] = isLuma(ComponentID(compIdx))? lumaBitShift : chromaBitShift;
  }
}

Void TComSampleAdaptiveOffset::destroy()
{//���ٶ���
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

  if(typeIdc == SAO_TYPE_START_BO)//�ߴ�ģʽ
  {
    for(Int i=0; i< 4; i++)//CTUֻ��ѡ�����������ıߴ�
    {
      dstOffsets[(typeAuxInfo+ i)%NUM_SAO_BO_CLASSES] = codedOffset[(typeAuxInfo+ i)%NUM_SAO_BO_CLASSES]*(1<<m_offsetStepLog2[compIdx]);
    }
  }
  else //EO �߽�ģʽ
  {
    for(Int i=0; i< NUM_SAO_EO_CLASSES; i++)//����Ϊ5�ֱ߽�ģʽdstOffsets��ֵ
    {
      dstOffsets[i] = codedOffset[i] *(1<<m_offsetStepLog2[compIdx]);
    }
    assert(dstOffsets[SAO_CLASS_EO_PLAIN] == 0); //keep EO plain offset as zero
  }

}

Int TComSampleAdaptiveOffset::getMergeList(TComPic* pic, Int ctuRsAddr, SAOBlkParam* blkParams, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES])
{//SAO�����ں� ��MergeList�õ�SAO����
  Int ctuX = ctuRsAddr % m_numCTUInWidth;//��ǰCTU��ͼ���е�X���꣨��CTUΪ��λ��
  Int ctuY = ctuRsAddr / m_numCTUInWidth;//��ǰCTU��ͼ���е�Y���꣨��CTUΪ��λ��
  Int mergedCTUPos;
  Int numValidMergeCandidates = 0;

  for(Int mergeType=0; mergeType< NUM_SAO_MERGE_TYPES; mergeType++)//���������ں�ģʽ
  {
    SAOBlkParam* mergeCandidate = NULL;

    switch(mergeType)
    {
    case SAO_MERGE_ABOVE: //�ں��Ϸ������
      {
        if(ctuY > 0)//��Ϊ��һ�� �������Ϸ���
        {
          mergedCTUPos = ctuRsAddr- m_numCTUInWidth;//��ǰCTU�Ϸ�CTU��λ��
          if( pic->getSAOMergeAvailability(ctuRsAddr, mergedCTUPos) )//���Ϸ�CTU SAO�����ɻ��
          {
            mergeCandidate = &(blkParams[mergedCTUPos]);//�õ���ѡ����
          }
        }
      }
      break;
    case SAO_MERGE_LEFT://�ں��������
      {
        if(ctuX > 0)//��Ϊ��һ�� ����������
        {
          mergedCTUPos = ctuRsAddr- 1;//��ǰCTU���CTU��λ��
          if( pic->getSAOMergeAvailability(ctuRsAddr, mergedCTUPos) )//�����CTU SAO�����ɻ��
          {
            mergeCandidate = &(blkParams[mergedCTUPos]);//�õ���ѡ����
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

    mergeList[mergeType]=mergeCandidate;//��ǰCTU���ںϲ����б������Ϸ���
    if (mergeCandidate != NULL)
    {
      numValidMergeCandidates++;
    }
  }

  return numValidMergeCandidates;//������Ч��Ч�ںϺ�ѡ�����ĸ���
}


Void TComSampleAdaptiveOffset::reconstructBlkSAOParam(SAOBlkParam& recParam, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES])
{//�ؽ���CTU�����з�����SAO������Ϣ SAOBlkParam& recParam���������Ϣ
  const Int numberOfComponents = getNumberValidComponents(m_chromaFormatIDC);
  for(Int compIdx = 0; compIdx < numberOfComponents; compIdx++)//������������
  {
    const ComponentID component = ComponentID(compIdx);
    SAOOffset& offsetParam = recParam[component];//����������Ӧ��SAOOffset����

    if(offsetParam.modeIdc == SAO_MODE_OFF)//��÷���SAOģʽδ��  �����ִ����һ����
    {
      continue;
    }

    switch(offsetParam.modeIdc)
    {
    case SAO_MODE_NEW://�ߴ���߽�ģʽ
      {
        invertQuantOffsets(component, offsetParam.typeIdc, offsetParam.typeAuxInfo, offsetParam.offset, offsetParam.offset);
      }
      break;
    case SAO_MODE_MERGE://�����ں�ģʽ �������Ϸ������ֱ�ӵõ��ÿ�Ĳ���
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
{//�õ���ͼ������CTU���SAO������Ϣ
  for(Int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
  {
    m_picSAOEnabled[compIdx] = false;//��������SAOʹ�ܳ�ʼ����false
  }

  const Int numberOfComponents = getNumberValidComponents(m_chromaFormatIDC);

  for(Int ctuRsAddr=0; ctuRsAddr< m_numCTUsPic; ctuRsAddr++)//����ͼ������CTU��
  {
    SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES] = { NULL };
    getMergeList(pic, ctuRsAddr, saoBlkParams, mergeList);

    reconstructBlkSAOParam(saoBlkParams[ctuRsAddr], mergeList);//�ؽ���CTU��SAO����

    for(Int compIdx = 0; compIdx < numberOfComponents; compIdx++)//��÷����д��ڿ���SAOģʽ��CTU�� ��÷���SAOʹ��
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

  const Int maxSampleValueIncl = (1<< channelBitDepth )-1;//�������ֵ

  Int x,y, startX, startY, endX, endY, edgeType;
  Int firstLineStartX, firstLineEndX, lastLineStartX, lastLineEndX;
  Char signLeft, signRight, signDown;

  Pel* srcLine = srcBlk;//SAO����ǰ������ֵ
  Pel* resLine = resBlk;//SAO�����������ֵ

  switch(typeIdx)//SAO��������
  {
  case SAO_TYPE_EO_0://ˮƽ����߽粹��
    {
      offset += 2;
      startX = isLeftAvail ? 0 : 1;//��ǰ����������������
      endX   = isRightAvail ? width : (width -1);////��ǰ�����Ҳ����������
      for (y=0; y< height; y++)//������
      {
        signLeft = (Char)sgn(srcLine[startX] - srcLine[startX-1]);//��ǰ������������ش�С
        for (x=startX; x< endX; x++)//������
        {
          signRight = (Char)sgn(srcLine[x] - srcLine[x+1]); //��ǰ�������Ҳ����ش�С
          edgeType =  signRight + signLeft;//�߽粹������(��5�� -2 -1 0 1 2)
          signLeft  = -signRight;//���������ƶ�һ�� ��ǰ���ص�signRight����һ����signLeft���෴ֵ

          resLine[x] = Clip3<Int>(0, maxSampleValueIncl, srcLine[x] + offset[edgeType]);//��ͬ������϶�Ӧ����ֵ
        }
        srcLine  += srcStride;//��һ��
        resLine += resStride;
      }

    }
    break;
  case SAO_TYPE_EO_90://��ֱ����߽粹�� ��ˮƽ���������ͬ ��������
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
  case SAO_TYPE_EO_135://135�ȷ��� �߽粹��
    {
      offset += 2;
      Char *signUpLine, *signDownLine, *signTmpLine;

      signUpLine  = m_signLineBuf1;
      signDownLine= m_signLineBuf2;

      startX = isLeftAvail ? 0 : 1 ;//��֤��ǰ�������������
      endX   = isRightAvail ? width : (width-1);//��֤��ǰ�����Ҳ�������

      //prepare 2nd line's upper sign
      Pel* srcLineBelow= srcLine+ srcStride;//�ڶ�������
      for (x=startX; x< endX+1; x++)//�ȼ۵ڶ��������������Ͻ����ش�С
      {
        signUpLine[x] = (Char)sgn(srcLineBelow[x] - srcLine[x- 1]);
      }

      //1st line
      Pel* srcLineAbove= srcLine- srcStride;
      firstLineStartX = isAboveLeftAvail ? 0 : 1;//�����������������Ƿ�ɵõ�
      firstLineEndX   = isAboveAvail? endX: 1;
      for(x= firstLineStartX; x< firstLineEndX; x++)//��һ�����ص����жϱ߽粹������ ��Ϊ��ȷ����һ���������Ϸ������Ƿ���Ի�� �����������޷���� �������жϸ�������
      {
        edgeType  =  sgn(srcLine[x] - srcLineAbove[x- 1]) - signUpLine[x+1];//������ �������رȽϴ�С �õ��߽粹������

        resLine[x] = Clip3<Int>(0, maxSampleValueIncl, srcLine[x] + offset[edgeType]);//����������ֵ
      }
      srcLine  += srcStride;//��һ��
      resLine  += resStride;


      //middle lines
      for (y= 1; y< height-1; y++)//�����м��м���߽粹������ �м��е��Ϸ����·�����һ���ɵõ�
      {
        srcLineBelow= srcLine+ srcStride;//��ǰ�����·�����

        for (x=startX; x<endX; x++)//��������
        {
          signDown =  (Char)sgn(srcLine[x] - srcLineBelow[x+ 1]);//���������������ش�С
          edgeType =  signDown + signUpLine[x];//������ �������ؼ���߽粹������
          resLine[x] = Clip3<Int>(0, maxSampleValueIncl, srcLine[x] + offset[edgeType]);//����������ֵ

          signDownLine[x+1] = -signDown;//����������ص�signDown�෴ֵ
        }
        signDownLine[startX] = (Char)sgn(srcLineBelow[startX] - srcLine[startX-1]);//���еĵ�һ�����ص�signDownLine�޷����������صõ�  ���ɼ���õ�

        signTmpLine  = signUpLine;
        signUpLine   = signDownLine;//�������ص�signUp���������ص�signDown �������������������ش�С��sign�������������������ش�С��sign���෴ֵ
        signDownLine = signTmpLine;

        srcLine += srcStride;//��һ��
        resLine += resStride;
      }

      //last line
      srcLineBelow= srcLine+ srcStride;
      lastLineStartX = isBelowAvail ? startX : (width -1);
      lastLineEndX   = isBelowRightAvail ? width : (width -1);
      for(x= lastLineStartX; x< lastLineEndX; x++)//���һ�����ص����жϱ߽粹������ ��Ϊ��ȷ��������������·������Ƿ���Ի�� �����������޷���� �������жϸ�������
      {
        edgeType =  sgn(srcLine[x] - srcLineBelow[x+ 1]) + signUpLine[x];
        resLine[x] = Clip3<Int>(0, maxSampleValueIncl, srcLine[x] + offset[edgeType]);

      }
    }
    break;
  case SAO_TYPE_EO_45://45�ȷ���ͬ135�ȷ��� �ʲ�������
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
  case SAO_TYPE_BO://�ߴ�����
    {
      const Int shiftBits = channelBitDepth - NUM_SAO_BO_CLASSES_LOG2;
      for (y=0; y< height; y++)//Ϊÿ�����ؼ��㲹�����ֵ
      {
        for (x=0; x< width; x++)
        {
          resLine[x] = Clip3<Int>(0, maxSampleValueIncl, srcLine[x] + offset[srcLine[x] >> shiftBits] );//srcLine[x] >> shiftBits�õ�������Ϊ32���ߴ�����һ��
        }
        srcLine += srcStride;//��һ��
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
{//ΪCTU�и�������SAO����
  Bool isLeftAvail,isRightAvail,isAboveAvail,isBelowAvail,isAboveLeftAvail,isAboveRightAvail,isBelowLeftAvail,isBelowRightAvail;

  const Int numberOfComponents = getNumberValidComponents(m_chromaFormatIDC);
  Bool bAllOff=true;
  for(Int compIdx = 0; compIdx < numberOfComponents; compIdx++)//�����з���������Ҫ���� �򷽷�ֱ�ӽ���
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

  Int yPos   = (ctuRsAddr / m_numCTUInWidth)*m_maxCUHeight;//��CTU��ͼ���е�λ��(CTUΪ��λ)
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

      Int  srcStride  = srcYuv->getStride(component);//�÷���ͼ���в�����С
      Pel* srcBlk     = srcYuv->getAddr(component) + blkYPos*srcStride + blkXPos;//�÷����е�ǰCTU����������ͼ���е�λ�ã�����Ϊ��λ��

      Int  resStride  = resYuv->getStride(component);
      Pel* resBlk     = resYuv->getAddr(component) + blkYPos*resStride + blkXPos;

      offsetBlock( pPic->getPicSym()->getSPS().getBitDepth(toChannelType(component)), ctbOffset.typeIdc, ctbOffset.offset
                  , srcBlk, resBlk, srcStride, resStride, blkWidth, blkHeight
                  , isLeftAvail, isRightAvail
                  , isAboveAvail, isBelowAvail
                  , isAboveLeftAvail, isAboveRightAvail
                  , isBelowLeftAvail, isBelowRightAvail
                  );//�Ը÷���SAO����
    }
  } //compIdx

}


Void TComSampleAdaptiveOffset::SAOProcess(TComPic* pDecPic)
{//��CTUΪ��λ ������ͼ��SAO����
  const Int numberOfComponents = getNumberValidComponents(m_chromaFormatIDC);
  Bool bAllDisabled=true;
  for(Int compIdx = 0; compIdx < numberOfComponents; compIdx++)//�����з���������Ҫ���� �򷽷�ֱ�ӽ���
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
  Bool  bPCMFilter = (pcPic->getSlice(0)->getSPS()->getUsePCM() && pcPic->getSlice(0)->getSPS()->getPCMFilterDisableFlag())? true : false;//PCM模式且不使用PCM滤波

  if(bPCMFilter || pcPic->getSlice(0)->getPPS()->getTransquantBypassEnableFlag())//为lossless模式 或PCM模式下不使用PCM滤波
  {
    for( UInt ctuRsAddr = 0; ctuRsAddr < pcPic->getNumberOfCtusInFrame() ; ctuRsAddr++ )//依次处理图像中的每个Ctu
    {
      TComDataCU* pcCU = pcPic->getCtu(ctuRsAddr);

      xPCMCURestoration(pcCU, 0, 0);//用PCM像素值还原重建像素
    }
  }
}

/** PCM CU restoration.
 * \param pcCU            pointer to current CU
 * \param uiAbsZorderIdx  part index
 * \param uiDepth         CU depth
 */
Void TComSampleAdaptiveOffset::xPCMCURestoration ( TComDataCU* pcCU, UInt uiAbsZorderIdx, UInt uiDepth )//PCM/lossless模式下得到Cu的重建像素值
{
  TComPic* pcPic     = pcCU->getPic();
  UInt uiCurNumParts = pcPic->getNumPartitionsInCtu() >> (uiDepth<<1);
  UInt uiQNumParts   = uiCurNumParts>>2;

  // go to sub-CU
  if( pcCU->getDepth(uiAbsZorderIdx) > uiDepth )//若该Cu存在子Cu
  {
    for ( UInt uiPartIdx = 0; uiPartIdx < 4; uiPartIdx++, uiAbsZorderIdx+=uiQNumParts )//则递归处理4个子Cu
    {
      UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsZorderIdx] ];
      UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsZorderIdx] ];
      if( ( uiLPelX < pcCU->getSlice()->getSPS()->getPicWidthInLumaSamples() ) && ( uiTPelY < pcCU->getSlice()->getSPS()->getPicHeightInLumaSamples() ) )//不得超出图像边界
      {
        xPCMCURestoration( pcCU, uiAbsZorderIdx, uiDepth+1 );
      }
    }
    return;
  }

  // restore PCM samples
  if ((pcCU->getIPCMFlag(uiAbsZorderIdx)&& pcPic->getSlice(0)->getSPS()->getPCMFilterDisableFlag()) || pcCU->isLosslessCoded( uiAbsZorderIdx))//该Cu为PCM模式且PCM下不滤波 或该Cu为lossless模式!!!
  {
    const UInt numComponents=pcPic->getNumberValidComponents();
    for(UInt comp=0; comp<numComponents; comp++)
    {
      xPCMSampleRestoration (pcCU, uiAbsZorderIdx, uiDepth, ComponentID(comp));//得到图像的重建像素值
    }
  }
}

/** PCM sample restoration.
 * \param pcCU           pointer to current CU
 * \param uiAbsZorderIdx part index
 * \param uiDepth        CU depth
 * \param compID         texture component type
 */
Void TComSampleAdaptiveOffset::xPCMSampleRestoration (TComDataCU* pcCU, UInt uiAbsZorderIdx, UInt uiDepth, const ComponentID compID)//由PCM像素得到重建像素值
{
        TComPicYuv* pcPicYuvRec = pcCU->getPic()->getPicYuvRec();
        UInt uiPcmLeftShiftBit;
  const UInt uiMinCoeffSize = pcCU->getPic()->getMinCUWidth()*pcCU->getPic()->getMinCUHeight();
  const UInt csx=pcPicYuvRec->getComponentScaleX(compID);
  const UInt csy=pcPicYuvRec->getComponentScaleY(compID);
  const UInt uiOffset   = (uiMinCoeffSize*uiAbsZorderIdx)>>(csx+csy);

        Pel *piSrc = pcPicYuvRec->getAddr(compID, pcCU->getCtuRsAddr(), uiAbsZorderIdx);//重建像素起始地址
  const Pel *piPcm = pcCU->getPCMSample(compID) + uiOffset;//PCM像素起始地址
  const UInt uiStride  = pcPicYuvRec->getStride(compID);
  const TComSPS &sps = *(pcCU->getSlice()->getSPS());
  const UInt uiWidth  = ((sps.getMaxCUWidth()  >> uiDepth) >> csx);
  const UInt uiHeight = ((sps.getMaxCUHeight() >> uiDepth) >> csy);

  if ( pcCU->isLosslessCoded(uiAbsZorderIdx) && !pcCU->getIPCMFlag(uiAbsZorderIdx) )//lossless模式下 不移位
  {
    uiPcmLeftShiftBit = 0;
  }
  else
  {
    uiPcmLeftShiftBit = sps.getBitDepth(toChannelType(compID)) - sps.getPCMBitDepth(toChannelType(compID));
  }

  for(UInt uiY = 0; uiY < uiHeight; uiY++ )//遍历每个像素
  {
    for(UInt uiX = 0; uiX < uiWidth; uiX++ )
    {
      piSrc[uiX] = (piPcm[uiX] << uiPcmLeftShiftBit);//将还原位深后的PCM像素值赋给重建像素
    }
    piPcm += uiWidth;
    piSrc += uiStride;
  }
}

//! \}
