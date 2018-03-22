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

/** \file     TComSlice.cpp
    \brief    slice header and SPS class
*/

#include "CommonDef.h"
#include "TComSlice.h"
#include "TComPic.h"
#include "TLibEncoder/TEncSbac.h"
#include "TLibDecoder/TDecSbac.h"


//! \ingroup TLibCommon
//! \{
/**该部分程序涉及到一些参考图像管理不好理解的概念 如RPS RPS预测 各种图像类型等
 * 建议在阅读先仔细阅读理解《HEVC Algorithms and Archietectures》中第2章和
 * 《software manual》中关于GOP structure的描述！
 */
TComSlice::TComSlice()//构造函数　为Slice中的参数附上默认值
: m_iPPSId                        ( -1 )
, m_PicOutputFlag                 ( true )
, m_iPOC                          ( 0 )
, m_iLastIDR                      ( 0 )
, m_iAssociatedIRAP               ( 0 )
, m_iAssociatedIRAPType           ( NAL_UNIT_INVALID )
, m_pRPS                          ( 0 )
, m_localRPS                      ( )
, m_rpsIdx                        ( 0 )
, m_RefPicListModification        ( )
, m_eNalUnitType                  ( NAL_UNIT_CODED_SLICE_IDR_W_RADL )
, m_eSliceType                    ( I_SLICE )
, m_iSliceQp                      ( 0 )
, m_dependentSliceSegmentFlag     ( false )
#if ADAPTIVE_QP_SELECTION
, m_iSliceQpBase                  ( 0 )
#endif
, m_ChromaQpAdjEnabled            ( false )
, m_deblockingFilterDisable       ( false )
, m_deblockingFilterOverrideFlag  ( false )
, m_deblockingFilterBetaOffsetDiv2( 0 )
, m_deblockingFilterTcOffsetDiv2  ( 0 )
, m_bCheckLDC                     ( false )
, m_iSliceQpDelta                 ( 0 )
, m_iDepth                        ( 0 )
, m_bRefenced                     ( false )
, m_pcVPS                         ( NULL )
, m_pcSPS                         ( NULL )
, m_pcPPS                         ( NULL )
, m_pcPic                         ( NULL )
, m_colFromL0Flag                 ( true )
, m_noOutputPriorPicsFlag         ( false )
, m_noRaslOutputFlag              ( false )
, m_handleCraAsBlaFlag            ( false )
, m_colRefIdx                     ( 0 )
, m_maxNumMergeCand               ( 0 )
, m_uiTLayer                      ( 0 )
, m_bTLayerSwitchingFlag          ( false )
, m_sliceMode                     ( NO_SLICES )
, m_sliceArgument                 ( 0 )
, m_sliceCurStartCtuTsAddr        ( 0 )
, m_sliceCurEndCtuTsAddr          ( 0 )
, m_sliceIdx                      ( 0 )
, m_sliceSegmentMode              ( NO_SLICES )
, m_sliceSegmentArgument          ( 0 )
, m_sliceSegmentCurStartCtuTsAddr ( 0 )
, m_sliceSegmentCurEndCtuTsAddr   ( 0 )
, m_nextSlice                     ( false )
, m_nextSliceSegment              ( false )
, m_sliceBits                     ( 0 )
, m_sliceSegmentBits              ( 0 )
, m_bFinalized                    ( false )
, m_bTestWeightPred               ( false )
, m_bTestWeightBiPred             ( false )
, m_substreamSizes                ( )
, m_cabacInitFlag                 ( false )
, m_bLMvdL1Zero                   ( false )
, m_temporalLayerNonReferenceFlag ( false )
, m_LFCrossSliceBoundaryFlag      ( false )
, m_enableTMVPFlag                ( true )
, m_encCABACTableIdx              (I_SLICE)
{
  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_aiNumRefIdx[i] = 0;
  }

  for (UInt component = 0; component < MAX_NUM_COMPONENT; component++)
  {
    m_lambdas            [component] = 0.0;
    m_iSliceChromaQpDelta[component] = 0;
  }

  initEqualRef();

  for ( Int idx = 0; idx < MAX_NUM_REF; idx++ )
  {
    m_list1IdxToList0Idx[idx] = -1;
  }

  for(Int iNumCount = 0; iNumCount < MAX_NUM_REF; iNumCount++)
  {
    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      m_apcRefPicList [i][iNumCount] = NULL;
      m_aiRefPOCList  [i][iNumCount] = 0;
    }
  }

  resetWpScaling();
  initWpAcDcParam();

  for(Int ch=0; ch < MAX_NUM_CHANNEL_TYPE; ch++)
  {
    m_saoEnabledFlag[ch] = false;
  }
}

TComSlice::~TComSlice()//析构函数
{
}


Void TComSlice::initSlice()//为slice中部分参数初始化
{
  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_aiNumRefIdx[i]      = 0;
  }
  m_colFromL0Flag = true;

  m_colRefIdx = 0;
  initEqualRef();

  m_bCheckLDC = false;

  for (UInt component = 0; component < MAX_NUM_COMPONENT; component++)
  {
    m_iSliceChromaQpDelta[component] = 0;
  }

  m_maxNumMergeCand = MRG_MAX_NUM_CANDS;

  m_bFinalized=false;

  m_substreamSizes.clear();
  m_cabacInitFlag        = false;
  m_enableTMVPFlag = true;
}

Bool TComSlice::getRapPicFlag() const//判断是否为IRAP（Intra random access point (IRAP) pictures）
{
  return getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA;
}


Void  TComSlice::sortPicList (TComList<TComPic*>& rcListPic)//冒泡排序法!　只不过该排序是根据图像的POC值对图像在列表中的位置排序
{
  TComPic*    pcPicExtract;
  TComPic*    pcPicInsert;

  TComList<TComPic*>::iterator    iterPicExtract;
  TComList<TComPic*>::iterator    iterPicExtract_1;
  TComList<TComPic*>::iterator    iterPicInsert;

  for (Int i = 1; i < (Int)(rcListPic.size()); i++)
  {
    iterPicExtract = rcListPic.begin();
    for (Int j = 0; j < i; j++)
    {
      iterPicExtract++;
    }
    pcPicExtract = *(iterPicExtract);
    pcPicExtract->setCurrSliceIdx(0);

    iterPicInsert = rcListPic.begin();
    while (iterPicInsert != iterPicExtract)
    {
      pcPicInsert = *(iterPicInsert);
      pcPicInsert->setCurrSliceIdx(0);
      if (pcPicInsert->getPOC() >= pcPicExtract->getPOC())
      {
        break;
      }

      iterPicInsert++;
    }

    iterPicExtract_1 = iterPicExtract;    iterPicExtract_1++;

    //  swap iterPicExtract and iterPicInsert, iterPicExtract = curr. / iterPicInsert = insertion position
    rcListPic.insert (iterPicInsert, iterPicExtract, iterPicExtract_1);
    rcListPic.erase  (iterPicExtract);//这两步完成相邻两幅图像在列表中位置的交换
  }
}

TComPic* TComSlice::xGetRefPic (TComList<TComPic*>& rcListPic, Int poc)//在图像列表中找到POC值为poc的参考图像
{
  TComList<TComPic*>::iterator  iterPic = rcListPic.begin();
  TComPic*                      pcPic = *(iterPic);
  while ( iterPic != rcListPic.end() )
  {
    if(pcPic->getPOC() == poc)
    {
      break;
    }
    iterPic++;
    pcPic = *(iterPic);
  }
  return  pcPic;
}


TComPic* TComSlice::xGetLongTermRefPic(TComList<TComPic*>& rcListPic, Int poc, Bool pocHasMsb)//在图像列表中找到POC值为poc的参考图像
{//Msb为最高有效位　pocHasMsb表示poc值是否不被限制有效位（最高位是否有效）
  TComList<TComPic*>::iterator  iterPic = rcListPic.begin();
  TComPic*                      pcPic = *(iterPic);
  TComPic*                      pcStPic = pcPic;

  Int pocCycle = 1 << getSPS()->getBitsForPOC();
  if (!pocHasMsb)
  {
    poc = poc & (pocCycle - 1);
  }

  while ( iterPic != rcListPic.end() )
  {
    pcPic = *(iterPic);
    if (pcPic && pcPic->getPOC()!=this->getPOC() && pcPic->getSlice( 0 )->isReferenced())
    {
      Int picPoc = pcPic->getPOC();
      if (!pocHasMsb)
      {
        picPoc = picPoc & (pocCycle - 1);
      }

      if (poc == picPoc)
      {
        if(pcPic->getIsLongTerm())
        {
          return pcPic;
        }
        else
        {
          pcStPic = pcPic;
        }
        break;
      }
    }

    iterPic++;
  }

  return  pcStPic;
}

Void TComSlice::setRefPOCList ()//依次将参考图像列表中的图像转换成其POC值
{
  for (Int iDir = 0; iDir < NUM_REF_PIC_LIST_01; iDir++)
  {
    for (Int iNumRefIdx = 0; iNumRefIdx < m_aiNumRefIdx[iDir]; iNumRefIdx++)//参考图像索引数　注意:参考图像在参考图像列表中的索引表示在列表中的位置　POC表示其为哪一帧图像　二者一般不相等
    {
      m_aiRefPOCList[iDir][iNumRefIdx] = m_apcRefPicList[iDir][iNumRefIdx]->getPOC();
    }
  }

}

Void TComSlice::setList1IdxToList0Idx()//建立list1参考图像索引到list0参考图像索引的映射表
{
  Int idxL0, idxL1;
  for ( idxL1 = 0; idxL1 < getNumRefIdx( REF_PIC_LIST_1 ); idxL1++ )
  {
    m_list1IdxToList0Idx[idxL1] = -1;
    for ( idxL0 = 0; idxL0 < getNumRefIdx( REF_PIC_LIST_0 ); idxL0++ )
    {
      if ( m_apcRefPicList[REF_PIC_LIST_0][idxL0]->getPOC() == m_apcRefPicList[REF_PIC_LIST_1][idxL1]->getPOC() )//若两列表中存在相同的参考图像
      {
        m_list1IdxToList0Idx[idxL1] = idxL0;//则添加list1中该图像的索引到list0中该图像索引的映射
        break;
      }
    }
  }
}

Void TComSlice::setRefPicList( TComList<TComPic*>& rcListPic, Bool checkNumPocTotalCurr )
{
  if (!checkNumPocTotalCurr)
  {
    if (m_eSliceType == I_SLICE)//I帧为帧内预测　不存在参考图像直接返回
    {
      ::memset( m_apcRefPicList, 0, sizeof (m_apcRefPicList));
      ::memset( m_aiNumRefIdx,   0, sizeof ( m_aiNumRefIdx ));

      return;
    }

    m_aiNumRefIdx[REF_PIC_LIST_0] = getNumRefIdx(REF_PIC_LIST_0);//list0参考图像数
    m_aiNumRefIdx[REF_PIC_LIST_1] = getNumRefIdx(REF_PIC_LIST_1);//list１参考图像数
  }

  TComPic*  pcRefPic= NULL;
  static const UInt MAX_NUM_NEGATIVE_PICTURES=16;
  TComPic*  RefPicSetStCurr0[MAX_NUM_NEGATIVE_PICTURES];
  TComPic*  RefPicSetStCurr1[MAX_NUM_NEGATIVE_PICTURES];
  TComPic*  RefPicSetLtCurr[MAX_NUM_NEGATIVE_PICTURES];
  UInt NumPicStCurr0 = 0;
  UInt NumPicStCurr1 = 0;
  UInt NumPicLtCurr = 0;
  Int i;

  for(i=0; i < m_pRPS->getNumberOfNegativePictures(); i++)//设置前向short term参考图像
  {
    if(m_pRPS->getUsed(i))//如果该图像被当前图像用作参考图像
    {
      pcRefPic = xGetRefPic(rcListPic, getPOC()+m_pRPS->getDeltaPOC(i));//得到该参考图像
      pcRefPic->setIsLongTerm(0);//不为long term参考图像
      pcRefPic->getPicYuvRec()->extendPicBorder();
      RefPicSetStCurr0[NumPicStCurr0] = pcRefPic;//将该图像添加至RefPicSetStCurr0列表中
      NumPicStCurr0++;//索引+1
      pcRefPic->setCheckLTMSBPresent(false);//是否检查short term参考图像pOC值最高有效位
    }
  }

  for(; i < m_pRPS->getNumberOfNegativePictures()+m_pRPS->getNumberOfPositivePictures(); i++)//设置后向short term参考图像
  {
    if(m_pRPS->getUsed(i))
    {
      pcRefPic = xGetRefPic(rcListPic, getPOC()+m_pRPS->getDeltaPOC(i));
      pcRefPic->setIsLongTerm(0);
      pcRefPic->getPicYuvRec()->extendPicBorder();
      RefPicSetStCurr1[NumPicStCurr1] = pcRefPic;//将该图像添加至RefPicSetStCurr１列表中
      NumPicStCurr1++;
      pcRefPic->setCheckLTMSBPresent(false);
    }
  }

  for(i = m_pRPS->getNumberOfNegativePictures()+m_pRPS->getNumberOfPositivePictures()+m_pRPS->getNumberOfLongtermPictures()-1; i > m_pRPS->getNumberOfNegativePictures()+m_pRPS->getNumberOfPositivePictures()-1 ; i--)
  {//设置long term参考图像　
    if(m_pRPS->getUsed(i))
    {
      pcRefPic = xGetLongTermRefPic(rcListPic, m_pRPS->getPOC(i), m_pRPS->getCheckLTMSBPresent(i));
      pcRefPic->setIsLongTerm(1);//为long term参考图像
      pcRefPic->getPicYuvRec()->extendPicBorder();
      RefPicSetLtCurr[NumPicLtCurr] = pcRefPic;
      NumPicLtCurr++;
    }
    if(pcRefPic==NULL)
    {
      pcRefPic = xGetLongTermRefPic(rcListPic, m_pRPS->getPOC(i), m_pRPS->getCheckLTMSBPresent(i));
    }
    pcRefPic->setCheckLTMSBPresent(m_pRPS->getCheckLTMSBPresent(i));
  }

  // ref_pic_list_init
  TComPic*  rpsCurrList0[MAX_NUM_REF+1];
  TComPic*  rpsCurrList1[MAX_NUM_REF+1];
  Int numPicTotalCurr = NumPicStCurr0 + NumPicStCurr1 + NumPicLtCurr;//参考图像总数

  if (checkNumPocTotalCurr)//检查参考图像总数
  {
    // The variable NumPocTotalCurr is derived as specified in subclause 7.4.7.2. It is a requirement of bitstream conformance that the following applies to the value of NumPocTotalCurr:
    // - If the current picture is a BLA or CRA picture, the value of NumPocTotalCurr shall be equal to 0.
    // - Otherwise, when the current picture contains a P or B slice, the value of NumPocTotalCurr shall not be equal to 0.
    if (getRapPicFlag())//IPAP为帧内预测　不存在参考图像　故numPicTotalCurr一定为０
    {
      assert(numPicTotalCurr == 0);
    }

    if (m_eSliceType == I_SLICE)//I帧为帧内预测　不存在参考图像直接返回
    {
      ::memset( m_apcRefPicList, 0, sizeof (m_apcRefPicList));
      ::memset( m_aiNumRefIdx,   0, sizeof ( m_aiNumRefIdx ));

      return;
    }

    assert(numPicTotalCurr > 0);//不为I slice　一定存在参考图像
    // general tier and level limit:
    assert(numPicTotalCurr <= 8);//general tier and level限制参考图像数不得大于8

    m_aiNumRefIdx[0] = getNumRefIdx(REF_PIC_LIST_0);
    m_aiNumRefIdx[1] = getNumRefIdx(REF_PIC_LIST_1);
  }

  Int cIdx = 0;
  for ( i=0; i<NumPicStCurr0; i++, cIdx++)//将得到的前向short term参考图像添加到rpsCurrList0中
  {
    rpsCurrList0[cIdx] = RefPicSetStCurr0[i];
  }
  for ( i=0; i<NumPicStCurr1; i++, cIdx++)//将得到的后向short term参考图像添加到rpsCurrList0中
  {
    rpsCurrList0[cIdx] = RefPicSetStCurr1[i];
  }
  for ( i=0; i<NumPicLtCurr;  i++, cIdx++)//将得到的long term参考图像添加到rpsCurrList0中
  {
    rpsCurrList0[cIdx] = RefPicSetLtCurr[i];
  }
  assert(cIdx == numPicTotalCurr);

  if (m_eSliceType==B_SLICE)//若为B slice则还需rpsCurrList1 双向预测为前后两个相反的方向　故list1中前向和后向参考图像与list0中相反
  {
    cIdx = 0;
    for ( i=0; i<NumPicStCurr1; i++, cIdx++)
    {
      rpsCurrList1[cIdx] = RefPicSetStCurr1[i];
    }
    for ( i=0; i<NumPicStCurr0; i++, cIdx++)
    {
      rpsCurrList1[cIdx] = RefPicSetStCurr0[i];
    }
    for ( i=0; i<NumPicLtCurr;  i++, cIdx++)
    {
      rpsCurrList1[cIdx] = RefPicSetLtCurr[i];
    }
    assert(cIdx == numPicTotalCurr);
  }

  ::memset(m_bIsUsedAsLongTerm, 0, sizeof(m_bIsUsedAsLongTerm));

  for (Int rIdx = 0; rIdx < m_aiNumRefIdx[REF_PIC_LIST_0]; rIdx ++)//构建最终指定大小的参考图像列表0　
  {//RefPicSetIdx为codewords　构建最终的参考列表时用于指定暂时参考列表中的参考图像
    cIdx = m_RefPicListModification.getRefPicListModificationFlagL0() ? m_RefPicListModification.getRefPicSetIdxL0(rIdx) : rIdx % numPicTotalCurr;
    assert(cIdx >= 0 && cIdx < numPicTotalCurr);
    m_apcRefPicList[REF_PIC_LIST_0][rIdx] = rpsCurrList0[ cIdx ];
    m_bIsUsedAsLongTerm[REF_PIC_LIST_0][rIdx] = ( cIdx >= NumPicStCurr0 + NumPicStCurr1 );
  }
  if ( m_eSliceType != B_SLICE )//不为B slice　则list1为０
  {
    m_aiNumRefIdx[REF_PIC_LIST_1] = 0;
    ::memset( m_apcRefPicList[REF_PIC_LIST_1], 0, sizeof(m_apcRefPicList[REF_PIC_LIST_1]));
  }
  else
  {
    for (Int rIdx = 0; rIdx < m_aiNumRefIdx[REF_PIC_LIST_1]; rIdx ++)//构建最终指定大小的参考图像列表1
    {
      cIdx = m_RefPicListModification.getRefPicListModificationFlagL1() ? m_RefPicListModification.getRefPicSetIdxL1(rIdx) : rIdx % numPicTotalCurr;
      assert(cIdx >= 0 && cIdx < numPicTotalCurr);
      m_apcRefPicList[REF_PIC_LIST_1][rIdx] = rpsCurrList1[ cIdx ];
      m_bIsUsedAsLongTerm[REF_PIC_LIST_1][rIdx] = ( cIdx >= NumPicStCurr0 + NumPicStCurr1 );
    }
  }
}

Int TComSlice::getNumRpsCurrTempList() const//计算总的参考图像数
{
  Int numRpsCurrTempList = 0;

  if (m_eSliceType == I_SLICE)//I帧为帧内预测　不存在参考图像直接返回0
  {
    return 0;
  }
  for(UInt i=0; i < m_pRPS->getNumberOfNegativePictures()+ m_pRPS->getNumberOfPositivePictures() + m_pRPS->getNumberOfLongtermPictures(); i++)
  {
    if(m_pRPS->getUsed(i))
    {
      numRpsCurrTempList++;
    }
  }
  return numRpsCurrTempList;
}

Void TComSlice::initEqualRef()//初始化参考图像列表中　两两参考图像是否相同的二维表　
{
  for (Int iDir = 0; iDir < NUM_REF_PIC_LIST_01; iDir++)
  {
    for (Int iRefIdx1 = 0; iRefIdx1 < MAX_NUM_REF; iRefIdx1++)
    {
      for (Int iRefIdx2 = iRefIdx1; iRefIdx2 < MAX_NUM_REF; iRefIdx2++)
      {
        m_abEqualRef[iDir][iRefIdx1][iRefIdx2] = m_abEqualRef[iDir][iRefIdx2][iRefIdx1] = (iRefIdx1 == iRefIdx2? true : false);
      }
    }
  }
}

Void TComSlice::checkColRefIdx(UInt curSliceIdx, TComPic* pic)//检查给定图像中不同slice的同位图像是不是同一帧图像
{
  Int i;
  TComSlice* curSlice = pic->getSlice(curSliceIdx);
  Int currColRefPOC =  curSlice->getRefPOC( RefPicList(1 - curSlice->getColFromL0Flag()), curSlice->getColRefIdx());//同位块所在的图像
  TComSlice* preSlice;
  Int preColRefPOC;
  for(i=curSliceIdx-1; i>=0; i--)//一帧图像中所有slice的同位图像应该为同一帧图像
  {
    preSlice = pic->getSlice(i);
    if(preSlice->getSliceType() != I_SLICE)
    {
      preColRefPOC  = preSlice->getRefPOC( RefPicList(1 - preSlice->getColFromL0Flag()), preSlice->getColRefIdx());
      if(currColRefPOC != preColRefPOC)
      {
        printf("Collocated_ref_idx shall always be the same for all slices of a coded picture!\n");
        exit(EXIT_FAILURE);
      }
      else
      {
        break;
      }
    }
  }
}

Void TComSlice::checkCRA(const TComReferencePictureSet *pReferencePictureSet, Int& pocCRA, NalUnitType& associatedIRAPType, TComList<TComPic *>& rcListPic)
{
  for(Int i = 0; i < pReferencePictureSet->getNumberOfNegativePictures()+pReferencePictureSet->getNumberOfPositivePictures(); i++)
  {//该slice的short term参考图像
    if(pocCRA < MAX_UINT && getPOC() > pocCRA)//如果当前slice在该CRA之后 则要确保该slice参考的图像也在该CRA之后（因为随机接入点在CRA时无法获得CRA之前的图像）
    {
      assert(getPOC()+pReferencePictureSet->getDeltaPOC(i) >= pocCRA);
    }
  }
  for(Int i = pReferencePictureSet->getNumberOfNegativePictures()+pReferencePictureSet->getNumberOfPositivePictures(); i < pReferencePictureSet->getNumberOfPictures(); i++)
  {//该slice的long term参考图像
    if(pocCRA < MAX_UINT && getPOC() > pocCRA)//如果当前slice在该CRA之后 则要确保该slice参考的图像也在该CRA之后
    {
      if (!pReferencePictureSet->getCheckLTMSBPresent(i))//参考图像的最高有效位倍限制
      {
        assert(xGetLongTermRefPic(rcListPic, pReferencePictureSet->getPOC(i), false)->getPOC() >= pocCRA);
      }
      else
      {
        assert(pReferencePictureSet->getPOC(i) >= pocCRA);
      }
    }
  }
  if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP ) // IDR picture found//如果该slice属于IDR图像
  {
    pocCRA = getPOC();//该slice所属图像即为CRA
    associatedIRAPType = getNalUnitType();//得到具体IRAP类型
  }
  else if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA ) // CRA picture found//如果该slice属于CRA图像
  {
    pocCRA = getPOC();
    associatedIRAPType = getNalUnitType();//得到具体IRAP类型
  }
  else if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
         || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
         || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP ) // BLA picture found//如果该slice属于BLA图像
  {
    pocCRA = getPOC();
    associatedIRAPType = getNalUnitType();//得到具体IRAP类型
  }
}

/** Function for marking the reference pictures when an IDR/CRA/CRANT/BLA/BLANT is encountered.
 * \param pocCRA POC of the CRA/CRANT/BLA/BLANT picture
 * \param bRefreshPending flag indicating if a deferred decoding refresh is pending
 * \param rcListPic reference to the reference picture list
 * This function marks the reference pictures as "unused for reference" in the following conditions.
 * If the nal_unit_type is IDR/BLA/BLANT, all pictures in the reference picture list
 * are marked as "unused for reference"
 *    If the nal_unit_type is BLA/BLANT, set the pocCRA to the temporal reference of the current picture.
 * Otherwise
 *    If the bRefreshPending flag is true (a deferred decoding refresh is pending) and the current
 *    temporal reference is greater than the temporal reference of the latest CRA/CRANT/BLA/BLANT picture (pocCRA),
 *    mark all reference pictures except the latest CRA/CRANT/BLA/BLANT picture as "unused for reference" and set
 *    the bRefreshPending flag to false.
 *    If the nal_unit_type is CRA/CRANT, set the bRefreshPending flag to true and pocCRA to the temporal
 *    reference of the current picture.
 * Note that the current picture is already placed in the reference list and its marking is not changed.
 * If the current picture has a nal_ref_idc that is not 0, it will remain marked as "used for reference".
 */
Void TComSlice::decodingRefreshMarking(Int& pocCRA, Bool& bRefreshPending, TComList<TComPic*>& rcListPic, const bool bEfficientFieldIRAPEnabled)
{
  TComPic* rpcPic;
  Int      pocCurr = getPOC();

  if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
    || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
    || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
    || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
    || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP )  // IDR or BLA picture //当前帧为IDR或BLA
  {
    // mark all pictures as not used for reference
    TComList<TComPic*>::iterator        iterPic       = rcListPic.begin();
    while (iterPic != rcListPic.end())//除当前帧之前解码的所有帧不用作参考帧 因为当前IDR帧之后的帧一定不会继续参考前面这些帧
    {
      rpcPic = *(iterPic);
      rpcPic->setCurrSliceIdx(0);
      if (rpcPic->getPOC() != pocCurr)
      {
        rpcPic->getSlice(0)->setReferenced(false);
      }
      iterPic++;
    }
    if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP )//当前帧为BLA
    {
      pocCRA = pocCurr;
    }
    if (bEfficientFieldIRAPEnabled)
    {
      bRefreshPending = true;//表示需要marking pending 将关键帧之前的帧去掉 以前的帧不用作参考帧
    }
  }
  else // CRA or No DR
  {
    if(bEfficientFieldIRAPEnabled && (getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_IDR_N_LP || getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL))//当前帧相近的帧为IDR
    {
      if (bRefreshPending==true && pocCurr > m_iLastIDR) // IDR reference marking pending 
      {//当前帧在IDR之后并且bRefreshPending为true 则IDR之前的帧不作参考图像
        TComList<TComPic*>::iterator iterPic = rcListPic.begin();
        while (iterPic != rcListPic.end())
        {
          rpcPic = *(iterPic);
          if (rpcPic->getPOC() != pocCurr && rpcPic->getPOC() != m_iLastIDR)
          {
            rpcPic->getSlice(0)->setReferenced(false);
          }
          iterPic++;
        }
        bRefreshPending = false;//bRefreshPending为false表示marking pending已完成 无IDR前置参考帧
      }
    }
    else
    {
      if (bRefreshPending==true && pocCurr > pocCRA) // CRA reference marking pending////当前帧相近的帧为CRA
      {
        TComList<TComPic*>::iterator iterPic = rcListPic.begin();
        while (iterPic != rcListPic.end())
        {
          rpcPic = *(iterPic);
          if (rpcPic->getPOC() != pocCurr && rpcPic->getPOC() != pocCRA)
          {
            rpcPic->getSlice(0)->setReferenced(false);
          }
          iterPic++;
        }
        bRefreshPending = false;//bRefreshPending为false表示marking pending已完成 无CRA前置参考帧
      }
    }
    if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA ) // CRA picture found 更新CRA 需再次RefreshPending
    {
      bRefreshPending = true;//
      pocCRA = pocCurr;
    }
  }
}

Void TComSlice::copySliceInfo(TComSlice *pSrc)//将给定的slice信息复制到当前slice
{
  assert( pSrc != NULL );

  Int i, j, k;

  m_iPOC                 = pSrc->m_iPOC;//slice所在图像的POC值
  m_eNalUnitType         = pSrc->m_eNalUnitType;//slice所在图像的NalUnitType(IRAP CRA BLA等)
  m_eSliceType           = pSrc->m_eSliceType;//slice类型(I B P帧)
  m_iSliceQp             = pSrc->m_iSliceQp;//sliceQP值
#if ADAPTIVE_QP_SELECTION
  m_iSliceQpBase         = pSrc->m_iSliceQpBase;
#endif
  m_ChromaQpAdjEnabled = pSrc->m_ChromaQpAdjEnabled;//是否调整色度分量QP值
  m_deblockingFilterDisable   = pSrc->m_deblockingFilterDisable;//是否关闭去方块滤波
  m_deblockingFilterOverrideFlag = pSrc->m_deblockingFilterOverrideFlag;//在slice的头部是否出现去方块滤波参数 
  m_deblockingFilterBetaOffsetDiv2 = pSrc->m_deblockingFilterBetaOffsetDiv2;//slice的去方块参数beta/2的补偿值
  m_deblockingFilterTcOffsetDiv2 = pSrc->m_deblockingFilterTcOffsetDiv2;//slice的去方块参数tc/2的补偿值

  for (i = 0; i < NUM_REF_PIC_LIST_01; i++)//slice的参考图像数
  {
    m_aiNumRefIdx[i]     = pSrc->m_aiNumRefIdx[i];
  }

  for (i = 0; i < MAX_NUM_REF; i++)//slice参考图像list1到list0的映射表（即给定参考图像在list1中的索引可以得到该图像在llist0中的索引）
  {
    m_list1IdxToList0Idx[i] = pSrc->m_list1IdxToList0Idx[i];
  }

  m_bCheckLDC             = pSrc->m_bCheckLDC;
  m_iSliceQpDelta        = pSrc->m_iSliceQpDelta;//slice的Delta QP值 计算最终的QP
  for (UInt component = 0; component < MAX_NUM_COMPONENT; component++)
  {
    m_iSliceChromaQpDelta[component] = pSrc->m_iSliceChromaQpDelta[component];//slice的色度分量Delta QP值 计算色度分量最终的QP
  }
  for (i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    for (j = 0; j < MAX_NUM_REF; j++)
    {
      m_apcRefPicList[i][j]  = pSrc->m_apcRefPicList[i][j];//slice的参考图像
      m_aiRefPOCList[i][j]   = pSrc->m_aiRefPOCList[i][j];//slice参考图像的POC值
      m_bIsUsedAsLongTerm[i][j] = pSrc->m_bIsUsedAsLongTerm[i][j];//slice的参考图像是否为LongTerm
    }
    m_bIsUsedAsLongTerm[i][MAX_NUM_REF] = pSrc->m_bIsUsedAsLongTerm[i][MAX_NUM_REF];
  }
  m_iDepth               = pSrc->m_iDepth;

  // referenced slice
  m_bRefenced            = pSrc->m_bRefenced;//slice是否被用作参考

  // access channel
  m_pRPS                = pSrc->m_pRPS;//slice的参考图像集
  m_iLastIDR             = pSrc->m_iLastIDR;//slice最近的IDR的POC值

  m_pcPic                = pSrc->m_pcPic;//slice所在的图像

  m_colFromL0Flag        = pSrc->m_colFromL0Flag;//slice的同位图像是否在list0中
  m_colRefIdx            = pSrc->m_colRefIdx;//slice的同位图像在列表中的索引

  setLambdas(pSrc->getLambdas());

  for (i = 0; i < NUM_REF_PIC_LIST_01; i++)//两参考图像列表中相同图像的映射表
  {
    for (j = 0; j < MAX_NUM_REF; j++)
    {
      for (k =0; k < MAX_NUM_REF; k++)
      {
        m_abEqualRef[i][j][k] = pSrc->m_abEqualRef[i][j][k];
      }
    }
  }

  m_uiTLayer                      = pSrc->m_uiTLayer;//slice所在的时域层
  m_bTLayerSwitchingFlag          = pSrc->m_bTLayerSwitchingFlag;//slice是否可以进行时域层切换

  m_sliceMode                     = pSrc->m_sliceMode;//slice的约束条件（CTU?byte?tiles?）
  m_sliceArgument                 = pSrc->m_sliceArgument;
  m_sliceCurStartCtuTsAddr        = pSrc->m_sliceCurStartCtuTsAddr;//该slice起始CTu在图像中的位置（Tile-scan ）
  m_sliceCurEndCtuTsAddr          = pSrc->m_sliceCurEndCtuTsAddr;//该slice最后一个CTu在图像中的位置（Tile-scan ）
  m_sliceIdx                      = pSrc->m_sliceIdx;//slice在图像中的索引
  m_sliceSegmentMode              = pSrc->m_sliceSegmentMode;
  m_sliceSegmentArgument          = pSrc->m_sliceSegmentArgument;
  m_sliceSegmentCurStartCtuTsAddr = pSrc->m_sliceSegmentCurStartCtuTsAddr;
  m_sliceSegmentCurEndCtuTsAddr   = pSrc->m_sliceSegmentCurEndCtuTsAddr;
  m_nextSlice                     = pSrc->m_nextSlice;//是否有下一个SLice
  m_nextSliceSegment              = pSrc->m_nextSliceSegment;

  for ( UInt e=0 ; e<NUM_REF_PIC_LIST_01 ; e++ )//参考图像列表中的图像用作参考时的预测权重表
  {
    for ( UInt n=0 ; n<MAX_NUM_REF ; n++ )
    {
      memcpy(m_weightPredTable[e][n], pSrc->m_weightPredTable[e][n], sizeof(WPScalingParam)*MAX_NUM_COMPONENT );
    }
  }

  for( UInt ch = 0 ; ch < MAX_NUM_CHANNEL_TYPE; ch++)//各通道是否使用样点自适应补偿(SAO)
  {
    m_saoEnabledFlag[ch] = pSrc->m_saoEnabledFlag[ch];
  }

  m_cabacInitFlag                 = pSrc->m_cabacInitFlag;//该slice cabac是否初始化

  m_bLMvdL1Zero                   = pSrc->m_bLMvdL1Zero;//该slice在list1中的Mvd是否为0?
  m_LFCrossSliceBoundaryFlag      = pSrc->m_LFCrossSliceBoundaryFlag;//滤波是否允许跨过slice边界
  m_enableTMVPFlag                = pSrc->m_enableTMVPFlag;//是否使用时域MV预测
  m_maxNumMergeCand               = pSrc->m_maxNumMergeCand;//该slice merge最大候选矢量数
  m_encCABACTableIdx              = pSrc->m_encCABACTableIdx;//编码该slice时所用CABAC表的索引
}


/** Function for setting the slice's temporal layer ID and corresponding temporal_layer_switching_point_flag.
 * \param uiTLayer Temporal layer ID of the current slice
 * The decoder calls this function to set temporal_layer_switching_point_flag for each temporal layer based on
 * the SPS's temporal_id_nesting_flag and the parsed PPS.  Then, current slice's temporal layer ID and
 * temporal_layer_switching_point_flag is set accordingly.
 */
Void TComSlice::setTLayerInfo( UInt uiTLayer )
{
  m_uiTLayer = uiTLayer;
}

/** Function for checking if this is a switching-point
*/
Bool TComSlice::isTemporalLayerSwitchingPoint(TComList<TComPic*>& rcListPic)//详见相关资料TSA的定义 switching-point意味着在该处可以安全的切换至其他时域层 
{                                                                           //因为解码顺序在该图像之后的图像不依赖之前高时域层未解码图像
  TComPic* rpcPic;
  // loop through all pictures in the reference picture buffer
  TComList<TComPic*>::iterator iterPic = rcListPic.begin();
  while ( iterPic != rcListPic.end())//参考图列表中所有在当前图像之前的被用作参考的图像的时域层都小于当前slice所在图像的时域层 则说明该图像为时域层切换点
  {
    rpcPic = *(iterPic++);
    if(rpcPic->getSlice(0)->isReferenced() && rpcPic->getPOC() != getPOC())
    {
      if(rpcPic->getTLayer() >= getTLayer())//大于或等于TSA所在时域层之前的图像未解码
      {
        return false;//否则 不为时域切换点
      }
    }
  }
  return true;
}

/** Function for checking if this is a STSA candidate
 */
//STSA和TSA的区别在于TSA可切换至任意高的时域层 而STSA只能切换至和STSA一样的时域层
Bool TComSlice::isStepwiseTemporalLayerSwitchingPointCandidate(TComList<TComPic*>& rcListPic)//详见相关资料STSA的定义
{
  TComPic* rpcPic;

  TComList<TComPic*>::iterator iterPic = rcListPic.begin();
  while ( iterPic != rcListPic.end())
  {
    rpcPic = *(iterPic++);
    if(rpcPic->getSlice(0)->isReferenced() &&  (rpcPic->getUsedByCurr()==true) && rpcPic->getPOC() != getPOC())//只需要对被当前帧参考的图像做参考的图像判断 因此TSA一定时STSA
    {
      if(rpcPic->getTLayer() >= getTLayer())//大于或等于TSA所在时域层之前的图像未解码
      {
        return false;
      }
    }
  }
  return true;
}


Void TComSlice::checkLeadingPictureRestrictions(TComList<TComPic*>& rcListPic)//检测当前slice所在图像是否满足LeadingPicture(LP)的限制 LP指输出顺序在邻近IRAP之前 解码顺序在邻近IRAP之后（详见LeadingPicture相关资料）
{                                                                             //该限制的目的一是为了保证各种NAUL类型定义的正确性 二是在random access时消除不平坦的输出 确保输出时没有因缺失参考帧而无法解码的帧
  TComPic* rpcPic;

  Int nalUnitType = this->getNalUnitType();//当前slice所属图像的NALU类型

  // When a picture is a leading picture, it shall be a RADL or RASL picture.
  if(this->getAssociatedIRAPPOC() > this->getPOC())//该slice所属图像的POC值小于其邻近的IRAP（输出顺序在IRAP之前） 说明为leading picture
  {
    // Do not check IRAP pictures since they may get a POC lower than their associated IRAP
    if(nalUnitType < NAL_UNIT_CODED_SLICE_BLA_W_LP ||  //如果图像NAUL类型不为IRAP 则图像NAUL类型一定为RASL或RADL
       nalUnitType > NAL_UNIT_RESERVED_IRAP_VCL23)
    {
      assert(nalUnitType == NAL_UNIT_CODED_SLICE_RASL_N ||
             nalUnitType == NAL_UNIT_CODED_SLICE_RASL_R ||
             nalUnitType == NAL_UNIT_CODED_SLICE_RADL_N ||
             nalUnitType == NAL_UNIT_CODED_SLICE_RADL_R);
    }
  }

  // When a picture is a trailing picture, it shall not be a RADL or RASL picture.
  if(this->getAssociatedIRAPPOC() < this->getPOC())//该slice所属图像的POC值大于其邻近的IRAP（输出顺序在IRAP之后） 说明为trailing picture
  {
    assert(nalUnitType != NAL_UNIT_CODED_SLICE_RASL_N &&
           nalUnitType != NAL_UNIT_CODED_SLICE_RASL_R &&
           nalUnitType != NAL_UNIT_CODED_SLICE_RADL_N &&
           nalUnitType != NAL_UNIT_CODED_SLICE_RADL_R);
  }//trailing picture图像一定不为RASL或RADL

  // No RASL pictures shall be present in the bitstream that are associated
  // with a BLA picture having nal_unit_type equal to BLA_W_RADL or BLA_N_LP.
  if(nalUnitType == NAL_UNIT_CODED_SLICE_RASL_N ||   //如果该slice所属图像NAUL类型为RASL
     nalUnitType == NAL_UNIT_CODED_SLICE_RASL_R)
  {
    assert(this->getAssociatedIRAPType() != NAL_UNIT_CODED_SLICE_BLA_W_RADL &&
           this->getAssociatedIRAPType() != NAL_UNIT_CODED_SLICE_BLA_N_LP);
  }//则其邻近的IRAP一定不为BLA_W_RADL（May have RADL leading）和BLA_N_LP（Without leading pictures）

  // No RASL pictures shall be present in the bitstream that are associated with
  // an IDR picture.
  if(nalUnitType == NAL_UNIT_CODED_SLICE_RASL_N ||   //如果该slice所属图像NAUL类型为RASL
     nalUnitType == NAL_UNIT_CODED_SLICE_RASL_R)
  {
    assert(this->getAssociatedIRAPType() != NAL_UNIT_CODED_SLICE_IDR_N_LP   &&
           this->getAssociatedIRAPType() != NAL_UNIT_CODED_SLICE_IDR_W_RADL);
  }//则其邻近的IRAP一定不为IDR

  // No RADL pictures shall be present in the bitstream that are associated with
  // a BLA picture having nal_unit_type equal to BLA_N_LP or that are associated
  // with an IDR picture having nal_unit_type equal to IDR_N_LP.
  if(nalUnitType == NAL_UNIT_CODED_SLICE_RADL_N ||  //如果该slice所属图像NAUL类型为RADL
     nalUnitType == NAL_UNIT_CODED_SLICE_RADL_R)
  {
    assert(this->getAssociatedIRAPType() != NAL_UNIT_CODED_SLICE_BLA_N_LP   &&
           this->getAssociatedIRAPType() != NAL_UNIT_CODED_SLICE_IDR_N_LP);
  }//则其邻近的IRAP一定不为IDR_N_LP和BLA_N_LP（Without leading pictures）

  // loop through all pictures in the reference picture buffer
  TComList<TComPic*>::iterator iterPic = rcListPic.begin();
  while ( iterPic != rcListPic.end())//遍历参考图像缓存中的所有图像
  {
    rpcPic = *(iterPic++);
    if(!rpcPic->getReconMark())//该图像未被重建 则直接检测下一帧图像
    {
      continue;
    }
    if (rpcPic->getPOC() == this->getPOC())//该图像为当前图像 则直接检测下一帧图像
    {
      continue;
    }

    // Any picture that has PicOutputFlag equal to 1 that precedes an IRAP picture
    // in decoding order shall precede the IRAP picture in output order.
    // (Note that any picture following in output order would be present in the DPB)
    if(rpcPic->getSlice(0)->getPicOutputFlag() == 1 && !this->getNoOutputPriorPicsFlag())//该帧图像需要被输出且解码顺序在当前IRAP之前
    {
      if(nalUnitType == NAL_UNIT_CODED_SLICE_BLA_N_LP    ||
         nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_LP    ||
         nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_RADL  ||
         nalUnitType == NAL_UNIT_CODED_SLICE_CRA         ||
         nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP    ||
         nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL)
      {
        assert(rpcPic->getPOC() < this->getPOC());//则输出顺序应该在当前IRAP之前(若该帧图像输出顺序在IRAP之后 而解码在其之前 那么当前random access在该IRAP时 当前图像无法顺利解码 却要输出 则会导致不平坦的输出)
      }
    }

    // Any picture that has PicOutputFlag equal to 1 that precedes an IRAP picture
    // in decoding order shall precede any RADL picture associated with the IRAP
    // picture in output order.
    if(rpcPic->getSlice(0)->getPicOutputFlag() == 1)//该帧图像需要输出
    {
      if((nalUnitType == NAL_UNIT_CODED_SLICE_RADL_N ||  //当前图像为RADL
          nalUnitType == NAL_UNIT_CODED_SLICE_RADL_R))
      {
        // rpcPic precedes the IRAP in decoding order
        if(this->getAssociatedIRAPPOC() > rpcPic->getSlice(0)->getAssociatedIRAPPOC())//该帧图像解码顺序在邻近的IRAP之前
        {
          // rpcPic must not be the IRAP picture
          if(this->getAssociatedIRAPPOC() != rpcPic->getPOC())
          {
            assert(rpcPic->getPOC() < this->getPOC());//则要保证该帧图像输出顺序在当前RADL图像之前 （同样是为了避免不平坦的输出）
          }
        }
      }
    }

    // When a picture is a leading picture, it shall precede, in decoding order,
    // all trailing pictures that are associated with the same IRAP picture.
      if(nalUnitType == NAL_UNIT_CODED_SLICE_RASL_N ||       //当前图像为LP
         nalUnitType == NAL_UNIT_CODED_SLICE_RASL_R ||
         nalUnitType == NAL_UNIT_CODED_SLICE_RADL_N ||
         nalUnitType == NAL_UNIT_CODED_SLICE_RADL_R)
      {
        if(rpcPic->getSlice(0)->getAssociatedIRAPPOC() == this->getAssociatedIRAPPOC())
        {
          // rpcPic is a picture that preceded the leading in decoding order since it exist in the DPB
          // rpcPic would violate the constraint if it was a trailing picture
          assert(rpcPic->getPOC() <= this->getAssociatedIRAPPOC());//则该帧图像一定不为trailing picture
        }
      }

    // Any RASL picture associated with a CRA or BLA picture shall precede any
    // RADL picture associated with the CRA or BLA picture in output order
    if(nalUnitType == NAL_UNIT_CODED_SLICE_RASL_N || //当前图像为RASL
       nalUnitType == NAL_UNIT_CODED_SLICE_RASL_R)
    {
      if((this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_BLA_N_LP   ||
          this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_BLA_W_LP   ||
          this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL ||
          this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_CRA)       &&
          this->getAssociatedIRAPPOC() == rpcPic->getSlice(0)->getAssociatedIRAPPOC())//当前图像与该帧图像邻近的IRAP图像相同 且该IRAP为CRA or BLA
      {
        if(rpcPic->getSlice(0)->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_N ||  //该帧图像为RADL
           rpcPic->getSlice(0)->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_R)
        {
          assert(rpcPic->getPOC() > this->getPOC());//这该帧图像（RADL）输出顺序一定在当前图像（RASL）之后
        }
      }
    }

    // Any RASL picture associated with a CRA picture shall follow, in output
    // order, any IRAP picture that precedes the CRA picture in decoding order.
    if(nalUnitType == NAL_UNIT_CODED_SLICE_RASL_N || //当前图像为RASL
       nalUnitType == NAL_UNIT_CODED_SLICE_RASL_R)
    {
      if(this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_CRA)//邻近IRAP图像为CRA
      {
        if(rpcPic->getSlice(0)->getPOC() < this->getAssociatedIRAPPOC() &&            //该帧图像为当前图像邻近CRA图像的前一帧IRAP图像
           (rpcPic->getSlice(0)->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP   ||
            rpcPic->getSlice(0)->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP   ||
            rpcPic->getSlice(0)->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL ||
            rpcPic->getSlice(0)->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP   ||
            rpcPic->getSlice(0)->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
            rpcPic->getSlice(0)->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA))
        {
          assert(this->getPOC() > rpcPic->getSlice(0)->getPOC());//任何RASL输出顺序应该在 解码顺序在其邻近的CRA之前的IRAP之后
        }
      }
    }
  }
}



/** Function for applying picture marking based on the Reference Picture Set in pReferencePictureSet.
*/
Void TComSlice::applyReferencePictureSet( TComList<TComPic*>& rcListPic, const TComReferencePictureSet *pReferencePictureSet)//applying picture marking 根据RPS标记参考图像缓存中的图像
{
  TComPic* rpcPic;
  Int i, isReference;

  checkLeadingPictureRestrictions(rcListPic);//检查参考图像缓存中的图像是否都满足不同类型图像之间的约束

  // loop through all pictures in the reference picture buffer
  TComList<TComPic*>::iterator iterPic = rcListPic.begin();
  while ( iterPic != rcListPic.end())//遍历参考图像缓存中的所有图像
  {
    rpcPic = *(iterPic++);

    if(!rpcPic->getSlice( 0 )->isReferenced())//如果该帧图像不被用作参考 则直接执行下一帧图像
    {
      continue;
    }

    isReference = 0;//初始化该帧图像不会被用作参考
    // loop through all pictures in the Reference Picture Set
    // to see if the picture should be kept as reference picture
    for(i=0;i<pReferencePictureSet->getNumberOfPositivePictures()+pReferencePictureSet->getNumberOfNegativePictures();i++)//遍历RPS中的short term参考图像
    {
      if(!rpcPic->getIsLongTerm() && rpcPic->getPicSym()->getSlice(0)->getPOC() == this->getPOC() + pReferencePictureSet->getDeltaPOC(i))//参考图像缓存中该帧图像不为long term且存在于当前图像的RPS中
      {
        isReference = 1;//则该帧图像用作参考图像
        rpcPic->setUsedByCurr(pReferencePictureSet->getUsed(i));//该帧图像是否为被当前图像用作参考
        rpcPic->setIsLongTerm(0);//不为long term 参考图像
      }
    }
    for(;i<pReferencePictureSet->getNumberOfPictures();i++)//遍历RPS中的long term参考图像
    {
      if(pReferencePictureSet->getCheckLTMSBPresent(i)==true)// 根据m_bCheckLTMSB来决定long term 参考图像的POC值最高有效位的处理
      {
        if(rpcPic->getIsLongTerm() && (rpcPic->getPicSym()->getSlice(0)->getPOC()) == pReferencePictureSet->getPOC(i))//参考图像缓存中该帧图像为long term且存在于当前图像的RPS中
        {
          isReference = 1;//则该帧图像用作参考图像
          rpcPic->setUsedByCurr(pReferencePictureSet->getUsed(i));//该帧图像是否被当前图像用作参考
        }
      }
      else//同上 
      {
        Int pocCycle = 1<<rpcPic->getPicSym()->getSlice(0)->getSPS()->getBitsForPOC();
        Int curPoc = rpcPic->getPicSym()->getSlice(0)->getPOC() & (pocCycle-1);
        Int refPoc = pReferencePictureSet->getPOC(i) & (pocCycle-1);
        if(rpcPic->getIsLongTerm() && curPoc == refPoc)
        {
          isReference = 1;
          rpcPic->setUsedByCurr(pReferencePictureSet->getUsed(i));
        }
      }

    }
    // mark the picture as "unused for reference" if it is not in
    // the Reference Picture Set
    if(rpcPic->getPicSym()->getSlice(0)->getPOC() != this->getPOC() && isReference == 0)//该帧图像不被用作参考
    {
      rpcPic->getSlice( 0 )->setReferenced( false );//该帧图像不为参考图像
      rpcPic->setUsedByCurr(0);//该帧图像不被当前图像用作参考
      rpcPic->setIsLongTerm(0);//不为参考图像自然也不是long term参考图像
    }
    //check that pictures of higher temporal layers are not used
    assert(rpcPic->getSlice( 0 )->isReferenced()==0||rpcPic->getUsedByCurr()==0||rpcPic->getTLayer()<=this->getTLayer());//低时域层图像不能参考高时域层图像
    //check that pictures of higher or equal temporal layer are not in the RPS if the current picture is a TSA picture
    if(this->getNalUnitType() == NAL_UNIT_CODED_SLICE_TSA_R || this->getNalUnitType() == NAL_UNIT_CODED_SLICE_TSA_N)
    {
      assert(rpcPic->getSlice( 0 )->isReferenced()==0||rpcPic->getTLayer()<this->getTLayer());//TSA图像的RPS中没有时域层高于或等于TSA所在时域层的图像（TSA的定义）
    }
    //check that pictures marked as temporal layer non-reference pictures are not used for reference
    if(rpcPic->getPicSym()->getSlice(0)->getPOC() != this->getPOC() && rpcPic->getTLayer()==this->getTLayer())//参考图像缓存中与当前图像相同时域层但没用作参考的图像
    {
      assert(rpcPic->getSlice( 0 )->isReferenced()==0||rpcPic->getUsedByCurr()==0||rpcPic->getSlice( 0 )->getTemporalLayerNonReferenceFlag()==false);
    }
  }
}

/** Function for applying picture marking based on the Reference Picture Set in pReferencePictureSet.
*/
//pReferencePictureSet为当前slice的PRS
Int TComSlice::checkThatAllRefPicsAreAvailable( TComList<TComPic*>& rcListPic, const TComReferencePictureSet *pReferencePictureSet, Bool printErrors, Int pocRandomAccess, Bool bUseRecoveryPoint)//LastRecoveryPicPOC
{//pocRandomAccess为random access（随机接入）时可以获得的最近的图像的POC值 输出顺序在该图像之前的图像无法获得
  Int atLeastOneUnabledByRecoveryPoint = 0;
  Int atLeastOneFlushedByPreviousIDR = 0;
  TComPic* rpcPic;
  Int i, isAvailable;
  Int atLeastOneLost = 0;
  Int atLeastOneRemoved = 0;
  Int iPocLost = 0;

  // loop through all long-term pictures in the Reference Picture Set
  // to see if the picture should be kept as reference picture
  for(i=pReferencePictureSet->getNumberOfNegativePictures()+pReferencePictureSet->getNumberOfPositivePictures();i<pReferencePictureSet->getNumberOfPictures();i++)//遍历RPS中的long term参考图像
  {
    isAvailable = 0;//初始化参考图像不可获得
    // loop through all pictures in the reference picture buffer
    TComList<TComPic*>::iterator iterPic = rcListPic.begin();
    while ( iterPic != rcListPic.end())//遍历参考图像缓存中的所有图像
    {
      rpcPic = *(iterPic++);
      if(pReferencePictureSet->getCheckLTMSBPresent(i)==true)// 根据m_bCheckLTMSB来决定long term 参考图像的POC值最高有效位的处理
      {
        if(rpcPic->getIsLongTerm() && (rpcPic->getPicSym()->getSlice(0)->getPOC()) == pReferencePictureSet->getPOC(i) && rpcPic->getSlice(0)->isReferenced())//该帧图像为long term 被用作参考且为当前图像RPS中的参考图像
        {
          if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess && this->getPOC() + pReferencePictureSet->getDeltaPOC(i) < pocRandomAccess)//当前图像的该参考图像在当前图像前面的pocRandomAccess的前面
          {                                                                                                                                   //一定要保证当前图像的POC大于pocRandomAccess 否则当前图像就属于另一个RandomAccess后
            isAvailable = 0;//说明参考图像无法获得
          }
          else
          {
            isAvailable = 1;//否则可以获得该参考图像
          }
        }
      }
      else//同上
      {
        Int pocCycle = 1<<rpcPic->getPicSym()->getSlice(0)->getSPS()->getBitsForPOC();
        Int curPoc = rpcPic->getPicSym()->getSlice(0)->getPOC() & (pocCycle-1);
        Int refPoc = pReferencePictureSet->getPOC(i) & (pocCycle-1);
        if(rpcPic->getIsLongTerm() && curPoc == refPoc && rpcPic->getSlice(0)->isReferenced())
        {
          if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess && this->getPOC() + pReferencePictureSet->getDeltaPOC(i) < pocRandomAccess)
          {
            isAvailable = 0;
          }
          else
          {
            isAvailable = 1;
          }
        }
      }
    }
    // if there was no such long-term check the short terms
    if(!isAvailable)//参考图像缓存中的long term参考图像没有PRS中的参考图像 继续在参考图像缓存中的short term中找
    {
      iterPic = rcListPic.begin();
      while ( iterPic != rcListPic.end())
      {
        rpcPic = *(iterPic++);

        Int pocCycle = 1 << rpcPic->getPicSym()->getSlice(0)->getSPS()->getBitsForPOC();
        Int curPoc = rpcPic->getPicSym()->getSlice(0)->getPOC();//当前图像的POC
        Int refPoc = pReferencePictureSet->getPOC(i);//参考图像的POC
        if (!pReferencePictureSet->getCheckLTMSBPresent(i))
        {
          curPoc = curPoc & (pocCycle - 1);
          refPoc = refPoc & (pocCycle - 1);
        }

        if (rpcPic->getSlice(0)->isReferenced() && curPoc == refPoc)//不用保证参考图像缓存中的图像为long term（因为short term可能在当前图像参考后变为long term）
        {
          if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess && this->getPOC() + pReferencePictureSet->getDeltaPOC(i) < pocRandomAccess)//参考图像在pocRandomAccess之前 
          {
            isAvailable = 0;//randomaccess时无法获得
          }
          else
          {
            isAvailable = 1;
            rpcPic->setIsLongTerm(1);//从此将该参考图像标记成long term参考图像
            break;
          }
        }
      }
    }
    // report that a picture is lost if it is in the Reference Picture Set
    // but not available as reference picture
    if(isAvailable == 0)//如果当前图像RPS中参考图像无法获得
    {
      if (this->getPOC() + pReferencePictureSet->getDeltaPOC(i) >= pocRandomAccess)//在RandomAccess时本应该可以获得该参考图像
      {
        if(!pReferencePictureSet->getUsed(i) )//该参考图像不被当前图像用作参考
        {
          if(printErrors)
          {//该参考图像不被当前用作参考而可能在缓存中错误的移除了
            printf("\nLong-term reference picture with POC = %3d seems to have been removed or not correctly decoded.", this->getPOC() + pReferencePictureSet->getDeltaPOC(i));
          }
          atLeastOneRemoved = 1;//设置标志
        }
        else//该参考图像被当前图像用作参考
        {
          if(printErrors)
          {//在缓存中产生错误而丢失
            printf("\nLong-term reference picture with POC = %3d is lost or not correctly decoded!", this->getPOC() + pReferencePictureSet->getDeltaPOC(i));
          }
          atLeastOneLost = 1;//设置标志
          iPocLost=this->getPOC() + pReferencePictureSet->getDeltaPOC(i);//丢失的参考图像的POC值
        }
      }
      else if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess)//因randomaccess时该参考图像在pocrandomaccess之前而无法恢复
      {
        atLeastOneUnabledByRecoveryPoint = 1;//设置标志
      }
      else if(bUseRecoveryPoint && (this->getAssociatedIRAPType()==NAL_UNIT_CODED_SLICE_IDR_N_LP || this->getAssociatedIRAPType()==NAL_UNIT_CODED_SLICE_IDR_W_RADL))//当前图像邻近IRAP为IDR
      {//randomaccess时该参考图像在pocrandomaccess之前 当前图像也在pocrandomaccess之前 该情况下本应可以获得参考图像 但当前图像邻近IRAP为IDR时 IDR为CVS中解码第一帧 无法获得较IDR更之前的帧
        atLeastOneFlushedByPreviousIDR = 1;
      }
    }
  }
  // loop through all short-term pictures in the Reference Picture Set
  // to see if the picture should be kept as reference picture
  for(i=0;i<pReferencePictureSet->getNumberOfNegativePictures()+pReferencePictureSet->getNumberOfPositivePictures();i++)//遍历RPS中的short term参考图像
  {//RPS中short term不用在参考图像缓存中的long term中找 因为参考图像缓存中的long term不可能转换成short term 其他基本同上 不在叙述
    isAvailable = 0;
    // loop through all pictures in the reference picture buffer
    TComList<TComPic*>::iterator iterPic = rcListPic.begin();
    while ( iterPic != rcListPic.end())
    {
      rpcPic = *(iterPic++);

      if(!rpcPic->getIsLongTerm() && rpcPic->getPicSym()->getSlice(0)->getPOC() == this->getPOC() + pReferencePictureSet->getDeltaPOC(i) && rpcPic->getSlice(0)->isReferenced())
      {
        if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess && this->getPOC() + pReferencePictureSet->getDeltaPOC(i) < pocRandomAccess)
        {
          isAvailable = 0;
        }
        else
        {
          isAvailable = 1;
        }
      }
    }
    // report that a picture is lost if it is in the Reference Picture Set
    // but not available as reference picture
    if(isAvailable == 0)
    {
      if (this->getPOC() + pReferencePictureSet->getDeltaPOC(i) >= pocRandomAccess)
      {
        if(!pReferencePictureSet->getUsed(i) )//不用作参考图像 被移除
        {
          if(printErrors)
          {
            printf("\nShort-term reference picture with POC = %3d seems to have been removed or not correctly decoded.", this->getPOC() + pReferencePictureSet->getDeltaPOC(i));
          }
          atLeastOneRemoved = 1;
        }
        else//用作参考图形 但不存在于rcListPic中  说明该帧图像丢失
        {
          if(printErrors)
          {
            printf("\nShort-term reference picture with POC = %3d is lost or not correctly decoded!", this->getPOC() + pReferencePictureSet->getDeltaPOC(i));
          }
          atLeastOneLost = 1;
          iPocLost=this->getPOC() + pReferencePictureSet->getDeltaPOC(i);
        }
      }
      else if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess)
      {
        atLeastOneUnabledByRecoveryPoint = 1;
      }
      else if(bUseRecoveryPoint && (this->getAssociatedIRAPType()==NAL_UNIT_CODED_SLICE_IDR_N_LP || this->getAssociatedIRAPType()==NAL_UNIT_CODED_SLICE_IDR_W_RADL))
      {
        atLeastOneFlushedByPreviousIDR = 1;
      }
    }
  }
  //根据不同的情况返回不同的值
  if(atLeastOneUnabledByRecoveryPoint || atLeastOneFlushedByPreviousIDR)
  {
    return -1;
  }    
  if(atLeastOneLost)
  {
    return iPocLost+1;
  }
  if(atLeastOneRemoved)
  {
    return -2;
  }
  else
  {
    return 0;
  }
}

/** Function for constructing an explicit Reference Picture Set out of the available pictures in a referenced Reference Picture Set
*/
//创建本地的RPS
Void TComSlice::createExplicitReferencePictureSetFromReference( TComList<TComPic*>& rcListPic, const TComReferencePictureSet *pReferencePictureSet, Bool isRAP, Int pocRandomAccess, Bool bUseRecoveryPoint, const Bool bEfficientFieldIRAPEnabled)
{
  TComPic* rpcPic;
  Int i, j;
  Int k = 0;
  Int nrOfNegativePictures = 0;
  Int nrOfPositivePictures = 0;
  TComReferencePictureSet* pLocalRPS = this->getLocalRPS();//当前slice LocalRPS的地址 LocalRPS指的是在slice header中PRS 而不是通过PRS索引在SPS中对应的RPS
  (*pLocalRPS)=TComReferencePictureSet();//LocalRPS初始化 置成默认值

  Bool irapIsInRPS = false; // Used when bEfficientFieldIRAPEnabled==true

  // loop through all pictures in the Reference Picture Set
  for(i=0;i<pReferencePictureSet->getNumberOfPictures();i++)//遍历给定RPS中的所有图像
  {
    j = 0;
    // loop through all pictures in the reference picture buffer
    TComList<TComPic*>::iterator iterPic = rcListPic.begin();
    while ( iterPic != rcListPic.end())//遍历参考图像缓存中的所有图像
    {
      j++;
      rpcPic = *(iterPic++);

      if(rpcPic->getPicSym()->getSlice(0)->getPOC() == this->getPOC() + pReferencePictureSet->getDeltaPOC(i) && rpcPic->getSlice(0)->isReferenced())
      {//如果参考图像缓存中该图像在RPS中用作参考图像 则将该图像添加到本地RPS中
        // This picture exists as a reference picture
        // and should be added to the explicit Reference Picture Set
        pLocalRPS->setDeltaPOC(k, pReferencePictureSet->getDeltaPOC(i));//给本地RPS中给第K帧参考图像设置参考信息
        pLocalRPS->setUsed(k, pReferencePictureSet->getUsed(i) && (!isRAP));
        if (bEfficientFieldIRAPEnabled)
        {
          pLocalRPS->setUsed(k, pLocalRPS->getUsed(k) && !(bUseRecoveryPoint && this->getPOC() > pocRandomAccess && this->getPOC() + pReferencePictureSet->getDeltaPOC(i) < pocRandomAccess) );
        }//确保该帧参考图像当UseRecoveryPoint时可以获得 才能被标示为被当前图像用作参考

        if(pLocalRPS->getDeltaPOC(k) < 0)//DeltaPOC<0 说明为前置参考图像
        {
          nrOfNegativePictures++;
        }
        else//否则为后置参考图像
        {
          if(bEfficientFieldIRAPEnabled && rpcPic->getPicSym()->getSlice(0)->getPOC() == this->getAssociatedIRAPPOC() && this->getAssociatedIRAPPOC() == this->getPOC()+1)//如果当前图像邻近的IRAP在参考图像缓存中 则邻近IRAP已被添加至本地RPS
          {
            irapIsInRPS = true;//irapIsInRPS标志置为1
          }
          nrOfPositivePictures++;
        }
        k++;
      }
    }
  }

  Bool useNewRPS = false;
  // if current picture is complimentary field associated to IRAP, add the IRAP to its RPS. 
  if(bEfficientFieldIRAPEnabled && m_pcPic->isField() && !irapIsInRPS)//IRAP不在给定的RPS中
  {
    TComList<TComPic*>::iterator iterPic = rcListPic.begin();
    while ( iterPic != rcListPic.end())
    {
      rpcPic = *(iterPic++);
      if(rpcPic->getPicSym()->getSlice(0)->getPOC() == this->getAssociatedIRAPPOC() && this->getAssociatedIRAPPOC() == this->getPOC()+1)//将IRAP添加值本地RPS
      {
        pLocalRPS->setDeltaPOC(k, 1);
        pLocalRPS->setUsed(k, true);
        nrOfPositivePictures++;
        k ++;
        useNewRPS = true;
      }
    }
  }
  pLocalRPS->setNumberOfNegativePictures(nrOfNegativePictures);//设置本地RPS前置参考图像数
  pLocalRPS->setNumberOfPositivePictures(nrOfPositivePictures);//设置本地RPS后置参考图像数
  pLocalRPS->setNumberOfPictures(nrOfNegativePictures+nrOfPositivePictures);//设置本地RPS总参考图像数
  // This is a simplistic inter rps example. A smarter encoder will look for a better reference RPS to do the
  // inter RPS prediction with.  Here we just use the reference used by pReferencePictureSet.
  // If pReferencePictureSet is not inter_RPS_predicted, then inter_RPS_prediction is for the current RPS also disabled.
  if (!pReferencePictureSet->getInterRPSPrediction() || useNewRPS )//如果给定的RPS不是由参考RPS预测得到 
  {
    pLocalRPS->setInterRPSPrediction(false);//那么本地PRS也无法由预测RPS得到（因为本地PRS是由给定的PRS得到）
    pLocalRPS->setNumRefIdc(0);
  }
  else//如果给定的RPS是由参考RPS预测得到 那么本地PRS也由预测RPS得到 （该处应事先理解好PRS预测等内容）
  {
    Int rIdx =  this->getRPSidx() - pReferencePictureSet->getDeltaRIdxMinus1() - 1;//当前slice所属图像的RPS所参考的RPS在SPS中的索引
    Int deltaRPS = pReferencePictureSet->getDeltaRPS();//给定的RPS与其参考的PRS的POC差值
    const TComReferencePictureSet* pcRefRPS = this->getSPS()->getRPSList()->getReferencePictureSet(rIdx);//当前slice所属图像的RPS所参考的RPS
    Int iRefPics = pcRefRPS->getNumberOfPictures();//用作参考的PRS的参考图像数
    Int iNewIdc=0;
    for(i=0; i<= iRefPics; i++)//预测得到的PRS的num_ref_idcs为用做参考的PRS的um_ref_idcs+1 +1即为用做参考的PRS所属的图像
    {
      Int deltaPOC = ((i != iRefPics)? pcRefRPS->getDeltaPOC(i) : 0);//当i == iRefPics时 参考图像即为用做参考的PRS所属的图像本身 故deltaPOC为0 // check if the reference abs POC is >= 0
      Int iRefIdc = 0;//初始化赋值为0 说明参考PRS中参考图像不再被用作参考图像
      for (j=0; j < pLocalRPS->getNumberOfPictures(); j++) // loop through the  pictures in the new RPS
      {
        if ( (deltaPOC + deltaRPS) == pLocalRPS->getDeltaPOC(j))//如果参考PRS中的参考图像也为本地PRS中的参考图像
        {
          if (pLocalRPS->getUsed(j))//若该参考图像被当前图像用作参考
          {
            iRefIdc = 1;
          }
          else//未被当前图像用作参考（以后被用作参考图像）
          {
            iRefIdc = 2;
          }
        }
      }
      pLocalRPS->setRefIdc(i, iRefIdc);//给对应帧设置iRefIdc
      iNewIdc++;
    }
    pLocalRPS->setInterRPSPrediction(true);//本地RPS为参考RPS预测得到
    pLocalRPS->setNumRefIdc(iNewIdc);//设置NumRefIdc
    pLocalRPS->setDeltaRPS(deltaRPS);//设置tDeltaRPS
    pLocalRPS->setDeltaRIdxMinus1(pReferencePictureSet->getDeltaRIdxMinus1() + this->getSPS()->getRPSList()->getNumberOfReferencePictureSets() - this->getRPSidx());//当前PRS索引减去参考PRS索引值减1 (PRS索引可以认为为图像的解码顺序)
  }

  this->setRPS(pLocalRPS);//设置当前slice的RPS为本地PRS
  this->setRPSidx(-1);//-1表示使用本地RPS 而不是通过索引使用SPS中的RPS
}

//! get AC and DC values for weighted pred
Void  TComSlice::getWpAcDcParam(WPACDCParam *&wp)//得到用于加权预测的权重值
{
  wp = m_weightACDCParam;
}

//! init AC and DC values for weighted pred
Void  TComSlice::initWpAcDcParam()//初始化各个分量用于加权预测的权重值
{
  for(Int iComp = 0; iComp < MAX_NUM_COMPONENT; iComp++ )
  {
    m_weightACDCParam[iComp].iAC = 0;
    m_weightACDCParam[iComp].iDC = 0;
  }
}

//! get tables for weighted prediction
Void  TComSlice::getWpScaling( RefPicList e, Int iRefIdx, WPScalingParam *&wp )//得到参考图像列表中对应参考图像加权预测的权重信息
{
  assert (e<NUM_REF_PIC_LIST_01);
  wp = m_weightPredTable[e][iRefIdx];
}

//! reset Default WP tables settings : no weight.
Void  TComSlice::resetWpScaling()//将所有参考图像各个分量的权重信息重置为默认值
{
  for ( Int e=0 ; e<NUM_REF_PIC_LIST_01 ; e++ )
  {
    for ( Int i=0 ; i<MAX_NUM_REF ; i++ )
    {
      for ( Int yuv=0 ; yuv<MAX_NUM_COMPONENT ; yuv++ )
      {
        WPScalingParam  *pwp = &(m_weightPredTable[e][i][yuv]);
        pwp->bPresentFlag      = false;
        pwp->uiLog2WeightDenom = 0;
        pwp->uiLog2WeightDenom = 0;
        pwp->iWeight           = 1;
        pwp->iOffset           = 0;
      }
    }
  }
}

//! init WP table
Void  TComSlice::initWpScaling(const TComSPS *sps)//初始化加权预测权重表
{
  const Bool bUseHighPrecisionPredictionWeighting = sps->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag();
  for ( Int e=0 ; e<NUM_REF_PIC_LIST_01 ; e++ )
  {
    for ( Int i=0 ; i<MAX_NUM_REF ; i++ )
    {
      for ( Int yuv=0 ; yuv<MAX_NUM_COMPONENT ; yuv++ )
      {
        WPScalingParam  *pwp = &(m_weightPredTable[e][i][yuv]);
        if ( !pwp->bPresentFlag )//pwp不存在 
        {//则iWeight和iOffset推断如下
          // Inferring values not present :
          pwp->iWeight = (1 << pwp->uiLog2WeightDenom);
          pwp->iOffset = 0;
        }

        const Int offsetScalingFactor = bUseHighPrecisionPredictionWeighting ? 1 : (1 << (sps->getBitDepth(toChannelType(ComponentID(yuv)))-8));
       // Weighted prediction scaling values built from above parameters (bitdepth scaled): 用于加权预测时提高计算精度
        pwp->w      = pwp->iWeight;
        pwp->o      = pwp->iOffset * offsetScalingFactor; //NOTE: This value of the ".o" variable is never used - .o is set immediately before it gets used
        pwp->shift  = pwp->uiLog2WeightDenom;
        pwp->round  = (pwp->uiLog2WeightDenom>=1) ? (1 << (pwp->uiLog2WeightDenom-1)) : (0);
      }
    }
  }
}

// ------------------------------------------------------------------------------------------------
// Video parameter set (VPS)
// ------------------------------------------------------------------------------------------------
TComVPS::TComVPS()
: m_VPSId                     (  0)
, m_uiMaxTLayers              (  1)
, m_uiMaxLayers               (  1)
, m_bTemporalIdNestingFlag    (false)
, m_numHrdParameters          (  0)
, m_maxNuhReservedZeroLayerId (  0)
, m_hrdParameters             ()
, m_hrdOpSetIdx               ()
, m_cprmsPresentFlag          ()
{

  for( Int i = 0; i < MAX_TLAYER; i++)
  {
    m_numReorderPics[i] = 0;
    m_uiMaxDecPicBuffering[i] = 1;
    m_uiMaxLatencyIncrease[i] = 0;
  }
}

TComVPS::~TComVPS()
{
}

// ------------------------------------------------------------------------------------------------
// Sequence parameter set (SPS)//该部分涉及许多参数 仅做简单描述 待到使用时 会有更深刻的理解
// ------------------------------------------------------------------------------------------------

TComSPSRExt::TComSPSRExt()
 : m_transformSkipRotationEnabledFlag   (false)
 , m_transformSkipContextEnabledFlag    (false)
// m_rdpcmEnabledFlag initialized below
 , m_extendedPrecisionProcessingFlag    (false)
 , m_intraSmoothingDisabledFlag         (false)
 , m_highPrecisionOffsetsEnabledFlag    (false)
 , m_persistentRiceAdaptationEnabledFlag(false)
 , m_cabacBypassAlignmentEnabledFlag    (false)
{
  for (UInt signallingModeIndex = 0; signallingModeIndex < NUMBER_OF_RDPCM_SIGNALLING_MODES; signallingModeIndex++)
  {
    m_rdpcmEnabledFlag[signallingModeIndex] = false;
  }
}

TComSPS::TComSPS()//SPS参数初始化
: m_SPSId                     (  0)//表示SPS的标示号
, m_VPSId                     (  0)//当前激活的VPS的ID号
, m_chromaFormatIdc           (CHROMA_420)//色度采样格式
, m_uiMaxTLayers              (  1)//最大时域层
// Structure
, m_picWidthInLumaSamples     (352)//图像中亮度样点的宽度
, m_picHeightInLumaSamples    (288)//图像中亮度样点的高度
, m_log2MinCodingBlockSize    (  0)//最小的亮度编码块的大小
, m_log2DiffMaxMinCodingBlockSize(0)//最大的亮度编码块和最小亮度块的的差值
, m_uiMaxCUWidth              ( 32)//最大CU块的宽
, m_uiMaxCUHeight             ( 32)//最大CU块的高
, m_uiMaxTotalCUDepth         (  3)//CTU划分为CU时允许的最大深度
, m_bLongTermRefsPresent      (false)//帧间预测时是否使用long term参考图像
, m_uiQuadtreeTULog2MaxSize   (  0)//TU块允许的最大大小
, m_uiQuadtreeTULog2MinSize   (  0)//TU块允许的最小大小
, m_uiQuadtreeTUMaxDepthInter (  0)//帧间预测时Cu划分为TU允许的最大深度
, m_uiQuadtreeTUMaxDepthIntra (  0)//帧内预测时Cu划分为TU允许的最大深度
// Tool list
, m_usePCM                    (false)//是否使用PCM
, m_pcmLog2MaxSize            (  5)//PCM模式下 编码块的最大尺寸
, m_uiPCMLog2MinSize          (  7)//PCM模式下 编码块的最小尺寸 ？
, m_bPCMFilterDisableFlag     (false)//PCM模式下 是否允许滤波
, m_uiBitsForPOC              (  8)//表示图像POC值时的位数
, m_numLongTermRefPicSPS      (  0)//SPS中ong term参考图像数目
, m_uiMaxTrSize               ( 32)//最大变换块大小
, m_bUseSAO                   (false)//是否使用SAO
, m_bTemporalIdNestingFlag    (false)//和max_sub_layers联合使用 规定是否额外限制CVS的帧间预测
, m_scalingListEnabledFlag    (false)//对变化系数量化过程中是否使用量化矩阵
, m_useStrongIntraSmoothing   (false)//是否允许对帧内参考像素进行强滤波
, m_vuiParametersPresentFlag  (false)//是否含有语法结构体m_vuiParameters()
, m_vuiParameters             ()
{
  for(Int ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
  {
    m_bitDepths.recon[ch] = 8;
#if O0043_BEST_EFFORT_DECODING
    m_bitDepths.stream[ch] = 8;
#endif
    m_pcmBitDepths[ch] = 8;
    m_qpBDOffset   [ch] = 0;
  }

  for ( Int i = 0; i < MAX_TLAYER; i++ )
  {
    m_uiMaxLatencyIncrease[i] = 0;
    m_uiMaxDecPicBuffering[i] = 1;
    m_numReorderPics[i]       = 0;
  }

  ::memset(m_ltRefPicPocLsbSps, 0, sizeof(m_ltRefPicPocLsbSps));
  ::memset(m_usedByCurrPicLtSPSFlag, 0, sizeof(m_usedByCurrPicLtSPSFlag));
}

TComSPS::~TComSPS()
{
  m_RPSList.destroy();
}

Void  TComSPS::createRPSList( Int numRPS )
{
  m_RPSList.destroy();
  m_RPSList.create(numRPS);
}


const Int TComSPS::m_winUnitX[]={1,2,2,1};
const Int TComSPS::m_winUnitY[]={1,2,1,1};

TComPPSRExt::TComPPSRExt()
: m_log2MaxTransformSkipBlockSize      (2)
, m_crossComponentPredictionEnabledFlag(false)
, m_diffCuChromaQpOffsetDepth          (0)
, m_chromaQpOffsetListLen              (0)
// m_ChromaQpAdjTableIncludingNullEntry initialized below
// m_log2SaoOffsetScale initialized below
{
  m_ChromaQpAdjTableIncludingNullEntry[0].u.comp.CbOffset = 0; // Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0. This is initialised here and never subsequently changed.
  m_ChromaQpAdjTableIncludingNullEntry[0].u.comp.CrOffset = 0;
  for(Int ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
  {
    m_log2SaoOffsetScale[ch] = 0;
  }
}

TComPPS::TComPPS()
: m_PPSId                            (0)//当前激活的PPS的ID号
, m_SPSId                            (0)//当前激活的SPS的ID号
, m_picInitQPMinus26                 (0)//规定了每个Slice中亮度分量的量化参数初始值
, m_useDQP                           (false)//是否使用QG
, m_bConstrainedIntraPred            (false)//是否允许使用采用帧间预测模式的邻近块信息进行帧内预测
, m_bSliceChromaQpFlag               (false)//slice中是否存在色度QP
, m_uiMaxCuDQPDepth                  (0)//QG相对CTU的深度
, m_chromaCbQpOffset                 (0)//色度分量Cb采用的QP值相对亮度分量Qp值的偏移量
, m_chromaCrQpOffset                 (0)//色度分量Cr采用的QP值相对亮度分量Qp值的偏移量
, m_numRefIdxL0DefaultActive         (1)//引用list0中参考图像数目的最大默认值
, m_numRefIdxL1DefaultActive         (1)//引用list1中参考图像数目的最大默认值
, m_TransquantBypassEnableFlag       (false)//是否使用TransquantBypass
, m_useTransformSkip                 (false)//是否使用TransformSkip模式
, m_dependentSliceSegmentsEnabledFlag(false)//表示slice头中是否存在dependent_slice_segment_flag 用与当前slice是否为依赖片
, m_tilesEnabledFlag                 (false)//是否使用tile模式
, m_entropyCodingSyncEnabledFlag     (false)//是否使用熵编码同步机制
, m_loopFilterAcrossTilesEnabledFlag (true)//环路滤波是否允许跨越Tiles边界
, m_uniformSpacingFlag               (false)//表示图像中tile的列边界和行边界的分布是否一致
, m_numTileColumnsMinus1             (0)//Tiles列数
, m_numTileRowsMinus1                (0)//Tiles行数
, m_signHideFlag                     (false)//是否使用signHide技术
, m_cabacInitPresentFlag             (false)//是否需要判断CABAC中使用何种方法来确定上下文变量的初始值
, m_sliceHeaderExtensionPresentFlag  (false)//是否允许扩展 供将来使用
, m_loopFilterAcrossSlicesEnabledFlag(false)//环路滤波是否允许跨越slices边界
, m_listsModificationPresentFlag     (0)//是否需要修正参考图像列表
, m_numExtraSliceHeaderBits          (0)//slice header中额外的比特数 留作将来使用
{
}

TComPPS::~TComPPS()
{
}

TComReferencePictureSet::TComReferencePictureSet()//PRS参数 之前已描述过
: m_numberOfPictures (0)
, m_numberOfNegativePictures (0)
, m_numberOfPositivePictures (0)
, m_numberOfLongtermPictures (0)
, m_interRPSPrediction (0)
, m_deltaRIdxMinus1 (0)
, m_deltaRPS (0)
, m_numRefIdc (0)
{
  ::memset( m_deltaPOC, 0, sizeof(m_deltaPOC) );
  ::memset( m_POC, 0, sizeof(m_POC) );
  ::memset( m_used, 0, sizeof(m_used) );
  ::memset( m_refIdc, 0, sizeof(m_refIdc) );
  ::memset( m_bCheckLTMSB, 0, sizeof(m_bCheckLTMSB) );
  ::memset( m_pocLSBLT, 0, sizeof(m_pocLSBLT) );
  ::memset( m_deltaPOCMSBCycleLT, 0, sizeof(m_deltaPOCMSBCycleLT) );
  ::memset( m_deltaPocMSBPresentFlag, 0, sizeof(m_deltaPocMSBPresentFlag) );
}

TComReferencePictureSet::~TComReferencePictureSet()
{
}
/**
 简单的set get方法 所涉及参数已在之前说明过 不在叙述
 */
Void TComReferencePictureSet::setUsed(Int bufferNum, Bool used)
{
  m_used[bufferNum] = used;
}

Void TComReferencePictureSet::setDeltaPOC(Int bufferNum, Int deltaPOC)
{
  m_deltaPOC[bufferNum] = deltaPOC;
}

Void TComReferencePictureSet::setNumberOfPictures(Int numberOfPictures)
{
  m_numberOfPictures = numberOfPictures;
}

Int TComReferencePictureSet::getUsed(Int bufferNum) const
{
  return m_used[bufferNum];
}

Int TComReferencePictureSet::getDeltaPOC(Int bufferNum) const
{
  return m_deltaPOC[bufferNum];
}

Int TComReferencePictureSet::getNumberOfPictures() const
{
  return m_numberOfPictures;
}

Int TComReferencePictureSet::getPOC(Int bufferNum) const
{
  return m_POC[bufferNum];
}

Void TComReferencePictureSet::setPOC(Int bufferNum, Int POC)
{
  m_POC[bufferNum] = POC;
}

Bool TComReferencePictureSet::getCheckLTMSBPresent(Int bufferNum) const
{
  return m_bCheckLTMSB[bufferNum];
}

Void TComReferencePictureSet::setCheckLTMSBPresent(Int bufferNum, Bool b)
{
  m_bCheckLTMSB[bufferNum] = b;
}

//! set the reference idc value at uiBufferNum entry to the value of iRefIdc
Void TComReferencePictureSet::setRefIdc(Int bufferNum, Int refIdc)
{
  m_refIdc[bufferNum] = refIdc;
}

//! get the reference idc value at uiBufferNum
Int  TComReferencePictureSet::getRefIdc(Int bufferNum) const
{
  return m_refIdc[bufferNum];
}

/** Sorts the deltaPOC and Used by current values in the RPS based on the deltaPOC values.
 *  deltaPOC values are sorted with -ve values before the +ve values.  -ve values are in decreasing order.
 *  +ve values are in increasing order.
 * \returns Void
 */
Void TComReferencePictureSet::sortDeltaPOC()//排列DeltaPOC的顺序
{
  // sort in increasing order (smallest first)
  for(Int j=1; j < getNumberOfPictures(); j++)//从小到大排序 每次循环得到前j+1个排列好的数
  {
    Int deltaPOC = getDeltaPOC(j);
    Bool used = getUsed(j);
    for (Int k=j-1; k >= 0; k--)
    {
      Int temp = getDeltaPOC(k);
      if (deltaPOC < temp)//交换二者顺序
      {
        setDeltaPOC(k+1, temp);
        setUsed(k+1, getUsed(k));
        setDeltaPOC(k, deltaPOC);
        setUsed(k, used);
      }
    }
  }
  // flip the negative values to largest first
  Int numNegPics = getNumberOfNegativePictures();
  for(Int j=0, k=numNegPics-1; j < numNegPics>>1; j++, k--)//将负数部分从中心点翻转 将较大的负数置与较小的负数之前
  {
    Int deltaPOC = getDeltaPOC(j);
    Bool used = getUsed(j);
    setDeltaPOC(j, getDeltaPOC(k));
    setUsed(j, getUsed(k));
    setDeltaPOC(k, deltaPOC);
    setUsed(k, used);
  }
}

/** Prints the deltaPOC and RefIdc (if available) values in the RPS.
 *  A "*" is added to the deltaPOC value if it is Used bu current.
 * \returns Void
 */
Void TComReferencePictureSet::printDeltaPOC() const//打印deltaPOC和RefIdc信息 若参考图像被当前图像所参考 其deltaPOC上会加上*
{
  printf("DeltaPOC = { ");
  for(Int j=0; j < getNumberOfPictures(); j++)
  {
    printf("%d%s ", getDeltaPOC(j), (getUsed(j)==1)?"*":"");
  }
  if (getInterRPSPrediction())
  {
    printf("}, RefIdc = { ");
    for(Int j=0; j < getNumRefIdc(); j++)
    {
      printf("%d ", getRefIdc(j));
    }
  }
  printf("}\n");
}

TComRefPicListModification::TComRefPicListModification()
: m_refPicListModificationFlagL0 (false)
, m_refPicListModificationFlagL1 (false)
{
  ::memset( m_RefPicSetIdxL0, 0, sizeof(m_RefPicSetIdxL0) );
  ::memset( m_RefPicSetIdxL1, 0, sizeof(m_RefPicSetIdxL1) );
}

TComRefPicListModification::~TComRefPicListModification()
{
}

TComScalingList::TComScalingList()//量化矩阵构造函数
{
  for(UInt sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(UInt listId = 0; listId < SCALING_LIST_NUM; listId++)
    {//初始化的值为0！
      m_scalingListCoef[sizeId][listId].resize(min<Int>(MAX_MATRIX_COEF_NUM,(Int)g_scalingListSize[sizeId]));//量化矩阵大小不得超过8*8时因为8*8以上的大小可以通过8*8上采用得到
    }
  }
}

/**
 * 此处开始涉及量化矩阵的内容
 * 建议先查阅《T-REC-H》中Scaling list data semantics的内容
 */
/** set default quantization matrix to array
*/
Void TComScalingList::setDefaultScalingList()//将量化矩阵设为默认的量化矩阵
{
  for(UInt sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(UInt listId=0;listId<SCALING_LIST_NUM;listId++)
    {
      processDefaultMatrix(sizeId, listId);
    }
  }
}
/** check if use default quantization matrix
 * \returns true if use default quantization matrix in all size
*/
Bool TComScalingList::checkDefaultScalingList()//检查所有大小的量化矩阵是否都为默认量化矩阵
{
  UInt defaultCounter=0;

  for(UInt sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(UInt listId=0;listId<SCALING_LIST_NUM;listId++)
    {
      if( !memcmp(getScalingListAddress(sizeId,listId), getScalingListDefaultAddress(sizeId, listId),sizeof(Int)*min(MAX_MATRIX_COEF_NUM,(Int)g_scalingListSize[sizeId])) // check value of matrix
     && ((sizeId < SCALING_LIST_16x16) || (getScalingListDC(sizeId,listId) == 16))) // check DC value
      {
        defaultCounter++;//统计使用默认量化矩阵的量化矩阵数
      }
    }
  }

  return (defaultCounter == (SCALING_LIST_NUM * SCALING_LIST_SIZE_NUM )) ? false : true;
}

/** get scaling matrix from RefMatrixID
 * \param sizeId    size index
 * \param listId    index of input matrix
 * \param refListId index of reference matrix
 */
Void TComScalingList::processRefMatrix( UInt sizeId, UInt listId , UInt refListId )//从给定的参考矩阵中得到量化矩阵
{
  ::memcpy(getScalingListAddress(sizeId, listId),((listId == refListId)? getScalingListDefaultAddress(sizeId, refListId): getScalingListAddress(sizeId, refListId)),sizeof(Int)*min(MAX_MATRIX_COEF_NUM,(Int)g_scalingListSize[sizeId]));
}

Void TComScalingList::checkPredMode(UInt sizeId, UInt listId)//判断scaling_list_pred_mode_flag
{//若scaling_list_pred_mode_flag为0 说明当前量化矩阵可以通过之前存在的量化矩阵（参考量化矩阵）得到 若为1 则当前量化矩阵需要通过自定义量化矩阵得到
  Int predListStep = (sizeId == SCALING_LIST_32x32? (SCALING_LIST_NUM/NUMBER_OF_PREDICTION_MODES) : 1); // if 32x32, skip over chroma entries.

  for(Int predListIdx = (Int)listId ; predListIdx >= 0; predListIdx-=predListStep)
  {
    if( !memcmp(getScalingListAddress(sizeId,listId),((listId == predListIdx) ?
      getScalingListDefaultAddress(sizeId, predListIdx): getScalingListAddress(sizeId, predListIdx)),sizeof(Int)*min(MAX_MATRIX_COEF_NUM,(Int)g_scalingListSize[sizeId])) // check value of matrix
     && ((sizeId < SCALING_LIST_16x16) || (getScalingListDC(sizeId,listId) == getScalingListDC(sizeId,predListIdx)))) // check DC value
    {//若存在已有的量化矩阵与当前量化矩阵相同 则可用之前的量化矩阵做参考 得到当前量化矩阵
      setRefMatrixId(sizeId, listId, predListIdx);//设置参考量化矩阵
      setScalingListPredModeFlag(sizeId, listId, false);
      return;
    }
  }
  setScalingListPredModeFlag(sizeId, listId, true);//需通过自定义量化矩阵得到
}

static Void outputScalingListHelp(std::ostream &os)//打印量化矩阵的类型
{
  os << "The scaling list file specifies all matrices and their DC values; none can be missing,\n"
         "but their order is arbitrary.\n\n"
         "The matrices are specified by:\n"
         "<matrix name><unchecked data>\n"
         "  <value>,<value>,<value>,....\n\n"
         "  Line-feeds can be added arbitrarily between values, and the number of values needs to be\n"
         "  at least the number of entries for the matrix (superfluous entries are ignored).\n"
         "  The <unchecked data> is text on the same line as the matrix that is not checked\n"
         "  except to ensure that the matrix name token is unique. It is recommended that it is ' ='\n"
         "  The values in the matrices are the absolute values (0-255), not the delta values as\n"
         "  exchanged between the encoder and decoder\n\n"
         "The DC values (for matrix sizes larger than 8x8) are specified by:\n"
         "<matrix name>_DC<unchecked data>\n"
         "  <value>\n";

  os << "The permitted matrix names are:\n";
  for(UInt sizeIdc = 0; sizeIdc < SCALING_LIST_SIZE_NUM; sizeIdc++)
  {
    for(UInt listIdc = 0; listIdc < SCALING_LIST_NUM; listIdc++)
    {
      if ((sizeIdc!=SCALING_LIST_32x32) || (listIdc%(SCALING_LIST_NUM/NUMBER_OF_PREDICTION_MODES) == 0))
      {
        os << "  " << MatrixType[sizeIdc][listIdc] << '\n';
      }
    }
  }
}

Void TComScalingList::outputScalingLists(std::ostream &os) const //打印不同的化矩阵的量化系数
{
  for(UInt sizeIdc = 0; sizeIdc < SCALING_LIST_SIZE_NUM; sizeIdc++)
  {
    const UInt size = min(8,4<<(sizeIdc));
    for(UInt listIdc = 0; listIdc < SCALING_LIST_NUM; listIdc++)
    {
      if ((sizeIdc!=SCALING_LIST_32x32) || (listIdc%(SCALING_LIST_NUM/NUMBER_OF_PREDICTION_MODES) == 0))
      {
        const Int *src = getScalingListAddress(sizeIdc, listIdc);
        os << (MatrixType[sizeIdc][listIdc]) << " =\n  ";
        for(UInt y=0; y<size; y++)//打印量化矩阵
        {
          for(UInt x=0; x<size; x++, src++)
          {
            os << std::setw(3) << (*src) << ", ";
          }
          os << (y+1<size?"\n  ":"\n");
        }
        if(sizeIdc > SCALING_LIST_8x8)
        {
          os << MatrixType_DC[sizeIdc][listIdc] << " = \n  " << std::setw(3) << getScalingListDC(sizeIdc, listIdc) << "\n";
        }
        os << "\n";
      }
    }
  }
}

Bool TComScalingList::xParseScalingList(Char* pchFile)//从给定的文件中读取量化矩阵
{
  static const Int LINE_SIZE=1024;
  FILE *fp = NULL;
  Char line[LINE_SIZE];

  if (pchFile == NULL)//若为给定文件 输出错误信息
  {
    fprintf(stderr, "Error: no scaling list file specified. Help on scaling lists being output\n");
    outputScalingListHelp(std::cout);
    std::cout << "\n\nExample scaling list file using default values:\n\n";
    outputScalingLists(std::cout);
    exit (1);
    return true;
  }
  else if ((fp = fopen(pchFile,"r")) == (FILE*)NULL)//打开文件为空
  {
    fprintf(stderr, "Error: cannot open scaling list file %s for reading\n",pchFile);
    return true;
  }

  for(UInt sizeIdc = 0; sizeIdc < SCALING_LIST_SIZE_NUM; sizeIdc++)//遍历各种大小的量化矩阵
  {
    const UInt size = min(MAX_MATRIX_COEF_NUM,(Int)g_scalingListSize[sizeIdc]);

    for(UInt listIdc = 0; listIdc < SCALING_LIST_NUM; listIdc++)//遍历各种类型的量化矩阵
    {
      Int * const src = getScalingListAddress(sizeIdc, listIdc);//量化矩阵起始地址

      if ((sizeIdc==SCALING_LIST_32x32) && (listIdc%(SCALING_LIST_NUM/NUMBER_OF_PREDICTION_MODES) != 0)) // derive chroma32x32 from chroma16x16
      {//chroma32x32的量化矩阵从 chroma16x16的量化矩阵得到
        const Int *srcNextSmallerSize = getScalingListAddress(sizeIdc-1, listIdc);
        for(UInt i=0; i<size; i++)
        {
          src[i] = srcNextSmallerSize[i];
        }
        setScalingListDC(sizeIdc,listIdc,(sizeIdc > SCALING_LIST_8x8) ? getScalingListDC(sizeIdc-1, listIdc) : src[0]);
      }
      else
      {
        {
          fseek(fp, 0, SEEK_SET);
          Bool bFound=false;
          while ((!feof(fp)) && (!bFound))
          {
            Char *ret = fgets(line, LINE_SIZE, fp);
            Char *findNamePosition= ret==NULL ? NULL : strstr(line, MatrixType[sizeIdc][listIdc]);//在给定文件中是否找到对应量化矩阵类型的字符 strstr(str1,str2) 函数用于判断字符串str2是否是str1的子串
            // This could be a match against the DC string as well, so verify it isn't
            if (findNamePosition!= NULL && (MatrixType_DC[sizeIdc][listIdc]==NULL || strstr(line, MatrixType_DC[sizeIdc][listIdc])==NULL))
            {
              bFound=true;//在文件中找到该量化矩阵的字符
            }
          }
          if (!bFound)//若未找到 打印错误信息
          {
            fprintf(stderr, "Error: cannot find Matrix %s from scaling list file %s\n", MatrixType[sizeIdc][listIdc], pchFile);
            return true;
          }
        }
        for (UInt i=0; i<size; i++)//读取自定义文件中量化矩阵的值
        {
          Int data;
          if (fscanf(fp, "%d,", &data)!=1)//若读取发生错误打印错误信息
          {
            fprintf(stderr, "Error: cannot read value #%d for Matrix %s from scaling list file %s at file position %ld\n", i, MatrixType[sizeIdc][listIdc], pchFile, ftell(fp));
            return true;
          }
          if (data<0 || data>255)//若值的范围不在（0,255）打印错误信息
          {
            fprintf(stderr, "Error: QMatrix entry #%d of value %d for Matrix %s from scaling list file %s at file position %ld is out of range (0 to 255)\n", i, data, MatrixType[sizeIdc][listIdc], pchFile, ftell(fp));
            return true;
          }
          src[i] = data;//将读取的值赋给量化矩阵
        }

        //set DC value for default matrix check
        setScalingListDC(sizeIdc,listIdc,src[0]);

        if(sizeIdc > SCALING_LIST_8x8)//量化矩阵大于8*8 需重新设置DC值
        {
          {
            fseek(fp, 0, SEEK_SET);
            Bool bFound=false;
            while ((!feof(fp)) && (!bFound))//同上
            {
              Char *ret = fgets(line, LINE_SIZE, fp);
              Char *findNamePosition= ret==NULL ? NULL : strstr(line, MatrixType_DC[sizeIdc][listIdc]);
              if (findNamePosition!= NULL)
              {
                // This won't be a match against the non-DC string.
                bFound=true;
              }
            }
            if (!bFound)//若未找到 打印错误信息
            {
              fprintf(stderr, "Error: cannot find DC Matrix %s from scaling list file %s\n", MatrixType_DC[sizeIdc][listIdc], pchFile);
              return true;
            }
          }
          Int data;
          if (fscanf(fp, "%d,", &data)!=1)//若读取发生错误打印错误信息
          {
            fprintf(stderr, "Error: cannot read DC %s from scaling list file %s at file position %ld\n", MatrixType_DC[sizeIdc][listIdc], pchFile, ftell(fp));
            return true;
          }
          if (data<0 || data>255)//若值的范围不在（0,255）打印错误信息
          {
            fprintf(stderr, "Error: DC value %d for Matrix %s from scaling list file %s at file position %ld is out of range (0 to 255)\n", data, MatrixType[sizeIdc][listIdc], pchFile, ftell(fp));
            return true;
          }
          //overwrite DC value when size of matrix is larger than 16x16
          setScalingListDC(sizeIdc,listIdc,data);//量化矩阵大于8*8 需重新设置DC值
        }
      }
    }
  }
//  std::cout << "\n\nRead scaling lists of:\n\n";
//  outputScalingLists(std::cout);

  fclose(fp);
  return false;
}


/** get default address of quantization matrix
 * \param sizeId size index
 * \param listId list index
 * \returns pointer of quantization matrix
 */
const Int* TComScalingList::getScalingListDefaultAddress(UInt sizeId, UInt listId)
{//����Ĭ�����������ַ
  const Int *src = 0;
  switch(sizeId)//�ж����������С
  {
    case SCALING_LIST_4x4://4*4��Сֱ�ӵõ�
      src = g_quantTSDefault4x4;
      break;
    case SCALING_LIST_8x8:
    case SCALING_LIST_16x16:
    case SCALING_LIST_32x32:
      src = (listId < (SCALING_LIST_NUM/NUMBER_OF_PREDICTION_MODES) ) ? g_quantIntraDefault8x8 : g_quantInterDefault8x8;//������С��ѡ���Ӧģʽ8*8�������� 16*16 32*32��8*8�ϲ����õ�
      break;
    default:
      assert(0);
      src = NULL;
      break;
  }
  return src;
}

/** process of default matrix
 * \param sizeId size index
 * \param listId index of input matrix
 */
Void TComScalingList::processDefaultMatrix(UInt sizeId, UInt listId)//使用默认量化矩阵作为量化矩阵
{
  ::memcpy(getScalingListAddress(sizeId, listId),getScalingListDefaultAddress(sizeId,listId),sizeof(Int)*min(MAX_MATRIX_COEF_NUM,(Int)g_scalingListSize[sizeId]));
  setScalingListDC(sizeId,listId,SCALING_LIST_DC);
}

/** check DC value of matrix for default matrix signaling
 */
Void TComScalingList::checkDcOfMatrix()//量化矩阵的DC值为0 则将其设为默认量化矩阵
{
  for(UInt sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(UInt listId = 0; listId < SCALING_LIST_NUM; listId++)
    {
      //check default matrix?
      if(getScalingListDC(sizeId,listId) == 0)
      {
        processDefaultMatrix(sizeId, listId);
      }
    }
  }
}

ParameterSetManager::ParameterSetManager()
: m_vpsMap(MAX_NUM_VPS)
, m_spsMap(MAX_NUM_SPS)
, m_ppsMap(MAX_NUM_PPS)
, m_activeVPSId(-1)
, m_activeSPSId(-1)
{
}


ParameterSetManager::~ParameterSetManager()
{
}

//! activate a SPS from a active parameter sets SEI message
//! \returns true, if activation is successful
//Bool ParameterSetManager::activateSPSWithSEI(Int spsId)
//{
//  TComSPS *sps = m_spsMap.getPS(spsId);
//  if (sps)
//  {
//    Int vpsId = sps->getVPSId();
//    TComVPS *vps = m_vpsMap.getPS(vpsId);
//    if (vps)
//    {
//      m_activeVPS = *(vps);
//      m_activeSPS = *(sps);
//      return true;
//    }
//    else
//    {
//      printf("Warning: tried to activate SPS using an Active parameter sets SEI message. Referenced VPS does not exist.");
//    }
//  }
//  else
//  {
//    printf("Warning: tried to activate non-existing SPS using an Active parameter sets SEI message.");
//  }
//  return false;
//}

//! activate a PPS and depending on isIDR parameter also SPS and VPS
//! \returns true, if activation is successful
Bool ParameterSetManager::activatePPS(Int ppsId, Bool isIRAP)//激活ID为ppsId的PPS 激活成功返回真(类似链表 PPS->SPS->VPS)
{
  TComPPS *pps = m_ppsMap.getPS(ppsId);//ID为ppsId的PPS
  if (pps)//如果该PPS存在
  {
    Int spsId = pps->getSPSId();//得到该PPS所用SPS的ID
    if (!isIRAP && (spsId != m_activeSPSId ))//该PPS所用SPS未激活 即无法得到该SPS参数 而该帧图像又不为IDR(IDR图像不需要SPS参数)
    {
      printf("Warning: tried to activate PPS referring to a inactive SPS at non-IDR.");//则该帧图像的PPS激活失败
    }
    else//该PPS所用SPS激活
    {
      TComSPS *sps = m_spsMap.getPS(spsId);//该PPS所用SPS
      if (sps)//该PPS所用SPS存在
      {
        Int vpsId = sps->getVPSId();//得到该PPS所用SPS所用的VPS的ID
        if (!isIRAP && (vpsId != m_activeVPSId ))//该SPS所用vps未激活 即无法得到该vps参数 而该帧图像又不为IDR(IDR图像不需要vps参数)
        {
          printf("Warning: tried to activate PPS referring to a inactive VPS at non-IDR.");
        }
        else
        {
          TComVPS *vps =m_vpsMap.getPS(vpsId);
          if (vps)//该SPS所用vps存在
          {
            m_activeVPSId = vpsId;//保存vpsId
            m_activeSPSId = spsId;//保存spsId
            return true;//激活成功
          }
          else
          {
            printf("Warning: tried to activate PPS that refers to a non-existing VPS.");
          }
        }
      }
      else
      {
        printf("Warning: tried to activate a PPS that refers to a non-existing SPS.");
      }
    }
  }
  else
  {
    printf("Warning: tried to activate non-existing PPS.");
  }

  // Failed to activate if reach here.
  m_activeSPSId=-1;//激活失败 置为-1作为标志
  m_activeVPSId=-1;
  return false;
}

ProfileTierLevel::ProfileTierLevel()
  : m_profileSpace    (0)
  , m_tierFlag        (Level::MAIN)
  , m_profileIdc      (Profile::NONE)
  , m_levelIdc        (Level::NONE)
  , m_progressiveSourceFlag  (false)
  , m_interlacedSourceFlag   (false)
  , m_nonPackedConstraintFlag(false)
  , m_frameOnlyConstraintFlag(false)
{
  ::memset(m_profileCompatibilityFlag, 0, sizeof(m_profileCompatibilityFlag));
}

TComPTL::TComPTL()
{
  ::memset(m_subLayerProfilePresentFlag, 0, sizeof(m_subLayerProfilePresentFlag));
  ::memset(m_subLayerLevelPresentFlag,   0, sizeof(m_subLayerLevelPresentFlag  ));
}

Void calculateParameterSetChangedFlag(Bool &bChanged, const std::vector<UChar> *pOldData, const std::vector<UChar> &newData)//比较给定的给定两组数是否改变
{
  if (!bChanged)
  {
    if ((pOldData==0 && pOldData!=0) || (pOldData!=0 && pOldData==0))//一组为0 另一组不为0 则肯定改变过
    {
      bChanged=true;
    }
    else if (pOldData!=0 && pOldData!=0)//两组都不为0
    {
      // compare the two
      if (pOldData->size() != pOldData->size())//若两组数的个数不同 则肯定改变过
      {
        bChanged=true;
      }
      else
      {
        const UChar *pNewDataArray=&(newData)[0];
        const UChar *pOldDataArray=&(*pOldData)[0];
        if (memcmp(pOldDataArray, pNewDataArray, pOldData->size()))//若两组数不完全一样 则肯定改变过
        {
          bChanged=true;
        }
      }
    }
  }
}

//! \}
