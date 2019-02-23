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

/** \file     TEncSlice.cpp
    \brief    slice encoder class
*/

#include "TEncTop.h"
#include "TEncSlice.h"
#include <math.h>

//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TEncSlice::TEncSlice()
 : m_encCABACTableIdx(I_SLICE)
{
  m_apcPicYuvPred = NULL;
  m_apcPicYuvResi = NULL;

  m_pdRdPicLambda = NULL;
  m_pdRdPicQp     = NULL;
  m_piRdPicQp     = NULL;
}

TEncSlice::~TEncSlice()
{
}

Void TEncSlice::create( Int iWidth, Int iHeight, ChromaFormat chromaFormat, UInt iMaxCUWidth, UInt iMaxCUHeight, UChar uhTotalDepth )//创建当前SLice的预测图像和残差图像
{
  // create prediction picture
  if ( m_apcPicYuvPred == NULL )
  {
    m_apcPicYuvPred  = new TComPicYuv;
    m_apcPicYuvPred->create( iWidth, iHeight, chromaFormat, iMaxCUWidth, iMaxCUHeight, uhTotalDepth, true );
  }

  // create residual picture
  if( m_apcPicYuvResi == NULL )
  {
    m_apcPicYuvResi  = new TComPicYuv;
    m_apcPicYuvResi->create( iWidth, iHeight, chromaFormat, iMaxCUWidth, iMaxCUHeight, uhTotalDepth, true );
  }
}

Void TEncSlice::destroy()//销毁对象 释放资源
{
  // destroy prediction picture
  if ( m_apcPicYuvPred )
  {
    m_apcPicYuvPred->destroy();
    delete m_apcPicYuvPred;
    m_apcPicYuvPred  = NULL;
  }

  // destroy residual picture
  if ( m_apcPicYuvResi )
  {
    m_apcPicYuvResi->destroy();
    delete m_apcPicYuvResi;
    m_apcPicYuvResi  = NULL;
  }

  // free lambda and QP arrays
  if ( m_pdRdPicLambda )
  {
    xFree( m_pdRdPicLambda );
    m_pdRdPicLambda = NULL;
  }
  if ( m_pdRdPicQp )
  {
    xFree( m_pdRdPicQp );
    m_pdRdPicQp = NULL;
  }
  if ( m_piRdPicQp )
  {
    xFree( m_piRdPicQp );
    m_piRdPicQp = NULL;
  }
}

Void TEncSlice::init( TEncTop* pcEncTop )
{
  m_pcCfg             = pcEncTop;
  m_pcListPic         = pcEncTop->getListPic();

  m_pcGOPEncoder      = pcEncTop->getGOPEncoder();
  m_pcCuEncoder       = pcEncTop->getCuEncoder();
  m_pcPredSearch      = pcEncTop->getPredSearch();

  m_pcEntropyCoder    = pcEncTop->getEntropyCoder();
  m_pcSbacCoder       = pcEncTop->getSbacCoder();
  m_pcBinCABAC        = pcEncTop->getBinCABAC();
  m_pcTrQuant         = pcEncTop->getTrQuant();

  m_pcRdCost          = pcEncTop->getRdCost();
  m_pppcRDSbacCoder   = pcEncTop->getRDSbacCoder();
  m_pcRDGoOnSbacCoder = pcEncTop->getRDGoOnSbacCoder();//通过TEncTop中的对象初始化TEncSlice中编码相关的对象

  // create lambda and QP arrays
  m_pdRdPicLambda     = (Double*)xMalloc( Double, m_pcCfg->getDeltaQpRD() * 2 + 1 );
  m_pdRdPicQp         = (Double*)xMalloc( Double, m_pcCfg->getDeltaQpRD() * 2 + 1 );
  m_piRdPicQp         = (Int*   )xMalloc( Int,    m_pcCfg->getDeltaQpRD() * 2 + 1 );//分配数组变量的内存空间
  m_pcRateCtrl        = pcEncTop->getRateCtrl();
}



Void
TEncSlice::setUpLambda(TComSlice* slice, const Double dLambda, Int iQP)//设置lambda值  色度分量的lambda值由亮度分量的lambda值计算得到 Wiegand and Ohm等人将lambda与QP值间建立了如下关系
{                                                                      //lambda=C*Qstep^2 故色度分量的lambda值可以通过量化步长平方的比值得到
  // store lambda
  m_pcRdCost ->setLambda( dLambda, slice->getSPS()->getBitDepths() );//保存lambda值

  // for RDO
  // in RdCost there is only one lambda because the luma and chroma bits are not separated, instead we weight the distortion of chroma.
  Double dLambdas[MAX_NUM_COMPONENT] = { dLambda };
  for(UInt compIdx=1; compIdx<MAX_NUM_COMPONENT; compIdx++)//遍历Cr Cb色度分量 计算其lambda值
  {
    const ComponentID compID=ComponentID(compIdx);
    Int chromaQPOffset = slice->getPPS()->getQpOffset(compID) + slice->getSliceChromaQpDelta(compID);//该色度分量的QP偏移值
    Int qpc=(iQP + chromaQPOffset < 0) ? iQP : getScaledChromaQP(iQP + chromaQPOffset, m_pcCfg->getChromaFormatIdc());//该色度分量的QP值
    Double tmpWeight = pow( 2.0, (iQP-qpc)/3.0 );  // takes into account of the chroma qp mapping and chroma qp Offset//计算Wchroma
    m_pcRdCost->setDistortionWeight(compID, tmpWeight);//设置色度失真的权重//Jmode = (SSEluma + Wchroma*SSEchroma) + lambda_mode*Rmode
    dLambdas[compIdx]=dLambda/tmpWeight;//该色度分量的lambda_chroma值为dLambda/Wchroma
  }

#if RDOQ_CHROMA_LAMBDA//若色度分量使用率失真优化量化
// for RDOQ
  m_pcTrQuant->setLambdas( dLambdas );//设置各分量Lambdas(用于RDOQ)
#else
  m_pcTrQuant->setLambda( dLambda );
#endif

// For SAO
  slice   ->setLambdas( dLambdas );//设置各分量Lambdas(用于SAO)
}



/**
 - non-referenced frame marking
 - QP computation based on temporal structure
 - lambda computation based on QP
 - set temporal layer ID and the parameter sets
 .
 \param pcPic         picture class
 \param pocLast       POC of last picture
 \param pocCurr       current POC
 \param iNumPicRcvd   number of received pictures
 \param iGOPid        POC offset for hierarchical structure
 \param rpcSlice      slice header class
 \param isField       true for field coding
 */
//该部分许多参数可参考<software manual>
Void TEncSlice::initEncSlice( TComPic* pcPic, Int pocLast, Int pocCurr, Int iGOPid, TComSlice*& rpcSlice, Bool isField )
{
  Double dQP;
  Double dLambda;

  rpcSlice = pcPic->getSlice(0);//将rpcSlice指向该图像的第一个slice!!!!
  rpcSlice->setSliceBits(0);
  rpcSlice->setPic( pcPic );//该slice所在图像
  rpcSlice->initSlice();
  rpcSlice->setPicOutputFlag( true );//该slice需要输出
  rpcSlice->setPOC( pocCurr );//该slice所在图像的POC值

  // depth computation based on GOP size
  Int depth;
  {
    Int poc = rpcSlice->getPOC();////该slice所在图像的POC值
    if(isField)//计算该slice所在图像在一个GOP中的位置
    {
      poc = (poc/2) % (m_pcCfg->getGOPSize()/2);
    }
    else
    {
      poc = poc % m_pcCfg->getGOPSize();   
    }

    if ( poc == 0 )//计算该图像在GOP中的深度
    {//该图像为GOP中第一帧图像则层级为0
      depth = 0;
    }
    else
    {
      Int step = m_pcCfg->getGOPSize();
      depth    = 0;
      for( Int i=step>>1; i>=1; i>>=1 )
      {
        for ( Int j=i; j<m_pcCfg->getGOPSize(); j+=step )
        {
          if ( j == poc )
          {
            i=0;
            break;
          }
        }
        step >>= 1;
        depth++;
      }
    }

    if(m_pcCfg->getHarmonizeGopFirstFieldCoupleEnabled() && poc != 0)
    {
      if (isField && ((rpcSlice->getPOC() % 2) == 1))
      {
        depth ++;
      }
    }
  }

  // slice type
  SliceType eSliceType;

  eSliceType=B_SLICE;
  if(!(isField && pocLast == 1) || !m_pcCfg->getEfficientFieldIRAPEnabled())
  {
    if(m_pcCfg->getDecodingRefreshType() == 3)//Use recovery point SEI messages to indicate random access
    {//poclast=0说明为视频流中第一帧图像  pocCurr % m_pcCfg->getIntraPeriod() = 0说明该图像正好为帧内编码帧  GOPSize=0说明每一帧都独立编解码故一定为I_slice
      eSliceType = (pocLast == 0 || pocCurr % m_pcCfg->getIntraPeriod() == 0             || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
    }
    else
    {
      eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
    }
  }

  rpcSlice->setSliceType    ( eSliceType );//设置slice类型

  // ------------------------------------------------------------------------------------------------------------------
  // Non-referenced frame marking//可参阅<HECV book>中Sub-layer Reference and Sub-layer Non-reference Pictures的说明
  // ------------------------------------------------------------------------------------------------------------------

  if(pocLast == 0)//为编码视频流中第一帧图像
  {
    rpcSlice->setTemporalLayerNonReferenceFlag(false);//视频流中第一帧图像一定用作参考图像 故为Sub-layer Reference图像 Sub-layer Reference图像表明该图像被同时域层图像用作参考 Sub-layer Non-reference Pictures表明该图像不被同时域层任何图像用作参考
  }
  else
  {
    rpcSlice->setTemporalLayerNonReferenceFlag(!m_pcCfg->getGOPEntry(iGOPid).m_refPic);//m_refPic表明图像是否为Sub-layer Reference图像  m_refPic的求得可参阅TAppEnCfg.cpp中1849行往后的代码
  }
  rpcSlice->setReferenced(true);//初始化isReferenced=true!!!!很重要 与TcomSlice.ccp中方法对应

  // ------------------------------------------------------------------------------------------------------------------
  // QP setting
  // ------------------------------------------------------------------------------------------------------------------

  dQP = m_pcCfg->getQP();//从配置文件中获得QP值
  if(eSliceType!=I_SLICE)//不为I_slice
  {
    if (!(( m_pcCfg->getMaxDeltaQP() == 0 ) && (dQP == -rpcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA) ) && (rpcSlice->getPPS()->getTransquantBypassEnableFlag())))//QpBDOffset QP值的位深偏移量 初始化为0
    {
      dQP += m_pcCfg->getGOPEntry(iGOPid).m_QPOffset;//QP值加上配置文件中的QPOffset
    }
  }

  // modify QP
  Int* pdQPs = m_pcCfg->getdQPs();
  if ( pdQPs )//若存在pdQPs
  {
    dQP += pdQPs[ rpcSlice->getPOC() ];//最终QP加上pdQPs中对应位置dQP值
  }

  if (m_pcCfg->getCostMode()==COST_LOSSLESS_CODING)//COST_LOSSLESS_CODING损耗模式下 QP设置为0
  {
    dQP=LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP;
    m_pcCfg->setDeltaQpRD(0);//DeltaQpRD设置为0(用于slice多QP值优化)
  }

  // ------------------------------------------------------------------------------------------------------------------
  // Lambda computation
  // ------------------------------------------------------------------------------------------------------------------

  Int iQP;
  Double dOrigQP = dQP;

  // pre-compute lambda and QP values for all possible QP candidates
  for ( Int iDQpIdx = 0; iDQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; iDQpIdx++ )//遍历该slice所有可能的QP值  DeltaQpRD为slice多QP值优化时QP允许变动的绝对值范围
  {
    // compute QP value
    dQP = dOrigQP + ((iDQpIdx+1)>>1)*(iDQpIdx%2 ? -1 : 1);//当前QP值 (原QP值加上QP值的改变量)

    // compute lambda value//lambda=alpha*Wk*2^((QP-12)/3.0) Wk与配置文件 slice类型 图像在GOP中的层级 是否用作参考等有关 具体的计算过程可参考<software manual>中LambdaFromQpEnable部分的详细说明!!!
    Int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );//GOP中B帧图像数
    Int    SHIFT_QP = 12;

    Double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(Double)(isField ? NumberBFrames/2 : NumberBFrames) );//计算alpha值 其值与当前图像是否作为参考图像及GOP中B帧图像数有关

#if FULL_NBIT
    Int    bitdepth_luma_qp_scale = 6 * (rpcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 8);
#else
    Int    bitdepth_luma_qp_scale = 0;
#endif
    Double qp_temp = (Double) dQP + bitdepth_luma_qp_scale - SHIFT_QP;
#if FULL_NBIT
    Double qp_temp_orig = (Double) dQP - SHIFT_QP;
#endif
    // Case #1: I or P-slices (key-frame)
    Double dQPFactor = m_pcCfg->getGOPEntry(iGOPid).m_QPFactor;//由配置文件得到QPFactor 
    if ( eSliceType==I_SLICE )
    {
      dQPFactor=0.57*dLambda_scale;//I_slice时Wk为0.57
    }
    dLambda = dQPFactor*pow( 2.0, qp_temp/3.0 );//计算lambda

    if ( depth>0 )//大于零一定为B slice
    {
#if FULL_NBIT
        dLambda *= Clip3( 2.00, 4.00, (qp_temp_orig / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )
#else
        dLambda *= Clip3( 2.00, 4.00, (qp_temp / 6.0) ); // (j == B_SLICE && p_cur_frm->layer != 0 )//当slice为B slice且层级不为顶层时还需*Clip3( 2.00, 4.00, (qp_temp / 6.0) )
#endif
    }

    // if hadamard is used in ME process
    if ( !m_pcCfg->getUseHADME() && rpcSlice->getSliceType( ) != I_SLICE )//若运动估计过程使用HAD 则lambda还需* 0.95
    {
      dLambda *= 0.95;
    }

    iQP = max( -rpcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );//QP范围
    //设置该iDQpIdx(DeltaQpRD范围内)对应的Lambda dQP IQP值
    m_pdRdPicLambda[iDQpIdx] = dLambda;//设置Lambda
    m_pdRdPicQp    [iDQpIdx] = dQP;
    m_piRdPicQp    [iDQpIdx] = iQP;
  }

  // obtain dQP = 0 case//得到 dQp=0的情况
  dLambda = m_pdRdPicLambda[0];
  dQP     = m_pdRdPicQp    [0];
  iQP     = m_piRdPicQp    [0];

  if( rpcSlice->getSliceType( ) != I_SLICE )//不为I slice
  {
    dLambda *= m_pcCfg->getLambdaModifier( m_pcCfg->getGOPEntry(iGOPid).m_temporalId );//乘上对应时域层的因子
  }

  setUpLambda(rpcSlice, dLambda, iQP);//设置该slice的 Lambdas值

  if (m_pcCfg->getFastMEForGenBLowDelayEnabled())//广义B帧使用快速运动估计
  {
    // restore original slice type

    if(!(isField && pocLast == 1) || !m_pcCfg->getEfficientFieldIRAPEnabled())
    {
      if(m_pcCfg->getDecodingRefreshType() == 3)
      {
        eSliceType = (pocLast == 0 || (pocCurr)                     % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
      }
      else
      {
        eSliceType = (pocLast == 0 || (pocCurr - (isField ? 1 : 0)) % m_pcCfg->getIntraPeriod() == 0 || m_pcGOPEncoder->getGOPSize() == 0) ? I_SLICE : eSliceType;
      }
    }

    rpcSlice->setSliceType        ( eSliceType );
  }//为什么要重新store一次???

  if (m_pcCfg->getUseRecalculateQPAccordingToLambda())//根据lambda值重新计算QP值(用于码率控制 一般由码率计算得到lambda值)
  {
    dQP = xGetQPValueAccordingToLambda( dLambda );//由lambda值计算Qp值
    iQP = max( -rpcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), min( MAX_QP, (Int) floor( dQP + 0.5 ) ) );//最终QP值
  }

  rpcSlice->setSliceQp           ( iQP );//设置sliceQP值
#if ADAPTIVE_QP_SELECTION
  rpcSlice->setSliceQpBase       ( iQP );
#endif
  rpcSlice->setSliceQpDelta      ( 0 );//设置slice deltaQp值为0 (为dQp=0的情况)
  rpcSlice->setSliceChromaQpDelta( COMPONENT_Cb, 0 );
  rpcSlice->setSliceChromaQpDelta( COMPONENT_Cr, 0 );//设置色度分量slice deltaQP值为0
  rpcSlice->setUseChromaQpAdj( rpcSlice->getPPS()->getPpsRangeExtension().getChromaQpOffsetListEnabledFlag() );//由PPS信息设置是否使用色度QP偏移量列表
  rpcSlice->setNumRefIdx(REF_PIC_LIST_0,m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive);
  rpcSlice->setNumRefIdx(REF_PIC_LIST_1,m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive);//设置list0 list1中的参考图像数
  //设置去方块滤波相关参数
  if ( m_pcCfg->getDeblockingFilterMetric() )//Specifies the use of a deblocking filter metric to evaluate the suitability of deblocking
  {
    rpcSlice->setDeblockingFilterOverrideFlag(true);
    rpcSlice->setDeblockingFilterDisable(false);//使用去方块滤波
    rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );//设置去方块滤波中BetaOffsetDiv2参数为0
    rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );//设置去方块滤波中TcOffsetDiv2参数为0
  }
  else if (rpcSlice->getPPS()->getDeblockingFilterControlPresentFlag())//PPS信息中存在去方块滤波控制标志
  {
    rpcSlice->setDeblockingFilterOverrideFlag( rpcSlice->getPPS()->getDeblockingFilterOverrideEnabledFlag() );
    rpcSlice->setDeblockingFilterDisable( rpcSlice->getPPS()->getPicDisableDeblockingFilterFlag() );//由PPS信息得到该slice是否使用去方块滤波
    if ( !rpcSlice->getDeblockingFilterDisable())//若该slice使用去方块滤波
    {
      if ( rpcSlice->getDeblockingFilterOverrideFlag() && eSliceType!=I_SLICE)//设置slice的去方块滤波中的TcOffsetDiv2和BetaOffsetDiv2参数
      {
        rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_betaOffsetDiv2 + m_pcCfg->getLoopFilterBetaOffset()  );
        rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getGOPEntry(iGOPid).m_tcOffsetDiv2 + m_pcCfg->getLoopFilterTcOffset() );
      }
      else//不使用GOP参数中的TcOffsetDiv2和BetaOffsetDiv2
      {
        rpcSlice->setDeblockingFilterBetaOffsetDiv2( m_pcCfg->getLoopFilterBetaOffset() );
        rpcSlice->setDeblockingFilterTcOffsetDiv2( m_pcCfg->getLoopFilterTcOffset() );
      }
    }
  }
  else//no DeblockingFilterControlPresentFlag no DeblockingFilterMetric
  {
    rpcSlice->setDeblockingFilterOverrideFlag( false );
    rpcSlice->setDeblockingFilterDisable( false );
    rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
    rpcSlice->setDeblockingFilterTcOffsetDiv2( 0 );//直接设置TcOffsetDiv2和BetaOffsetDiv2为0
  }

  rpcSlice->setDepth            ( depth );//设置slice的层级

  pcPic->setTLayer( m_pcCfg->getGOPEntry(iGOPid).m_temporalId );//根据配置文件中iGOPid所在GOP的时域层设置该图像的时域层
  if(eSliceType==I_SLICE)//I slice 时域层只能为0 (高时域层的图像必须参考其他图像)
  {
    pcPic->setTLayer(0);
  }
  rpcSlice->setTLayer( pcPic->getTLayer() );//设置slice的时域层

  assert( m_apcPicYuvPred );
  assert( m_apcPicYuvResi );

  pcPic->setPicYuvPred( m_apcPicYuvPred );
  pcPic->setPicYuvResi( m_apcPicYuvResi );//设置该图像的预测图像和残差图像
  rpcSlice->setSliceMode            ( m_pcCfg->getSliceMode()            );//设置slice的划分模式
  rpcSlice->setSliceArgument        ( m_pcCfg->getSliceArgument()        );//根据slice的划分模式设置一个slice中最大的CTus或最大的Tiles或最大的bytes
  rpcSlice->setSliceSegmentMode     ( m_pcCfg->getSliceSegmentMode()     );//设置ss的划分模式
  rpcSlice->setSliceSegmentArgument ( m_pcCfg->getSliceSegmentArgument() );//设置根据ss的划分模式设置一个slice中最大的CTus或最大的Tiles或最大的bytes
  rpcSlice->setMaxNumMergeCand        ( m_pcCfg->getMaxNumMergeCand()        );//根据配置文件设置该slice的merge候选列表中允许的最大MV个数
}

Void TEncSlice::resetQP( TComPic* pic, Int sliceQP, Double lambda )//用给定的sliceQP值 lambda值重置该slice的QP值和ambda值
{
  TComSlice* slice = pic->getSlice(0);

  // store lambda
  slice->setSliceQp( sliceQP );
#if ADAPTIVE_QP_SELECTION
  slice->setSliceQpBase ( sliceQP );
#endif
  setUpLambda(slice, lambda, sliceQP);
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Void TEncSlice::setSearchRange( TComSlice* pcSlice )//设置该slice运动估计时允许的搜索范围
{
  Int iCurrPOC = pcSlice->getPOC();
  Int iRefPOC;
  Int iGOPSize = m_pcCfg->getGOPSize();
  Int iOffset = (iGOPSize >> 1);
  Int iMaxSR = m_pcCfg->getSearchRange();//根据配置文件得到最大搜索范围
  Int iNumPredDir = pcSlice->isInterP() ? 1 : 2;//预测方向
  //计算自适应搜索范围 搜索范围根据当前图像和参考图像间的POC的差值动态调整
  for (Int iDir = 0; iDir <= iNumPredDir; iDir++)//遍历list0 list1两个参考图像列表
  {
    //RefPicList e = (RefPicList)iDir;
    RefPicList  e = ( iDir ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
    for (Int iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(e); iRefIdx++)//遍历参考图像列表中的参考图像
    {
      iRefPOC = pcSlice->getRefPic(e, iRefIdx)->getPOC();//该参考图像的POC值
      Int iNewSR = Clip3(8, iMaxSR, (iMaxSR*ADAPT_SR_SCALE*abs(iCurrPOC - iRefPOC)+iOffset)/iGOPSize);
      m_pcPredSearch->setAdaptiveSearchRange(iDir, iRefIdx, iNewSR);//设置自适应搜索范围
    }
  }
}

/**
 Multi-loop slice encoding for different slice QP

 \param pcPic    picture class
 */
Void TEncSlice::precompressSlice( TComPic* pcPic )//用不同的QP值压缩该slice 并计算出最优的QP值
{
  // if deltaQP RD is not used, simply return
  if ( m_pcCfg->getDeltaQpRD() == 0 )//不使用 deltaQP RD 直接结束方法
  {
    return;
  }

  if ( m_pcCfg->getUseRateCtrl() )//slice多QP值优化时不允许使用码率控制
  {
    printf( "\nMultiple QP optimization is not allowed when rate control is enabled." );
    assert(0);
    return;
  }

  TComSlice* pcSlice        = pcPic->getSlice(getSliceIdx());

  if (pcSlice->getDependentSliceSegmentFlag())//依赖SS
  {
    // if this is a dependent slice segment, then it was optimised
    // when analysing the entire slice.
    return;
  }

  if (pcSlice->getSliceMode()==FIXED_NUMBER_OF_BYTES)//该slice若固定byte数则无法优化slice的QP值
  {
    // TODO: investigate use of average cost per CTU so that this Slice Mode can be used.
    printf( "\nUnable to optimise Slice-level QP if Slice Mode is set to FIXED_NUMBER_OF_BYTES\n" );
    assert(0);
    return;
  }

  Double     dPicRdCostBest = MAX_DOUBLE;
  UInt       uiQpIdxBest = 0;

  Double dFrameLambda;
#if FULL_NBIT
  Int    SHIFT_QP = 12 + 6 * (pcSlice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 8);
#else
  Int    SHIFT_QP = 12;
#endif

  // set frame lambda
  if (m_pcCfg->getGOPSize() > 1)//若GOP图像数大于1则需判断该slice是否为B slice
  {
    dFrameLambda = 0.68 * pow (2, (m_piRdPicQp[0]  - SHIFT_QP) / 3.0) * (pcSlice->isInterB()? 2 : 1);
  }
  else
  {
    dFrameLambda = 0.68 * pow (2, (m_piRdPicQp[0] - SHIFT_QP) / 3.0);
  }
  m_pcRdCost      ->setFrameLambda(dFrameLambda);//设置dFrameLambda

  // for each QP candidate
  for ( UInt uiQpIdx = 0; uiQpIdx < 2 * m_pcCfg->getDeltaQpRD() + 1; uiQpIdx++ )//遍历所有可能的QP值
  {
    pcSlice       ->setSliceQp             ( m_piRdPicQp    [uiQpIdx] );//设置该slice的QP值
#if ADAPTIVE_QP_SELECTION
    pcSlice       ->setSliceQpBase         ( m_piRdPicQp    [uiQpIdx] );
#endif
    setUpLambda(pcSlice, m_pdRdPicLambda[uiQpIdx], m_piRdPicQp    [uiQpIdx]);//设置该slice的Lambda值(initEncSlice中已经计算过)

    // try compress
    compressSlice   ( pcPic, true, m_pcCfg->getFastDeltaQp());//压缩该slice

    UInt64 uiPicDist        = m_uiPicDist; // Distortion, as calculated by compressSlice.//该slice在该QP值下的失真
    // NOTE: This distortion is the chroma-weighted SSE distortion for the slice.
    //       Previously a standard SSE distortion was calculated (for the entire frame).
    //       Which is correct?

    // TODO: Update loop filter, SAO and distortion calculation to work on one slice only.
    // m_pcGOPEncoder->preLoopFilterPicAll( pcPic, uiPicDist );

    // compute RD cost and choose the best
    Double dPicRdCost = m_pcRdCost->calcRdCost64( m_uiPicTotalBits, uiPicDist, true, DF_SSE_FRAME); // NOTE: Is the 'true' parameter really necessary?////该slice在该QP值下的总损耗

    if ( dPicRdCost < dPicRdCostBest )//若该QP下的slic损耗优于之前最优损耗
    {
      uiQpIdxBest    = uiQpIdx;//更新最优QP值的索引
      dPicRdCostBest = dPicRdCost;//更新最优总损耗
    }
  }

  // set best values
  pcSlice       ->setSliceQp             ( m_piRdPicQp    [uiQpIdxBest] );//设置slic的QP值为最优QP值
#if ADAPTIVE_QP_SELECTION
  pcSlice       ->setSliceQpBase         ( m_piRdPicQp    [uiQpIdxBest] );
#endif
  setUpLambda(pcSlice, m_pdRdPicLambda[uiQpIdxBest], m_piRdPicQp    [uiQpIdxBest]);//设置slice的最优lambda值为最优QP对应下的lambda值
}

Void TEncSlice::calCostSliceI(TComPic* pcPic) // TODO: this only analyses the first slice segment. What about the others?
{
  Double            iSumHadSlice      = 0;
  TComSlice * const pcSlice           = pcPic->getSlice(getSliceIdx());//当前slice
  const TComSPS    &sps               = *(pcSlice->getSPS());
  const Int         shift             = sps.getBitDepth(CHANNEL_TYPE_LUMA)-8;
  const Int         offset            = (shift>0)?(1<<(shift-1)):0;//取整时的偏移量

  pcSlice->setSliceSegmentBits(0);

  UInt startCtuTsAddr, boundingCtuTsAddr;
  xDetermineStartAndBoundingCtuTsAddr ( startCtuTsAddr, boundingCtuTsAddr, pcPic );//得到该slice中第一个ss起始Ctu和末尾Ctu的Ts位置

  for( UInt ctuTsAddr = startCtuTsAddr, ctuRsAddr = pcPic->getPicSym()->getCtuTsToRsAddrMap( startCtuTsAddr);
       ctuTsAddr < boundingCtuTsAddr;
       ctuRsAddr = pcPic->getPicSym()->getCtuTsToRsAddrMap(++ctuTsAddr) )//遍历ss中所有Ctu
  {
    // initialize CU encoder
    TComDataCU* pCtu = pcPic->getCtu( ctuRsAddr );
    pCtu->initCtu( pcPic, ctuRsAddr );//初始化该Ctu

    Int height  = min( sps.getMaxCUHeight(),sps.getPicHeightInLumaSamples() - ctuRsAddr / pcPic->getFrameWidthInCtus() * sps.getMaxCUHeight() );
    Int width   = min( sps.getMaxCUWidth(), sps.getPicWidthInLumaSamples()  - ctuRsAddr % pcPic->getFrameWidthInCtus() * sps.getMaxCUWidth() );//Ctu的宽和高(在图像边界处的Ctu大小不一定为MaxCU)

    Int iSumHad = m_pcCuEncoder->updateCtuDataISlice(pCtu, width, height);//计算该Ctu的SumHad

    (m_pcRateCtrl->getRCPic()->getLCU(ctuRsAddr)).m_costIntra=(iSumHad+offset)>>shift;//设置该ss中每个Ctu的帧内预测损耗(SumHad)
    iSumHadSlice += (m_pcRateCtrl->getRCPic()->getLCU(ctuRsAddr)).m_costIntra;//该ss的SumHad

  }
  m_pcRateCtrl->getRCPic()->setTotalIntraCost(iSumHadSlice);//设置该ss的帧内预测总损耗
}

/** \param pcPic   picture class
 */
Void TEncSlice::compressSlice( TComPic* pcPic, const Bool bCompressEntireSlice, const Bool bFastDeltaQP )
{
  // if bCompressEntireSlice is true, then the entire slice (not slice segment) is compressed,
  //   effectively disabling the slice-segment-mode.

  UInt   startCtuTsAddr;
  UInt   boundingCtuTsAddr;
  TComSlice* const pcSlice            = pcPic->getSlice(getSliceIdx());
  pcSlice->setSliceSegmentBits(0);
  xDetermineStartAndBoundingCtuTsAddr ( startCtuTsAddr, boundingCtuTsAddr, pcPic );//得到该slice中第一个ss起始Ctu和末尾Ctu的Ts位置
  if (bCompressEntireSlice)//若压缩整个slice
  {
    boundingCtuTsAddr = pcSlice->getSliceCurEndCtuTsAddr();//则处理的末尾Ctu由ss的末尾Ctu变更为slice的末尾Ctu
    pcSlice->setSliceSegmentCurEndCtuTsAddr(boundingCtuTsAddr);//设置SS末尾Ctu位置为slice的末尾Ctu位置(之所以需要这样设置是因为EncGOP中对图像的压缩是以SS为单位的 见EncGOP.cpp中1406-1446行代码)
  }

  // initialize cost values - these are used by precompressSlice (they should be parameters).
  m_uiPicTotalBits  = 0;
  m_dPicRdCost      = 0; // NOTE: This is a write-only variable!
  m_uiPicDist       = 0;

  m_pcEntropyCoder->setEntropyCoder   ( m_pppcRDSbacCoder[0][CI_CURR_BEST] );
  m_pcEntropyCoder->resetEntropy      ( pcSlice );

  TEncBinCABAC* pRDSbacCoder = (TEncBinCABAC *) m_pppcRDSbacCoder[0][CI_CURR_BEST]->getEncBinIf();
  pRDSbacCoder->setBinCountingEnableFlag( false );
  pRDSbacCoder->setBinsCoded( 0 );

  TComBitCounter  tempBitCounter;
  const UInt      frameWidthInCtus = pcPic->getPicSym()->getFrameWidthInCtus();//该帧图像的宽(以Ctu为单位)
  
  m_pcCuEncoder->setFastDeltaQp(bFastDeltaQP);//设置是否使用FastDeltaQp

  //------------------------------------------------------------------------------
  //  Weighted Prediction parameters estimation.
  //------------------------------------------------------------------------------
  // calculate AC/DC values for current picture
  if( pcSlice->getPPS()->getUseWP() || pcSlice->getPPS()->getWPBiPred() )//若允许使用加权预测
  {
    xCalcACDCParamSlice(pcSlice);//计算当前原始图像的AC DC值
  }

  const Bool bWp_explicit = (pcSlice->getSliceType()==P_SLICE && pcSlice->getPPS()->getUseWP()) || (pcSlice->getSliceType()==B_SLICE && pcSlice->getPPS()->getWPBiPred());

  if ( bWp_explicit )//该slice使用加权预测
  {
    //------------------------------------------------------------------------------
    //  Weighted Prediction implemented at Slice level. SliceMode=2 is not supported yet.
    //------------------------------------------------------------------------------
    if ( pcSlice->getSliceMode()==FIXED_NUMBER_OF_BYTES || pcSlice->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES )//固定比特数的slice不支持加权预测
    {
      printf("Weighted Prediction is not supported with slice mode determined by max number of bins.\n"); exit(0);
    }

    xEstimateWPParamSlice( pcSlice );//计算并设置slice的参考图像的加权参数
    pcSlice->initWpScaling(pcSlice->getSPS());//用计算得到的参考图像加权参数初始化加权预测权重表

    // check WP on/off
    xCheckWPEnable( pcSlice );//检测是否使用加权预测
  }

#if ADAPTIVE_QP_SELECTION
  if( m_pcCfg->getUseAdaptQpSelect() && !(pcSlice->getDependentSliceSegmentFlag()))
  {
    // TODO: this won't work with dependent slices: they do not have their own QP. Check fix to mask clause execution with && !(pcSlice->getDependentSliceSegmentFlag())
    m_pcTrQuant->clearSliceARLCnt(); // TODO: this looks wrong for multiple slices - the results of all but the last slice will be cleared before they are used (all slices compressed, and then all slices encoded)
    if(pcSlice->getSliceType()!=I_SLICE)
    {
      Int qpBase = pcSlice->getSliceQpBase();
      pcSlice->setSliceQp(qpBase + m_pcTrQuant->getQpDelta(qpBase));
    }
  }
#endif



  // Adjust initial state if this is the start of a dependent slice.
  {
    const UInt      ctuRsAddr               = pcPic->getPicSym()->getCtuTsToRsAddrMap( startCtuTsAddr);
    const UInt      currentTileIdx          = pcPic->getPicSym()->getTileIdxMap(ctuRsAddr);//slice/ss首Ctu所在tile索引
    const TComTile *pCurrentTile            = pcPic->getPicSym()->getTComTile(currentTileIdx);//slice/ss首Ctu所在tile索引
    const UInt      firstCtuRsAddrOfTile    = pCurrentTile->getFirstCtuRsAddr();//该tile中第一个Ctu的RS地址
    if( pcSlice->getDependentSliceSegmentFlag() && ctuRsAddr != firstCtuRsAddrOfTile )//该slice为依赖ss且ss首Ctu不为tile的首Ctu
    {
      // This will only occur if dependent slice-segments (m_entropyCodingSyncContextState=true) are being used.
      if( pCurrentTile->getTileWidthInCtus() >= 2 || !m_pcCfg->getWaveFrontsynchro() )
      {
        m_pppcRDSbacCoder[0][CI_CURR_BEST]->loadContexts( &m_lastSliceSegmentEndContextState );//使用上个ss的上下文状态
      }
    }
  }

  // for every CTU in the slice segment (may terminate sooner if there is a byte limit on the slice-segment)

  for( UInt ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ++ctuTsAddr )//依次处理ss/slice中每一个Ctu
  {
    const UInt ctuRsAddr = pcPic->getPicSym()->getCtuTsToRsAddrMap(ctuTsAddr);
    // initialize CTU encoder
    TComDataCU* pCtu = pcPic->getCtu( ctuRsAddr );
    pCtu->initCtu( pcPic, ctuRsAddr );//初始化该Ctu

    // update CABAC state
    const UInt firstCtuRsAddrOfTile = pcPic->getPicSym()->getTComTile(pcPic->getPicSym()->getTileIdxMap(ctuRsAddr))->getFirstCtuRsAddr();//Ctu所在tile中第一个Ctu的RS地址
    const UInt tileXPosInCtus = firstCtuRsAddrOfTile % frameWidthInCtus;//该tile中首Ctu在图像中的X位置
    const UInt ctuXPosInCtus  = ctuRsAddr % frameWidthInCtus;//该Ctu在图像中的X位置
    
    if (ctuRsAddr == firstCtuRsAddrOfTile)//tile的首Ctu需resetEntropy
    {
      m_pppcRDSbacCoder[0][CI_CURR_BEST]->resetEntropy(pcSlice);
    }
    else if ( ctuXPosInCtus == tileXPosInCtus && m_pcCfg->getWaveFrontsynchro())//该Ctu为tile中一行Ctu的首Ctu且使用波前并行处理//该部分可详见<HECV book>中Wavefront Parallel Processing说明!!!!
    {
      // reset and then update contexts to the state at the end of the top-right CTU (if within current slice and tile).
      m_pppcRDSbacCoder[0][CI_CURR_BEST]->resetEntropy(pcSlice);
      // Sync if the Top-Right is available.
      TComDataCU *pCtuUp = pCtu->getCtuAbove();//当前Ctu上方Ctu
      if ( pCtuUp && ((ctuRsAddr%frameWidthInCtus+1) < frameWidthInCtus)  )//右上Ctu可获得
      {
        TComDataCU *pCtuTR = pcPic->getCtu( ctuRsAddr - frameWidthInCtus + 1 );//右上Ctu
        if ( pCtu->CUIsFromSameSliceAndTile(pCtuTR) )//若当前Ctu与右上Ctu在同一Slice和Tile
        {
          // Top-Right is available, we use it.
          m_pppcRDSbacCoder[0][CI_CURR_BEST]->loadContexts( &m_entropyCodingSyncContextState );//则该行的首Ctu的上下文状态用右上Ctu的上下文状态
        }
      }
    }

    // set go-on entropy coder (used for all trial encodings - the cu encoder and encoder search also have a copy of the same pointer)
    m_pcEntropyCoder->setEntropyCoder ( m_pcRDGoOnSbacCoder );
    m_pcEntropyCoder->setBitstream( &tempBitCounter );
    tempBitCounter.resetBits();
    m_pcRDGoOnSbacCoder->load( m_pppcRDSbacCoder[0][CI_CURR_BEST] ); // this copy is not strictly necessary here, but indicates that the GoOnSbacCoder
                                                                     // is reset to a known state before every decision process.

    ((TEncBinCABAC*)m_pcRDGoOnSbacCoder->getEncBinIf())->setBinCountingEnableFlag(true);

    Double oldLambda = m_pcRdCost->getLambda();
    if ( m_pcCfg->getUseRateCtrl() )//若使用码率控制
    {
      Int estQP        = pcSlice->getSliceQp();
      Double estLambda = -1.0;
      Double bpp       = -1.0;

      if ( ( pcPic->getSlice( 0 )->getSliceType() == I_SLICE && m_pcCfg->getForceIntraQP() ) || !m_pcCfg->getLCULevelRC() )//若帧内预测强制使用给定的QP值或不进行Ctu级的码率控制
      {
        estQP = pcSlice->getSliceQp();//则不需要调整QP值 直接使用sliceQp
      }
      else//需要根据码率调整QP值
      {
        bpp = m_pcRateCtrl->getRCPic()->getLCUTargetBpp(pcSlice->getSliceType());//得打mubiaoBpp值
        if ( pcPic->getSlice( 0 )->getSliceType() == I_SLICE)//I_SLICE
        {
          estLambda = m_pcRateCtrl->getRCPic()->getLCUEstLambdaAndQP(bpp, pcSlice->getSliceQp(), &estQP);//计算I slice的Qp值和estLambda值
        }
        else//帧间预测
        {
          estLambda = m_pcRateCtrl->getRCPic()->getLCUEstLambda( bpp );//计算P/Bslice的Lambda值
          estQP     = m_pcRateCtrl->getRCPic()->getLCUEstQP    ( estLambda, pcSlice->getSliceQp() );//计算P/Bslice的Qp值
        }

        estQP     = Clip3( -pcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, estQP );

        m_pcRdCost->setLambda(estLambda, pcSlice->getSPS()->getBitDepths());//设置Lambda

#if RDOQ_CHROMA_LAMBDA//为色度分量的率失真优化量化设置lambda值
        // set lambda for RDOQ
        const Double chromaLambda = estLambda / m_pcRdCost->getChromaWeight();
        const Double lambdaArray[MAX_NUM_COMPONENT] = { estLambda, chromaLambda, chromaLambda };
        m_pcTrQuant->setLambdas( lambdaArray );
#else
        m_pcTrQuant->setLambda( estLambda );
#endif
      }

      m_pcRateCtrl->setRCQP( estQP );//设置码率控制对应的Qp值
#if ADAPTIVE_QP_SELECTION
      pCtu->getSlice()->setSliceQpBase( estQP );
#endif
    }

    // run CTU trial encoder
    m_pcCuEncoder->compressCtu( pCtu );//压缩该Ctu


    // All CTU decisions have now been made. Restore entropy coder to an initial stage, ready to make a true encode,
    // which will result in the state of the contexts being correct. It will also count up the number of bits coded,
    // which is used if there is a limit of the number of bytes per slice-segment.

    m_pcEntropyCoder->setEntropyCoder ( m_pppcRDSbacCoder[0][CI_CURR_BEST] );
    m_pcEntropyCoder->setBitstream( &tempBitCounter );
    pRDSbacCoder->setBinCountingEnableFlag( true );////对经CABAC编码(包括常规编码和旁路编码)的开始bins计数
    m_pppcRDSbacCoder[0][CI_CURR_BEST]->resetBits();
    pRDSbacCoder->setBinsCoded( 0 );//初始化已编码的bins数为零

    // encode CTU and calculate the true bit counters.
    m_pcCuEncoder->encodeCtu( pCtu );//编码该Ctu


    pRDSbacCoder->setBinCountingEnableFlag( false );

    const Int numberOfWrittenBits = m_pcEntropyCoder->getNumberOfWrittenBits();//该Ctu编码的比特位

    // Calculate if this CTU puts us over slice bit size.
    // cannot terminate if current slice/slice-segment would be 0 Ctu in size,
    const UInt validEndOfSliceCtuTsAddr = ctuTsAddr + (ctuTsAddr == startCtuTsAddr ? 1 : 0);//当ctuTsAddr = startCtuTsAddr时 ss/slice有效末尾位置需要加1是因为slice/ss的大小不能为0个Ctu
    // Set slice end parameter
    if(pcSlice->getSliceMode()==FIXED_NUMBER_OF_BYTES && pcSlice->getSliceBits()+numberOfWrittenBits > (pcSlice->getSliceArgument()<<3))//若规定slice为固定比特数且从起始Ctu到该Ctu的总比特数超过规定的大小
    {
      pcSlice->setSliceSegmentCurEndCtuTsAddr(validEndOfSliceCtuTsAddr);
      pcSlice->setSliceCurEndCtuTsAddr(validEndOfSliceCtuTsAddr);//则以该Ctu作为slice/ss的末尾(slice包括ss)
      boundingCtuTsAddr=validEndOfSliceCtuTsAddr;//slice边界为该Ctu
    }
    else if((!bCompressEntireSlice) && pcSlice->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES && pcSlice->getSliceSegmentBits()+numberOfWrittenBits > (pcSlice->getSliceSegmentArgument()<<3))
    {//若不压缩整个Slice 规定ss为固定比特数且从起始Ctu到该Ctu的总比特数超过规定的大小(若压缩整个slice则标明ss的末尾Ctu无意义)
      pcSlice->setSliceSegmentCurEndCtuTsAddr(validEndOfSliceCtuTsAddr);//则以该Ctu作为ss的末尾
      boundingCtuTsAddr=validEndOfSliceCtuTsAddr;//ss边界为该Ctu
    }

    if (boundingCtuTsAddr <= ctuTsAddr)//该Ctu位于slice/ss的末尾
    {
      break;//跳出循环(该slice/ss的所有Ctu处理完)
    }

    pcSlice->setSliceBits( (UInt)(pcSlice->getSliceBits() + numberOfWrittenBits) );//设置slice编码的总比特数(准确说为slice起始Ctu到当前Ctu的编码比特数)
    pcSlice->setSliceSegmentBits(pcSlice->getSliceSegmentBits()+numberOfWrittenBits);//设置ss编码的总比特数(准确说为ss起始Ctu到当前Ctu的编码比特数)

    // Store probabilities of second CTU in line into buffer - used only if wavefront-parallel-processing is enabled.
    if ( ctuXPosInCtus == tileXPosInCtus+1 && m_pcCfg->getWaveFrontsynchro())//波前并行时保存一行Ctu中的第二个Ctu的上下文状态(用作下一行起始Ctu的初始概率状态)
    {
      m_entropyCodingSyncContextState.loadContexts(m_pppcRDSbacCoder[0][CI_CURR_BEST]);
    }


    if ( m_pcCfg->getUseRateCtrl() )//使用码率控制
    {
      Int actualQP        = g_RCInvalidQPValue;
      Double actualLambda = m_pcRdCost->getLambda();
      Int actualBits      = pCtu->getTotalBits();
      Int numberOfEffectivePixels    = 0;
      for ( Int idx = 0; idx < pcPic->getNumPartitionsInCtu(); idx++ )//遍历该Ctu中所有4*4小块
      {
        if ( pCtu->getPredictionMode( idx ) != NUMBER_OF_PREDICTION_MODES && ( !pCtu->isSkipped( idx ) ) )//是否存在有效像素数
        {
          numberOfEffectivePixels = numberOfEffectivePixels + 16;
          break;
        }
      }

      if ( numberOfEffectivePixels == 0 )//该Ctu不存在有效像素
      {
        actualQP = g_RCInvalidQPValue;//使用无效QP值
      }
      else
      {
        actualQP = pCtu->getQP( 0 );//否则使用该Ctu的Qp值
      }
      m_pcRdCost->setLambda(oldLambda, pcSlice->getSPS()->getBitDepths());//设置lambda值
      m_pcRateCtrl->getRCPic()->updateAfterCTU( m_pcRateCtrl->getRCPic()->getLCUCoded(), actualBits, actualQP, actualLambda,
                                                pCtu->getSlice()->getSliceType() == I_SLICE ? 0 : m_pcCfg->getLCULevelRC() );//更新码率控制参数
    }

    m_uiPicTotalBits += pCtu->getTotalBits();//该slice/ss的总比特数
    m_dPicRdCost     += pCtu->getTotalCost();//该slice/ss的总损耗
    m_uiPicDist      += pCtu->getTotalDistortion();//该slice/ss的总失真(准确说为从起始Ctu到slice/ss中当前Ctu)
  }

  // store context state at the end of this slice-segment, in case the next slice is a dependent slice and continues using the CABAC contexts.
  if( pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag() )//若为依赖ss 则该ss的上下文状态会被下个ss使用
  {
    m_lastSliceSegmentEndContextState.loadContexts( m_pppcRDSbacCoder[0][CI_CURR_BEST] );//ctx end of dep.slice//保存该ss的上下文状态
  }

  // stop use of temporary bit counter object.
  m_pppcRDSbacCoder[0][CI_CURR_BEST]->setBitstream(NULL);
  m_pcRDGoOnSbacCoder->setBitstream(NULL); // stop use of tempBitCounter.

  // TODO: optimise cabac_init during compress slice to improve multi-slice operation
  //if (pcSlice->getPPS()->getCabacInitPresentFlag() && !pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag())
  //{
  //  m_encCABACTableIdx = m_pcEntropyCoder->determineCabacInitIdx();
  //}
  //else
  //{
  //  m_encCABACTableIdx = pcSlice->getSliceType();
  //}
}

Void TEncSlice::encodeSlice   ( TComPic* pcPic, TComOutputBitstream* pcSubstreams, UInt &numBinsCoded )//编码一个SS
{
  TComSlice *const pcSlice           = pcPic->getSlice(getSliceIdx());

  const UInt startCtuTsAddr          = pcSlice->getSliceSegmentCurStartCtuTsAddr();
  const UInt boundingCtuTsAddr       = pcSlice->getSliceSegmentCurEndCtuTsAddr();//ss起始地址和结束地址

  const UInt frameWidthInCtus        = pcPic->getPicSym()->getFrameWidthInCtus();//一帧图像中一行Ctu的Ctu个数(图像的以Ctu为单位的宽)
  const Bool depSliceSegmentsEnabled = pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag();//是否允许使用ss
  const Bool wavefrontsEnabled       = pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag();//是否使用波前并行(WF)

  // initialise entropy coder for the slice
  m_pcSbacCoder->init( (TEncBinIf*)m_pcBinCABAC );
  m_pcEntropyCoder->setEntropyCoder ( m_pcSbacCoder );//设置m_pcEntropyCoder的编码器为m_pcSbacCoder
  m_pcEntropyCoder->resetEntropy    ( pcSlice );//设置该slice的初始CABAC状态

  numBinsCoded = 0;
  m_pcBinCABAC->setBinCountingEnableFlag( true );//对经CABAC编码(包括常规编码和旁路编码)的bins计数
  m_pcBinCABAC->setBinsCoded(0);//初始CABAC编码的bins为零

#if ENC_DEC_TRACE
  g_bJustDoIt = g_bEncDecTraceEnable;
#endif
  DTRACE_CABAC_VL( g_nSymbolCounter++ );
  DTRACE_CABAC_T( "\tPOC: " );
  DTRACE_CABAC_V( pcPic->getPOC() );
  DTRACE_CABAC_T( "\n" );
#if ENC_DEC_TRACE
  g_bJustDoIt = g_bEncDecTraceDisable;
#endif


  if (depSliceSegmentsEnabled)//若允许使用依赖的ss
  {
    // modify initial contexts with previous slice segment if this is a dependent slice.
    const UInt ctuRsAddr        = pcPic->getPicSym()->getCtuTsToRsAddrMap( startCtuTsAddr );
    const UInt currentTileIdx=pcPic->getPicSym()->getTileIdxMap(ctuRsAddr);
    const TComTile *pCurrentTile=pcPic->getPicSym()->getTComTile(currentTileIdx);
    const UInt firstCtuRsAddrOfTile = pCurrentTile->getFirstCtuRsAddr();

    if( pcSlice->getDependentSliceSegmentFlag() && ctuRsAddr != firstCtuRsAddrOfTile )//slice为ss且该ss首Ctu不为tile首Ctu 
    {
      if( pCurrentTile->getTileWidthInCtus() >= 2 || !wavefrontsEnabled )//波前并行时保证一行Ctu数大于等于2 因为波前并行该行的首Ctu的初始概率状态使用上行的第二个Ctu的上下文状态
      {
        m_pcSbacCoder->loadContexts(&m_lastSliceSegmentEndContextState);//则该ss的初始概率状态使用上个ss的上下文状态
      }
    }
  }

  // for every CTU in the slice segment...

  for( UInt ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ++ctuTsAddr )//依次处理ss中每个Ctu
  {
    const UInt ctuRsAddr = pcPic->getPicSym()->getCtuTsToRsAddrMap(ctuTsAddr);
    const TComTile &currentTile = *(pcPic->getPicSym()->getTComTile(pcPic->getPicSym()->getTileIdxMap(ctuRsAddr)));//该Ctu所在的tile
    const UInt firstCtuRsAddrOfTile = currentTile.getFirstCtuRsAddr();//tile的起始ctu(左上角Ctu)
    const UInt tileXPosInCtus       = firstCtuRsAddrOfTile % frameWidthInCtus;
    const UInt tileYPosInCtus       = firstCtuRsAddrOfTile / frameWidthInCtus;//tile(左上角Ctu)在图像中的Ctu位置
    const UInt ctuXPosInCtus        = ctuRsAddr % frameWidthInCtus;
    const UInt ctuYPosInCtus        = ctuRsAddr / frameWidthInCtus;//该Ctu在图像中的Ctu位置
    const UInt uiSubStrm=pcPic->getSubstreamForCtuAddr(ctuRsAddr, true, pcSlice);//Substream位置索引
    TComDataCU* pCtu = pcPic->getCtu( ctuRsAddr );//该Ctu

    m_pcEntropyCoder->setBitstream( &pcSubstreams[uiSubStrm] );

    // set up CABAC contexts' state for this CTU
    if (ctuRsAddr == firstCtuRsAddrOfTile)//该Ctu为tile中第一个Ctu 则重置该CTU的CABAC状态为slice的初始CABAC状态
    {
      if (ctuTsAddr != startCtuTsAddr) // if it is the first CTU, then the entropy coder has already been reset
      {//若为slice的第一个CTU则无需重置　因为CABAC初始状态已在函数开头重置过
        m_pcEntropyCoder->resetEntropy(pcSlice);
      }
    }
    else if (ctuXPosInCtus == tileXPosInCtus && wavefrontsEnabled)//该Ctu为tile中一行Ctu的首Ctu且使用波前并行处理
    {
      // Synchronize cabac probabilities with upper-right CTU if it's available and at the start of a line.
      if (ctuTsAddr != startCtuTsAddr) // if it is the first CTU, then the entropy coder has already been reset
      {
        m_pcEntropyCoder->resetEntropy(pcSlice);//resetEntropy
      }
      TComDataCU *pCtuUp = pCtu->getCtuAbove();//该Ctu上方Ctu
      if ( pCtuUp && ((ctuRsAddr%frameWidthInCtus+1) < frameWidthInCtus)  )//右上Ctu可获得
      {
        TComDataCU *pCtuTR = pcPic->getCtu( ctuRsAddr - frameWidthInCtus + 1 );//右上Ctu
        if ( pCtu->CUIsFromSameSliceAndTile(pCtuTR) )//若当前Ctu与右上Ctu在同一Slice和Tile
        {
          // Top-right is available, so use it.
          m_pcSbacCoder->loadContexts( &m_entropyCodingSyncContextState );//则该行的首Ctu的上下文状态用右上Ctu的上下文状态
        }
      }
    }


    if ( pcSlice->getSPS()->getUseSAO() )//若使用SAO
    {
      Bool bIsSAOSliceEnabled = false;
      Bool sliceEnabled[MAX_NUM_COMPONENT];
      for(Int comp=0; comp < MAX_NUM_COMPONENT; comp++)
      {
        ComponentID compId=ComponentID(comp);
        sliceEnabled[compId] = pcSlice->getSaoEnabledFlag(toChannelType(compId)) && (comp < pcPic->getNumberValidComponents());//该slice的该分量是否使用SAO
        if (sliceEnabled[compId])
        {
          bIsSAOSliceEnabled=true;//若存在分量使用SAO 则该slice SAO标志置为真
        }
      }
      if (bIsSAOSliceEnabled)//若该slice使用SAO
      {
        SAOBlkParam& saoblkParam = (pcPic->getPicSym()->getSAOBlkParam())[ctuRsAddr];

        Bool leftMergeAvail = false;
        Bool aboveMergeAvail= false;
        //merge left condition
        Int rx = (ctuRsAddr % frameWidthInCtus);
        if(rx > 0)//该Ctu左侧存在Ctu
        {
          leftMergeAvail = pcPic->getSAOMergeAvailability(ctuRsAddr, ctuRsAddr-1);//在同一个tile 同一个slice才能获得
        }

        //merge up condition
        Int ry = (ctuRsAddr / frameWidthInCtus);
        if(ry > 0)//该Ctu上方存在Ctu
        {
          aboveMergeAvail = pcPic->getSAOMergeAvailability(ctuRsAddr, ctuRsAddr-frameWidthInCtus);//是否在同一个tile 同一个slice才能获得
        }

        m_pcEntropyCoder->encodeSAOBlkParam(saoblkParam, pcPic->getPicSym()->getSPS().getBitDepths(), sliceEnabled, leftMergeAvail, aboveMergeAvail);//编码该Ctu的SAO参数
      }
    }

#if ENC_DEC_TRACE
    g_bJustDoIt = g_bEncDecTraceEnable;
#endif
      m_pcCuEncoder->encodeCtu( pCtu );//编码该Ctu
#if ENC_DEC_TRACE
    g_bJustDoIt = g_bEncDecTraceDisable;
#endif

    //Store probabilities of second CTU in line into buffer
    if ( ctuXPosInCtus == tileXPosInCtus+1 && wavefrontsEnabled)//波前并行时保存一行Ctu中的第二个Ctu的上下文状态(用作下一行起始Ctu的初始概率状态)
    {
      m_entropyCodingSyncContextState.loadContexts( m_pcSbacCoder );
    }

    // terminate the sub-stream, if required (end of slice-segment, end of tile, end of wavefront-CTU-row):
    if (ctuTsAddr+1 == boundingCtuTsAddr ||  //ss的末尾 end of slice-segment
         (  ctuXPosInCtus + 1 == tileXPosInCtus + currentTile.getTileWidthInCtus() &&//tile的末尾 end of tile
          ( ctuYPosInCtus + 1 == tileYPosInCtus + currentTile.getTileHeightInCtus() || wavefrontsEnabled)//波前并行时 Ctu行的末尾 wavefront-CTU-row
         )
       )//终止sub-stream
    {
      m_pcEntropyCoder->encodeTerminatingBit(1);//编码终止比特位1(表明不在编码下个Ctu)
      m_pcEntropyCoder->encodeSliceFinish();//编码slice完成
      // Byte-alignment in slice_data() when new tile
      pcSubstreams[uiSubStrm].writeByteAlignment();

      // write sub-stream size
      if (ctuTsAddr+1 != boundingCtuTsAddr)
      {
        pcSlice->addSubstreamSize( (pcSubstreams[uiSubStrm].getNumberOfWrittenBits() >> 3) + pcSubstreams[uiSubStrm].countStartCodeEmulations() );// add substream to the end of the current bitstream
      }
    }
  } // CTU-loop

  if( depSliceSegmentsEnabled )//若允许使用ss
  {
    m_lastSliceSegmentEndContextState.loadContexts( m_pcSbacCoder );//ctx end of dep.slice//保存该ss的概率状态供下个ss使用
  }

#if ADAPTIVE_QP_SELECTION
  if( m_pcCfg->getUseAdaptQpSelect() )
  {
    m_pcTrQuant->storeSliceQpNext(pcSlice); // TODO: this will only be storing the adaptive QP state of the very last slice-segment that is not dependent in the frame... Perhaps this should be moved to the compress slice loop.
  }
#endif

  if (pcSlice->getPPS()->getCabacInitPresentFlag() && !pcSlice->getPPS()->getDependentSliceSegmentsEnabledFlag())
  {//
    m_encCABACTableIdx = m_pcEntropyCoder->determineCabacInitIdx(pcSlice);//为下一个slice选择较优的cabac初始化索引入口 optimise cabac_init during compress slice to improve multi-slice operation
  }                                                                       //This index is used for the next P/B slice when cabac_init_present_flag is true
  else
  {
    m_encCABACTableIdx = pcSlice->getSliceType();
  }
  
  numBinsCoded = m_pcBinCABAC->getBinsCoded();//已编码的bins
}

Void TEncSlice::calculateBoundingCtuTsAddrForSlice(UInt &startCtuTSAddrSlice, UInt &boundingCtuTSAddrSlice, Bool &haveReachedTileBoundary,
                                                   TComPic* pcPic, const Int sliceMode, const Int sliceArgument)
{
  TComSlice* pcSlice = pcPic->getSlice(getSliceIdx());
  const UInt numberOfCtusInFrame = pcPic->getNumberOfCtusInFrame();
  const TComPPS &pps=*(pcSlice->getPPS());
  boundingCtuTSAddrSlice=0;
  haveReachedTileBoundary=false;

  switch (sliceMode)//slice的划分模式
  {
    case FIXED_NUMBER_OF_CTU://固定Ctu数模式
      {
        UInt ctuAddrIncrement    = sliceArgument;//一个slice/ss中的Ctu数
        boundingCtuTSAddrSlice  = ((startCtuTSAddrSlice + ctuAddrIncrement) < numberOfCtusInFrame) ? (startCtuTSAddrSlice + ctuAddrIncrement) : numberOfCtusInFrame;//slice/ss边界Ctu为起始Ctu经过固定Ctu后的Ctu 若超出图像总Ctu数 则边界Ctu为图像中最后一个Ctu
      }
      break;
    case FIXED_NUMBER_OF_BYTES://固定比特数模式
      boundingCtuTSAddrSlice  = numberOfCtusInFrame; // This will be adjusted later if required.
      break;
    case FIXED_NUMBER_OF_TILES://固定tile模式
      {
        const UInt tileIdx        = pcPic->getPicSym()->getTileIdxMap( pcPic->getPicSym()->getCtuTsToRsAddrMap(startCtuTSAddrSlice) );//起始Ctu所在tile索引
        const UInt tileTotalCount = (pcPic->getPicSym()->getNumTileColumnsMinus1()+1) * (pcPic->getPicSym()->getNumTileRowsMinus1()+1);//图像中总tile数
        UInt ctuAddrIncrement   = 0;

        for(UInt tileIdxIncrement = 0; tileIdxIncrement < sliceArgument; tileIdxIncrement++)//依次加上每个tile中的Ctus数
        {
          if((tileIdx + tileIdxIncrement) < tileTotalCount)
          {
            UInt tileWidthInCtus   = pcPic->getPicSym()->getTComTile(tileIdx + tileIdxIncrement)->getTileWidthInCtus();
            UInt tileHeightInCtus  = pcPic->getPicSym()->getTComTile(tileIdx + tileIdxIncrement)->getTileHeightInCtus();
            ctuAddrIncrement    += (tileWidthInCtus * tileHeightInCtus);//总的Ctu增加数加上每个tile中Ctus数
          }
        }

        boundingCtuTSAddrSlice  = ((startCtuTSAddrSlice + ctuAddrIncrement) < numberOfCtusInFrame) ? (startCtuTSAddrSlice + ctuAddrIncrement) : numberOfCtusInFrame;//根据slice起始Ctu得到固定tile数的slice边界Ctu位置(不得超出图像)
      }
      break;
    default:
      boundingCtuTSAddrSlice    = numberOfCtusInFrame;//以上3中模式均不是 则整帧图像可看作一个slice
      break;
  }

  // Adjust for tiles and wavefronts.
  const Bool wavefrontsAreEnabled = pps.getEntropyCodingSyncEnabledFlag();
  //In order to maintain bitstream conformance for the CTU row partitioning approach
  if ((sliceMode == FIXED_NUMBER_OF_CTU || sliceMode == FIXED_NUMBER_OF_BYTES) &&
      (pps.getNumTileRowsMinus1() > 0 || pps.getNumTileColumnsMinus1() > 0))//固定tile模式下
  {
    const UInt ctuRSAddr                  = pcPic->getPicSym()->getCtuTsToRsAddrMap(startCtuTSAddrSlice);//slice/ss起始Ctu地址
    const UInt startTileIdx               = pcPic->getPicSym()->getTileIdxMap(ctuRSAddr);////slice/ss起始Ctu所在tile的索引

    const TComTile *pStartingTile         = pcPic->getPicSym()->getTComTile(startTileIdx);//slice/ss起始Ctu所在tile
    const UInt tileStartTsAddr            = pcPic->getPicSym()->getCtuRsToTsAddrMap(pStartingTile->getFirstCtuRsAddr());//slice/ss起始Ctu所在tile的首Ctu的Ts地址
    const UInt tileStartWidth             = pStartingTile->getTileWidthInCtus();
    const UInt tileStartHeight            = pStartingTile->getTileHeightInCtus();//slice/ss起始Ctu所在tile的宽和高(以Ctu为单位)
    const UInt tileLastTsAddr_excl        = tileStartTsAddr + tileStartWidth*tileStartHeight;//slice/ss起始Ctu所在tile的最后一个Ctu的Ts地址
    const UInt tileBoundingCtuTsAddrSlice = tileLastTsAddr_excl;

    const UInt ctuColumnOfStartingTile    = ((startCtuTSAddrSlice-tileStartTsAddr)%tileStartWidth);//slice起始Ctu在tile中的Ctu列
    if (wavefrontsAreEnabled && ctuColumnOfStartingTile!=0)//波前并行时 若slice起始Ctu不为Ctu行的首Ctu 则该slice必须结束在同一行Ctu
    {
      // WPP: if a slice does not start at the beginning of a CTB row, it must end within the same CTB row
      const UInt numberOfCTUsToEndOfRow            = tileStartWidth - ctuColumnOfStartingTile;//该slice的起始Ctu距离该行Ctu末尾Ctu的Ctu数(同一个tile中)
      const UInt wavefrontTileBoundingCtuAddrSlice = startCtuTSAddrSlice + numberOfCTUsToEndOfRow;//该slice允许的最大Ctu边界位置
      if (wavefrontTileBoundingCtuAddrSlice < boundingCtuTSAddrSlice)//若该slice边界Ctu超出其起始Ctu所在Ctu行(同一个tile中)
      {
        boundingCtuTSAddrSlice = wavefrontTileBoundingCtuAddrSlice;
      }
    }//限定该slice结束在同一行Ctu

    if (tileBoundingCtuTsAddrSlice < boundingCtuTSAddrSlice)
    {
      boundingCtuTSAddrSlice = tileBoundingCtuTsAddrSlice;
      haveReachedTileBoundary = true;
    }
  }
  else if ((sliceMode == FIXED_NUMBER_OF_CTU || sliceMode == FIXED_NUMBER_OF_BYTES) && wavefrontsAreEnabled && ((startCtuTSAddrSlice % pcPic->getFrameWidthInCtus()) != 0))//固定Ctu模式下
  {
    // Adjust for wavefronts (no tiles).
    // WPP: if a slice does not start at the beginning of a CTB row, it must end within the same CTB row
    boundingCtuTSAddrSlice = min(boundingCtuTSAddrSlice, startCtuTSAddrSlice - (startCtuTSAddrSlice % pcPic->getFrameWidthInCtus()) + (pcPic->getFrameWidthInCtus()));
  }//同理该slice边界Ctu超出其起始Ctu所在Ctu行
}

/** Determines the starting and bounding CTU address of current slice / dependent slice
 * \param [out] startCtuTsAddr
 * \param [out] boundingCtuTsAddr
 * \param [in]  pcPic

 * Updates startCtuTsAddr, boundingCtuTsAddr with appropriate CTU address
 */
Void TEncSlice::xDetermineStartAndBoundingCtuTsAddr  ( UInt& startCtuTsAddr, UInt& boundingCtuTsAddr, TComPic* pcPic )
{
  TComSlice* pcSlice                 = pcPic->getSlice(getSliceIdx());

  // Non-dependent slice
  UInt startCtuTsAddrSlice           = pcSlice->getSliceCurStartCtuTsAddr();
  Bool haveReachedTileBoundarySlice  = false;
  UInt boundingCtuTsAddrSlice;
  calculateBoundingCtuTsAddrForSlice(startCtuTsAddrSlice, boundingCtuTsAddrSlice, haveReachedTileBoundarySlice, pcPic,
                                     m_pcCfg->getSliceMode(), m_pcCfg->getSliceArgument());//计算slice的边界Ctu位置
  pcSlice->setSliceCurEndCtuTsAddr(   boundingCtuTsAddrSlice );
  pcSlice->setSliceCurStartCtuTsAddr( startCtuTsAddrSlice    );//设置slice的起始和边界Ctu位置

  // Dependent slice
  UInt startCtuTsAddrSliceSegment          = pcSlice->getSliceSegmentCurStartCtuTsAddr();
  Bool haveReachedTileBoundarySliceSegment = false;
  UInt boundingCtuTsAddrSliceSegment;
  calculateBoundingCtuTsAddrForSlice(startCtuTsAddrSliceSegment, boundingCtuTsAddrSliceSegment, haveReachedTileBoundarySliceSegment, pcPic,
                                     m_pcCfg->getSliceSegmentMode(), m_pcCfg->getSliceSegmentArgument());//计算ss的边界Ctu位置
  if (boundingCtuTsAddrSliceSegment>boundingCtuTsAddrSlice)
  {
    boundingCtuTsAddrSliceSegment = boundingCtuTsAddrSlice;//ss边界Ctu位置不得超出slic边界Ctu位置(ss属于slice)
  }
  pcSlice->setSliceSegmentCurEndCtuTsAddr( boundingCtuTsAddrSliceSegment );
  pcSlice->setSliceSegmentCurStartCtuTsAddr(startCtuTsAddrSliceSegment);//设置ss的起始和边界Ctu位置

  // Make a joint decision based on reconstruction and dependent slice bounds
  startCtuTsAddr    = max(startCtuTsAddrSlice   , startCtuTsAddrSliceSegment   );
  boundingCtuTsAddr = boundingCtuTsAddrSliceSegment;//dependent ss bounds(之所以需要这样设置是因为EncGOP中对图像的压缩是以SS为单位的 见EncGOP.cpp中1406-1446行代码)
}

Double TEncSlice::xGetQPValueAccordingToLambda ( Double lambda )//根据lambda值得到QP值
{
  return 4.2005*log(lambda) + 13.7122;//由实验统计得到 用于码率控制
}

//! \}
