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

/** \file     TEncGOP.cpp
    \brief    GOP encoder class
*/

#include <list>
#include <algorithm>
#include <functional>

#include "TEncTop.h"
#include "TEncGOP.h"
#include "TEncAnalyze.h"
#include "libmd5/MD5.h"
#include "TLibCommon/SEI.h"
#include "TLibCommon/NAL.h"
#include "NALwrite.h"
#include <time.h>
#include <math.h>

#include <deque>
using namespace std;

//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================
Int getLSB(Int poc, Int maxLSB)//计算LSB
{
  if (poc >= 0)
  {
    return poc % maxLSB;
  }
  else
  {
    return (maxLSB - ((-poc) % maxLSB)) % maxLSB;
  }
}

TEncGOP::TEncGOP()
{
  m_iLastIDR            = 0;
  m_iGopSize            = 0;
  m_iNumPicCoded        = 0; //Niko
  m_bFirst              = true;
  m_iLastRecoveryPicPOC = 0;

  m_pcCfg               = NULL;
  m_pcSliceEncoder      = NULL;
  m_pcListPic           = NULL;

  m_pcEntropyCoder      = NULL;
  m_pcCavlcCoder        = NULL;
  m_pcSbacCoder         = NULL;
  m_pcBinCABAC          = NULL;

  m_bSeqFirst           = true;

  m_bRefreshPending     = 0;
  m_pocCRA            = 0;
  m_numLongTermRefPicSPS = 0;
  ::memset(m_ltRefPicPocLsbSps, 0, sizeof(m_ltRefPicPocLsbSps));
  ::memset(m_ltRefPicUsedByCurrPicFlag, 0, sizeof(m_ltRefPicUsedByCurrPicFlag));
  m_lastBPSEI         = 0;
  m_bufferingPeriodSEIPresentInAU = false;
  m_associatedIRAPType = NAL_UNIT_CODED_SLICE_IDR_N_LP;
  m_associatedIRAPPOC  = 0;
  return;
}

TEncGOP::~TEncGOP()
{
}

/** Create list to contain pointers to CTU start addresses of slice.
 */
Void  TEncGOP::create()
{
  m_bLongtermTestPictureHasBeenCoded = 0;
  m_bLongtermTestPictureHasBeenCoded2 = 0;
}

Void  TEncGOP::destroy()
{
}

Void TEncGOP::init ( TEncTop* pcTEncTop )
{
  m_pcEncTop     = pcTEncTop;
  m_pcCfg                = pcTEncTop;
  m_seiEncoder.init(m_pcCfg, pcTEncTop, this);
  m_pcSliceEncoder       = pcTEncTop->getSliceEncoder();
  m_pcListPic            = pcTEncTop->getListPic();

  m_pcEntropyCoder       = pcTEncTop->getEntropyCoder();
  m_pcCavlcCoder         = pcTEncTop->getCavlcCoder();
  m_pcSbacCoder          = pcTEncTop->getSbacCoder();
  m_pcBinCABAC           = pcTEncTop->getBinCABAC();
  m_pcLoopFilter         = pcTEncTop->getLoopFilter();

  m_pcSAO                = pcTEncTop->getSAO();
  m_pcRateCtrl           = pcTEncTop->getRateCtrl();
  m_lastBPSEI          = 0;
  m_totalCoded         = 0;

}

Int TEncGOP::xWriteVPS (AccessUnit &accessUnit, const TComVPS *vps)//将VPS参数编码比特流写入accessUnit 并返回其比特流的大小
{
  OutputNALUnit nalu(NAL_UNIT_VPS);
  m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);//nalu.m_Bitstream将保存vps的编码比特流
  m_pcEntropyCoder->encodeVPS(vps);//编码VPS参数
  accessUnit.push_back(new NALUnitEBSP(nalu));//将保存有vps编码比特流的nalu添加至accessUnit尾部
  return (Int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;//返回VPS编码比特流的大小
}

Int TEncGOP::xWriteSPS (AccessUnit &accessUnit, const TComSPS *sps)//将SPS参数编码比特流写入accessUnit 并返回其比特流的大小
{
  OutputNALUnit nalu(NAL_UNIT_SPS);
  m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
  m_pcEntropyCoder->encodeSPS(sps);
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (Int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;

}

Int TEncGOP::xWritePPS (AccessUnit &accessUnit, const TComPPS *pps)//将PPS参数编码比特流写入accessUnit 并返回其比特流的大小
{
  OutputNALUnit nalu(NAL_UNIT_PPS);
  m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
  m_pcEntropyCoder->encodePPS(pps);
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (Int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;
}


Int TEncGOP::xWriteParameterSets (AccessUnit &accessUnit, TComSlice *slice)//将VPS SPS PPS参数集编码比特流分别写入accessUnit 并返回参数集的总比特数数
{
  Int actualTotalBits = 0;

  actualTotalBits += xWriteVPS(accessUnit, m_pcEncTop->getVPS());
  actualTotalBits += xWriteSPS(accessUnit, slice->getSPS());
  actualTotalBits += xWritePPS(accessUnit, slice->getPPS());

  return actualTotalBits;
}

// write SEI list into one NAL unit and add it to the Access unit at auPos
Void TEncGOP::xWriteSEI (NalUnitType naluType, SEIMessages& seiMessages, AccessUnit &accessUnit, AccessUnit::iterator &auPos, Int temporalId, const TComSPS *sps)//将给定的SEI信息列表中的信息写入一个NAL unit 并将其插入到AccessUnit中给定的位置
{
  // don't do anything, if we get an empty list
  if (seiMessages.empty())//SEI列表为空直接范围
  {
    return;
  }
  OutputNALUnit nalu(naluType, temporalId);
  m_seiWriter.writeSEImessages(nalu.m_Bitstream, seiMessages, sps, false);//将SEI信息写入nalu
  auPos = accessUnit.insert(auPos, new NALUnitEBSP(nalu));//插入至accessUnit给定位置
  auPos++;
}

Void TEncGOP::xWriteSEISeparately (NalUnitType naluType, SEIMessages& seiMessages, AccessUnit &accessUnit, AccessUnit::iterator &auPos, Int temporalId, const TComSPS *sps)
{//基本同xWriteSEI 不同在于该方法将SEI信息列表中的每个SEI信息分别写入一个NAL unit中(即每个SEI对应一个NAL unit)并将每个NAL unit插入至给定的位置
  // don't do anything, if we get an empty list
  if (seiMessages.empty())
  {
    return;
  }
  for (SEIMessages::const_iterator sei = seiMessages.begin(); sei!=seiMessages.end(); sei++ )//分别处理SEI信息列表中的每个SEI信息
  {
    SEIMessages tmpMessages;
    tmpMessages.push_back(*sei);
    OutputNALUnit nalu(naluType, temporalId);
    m_seiWriter.writeSEImessages(nalu.m_Bitstream, tmpMessages, sps, false);
    auPos = accessUnit.insert(auPos, new NALUnitEBSP(nalu));
    auPos++;
  }
}

Void TEncGOP::xClearSEIs(SEIMessages& seiMessages, Bool deleteMessages)
{
  if (deleteMessages)
  {
    deleteSEIs(seiMessages);
  }
  else
  {
    seiMessages.clear();
  }
}

// write SEI messages as separate NAL units ordered
Void TEncGOP::xWriteLeadingSEIOrdered (SEIMessages& seiMessages, SEIMessages& duInfoSeiMessages, AccessUnit &accessUnit, Int temporalId, const TComSPS *sps, Bool testWrite)//按顺序写入不同的SEI信息
{
  AccessUnit::iterator itNalu = accessUnit.begin();

  while ( (itNalu!=accessUnit.end())&&
    ( (*itNalu)->m_nalUnitType==NAL_UNIT_ACCESS_UNIT_DELIMITER 
    || (*itNalu)->m_nalUnitType==NAL_UNIT_VPS
    || (*itNalu)->m_nalUnitType==NAL_UNIT_SPS
    || (*itNalu)->m_nalUnitType==NAL_UNIT_PPS
    ))
  {
    itNalu++;
  }//找到SEI信息写入accessUnit的位置(在VPS/SPS/PPS参数集之后)

  SEIMessages localMessages = seiMessages;
  SEIMessages currentMessages;
  
#if ENC_DEC_TRACE
  g_HLSTraceEnable = !testWrite;
#endif
  // The case that a specific SEI is not present is handled in xWriteSEI (empty list)
  //该部分SEI message参数可参见<HEVC boos>中HEVC SEI messages部分或<ITU-T Rec H.265>中Persistence scope of SEI messages说明!!!!
  // Active parameter sets SEI must always be the first SEI
  currentMessages = extractSeisByType(localMessages, SEI::ACTIVE_PARAMETER_SETS);//得到SEI信息列表中特定的SEI信息(ACTIVE_PARAMETER_SETS)
  assert (currentMessages.size() <= 1);
  xWriteSEI(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId, sps);//该SEI信息写入accessUnit
  xClearSEIs(currentMessages, !testWrite);//清除currentMessages(下个SEI使用)
  
  // Buffering period SEI must always be following active parameter sets
  currentMessages = extractSeisByType(localMessages, SEI::BUFFERING_PERIOD);
  assert (currentMessages.size() <= 1);
  xWriteSEI(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId, sps);//将SEI的BUFFERING_PERIOD信息写入accessUnit
  xClearSEIs(currentMessages, !testWrite);

  // Picture timing SEI must always be following buffering period
  currentMessages = extractSeisByType(localMessages, SEI::PICTURE_TIMING);
  assert (currentMessages.size() <= 1);
  xWriteSEI(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId, sps);//将SEI的PICTURE_TIMING信息写入accessUnit
  xClearSEIs(currentMessages, !testWrite);

  // Decoding unit info SEI must always be following picture timing
  if (!duInfoSeiMessages.empty())
  {
    currentMessages.push_back(duInfoSeiMessages.front());
    if (!testWrite)
    {
      duInfoSeiMessages.pop_front();
    }
    xWriteSEI(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId, sps);//将Decoding unit info写入accessUnit
    xClearSEIs(currentMessages, !testWrite);
  }

  // Scalable nesting SEI must always be the following DU info
  currentMessages = extractSeisByType(localMessages, SEI::SCALABLE_NESTING);
  xWriteSEISeparately(NAL_UNIT_PREFIX_SEI, currentMessages, accessUnit, itNalu, temporalId, sps);//将SEI的SCALABLE_NESTING信息写入accessUnit
  xClearSEIs(currentMessages, !testWrite);

  // And finally everything else one by one
  xWriteSEISeparately(NAL_UNIT_PREFIX_SEI, localMessages, accessUnit, itNalu, temporalId, sps);//将SEI信息列表中其他还未写入accessUnit的SEI信息(一个接一个)分别写入accessUnit
  xClearSEIs(localMessages, !testWrite);

  if (!testWrite)
  {
    seiMessages.clear();
  }
}


Void TEncGOP::xWriteLeadingSEIMessages (SEIMessages& seiMessages, SEIMessages& duInfoSeiMessages, AccessUnit &accessUnit, Int temporalId, const TComSPS *sps, std::deque<DUData> &duData)
{
  AccessUnit testAU;
  SEIMessages picTimingSEIs = getSeisByType(seiMessages, SEI::PICTURE_TIMING);
  assert (picTimingSEIs.size() < 2);
  SEIPictureTiming * picTiming = picTimingSEIs.empty() ? NULL : (SEIPictureTiming*) picTimingSEIs.front();

  // test writing
  xWriteLeadingSEIOrdered(seiMessages, duInfoSeiMessages, testAU, temporalId, sps, true);
  // update Timing and DU info SEI
  xUpdateDuData(testAU, duData);
  xUpdateTimingSEI(picTiming, duData, sps);
  xUpdateDuInfoSEI(duInfoSeiMessages, picTiming);
  // actual writing
  xWriteLeadingSEIOrdered(seiMessages, duInfoSeiMessages, accessUnit, temporalId, sps, false);

  // testAU will automatically be cleaned up when losing scope
}

Void TEncGOP::xWriteTrailingSEIMessages (SEIMessages& seiMessages, AccessUnit &accessUnit, Int temporalId, const TComSPS *sps)//将SEI后缀信息分别写入accessUnit末尾
{
  // Note: using accessUnit.end() works only as long as this function is called after slice coding and before EOS/EOB NAL units
  AccessUnit::iterator pos = accessUnit.end();
  xWriteSEISeparately(NAL_UNIT_SUFFIX_SEI, seiMessages, accessUnit, pos, temporalId, sps);
  deleteSEIs(seiMessages);
}

Void TEncGOP::xWriteDuSEIMessages (SEIMessages& duInfoSeiMessages, AccessUnit &accessUnit, Int temporalId, const TComSPS *sps, std::deque<DUData> &duData)//写入每个Du对应的duInfo
{
  const TComHRD *hrd = sps->getVuiParameters()->getHrdParameters();

  if( m_pcCfg->getDecodingUnitInfoSEIEnabled() && hrd->getSubPicCpbParamsPresentFlag() )
  {
    Int naluIdx = 0;
    AccessUnit::iterator nalu = accessUnit.begin();//accessUnit的起始位置

    // skip over first DU, we have a DU info SEI there already
    while (naluIdx < duData[0].accumNalsDU && nalu!=accessUnit.end())//跳过第一个DU 因为已经写入一个 DU info SEI
    {
      naluIdx++;
      nalu++;
    }

    SEIMessages::iterator duSEI = duInfoSeiMessages.begin();
    // loop over remaining DUs
    for (Int duIdx = 1; duIdx < duData.size(); duIdx++)//遍历所有的DU
    {
      if (duSEI == duInfoSeiMessages.end())
      {
        // if the number of generated SEIs matches the number of DUs, this should not happen
        assert (false);
        return;
      }
      // write the next SEI
      SEIMessages tmpSEI;
      tmpSEI.push_back(*duSEI);
      xWriteSEI(NAL_UNIT_PREFIX_SEI, tmpSEI, accessUnit, nalu, temporalId, sps);//将该duSEI写入accessUnit
      // nalu points to the position after the SEI, so we have to increase the index as well
      naluIdx++;
      while ((naluIdx < duData[duIdx].accumNalsDU) && nalu!=accessUnit.end())
      {
        naluIdx++;
        nalu++;
      }//下个duInfo写入accessUnit的位置 (保证duInfo在对应DU的后面)
      duSEI++;//下个DU对应的duInfo
    }
  }
  deleteSEIs(duInfoSeiMessages);
}

////该部分SEI message参数可参见<HEVC boos>中HEVC SEI messages部分或<ITU-T Rec H.265>中Persistence scope of SEI messages说明!!!!
Void TEncGOP::xCreateIRAPLeadingSEIMessages (SEIMessages& seiMessages, const TComSPS *sps, const TComPPS *pps)//创建前缀SEI信息列表
{
  OutputNALUnit nalu(NAL_UNIT_PREFIX_SEI);

  if(m_pcCfg->getActiveParameterSetsSEIEnabled())
  {
    SEIActiveParameterSets *sei = new SEIActiveParameterSets;
    m_seiEncoder.initSEIActiveParameterSets (sei, m_pcCfg->getVPS(), sps);
    seiMessages.push_back(sei);
  }

  if(m_pcCfg->getFramePackingArrangementSEIEnabled())
  {
    SEIFramePacking *sei = new SEIFramePacking;
    m_seiEncoder.initSEIFramePacking (sei, m_iNumPicCoded);
    seiMessages.push_back(sei);
  }

  if(m_pcCfg->getSegmentedRectFramePackingArrangementSEIEnabled())
  {
    SEISegmentedRectFramePacking *sei = new SEISegmentedRectFramePacking;
    m_seiEncoder.initSEISegmentedRectFramePacking(sei);
    seiMessages.push_back(sei);
  }

  if (m_pcCfg->getDisplayOrientationSEIAngle())
  {
    SEIDisplayOrientation *sei = new SEIDisplayOrientation;
    m_seiEncoder.initSEIDisplayOrientation(sei);
    seiMessages.push_back(sei);
  }

  if(m_pcCfg->getToneMappingInfoSEIEnabled())
  {
    SEIToneMappingInfo *sei = new SEIToneMappingInfo;
    m_seiEncoder.initSEIToneMappingInfo (sei);
    seiMessages.push_back(sei);
  }

  if(m_pcCfg->getTMCTSSEIEnabled())
  {
    SEITempMotionConstrainedTileSets *sei = new SEITempMotionConstrainedTileSets;
    m_seiEncoder.initSEITempMotionConstrainedTileSets(sei, pps);
    seiMessages.push_back(sei);
  }

  if(m_pcCfg->getTimeCodeSEIEnabled())
  {
    SEITimeCode *seiTimeCode = new SEITimeCode;
    m_seiEncoder.initSEITimeCode(seiTimeCode);
    seiMessages.push_back(seiTimeCode);
  }

  if(m_pcCfg->getKneeSEIEnabled())
  {
    SEIKneeFunctionInfo *sei = new SEIKneeFunctionInfo;
    m_seiEncoder.initSEIKneeFunctionInfo(sei);
    seiMessages.push_back(sei);
  }
    
  if(m_pcCfg->getMasteringDisplaySEI().colourVolumeSEIEnabled)
  {
    const TComSEIMasteringDisplay &seiCfg=m_pcCfg->getMasteringDisplaySEI();
    SEIMasteringDisplayColourVolume *sei = new SEIMasteringDisplayColourVolume;
    sei->values = seiCfg;
    seiMessages.push_back(sei);
  }
}

Void TEncGOP::xCreatePerPictureSEIMessages (Int picInGOP, SEIMessages& seiMessages, SEIMessages& nestedSeiMessages, TComSlice *slice)//创建与每帧图像相关的前缀SEI信息
{
  if( ( m_pcCfg->getBufferingPeriodSEIEnabled() ) && ( slice->getSliceType() == I_SLICE ) &&
    ( slice->getSPS()->getVuiParametersPresentFlag() ) &&
    ( ( slice->getSPS()->getVuiParameters()->getHrdParameters()->getNalHrdParametersPresentFlag() )
    || ( slice->getSPS()->getVuiParameters()->getHrdParameters()->getVclHrdParametersPresentFlag() ) ) )
  {
    SEIBufferingPeriod *bufferingPeriodSEI = new SEIBufferingPeriod();
    m_seiEncoder.initSEIBufferingPeriod(bufferingPeriodSEI, slice);
    seiMessages.push_back(bufferingPeriodSEI);
    m_bufferingPeriodSEIPresentInAU = true;

    if (m_pcCfg->getScalableNestingSEIEnabled())
    {
      SEIBufferingPeriod *bufferingPeriodSEIcopy = new SEIBufferingPeriod();
      bufferingPeriodSEI->copyTo(*bufferingPeriodSEIcopy);
      nestedSeiMessages.push_back(bufferingPeriodSEIcopy);
    }
  }

  if (picInGOP ==0 && m_pcCfg->getSOPDescriptionSEIEnabled() ) // write SOP description SEI (if enabled) at the beginning of GOP
  {
    SEISOPDescription* sopDescriptionSEI = new SEISOPDescription();
    m_seiEncoder.initSEISOPDescription(sopDescriptionSEI, slice, picInGOP, m_iLastIDR, m_iGopSize);
    seiMessages.push_back(sopDescriptionSEI);
  }

  if( ( m_pcEncTop->getRecoveryPointSEIEnabled() ) && ( slice->getSliceType() == I_SLICE ) )
  {
    if( m_pcEncTop->getGradualDecodingRefreshInfoEnabled() && !slice->getRapPicFlag() )
    {
      // Gradual decoding refresh SEI
      SEIGradualDecodingRefreshInfo *gradualDecodingRefreshInfoSEI = new SEIGradualDecodingRefreshInfo();
      gradualDecodingRefreshInfoSEI->m_gdrForegroundFlag = true; // Indicating all "foreground"
      seiMessages.push_back(gradualDecodingRefreshInfoSEI);
    }
    // Recovery point SEI
    SEIRecoveryPoint *recoveryPointSEI = new SEIRecoveryPoint();
    m_seiEncoder.initSEIRecoveryPoint(recoveryPointSEI, slice);
    seiMessages.push_back(recoveryPointSEI);
  }
  if (m_pcCfg->getTemporalLevel0IndexSEIEnabled())
  {
    SEITemporalLevel0Index *temporalLevel0IndexSEI = new SEITemporalLevel0Index();
    m_seiEncoder.initTemporalLevel0IndexSEI(temporalLevel0IndexSEI, slice);
    seiMessages.push_back(temporalLevel0IndexSEI);
  }

  if(slice->getSPS()->getVuiParametersPresentFlag() && m_pcCfg->getChromaSamplingFilterHintEnabled() && ( slice->getSliceType() == I_SLICE ))
  {
    SEIChromaSamplingFilterHint *seiChromaSamplingFilterHint = new SEIChromaSamplingFilterHint;
    m_seiEncoder.initSEIChromaSamplingFilterHint(seiChromaSamplingFilterHint, m_pcCfg->getChromaSamplingHorFilterIdc(), m_pcCfg->getChromaSamplingVerFilterIdc());
    seiMessages.push_back(seiChromaSamplingFilterHint);
  }

  if( m_pcEncTop->getNoDisplaySEITLayer() && ( slice->getTLayer() >= m_pcEncTop->getNoDisplaySEITLayer() ) )
  {
    SEINoDisplay *seiNoDisplay = new SEINoDisplay;
    seiNoDisplay->m_noDisplay = true;
    seiMessages.push_back(seiNoDisplay);
  }
}

Void TEncGOP::xCreateScalableNestingSEI (SEIMessages& seiMessages, SEIMessages& nestedSeiMessages)//创建ScalableNesting SEI信息
{
  SEIMessages tmpMessages;
  while (!nestedSeiMessages.empty())
  {
    SEI* sei=nestedSeiMessages.front();
    nestedSeiMessages.pop_front();
    tmpMessages.push_back(sei);
    SEIScalableNesting *nestingSEI = new SEIScalableNesting();
    m_seiEncoder.initSEIScalableNesting(nestingSEI, tmpMessages);
    seiMessages.push_back(nestingSEI);
    tmpMessages.clear();
  }
}

Void TEncGOP::xCreatePictureTimingSEI  (Int IRAPGOPid, SEIMessages& seiMessages, SEIMessages& nestedSeiMessages, SEIMessages& duInfoSeiMessages, TComSlice *slice, Bool isField, std::deque<DUData> &duData)//创建PictureTiming SEI信息
{
  Int picSptDpbOutputDuDelay = 0;
  SEIPictureTiming *pictureTimingSEI = new SEIPictureTiming();

  const TComVUI *vui = slice->getSPS()->getVuiParameters();
  const TComHRD *hrd = vui->getHrdParameters();

  // update decoding unit parameters
  if( ( m_pcCfg->getPictureTimingSEIEnabled() || m_pcCfg->getDecodingUnitInfoSEIEnabled() ) &&
    ( slice->getSPS()->getVuiParametersPresentFlag() ) &&
    (  hrd->getNalHrdParametersPresentFlag() || hrd->getVclHrdParametersPresentFlag() ) )
  {
    // DU parameters
    if( hrd->getSubPicCpbParamsPresentFlag() )
    {
      UInt numDU = (UInt) duData.size();
      pictureTimingSEI->m_numDecodingUnitsMinus1     = ( numDU - 1 );
      pictureTimingSEI->m_duCommonCpbRemovalDelayFlag = false;
      pictureTimingSEI->m_numNalusInDuMinus1.resize( numDU );
      pictureTimingSEI->m_duCpbRemovalDelayMinus1.resize( numDU );
    }
    pictureTimingSEI->m_auCpbRemovalDelay = std::min<Int>(std::max<Int>(1, m_totalCoded - m_lastBPSEI), static_cast<Int>(pow(2, static_cast<Double>(hrd->getCpbRemovalDelayLengthMinus1()+1)))); // Syntax element signalled as minus, hence the .
    pictureTimingSEI->m_picDpbOutputDelay = slice->getSPS()->getNumReorderPics(slice->getSPS()->getMaxTLayers()-1) + slice->getPOC() - m_totalCoded;
    if(m_pcCfg->getEfficientFieldIRAPEnabled() && IRAPGOPid > 0 && IRAPGOPid < m_iGopSize)
    {
      // if pictures have been swapped there is likely one more picture delay on their tid. Very rough approximation
      pictureTimingSEI->m_picDpbOutputDelay ++;
    }
    Int factor = hrd->getTickDivisorMinus2() + 2;
    pictureTimingSEI->m_picDpbOutputDuDelay = factor * pictureTimingSEI->m_picDpbOutputDelay;
    if( m_pcCfg->getDecodingUnitInfoSEIEnabled() )
    {
      picSptDpbOutputDuDelay = factor * pictureTimingSEI->m_picDpbOutputDelay;
    }
    if (m_bufferingPeriodSEIPresentInAU)
    {
      m_lastBPSEI = m_totalCoded;
    }

    if( hrd->getSubPicCpbParamsPresentFlag() )
    {
      Int i;
      UInt64 ui64Tmp;
      UInt uiPrev = 0;
      UInt numDU = ( pictureTimingSEI->m_numDecodingUnitsMinus1 + 1 );
      std::vector<UInt> &rDuCpbRemovalDelayMinus1 = pictureTimingSEI->m_duCpbRemovalDelayMinus1;
      UInt maxDiff = ( hrd->getTickDivisorMinus2() + 2 ) - 1;

      for( i = 0; i < numDU; i ++ )
      {
        pictureTimingSEI->m_numNalusInDuMinus1[ i ]       = ( i == 0 ) ? ( duData[i].accumNalsDU - 1 ) : ( duData[i].accumNalsDU- duData[i-1].accumNalsDU - 1 );
      }

      if( numDU == 1 )
      {
        rDuCpbRemovalDelayMinus1[ 0 ] = 0; /* don't care */
      }
      else
      {
        rDuCpbRemovalDelayMinus1[ numDU - 1 ] = 0;/* by definition */
        UInt tmp = 0;
        UInt accum = 0;

        for( i = ( numDU - 2 ); i >= 0; i -- )
        {
          ui64Tmp = ( ( ( duData[numDU - 1].accumBitsDU  - duData[i].accumBitsDU ) * ( vui->getTimingInfo()->getTimeScale() / vui->getTimingInfo()->getNumUnitsInTick() ) * ( hrd->getTickDivisorMinus2() + 2 ) ) / ( m_pcCfg->getTargetBitrate() ) );
          if( (UInt)ui64Tmp > maxDiff )
          {
            tmp ++;
          }
        }
        uiPrev = 0;

        UInt flag = 0;
        for( i = ( numDU - 2 ); i >= 0; i -- )
        {
          flag = 0;
          ui64Tmp = ( ( ( duData[numDU - 1].accumBitsDU  - duData[i].accumBitsDU ) * ( vui->getTimingInfo()->getTimeScale() / vui->getTimingInfo()->getNumUnitsInTick() ) * ( hrd->getTickDivisorMinus2() + 2 ) ) / ( m_pcCfg->getTargetBitrate() ) );

          if( (UInt)ui64Tmp > maxDiff )
          {
            if(uiPrev >= maxDiff - tmp)
            {
              ui64Tmp = uiPrev + 1;
              flag = 1;
            }
            else                            ui64Tmp = maxDiff - tmp + 1;
          }
          rDuCpbRemovalDelayMinus1[ i ] = (UInt)ui64Tmp - uiPrev - 1;
          if( (Int)rDuCpbRemovalDelayMinus1[ i ] < 0 )
          {
            rDuCpbRemovalDelayMinus1[ i ] = 0;
          }
          else if (tmp > 0 && flag == 1)
          {
            tmp --;
          }
          accum += rDuCpbRemovalDelayMinus1[ i ] + 1;
          uiPrev = accum;
        }
      }
    }
    
    if( m_pcCfg->getPictureTimingSEIEnabled() )
    {
      pictureTimingSEI->m_picStruct = (isField && slice->getPic()->isTopField())? 1 : isField? 2 : 0;
      seiMessages.push_back(pictureTimingSEI);

      if ( m_pcCfg->getScalableNestingSEIEnabled() ) // put picture timing SEI into scalable nesting SEI
      {
        SEIPictureTiming *pictureTimingSEIcopy = new SEIPictureTiming();
        pictureTimingSEI->copyTo(*pictureTimingSEIcopy);
        nestedSeiMessages.push_back(pictureTimingSEIcopy);
      }
    }

    if( m_pcCfg->getDecodingUnitInfoSEIEnabled() && hrd->getSubPicCpbParamsPresentFlag() )
    {
      for( Int i = 0; i < ( pictureTimingSEI->m_numDecodingUnitsMinus1 + 1 ); i ++ )
      {
        SEIDecodingUnitInfo *duInfoSEI = new SEIDecodingUnitInfo();
        duInfoSEI->m_decodingUnitIdx = i;
        duInfoSEI->m_duSptCpbRemovalDelay = pictureTimingSEI->m_duCpbRemovalDelayMinus1[i] + 1;
        duInfoSEI->m_dpbOutputDuDelayPresentFlag = false;
        duInfoSEI->m_picSptDpbOutputDuDelay = picSptDpbOutputDuDelay;

        duInfoSeiMessages.push_back(duInfoSEI);
      }
    }
  }
}

Void TEncGOP::xUpdateDuData(AccessUnit &testAU, std::deque<DUData> &duData)
{
  if (duData.empty())
  {
    return;
  }
  // fix first 
  UInt numNalUnits = (UInt)testAU.size();
  UInt numRBSPBytes = 0;
  for (AccessUnit::const_iterator it = testAU.begin(); it != testAU.end(); it++)
  {
    numRBSPBytes += UInt((*it)->m_nalUnitData.str().size());
  }
  duData[0].accumBitsDU += ( numRBSPBytes << 3 );
  duData[0].accumNalsDU += numNalUnits;

  // adapt cumulative sums for all following DUs
  // and add one DU info SEI, if enabled
  for (Int i=1; i<duData.size(); i++)
  {
    if (m_pcCfg->getDecodingUnitInfoSEIEnabled())
    {
      numNalUnits  += 1;
      numRBSPBytes += ( 5 << 3 );
    }
    duData[i].accumBitsDU += numRBSPBytes; // probably around 5 bytes
    duData[i].accumNalsDU += numNalUnits;
  }

  // The last DU may have a trailing SEI
  if (m_pcCfg->getDecodedPictureHashSEIEnabled())
  {
    duData.back().accumBitsDU += ( 20 << 3 ); // probably around 20 bytes - should be further adjusted, e.g. by type
    duData.back().accumNalsDU += 1;
  }

}
Void TEncGOP::xUpdateTimingSEI(SEIPictureTiming *pictureTimingSEI, std::deque<DUData> &duData, const TComSPS *sps)
{
  if (!pictureTimingSEI)
  {
    return;
  }
  const TComVUI *vui = sps->getVuiParameters();
  const TComHRD *hrd = vui->getHrdParameters();
  if( hrd->getSubPicCpbParamsPresentFlag() )
  {
    Int i;
    UInt64 ui64Tmp;
    UInt uiPrev = 0;
    UInt numDU = ( pictureTimingSEI->m_numDecodingUnitsMinus1 + 1 );
    std::vector<UInt> &rDuCpbRemovalDelayMinus1 = pictureTimingSEI->m_duCpbRemovalDelayMinus1;
    UInt maxDiff = ( hrd->getTickDivisorMinus2() + 2 ) - 1;

    for( i = 0; i < numDU; i ++ )
    {
      pictureTimingSEI->m_numNalusInDuMinus1[ i ]       = ( i == 0 ) ? ( duData[i].accumNalsDU - 1 ) : ( duData[i].accumNalsDU- duData[i-1].accumNalsDU - 1 );
    }

    if( numDU == 1 )
    {
      rDuCpbRemovalDelayMinus1[ 0 ] = 0; /* don't care */
    }
    else
    {
      rDuCpbRemovalDelayMinus1[ numDU - 1 ] = 0;/* by definition */
      UInt tmp = 0;
      UInt accum = 0;

      for( i = ( numDU - 2 ); i >= 0; i -- )
      {
        ui64Tmp = ( ( ( duData[numDU - 1].accumBitsDU  - duData[i].accumBitsDU ) * ( vui->getTimingInfo()->getTimeScale() / vui->getTimingInfo()->getNumUnitsInTick() ) * ( hrd->getTickDivisorMinus2() + 2 ) ) / ( m_pcCfg->getTargetBitrate() ) );
        if( (UInt)ui64Tmp > maxDiff )
        {
          tmp ++;
        }
      }
      uiPrev = 0;

      UInt flag = 0;
      for( i = ( numDU - 2 ); i >= 0; i -- )
      {
        flag = 0;
        ui64Tmp = ( ( ( duData[numDU - 1].accumBitsDU  - duData[i].accumBitsDU ) * ( vui->getTimingInfo()->getTimeScale() / vui->getTimingInfo()->getNumUnitsInTick() ) * ( hrd->getTickDivisorMinus2() + 2 ) ) / ( m_pcCfg->getTargetBitrate() ) );

        if( (UInt)ui64Tmp > maxDiff )
        {
          if(uiPrev >= maxDiff - tmp)
          {
            ui64Tmp = uiPrev + 1;
            flag = 1;
          }
          else                            ui64Tmp = maxDiff - tmp + 1;
        }
        rDuCpbRemovalDelayMinus1[ i ] = (UInt)ui64Tmp - uiPrev - 1;
        if( (Int)rDuCpbRemovalDelayMinus1[ i ] < 0 )
        {
          rDuCpbRemovalDelayMinus1[ i ] = 0;
        }
        else if (tmp > 0 && flag == 1)
        {
          tmp --;
        }
        accum += rDuCpbRemovalDelayMinus1[ i ] + 1;
        uiPrev = accum;
      }
    }
  }
}
Void TEncGOP::xUpdateDuInfoSEI(SEIMessages &duInfoSeiMessages, SEIPictureTiming *pictureTimingSEI)
{
  if (duInfoSeiMessages.empty() || (pictureTimingSEI == NULL))
  {
    return;
  }

  Int i=0;

  for (SEIMessages::iterator du = duInfoSeiMessages.begin(); du!= duInfoSeiMessages.end(); du++)
  {
    SEIDecodingUnitInfo *duInfoSEI = (SEIDecodingUnitInfo*) (*du);
    duInfoSEI->m_decodingUnitIdx = i;
    duInfoSEI->m_duSptCpbRemovalDelay = pictureTimingSEI->m_duCpbRemovalDelayMinus1[i] + 1;
    duInfoSEI->m_dpbOutputDuDelayPresentFlag = false;
    i++;
  }
}

static Void  //参见<T-REC-H.265>section 7.4.3.10
cabac_zero_word_padding(TComSlice *const pcSlice, TComPic *const pcPic, const std::size_t binCountsInNalUnits, const std::size_t numBytesInVclNalUnits, std::ostringstream &nalUnitData, const Bool cabacZeroWordPaddingEnabled)
{
  const TComSPS &sps=*(pcSlice->getSPS());
  const Int log2subWidthCxsubHeightC = (pcPic->getComponentScaleX(COMPONENT_Cb)+pcPic->getComponentScaleY(COMPONENT_Cb));
  const Int minCuWidth  = pcPic->getMinCUWidth();
  const Int minCuHeight = pcPic->getMinCUHeight();
  const Int paddedWidth = ((sps.getPicWidthInLumaSamples()  + minCuWidth  - 1) / minCuWidth) * minCuWidth;//将图像的宽填充为minCuWidth的最小整数倍数
  const Int paddedHeight= ((sps.getPicHeightInLumaSamples() + minCuHeight - 1) / minCuHeight) * minCuHeight;//将图像的高填充为minCuWidth的最小整数倍数
  const Int rawBits = paddedWidth * paddedHeight *
                         (sps.getBitDepth(CHANNEL_TYPE_LUMA) + 2*(sps.getBitDepth(CHANNEL_TYPE_CHROMA)>>log2subWidthCxsubHeightC));//编码填充的后图像的原始比特数(直接编码原始像素值)
  const std::size_t threshold = (32/3)*numBytesInVclNalUnits + (rawBits/32);//??不知道这公式是怎么来的
  if (binCountsInNalUnits >= threshold)
  {
    // need to add additional cabac zero words (each one accounts for 3 bytes (=00 00 03)) to increase numBytesInVclNalUnits
    const std::size_t targetNumBytesInVclNalUnits = ((binCountsInNalUnits - (rawBits/32))*3+31)/32;//??

    if (targetNumBytesInVclNalUnits>numBytesInVclNalUnits) // It should be!
    {
      const std::size_t numberOfAdditionalBytesNeeded=targetNumBytesInVclNalUnits - numBytesInVclNalUnits;
      const std::size_t numberOfAdditionalCabacZeroWords=(numberOfAdditionalBytesNeeded+2)/3;
      const std::size_t numberOfAdditionalCabacZeroBytes=numberOfAdditionalCabacZeroWords*3;
      if (cabacZeroWordPaddingEnabled)
      {
        std::vector<Char> zeroBytesPadding(numberOfAdditionalCabacZeroBytes, Char(0));
        for(std::size_t i=0; i<numberOfAdditionalCabacZeroWords; i++)
        {
          zeroBytesPadding[i*3+2]=3;  // 00 00 03
        }
        nalUnitData.write(&(zeroBytesPadding[0]), numberOfAdditionalCabacZeroBytes);
        printf("Adding %d bytes of padding\n", UInt(numberOfAdditionalCabacZeroWords*3));
      }
      else
      {
        printf("Standard would normally require adding %d bytes of padding\n", UInt(numberOfAdditionalCabacZeroWords*3));
      }
    }
  }
}

class EfficientFieldIRAPMapping
{
  private:
    Int  IRAPGOPid;
    Bool IRAPtoReorder;
    Bool swapIRAPForward;

  public:
    EfficientFieldIRAPMapping() :
      IRAPGOPid(-1),
      IRAPtoReorder(false),
      swapIRAPForward(false)
    { }

    Void initialize(const Bool isField, const Int gopSize, const Int POCLast, const Int numPicRcvd, const Int lastIDR, TEncGOP *pEncGop, TEncCfg *pCfg);

    Int adjustGOPid(const Int gopID);
    Int restoreGOPid(const Int gopID);
    Int GetIRAPGOPid() const { return IRAPGOPid; }
};

Void EfficientFieldIRAPMapping::initialize(const Bool isField, const Int gopSize, const Int POCLast, const Int numPicRcvd, const Int lastIDR, TEncGOP *pEncGop, TEncCfg *pCfg )
{
  if(isField)
  {
    Int pocCurr;
    for ( Int iGOPid=0; iGOPid < gopSize; iGOPid++ )
    {
      // determine actual POC
      if(POCLast == 0) //case first frame or first top field
      {
        pocCurr=0;
      }
      else if(POCLast == 1 && isField) //case first bottom field, just like the first frame, the poc computation is not right anymore, we set the right value
      {
        pocCurr = 1;
      }
      else
      {
        pocCurr = POCLast - numPicRcvd + pCfg->getGOPEntry(iGOPid).m_POC - isField;//计算当前图像的POC值(iPOCLast为当前GOP中图像最大的POC 减去iNumPicRcvd即为GOP中起始POC值 再加上GOP对应位置的m_POC值 即可得到该位置的POC值)
      }

      // check if POC corresponds to IRAP
      NalUnitType tmpUnitType = pEncGop->getNalUnitType(pocCurr, lastIDR, isField);
      if(tmpUnitType >= NAL_UNIT_CODED_SLICE_BLA_W_LP && tmpUnitType <= NAL_UNIT_CODED_SLICE_CRA) // if picture is an IRAP//当前帧为IRAP
      {
        if(pocCurr%2 == 0 && iGOPid < gopSize-1 && pCfg->getGOPEntry(iGOPid).m_POC == pCfg->getGOPEntry(iGOPid+1).m_POC-1)//编码顺序在该场后的场为前一帧图像的底场 //该情况下需调整GOP处理顺序
        { // if top field and following picture in enc order is associated bottom field
          IRAPGOPid = iGOPid;
          IRAPtoReorder = true; //需要调整GOPid的顺序 先处理前一帧图像的底场
          swapIRAPForward = true; //swapIRAPForward为true表示该IRAP原本在前(编码顺序) 需要调整顺序 在处理前一帧图像的底场之后处理
          break;
        }
        if(pocCurr%2 != 0 && iGOPid > 0 && pCfg->getGOPEntry(iGOPid).m_POC == pCfg->getGOPEntry(iGOPid-1).m_POC+1)//编码顺序在该场前的场为后一帧图像的顶场  //该情况下需调整GOP处理顺序
        {
          // if picture is an IRAP remember to process it first
          IRAPGOPid = iGOPid;
          IRAPtoReorder = true; //需要调整GOPid的顺序  先处理该场
          swapIRAPForward = false; //swapIRAPForward为false表示该IRAP原本在后(编码顺序) 需要调整顺序在 处理后一帧图像的顶场之前处理
          break;
        }
      }
    }
  }
}

Int EfficientFieldIRAPMapping::adjustGOPid(const Int GOPid)//调整GOP处理顺序
{
  if(IRAPtoReorder)//需要调整GOP处理顺序
  {
    if(swapIRAPForward)//将IRAP调整至前一帧图像的底场之后处理
    {
      if(GOPid == IRAPGOPid)
      {
        return IRAPGOPid +1;
      }
      else if(GOPid == IRAPGOPid +1)
      {
        return IRAPGOPid;
      }
    }
    else//将IRAP调整至后一帧图像的顶场之前处理
    {
      if(GOPid == IRAPGOPid -1)
      {
        return IRAPGOPid;
      }
      else if(GOPid == IRAPGOPid)
      {
        return IRAPGOPid -1;
      }
    }
  }
  return GOPid;//若不需要调整 则直接返回原始GOPid
}

Int EfficientFieldIRAPMapping::restoreGOPid(const Int GOPid)//还原GOP 将调整过后的GOPid还原成为调整之前的GOPid 
{
  if(IRAPtoReorder)//在按照相同方法调整一次顺序就可得到原来未调整之前的顺序
  {
    if(swapIRAPForward)
    {
      if(GOPid == IRAPGOPid)
      {
        IRAPtoReorder = false;
        return IRAPGOPid +1;
      }
      else if(GOPid == IRAPGOPid +1)
      {
        return GOPid -1;
      }
    }
    else
    {
      if(GOPid == IRAPGOPid)
      {
        return IRAPGOPid -1;
      }
      else if(GOPid == IRAPGOPid -1)
      {
        IRAPtoReorder = false;
        return IRAPGOPid;
      }
    }
  }//过程同上 不同在于还原完成后需将IRAPtoReorder标志置为false 表示还原完成
  return GOPid;
}


static UInt calculateCollocatedFromL1Flag(TEncCfg *pCfg, const Int GOPid, const Int gopSize)//计算是从list0还是list1中得到同位图像
{
  Int iCloseLeft=1, iCloseRight=-1;
  for(Int i = 0; i<pCfg->getGOPEntry(GOPid).m_numRefPics; i++)//遍历当前图像的所有参考图像
  {
    Int iRef = pCfg->getGOPEntry(GOPid).m_referencePics[i];//deltaPOC
    if(iRef>0&&(iRef<iCloseRight||iCloseRight==-1))//得到离当前图像最近的右侧(正向)参考图像
    {
      iCloseRight=iRef;
    }
    else if(iRef<0&&(iRef>iCloseLeft||iCloseLeft==1))//得到离当前图像最近的左侧(负向)参考图像
    {
      iCloseLeft=iRef;
    }
  }
  if(iCloseRight>-1)//若存在右侧(正向)参考图像
  {
    iCloseRight=iCloseRight+pCfg->getGOPEntry(GOPid).m_POC-1;//得到该右侧参考图像在GOP中的POC值 m_POC需要减1是因为配置文件中的GOP结构中图像POC值是从1开始  而实际计算时GOP结构中起始POC值为0
  }
  if(iCloseLeft<1)//若存在左侧(负向)参考图像
  {
    iCloseLeft=iCloseLeft+pCfg->getGOPEntry(GOPid).m_POC-1;//得到该左侧参考图像在GOP中的POC值
    while(iCloseLeft<0)
    {
      iCloseLeft+=gopSize;
    }
  }
  Int iLeftQP=0, iRightQP=0;
  for(Int i=0; i<gopSize; i++)//得到左右参考图像的QPOffset值
  {
    if(pCfg->getGOPEntry(i).m_POC==(iCloseLeft%gopSize)+1)
    {
      iLeftQP= pCfg->getGOPEntry(i).m_QPOffset;
    }
    if (pCfg->getGOPEntry(i).m_POC==(iCloseRight%gopSize)+1)
    {
      iRightQP=pCfg->getGOPEntry(i).m_QPOffset;
    }
  }
  if(iCloseRight>-1&&iRightQP<iLeftQP)//若存在右侧参考图像且左侧参考图像OP值较大
  {
    return 0;//从list0从得到同位图像
  }
  else
  {
    return 1;//从list1从得到同位图像
  }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================
Void TEncGOP::compressGOP( Int iPOCLast, Int iNumPicRcvd, TComList<TComPic*>& rcListPic,
                           TComList<TComPicYuv*>& rcListPicYuvRecOut, std::list<AccessUnit>& accessUnitsInGOP,
                           Bool isField, Bool isTff, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE )//压缩GOP(中每一帧图像)
{//iPOCLast为视频流中最新进入图像缓存列表中图像的POC值  iNumPicRcvd为接受的图像数 可理解为图像缓存列表中待编码的图像数 一般为GOP图像数
  // TODO: Split this function up.

  TComPic*        pcPic = NULL;
  TComPicYuv*     pcPicYuvRecOut;
  TComSlice*      pcSlice;
  TComOutputBitstream  *pcBitstreamRedirect;
  pcBitstreamRedirect = new TComOutputBitstream;
  AccessUnit::iterator  itLocationToPushSliceHeaderNALU; // used to store location where NALU containing slice header is to be inserted

  xInitGOP( iPOCLast, iNumPicRcvd, isField );//初始化m_iGopSize

  m_iNumPicCoded = 0;
  SEIMessages leadingSeiMessages;
  SEIMessages nestedSeiMessages;
  SEIMessages duInfoSeiMessages;
  SEIMessages trailingSeiMessages;
  std::deque<DUData> duData;
  SEIDecodingUnitInfo decodingUnitInfoSEI;

  EfficientFieldIRAPMapping effFieldIRAPMap;
  if (m_pcCfg->getEfficientFieldIRAPEnabled())
  {
    effFieldIRAPMap.initialize(isField, m_iGopSize, iPOCLast, iNumPicRcvd, m_iLastIDR, this, m_pcCfg);
  }

  // reset flag indicating whether pictures have been encoded
  for ( Int iGOPid=0; iGOPid < m_iGopSize; iGOPid++ )//重置EncodedFlag(该标志表示该帧图像是否已经被编码过)
  {
    m_pcCfg->setEncodedFlag(iGOPid, false);
  }

  for ( Int iGOPid=0; iGOPid < m_iGopSize; iGOPid++ )//依次处理(压缩)GOP中的每帧图像(GOPid表示Frame id 可理解为GOP中图像的编码/解码顺序)
  {
    if (m_pcCfg->getEfficientFieldIRAPEnabled())
    {
      iGOPid=effFieldIRAPMap.adjustGOPid(iGOPid);
    }

    //-- For time output for each slice
    clock_t iBeforeTime = clock();//用于计算编码时间

    UInt uiColDir = calculateCollocatedFromL1Flag(m_pcCfg, iGOPid, m_iGopSize);

    /////////////////////////////////////////////////////////////////////////////////////////////////// Initial to start encoding
    Int iTimeOffset;
    Int pocCurr;
    //pocCurr为当前图像在整个视频中的输出顺序(从0开始) iTimeOffset可理解为GOP中图像POC较该GOP中起始POC的偏移量
    if(iPOCLast == 0) //case first frame or first top field//为第一帧图像或第一帧顶场 poc值为0
    {
      pocCurr=0;
      iTimeOffset = 1;
    }
    else if(iPOCLast == 1 && isField) //case first bottom field, just like the first frame, the poc computation is not right anymore, we set the right value//第一帧底场 pocCurr值为1
    {
      pocCurr = 1;
      iTimeOffset = 1;
    }
    else
    {
      pocCurr = iPOCLast - iNumPicRcvd + m_pcCfg->getGOPEntry(iGOPid).m_POC - ((isField && m_iGopSize>1) ? 1:0);//计算当前图像的POC值(iPOCLast为当前GOP中图像最大的POC 减去iNumPicRcvd即为GOP中起始POC值 再加上GOP对应位置的m_POC值 即可得到该位置的POC值)
      iTimeOffset = m_pcCfg->getGOPEntry(iGOPid).m_POC;
    }

    if(pocCurr>=m_pcCfg->getFramesToBeEncoded())//图像范围待编码的图像范围 则执行下一帧(编码顺序)图像
    {
      if (m_pcCfg->getEfficientFieldIRAPEnabled())
      {
        iGOPid=effFieldIRAPMap.restoreGOPid(iGOPid);
      }
      continue;
    }

    if( getNalUnitType(pocCurr, m_iLastIDR, isField) == NAL_UNIT_CODED_SLICE_IDR_W_RADL || getNalUnitType(pocCurr, m_iLastIDR, isField) == NAL_UNIT_CODED_SLICE_IDR_N_LP )//如果当前图像为IDR
    {
      m_iLastIDR = pocCurr;//则更新m_iLastIDR(邻近的IDR图像的POC值)
    }
    // start a new access unit: create an entry in the list of output access units//One coded picture, together with the non-VCL NAL units that are associated with the coded picture, is called an HEVC access unit
    accessUnitsInGOP.push_back(AccessUnit());//新建一个access unit加入列表中 (一个已编码的图像对应一个access unit)
    AccessUnit& accessUnit = accessUnitsInGOP.back();//新建的access unit
    xGetBuffer( rcListPic, rcListPicYuvRecOut, iNumPicRcvd, iTimeOffset, pcPic, pcPicYuvRecOut, pocCurr, isField );//得到待压缩图像pcPic及重建图像pcPicYuvRecOut(用来保存重建图像)

    //  Slice data initialization
    pcPic->clearSliceBuffer();
    pcPic->allocateNewSlice();
    m_pcSliceEncoder->setSliceIdx(0);
    pcPic->setCurrSliceIdx(0);//初始化图像中第一个slice

    m_pcSliceEncoder->initEncSlice ( pcPic, iPOCLast, pocCurr, iGOPid, pcSlice, isField );//初始化该图像slice编码相关的信息至pcSlice (pcSlice指向该图像第一个slice)

    //Set Frame/Field coding
    pcSlice->getPic()->setField(isField);//否是为场编码

    pcSlice->setLastIDR(m_iLastIDR);//设置LastIDR
    pcSlice->setSliceIdx(0);//设置该slice索引为0
    //set default slice level flag to the same as SPS level flag
    pcSlice->setLFCrossSliceBoundaryFlag(  pcSlice->getPPS()->getLoopFilterAcrossSlicesEnabledFlag()  );//设置环路滤波是否允许跨越slice边界

    if(pcSlice->getSliceType()==B_SLICE&&m_pcCfg->getGOPEntry(iGOPid).m_sliceType=='P')
    {
      pcSlice->setSliceType(P_SLICE);
    }
    if(pcSlice->getSliceType()==B_SLICE&&m_pcCfg->getGOPEntry(iGOPid).m_sliceType=='I')
    {
      pcSlice->setSliceType(I_SLICE);
    }//根据配置文件以更为严格的slice类型为准(B>P>I 因为B slice 可以单向预测也可以帧内预测)
    
    // Set the nal unit type
    pcSlice->setNalUnitType(getNalUnitType(pocCurr, m_iLastIDR, isField));//设置图像NalUnit类型
    if(pcSlice->getTemporalLayerNonReferenceFlag())//若该图像为Sub-layer Non-reference Pictures
    {
      if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_TRAIL_R &&
          !(m_iGopSize == 1 && pcSlice->getSliceType() == I_SLICE))
        // Add this condition to avoid POC issues with encoder_intra_main.cfg configuration (see #1127 in bug tracker)
      {
        pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_TRAIL_N);
      }
      if(pcSlice->getNalUnitType()==NAL_UNIT_CODED_SLICE_RADL_R)
      {
        pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_RADL_N);
      }
      if(pcSlice->getNalUnitType()==NAL_UNIT_CODED_SLICE_RASL_R)
      {
        pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_RASL_N);
      }
    }//则设置对应类型图像为Sub-layer Non-reference类型

    if (m_pcCfg->getEfficientFieldIRAPEnabled())//看了好久还是每明白EfficientFieldIRAP是什么意思-_-||
    {
      if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA )  // IRAP picture
      {
        m_associatedIRAPType = pcSlice->getNalUnitType();//当当前图像为IRAP picture时更新邻近IRAP图像类型及POC值
        m_associatedIRAPPOC = pocCurr;
      }
      pcSlice->setAssociatedIRAPType(m_associatedIRAPType);
      pcSlice->setAssociatedIRAPPOC(m_associatedIRAPPOC);
    }
    // Do decoding refresh marking if any
    pcSlice->decodingRefreshMarking(m_pocCRA, m_bRefreshPending, rcListPic, m_pcCfg->getEfficientFieldIRAPEnabled());//当遇到IDR/CRA/CRANT/BLA/BLANT时 更新参考图像标志
    m_pcEncTop->selectReferencePictureSet(pcSlice, pocCurr, iGOPid);//得到该图像的RPS
    if (!m_pcCfg->getEfficientFieldIRAPEnabled())
    {
      if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA )  // IRAP picture
      {
        m_associatedIRAPType = pcSlice->getNalUnitType();
        m_associatedIRAPPOC = pocCurr;
      }
      pcSlice->setAssociatedIRAPType(m_associatedIRAPType);
      pcSlice->setAssociatedIRAPPOC(m_associatedIRAPPOC);//当当前图像为IRAP picture时更新邻近IRAP图像类型及POC值
    }

    if ((pcSlice->checkThatAllRefPicsAreAvailable(rcListPic, pcSlice->getRPS(), false, m_iLastRecoveryPicPOC, m_pcCfg->getDecodingRefreshType() == 3) != 0) || (pcSlice->isIRAP()) 
      || (m_pcCfg->getEfficientFieldIRAPEnabled() && isField && pcSlice->getAssociatedIRAPType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP && pcSlice->getAssociatedIRAPType() <= NAL_UNIT_CODED_SLICE_CRA && pcSlice->getAssociatedIRAPPOC() == pcSlice->getPOC()+1)
      )//若该图像存在参考图像在图像缓存列表中不可获得
    {
      pcSlice->createExplicitReferencePictureSetFromReference(rcListPic, pcSlice->getRPS(), pcSlice->isIRAP(), m_iLastRecoveryPicPOC, m_pcCfg->getDecodingRefreshType() == 3, m_pcCfg->getEfficientFieldIRAPEnabled());
    }//则根据图像缓存列表和参考RPS创建新的参考图像全部获得的RPS(实际的RPS应由实际的参考图像缓存得到)

    pcSlice->applyReferencePictureSet(rcListPic, pcSlice->getRPS());//根据RPS标记图像缓存列表中的图像是否为参考图像

    if(pcSlice->getTLayer() > 0 
      &&  !( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_N     // Check if not a leading picture
          || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_R
          || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_N
          || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_R )
        )//若该图像不为leading picture
    {
      if(pcSlice->isTemporalLayerSwitchingPoint(rcListPic) || pcSlice->getSPS()->getTemporalIdNestingFlag())//若该图像为时域层SwitchingPoint 则说明该图像为TSA(详见TComSlice.cpp 800行往后的代码)
      {
        if(pcSlice->getTemporalLayerNonReferenceFlag())//是否为Sub-layer Non-reference Pictures 
        {
          pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_TSA_N);
        }
        else
        {
          pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_TSA_R);
        }
      }//根据是否为Sub-layer Non-reference Pictures 设置为对应图像类型
      else if(pcSlice->isStepwiseTemporalLayerSwitchingPointCandidate(rcListPic))//若该图像为候选的STSA图像 //详见相关资料STSA的说明
      {
        Bool isSTSA=true;
        for(Int ii=iGOPid+1;(ii<m_pcCfg->getGOPSize() && isSTSA==true);ii++)//按照编码顺序遍历GOP中该图像其后的图像
        {
          Int lTid= m_pcCfg->getGOPEntry(ii).m_temporalId;//时域层
          if(lTid==pcSlice->getTLayer())//若其后图像的时域层与该图像相同
          {
            const TComReferencePictureSet* nRPS = pcSlice->getSPS()->getRPSList()->getReferencePictureSet(ii);//该图像的RPS
            for(Int jj=0;jj<nRPS->getNumberOfPictures();jj++)
            {
              if(nRPS->getUsed(jj))//RPS中参考图像被该图像用作参考
              {
                Int tPoc=m_pcCfg->getGOPEntry(ii).m_POC+nRPS->getDeltaPOC(jj);//该参考图像在GOP中的POC值
                Int kk=0;
                for(kk=0;kk<m_pcCfg->getGOPSize();kk++)//根据该参考图像在GOP中的POC值找到该参考图像在GOP中的位置(GOP id)
                {
                  if(m_pcCfg->getGOPEntry(kk).m_POC==tPoc)
                  {
                    break;
                  }
                }
                Int tTid=m_pcCfg->getGOPEntry(kk).m_temporalId;
                if(tTid >= pcSlice->getTLayer())//若参考图像的与该图像在同一时域层或更高的时域层
                {
                  isSTSA=false;//则该图像不为STSA
                  break;
                }
              }
            }
          }
        }
        if(isSTSA==true)//若该图像为STSA
        {
          if(pcSlice->getTemporalLayerNonReferenceFlag())//该图像是否为为Sub-layer Non-reference Pictures
          {
            pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_STSA_N);
          }
          else
          {
            pcSlice->setNalUnitType(NAL_UNIT_CODED_SLICE_STSA_R);
          }
        }//则根据该图像是否为为Sub-layer Non-reference Pictures设置对应STSA类型
      }
    }
    arrangeLongtermPicturesInRPS(pcSlice, rcListPic);//设置长期参考图像
    TComRefPicListModification* refPicListModification = pcSlice->getRefPicListModification();
    refPicListModification->setRefPicListModificationFlagL0(0);
    refPicListModification->setRefPicListModificationFlagL1(0);//设置是否使用RefPicListModification(用于最终参考图像列表的得到)
    pcSlice->setNumRefIdx(REF_PIC_LIST_0,min(m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive,pcSlice->getRPS()->getNumberOfPictures()));
    pcSlice->setNumRefIdx(REF_PIC_LIST_1,min(m_pcCfg->getGOPEntry(iGOPid).m_numRefPicsActive,pcSlice->getRPS()->getNumberOfPictures()));//设置参考图像列表中的参考图像数(参考图像数不得超过配置文件中规定的大小)

    //  Set reference list
    pcSlice->setRefPicList ( rcListPic );//根据RPS得到用于帧间搜索的参考图像列表

    //  Slice info. refinement
    if ( (pcSlice->getSliceType() == B_SLICE) && (pcSlice->getNumRefIdx(REF_PIC_LIST_1) == 0) )//若为B slice 但只能单向预测
    {
      pcSlice->setSliceType ( P_SLICE );//则更改其slice类型为P
    }
    pcSlice->setEncCABACTableIdx(m_pcSliceEncoder->getEncCABACTableIdx());//为slice设置较优的cabac初始化索引

    if (pcSlice->getSliceType() == B_SLICE)//该图像为B 帧
    {
      pcSlice->setColFromL0Flag(1-uiColDir);//设置是否从list0中获得同位图像
      Bool bLowDelay = true;
      Int  iCurrPOC  = pcSlice->getPOC();
      Int iRefIdx = 0;
      //只有当前该图像的list0 list1中所有参考图像POC值大于当前图像才为低延迟编码(低延迟指图像编解码顺序与输出顺序一致 此时不用调整图像顺序)
      for (iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(REF_PIC_LIST_0) && bLowDelay; iRefIdx++)
      {
        if ( pcSlice->getRefPic(REF_PIC_LIST_0, iRefIdx)->getPOC() > iCurrPOC )
        {
          bLowDelay = false;
        }
      }
      for (iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx(REF_PIC_LIST_1) && bLowDelay; iRefIdx++)
      {
        if ( pcSlice->getRefPic(REF_PIC_LIST_1, iRefIdx)->getPOC() > iCurrPOC )
        {
          bLowDelay = false;
        }
      }

      pcSlice->setCheckLDC(bLowDelay);
    }
    else//I帧和P帧一定为低延迟编码(random access一般为高延迟)
    {
      pcSlice->setCheckLDC(true);
    }

    uiColDir = 1-uiColDir;

    //-------------------------------------------------------------
    pcSlice->setRefPOCList();//设置可得到参考图像POC的映射表

    pcSlice->setList1IdxToList0Idx();//建立list1参考图像索引到list0参考图像索引的映射表(用于广义B 帧)

    if (m_pcEncTop->getTMVPModeId() == 2)//时域用运动矢量预测模式为2
    {
      if (iGOPid == 0) // first picture in SOP (i.e. forward B)
      {
        pcSlice->setEnableTMVPFlag(0);
      }
      else
      {
        // Note: pcSlice->getColFromL0Flag() is assumed to be always 0 and getcolRefIdx() is always 0.///!!!同位图像为参考图像列表中的第一帧(离当前图像最近的一帧)
        pcSlice->setEnableTMVPFlag(1);
      }//则除去GOP中第一帧(编码顺序)不使用时域MV预测外 其他帧图像均使用TMVP
    }
    else if (m_pcEncTop->getTMVPModeId() == 1)//时域用运动矢量预测模式为1
    {
      pcSlice->setEnableTMVPFlag(1);//使用TMVP
    }
    else//TMVPModeId为0
    {
      pcSlice->setEnableTMVPFlag(0);//不使用TMVP
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////// Compress a slice
    //  Slice compression
    if (m_pcCfg->getUseASR())//若使用自适应搜索范围
    {
      m_pcSliceEncoder->setSearchRange(pcSlice);//设置自适应搜索范围
    }

    Bool bGPBcheck=false;
    if ( pcSlice->getSliceType() == B_SLICE)//若为B帧(只有B帧才可能为广义B帧)
    {
      if ( pcSlice->getNumRefIdx(RefPicList( 0 ) ) == pcSlice->getNumRefIdx(RefPicList( 1 ) ) )
      {
        bGPBcheck=true;
        Int i;
        for ( i=0; i < pcSlice->getNumRefIdx(RefPicList( 1 ) ); i++ )
        {
          if ( pcSlice->getRefPOC(RefPicList(1), i) != pcSlice->getRefPOC(RefPicList(0), i) )
          {
            bGPBcheck=false;
            break;
          }
        }
      }//只有当该图像的list0 list1中参考图像完全一样 该帧图像才为广义B帧图像
    }
    if(bGPBcheck)//若使用广义B帧
    {
      pcSlice->setMvdL1ZeroFlag(true);//MvdL1Zero标志置为真 表示list1中MVD为0
    }
    else//否则置为false
    {
      pcSlice->setMvdL1ZeroFlag(false);
    }
    pcPic->getSlice(pcSlice->getSliceIdx())->setMvdL1ZeroFlag(pcSlice->getMvdL1ZeroFlag());


    Double lambda            = 0.0;
    Int actualHeadBits       = 0;
    Int actualTotalBits      = 0;
    Int estimatedBits        = 0;
    Int tmpBitsBeforeWriting = 0;
    if ( m_pcCfg->getUseRateCtrl() ) // TODO: does this work with multiple slices and slice-segments?//若使用码率控制(回头再看)
    {
      Int frameLevel = m_pcRateCtrl->getRCSeq()->getGOPID2Level( iGOPid );
      if ( pcPic->getSlice(0)->getSliceType() == I_SLICE )
      {
        frameLevel = 0;
      }
      m_pcRateCtrl->initRCPic( frameLevel );
      estimatedBits = m_pcRateCtrl->getRCPic()->getTargetBits();

      Int sliceQP = m_pcCfg->getInitialQP();
      if ( ( pcSlice->getPOC() == 0 && m_pcCfg->getInitialQP() > 0 ) || ( frameLevel == 0 && m_pcCfg->getForceIntraQP() ) ) // QP is specified
      {
        Int    NumberBFrames = ( m_pcCfg->getGOPSize() - 1 );
        Double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05*(Double)NumberBFrames );
        Double dQPFactor     = 0.57*dLambda_scale;
        Int    SHIFT_QP      = 12;
        Int    bitdepth_luma_qp_scale = 0;
        Double qp_temp = (Double) sliceQP + bitdepth_luma_qp_scale - SHIFT_QP;
        lambda = dQPFactor*pow( 2.0, qp_temp/3.0 );
      }
      else if ( frameLevel == 0 )   // intra case, but use the model
      {
        m_pcSliceEncoder->calCostSliceI(pcPic); // TODO: This only analyses the first slice segment - what about the others?

        if ( m_pcCfg->getIntraPeriod() != 1 )   // do not refine allocated bits for all intra case
        {
          Int bits = m_pcRateCtrl->getRCSeq()->getLeftAverageBits();
          bits = m_pcRateCtrl->getRCPic()->getRefineBitsForIntra( bits );
          if ( bits < 200 )
          {
            bits = 200;
          }
          m_pcRateCtrl->getRCPic()->setTargetBits( bits );
        }

        list<TEncRCPic*> listPreviousPicture = m_pcRateCtrl->getPicList();
        m_pcRateCtrl->getRCPic()->getLCUInitTargetBits();
        lambda  = m_pcRateCtrl->getRCPic()->estimatePicLambda( listPreviousPicture, pcSlice->getSliceType());
        sliceQP = m_pcRateCtrl->getRCPic()->estimatePicQP( lambda, listPreviousPicture );
      }
      else    // normal case
      {
        list<TEncRCPic*> listPreviousPicture = m_pcRateCtrl->getPicList();
        lambda  = m_pcRateCtrl->getRCPic()->estimatePicLambda( listPreviousPicture, pcSlice->getSliceType());
        sliceQP = m_pcRateCtrl->getRCPic()->estimatePicQP( lambda, listPreviousPicture );
      }

      sliceQP = Clip3( -pcSlice->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, sliceQP );
      m_pcRateCtrl->getRCPic()->setPicEstQP( sliceQP );

      m_pcSliceEncoder->resetQP( pcPic, sliceQP, lambda );
    }

    UInt uiNumSliceSegments = 1;

    // Allocate some coders, now the number of tiles are known.
    const Int numSubstreamsColumns = (pcSlice->getPPS()->getNumTileColumnsMinus1() + 1);//列以tile为单位
    const Int numSubstreamRows     = pcSlice->getPPS()->getEntropyCodingSyncEnabledFlag() ? pcPic->getFrameHeightInCtus() : (pcSlice->getPPS()->getNumTileRowsMinus1() + 1);//若使用波前并行则行以Ctu为单位否则以tile为单位
    const Int numSubstreams        = numSubstreamRows * numSubstreamsColumns;//Substream个数
    std::vector<TComOutputBitstream> substreamsOut(numSubstreams);

    // now compress (trial encode) the various slice segments (slices, and dependent slices)
    {
      const UInt numberOfCtusInFrame=pcPic->getPicSym()->getNumberOfCtusInFrame();//一帧图像中总Ctu个数
      pcSlice->setSliceCurStartCtuTsAddr( 0 );
      pcSlice->setSliceSegmentCurStartCtuTsAddr( 0 );//设置slice和ss起始ctu位置为0(从图像中第一个slice开始处理)

      for(UInt nextCtuTsAddr = 0; nextCtuTsAddr < numberOfCtusInFrame; )//(以ss为单位)依次处理图像中所有Ctu
      {
        m_pcSliceEncoder->precompressSlice( pcPic );//得到该slice的最优QP值
        m_pcSliceEncoder->compressSlice   ( pcPic, false, false );//压缩该ss

        const UInt curSliceSegmentEnd = pcSlice->getSliceSegmentCurEndCtuTsAddr();//该ss的(末)边界Ctu位置(由给定的slice/ss参数计算得到)
        if (curSliceSegmentEnd < numberOfCtusInFrame)//若该ss的(末)边界Ctu位置小于图像最后一个Ctu位置(该图像还有Ctu未压缩)
        {
          const Bool bNextSegmentIsDependentSlice=curSliceSegmentEnd<pcSlice->getSliceCurEndCtuTsAddr();//若ss的末边界位置小于slice的末边界位置 则说明下个ss为依赖ss(在slice内但不为slice中第一个ss)
          const UInt sliceBits=pcSlice->getSliceBits();//该slice的编码比特数
          pcPic->allocateNewSlice();//图像中下一个slice
          // prepare for next slice
          pcPic->setCurrSliceIdx                    ( uiNumSliceSegments );//设置当前slice/ss索引
          m_pcSliceEncoder->setSliceIdx             ( uiNumSliceSegments   );//设置slice/ss索引
          pcSlice = pcPic->getSlice                 ( uiNumSliceSegments   );//得到图像中需要处理的slice/ss
          assert(pcSlice->getPPS()!=0);
          pcSlice->copySliceInfo                    ( pcPic->getSlice(uiNumSliceSegments-1)  );//将上个slice/ss的信息复制到当前slice/ss(复制的信息为编码相关的参数信息 因为同一帧图像中的slice/ss编码信息相同)
          pcSlice->setSliceIdx                      ( uiNumSliceSegments   );//设置当前slice/ss的索引
          if (bNextSegmentIsDependentSlice)//若为依赖ss
          {
            pcSlice->setSliceBits(sliceBits);//需要将slice中之前的有所ss编码比特数赋给当前的ss(这样才能得到整个slice的比特数 )
          }
          else//若为新的slice
          {
            pcSlice->setSliceCurStartCtuTsAddr      ( curSliceSegmentEnd );//该slice的起始位置为上个ss的(末)边界位置
            pcSlice->setSliceBits(0);//设置该(待处理)slice的编码比特数为0
          }
          pcSlice->setDependentSliceSegmentFlag(bNextSegmentIsDependentSlice);//设置依赖ss标志
          pcSlice->setSliceSegmentCurStartCtuTsAddr ( curSliceSegmentEnd );//设置该slice中起始ss的位置为上个ss末边界位置
          // TODO: optimise cabac_init during compress slice to improve multi-slice operation
          // pcSlice->setEncCABACTableIdx(m_pcSliceEncoder->getEncCABACTableIdx());
          uiNumSliceSegments ++;//ss数加1
        }
        nextCtuTsAddr = curSliceSegmentEnd;//下个ss处理的起始位置为上个ss的末边界位置
      }
    }

    duData.clear();
    pcSlice = pcPic->getSlice(0);//将pcSlice指向图像中的起始slice

    // SAO parameter estimation using non-deblocked pixels for CTU bottom and right boundary areas
    if( pcSlice->getSPS()->getUseSAO() && m_pcCfg->getSaoCtuBoundary() )//若使用SAO并用Ctu下边界 右边界未去方块像素进行SAO参数估计
    {
      m_pcSAO->getPreDBFStatistics(pcPic);//计算SAO参数
    }

    //-- Loop filter
    Bool bLFCrossTileBoundary = pcSlice->getPPS()->getLoopFilterAcrossTilesEnabledFlag();//环路滤波是否允许跨越tile边界
    m_pcLoopFilter->setCfg(bLFCrossTileBoundary);
    if ( m_pcCfg->getDeblockingFilterMetric() )//若使用去方块滤波质量评价
    {
      applyDeblockingFilterMetric(pcPic, uiNumSliceSegments);//应用该质量评价
    }
    m_pcLoopFilter->loopFilterPic( pcPic );//对该图像进行环路滤波

    /////////////////////////////////////////////////////////////////////////////////////////////////// File writing
    // Set entropy coder
    m_pcEntropyCoder->setEntropyCoder   ( m_pcCavlcCoder );

    if ( m_bSeqFirst )
    {
      // write various parameter sets
      actualTotalBits += xWriteParameterSets(accessUnit, pcSlice);//将各种参数集写入accessUnit

      // create prefix SEI messages at the beginning of the sequence
      assert(leadingSeiMessages.empty());
      xCreateIRAPLeadingSEIMessages(leadingSeiMessages, pcSlice->getSPS(), pcSlice->getPPS());//将(每个序列开始的)前缀SEI信息写入leadingSeiMessages列表

      m_bSeqFirst = false;//标志置为false 表示已经处理过
    }

    // reset presence of BP SEI indication
    m_bufferingPeriodSEIPresentInAU = false;
    // create prefix SEI associated with a picture
    xCreatePerPictureSEIMessages(iGOPid, leadingSeiMessages, nestedSeiMessages, pcSlice);//创建与每帧图像相关的前缀SEI信息

    /* use the main bitstream buffer for storing the marshalled picture */
    m_pcEntropyCoder->setBitstream(NULL);

    pcSlice = pcPic->getSlice(0);//将pcSlice指向图像中的起始slice

    if (pcSlice->getSPS()->getUseSAO())//若允许使用SAO
    {
      Bool sliceEnabled[MAX_NUM_COMPONENT];
      TComBitCounter tempBitCounter;//TComBitCounter类型表示值计算编码比特数 而不实际编码 
      tempBitCounter.resetBits();
      m_pcEncTop->getRDGoOnSbacCoder()->setBitstream(&tempBitCounter);
      m_pcSAO->initRDOCabacCoder(m_pcEncTop->getRDGoOnSbacCoder(), pcSlice);
      m_pcSAO->SAOProcess(pcPic, sliceEnabled, pcPic->getSlice(0)->getLambdas(), m_pcCfg->getTestSAODisableAtPictureLevel(), m_pcCfg->getSaoEncodingRate(), m_pcCfg->getSaoEncodingRateChroma(), m_pcCfg->getSaoCtuBoundary());//进行SAO处理(通过率失真计算得到得到最优的SAO参数(也可能为不进行SAO)并进行处理 ) 并得到slice是否进行SAO
      m_pcSAO->PCMLFDisableProcess(pcPic);//PCM模式下且不进行PCM滤波则直接用PCM像素得到重建像素值
      m_pcEncTop->getRDGoOnSbacCoder()->setBitstream(NULL);

      //assign SAO slice header
      for(Int s=0; s< uiNumSliceSegments; s++)
      {
        pcPic->getSlice(s)->setSaoEnabledFlag(CHANNEL_TYPE_LUMA, sliceEnabled[COMPONENT_Y]);
        assert(sliceEnabled[COMPONENT_Cb] == sliceEnabled[COMPONENT_Cr]);
        pcPic->getSlice(s)->setSaoEnabledFlag(CHANNEL_TYPE_CHROMA, sliceEnabled[COMPONENT_Cb]);
      }//将计算结果设置至ss(是否使用SAO)
    }

    // pcSlice is currently slice 0.
    std::size_t binCountsInNalUnits   = 0; // For implementation of cabac_zero_word stuffing (section 7.4.3.10)
    std::size_t numBytesInVclNalUnits = 0; // For implementation of cabac_zero_word stuffing (section 7.4.3.10)

    for( UInt sliceSegmentStartCtuTsAddr = 0, sliceIdxCount=0; sliceSegmentStartCtuTsAddr < pcPic->getPicSym()->getNumberOfCtusInFrame(); sliceIdxCount++, sliceSegmentStartCtuTsAddr=pcSlice->getSliceSegmentCurEndCtuTsAddr() )//依次处理图像中每个slice/ss
    {
      pcSlice = pcPic->getSlice(sliceIdxCount);
      if(sliceIdxCount > 0 && pcSlice->getSliceType()!= I_SLICE)//该slice/ss不为图像中的第一个ss 且不为I_slice(I帧不存在同位图像之说)
      {
        pcSlice->checkColRefIdx(sliceIdxCount, pcPic);//检查同一个图像中的slice的同位图像是否为同一帧图像
      }
      pcPic->setCurrSliceIdx(sliceIdxCount);
      m_pcSliceEncoder->setSliceIdx(sliceIdxCount);

      pcSlice->setRPS(pcPic->getSlice(0)->getRPS());
      pcSlice->setRPSidx(pcPic->getSlice(0)->getRPSidx());//设置该slice的RPS及RPS索引(同一帧图像中的RPS相同 故引用图像中第一个ss的RPS信息即可)

      for ( UInt ui = 0 ; ui < numSubstreams; ui++ )
      {
        substreamsOut[ui].clear();//清空substreamsOut
      }

      m_pcEntropyCoder->setEntropyCoder   ( m_pcCavlcCoder );
      m_pcEntropyCoder->resetEntropy      ( pcSlice );
      /* start slice NALunit */
      OutputNALUnit nalu( pcSlice->getNalUnitType(), pcSlice->getTLayer() );//slice NALunit
      m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);

      pcSlice->setNoRaslOutputFlag(false);//初始化NoRaslOutputFlag为false
      if (pcSlice->isIRAP())//该图像为IRAP
      {
        if (pcSlice->getNalUnitType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP && pcSlice->getNalUnitType() <= NAL_UNIT_CODED_SLICE_IDR_N_LP)//该图像为IDR or BLA
        {
          pcSlice->setNoRaslOutputFlag(true);//NoRaslOutputFlag为真 表示该图像的RASL图像无法输出
        }
        //the inference for NoOutputPriorPicsFlag
        // KJS: This cannot happen at the encoder
        if (!m_bFirst && pcSlice->isIRAP() && pcSlice->getNoRaslOutputFlag())//不为视频中的第一帧图像 为IRAP图像且该图像的RASL图像无法输出(编码端不会出现这种情况)
        {
          if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA)//该图像为CAR
          {
            pcSlice->setNoOutputPriorPicsFlag(true);//该图像之前的图像均不会输出???
          }
        }
      }

      pcSlice->setEncCABACTableIdx(m_pcSliceEncoder->getEncCABACTableIdx());////为slice设置较优的cabac初始化索引

      tmpBitsBeforeWriting = m_pcEntropyCoder->getNumberOfWrittenBits();//在编码slice头之前的编码比特数
      m_pcEntropyCoder->encodeSliceHeader(pcSlice);//编码slice头
      actualHeadBits += ( m_pcEntropyCoder->getNumberOfWrittenBits() - tmpBitsBeforeWriting );//实际编码(所有)slice头的比特数

      pcSlice->setFinalized(true);//该slice编码完成??

      pcSlice->clearSubstreamSizes(  );//清除Substream
      {
        UInt numBinsCoded = 0;
        m_pcSliceEncoder->encodeSlice(pcPic, &(substreamsOut[0]), numBinsCoded);//编码该slice 将该slice的比特流写入vector<TComOutputBitstream> substreamsOut(numSubstreams)中的对应子流
        binCountsInNalUnits+=numBinsCoded;//总的编码二进制数
      }

      {
        // Construct the final bitstream by concatenating substreams.//通过链接子流来构建最终的比特流
        // The final bitstream is either nalu.m_Bitstream or pcBitstreamRedirect;
        // Complete the slice header info.
        m_pcEntropyCoder->setEntropyCoder   ( m_pcCavlcCoder );
        m_pcEntropyCoder->setBitstream(&nalu.m_Bitstream);
        m_pcEntropyCoder->encodeTilesWPPEntryPoint( pcSlice );//Write tiles and wavefront substreams sizes for the slice header (entry points).

        // Append substreams...//substreamsOut包含整帧图像编码的子流
        TComOutputBitstream *pcOut = pcBitstreamRedirect;
        const Int numZeroSubstreamsAtStartOfSlice  = pcPic->getSubstreamForCtuAddr(pcSlice->getSliceSegmentCurStartCtuTsAddr(), false, pcSlice);//该slice的比特流在substreamsOut中的对应起始位置
        const Int numSubstreamsToCode  = pcSlice->getNumberOfSubstreamSizes()+1;//该slice比特流的所包含Substreams数
        for ( UInt ui = 0 ; ui < numSubstreamsToCode; ui++ )
        {
          pcOut->addSubstream(&(substreamsOut[ui+numZeroSubstreamsAtStartOfSlice]));//将slice所包含的子流链接至pcBitstreamRedirect
        }
      }

      // If current NALU is the first NALU of slice (containing slice header) and more NALUs exist (due to multiple dependent slices) then buffer it.
      // If current NALU is the last NALU of slice and a NALU was buffered, then (a) Write current NALU (b) Update an write buffered NALU at approproate location in NALU list.
      Bool bNALUAlignedWrittenToList    = false; // used to ensure current NALU is not written more than once to the NALU list.
      xAttachSliceDataToNalUnit(nalu, pcBitstreamRedirect);//将 pcBitstreamRedirect中的比特流添加至nalu中
      accessUnit.push_back(new NALUnitEBSP(nalu));//将nalu中比特流写入accessUnit
      actualTotalBits += UInt(accessUnit.back()->m_nalUnitData.str().size()) * 8;//总比特数需加上新加入accessUnit的比特数 (*8是因为m_nalUnitData.str().size()为字节数)
      numBytesInVclNalUnits += (std::size_t)(accessUnit.back()->m_nalUnitData.str().size());//VclNalUnits的总字节数
      bNALUAlignedWrittenToList = true;

      if (!bNALUAlignedWrittenToList)
      {
        nalu.m_Bitstream.writeAlignZero();
        accessUnit.push_back(new NALUnitEBSP(nalu));
      }

      if( ( m_pcCfg->getPictureTimingSEIEnabled() || m_pcCfg->getDecodingUnitInfoSEIEnabled() ) &&
          ( pcSlice->getSPS()->getVuiParametersPresentFlag() ) &&
          ( ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getNalHrdParametersPresentFlag() )
         || ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getVclHrdParametersPresentFlag() ) ) &&
          ( pcSlice->getSPS()->getVuiParameters()->getHrdParameters()->getSubPicCpbParamsPresentFlag() ) )
      {
          UInt numNalus = 0;
        UInt numRBSPBytes = 0;
        for (AccessUnit::const_iterator it = accessUnit.begin(); it != accessUnit.end(); it++)
        {
          numRBSPBytes += UInt((*it)->m_nalUnitData.str().size());//accessUnit的总字节数
          numNalus ++;//accessUnit的总Nalus数
        }
        duData.push_back(DUData());//新建一个解码单元数据
        duData.back().accumBitsDU = ( numRBSPBytes << 3 );
        duData.back().accumNalsDU = numNalus;//为新的DUData附上该slice的AccessUnit的Nalus数和比特数
      }
    } // end iteration over slices

    // cabac_zero_words processing
    cabac_zero_word_padding(pcSlice, pcPic, binCountsInNalUnits, numBytesInVclNalUnits, accessUnit.back()->m_nalUnitData, m_pcCfg->getCabacZeroWordPaddingEnabled());

    pcPic->compressMotion();//压缩该帧图像的运动矢量信息

    //-- For time output for each slice
    Double dEncTime = (Double)(clock()-iBeforeTime) / CLOCKS_PER_SEC;//编码该帧图像的时间(秒)

    std::string digestStr;
    if (m_pcCfg->getDecodedPictureHashSEIEnabled())//若使用DecodedPictureHashSEI
    {
      SEIDecodedPictureHash *decodedPictureHashSei = new SEIDecodedPictureHash();
      m_seiEncoder.initDecodedPictureHashSEI(decodedPictureHashSei, pcPic, digestStr, pcSlice->getSPS()->getBitDepths());
      trailingSeiMessages.push_back(decodedPictureHashSei);//将SEIDecodedPictureHash信息加入后缀SEI信息列表
    }
    xWriteTrailingSEIMessages(trailingSeiMessages, accessUnit, pcSlice->getTLayer(), pcSlice->getSPS());//将后缀SEI信息写入accessUnit末尾

    m_pcCfg->setEncodedFlag(iGOPid, true);//设置该帧图像已经被编码标志

    xCalculateAddPSNRs( isField, isTff, iGOPid, pcPic, accessUnit, rcListPic, dEncTime, snr_conversion, printFrameMSE );//计算并打印该帧图像的失真信息

    if (!digestStr.empty())
    {
      if(m_pcCfg->getDecodedPictureHashSEIEnabled() == 1)//MD5
      {
        printf(" [MD5:%s]", digestStr.c_str());
      }
      else if(m_pcCfg->getDecodedPictureHashSEIEnabled() == 2)//CRC
      {
        printf(" [CRC:%s]", digestStr.c_str());
      }
      else if(m_pcCfg->getDecodedPictureHashSEIEnabled() == 3)//Checksum
      {
        printf(" [Checksum:%s]", digestStr.c_str());
      }
    }//根据DecodedPictureHashSEI不同类型的散列函数 打印每帧图像散列后得到的字符

    if ( m_pcCfg->getUseRateCtrl() )//若使用码率控制
    {
      Double avgQP     = m_pcRateCtrl->getRCPic()->calAverageQP();
      Double avgLambda = m_pcRateCtrl->getRCPic()->calAverageLambda();
      if ( avgLambda < 0.0 )
      {
        avgLambda = lambda;
      }

      m_pcRateCtrl->getRCPic()->updateAfterPicture( actualHeadBits, actualTotalBits, avgQP, avgLambda, pcSlice->getSliceType());
      m_pcRateCtrl->getRCPic()->addToPictureLsit( m_pcRateCtrl->getPicList() );

      m_pcRateCtrl->getRCSeq()->updateAfterPic( actualTotalBits );
      if ( pcSlice->getSliceType() != I_SLICE )
      {
        m_pcRateCtrl->getRCGOP()->updateAfterPicture( actualTotalBits );
      }
      else    // for intra picture, the estimated bits are used to update the current status in the GOP
      {
        m_pcRateCtrl->getRCGOP()->updateAfterPicture( estimatedBits );
      }
    }

    xCreatePictureTimingSEI(m_pcCfg->getEfficientFieldIRAPEnabled()?effFieldIRAPMap.GetIRAPGOPid():0, leadingSeiMessages, nestedSeiMessages, duInfoSeiMessages, pcSlice, isField, duData);//创建图像的TimingSEI信息并添加值leadingSeiMessages
    if (m_pcCfg->getScalableNestingSEIEnabled())//若使用ScalableNestingSEI
    {
      xCreateScalableNestingSEI (leadingSeiMessages, nestedSeiMessages);//则创建ScalableNestingSEI并添加至leadingSeiMessages
    }
    xWriteLeadingSEIMessages(leadingSeiMessages, duInfoSeiMessages, accessUnit, pcSlice->getTLayer(), pcSlice->getSPS(), duData);//将leadingSeiMessages中前缀SEI信息写入accessUnit中
    xWriteDuSEIMessages(duInfoSeiMessages, accessUnit, pcSlice->getTLayer(), pcSlice->getSPS(), duData);//将duInfoSEI信息写入accessUnit

    pcPic->getPicYuvRec()->copyToPic(pcPicYuvRecOut);//将重建图像保存至pcPicYuvRecOut

    pcPic->setReconMark   ( true );//该帧图像已从重建过
    m_bFirst = false;//第一帧图像处理过 m_bFirst置为false
    m_iNumPicCoded++;//编码图像数+1
    m_totalCoded ++;//总编码图像数
    /* logging: insert a newline at end of picture period */
    printf("\n");
    fflush(stdout);

    if (m_pcCfg->getEfficientFieldIRAPEnabled())
    {
      iGOPid=effFieldIRAPMap.restoreGOPid(iGOPid);
    }
  } // iGOPid-loop

  delete pcBitstreamRedirect;

  assert ( (m_iNumPicCoded == iNumPicRcvd) );//图像缓存列表中的新接收的图像一定全部处理完
}

Void TEncGOP::printOutSummary(UInt uiNumAllPicCoded, Bool isField, const Bool printMSEBasedSNR, const Bool printSequenceMSE, const BitDepths &bitDepths)//输出总结信息
{
  assert (uiNumAllPicCoded == m_gcAnalyzeAll.getNumPic());


  //--CFG_KDY
  const Int rateMultiplier=(isField?2:1);
  m_gcAnalyzeAll.setFrmRate( m_pcCfg->getFrameRate()*rateMultiplier );
  m_gcAnalyzeI.setFrmRate( m_pcCfg->getFrameRate()*rateMultiplier );
  m_gcAnalyzeP.setFrmRate( m_pcCfg->getFrameRate()*rateMultiplier );
  m_gcAnalyzeB.setFrmRate( m_pcCfg->getFrameRate()*rateMultiplier );//设置帧率
  const ChromaFormat chFmt = m_pcCfg->getChromaFormatIdc();

  //-- all
  printf( "\n\nSUMMARY --------------------------------------------------------\n" );
  m_gcAnalyzeAll.printOut('a', chFmt, printMSEBasedSNR, printSequenceMSE, bitDepths);//打印总的编码质量信息

  printf( "\n\nI Slices--------------------------------------------------------\n" );
  m_gcAnalyzeI.printOut('i', chFmt, printMSEBasedSNR, printSequenceMSE, bitDepths);//打印I帧编码质量信息

  printf( "\n\nP Slices--------------------------------------------------------\n" );
  m_gcAnalyzeP.printOut('p', chFmt, printMSEBasedSNR, printSequenceMSE, bitDepths);//打印P帧编码质量信息

  printf( "\n\nB Slices--------------------------------------------------------\n" );
  m_gcAnalyzeB.printOut('b', chFmt, printMSEBasedSNR, printSequenceMSE, bitDepths);//打印B帧编码质量信息

  if (!m_pcCfg->getSummaryOutFilename().empty())//若配置文件中存在保存总结信息的文件
  {
    m_gcAnalyzeAll.printSummary(chFmt, printSequenceMSE, bitDepths, m_pcCfg->getSummaryOutFilename());//将总结信息写入给定文件
  }

  if (!m_pcCfg->getSummaryPicFilenameBase().empty())//若配置文件中存在给定的SummaryPicFilenameBase文件
  {
    m_gcAnalyzeI.printSummary(chFmt, printSequenceMSE, bitDepths, m_pcCfg->getSummaryPicFilenameBase()+"I.txt");
    m_gcAnalyzeP.printSummary(chFmt, printSequenceMSE, bitDepths, m_pcCfg->getSummaryPicFilenameBase()+"P.txt");
    m_gcAnalyzeB.printSummary(chFmt, printSequenceMSE, bitDepths, m_pcCfg->getSummaryPicFilenameBase()+"B.txt");//则将I P B帧的总结信息分别写入对应文件
  }

  if(isField)//若为场编码
  {
    //-- interlaced summary
    m_gcAnalyzeAll_in.setFrmRate( m_pcCfg->getFrameRate());
    m_gcAnalyzeAll_in.setBits(m_gcAnalyzeAll.getBits());
    // prior to the above statement, the interlace analyser does not contain the correct total number of bits.

    printf( "\n\nSUMMARY INTERLACED ---------------------------------------------\n" );
    m_gcAnalyzeAll_in.printOut('a', chFmt, printMSEBasedSNR, printSequenceMSE, bitDepths);

    if (!m_pcCfg->getSummaryOutFilename().empty())
    {
      m_gcAnalyzeAll_in.printSummary(chFmt, printSequenceMSE, bitDepths, m_pcCfg->getSummaryOutFilename());
    }
  }//场编码总结信息

  printf("\nRVM: %.3lf\n" , xCalculateRVM());//输出RVM信息
}

Void TEncGOP::preLoopFilterPicAll( TComPic* pcPic, UInt64& ruiDist )
{
  Bool bCalcDist = false;
  m_pcLoopFilter->setCfg(m_pcCfg->getLFCrossTileBoundaryFlag());
  m_pcLoopFilter->loopFilterPic( pcPic );//对该图像环路滤波

  if (!bCalcDist)
  {
    ruiDist = xFindDistortionFrame(pcPic->getPicYuvOrg(), pcPic->getPicYuvRec(), pcPic->getPicSym()->getSPS().getBitDepths());//计算(环路滤波后的)重建图像与原始图像间的失真
  }
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================


Void TEncGOP::xInitGOP( Int iPOCLast, Int iNumPicRcvd, Bool isField )//初始化m_iGopSize
{
  assert( iNumPicRcvd > 0 );
  //  Exception for the first frames
  if ( ( isField && (iPOCLast == 0 || iPOCLast == 1) ) || (!isField  && (iPOCLast == 0))  )//为视频流中第一帧图像
  {
    m_iGopSize    = 1;
  }
  else
  {
    m_iGopSize    = m_pcCfg->getGOPSize();
  }
  assert (m_iGopSize > 0);

  return;
}


Void TEncGOP::xGetBuffer( TComList<TComPic*>&      rcListPic,
                         TComList<TComPicYuv*>&    rcListPicYuvRecOut,
                         Int                       iNumPicRcvd,
                         Int                       iTimeOffset,
                         TComPic*&                 rpcPic,
                         TComPicYuv*&              rpcPicYuvRecOut,
                         Int                       pocCurr,
                         Bool                      isField)//分别在重建图像列表 图像缓存列表中得到保存重建图像 读取待压缩图像的位置
{
  Int i;
  //  Rec. output //(rpcPicYuvRecOut为输出地址 即得到重建图像将保存在该位置)
  TComList<TComPicYuv*>::iterator     iterPicYuvRec = rcListPicYuvRecOut.end();

  if (isField && pocCurr > 1 && m_iGopSize!=1)
  {
    iTimeOffset--;
  }

  for ( i = 0; i < (iNumPicRcvd - iTimeOffset + 1); i++ )//找到重建输出图像在重建图像列表中(保存)的位置
  {
    iterPicYuvRec--;
  }

  rpcPicYuvRecOut = *(iterPicYuvRec);

  //  Current pic.(rpcPic为读取地址 即读取该地址的图像原始数据 用以后续的压缩)
  TComList<TComPic*>::iterator        iterPic       = rcListPic.begin();
  while (iterPic != rcListPic.end())//根据pocCurr值找到图像缓存列表中的该帧图像
  {
    rpcPic = *(iterPic);
    rpcPic->setCurrSliceIdx(0);
    if (rpcPic->getPOC() == pocCurr)
    {
      break;
    }
    iterPic++;
  }

  assert (rpcPic != NULL);
  assert (rpcPic->getPOC() == pocCurr);

  return;
}

UInt64 TEncGOP::xFindDistortionFrame (TComPicYuv* pcPic0, TComPicYuv* pcPic1, const BitDepths &bitDepths)//计算两帧图像间的失真 Dis=sum((x1-x2)^2) Sum of Square Error
{
  UInt64  uiTotalDiff = 0;

  for(Int chan=0; chan<pcPic0 ->getNumberValidComponents(); chan++)
  {
    const ComponentID ch=ComponentID(chan);
    Pel*  pSrc0   = pcPic0 ->getAddr(ch);
    Pel*  pSrc1   = pcPic1 ->getAddr(ch);
    UInt  uiShift     = 2 * DISTORTION_PRECISION_ADJUSTMENT(bitDepths.recon[toChannelType(ch)]-8);

    const Int   iStride = pcPic0->getStride(ch);
    const Int   iWidth  = pcPic0->getWidth(ch);
    const Int   iHeight = pcPic0->getHeight(ch);

    for(Int y = 0; y < iHeight; y++ )//依次两帧图像中的所有像素的差值
    {
      for(Int x = 0; x < iWidth; x++ )
      {
        Intermediate_Int iTemp = pSrc0[x] - pSrc1[x];
        uiTotalDiff += UInt64((iTemp*iTemp) >> uiShift);//计算总失真(差值的平方相加)SSE
      }
      pSrc0 += iStride;
      pSrc1 += iStride;
    }
  }

  return uiTotalDiff;//返回总失真
}

Void TEncGOP::xCalculateAddPSNRs( const Bool isField, const Bool isFieldTopFieldFirst, const Int iGOPid, TComPic* pcPic, const AccessUnit&accessUnit, TComList<TComPic*> &rcListPic, const Double dEncTime, const InputColourSpaceConversion snr_conversion, const Bool printFrameMSE )
{//isFieldTopFieldFirst表明将一帧图像分为两场时是顶场在前还是底场在前(以下分析假设其为真(先顶场再底场)便于分析!! 其值为假也类似)
  xCalculateAddPSNR( pcPic, pcPic->getPicYuvRec(), accessUnit, dEncTime, snr_conversion, printFrameMSE );////计算并打印给定图像/场的失真信息

  //In case of field coding, compute the interlaced PSNR for both fields
  if(isField)//若为场编码 则计算两场的失真信息均要计算(一帧图像分为顶场和底场)
  {
    Bool bothFieldsAreEncoded = false;
    Int correspondingFieldPOC = pcPic->getPOC();
    Int currentPicGOPPoc = m_pcCfg->getGOPEntry(iGOPid).m_POC;
    if(pcPic->getPOC() == 0)//第一个顶场
    {
      // particular case for POC 0 and 1.
      // If they are not encoded first and separately from other pictures, we need to change this
      // POC 0 is always encoded first then POC 1 is encoded
      bothFieldsAreEncoded = false;//只编码了第一个顶场 所以一帧图像的两场不可能每编码完
    }
    else if(pcPic->getPOC() == 1)//第一个底场
    {
      // if we are at POC 1, POC 0 has been encoded for sure
      correspondingFieldPOC = 0;//与第一个底场相对应的为一个顶场(先顶场再底场 故同一帧图像的底场为顶场POC值+1)
      bothFieldsAreEncoded = true;//第一帧图像的底场场编码完 顶场肯定也编码完
    }
    else
    {
      if(pcPic->getPOC()%2 == 1)//若为底场
      {
        correspondingFieldPOC -= 1; // all odd POC are associated with the preceding even POC (e.g poc 1 is associated to poc 0)
        currentPicGOPPoc      -= 1;//则该帧图像的顶场为底场减一
      }
      else//若为顶场
      {
        correspondingFieldPOC += 1; // all even POC are associated with the following odd POC (e.g poc 0 is associated to poc 1)
        currentPicGOPPoc      += 1;//则该帧图像的顶场为底场加一
      }
      for(Int i = 0; i < m_iGopSize; i ++)//在GOP中找到该帧图像的另一场 
      {
        if(m_pcCfg->getGOPEntry(i).m_POC == currentPicGOPPoc)
        {
          bothFieldsAreEncoded = m_pcCfg->getGOPEntry(i).m_isEncoded;//若该帧图像的另一场已编码则该帧图像的两场均编码完成
          break;
        }
      }
    }

    if(bothFieldsAreEncoded)//若该帧图像的两场均编码完成
    {
      //get complementary top field
      TComList<TComPic*>::iterator   iterPic = rcListPic.begin();
      while ((*iterPic)->getPOC() != correspondingFieldPOC)//在图像缓存列表中找到该帧图像的另一场
      {
        iterPic ++;
      }
      TComPic* correspondingFieldPic = *(iterPic);

      if( (pcPic->isTopField() && isFieldTopFieldFirst) || (!pcPic->isTopField() && !isFieldTopFieldFirst))//根据给定的场是否为顶场及是否顶场在前计算该帧图像(两场)的失真信息
      {
        xCalculateInterlacedAddPSNR(pcPic, correspondingFieldPic, pcPic->getPicYuvRec(), correspondingFieldPic->getPicYuvRec(), snr_conversion, printFrameMSE );
      }
      else
      {
        xCalculateInterlacedAddPSNR(correspondingFieldPic, pcPic, correspondingFieldPic->getPicYuvRec(), pcPic->getPicYuvRec(), snr_conversion, printFrameMSE );
      }
    }
  }
}

Void TEncGOP::xCalculateAddPSNR( TComPic* pcPic, TComPicYuv* pcPicD, const AccessUnit& accessUnit, Double dEncTime, const InputColourSpaceConversion conversion, const Bool printFrameMSE )//计算并打印给定图像的失真信息
{
  Double  dPSNR[MAX_NUM_COMPONENT];

  for(Int i=0; i<MAX_NUM_COMPONENT; i++)
  {
    dPSNR[i]=0.0;//初始化PSNR为0
  }

  TComPicYuv cscd;
  if (conversion!=IPCOLOURSPACE_UNCHANGED)//若需要转换色彩空间 则转换为对应色彩空间
  {
    cscd.create(pcPicD->getWidth(COMPONENT_Y), pcPicD->getHeight(COMPONENT_Y), pcPicD->getChromaFormat(), pcPicD->getWidth(COMPONENT_Y), pcPicD->getHeight(COMPONENT_Y), 0, false);
    TVideoIOYuv::ColourSpaceConvert(*pcPicD, cscd, conversion, false);
  }
  TComPicYuv &picd=(conversion==IPCOLOURSPACE_UNCHANGED)?*pcPicD : cscd;//重建图像数据

  //===== calculate PSNR =====
  Double MSEyuvframe[MAX_NUM_COMPONENT] = {0, 0, 0};//初始化MSE为0

  for(Int chan=0; chan<pcPicD->getNumberValidComponents(); chan++)
  {
    const ComponentID ch=ComponentID(chan);
    const TComPicYuv *pOrgPicYuv =(conversion!=IPCOLOURSPACE_UNCHANGED) ? pcPic ->getPicYuvTrueOrg() : pcPic ->getPicYuvOrg();//原始图像数据
    const Pel*  pOrg       = pOrgPicYuv->getAddr(ch);
    const Int   iOrgStride = pOrgPicYuv->getStride(ch);
    Pel*  pRec             = picd.getAddr(ch);
    const Int   iRecStride = picd.getStride(ch);
    const Int   iWidth  = pcPicD->getWidth (ch) - (m_pcEncTop->getPad(0) >> pcPic->getComponentScaleX(ch));
    const Int   iHeight = pcPicD->getHeight(ch) - ((m_pcEncTop->getPad(1) >> (pcPic->isField()?1:0)) >> pcPic->getComponentScaleY(ch));

    Int   iSize   = iWidth*iHeight;

    UInt64 uiSSDtemp=0;
    for(Int y = 0; y < iHeight; y++ )//遍历图像中的所有像素
    {
      for(Int x = 0; x < iWidth; x++ )
      {
        Intermediate_Int iDiff = (Intermediate_Int)( pOrg[x] - pRec[x] );
        uiSSDtemp   += iDiff * iDiff;
      }
      pOrg += iOrgStride;
      pRec += iRecStride;
    }//计算SSD
    const Int maxval = 255 << (pcPic->getPicSym()->getSPS().getBitDepth(toChannelType(ch)) - 8);
    const Double fRefValue = (Double) maxval * maxval * iSize;//需*iSize 是因为uiSSDtemp为所有像素的失真之和
    dPSNR[ch]         = ( uiSSDtemp ? 10.0 * log10( fRefValue / (Double)uiSSDtemp ) : 999.99 );//计算PSNR PSNR=10*log10((2^n-1)^2/MSE)
    MSEyuvframe[ch]   = (Double)uiSSDtemp/(iSize);//计算MSE
  }


  /* calculate the size of the access unit, excluding:
   *  - any AnnexB contributions (start_code_prefix, zero_byte, etc.,)
   *  - SEI NAL units
   */
  UInt numRBSPBytes = 0;
  for (AccessUnit::const_iterator it = accessUnit.begin(); it != accessUnit.end(); it++)//遍历AccessUnit中的每个NALUnit
  {
    UInt numRBSPBytes_nal = UInt((*it)->m_nalUnitData.str().size());//NAL的数据大小
    if (m_pcCfg->getSummaryVerboseness() > 0)//若配置文件中SummaryVerboseness大于0
    {
      printf("*** %6s numBytesInNALunit: %u\n", nalUnitTypeToString((*it)->m_nalUnitType), numRBSPBytes_nal);//打印其nal类型及字节数
    }
    if ((*it)->m_nalUnitType != NAL_UNIT_PREFIX_SEI && (*it)->m_nalUnitType != NAL_UNIT_SUFFIX_SEI)//若不为SEI
    {
      numRBSPBytes += numRBSPBytes_nal;//计算总的RBSP字节
    }
  }

  UInt uibits = numRBSPBytes * 8;
  m_vRVM_RP.push_back( uibits );

  //===== add PSNR =====
  m_gcAnalyzeAll.addResult (dPSNR, (Double)uibits, MSEyuvframe);//将该帧图像的失真信息添加至全部图像的总结信息中
  TComSlice*  pcSlice = pcPic->getSlice(0);
  if (pcSlice->isIntra())
  {
    m_gcAnalyzeI.addResult (dPSNR, (Double)uibits, MSEyuvframe);//若该帧为I帧 则将该帧图像的失真信息添加至I帧图像的总结信息中
  }
  if (pcSlice->isInterP())
  {
    m_gcAnalyzeP.addResult (dPSNR, (Double)uibits, MSEyuvframe);//若该帧为P帧 则将该帧图像的失真信息添加至P帧图像的总结信息中
  }
  if (pcSlice->isInterB())
  {
    m_gcAnalyzeB.addResult (dPSNR, (Double)uibits, MSEyuvframe);//若该帧为B帧 则将该帧图像的失真信息添加至B帧图像的总结信息中
  }

  Char c = (pcSlice->isIntra() ? 'I' : pcSlice->isInterP() ? 'P' : 'B');//该帧图像的类型
  if (!pcSlice->isReferenced())
  {
    c += 32;
  }//图像不用作参考 则变为小写

#if ADAPTIVE_QP_SELECTION
  printf("POC %4d TId: %1d ( %c-SLICE, nQP %d QP %d ) %10d bits",
         pcSlice->getPOC(),
         pcSlice->getTLayer(),
         c,
         pcSlice->getSliceQpBase(),
         pcSlice->getSliceQp(),
         uibits );//打印该帧图像的POC值 时域层 图像类型  QpBase(自适应Qp值时会使用到) QP值 图像比特数
#else
  printf("POC %4d TId: %1d ( %c-SLICE, QP %d ) %10d bits",
         pcSlice->getPOC()-pcSlice->getLastIDR(),
         pcSlice->getTLayer(),
         c,
         pcSlice->getSliceQp(),
         uibits );
#endif

  printf(" [Y %6.4lf dB    U %6.4lf dB    V %6.4lf dB]", dPSNR[COMPONENT_Y], dPSNR[COMPONENT_Cb], dPSNR[COMPONENT_Cr] );//打印图像的PSNR信息
  if (printFrameMSE)
  {
    printf(" [Y MSE %6.4lf  U MSE %6.4lf  V MSE %6.4lf]", MSEyuvframe[COMPONENT_Y], MSEyuvframe[COMPONENT_Cb], MSEyuvframe[COMPONENT_Cr] );//若需要打印图像的MSE 则打印打印图像的MSE
  }
  printf(" [ET %5.0f ]", dEncTime );//打印编码该帧图像的时间

  for (Int iRefList = 0; iRefList < 2; iRefList++)
  {
    printf(" [L%d ", iRefList);
    for (Int iRefIndex = 0; iRefIndex < pcSlice->getNumRefIdx(RefPicList(iRefList)); iRefIndex++)
    {
      printf ("%d ", pcSlice->getRefPOC(RefPicList(iRefList), iRefIndex)-pcSlice->getLastIDR());
    }
    printf("]");
  }//打印该帧图像的参考图像列表中参考图像与该帧图像邻近的IDR图像的POC差值

  cscd.destroy();
}

Void TEncGOP::xCalculateInterlacedAddPSNR( TComPic* pcPicOrgFirstField, TComPic* pcPicOrgSecondField,
                                           TComPicYuv* pcPicRecFirstField, TComPicYuv* pcPicRecSecondField,
                                           const InputColourSpaceConversion conversion, const Bool printFrameMSE )//计算场编码时的失真信息 过程同上 不在叙述
{
  const TComSPS &sps=pcPicOrgFirstField->getPicSym()->getSPS();
  Double  dPSNR[MAX_NUM_COMPONENT];
  TComPic    *apcPicOrgFields[2]={pcPicOrgFirstField, pcPicOrgSecondField};
  TComPicYuv *apcPicRecFields[2]={pcPicRecFirstField, pcPicRecSecondField};

  for(Int i=0; i<MAX_NUM_COMPONENT; i++)
  {
    dPSNR[i]=0.0;
  }

  TComPicYuv cscd[2 /* first/second field */];
  if (conversion!=IPCOLOURSPACE_UNCHANGED)
  {
    for(UInt fieldNum=0; fieldNum<2; fieldNum++)
    {
      TComPicYuv &reconField=*(apcPicRecFields[fieldNum]);
      cscd[fieldNum].create(reconField.getWidth(COMPONENT_Y), reconField.getHeight(COMPONENT_Y), reconField.getChromaFormat(), reconField.getWidth(COMPONENT_Y), reconField.getHeight(COMPONENT_Y), 0, false);
      TVideoIOYuv::ColourSpaceConvert(reconField, cscd[fieldNum], conversion, false);
      apcPicRecFields[fieldNum]=cscd+fieldNum;
    }
  }

  //===== calculate PSNR =====
  Double MSEyuvframe[MAX_NUM_COMPONENT] = {0, 0, 0};

  assert(apcPicRecFields[0]->getChromaFormat()==apcPicRecFields[1]->getChromaFormat());
  const UInt numValidComponents=apcPicRecFields[0]->getNumberValidComponents();

  for(Int chan=0; chan<numValidComponents; chan++)
  {
    const ComponentID ch=ComponentID(chan);
    assert(apcPicRecFields[0]->getWidth(ch)==apcPicRecFields[1]->getWidth(ch));
    assert(apcPicRecFields[0]->getHeight(ch)==apcPicRecFields[1]->getHeight(ch));

    UInt64 uiSSDtemp=0;
    const Int   iWidth  = apcPicRecFields[0]->getWidth (ch) - (m_pcEncTop->getPad(0) >> apcPicRecFields[0]->getComponentScaleX(ch));
    const Int   iHeight = apcPicRecFields[0]->getHeight(ch) - ((m_pcEncTop->getPad(1) >> 1) >> apcPicRecFields[0]->getComponentScaleY(ch));

    Int   iSize   = iWidth*iHeight;

    for(UInt fieldNum=0; fieldNum<2; fieldNum++)
    {
      TComPic *pcPic=apcPicOrgFields[fieldNum];
      TComPicYuv *pcPicD=apcPicRecFields[fieldNum];

      const Pel*  pOrg    = (conversion!=IPCOLOURSPACE_UNCHANGED) ? pcPic ->getPicYuvTrueOrg()->getAddr(ch) : pcPic ->getPicYuvOrg()->getAddr(ch);
      Pel*  pRec    = pcPicD->getAddr(ch);
      const Int   iStride = pcPicD->getStride(ch);


      for(Int y = 0; y < iHeight; y++ )
      {
        for(Int x = 0; x < iWidth; x++ )
        {
          Intermediate_Int iDiff = (Intermediate_Int)( pOrg[x] - pRec[x] );
          uiSSDtemp   += iDiff * iDiff;
        }
        pOrg += iStride;
        pRec += iStride;
      }
    }
    const Int maxval = 255 << (sps.getBitDepth(toChannelType(ch)) - 8);
    const Double fRefValue = (Double) maxval * maxval * iSize*2;
    dPSNR[ch]         = ( uiSSDtemp ? 10.0 * log10( fRefValue / (Double)uiSSDtemp ) : 999.99 );
    MSEyuvframe[ch]   = (Double)uiSSDtemp/(iSize*2);
  }

  UInt uibits = 0; // the number of bits for the pair is not calculated here - instead the overall total is used elsewhere.

  //===== add PSNR =====
  m_gcAnalyzeAll_in.addResult (dPSNR, (Double)uibits, MSEyuvframe);

  printf("\n                                      Interlaced frame %d: [Y %6.4lf dB    U %6.4lf dB    V %6.4lf dB]", pcPicOrgSecondField->getPOC()/2 , dPSNR[COMPONENT_Y], dPSNR[COMPONENT_Cb], dPSNR[COMPONENT_Cr] );
  if (printFrameMSE)
  {
    printf(" [Y MSE %6.4lf  U MSE %6.4lf  V MSE %6.4lf]", MSEyuvframe[COMPONENT_Y], MSEyuvframe[COMPONENT_Cb], MSEyuvframe[COMPONENT_Cr] );
  }

  for(UInt fieldNum=0; fieldNum<2; fieldNum++)
  {
    cscd[fieldNum].destroy();
  }
}

/** Function for deciding the nal_unit_type.
 * \param pocCurr POC of the current picture
 * \param lastIDR  POC of the last IDR picture
 * \param isField  true to indicate field coding
 * \returns the NAL unit type of the picture
 * This function checks the configuration and returns the appropriate nal_unit_type for the picture.
 */
NalUnitType TEncGOP::getNalUnitType(Int pocCurr, Int lastIDR, Bool isField)//得到给定图像的nal_unit类型 //该部分可参阅<HEVC book> section 2.2.2
{
  if (pocCurr == 0)//当前图像POC为0 为视频流中第一帧
  {
    return NAL_UNIT_CODED_SLICE_IDR_W_RADL;//IDR May have leading pictures 
  }

  if(m_pcCfg->getEfficientFieldIRAPEnabled() && isField && pocCurr == 1)
  {
    // to avoid the picture becoming an IRAP
    return NAL_UNIT_CODED_SLICE_TRAIL_R;
  }

  if(m_pcCfg->getDecodingRefreshType() != 3 && (pocCurr - isField) % m_pcCfg->getIntraPeriod() == 0)//当前图像正处在帧内预测帧
  {
    if (m_pcCfg->getDecodingRefreshType() == 1)//指明random access point为CRA
    {
      return NAL_UNIT_CODED_SLICE_CRA;
    }
    else if (m_pcCfg->getDecodingRefreshType() == 2)//指明random access point为IDR
    {
      return NAL_UNIT_CODED_SLICE_IDR_W_RADL;
    }
  }
  if(m_pocCRA>0)//当前图像存在邻近的CRA图像
  {
    if(pocCurr<m_pocCRA)//当前图像为该CRA的leading picture
    {
      // All leading pictures are being marked as TFD pictures here since current encoder uses all
      // reference pictures while encoding leading pictures. An encoder can ensure that a leading
      // picture can be still decodable when random accessing to a CRA/CRANT/BLA/BLANT picture by
      // controlling the reference pictures used for encoding that leading picture. Such a leading
      // picture need not be marked as a TFD picture.
      return NAL_UNIT_CODED_SLICE_RASL_R;//CRA的LP允许参考解码顺序在该CRA之前的图像 故当该CRA为random access point时当前图像无法顺利解码 所以其类型为RASL
    }
  }
  if (lastIDR>0)//当前图像存在邻近的IDR图像(该IDR图像为最新(编码顺序)的IDR)
  {
    if (pocCurr < lastIDR)//当前图像为该IDR的leading picture
    {
      return NAL_UNIT_CODED_SLICE_RADL_R;//IDR的LP无法参考解码顺序在该IDR之前的图像 故当该IDR为random access point时当前图像可以顺利解码 所以其类型为RADL
    }
  }
  return NAL_UNIT_CODED_SLICE_TRAIL_R;//若均不为以上图像类型 则为Ordinary Trailing (TRAIL) Pictures
}

Double TEncGOP::xCalculateRVM()
{
  Double dRVM = 0;

  if( m_pcCfg->getGOPSize() == 1 && m_pcCfg->getIntraPeriod() != 1 && m_pcCfg->getFramesToBeEncoded() > RVM_VCEGAM10_M * 2 )
  {
    // calculate RVM only for lowdelay configurations
    std::vector<Double> vRL , vB;
    size_t N = m_vRVM_RP.size();
    vRL.resize( N );
    vB.resize( N );

    Int i;
    Double dRavg = 0 , dBavg = 0;
    vB[RVM_VCEGAM10_M] = 0;
    for( i = RVM_VCEGAM10_M + 1 ; i < N - RVM_VCEGAM10_M + 1 ; i++ )
    {
      vRL[i] = 0;
      for( Int j = i - RVM_VCEGAM10_M ; j <= i + RVM_VCEGAM10_M - 1 ; j++ )
      {
        vRL[i] += m_vRVM_RP[j];
      }
      vRL[i] /= ( 2 * RVM_VCEGAM10_M );
      vB[i] = vB[i-1] + m_vRVM_RP[i] - vRL[i];
      dRavg += m_vRVM_RP[i];
      dBavg += vB[i];
    }

    dRavg /= ( N - 2 * RVM_VCEGAM10_M );
    dBavg /= ( N - 2 * RVM_VCEGAM10_M );

    Double dSigamB = 0;
    for( i = RVM_VCEGAM10_M + 1 ; i < N - RVM_VCEGAM10_M + 1 ; i++ )
    {
      Double tmp = vB[i] - dBavg;
      dSigamB += tmp * tmp;
    }
    dSigamB = sqrt( dSigamB / ( N - 2 * RVM_VCEGAM10_M ) );

    Double f = sqrt( 12.0 * ( RVM_VCEGAM10_M - 1 ) / ( RVM_VCEGAM10_M + 1 ) );

    dRVM = dSigamB / dRavg * f;
  }

  return( dRVM );
}

/** Attaches the input bitstream to the stream in the output NAL unit
    Updates rNalu to contain concatenated bitstream. rpcBitstreamRedirect is cleared at the end of this function call.
 *  \param codedSliceData contains the coded slice data (bitstream) to be concatenated to rNalu
 *  \param rNalu          target NAL unit
 */
Void TEncGOP::xAttachSliceDataToNalUnit (OutputNALUnit& rNalu, TComOutputBitstream* codedSliceData)//将 codedSliceData中的比特流添加至nalu中
{
  // Byte-align
  rNalu.m_Bitstream.writeByteAlignment();   // Slice header byte-alignment

  // Perform bitstream concatenation
  if (codedSliceData->getNumberOfWrittenBits() > 0)//编码的slice数据大于0比特
  {
    rNalu.m_Bitstream.addSubstream(codedSliceData);//将codedSliceData中的比特流添加至nalu中
  }

  m_pcEntropyCoder->setBitstream(&rNalu.m_Bitstream);

  codedSliceData->clear();//将codedSliceData清空 下个slice还需使用
}

// Function will arrange the long-term pictures in the decreasing order of poc_lsb_lt,
// and among the pictures with the same lsb, it arranges them in increasing delta_poc_msb_cycle_lt value
Void TEncGOP::arrangeLongtermPicturesInRPS(TComSlice *pcSlice, TComList<TComPic*>& rcListPic)
{
  if(pcSlice->getRPS()->getNumberOfLongtermPictures() == 0)//若RPS中不存在长期参考图像 则直接返回
  {
    return;
  }
  // we can only modify the local RPS!
  assert (pcSlice->getRPSidx()==-1);//只有本地RPS需要处理
  TComReferencePictureSet *rps = pcSlice->getLocalRPS();//得到该slice的RPS

  // Arrange long-term reference pictures in the correct order of LSB and MSB,
  // and assign values for pocLSBLT and MSB present flag
  Int longtermPicsPoc[MAX_NUM_REF_PICS], longtermPicsLSB[MAX_NUM_REF_PICS], indices[MAX_NUM_REF_PICS];
  Int longtermPicsMSB[MAX_NUM_REF_PICS];
  Bool mSBPresentFlag[MAX_NUM_REF_PICS];
  ::memset(longtermPicsPoc, 0, sizeof(longtermPicsPoc));    // Store POC values of LTRP//存储长期参考图像的POC值
  ::memset(longtermPicsLSB, 0, sizeof(longtermPicsLSB));    // Store POC LSB values of LTRP//存储长期参考图像的LSB
  ::memset(longtermPicsMSB, 0, sizeof(longtermPicsMSB));    // Store POC LSB values of LTRP//存储长期参考图像的MSB
  ::memset(indices        , 0, sizeof(indices));            // Indices to aid in tracking sorted LTRPs
  ::memset(mSBPresentFlag , 0, sizeof(mSBPresentFlag));     // Indicate if MSB needs to be present

  // Get the long-term reference pictures
  Int offset = rps->getNumberOfNegativePictures() + rps->getNumberOfPositivePictures();
  Int i, ctr = 0;
  Int maxPicOrderCntLSB = 1 << pcSlice->getSPS()->getBitsForPOC();
  for(i = rps->getNumberOfPictures() - 1; i >= offset; i--, ctr++)//计算RPS中的所有长期参考图像的POC LSB MSB
  {
    longtermPicsPoc[ctr] = rps->getPOC(i);                                  // LTRP POC
    longtermPicsLSB[ctr] = getLSB(longtermPicsPoc[ctr], maxPicOrderCntLSB); // LTRP POC LSB
    indices[ctr]      = i;
    longtermPicsMSB[ctr] = longtermPicsPoc[ctr] - longtermPicsLSB[ctr];
  }
  Int numLongPics = rps->getNumberOfLongtermPictures();
  assert(ctr == numLongPics);

  // Arrange pictures in decreasing order of MSB;
  for(i = 0; i < numLongPics; i++)//按MSB递减的顺序排列长期参考图像
  {
    for(Int j = 0; j < numLongPics - 1; j++)
    {
      if(longtermPicsMSB[j] < longtermPicsMSB[j+1])
      {
        std::swap(longtermPicsPoc[j], longtermPicsPoc[j+1]);
        std::swap(longtermPicsLSB[j], longtermPicsLSB[j+1]);
        std::swap(longtermPicsMSB[j], longtermPicsMSB[j+1]);
        std::swap(indices[j]        , indices[j+1]        );//交换位置
      }
    }
  }

  for(i = 0; i < numLongPics; i++)//遍历所有长期参考图像
  {
    // Check if MSB present flag should be enabled.
    // Check if the buffer contains any pictures that have the same LSB.
    TComList<TComPic*>::iterator  iterPic = rcListPic.begin();
    TComPic*                      pcPic;
    while ( iterPic != rcListPic.end() )//遍历图像缓存列表中的图像
    {
      pcPic = *iterPic;
      if( (getLSB(pcPic->getPOC(), maxPicOrderCntLSB) == longtermPicsLSB[i])   &&     // Same LSB
                                      (pcPic->getSlice(0)->isReferenced())     &&    // Reference picture
                                        (pcPic->getPOC() != longtermPicsPoc[i])    )  // Not the LTRP itself
      {//这两帧图像同作为参考图像且具有相同的LSB但不为同一帧图像 则RPS中该长期参考图像还需用MSB指明 (因为只用LSB已经无法指明该长期参考图像在图像缓存列表中的唯一性)
        mSBPresentFlag[i] = true;
        break;
      }
      iterPic++;
    }
  }

  // tempArray for usedByCurr flag
  Bool tempArray[MAX_NUM_REF_PICS]; ::memset(tempArray, 0, sizeof(tempArray));
  for(i = 0; i < numLongPics; i++)
  {
    tempArray[i] = rps->getUsed(indices[i]);//指明长期参考图像是否被当前图像使用
  }
  // Now write the final values;
  ctr = 0;
  Int currMSB = 0, currLSB = 0;
  // currPicPoc = currMSB + currLSB
  currLSB = getLSB(pcSlice->getPOC(), maxPicOrderCntLSB);
  currMSB = pcSlice->getPOC() - currLSB;

  for(i = rps->getNumberOfPictures() - 1; i >= offset; i--, ctr++)//按指定的长期参考图像顺序给设置RPS
  {
    rps->setPOC                   (i, longtermPicsPoc[ctr]);
    rps->setDeltaPOC              (i, - pcSlice->getPOC() + longtermPicsPoc[ctr]);
    rps->setUsed                  (i, tempArray[ctr]);
    rps->setPocLSBLT              (i, longtermPicsLSB[ctr]);
    rps->setDeltaPocMSBCycleLT    (i, (currMSB - (longtermPicsPoc[ctr] - longtermPicsLSB[ctr])) / maxPicOrderCntLSB);
    rps->setDeltaPocMSBPresentFlag(i, mSBPresentFlag[ctr]);

    assert(rps->getDeltaPocMSBCycleLT(i) >= 0);   // Non-negative value
  }
  for(i = rps->getNumberOfPictures() - 1, ctr = 1; i >= offset; i--, ctr++)
  {
    for(Int j = rps->getNumberOfPictures() - 1 - ctr; j >= offset; j--)
    {
      // Here at the encoder we know that we have set the full POC value for the LTRPs, hence we
      // don't have to check the MSB present flag values for this constraint.
      assert( rps->getPOC(i) != rps->getPOC(j) ); // If assert fails, LTRP entry repeated in RPS!!!
    }
  }//确保RPS中不存在相同的图像
}

Void TEncGOP::applyDeblockingFilterMetric( TComPic* pcPic, UInt uiNumSlices )
{
  TComPicYuv* pcPicYuvRec = pcPic->getPicYuvRec();//重建图像
  Pel* Rec    = pcPicYuvRec->getAddr(COMPONENT_Y);//亮度分量重建像素起始地址
  Pel* tempRec = Rec;
  Int  stride = pcPicYuvRec->getStride(COMPONENT_Y);
  UInt log2maxTB = pcPic->getSlice(0)->getSPS()->getQuadtreeTULog2MaxSize();
  UInt maxTBsize = (1<<log2maxTB);//最大Tb块
  const UInt minBlockArtSize = 8;
  const UInt picWidth = pcPicYuvRec->getWidth(COMPONENT_Y);
  const UInt picHeight = pcPicYuvRec->getHeight(COMPONENT_Y);
  const UInt noCol = (picWidth>>log2maxTB);//列(以最大Tb块为单位)
  const UInt noRows = (picHeight>>log2maxTB);//行(以最大Tb块为单位)
  assert(noCol > 1);
  assert(noRows > 1);
  UInt64 *colSAD = (UInt64*)malloc(noCol*sizeof(UInt64));
  UInt64 *rowSAD = (UInt64*)malloc(noRows*sizeof(UInt64));
  UInt colIdx = 0;
  UInt rowIdx = 0;
  Pel p0, p1, p2, q0, q1, q2;

  Int qp = pcPic->getSlice(0)->getSliceQp();
  const Int bitDepthLuma=pcPic->getSlice(0)->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA);
  Int bitdepthScale = 1 << (bitDepthLuma-8);
  Int beta = TComLoopFilter::getBeta( qp ) * bitdepthScale;//去方块滤波中的beta值
  const Int thr2 = (beta>>2);//beta/4
  const Int thr1 = 2*bitdepthScale;
  UInt a = 0;

  memset(colSAD, 0, noCol*sizeof(UInt64));
  memset(rowSAD, 0, noRows*sizeof(UInt64));

  if (maxTBsize > minBlockArtSize)
  {
    // Analyze vertical artifact edges
    for(Int c = maxTBsize; c < picWidth; c += maxTBsize)//分析垂直边界
    {
      for(Int r = 0; r < picHeight; r++)//处理图像每一行(Tb)边界像素
      {
        p2 = Rec[c-3];
        p1 = Rec[c-2];
        p0 = Rec[c-1];//P块像素
        q0 = Rec[c];
        q1 = Rec[c+1];
        q2 = Rec[c+2];//Q块像素
        a = ((abs(p2-(p1<<1)+p0)+abs(q0-(q1<<1)+q2))<<1);//该行P块像素变化率和Q块像素变化率之和
        if ( thr1 < a && a < thr2)//在要求的范围内
        {
          colSAD[colIdx] += abs(p0 - q0);//计算边界两侧像素的SAD
        }
        Rec += stride;//下一行
      }
      colIdx++;
      Rec = tempRec;//回到起始行
    }

    // Analyze horizontal artifact edges
    for(Int r = maxTBsize; r < picHeight; r += maxTBsize)//分析水平边界 同上
    {
      for(Int c = 0; c < picWidth; c++)
      {
        p2 = Rec[c + (r-3)*stride];
        p1 = Rec[c + (r-2)*stride];
        p0 = Rec[c + (r-1)*stride];
        q0 = Rec[c + r*stride];
        q1 = Rec[c + (r+1)*stride];
        q2 = Rec[c + (r+2)*stride];
        a = ((abs(p2-(p1<<1)+p0)+abs(q0-(q1<<1)+q2))<<1);
        if (thr1 < a && a < thr2)
        {
          rowSAD[rowIdx] += abs(p0 - q0);
        }
      }
      rowIdx++;
    }
  }

  UInt64 colSADsum = 0;
  UInt64 rowSADsum = 0;
  for(Int c = 0; c < noCol-1; c++)//所有垂直边界SAD之和
  {
    colSADsum += colSAD[c];
  }
  for(Int r = 0; r < noRows-1; r++)//所有水平边界SAD之和
  {
    rowSADsum += rowSAD[r];
  }

  colSADsum <<= 10;
  rowSADsum <<= 10;//保证整数运算精度
  colSADsum /= (noCol-1);
  colSADsum /= picHeight;//图像中垂直边界两侧(一行)的平均像素差
  rowSADsum /= (noRows-1);
  rowSADsum /= picWidth;//图像中水平边界两侧(一列)的平均像素差

  UInt64 avgSAD = ((colSADsum + rowSADsum)>>1);//图像maxTb边界处的平均像素差
  avgSAD >>= (bitDepthLuma-8);

  if ( avgSAD > 2048 )//大于给定的阈值 2048<<10=2
  {
    avgSAD >>= 9;
    Int offset = Clip3(2,6,(Int)avgSAD);
    for (Int i=0; i<uiNumSlices; i++)//设置图像中每个slice使用去方块滤波及参数Beta Tc的偏移值
    {
      pcPic->getSlice(i)->setDeblockingFilterOverrideFlag(true);
      pcPic->getSlice(i)->setDeblockingFilterDisable(false);
      pcPic->getSlice(i)->setDeblockingFilterBetaOffsetDiv2( offset );
      pcPic->getSlice(i)->setDeblockingFilterTcOffsetDiv2( offset );
    }
  }
  else//否则 由PPS信息设置去方块滤波参数
  {
    for (Int i=0; i<uiNumSlices; i++)
    {
      pcPic->getSlice(i)->setDeblockingFilterOverrideFlag(false);
      pcPic->getSlice(i)->setDeblockingFilterDisable(        pcPic->getSlice(i)->getPPS()->getPicDisableDeblockingFilterFlag() );
      pcPic->getSlice(i)->setDeblockingFilterBetaOffsetDiv2( pcPic->getSlice(i)->getPPS()->getDeblockingFilterBetaOffsetDiv2() );
      pcPic->getSlice(i)->setDeblockingFilterTcOffsetDiv2(   pcPic->getSlice(i)->getPPS()->getDeblockingFilterTcOffsetDiv2()   );
    }
  }

  free(colSAD);
  free(rowSAD);
}

//! \}
