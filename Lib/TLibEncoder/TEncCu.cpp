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

/** \file     TEncCu.cpp
    \brief    Coding Unit (CU) encoder class
*/

#include <stdio.h>
#include "TEncTop.h"
#include "TEncCu.h"
#include "TEncAnalyze.h"
#include "TLibCommon/Debug.h"

#include <cmath>
#include <algorithm>
using namespace std;


//! \ingroup TLibEncoder
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

/**
 \param    uhTotalDepth  total number of allowable depth
 \param    uiMaxWidth    largest CU width
 \param    uiMaxHeight   largest CU height
 \param    chromaFormat  chroma format
 */
Void TEncCu::create(UChar uhTotalDepth, UInt uiMaxWidth, UInt uiMaxHeight, ChromaFormat chromaFormat)//创建TEncCu类所需要用到的对象和变量
{//uhTotalDepth为允许的总深度
  Int i;

  m_uhTotalDepth   = uhTotalDepth + 1;
  m_ppcBestCU      = new TComDataCU*[m_uhTotalDepth-1];
  m_ppcTempCU      = new TComDataCU*[m_uhTotalDepth-1];

  m_ppcPredYuvBest = new TComYuv*[m_uhTotalDepth-1];
  m_ppcResiYuvBest = new TComYuv*[m_uhTotalDepth-1];
  m_ppcRecoYuvBest = new TComYuv*[m_uhTotalDepth-1];
  m_ppcPredYuvTemp = new TComYuv*[m_uhTotalDepth-1];
  m_ppcResiYuvTemp = new TComYuv*[m_uhTotalDepth-1];
  m_ppcRecoYuvTemp = new TComYuv*[m_uhTotalDepth-1];
  m_ppcOrigYuv     = new TComYuv*[m_uhTotalDepth-1];

  UInt uiNumPartitions;
  for( i=0 ; i<m_uhTotalDepth-1 ; i++)//创建各个深度下的对象
  {
    uiNumPartitions = 1<<( ( m_uhTotalDepth - i - 1 )<<1 );//该深度下CU中4*4小块个数
    UInt uiWidth  = uiMaxWidth  >> i;
    UInt uiHeight = uiMaxHeight >> i;

    m_ppcBestCU[i] = new TComDataCU; m_ppcBestCU[i]->create( chromaFormat, uiNumPartitions, uiWidth, uiHeight, false, uiMaxWidth >> (m_uhTotalDepth - 1) );
    m_ppcTempCU[i] = new TComDataCU; m_ppcTempCU[i]->create( chromaFormat, uiNumPartitions, uiWidth, uiHeight, false, uiMaxWidth >> (m_uhTotalDepth - 1) );

    m_ppcPredYuvBest[i] = new TComYuv; m_ppcPredYuvBest[i]->create(uiWidth, uiHeight, chromaFormat);
    m_ppcResiYuvBest[i] = new TComYuv; m_ppcResiYuvBest[i]->create(uiWidth, uiHeight, chromaFormat);
    m_ppcRecoYuvBest[i] = new TComYuv; m_ppcRecoYuvBest[i]->create(uiWidth, uiHeight, chromaFormat);

    m_ppcPredYuvTemp[i] = new TComYuv; m_ppcPredYuvTemp[i]->create(uiWidth, uiHeight, chromaFormat);
    m_ppcResiYuvTemp[i] = new TComYuv; m_ppcResiYuvTemp[i]->create(uiWidth, uiHeight, chromaFormat);
    m_ppcRecoYuvTemp[i] = new TComYuv; m_ppcRecoYuvTemp[i]->create(uiWidth, uiHeight, chromaFormat);

    m_ppcOrigYuv    [i] = new TComYuv; m_ppcOrigYuv    [i]->create(uiWidth, uiHeight, chromaFormat);
  }

  m_bEncodeDQP                     = false;
  m_stillToCodeChromaQpOffsetFlag  = false;
  m_cuChromaQpOffsetIdxPlus1       = 0;
  m_bFastDeltaQP                   = false;

  // initialize partition order.
  UInt* piTmp = &g_auiZscanToRaster[0];
  initZscanToRaster( m_uhTotalDepth, 1, 0, piTmp);//初始化Zscan 到 Raster的映射表
  initRasterToZscan( uiMaxWidth, uiMaxHeight, m_uhTotalDepth );//初始化Raster 到 Zscan的映射表

  // initialize conversion matrix from partition index to pel
  initRasterToPelXY( uiMaxWidth, uiMaxHeight, m_uhTotalDepth );//初始化Raster到像素坐标的映射表
}

Void TEncCu::destroy()//销毁对象 释放资源
{
  Int i;

  for( i=0 ; i<m_uhTotalDepth-1 ; i++)
  {
    if(m_ppcBestCU[i])
    {
      m_ppcBestCU[i]->destroy();      delete m_ppcBestCU[i];      m_ppcBestCU[i] = NULL;
    }
    if(m_ppcTempCU[i])
    {
      m_ppcTempCU[i]->destroy();      delete m_ppcTempCU[i];      m_ppcTempCU[i] = NULL;
    }
    if(m_ppcPredYuvBest[i])
    {
      m_ppcPredYuvBest[i]->destroy(); delete m_ppcPredYuvBest[i]; m_ppcPredYuvBest[i] = NULL;
    }
    if(m_ppcResiYuvBest[i])
    {
      m_ppcResiYuvBest[i]->destroy(); delete m_ppcResiYuvBest[i]; m_ppcResiYuvBest[i] = NULL;
    }
    if(m_ppcRecoYuvBest[i])
    {
      m_ppcRecoYuvBest[i]->destroy(); delete m_ppcRecoYuvBest[i]; m_ppcRecoYuvBest[i] = NULL;
    }
    if(m_ppcPredYuvTemp[i])
    {
      m_ppcPredYuvTemp[i]->destroy(); delete m_ppcPredYuvTemp[i]; m_ppcPredYuvTemp[i] = NULL;
    }
    if(m_ppcResiYuvTemp[i])
    {
      m_ppcResiYuvTemp[i]->destroy(); delete m_ppcResiYuvTemp[i]; m_ppcResiYuvTemp[i] = NULL;
    }
    if(m_ppcRecoYuvTemp[i])
    {
      m_ppcRecoYuvTemp[i]->destroy(); delete m_ppcRecoYuvTemp[i]; m_ppcRecoYuvTemp[i] = NULL;
    }
    if(m_ppcOrigYuv[i])
    {
      m_ppcOrigYuv[i]->destroy();     delete m_ppcOrigYuv[i];     m_ppcOrigYuv[i] = NULL;
    }
  }
  if(m_ppcBestCU)
  {
    delete [] m_ppcBestCU;
    m_ppcBestCU = NULL;
  }
  if(m_ppcTempCU)
  {
    delete [] m_ppcTempCU;
    m_ppcTempCU = NULL;
  }

  if(m_ppcPredYuvBest)
  {
    delete [] m_ppcPredYuvBest;
    m_ppcPredYuvBest = NULL;
  }
  if(m_ppcResiYuvBest)
  {
    delete [] m_ppcResiYuvBest;
    m_ppcResiYuvBest = NULL;
  }
  if(m_ppcRecoYuvBest)
  {
    delete [] m_ppcRecoYuvBest;
    m_ppcRecoYuvBest = NULL;
  }
  if(m_ppcPredYuvTemp)
  {
    delete [] m_ppcPredYuvTemp;
    m_ppcPredYuvTemp = NULL;
  }
  if(m_ppcResiYuvTemp)
  {
    delete [] m_ppcResiYuvTemp;
    m_ppcResiYuvTemp = NULL;
  }
  if(m_ppcRecoYuvTemp)
  {
    delete [] m_ppcRecoYuvTemp;
    m_ppcRecoYuvTemp = NULL;
  }
  if(m_ppcOrigYuv)
  {
    delete [] m_ppcOrigYuv;
    m_ppcOrigYuv = NULL;
  }
}

/** \param    pcEncTop      pointer of encoder class
 */
Void TEncCu::init( TEncTop* pcEncTop )//用TEncTop来初始化TEncCu
{
  m_pcEncCfg           = pcEncTop;
  m_pcPredSearch       = pcEncTop->getPredSearch();
  m_pcTrQuant          = pcEncTop->getTrQuant();
  m_pcRdCost           = pcEncTop->getRdCost();

  m_pcEntropyCoder     = pcEncTop->getEntropyCoder();
  m_pcBinCABAC         = pcEncTop->getBinCABAC();

  m_pppcRDSbacCoder    = pcEncTop->getRDSbacCoder();
  m_pcRDGoOnSbacCoder  = pcEncTop->getRDGoOnSbacCoder();

  m_pcRateCtrl         = pcEncTop->getRateCtrl();
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/** 
 \param  pCtu pointer of CU data class
 */
Void TEncCu::compressCtu( TComDataCU* pCtu )//压缩Ctu(即根据RDO找到Ctu最优的Cu分割)
{
  // initialize CU data
  m_ppcBestCU[0]->initCtu( pCtu->getPic(), pCtu->getCtuRsAddr() );
  m_ppcTempCU[0]->initCtu( pCtu->getPic(), pCtu->getCtuRsAddr() );//初始化m_ppcBestCU[0]和m_ppcTempCU[0]的Ctu信息

  // analysis of CU
  DEBUG_STRING_NEW(sDebug)

  xCompressCU( m_ppcBestCU[0], m_ppcTempCU[0], 0 DEBUG_STRING_PASS_INTO(sDebug) );//压缩Ctu 将最优的Cu分割保存在m_ppcBestCU[0]中
  DEBUG_STRING_OUTPUT(std::cout, sDebug)

#if ADAPTIVE_QP_SELECTION
  if( m_pcEncCfg->getUseAdaptQpSelect() )
  {
    if(pCtu->getSlice()->getSliceType()!=I_SLICE) //IIII
    {
      xCtuCollectARLStats( pCtu );
    }
  }
#endif
}
/** \param  pCtu  pointer of CU data class
 */
Void TEncCu::encodeCtu ( TComDataCU* pCtu )//编码一个Ctu
{
  if ( pCtu->getSlice()->getPPS()->getUseDQP() )//若使用DQP 则设dQP标志为真
  {
    setdQPFlag(true);
  }

  if ( pCtu->getSlice()->getUseChromaQpAdj() )//若使用ChromaQpAdj 则设ChromaQpAdj标志为真
  {
    setCodeChromaQpAdjFlag(true);
  }

  // Encode CU data
  xEncodeCU( pCtu, 0, 0 );//编码该Ctu
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================
//! Derive small set of test modes for AMP encoder speed-up
#if AMP_ENC_SPEEDUP
#if AMP_MRG
Void TEncCu::deriveTestModeAMP (TComDataCU *pcBestCU, PartSize eParentPartSize, Bool &bTestAMP_Hor, Bool &bTestAMP_Ver, Bool &bTestMergeAMP_Hor, Bool &bTestMergeAMP_Ver)
#else
Void TEncCu::deriveTestModeAMP (TComDataCU *pcBestCU, PartSize eParentPartSize, Bool &bTestAMP_Hor, Bool &bTestAMP_Ver)
#endif
{//根据当前Cu的最优Pu分割模式及父Cu的最优Pu分割模式决定当前Cu的Pu分割是否允许使用非对称分割和MergeAMP(是否允许融合非对称分割)  可以在几乎不影响编码效率的情况下有效降低计算复杂度 因为需要遍历的非对称分割的种类减少了
  if ( pcBestCU->getPartitionSize(0) == SIZE_2NxN )//若Cu的当前最优Pu分割为2NxN
  {
    bTestAMP_Hor = true;//则还需测试2NxnU和2NxnD
  }
  else if ( pcBestCU->getPartitionSize(0) == SIZE_Nx2N )//若Cu的当前最优Pu分割为Nx2N
  {
    bTestAMP_Ver = true;//则还需测试nLx2N和nRx2N
  }
  else if ( pcBestCU->getPartitionSize(0) == SIZE_2Nx2N && pcBestCU->getMergeFlag(0) == false && pcBestCU->isSkipped(0) == false )//当前Cu最优分割模式为2N×2N且不使用merge 不为skip
  {
    bTestAMP_Hor = true;
    bTestAMP_Ver = true;//则所有非对称模式均要测试
  }

#if AMP_MRG
  //! Utilizing the partition size of parent PU
  if ( eParentPartSize >= SIZE_2NxnU && eParentPartSize <= SIZE_nRx2N )//父Cu的最优Pu分割为非对称分割
  {
    bTestMergeAMP_Hor = true;
    bTestMergeAMP_Ver = true;//则所有非对称模式均要测试merge
  }

  if ( eParentPartSize == NUMBER_OF_PART_SIZES ) //! if parent is intra//父Cu为帧内预测
  {
    if ( pcBestCU->getPartitionSize(0) == SIZE_2NxN )//Cu的当前最优分割为2NxN
    {
      bTestMergeAMP_Hor = true;//则2NxnU和2NxnD下需测试merge
    }
    else if ( pcBestCU->getPartitionSize(0) == SIZE_Nx2N )
    {
      bTestMergeAMP_Ver = true;//则nLx2N和nRx2N下需测试merge
    }
  }

  if ( pcBestCU->getPartitionSize(0) == SIZE_2Nx2N && pcBestCU->isSkipped(0) == false )////当前Cu最优分割模式为2N×2N且 不为skip
  {
    bTestMergeAMP_Hor = true;
    bTestMergeAMP_Ver = true;//则所有非对称模式均要测试merge
  }

  if ( pcBestCU->getWidth(0) == 64 )//Cu大小为64
  {
    bTestAMP_Hor = false;
    bTestAMP_Ver = false;//不需要测试非对称模式
  }
#else
  //! Utilizing the partition size of parent PU
  if ( eParentPartSize >= SIZE_2NxnU && eParentPartSize <= SIZE_nRx2N )
  {
    bTestAMP_Hor = true;
    bTestAMP_Ver = true;
  }

  if ( eParentPartSize == SIZE_2Nx2N )
  {
    bTestAMP_Hor = false;
    bTestAMP_Ver = false;
  }
#endif
}
#endif


// ====================================================================================================================
// Protected member functions
// ====================================================================================================================
/** Compress a CU block recursively with enabling sub-CTU-level delta QP
 *  - for loop of QP value to compress the current CU with all possible QP
*/
#if AMP_ENC_SPEEDUP
Void TEncCu::xCompressCU( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, const UInt uiDepth DEBUG_STRING_FN_DECLARE(sDebug_), PartSize eParentPartSize )//压缩Cu 将该Cu的最优分割保存至rpcBestCU
#else
Void TEncCu::xCompressCU( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, const UInt uiDepth )
#endif
{
  TComPic* pcPic = rpcBestCU->getPic();
  DEBUG_STRING_NEW(sDebug)
  const TComPPS &pps=*(rpcTempCU->getSlice()->getPPS());//PPS信息
  const TComSPS &sps=*(rpcTempCU->getSlice()->getSPS());//SPS信息
  
  // These are only used if getFastDeltaQp() is true
  const UInt fastDeltaQPCuMaxSize    = Clip3(sps.getMaxCUHeight()>>sps.getLog2DiffMaxMinCodingBlockSize(), sps.getMaxCUHeight(), 32u);//使用fastDeltaQP允许的最大Cu大小

  // get Original YUV data from picture
  m_ppcOrigYuv[uiDepth]->copyFromPicYuv( pcPic->getPicYuvOrg(), rpcBestCU->getCtuRsAddr(), rpcBestCU->getZorderIdxInCtu() );//将该Cu在原始图像中的原像素值复制到m_ppcOrigYuv[uiDepth]

  // variable for Cbf fast mode PU decision
  Bool    doNotBlockPu = true;
  Bool    earlyDetectionSkipMode = false;//初始化快速模式判决参数

  const UInt uiLPelX   = rpcBestCU->getCUPelX();//该Cu的左边界的X坐标
  const UInt uiRPelX   = uiLPelX + rpcBestCU->getWidth(0)  - 1;//该Cu的右边界的X坐标
  const UInt uiTPelY   = rpcBestCU->getCUPelY();//该Cu的上边界的Y坐标
  const UInt uiBPelY   = uiTPelY + rpcBestCU->getHeight(0) - 1;//该Cu的下边界的Y坐标
  const UInt uiWidth   = rpcBestCU->getWidth(0);//该Cu的宽

  Int iBaseQP = xComputeQP( rpcBestCU, uiDepth );//得到该Cu的基础Qp值
  Int iMinQP;
  Int iMaxQP;
  Bool isAddLowestQP = false;

  const UInt numberValidComponents = rpcBestCU->getPic()->getNumberValidComponents();//有效分量数

  if( uiDepth <= pps.getMaxCuDQPDepth() )//若该Cu深度不大于允许使用DeltaQP的最大深度
  {
    Int idQP = m_pcEncCfg->getMaxDeltaQP();//DeltaQP
    iMinQP = Clip3( -sps.getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP-idQP );//该CU可能的最小Qp
    iMaxQP = Clip3( -sps.getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP+idQP );//该CU可能的最大Qp
  }
  else
  {
    iMinQP = rpcTempCU->getQP(0);
    iMaxQP = rpcTempCU->getQP(0);
  }//否则直接得到Qp值

  if ( m_pcEncCfg->getUseRateCtrl() )//若使用率控制 则使用率控制对应的Qp值
  {
    iMinQP = m_pcRateCtrl->getRCQP();
    iMaxQP = m_pcRateCtrl->getRCQP();
  }

  // transquant-bypass (TQB) processing loop variable initialisation ---
  //此处lowestQP为测试transquant-bypass时的QP 因为TQB的失真与QP值无关 选择iMinQP为TQB的QP而不是QP=0是因为delta QPs更小 且和遍历所有非TQB下的QP值形式统一
  const Int lowestQP = iMinQP; // For TQB, use this QP which is the lowest non TQB QP tested (rather than QP'=0) - that way delta QPs are smaller, and TQB can be tested at all CU levels.

  if ( (pps.getTransquantBypassEnableFlag()) )//若允许使用TransquantBypass
  {
    isAddLowestQP = true; // mark that the first iteration is to cost TQB mode.//标志 表示测试TQB模式
    iMinQP = iMinQP - 1;  // increase loop variable range by 1, to allow testing of TQB mode along with other QPs//增加一次循环
    if ( m_pcEncCfg->getCUTransquantBypassFlagForceValue() )
    {
      iMaxQP = iMinQP;
    }
  }

  TComSlice * pcSlice = rpcTempCU->getPic()->getSlice(rpcTempCU->getPic()->getCurrSliceIdx());

  const Bool bBoundary = !( uiRPelX < sps.getPicWidthInLumaSamples() && uiBPelY < sps.getPicHeightInLumaSamples() );//该Cu是否在图像(的下边界或右边界)之内

  if ( !bBoundary )//the CTUs at the borders are inferred to be split until the boundaries of the resulting blocks coincide with the picture boundary//在图像下或右边界的Cu被推断为继续分割 故不用执行不分割的情况
  {
    for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)//遍历所有可能的QP值
    {
      const Bool bIsLosslessMode = isAddLowestQP && (iQP == iMinQP);//是否为lossless模式(规定若允许使用lossless 则第一个检测的模式即为lossless模式)

      if (bIsLosslessMode)
      {
        iQP = lowestQP;
      }

      m_cuChromaQpOffsetIdxPlus1 = 0;
      if (pcSlice->getUseChromaQpAdj())//若使用ChromaQpAdj 则得到色度Qp偏移量的索引(根据索引可以得到对应的色度QP偏移量)
      {
        /* Pre-estimation of chroma QP based on input block activity may be performed
         * here, using for example m_ppcOrigYuv[uiDepth] */
        /* To exercise the current code, the index used for adjustment is based on
         * block position
         */
        Int lgMinCuSize = sps.getLog2MinCodingBlockSize() +
                          std::max<Int>(0, sps.getLog2DiffMaxMinCodingBlockSize()-Int(pps.getPpsRangeExtension().getDiffCuChromaQpOffsetDepth()));
        m_cuChromaQpOffsetIdxPlus1 = ((uiLPelX >> lgMinCuSize) + (uiTPelY >> lgMinCuSize)) % (pps.getPpsRangeExtension().getChromaQpOffsetListLen() + 1);
      }

      rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );//初始化rpcTempCU的预测信息

      // do inter modes, SKIP and 2Nx2N
      if( rpcBestCU->getSlice()->getSliceType() != I_SLICE )//若不为I_slice(I_slice无法帧间预测)
      {
        // 2Nx2N
        if(m_pcEncCfg->getUseEarlySkipDetection())//使用早期跳出策略(满足条件则直接跳过剩余候选模式)
        {
          xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2Nx2N DEBUG_STRING_PASS_INTO(sDebug) );//检测2Nx2N 将最优的预测信息保存至rpcBestCU
          rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );//by Competition for inter_2Nx2N////初始化rpcTempCU的预测信息
        }
        // SKIP
        xCheckRDCostMerge2Nx2N( rpcBestCU, rpcTempCU DEBUG_STRING_PASS_INTO(sDebug), &earlyDetectionSkipMode );//by Merge for inter_2Nx2N//2Nx2N下 检测merge方式是否更优
        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );//初始化rpcTempCU的预测信息

        if(!m_pcEncCfg->getUseEarlySkipDetection())
        {
          // 2Nx2N, NxN
          xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2Nx2N DEBUG_STRING_PASS_INTO(sDebug) );
          rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
          if(m_pcEncCfg->getUseCbfFastMode())//若使用快速CBF策略
          {
            doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;//预测残差是否包含非零的变换系数(若对应的预测残差不包含非零的变换系数 则跳过该Cu的其余候选模式)
          }
        }
      }

      if (bIsLosslessMode) // Restore loop variable if lossless mode was searched.
      {
        iQP = iMinQP;//lossless模式下需将QP值还原回去(因为lossless模式使用的Qp为lowestQP 不还原回去将少遍历一个QP值)
      }
    }

    if(!earlyDetectionSkipMode)//若未Early_Skip(早期跳出)
    {
      for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)//遍历所有可能的QP值
      {
        const Bool bIsLosslessMode = isAddLowestQP && (iQP == iMinQP); // If lossless, then iQP is irrelevant for subsequent modules.

        if (bIsLosslessMode)
        {
          iQP = lowestQP;
        }

        rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

        // do inter modes, NxN, 2NxN, and Nx2N
        if( rpcBestCU->getSlice()->getSliceType() != I_SLICE )
        {
          // 2Nx2N, NxN

          if(!( (rpcBestCU->getWidth(0)==8) && (rpcBestCU->getHeight(0)==8) ))//该Cu大小不能为8*8 因为帧间预测4*4大小的Pu块不被允许
          {                                                                         //<HEVC book>中说明:the .M/2 .M/2 mode is only supported if the selected minimum CU size is larger than 8 8 luma samples,                                                                      
            if( uiDepth == sps.getLog2DiffMaxMinCodingBlockSize() && doNotBlockPu)  // so that blocks of8 4 and 4 8 are the smallest block sizes that can be used for motion-compensatedprediction
            {//只有当Cu为规定的最小Cu 且2N*2N存在非零变换系数 才检测N*N 因为不为最小Cu时 N*N的Pu分割相当与下一深度Cu的2N*2N
              xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_NxN DEBUG_STRING_PASS_INTO(sDebug)   );////检测2Nx2N 将最优的预测信息保存至rpcBestCU
              rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );//初始化rpcTempCU的预测信息 供下一模式使用
            }
          }

          if(doNotBlockPu)
          {
            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_Nx2N DEBUG_STRING_PASS_INTO(sDebug)  );//检测Nx2N 将最优的预测信息保存至rpcBestCU
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );//初始化rpcTempCU的预测信息 供下一模式使用
            if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_Nx2N )//若当前最优模式为所检测的模式 则更新doNotBlockPu 
            {
              doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
            }
          }
          if(doNotBlockPu)
          {
            xCheckRDCostInter      ( rpcBestCU, rpcTempCU, SIZE_2NxN DEBUG_STRING_PASS_INTO(sDebug)  );//检测2NxN 将最优的预测信息保存至rpcBestCU
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );//初始化rpcTempCU的预测信息 供下一模式使用
            if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxN)//若当前最优模式为所检测的模式 则更新doNotBlockPu 
            {
              doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
            }
          }

          //! Try AMP (SIZE_2NxnU, SIZE_2NxnD, SIZE_nLx2N, SIZE_nRx2N)
          if(sps.getUseAMP() && uiDepth < sps.getLog2DiffMaxMinCodingBlockSize() )//若允许使用非对称分割模式且Cu大于最小Cu (因为最小Cu无法做非对称分割)
          {
#if AMP_ENC_SPEEDUP
            Bool bTestAMP_Hor = false, bTestAMP_Ver = false;

#if AMP_MRG
            Bool bTestMergeAMP_Hor = false, bTestMergeAMP_Ver = false;

            deriveTestModeAMP (rpcBestCU, eParentPartSize, bTestAMP_Hor, bTestAMP_Ver, bTestMergeAMP_Hor, bTestMergeAMP_Ver);//得到需要检测的非对称模式
#else
            deriveTestModeAMP (rpcBestCU, eParentPartSize, bTestAMP_Hor, bTestAMP_Ver);
#endif

            //! Do horizontal AMP
            if ( bTestAMP_Hor )
            {
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnU DEBUG_STRING_PASS_INTO(sDebug) );//检测2NxnU
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );//初始化rpcTempCU的预测信息
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnU )//若当前最优模式为所检测的模式 则更新doNotBlockPu 
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnD DEBUG_STRING_PASS_INTO(sDebug) );//检测2NxnU
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );//初始化rpcTempCU的预测信息
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnD )//若当前最优模式为所检测的模式 则更新doNotBlockPu 
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
            }
#if AMP_MRG
            else if ( bTestMergeAMP_Hor )//同上 
            {
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnU DEBUG_STRING_PASS_INTO(sDebug), true );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnU )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnD DEBUG_STRING_PASS_INTO(sDebug), true );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_2NxnD )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
            }//TestMergeAMP是较TestAMP一种快速算法 该算法下只使用merge候选列表中的Mv做RDO而不用复杂的搜索计算求得最佳的MV及最佳RD
#endif

            //! Do horizontal AMP
            if ( bTestAMP_Ver )
            {
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nLx2N DEBUG_STRING_PASS_INTO(sDebug) );//检测nLx2N
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );//初始化rpcTempCU的预测信息
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_nLx2N )//若当前最优模式为所检测的模式 则更新doNotBlockPu 
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nRx2N DEBUG_STRING_PASS_INTO(sDebug) );//检测nRx2N
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );//初始化rpcTempCU的预测信息
              }
            }
#if AMP_MRG
            else if ( bTestMergeAMP_Ver )//若不需要TestAMP_Ver 且需要TestMergeAMP_Ver  
            {
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nLx2N DEBUG_STRING_PASS_INTO(sDebug), true );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
                if(m_pcEncCfg->getUseCbfFastMode() && rpcBestCU->getPartitionSize(0) == SIZE_nLx2N )
                {
                  doNotBlockPu = rpcBestCU->getQtRootCbf( 0 ) != 0;
                }
              }
              if(doNotBlockPu)
              {
                xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nRx2N DEBUG_STRING_PASS_INTO(sDebug), true );
                rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
              }
            }//TestMergeAMP是较TestAMP一种快速算法 该算法下只使用merge候选列表中的Mv做RDO而不用复杂的搜索计算求得最佳的MV及最佳RD
#endif

#else
            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnU );
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_2NxnD );
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nLx2N );
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

            xCheckRDCostInter( rpcBestCU, rpcTempCU, SIZE_nRx2N );
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

#endif
          }
        }

        // do normal intra modes
        // speedup for inter frames
        Double intraCost = 0.0;

        if((rpcBestCU->getSlice()->getSliceType() == I_SLICE)                                        || //该slice为I_slice(I slice只能使用帧内预测 故一定需要检测帧内Pu分割模式)
            ((!m_pcEncCfg->getDisableIntraPUsInInterSlices()) && (                                      //为P 或B slice 但允许帧内预测 且 帧间预测的最优Pu分割模式存在非零的变换系数
              (rpcBestCU->getCbf( 0, COMPONENT_Y  ) != 0)                                            ||
             ((rpcBestCU->getCbf( 0, COMPONENT_Cb ) != 0) && (numberValidComponents > COMPONENT_Cb)) ||
             ((rpcBestCU->getCbf( 0, COMPONENT_Cr ) != 0) && (numberValidComponents > COMPONENT_Cr))  // avoid very complex intra if it is unlikely
            )))
        {
          xCheckRDCostIntra( rpcBestCU, rpcTempCU, intraCost, SIZE_2Nx2N DEBUG_STRING_PASS_INTO(sDebug) );//检测帧内预测2Nx2N
          rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );//初始化rpcTempCU的预测信息
          if( uiDepth == sps.getLog2DiffMaxMinCodingBlockSize() )//Cu为最小Cu时 帧内预测Pu的分割才能为NxN 因为不为最小Cu时 N*N的Pu分割相当与下一深度Cu的2N*2N
          {
            if( rpcTempCU->getWidth(0) > ( 1 << sps.getQuadtreeTULog2MinSize() ) )//Cu要一定大于最小的Tu大小
            {
              Double tmpIntraCost;
              xCheckRDCostIntra( rpcBestCU, rpcTempCU, tmpIntraCost, SIZE_NxN DEBUG_STRING_PASS_INTO(sDebug)   );//检测帧内预测NxN
              intraCost = std::min(intraCost, tmpIntraCost);//帧内预测的最优损耗
              rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );//初始化rpcTempCU的预测信息
            }
          }
        }

        // test PCM
        if(sps.getUsePCM()
          && rpcTempCU->getWidth(0) <= (1<<sps.getPCMLog2MaxSize())
          && rpcTempCU->getWidth(0) >= (1<<sps.getPCMLog2MinSize()) )//若允许使用PCM模式且Cu在PCM模式下规定的大小范围内
        {//(PCM模式下直接传输原始像素值)
          UInt uiRawBits = getTotalBits(rpcBestCU->getWidth(0), rpcBestCU->getHeight(0), rpcBestCU->getPic()->getChromaFormat(), sps.getBitDepths().recon);//可以粗略的理解为PCM模式下的编码比特数(并不是确定的 只用于大概的判断 避免无效的检测)
          UInt uiBestBits = rpcBestCU->getTotalBits();//最优预测及分割模式下的编码比特数
          if((uiBestBits > uiRawBits) || (rpcBestCU->getTotalCost() > m_pcRdCost->calcRdCost(uiRawBits, 0)))//如果粗略判断下PCM可能较之前的模式更优
          {
            xCheckIntraPCM (rpcBestCU, rpcTempCU);//则严格检测PCM模式 
            rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );//初始化rpcTempCU的预测信息
          }
        }

        if (bIsLosslessMode) // Restore loop variable if lossless mode was searched.
        {
          iQP = iMinQP;
        }
      }
    }

    if( rpcBestCU->getTotalCost()!=MAX_DOUBLE )
    {
      m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_NEXT_BEST]);
      m_pcEntropyCoder->resetBits();
      m_pcEntropyCoder->encodeSplitFlag( rpcBestCU, 0, uiDepth, true );//编码Split标志(SplitFlag为0)
      rpcBestCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // split bits//编码总比特位
      rpcBestCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
      rpcBestCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcBestCU->getTotalBits(), rpcBestCU->getTotalDistortion() );//该Cu不不分割为子Cu的情况下最优损耗
      m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[uiDepth][CI_NEXT_BEST]);
    }
  }

  // copy original YUV samples to PCM buffer
  if( rpcBestCU->getTotalCost()!=MAX_DOUBLE && rpcBestCU->isLosslessCoded(0) && (rpcBestCU->getIPCMFlag(0) == false))//为lossless模式
  {
    xFillPCMBuffer(rpcBestCU, m_ppcOrigYuv[uiDepth]);//将原像素值复制到PCM buffer
  }
  //得到Cu可能的Qp值范围
  if( uiDepth == pps.getMaxCuDQPDepth() )
  {
    Int idQP = m_pcEncCfg->getMaxDeltaQP();
    iMinQP = Clip3( -sps.getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP-idQP );
    iMaxQP = Clip3( -sps.getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQP+idQP );
  }
  else if( uiDepth < pps.getMaxCuDQPDepth() )
  {
    iMinQP = iBaseQP;
    iMaxQP = iBaseQP;
  }
  else
  {
    const Int iStartQP = rpcTempCU->getQP(0);
    iMinQP = iStartQP;
    iMaxQP = iStartQP;
  }

  if ( m_pcEncCfg->getUseRateCtrl() )
  {
    iMinQP = m_pcRateCtrl->getRCQP();
    iMaxQP = m_pcRateCtrl->getRCQP();
  }

  if ( m_pcEncCfg->getCUTransquantBypassFlagForceValue() )
  {
    iMaxQP = iMinQP; // If all TUs are forced into using transquant bypass, do not loop here.
  }

  const Bool bSubBranch = bBoundary || !( m_pcEncCfg->getUseEarlyCU() && rpcBestCU->getTotalCost()!=MAX_DOUBLE && rpcBestCU->isSkipped(0) );//使用EarlyCU策略若当前Cu的最优编码模式是Skip模式 则终止该Cu后续四叉树划分及模式判决

  if( bSubBranch && uiDepth < sps.getLog2DiffMaxMinCodingBlockSize() && (!getFastDeltaQp() || uiWidth > fastDeltaQPCuMaxSize || bBoundary))//当该CU大于最小Cu时该CU才能四叉树分割(bBoundary为真 该Cu一定分割)
  {
    // further split
    for (Int iQP=iMinQP; iQP<=iMaxQP; iQP++)//遍历该Cu所有可能的QP值
    {
      const Bool bIsLosslessMode = false; // False at this level. Next level down may set it to true.

      rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );

      UChar       uhNextDepth         = uiDepth+1;
      TComDataCU* pcSubBestPartCU     = m_ppcBestCU[uhNextDepth];
      TComDataCU* pcSubTempPartCU     = m_ppcTempCU[uhNextDepth];
      DEBUG_STRING_NEW(sTempDebug)

      for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++ )//遍历该Cu的四个子Cu
      {
        pcSubBestPartCU->initSubCU( rpcTempCU, uiPartUnitIdx, uhNextDepth, iQP );           // clear sub partition datas or init.
        pcSubTempPartCU->initSubCU( rpcTempCU, uiPartUnitIdx, uhNextDepth, iQP );           // clear sub partition datas or init.//初始化下一深度的子Cu

        if( ( pcSubBestPartCU->getCUPelX() < sps.getPicWidthInLumaSamples() ) && ( pcSubBestPartCU->getCUPelY() < sps.getPicHeightInLumaSamples() ) )//该子Cu在图像未完全超出图像范围
        {
          if ( 0 == uiPartUnitIdx) //initialize RD with previous depth buffer
          {
            m_pppcRDSbacCoder[uhNextDepth][CI_CURR_BEST]->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);
          }
          else
          {
            m_pppcRDSbacCoder[uhNextDepth][CI_CURR_BEST]->load(m_pppcRDSbacCoder[uhNextDepth][CI_NEXT_BEST]);
          }

#if AMP_ENC_SPEEDUP
          DEBUG_STRING_NEW(sChild)
          if ( !(rpcBestCU->getTotalCost()!=MAX_DOUBLE && rpcBestCU->isInter(0)) )//该情况下无法父Cu无法提供有效信息来缩减子Cu的AMP模式
          {
            xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth DEBUG_STRING_PASS_INTO(sChild), NUMBER_OF_PART_SIZES );
          }
          else//根据父Cu的最优Pu分割模式 来确定子Cu后续编码中可使用的AMP模式
          {

            xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth DEBUG_STRING_PASS_INTO(sChild), rpcBestCU->getPartitionSize(0) );
          }
          DEBUG_STRING_APPEND(sTempDebug, sChild)
#else
          xCompressCU( pcSubBestPartCU, pcSubTempPartCU, uhNextDepth );
#endif

          rpcTempCU->copyPartFrom( pcSubBestPartCU, uiPartUnitIdx, uhNextDepth );         // Keep best part data to current temporary data.//将子CU最优的预测信息复制到rpcTempCU对应位置
          xCopyYuv2Tmp( pcSubBestPartCU->getTotalNumPart()*uiPartUnitIdx, uhNextDepth );//将子Cu最优预测和分割模式下的预测像素值和重建像素值分别保存至m_ppcRecoYuvTemp和m_ppcPredYuvBest上一深度对应位置
        }
        else//该子Cu完全超出图像范围 则不用处理该子Cu
        {
          pcSubBestPartCU->copyToPic( uhNextDepth );
          rpcTempCU->copyPartFrom( pcSubBestPartCU, uiPartUnitIdx, uhNextDepth );
        }
      }

      m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uhNextDepth][CI_NEXT_BEST]);
      if( !bBoundary )//若Cu不在图像之内 则Cu一定四叉树分割 故不用编码Split标志
      {
        m_pcEntropyCoder->resetBits();
        m_pcEntropyCoder->encodeSplitFlag( rpcTempCU, 0, uiDepth, true );//编码Split标志

        rpcTempCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // split bits//编码总比特数
        rpcTempCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
      }
      rpcTempCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );//该Cu四叉树分割为4个子Cu的总损耗

      if( uiDepth == pps.getMaxCuDQPDepth() && pps.getUseDQP())//uiDepth为允许使用DQP的最大深度
      {
        Bool hasResidual = false;
        for( UInt uiBlkIdx = 0; uiBlkIdx < rpcTempCU->getTotalNumPart(); uiBlkIdx ++)//遍历Cu中所有4*4小块 判断该Cu中是否存在非零变换系数
        {
          if( (     rpcTempCU->getCbf(uiBlkIdx, COMPONENT_Y)
                || (rpcTempCU->getCbf(uiBlkIdx, COMPONENT_Cb) && (numberValidComponents > COMPONENT_Cb))
                || (rpcTempCU->getCbf(uiBlkIdx, COMPONENT_Cr) && (numberValidComponents > COMPONENT_Cr)) ) )//若存在分量存在非零变换系数
          {
            hasResidual = true;
            break;
          }
        }

        if ( hasResidual )//该CU存在残差值
        {
          m_pcEntropyCoder->resetBits();
          m_pcEntropyCoder->encodeQP( rpcTempCU, 0, false );//编码deltaQP值
          rpcTempCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // dQP bits//编码总比特数
          rpcTempCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
          rpcTempCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );//该Cu四叉树分割为4个子Cu的总损耗

          Bool foundNonZeroCbf = false;
          rpcTempCU->setQPSubCUs( rpcTempCU->getRefQP( 0 ), 0, uiDepth, foundNonZeroCbf );
          assert( foundNonZeroCbf );
        }
        else//不存在残差值 也就说明不需要准确编码QP值 直接设置QP值为预测QP值 这样可以有效节省编码deltaQP的比特数
        {
          rpcTempCU->setQPSubParts( rpcTempCU->getRefQP( 0 ), 0, uiDepth ); // set QP to default QP
        }
      }

      m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);

      // If the configuration being tested exceeds the maximum number of bytes for a slice / slice-segment, then
      // a proper RD evaluation cannot be performed. Therefore, termination of the
      // slice/slice-segment must be made prior to this CTU.
      // This can be achieved by forcing the decision to be that of the rpcTempCU.
      // The exception is each slice / slice-segment must have at least one CTU.
      if (rpcBestCU->getTotalCost()!=MAX_DOUBLE)
      {
        const Bool isEndOfSlice        =    pcSlice->getSliceMode()==FIXED_NUMBER_OF_BYTES
                                         && ((pcSlice->getSliceBits()+rpcBestCU->getTotalBits())>pcSlice->getSliceArgument()<<3)
                                         && rpcBestCU->getCtuRsAddr() != pcPic->getPicSym()->getCtuTsToRsAddrMap(pcSlice->getSliceCurStartCtuTsAddr())
                                         && rpcBestCU->getCtuRsAddr() != pcPic->getPicSym()->getCtuTsToRsAddrMap(pcSlice->getSliceSegmentCurStartCtuTsAddr());
        const Bool isEndOfSliceSegment =    pcSlice->getSliceSegmentMode()==FIXED_NUMBER_OF_BYTES
                                         && ((pcSlice->getSliceSegmentBits()+rpcBestCU->getTotalBits()) > pcSlice->getSliceSegmentArgument()<<3)
                                         && rpcBestCU->getCtuRsAddr() != pcPic->getPicSym()->getCtuTsToRsAddrMap(pcSlice->getSliceSegmentCurStartCtuTsAddr());
                                             // Do not need to check slice condition for slice-segment since a slice-segment is a subset of a slice.
        if(isEndOfSlice||isEndOfSliceSegment)
        {
          rpcBestCU->getTotalCost()=MAX_DOUBLE;
        }
      }
      //通过比较该Cu的率失真与该Cu四叉树分割的4个子Cu的率失真之和 保证rpcBestCU为最优Cu分割(是否四叉树分割该Cu)
      xCheckBestMode( rpcBestCU, rpcTempCU, uiDepth DEBUG_STRING_PASS_INTO(sDebug) DEBUG_STRING_PASS_INTO(sTempDebug) DEBUG_STRING_PASS_INTO(false) ); // RD compare current larger prediction
                                                                                                                                                       // with sub partitioned prediction.
    }
  }

  DEBUG_STRING_APPEND(sDebug_, sDebug);

  rpcBestCU->copyToPic(uiDepth);                                                     // Copy Best data to Picture for next partition prediction.//将最优Cu信息复制到图像中(最终的目的就是完成整个图像中每个Ctu的Cu分割) 

  xCopyYuv2Pic( rpcBestCU->getPic(), rpcBestCU->getCtuRsAddr(), rpcBestCU->getZorderIdxInCtu(), uiDepth, uiDepth );   // Copy Yuv data to picture Yuv//将最优分割下的重建像素和预测像素复制到重建图像和预测图像
  if (bBoundary)
  {
    return;
  }

  // Assert if Best prediction mode is NONE
  // Selected mode's RD-cost must be not MAX_DOUBLE.
  assert( rpcBestCU->getPartitionSize ( 0 ) != NUMBER_OF_PART_SIZES       );
  assert( rpcBestCU->getPredictionMode( 0 ) != NUMBER_OF_PREDICTION_MODES );
  assert( rpcBestCU->getTotalCost     (   ) != MAX_DOUBLE                 );
}

/** finish encoding a cu and handle end-of-slice conditions
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth
 * \returns Void
 */
Void TEncCu::finishCU( TComDataCU* pcCU, UInt uiAbsPartIdx )
{
  TComPic* pcPic = pcCU->getPic();
  TComSlice * pcSlice = pcCU->getPic()->getSlice(pcCU->getPic()->getCurrSliceIdx());

  //Calculate end address
  const Int  currentCTUTsAddr = pcPic->getPicSym()->getCtuRsToTsAddrMap(pcCU->getCtuRsAddr());
  const Bool isLastSubCUOfCtu = pcCU->isLastSubCUOfCtu(uiAbsPartIdx);
  if ( isLastSubCUOfCtu )//若该Cu为Ctu中最后一个CU
  {
    // The 1-terminating bit is added to all streams, so don't add it here when it's 1.
    // i.e. when the slice segment CurEnd CTU address is the current CTU address+1.
    if (pcSlice->getSliceSegmentCurEndCtuTsAddr() != currentCTUTsAddr+1)//该Ctu(Ts顺序)的下一个Ctu不为SliceSegment的最后一个Ctu
    {
      m_pcEntropyCoder->encodeTerminatingBit( 0 );//编码终止位0
    }
  }
}

/** Compute QP for each CU
 * \param pcCU Target CU
 * \param uiDepth CU depth
 * \returns quantization parameter
 */
Int TEncCu::xComputeQP( TComDataCU* pcCU, UInt uiDepth )//为每个CU计算QP值 (没看懂自适应QP值的QpOffset值是怎么得到的)
{
  Int iBaseQp = pcCU->getSlice()->getSliceQp();
  Int iQpOffset = 0;
  if ( m_pcEncCfg->getUseAdaptiveQP() )//若使用自适应QP值
  {
    TEncPic* pcEPic = dynamic_cast<TEncPic*>( pcCU->getPic() );
    UInt uiAQDepth = min( uiDepth, pcEPic->getMaxAQDepth()-1 );
    TEncPicQPAdaptationLayer* pcAQLayer = pcEPic->getAQLayer( uiAQDepth );
    UInt uiAQUPosX = pcCU->getCUPelX() / pcAQLayer->getAQPartWidth();
    UInt uiAQUPosY = pcCU->getCUPelY() / pcAQLayer->getAQPartHeight();
    UInt uiAQUStride = pcAQLayer->getAQPartStride();
    TEncQPAdaptationUnit* acAQU = pcAQLayer->getQPAdaptationUnit();

    Double dMaxQScale = pow(2.0, m_pcEncCfg->getQPAdaptationRange()/6.0);
    Double dAvgAct = pcAQLayer->getAvgActivity();
    Double dCUAct = acAQU[uiAQUPosY * uiAQUStride + uiAQUPosX].getActivity();
    Double dNormAct = (dMaxQScale*dCUAct + dAvgAct) / (dCUAct + dMaxQScale*dAvgAct);
    Double dQpOffset = log(dNormAct) / log(2.0) * 6.0;
    iQpOffset = Int(floor( dQpOffset + 0.49999 ));
  }

  return Clip3(-pcCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA), MAX_QP, iBaseQp+iQpOffset );
}

/** encode a CU block recursively
 * \param pcCU
 * \param uiAbsPartIdx
 * \param uiDepth
 * \returns Void
 */
Void TEncCu::xEncodeCU( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )//编码Cu块
{
        TComPic   *const pcPic   = pcCU->getPic();
        TComSlice *const pcSlice = pcCU->getSlice();
  const TComSPS   &sps =*(pcSlice->getSPS());
  const TComPPS   &pps =*(pcSlice->getPPS());

  const UInt maxCUWidth  = sps.getMaxCUWidth();
  const UInt maxCUHeight = sps.getMaxCUHeight();

        Bool bBoundary = false;
        UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];//该Cu左边界在Ctu中的像素位置
  const UInt uiRPelX   = uiLPelX + (maxCUWidth>>uiDepth)  - 1;//该Cu右边界在Ctu中的像素位置
        UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];//该Cu上边界在Ctu中的像素位置
  const UInt uiBPelY   = uiTPelY + (maxCUHeight>>uiDepth) - 1;//该Cu下边界在Ctu中的像素位置

  if( ( uiRPelX < sps.getPicWidthInLumaSamples() ) && ( uiBPelY < sps.getPicHeightInLumaSamples() ) )//只有当前Cu完全在图像内时 才需要编码split标志 
  {
    m_pcEntropyCoder->encodeSplitFlag( pcCU, uiAbsPartIdx, uiDepth );
  }
  else//当Cu未完全在图像内时 不需要编码split标志 因为此时推断Cu一定四叉树分割
  {
    bBoundary = true;
  }

  if( ( ( uiDepth < pcCU->getDepth( uiAbsPartIdx ) ) && ( uiDepth < sps.getLog2DiffMaxMinCodingBlockSize() ) ) || bBoundary )//若该Cu四叉树分割且分割后的子Cu的大小在允许的范围
  {
    UInt uiQNumParts = ( pcPic->getNumPartitionsInCtu() >> (uiDepth<<1) )>>2;//该Cu的子Cu中4*4小块数
    if( uiDepth == pps.getMaxCuDQPDepth() && pps.getUseDQP())//只需要判断uiDepth == pps.getMaxCuDQPDepth() 而不用判断小于的情况是因为递归过程中更深的深度会被后续处理到
    {
      setdQPFlag(true);//设置使用deltaQP标志为真
    }

    if( uiDepth == pps.getPpsRangeExtension().getDiffCuChromaQpOffsetDepth() && pcSlice->getUseChromaQpAdj())
    {
      setCodeChromaQpAdjFlag(true);//设置使用ChromaQpOffset标志为真
    }

    for ( UInt uiPartUnitIdx = 0; uiPartUnitIdx < 4; uiPartUnitIdx++, uiAbsPartIdx+=uiQNumParts )//遍历Cu的4个子Cu
    {
      uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];
      uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
      if( ( uiLPelX < sps.getPicWidthInLumaSamples() ) && ( uiTPelY < sps.getPicHeightInLumaSamples() ) )//若该子Cu完全不在图像中则不用编码该Cu 否则编码该子Cu
      {
        xEncodeCU( pcCU, uiAbsPartIdx, uiDepth+1 );//递归处理子Cu
      }
    }
    return;
  }

  if( uiDepth <= pps.getMaxCuDQPDepth() && pps.getUseDQP())//允许使用deltaQP且该Cu深度在允许使用deltaQP的范围内
  {
    setdQPFlag(true);//设置使用deltaQP标志为真
  }

  if( uiDepth <= pps.getPpsRangeExtension().getDiffCuChromaQpOffsetDepth() && pcSlice->getUseChromaQpAdj())
  {
    setCodeChromaQpAdjFlag(true);//设置使用ChromaQpOffset标志为真
  }

  if (pps.getTransquantBypassEnableFlag())//若允许使用TransquantBypass
  {
    m_pcEntropyCoder->encodeCUTransquantBypassFlag( pcCU, uiAbsPartIdx );//则编码TransquantBypass标志
  }

  if( !pcSlice->isIntra() )//不为帧内预测
  {
    m_pcEntropyCoder->encodeSkipFlag( pcCU, uiAbsPartIdx );//则还需编码SKIP模式标志
  }

  if( pcCU->isSkipped( uiAbsPartIdx ) )//若该Cu使用skip模式
  {
    m_pcEntropyCoder->encodeMergeIndex( pcCU, uiAbsPartIdx );//则还需编码mv在merge候选列表中的索引
    finishCU(pcCU,uiAbsPartIdx);//该Cu编码结束 
    return;//方法结束
  }
  //不为skip模式
  m_pcEntropyCoder->encodePredMode( pcCU, uiAbsPartIdx );//则编码预测模式
  m_pcEntropyCoder->encodePartSize( pcCU, uiAbsPartIdx, uiDepth );//编码Cu的Pu分割模式

  if (pcCU->isIntra( uiAbsPartIdx ) && pcCU->getPartitionSize( uiAbsPartIdx ) == SIZE_2Nx2N )//若该Cu为帧内预测且Pu分割模式为2Nx2N 则该Cu可能为PCM模式
  {
    m_pcEntropyCoder->encodeIPCMInfo( pcCU, uiAbsPartIdx );//则编码PCM信息 注:该方法会根据Cu是否使用PCM来编码相应信息 如不使用PCM模式时只编码PCM标志 使用PCM时 编码PCM标志及PCM像素值

    if(pcCU->getIPCMFlag(uiAbsPartIdx))//若该Cu使用PCM模式
    {
      // Encode slice finish
      finishCU(pcCU,uiAbsPartIdx);//该Cu编码结束 
      return;//方法结束
    }
  }

  // prediction Info ( Intra : direction mode, Inter : Mv, reference idx )
  m_pcEntropyCoder->encodePredInfo( pcCU, uiAbsPartIdx );//编码预测信息(帧内预测为预测方向模式 帧间预测为MV及参考图像索引)

  // Encode Coefficients
  Bool bCodeDQP = getdQPFlag();
  Bool codeChromaQpAdj = getCodeChromaQpAdjFlag();
  m_pcEntropyCoder->encodeCoeff( pcCU, uiAbsPartIdx, uiDepth, bCodeDQP, codeChromaQpAdj );//编码系数(包括变换系数 cbf QP值 ChromaQpAdj值等)
  setCodeChromaQpAdjFlag( codeChromaQpAdj );
  setdQPFlag( bCodeDQP );//还原标志

  // --- write terminating bit ---
  finishCU(pcCU,uiAbsPartIdx);////该Cu编码结束 写入结束位　告诉编码器停下来(算数编码需要结束位)
}

Int xCalcHADs8x8_ISlice(Pel *piOrg, Int iStrideOrg)//计算8×8的HAD变换系数绝对值(减去(0,0)处系数)之和
{
  Int k, i, j, jj;
  Int diff[64], m1[8][8], m2[8][8], m3[8][8], iSumHad = 0;

  for( k = 0; k < 64; k += 8 )
  {
    diff[k+0] = piOrg[0] ;
    diff[k+1] = piOrg[1] ;
    diff[k+2] = piOrg[2] ;
    diff[k+3] = piOrg[3] ;
    diff[k+4] = piOrg[4] ;
    diff[k+5] = piOrg[5] ;
    diff[k+6] = piOrg[6] ;
    diff[k+7] = piOrg[7] ;

    piOrg += iStrideOrg;
  }

  //horizontal
  for (j=0; j < 8; j++)
  {
    jj = j << 3;
    m2[j][0] = diff[jj  ] + diff[jj+4];
    m2[j][1] = diff[jj+1] + diff[jj+5];
    m2[j][2] = diff[jj+2] + diff[jj+6];
    m2[j][3] = diff[jj+3] + diff[jj+7];
    m2[j][4] = diff[jj  ] - diff[jj+4];
    m2[j][5] = diff[jj+1] - diff[jj+5];
    m2[j][6] = diff[jj+2] - diff[jj+6];
    m2[j][7] = diff[jj+3] - diff[jj+7];

    m1[j][0] = m2[j][0] + m2[j][2];
    m1[j][1] = m2[j][1] + m2[j][3];
    m1[j][2] = m2[j][0] - m2[j][2];
    m1[j][3] = m2[j][1] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][6];
    m1[j][5] = m2[j][5] + m2[j][7];
    m1[j][6] = m2[j][4] - m2[j][6];
    m1[j][7] = m2[j][5] - m2[j][7];

    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][5];
    m2[j][5] = m1[j][4] - m1[j][5];
    m2[j][6] = m1[j][6] + m1[j][7];
    m2[j][7] = m1[j][6] - m1[j][7];
  }

  //vertical
  for (i=0; i < 8; i++)
  {
    m3[0][i] = m2[0][i] + m2[4][i];
    m3[1][i] = m2[1][i] + m2[5][i];
    m3[2][i] = m2[2][i] + m2[6][i];
    m3[3][i] = m2[3][i] + m2[7][i];
    m3[4][i] = m2[0][i] - m2[4][i];
    m3[5][i] = m2[1][i] - m2[5][i];
    m3[6][i] = m2[2][i] - m2[6][i];
    m3[7][i] = m2[3][i] - m2[7][i];

    m1[0][i] = m3[0][i] + m3[2][i];
    m1[1][i] = m3[1][i] + m3[3][i];
    m1[2][i] = m3[0][i] - m3[2][i];
    m1[3][i] = m3[1][i] - m3[3][i];
    m1[4][i] = m3[4][i] + m3[6][i];
    m1[5][i] = m3[5][i] + m3[7][i];
    m1[6][i] = m3[4][i] - m3[6][i];
    m1[7][i] = m3[5][i] - m3[7][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
  }

  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 8; j++)
    {
      iSumHad += abs(m2[i][j]);
    }
  }
  iSumHad -= abs(m2[0][0]);
  iSumHad =(iSumHad+2)>>2;
  return(iSumHad);
}

Int  TEncCu::updateCtuDataISlice(TComDataCU* pCtu, Int width, Int height)//计算Ctu中8*8 HAD变换系数绝对值(减去(0,0)处系数)之和
{
  Int  xBl, yBl;
  const Int iBlkSize = 8;

  Pel* pOrgInit   = pCtu->getPic()->getPicYuvOrg()->getAddr(COMPONENT_Y, pCtu->getCtuRsAddr(), 0);
  Int  iStrideOrig = pCtu->getPic()->getPicYuvOrg()->getStride(COMPONENT_Y);
  Pel  *pOrg;

  Int iSumHad = 0;
  for ( yBl=0; (yBl+iBlkSize)<=height; yBl+= iBlkSize)//遍历Ctu中所有8*8的块
  {
    for ( xBl=0; (xBl+iBlkSize)<=width; xBl+= iBlkSize)
    {
      pOrg = pOrgInit + iStrideOrig*yBl + xBl;
      iSumHad += xCalcHADs8x8_ISlice(pOrg, iStrideOrig);
    }
  }
  return(iSumHad);
}

/** check RD costs for a CU block encoded with merge
 * \param rpcBestCU
 * \param rpcTempCU
 * \param earlyDetectionSkipMode
 */
Void TEncCu::xCheckRDCostMerge2Nx2N( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU DEBUG_STRING_FN_DECLARE(sDebug), Bool *earlyDetectionSkipMode )
{
  assert( rpcTempCU->getSlice()->getSliceType() != I_SLICE );//帧内预测无法使用merge
  if(getFastDeltaQp())//快速deltaQP模式中不检测merge
  {
    return;   // never check merge in fast deltaqp mode
  }
  TComMvField  cMvFieldNeighbours[2 * MRG_MAX_NUM_CANDS]; // double length for mv of both lists//乘以2 是因为两个参考图像列表
  UChar uhInterDirNeighbours[MRG_MAX_NUM_CANDS];
  Int numValidMergeCand = 0;
  const Bool bTransquantBypassFlag = rpcTempCU->getCUTransquantBypass(0);//UTransquantBypass标志

  for( UInt ui = 0; ui < rpcTempCU->getSlice()->getMaxNumMergeCand(); ++ui )
  {
    uhInterDirNeighbours[ui] = 0;
  }//初始化候选MV的预测方向
  UChar uhDepth = rpcTempCU->getDepth( 0 );
  rpcTempCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uhDepth ); // interprets depth relative to CTU level//将Cu的Pu分割模式设置为2Nx2N (设置为2Nx2N是因为从Cu块使用merge检查率失真损耗 2Nx2N下Pu块大小同Cu)
  rpcTempCU->getInterMergeCandidates( 0, 0, cMvFieldNeighbours,uhInterDirNeighbours, numValidMergeCand );//得到该Cu的merg候选MV列表

  Int mergeCandBuffer[MRG_MAX_NUM_CANDS];
  for( UInt ui = 0; ui < numValidMergeCand; ++ui )
  {
    mergeCandBuffer[ui] = 0;
  }

  Bool bestIsSkip = false;

  UInt iteration;
  if ( rpcTempCU->isLosslessCoded(0))//lossless模式下 残差不能强制为0(因为lossless模式为无损 强制残差为0 则无法保证为无损)
  {
    iteration = 1;//uiNoResidual 不可能为1
  }
  else
  {
    iteration = 2;
  }
  DEBUG_STRING_NEW(bestStr)

  for( UInt uiNoResidual = 0; uiNoResidual < iteration; ++uiNoResidual )//遍历是否强制残差为0两种情况
  {
    for( UInt uiMergeCand = 0; uiMergeCand < numValidMergeCand; ++uiMergeCand )//遍历merg候选列表中所有MV
    {
      if(!(uiNoResidual==1 && mergeCandBuffer[uiMergeCand]==1))//mergeCandBuffer标志为真 则无需检测强制残差为0的情况
      {
        if( !(bestIsSkip && uiNoResidual == 0) )//bestIsSkip为真 则说明不再检测merge候选列表中其他MV
        {
          DEBUG_STRING_NEW(tmpStr)
          // set MC parameters//设置运动补偿参数
          rpcTempCU->setPredModeSubParts( MODE_INTER, 0, uhDepth ); // interprets depth relative to CTU level
          rpcTempCU->setCUTransquantBypassSubParts( bTransquantBypassFlag, 0, uhDepth );
          rpcTempCU->setChromaQpAdjSubParts( bTransquantBypassFlag ? 0 : m_cuChromaQpOffsetIdxPlus1, 0, uhDepth );
          rpcTempCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uhDepth ); // interprets depth relative to CTU level
          rpcTempCU->setMergeFlagSubParts( true, 0, 0, uhDepth ); // interprets depth relative to CTU level
          rpcTempCU->setMergeIndexSubParts( uiMergeCand, 0, 0, uhDepth ); // interprets depth relative to CTU level
          rpcTempCU->setInterDirSubParts( uhInterDirNeighbours[uiMergeCand], 0, 0, uhDepth ); // interprets depth relative to CTU level
          rpcTempCU->getCUMvField( REF_PIC_LIST_0 )->setAllMvField( cMvFieldNeighbours[0 + 2*uiMergeCand], SIZE_2Nx2N, 0, 0 ); // interprets depth relative to rpcTempCU level
          rpcTempCU->getCUMvField( REF_PIC_LIST_1 )->setAllMvField( cMvFieldNeighbours[1 + 2*uiMergeCand], SIZE_2Nx2N, 0, 0 ); // interprets depth relative to rpcTempCU level

          // do MC
          m_pcPredSearch->motionCompensation ( rpcTempCU, m_ppcPredYuvTemp[uhDepth] );//用merge候选列表中的MV做运动补偿
          // estimate residual and encode everything
          m_pcPredSearch->encodeResAndCalcRdInterCU( rpcTempCU,
                                                     m_ppcOrigYuv    [uhDepth],
                                                     m_ppcPredYuvTemp[uhDepth],
                                                     m_ppcResiYuvTemp[uhDepth],
                                                     m_ppcResiYuvBest[uhDepth],
                                                     m_ppcRecoYuvTemp[uhDepth],
                                                     (uiNoResidual != 0) DEBUG_STRING_PASS_INTO(tmpStr) );//计算merge列表中该MV的总损耗

#if DEBUG_STRING
          DebugInterPredResiReco(tmpStr, *(m_ppcPredYuvTemp[uhDepth]), *(m_ppcResiYuvBest[uhDepth]), *(m_ppcRecoYuvTemp[uhDepth]), DebugStringGetPredModeMask(rpcTempCU->getPredictionMode(0)));
#endif

          if ((uiNoResidual == 0) && (rpcTempCU->getQtRootCbf(0) == 0))//当允许存在非零残差时变换系数为0 故设置标志 强制残差为0时不再需要检测该MV
          {
            // If no residual when allowing for one, then set mark to not try case where residual is forced to 0
            mergeCandBuffer[uiMergeCand] = 1;
          }

          Int orgQP = rpcTempCU->getQP( 0 );
          xCheckDQP( rpcTempCU );//设置 编码CU的QP值
          xCheckBestMode(rpcBestCU, rpcTempCU, uhDepth DEBUG_STRING_PASS_INTO(bestStr) DEBUG_STRING_PASS_INTO(tmpStr));//更新最优的Cu信息至rpcBestCU

          rpcTempCU->initEstData( uhDepth, orgQP, bTransquantBypassFlag );//初始化rpcTempCU的预测信息

          if( m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip )//若允许使用快速merge判决
          {
            bestIsSkip = rpcBestCU->getQtRootCbf(0) == 0;//rpcBestCU中不存在非零变换系数 则bestIsSkip置为真(bestIsSkip为真 则说明不再检测merge候选列表中其他MV)
          }
        }
      }
    }

    if(uiNoResidual == 0 && m_pcEncCfg->getUseEarlySkipDetection())//不强制残差为0(强制残差为0 cbf一定为0 )且使用早期跳出策略(EarlySkip)
    {
      if(rpcBestCU->getQtRootCbf( 0 ) == 0)//该CU不存在非零变换系数
      {
        if( rpcBestCU->getMergeFlag( 0 ))//该Cu为merge得到的运动矢量
        {
          *earlyDetectionSkipMode = true;//DetectionSkip标志置为真
        }
        else if(m_pcEncCfg->getFastSearch() != SELECTIVE)//该Cu的运动矢量不为merge列表中的运动矢量
        {
          Int absoulte_MV=0;
          for ( UInt uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
          {
            if ( rpcBestCU->getSlice()->getNumRefIdx( RefPicList( uiRefListIdx ) ) > 0 )
            {
              TComCUMvField* pcCUMvField = rpcBestCU->getCUMvField(RefPicList( uiRefListIdx ));
              Int iHor = pcCUMvField->getMvd( 0 ).getAbsHor();
              Int iVer = pcCUMvField->getMvd( 0 ).getAbsVer();
              absoulte_MV+=iHor+iVer;
            }
          }//有效运动矢量差绝对值之和

          if(absoulte_MV == 0)//MVD绝对值之和为0
          {
            *earlyDetectionSkipMode = true;//DetectionSkip标志置为真
          }
        }
      }
    }
  }
  DEBUG_STRING_APPEND(sDebug, bestStr)
}


#if AMP_MRG
Void TEncCu::xCheckRDCostInter( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize DEBUG_STRING_FN_DECLARE(sDebug), Bool bUseMRG)//计算帧间预测Cu在给定Pu分割模式下的率失真 并根据率失真更新Cu最优信息
#else
Void TEncCu::xCheckRDCostInter( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize )
#endif
{
  DEBUG_STRING_NEW(sTest)

  if(getFastDeltaQp())//使用fast deltaqp mode
  {
    const TComSPS &sps=*(rpcTempCU->getSlice()->getSPS());
    const UInt fastDeltaQPCuMaxSize = Clip3(sps.getMaxCUHeight()>>(sps.getLog2DiffMaxMinCodingBlockSize()), sps.getMaxCUHeight(), 32u);
    if(ePartSize != SIZE_2Nx2N || rpcTempCU->getWidth( 0 ) > fastDeltaQPCuMaxSize)//fast deltaqp mode下 帧间预测只有2Nx2NPu的分割模式需要被检测
    {
      return; // only check necessary 2Nx2N Inter in fast deltaqp mode
    }
  }

  // prior to this, rpcTempCU will have just been reset using rpcTempCU->initEstData( uiDepth, iQP, bIsLosslessMode );
  UChar uhDepth = rpcTempCU->getDepth( 0 );

  rpcTempCU->setPartSizeSubParts  ( ePartSize,  0, uhDepth );//设置Cu的Pu分割模式
  rpcTempCU->setPredModeSubParts  ( MODE_INTER, 0, uhDepth );//设置Cu的预测模式
  rpcTempCU->setChromaQpAdjSubParts( rpcTempCU->getCUTransquantBypass(0) ? 0 : m_cuChromaQpOffsetIdxPlus1, 0, uhDepth );//设置ChromaQpAdj值

#if AMP_MRG
  rpcTempCU->setMergeAMP (true);
  m_pcPredSearch->predInterSearch ( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcRecoYuvTemp[uhDepth] DEBUG_STRING_PASS_INTO(sTest), false, bUseMRG );//帧间搜索(运动矢量及参考图像) 
#else
  m_pcPredSearch->predInterSearch ( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcRecoYuvTemp[uhDepth] );
#endif

#if AMP_MRG
  if ( !rpcTempCU->getMergeAMP() )
  {
    return;
  }
#endif

  m_pcPredSearch->encodeResAndCalcRdInterCU( rpcTempCU, m_ppcOrigYuv[uhDepth], m_ppcPredYuvTemp[uhDepth], m_ppcResiYuvTemp[uhDepth], m_ppcResiYuvBest[uhDepth], m_ppcRecoYuvTemp[uhDepth], false DEBUG_STRING_PASS_INTO(sTest) );//根据帧间搜索后的MV及参考图像计算失真及编码比特数
  rpcTempCU->getTotalCost()  = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );//计算Cu在该Pu分割模式下的最优总损耗

#if DEBUG_STRING
  DebugInterPredResiReco(sTest, *(m_ppcPredYuvTemp[uhDepth]), *(m_ppcResiYuvBest[uhDepth]), *(m_ppcRecoYuvTemp[uhDepth]), DebugStringGetPredModeMask(rpcTempCU->getPredictionMode(0)));
#endif

  xCheckDQP( rpcTempCU );//设置 编码CU的QP值
  xCheckBestMode(rpcBestCU, rpcTempCU, uhDepth DEBUG_STRING_PASS_INTO(sDebug) DEBUG_STRING_PASS_INTO(sTest));//更新最优Cu信息至rpcBestCU
}

Void TEncCu::xCheckRDCostIntra( TComDataCU *&rpcBestCU,
                                TComDataCU *&rpcTempCU,
                                Double      &cost,
                                PartSize     eSize
                                DEBUG_STRING_FN_DECLARE(sDebug) )//计算帧内预测Cu在给定Pu分割模式下的率失真并根据率失真更新Cu最优信息
{
  DEBUG_STRING_NEW(sTest)

  if(getFastDeltaQp())
  {
    const TComSPS &sps=*(rpcTempCU->getSlice()->getSPS());
    const UInt fastDeltaQPCuMaxSize = Clip3(sps.getMaxCUHeight()>>(sps.getLog2DiffMaxMinCodingBlockSize()), sps.getMaxCUHeight(), 32u);
    if(rpcTempCU->getWidth( 0 ) > fastDeltaQPCuMaxSize)//fast deltaqp mode下 帧内预测只有2Nx2NPu的分割模式需要被检测
    {
      return; // only check necessary 2Nx2N Intra in fast deltaqp mode
    }
  }

  UInt uiDepth = rpcTempCU->getDepth( 0 );

  rpcTempCU->setSkipFlagSubParts( false, 0, uiDepth );//设置CU的skip标志为false (只有帧间预测才可使用skip模式)

  rpcTempCU->setPartSizeSubParts( eSize, 0, uiDepth );//设置Cu的Pu分割模式
  rpcTempCU->setPredModeSubParts( MODE_INTRA, 0, uiDepth );//设置Cu的预测模式
  rpcTempCU->setChromaQpAdjSubParts( rpcTempCU->getCUTransquantBypass(0) ? 0 : m_cuChromaQpOffsetIdxPlus1, 0, uiDepth );//设置ChromaQpAdj值

  Pel resiLuma[NUMBER_OF_STORED_RESIDUAL_TYPES][MAX_CU_SIZE * MAX_CU_SIZE];
  
  m_pcPredSearch->estIntraPredLumaQT( rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], m_ppcResiYuvTemp[uiDepth], m_ppcRecoYuvTemp[uiDepth], resiLuma DEBUG_STRING_PASS_INTO(sTest) );//计算该Cu在该Pu分割模式下的最优Tu分割及失真

  m_ppcRecoYuvTemp[uiDepth]->copyToPicComponent(COMPONENT_Y, rpcTempCU->getPic()->getPicYuvRec(), rpcTempCU->getCtuRsAddr(), rpcTempCU->getZorderIdxInCtu() );//将该Cu的重建像素保存至该Cu的重建图像中

  if (rpcBestCU->getPic()->getChromaFormat()!=CHROMA_400)//若该Cu存在色度分量 则还需计算色度分量QT
  {
    m_pcPredSearch->estIntraPredChromaQT( rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], m_ppcResiYuvTemp[uiDepth], m_ppcRecoYuvTemp[uiDepth], resiLuma DEBUG_STRING_PASS_INTO(sTest) );
  }

  m_pcEntropyCoder->resetBits();

  if ( rpcTempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
  {
    m_pcEntropyCoder->encodeCUTransquantBypassFlag( rpcTempCU, 0,          true );//若允许使用TransquantBypass 则需编码TransquantBypass标志
  }

  m_pcEntropyCoder->encodeSkipFlag ( rpcTempCU, 0,          true );//编码skip标志
  m_pcEntropyCoder->encodePredMode( rpcTempCU, 0,          true );//编码预测模式
  m_pcEntropyCoder->encodePartSize( rpcTempCU, 0, uiDepth, true );//编码Pu分割模式
  m_pcEntropyCoder->encodePredInfo( rpcTempCU, 0 );//编码预测信息
  m_pcEntropyCoder->encodeIPCMInfo(rpcTempCU, 0, true );//编码PCM信息

  // Encode Coefficients
  Bool bCodeDQP = getdQPFlag();
  Bool codeChromaQpAdjFlag = getCodeChromaQpAdjFlag();
  m_pcEntropyCoder->encodeCoeff( rpcTempCU, 0, uiDepth, bCodeDQP, codeChromaQpAdjFlag );//编码系数(包括变换系数 cbf QP值 ChromaQpAdj值等)
  setCodeChromaQpAdjFlag( codeChromaQpAdjFlag );
  setdQPFlag( bCodeDQP );//还原标志

  m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);

  rpcTempCU->getTotalBits() = m_pcEntropyCoder->getNumberOfWrittenBits();//编码总比特数
  rpcTempCU->getTotalBins() = ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
  rpcTempCU->getTotalCost() = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );//编码总损耗(未加上编码QP值的比特数)

  xCheckDQP( rpcTempCU );//设置 编码QP值

  cost = rpcTempCU->getTotalCost();//编码总损耗

  xCheckBestMode(rpcBestCU, rpcTempCU, uiDepth DEBUG_STRING_PASS_INTO(sDebug) DEBUG_STRING_PASS_INTO(sTest));////更新最优Cu信息至rpcBestCU
}


/** Check R-D costs for a CU with PCM mode.
 * \param rpcBestCU pointer to best mode CU data structure
 * \param rpcTempCU pointer to testing mode CU data structure
 * \returns Void
 *
 * \note Current PCM implementation encodes sample values in a lossless way. The distortion of PCM mode CUs are zero. PCM mode is selected if the best mode yields bits greater than that of PCM mode.
 */
Void TEncCu::xCheckIntraPCM( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU )
{
  if(getFastDeltaQp())
  {
    const TComSPS &sps=*(rpcTempCU->getSlice()->getSPS());
    const UInt fastDeltaQPCuMaxPCMSize = Clip3((UInt)1<<sps.getPCMLog2MinSize(), (UInt)1<<sps.getPCMLog2MaxSize(), 32u);
    if (rpcTempCU->getWidth( 0 ) > fastDeltaQPCuMaxPCMSize)//Cu大小大于fast deltaqp mode中PCM模式允许的最大Cu大小
    {
      return;   // only check necessary PCM in fast deltaqp mode
    }
  }
  
  UInt uiDepth = rpcTempCU->getDepth( 0 );

  rpcTempCU->setSkipFlagSubParts( false, 0, uiDepth );//编码skip标志

  rpcTempCU->setIPCMFlag(0, true);
  rpcTempCU->setIPCMFlagSubParts (true, 0, rpcTempCU->getDepth(0));//设置PCM标志为真
  rpcTempCU->setPartSizeSubParts( SIZE_2Nx2N, 0, uiDepth );//设置PU分割模式
  rpcTempCU->setPredModeSubParts( MODE_INTRA, 0, uiDepth );//设置预测模式 (PCM模式下 只能为帧内预测 2Nx2N)
  rpcTempCU->setTrIdxSubParts ( 0, 0, uiDepth );//设置TrIdx
  rpcTempCU->setChromaQpAdjSubParts( rpcTempCU->getCUTransquantBypass(0) ? 0 : m_cuChromaQpOffsetIdxPlus1, 0, uiDepth );//设置ChromaQpAdj值

  m_pcPredSearch->IPCMSearch( rpcTempCU, m_ppcOrigYuv[uiDepth], m_ppcPredYuvTemp[uiDepth], m_ppcResiYuvTemp[uiDepth], m_ppcRecoYuvTemp[uiDepth]);//计算PCM模式下的编码比特位 失真及总损耗

  m_pcRDGoOnSbacCoder->load(m_pppcRDSbacCoder[uiDepth][CI_CURR_BEST]);

  m_pcEntropyCoder->resetBits();

  if ( rpcTempCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
  {
    m_pcEntropyCoder->encodeCUTransquantBypassFlag( rpcTempCU, 0,          true );//若允许使用TransquantBypass 则需编码TransquantBypass标志
  }
  //编码预测信息
  m_pcEntropyCoder->encodeSkipFlag ( rpcTempCU, 0,          true );
  m_pcEntropyCoder->encodePredMode ( rpcTempCU, 0,          true );
  m_pcEntropyCoder->encodePartSize ( rpcTempCU, 0, uiDepth, true );
  m_pcEntropyCoder->encodeIPCMInfo ( rpcTempCU, 0, true );//编码PCM信息(编码PCM标志 PCM像素值)

  m_pcRDGoOnSbacCoder->store(m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]);

  rpcTempCU->getTotalBits() = m_pcEntropyCoder->getNumberOfWrittenBits();//PCM模式下的编码总比特数
  rpcTempCU->getTotalBins() = ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
  rpcTempCU->getTotalCost() = m_pcRdCost->calcRdCost( rpcTempCU->getTotalBits(), rpcTempCU->getTotalDistortion() );//PCM模式下的总损耗(为编码QP值时的总损耗)

  xCheckDQP( rpcTempCU );//设置 编码QP值
  DEBUG_STRING_NEW(a)
  DEBUG_STRING_NEW(b)
  xCheckBestMode(rpcBestCU, rpcTempCU, uiDepth DEBUG_STRING_PASS_INTO(a) DEBUG_STRING_PASS_INTO(b));//更新最优Cu信息至rpcBestCU
}

/** check whether current try is the best with identifying the depth of current try
 * \param rpcBestCU
 * \param rpcTempCU
 * \param uiDepth
 */
Void TEncCu::xCheckBestMode( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth DEBUG_STRING_FN_DECLARE(sParent) DEBUG_STRING_FN_DECLARE(sTest) DEBUG_STRING_PASS_INTO(Bool bAddSizeInfo) )//根据Cu的总损耗更新最优Cu信息至rpcBestCU
{
  if( rpcTempCU->getTotalCost() < rpcBestCU->getTotalCost() )//rpcTempCU总损耗优于rpcBestCU
  {
    TComYuv* pcYuv;
    // Change Information data
    TComDataCU* pcCU = rpcBestCU;
    rpcBestCU = rpcTempCU;
    rpcTempCU = pcCU;//则交换两者Cu信息

    // Change Prediction data
    pcYuv = m_ppcPredYuvBest[uiDepth];
    m_ppcPredYuvBest[uiDepth] = m_ppcPredYuvTemp[uiDepth];
    m_ppcPredYuvTemp[uiDepth] = pcYuv;//交换预测像素值

    // Change Reconstruction data
    pcYuv = m_ppcRecoYuvBest[uiDepth];
    m_ppcRecoYuvBest[uiDepth] = m_ppcRecoYuvTemp[uiDepth];
    m_ppcRecoYuvTemp[uiDepth] = pcYuv;//交换重建像素值

    pcYuv = NULL;
    pcCU  = NULL;

    // store temp best CI for next CU coding
    m_pppcRDSbacCoder[uiDepth][CI_TEMP_BEST]->store(m_pppcRDSbacCoder[uiDepth][CI_NEXT_BEST]);


#if DEBUG_STRING
    DEBUG_STRING_SWAP(sParent, sTest)
    const PredMode predMode=rpcBestCU->getPredictionMode(0);
    if ((DebugOptionList::DebugString_Structure.getInt()&DebugStringGetPredModeMask(predMode)) && bAddSizeInfo)
    {
      std::stringstream ss(stringstream::out);
      ss <<"###: " << (predMode==MODE_INTRA?"Intra   ":"Inter   ") << partSizeToString[rpcBestCU->getPartitionSize(0)] << " CU at " << rpcBestCU->getCUPelX() << ", " << rpcBestCU->getCUPelY() << " width=" << UInt(rpcBestCU->getWidth(0)) << std::endl;
      sParent+=ss.str();
    }
#endif
  }
}

Void TEncCu::xCheckDQP( TComDataCU* pcCU )//设置并编码Qp值
{
  UInt uiDepth = pcCU->getDepth( 0 );

  const TComPPS &pps = *(pcCU->getSlice()->getPPS());
  if ( pps.getUseDQP() && uiDepth <= pps.getMaxCuDQPDepth() )
  {
    if ( pcCU->getQtRootCbf( 0) )//存在非零变换系数
    {
      m_pcEntropyCoder->resetBits();
      m_pcEntropyCoder->encodeQP( pcCU, 0, false );//编码deltaQP值
      pcCU->getTotalBits() += m_pcEntropyCoder->getNumberOfWrittenBits(); // dQP bits//加上编码deltaQP值的总编码比特数
      pcCU->getTotalBins() += ((TEncBinCABAC *)((TEncSbac*)m_pcEntropyCoder->m_pcEntropyCoderIf)->getEncBinIf())->getBinsCoded();
      pcCU->getTotalCost() = m_pcRdCost->calcRdCost( pcCU->getTotalBits(), pcCU->getTotalDistortion() );//更新编码总损耗
    }
    else//不存在残差值 也就说明不需要准确编码QP值 直接设置QP值为预测QP值 这样可以有效节省编码deltaQP的比特数
    {
      pcCU->setQPSubParts( pcCU->getRefQP( 0 ), 0, uiDepth ); // set QP to default QP
    }
  }
}

Void TEncCu::xCopyAMVPInfo (AMVPInfo* pSrc, AMVPInfo* pDst)//复制AMVP信息
{
  pDst->iN = pSrc->iN;
  for (Int i = 0; i < pSrc->iN; i++)
  {
    pDst->m_acMvCand[i] = pSrc->m_acMvCand[i];//复制每一个候选运动矢量
  }
}
Void TEncCu::xCopyYuv2Pic(TComPic* rpcPic, UInt uiCUAddr, UInt uiAbsPartIdx, UInt uiDepth, UInt uiSrcDepth )
{
  UInt uiAbsPartIdxInRaster = g_auiZscanToRaster[uiAbsPartIdx];
  UInt uiSrcBlkWidth = rpcPic->getNumPartInCtuWidth() >> (uiSrcDepth);
  UInt uiBlkWidth    = rpcPic->getNumPartInCtuWidth() >> (uiDepth);
  UInt uiPartIdxX = ( ( uiAbsPartIdxInRaster % rpcPic->getNumPartInCtuWidth() ) % uiSrcBlkWidth) / uiBlkWidth;
  UInt uiPartIdxY = ( ( uiAbsPartIdxInRaster / rpcPic->getNumPartInCtuWidth() ) % uiSrcBlkWidth) / uiBlkWidth;
  UInt uiPartIdx = uiPartIdxY * ( uiSrcBlkWidth / uiBlkWidth ) + uiPartIdxX;//位置信息
  m_ppcRecoYuvBest[uiSrcDepth]->copyToPicYuv( rpcPic->getPicYuvRec (), uiCUAddr, uiAbsPartIdx, uiDepth - uiSrcDepth, uiPartIdx);//将最优重建像素复制(保存)到重建图像

  m_ppcPredYuvBest[uiSrcDepth]->copyToPicYuv( rpcPic->getPicYuvPred (), uiCUAddr, uiAbsPartIdx, uiDepth - uiSrcDepth, uiPartIdx);//将最优预测像素复制(保存)到预测图像
}

Void TEncCu::xCopyYuv2Tmp( UInt uiPartUnitIdx, UInt uiNextDepth )//将子Cu最优预测和分割模式下的预测像素值和重建像素值分别保存至m_ppcRecoYuvTemp和m_ppcPredYuvBest中上一深度对应位置
{
  UInt uiCurrDepth = uiNextDepth - 1;
  m_ppcRecoYuvBest[uiNextDepth]->copyToPartYuv( m_ppcRecoYuvTemp[uiCurrDepth], uiPartUnitIdx );
  m_ppcPredYuvBest[uiNextDepth]->copyToPartYuv( m_ppcPredYuvBest[uiCurrDepth], uiPartUnitIdx);//???为什么不是复制到m_ppcPredYuvTemp[uiCurrDepth]
}

/** Function for filling the PCM buffer of a CU using its original sample array
 * \param pCU pointer to current CU
 * \param pOrgYuv pointer to original sample array
 */
Void TEncCu::xFillPCMBuffer     ( TComDataCU* pCU, TComYuv* pOrgYuv )////将原像素值复制到PCM buffer
{
  const ChromaFormat format = pCU->getPic()->getChromaFormat();
  const UInt numberValidComponents = getNumberValidComponents(format);
  for (UInt componentIndex = 0; componentIndex < numberValidComponents; componentIndex++)//遍历所有分量
  {
    const ComponentID component = ComponentID(componentIndex);

    const UInt width  = pCU->getWidth(0)  >> getComponentScaleX(component, format);
    const UInt height = pCU->getHeight(0) >> getComponentScaleY(component, format);//该分量Cu块的宽和高

    Pel *source      = pOrgYuv->getAddr(component, 0, width);
    Pel *destination = pCU->getPCMSample(component);//PCM buffer起始地址

    const UInt sourceStride = pOrgYuv->getStride(component);

    for (Int line = 0; line < height; line++)//复制Cu块中的所有像素值至PCM buffer
    {
      for (Int column = 0; column < width; column++)
      {
        destination[column] = source[column];
      }

      source      += sourceStride;
      destination += width;
    }
  }
}

#if ADAPTIVE_QP_SELECTION
/** Collect ARL statistics from one block
  */
Int TEncCu::xTuCollectARLStats(TCoeff* rpcCoeff, TCoeff* rpcArlCoeff, Int NumCoeffInCU, Double* cSum, UInt* numSamples )
{
  for( Int n = 0; n < NumCoeffInCU; n++ )
  {
    TCoeff u = abs( rpcCoeff[ n ] );
    TCoeff absc = rpcArlCoeff[ n ];

    if( u != 0 )
    {
      if( u < LEVEL_RANGE )
      {
        cSum[ u ] += ( Double )absc;
        numSamples[ u ]++;
      }
      else
      {
        cSum[ LEVEL_RANGE ] += ( Double )absc - ( Double )( u << ARL_C_PRECISION );
        numSamples[ LEVEL_RANGE ]++;
      }
    }
  }

  return 0;
}

//! Collect ARL statistics from one CTU
Void TEncCu::xCtuCollectARLStats(TComDataCU* pCtu )
{
  Double cSum[ LEVEL_RANGE + 1 ];     //: the sum of DCT coefficients corresponding to data type and quantization output
  UInt numSamples[ LEVEL_RANGE + 1 ]; //: the number of coefficients corresponding to data type and quantization output

  TCoeff* pCoeffY = pCtu->getCoeff(COMPONENT_Y);
  TCoeff* pArlCoeffY = pCtu->getArlCoeff(COMPONENT_Y);
  const TComSPS &sps = *(pCtu->getSlice()->getSPS());

  const UInt uiMinCUWidth = sps.getMaxCUWidth() >> sps.getMaxTotalCUDepth(); // NOTE: ed - this is not the minimum CU width. It is the square-root of the number of coefficients per part.
  const UInt uiMinNumCoeffInCU = 1 << uiMinCUWidth;                          // NOTE: ed - what is this?

  memset( cSum, 0, sizeof( Double )*(LEVEL_RANGE+1) );
  memset( numSamples, 0, sizeof( UInt )*(LEVEL_RANGE+1) );

  // Collect stats to cSum[][] and numSamples[][]
  for(Int i = 0; i < pCtu->getTotalNumPart(); i ++ )
  {
    UInt uiTrIdx = pCtu->getTransformIdx(i);

    if(pCtu->isInter(i) && pCtu->getCbf( i, COMPONENT_Y, uiTrIdx ) )
    {
      xTuCollectARLStats(pCoeffY, pArlCoeffY, uiMinNumCoeffInCU, cSum, numSamples);
    }//Note that only InterY is processed. QP rounding is based on InterY data only.

    pCoeffY  += uiMinNumCoeffInCU;
    pArlCoeffY  += uiMinNumCoeffInCU;
  }

  for(Int u=1; u<LEVEL_RANGE;u++)
  {
    m_pcTrQuant->getSliceSumC()[u] += cSum[ u ] ;
    m_pcTrQuant->getSliceNSamples()[u] += numSamples[ u ] ;
  }
  m_pcTrQuant->getSliceSumC()[LEVEL_RANGE] += cSum[ LEVEL_RANGE ] ;
  m_pcTrQuant->getSliceNSamples()[LEVEL_RANGE] += numSamples[ LEVEL_RANGE ] ;
}
#endif
//! \}
