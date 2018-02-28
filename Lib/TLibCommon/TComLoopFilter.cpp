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

/** \file     TComLoopFilter.cpp
    \brief    deblocking filter
*/

#include "TComLoopFilter.h"
#include "TComSlice.h"
#include "TComMv.h"
#include "TComTU.h"

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constants
// ====================================================================================================================

//#define   EDGE_VER    0
//#define   EDGE_HOR    1

#define DEFAULT_INTRA_TC_OFFSET 2 ///< Default intra TC offset

// ====================================================================================================================
// Tables
// ====================================================================================================================

const UChar TComLoopFilter::sm_tcTable[MAX_QP + 1 + DEFAULT_INTRA_TC_OFFSET] =
{//阈值tc表 tc为边界处像素值差别的判决门限　　　
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,5,5,6,6,7,8,9,10,11,13,14,16,18,20,22,24
};

const UChar TComLoopFilter::sm_betaTable[MAX_QP + 1] =
{//阈值β表　β为滤波开关的判决门限
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6,7,8,9,10,11,12,13,14,15,16,17,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60,62,64
};

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TComLoopFilter::TComLoopFilter()
: m_uiNumPartitions(0)
, m_bLFCrossTileBoundary(true)
{
  for( Int edgeDir = 0; edgeDir < NUM_EDGE_DIR; edgeDir++ )
  {
    m_aapucBS       [edgeDir] = NULL;
    m_aapbEdgeFilter[edgeDir] = NULL;
  }
}

TComLoopFilter::~TComLoopFilter()
{
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================
Void TComLoopFilter::setCfg( Bool bLFCrossTileBoundary )
{
  m_bLFCrossTileBoundary = bLFCrossTileBoundary;
}

Void TComLoopFilter::create( UInt uiMaxCUDepth )
{
  destroy();
  m_uiNumPartitions = 1 << ( uiMaxCUDepth<<1 );//一个CTU中最小块的个数（４＊４）
  for( Int edgeDir = 0; edgeDir < NUM_EDGE_DIR; edgeDir++ )
  {
    m_aapucBS       [edgeDir] = new UChar[m_uiNumPartitions];
    m_aapbEdgeFilter[edgeDir] = new Bool [m_uiNumPartitions];
  }
}

Void TComLoopFilter::destroy()
{
  for( Int edgeDir = 0; edgeDir < NUM_EDGE_DIR; edgeDir++ )
  {
    if (m_aapucBS[edgeDir] != NULL)
    {
      delete [] m_aapucBS[edgeDir];
      m_aapucBS[edgeDir] = NULL;
    }

    if (m_aapbEdgeFilter[edgeDir])
    {
      delete [] m_aapbEdgeFilter[edgeDir];
      m_aapbEdgeFilter[edgeDir] = NULL;
    }
  }
}

/**
 - call deblocking function for every CU
 .
 \param  pcPic   picture class (TComPic) pointer
 */
Void TComLoopFilter::loopFilterPic( TComPic* pcPic )
{
  // Horizontal filtering
  for ( UInt ctuRsAddr = 0; ctuRsAddr < pcPic->getNumberOfCtusInFrame(); ctuRsAddr++ )//遍历所有的CTU
  {
    TComDataCU* pCtu = pcPic->getCtu( ctuRsAddr );//得到CTU

    ::memset( m_aapucBS       [EDGE_VER], 0, sizeof( UChar ) * m_uiNumPartitions );//给m_aapucBS[EDGE_VER]（边界强度）　m_aapbEdgeFilter[EDGE_VER]（滤波标志）初始值赋零
    ::memset( m_aapbEdgeFilter[EDGE_VER], 0, sizeof( Bool  ) * m_uiNumPartitions );

    // CU-based deblocking
    xDeblockCU( pCtu, 0, 0, EDGE_VER );//当前CTU垂直方向去方块
  }

  // Vertical filtering
  for ( UInt ctuRsAddr = 0; ctuRsAddr < pcPic->getNumberOfCtusInFrame(); ctuRsAddr++ )//遍历所有的CTU
  {
    TComDataCU* pCtu = pcPic->getCtu( ctuRsAddr );

    ::memset( m_aapucBS       [EDGE_HOR], 0, sizeof( UChar ) * m_uiNumPartitions );
    ::memset( m_aapbEdgeFilter[EDGE_HOR], 0, sizeof( Bool  ) * m_uiNumPartitions );

    // CU-based deblocking
    xDeblockCU( pCtu, 0, 0, EDGE_HOR );///当前CTU水平方向去方块
  }
}


// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

/**
 Deblocking filter process in CU-based (the same function as conventional's)

 \param pcCU             Pointer to CTU/CU structure
 \param uiAbsZorderIdx   Position in CU
 \param uiDepth          Depth in CU
 \param edgeDir          the direction of the edge in block boundary (horizontal/vertical), which is added newly
*/
Void TComLoopFilter::xDeblockCU( TComDataCU* pcCU, UInt uiAbsZorderIdx, UInt uiDepth, DeblockEdgeDir edgeDir )
{//uiAbsZorderIdx为　（４＊４）最小块为基本单位　TCU块按Z形扫描　的索引
  if(pcCU->getPic()==0||pcCU->getPartitionSize(uiAbsZorderIdx)==NUMBER_OF_PART_SIZES)//pic无效或AbsZorderIdx无效　getPartitionSize默认值为NUMBER_OF_PART_SIZES
  {
    return;
  }
  TComPic* pcPic     = pcCU->getPic();
  UInt uiCurNumParts = pcPic->getNumPartitionsInCtu() >> (uiDepth<<1);//当前深度CU中最小块的个数（４＊４）
  UInt uiQNumParts   = uiCurNumParts>>2;//当前深度下一深度最小块的个数
  const TComSPS &sps = *(pcCU->getSlice()->getSPS());//sps信息

  if( pcCU->getDepth(uiAbsZorderIdx) > uiDepth )//存在更深一层子CU
  {
    for ( UInt uiPartIdx = 0; uiPartIdx < 4; uiPartIdx++, uiAbsZorderIdx+=uiQNumParts )//对四个子CU去方块滤波
    {
      UInt uiLPelX   = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsZorderIdx] ];//CU左上角X坐标
      UInt uiTPelY   = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsZorderIdx] ];//CU左上角Y坐标
      if( ( uiLPelX < sps.getPicWidthInLumaSamples() ) && ( uiTPelY < sps.getPicHeightInLumaSamples() ) )//X Y坐标在规定范围内
      {
        xDeblockCU( pcCU, uiAbsZorderIdx, uiDepth+1, edgeDir );//（递归）下一层去方块滤波
      }
    }
    return;
  }

  xSetLoopfilterParam( pcCU, uiAbsZorderIdx );//设置环路滤波参数（不同边界滤波标志值）
  TComTURecurse tuRecurse(pcCU, uiAbsZorderIdx);//当前CU最上层TU块（即与CU等大小TU块）
  xSetEdgefilterTU   ( tuRecurse );//设置TU块边界滤波标志
  xSetEdgefilterPU   ( pcCU, uiAbsZorderIdx );//设置PU块边界滤波标志

  const UInt uiPelsInPart = sps.getMaxCUWidth() >> sps.getMaxTotalCUDepth();//最小TU块每行像素数(4)

  for( UInt uiPartIdx = uiAbsZorderIdx; uiPartIdx < uiAbsZorderIdx + uiCurNumParts; uiPartIdx++ )//遍历当前CU块的每个最小块（４＊４）
  {
    UInt uiBSCheck;//边界强度检查标志
    if( uiPelsInPart == 4 )//TU最小块为４＊４时
    {//去方块滤波是以８＊８的块为单位　垂直边界以８＊４为基本单位　水平边界以４＊８为基本单位　故每两个４＊４最小之间边界不要计算边界强度及滤波
      uiBSCheck = (edgeDir == EDGE_VER && uiPartIdx%2 == 0) || (edgeDir == EDGE_HOR && (uiPartIdx-((uiPartIdx>>2)<<2))/2 == 0);
    }
    else
    {
      uiBSCheck = 1;
    }

    if ( m_aapbEdgeFilter[edgeDir][uiPartIdx] && uiBSCheck )//需要计算边界强度且允许滤波
    {
      xGetBoundaryStrengthSingle ( pcCU, edgeDir, uiPartIdx );//计算边界强度
    }
  }

  UInt PartIdxIncr = DEBLOCK_SMALLEST_BLOCK / uiPelsInPart ? DEBLOCK_SMALLEST_BLOCK / uiPelsInPart : 1 ;//边界滤波步长（以４＊４最小块为基本单位）

  UInt uiSizeInPU = pcPic->getNumPartInCtuWidth()>>(uiDepth);//深度为uiDepth的CU每行最小块的个数
  const ChromaFormat chFmt=pcPic->getChromaFormat();//色度格式
  const UInt shiftFactor  = edgeDir == EDGE_VER ? pcPic->getComponentScaleX(COMPONENT_Cb) : pcPic->getComponentScaleY(COMPONENT_Cb);//cr cb相对于luma的缩小倍数（移位表示）
  const Bool bAlwaysDoChroma=chFmt==CHROMA_444;

  for ( Int iEdge = 0; iEdge < uiSizeInPU ; iEdge+=PartIdxIncr)//遍历所有边界
  {
    xEdgeFilterLuma     ( pcCU, uiAbsZorderIdx, uiDepth, edgeDir, iEdge );//luma分量边界滤波
    if ( chFmt!=CHROMA_400 && (bAlwaysDoChroma ||
                               (uiPelsInPart>DEBLOCK_SMALLEST_BLOCK) ||
                               (iEdge % ( (DEBLOCK_SMALLEST_BLOCK<<shiftFactor)/uiPelsInPart ) ) == 0
                              )
       )//色度格式不为4:0:0　不为4:4:4 除去不需要滤波的色度分量边界
    {
      xEdgeFilterChroma   ( pcCU, uiAbsZorderIdx, uiDepth, edgeDir, iEdge );//chroma分量边界滤波
    }
  }
}

Void TComLoopFilter::xSetEdgefilterMultiple( TComDataCU*    pcCU,
                                             UInt           uiAbsZorderIdx,
                                             UInt           uiDepth,
                                             DeblockEdgeDir edgeDir,
                                             Int            iEdgeIdx,
                                             Bool           bValue,
                                             UInt           uiWidthInBaseUnits,
                                             UInt           uiHeightInBaseUnits,
                                             const TComRectangle *rect)
{//如果edgeDir为VER 则iEdgeIdx指明垂直边界哪一列  如果edgeDir为HOR 则iEdgeIdx指明水平边界哪一行
  if ( uiWidthInBaseUnits == 0 )
  {
    uiWidthInBaseUnits  = pcCU->getPic()->getNumPartInCtuWidth () >> uiDepth;//深度为uiDepth的CU每行最小块的个数
  }
  if ( uiHeightInBaseUnits == 0 )
  {
    uiHeightInBaseUnits = pcCU->getPic()->getNumPartInCtuHeight() >> uiDepth;//深度为uiDepth的CU每列最小块的个数
  }
  const UInt uiNumElem = edgeDir == EDGE_VER ? uiHeightInBaseUnits : uiWidthInBaseUnits;//根据滤波模式得到对最小块个数
  assert( uiNumElem > 0 );
  assert( uiWidthInBaseUnits > 0 );
  assert( uiHeightInBaseUnits > 0 );
  for( UInt ui = 0; ui < uiNumElem; ui++ )//以最小块为单位遍历该CU的列或行
  {
    const UInt uiBsIdx = xCalcBsIdx( pcCU, uiAbsZorderIdx, edgeDir, iEdgeIdx, ui, rect );//计算该最小块的索引（z形扫描）
    m_aapbEdgeFilter[edgeDir][uiBsIdx] = bValue;//为该边界设置滤波标志
    if (iEdgeIdx == 0)
    {//LeftEdge和TopEdge需要另设置BS
      m_aapucBS[edgeDir][uiBsIdx] = bValue;//设置边界强度
    }
  }
}

Void TComLoopFilter::xSetEdgefilterTU(  TComTU &rTu )//深度遍历 为所有TU边界设置滤波标志
{
  TComDataCU* pcCU  = rTu.getCU();//该TU块所在CU块
  UInt uiTransDepthTotal = rTu.GetTransformDepthTotal();//该TU块所在深度

  if( pcCU->getTransformIdx( rTu.GetAbsPartIdxTU() ) + pcCU->getDepth( rTu.GetAbsPartIdxTU()) > uiTransDepthTotal )//深度遍历所有TU块
  {
    TComTURecurse tuChild(rTu, false);//当前CT分成四等块 并左上块为第一块 section=0
    do
    {
      xSetEdgefilterTU( tuChild );
    } while (tuChild.nextSection(rTu));//section++ 并移到下一块TU
    return;
  }

  const TComRectangle &rect = rTu.getRect(COMPONENT_Y);
  const TComSPS &sps=*(pcCU->getSlice()->getSPS());

  const UInt uiWidthInBaseUnits  = rect.width  / (sps.getMaxCUWidth()  >> sps.getMaxTotalCUDepth());//宽基本块个数
  const UInt uiHeightInBaseUnits = rect.height / (sps.getMaxCUHeight() >> sps.getMaxTotalCUDepth());//高基本块个数

  xSetEdgefilterMultiple( pcCU, rTu.GetAbsPartIdxCU(), uiTransDepthTotal, EDGE_VER, 0, m_stLFCUParam.bInternalEdge, uiWidthInBaseUnits, uiHeightInBaseUnits, &rect );//TU块边界滤波并设置边界强度  滤波边界必为TU块的边界 故iEdgeIdx为0
  xSetEdgefilterMultiple( pcCU, rTu.GetAbsPartIdxCU(), uiTransDepthTotal, EDGE_HOR, 0, m_stLFCUParam.bInternalEdge, uiWidthInBaseUnits, uiHeightInBaseUnits, &rect );
}

Void TComLoopFilter::xSetEdgefilterPU( TComDataCU* pcCU, UInt uiAbsZorderIdx )
{//对PU块边界设置滤波边界
  const UInt uiDepth = pcCU->getDepth( uiAbsZorderIdx );//该CU块在CTU块中的深度
  const UInt uiWidthInBaseUnits  = pcCU->getPic()->getNumPartInCtuWidth () >> uiDepth;//该CU宽含有基本块的个数（以4为基本单位）
  const UInt uiHeightInBaseUnits = pcCU->getPic()->getNumPartInCtuHeight() >> uiDepth;//该CU高含有基本块的个数
  const UInt uiHWidthInBaseUnits  = uiWidthInBaseUnits  >> 1;//1/2分割
  const UInt uiHHeightInBaseUnits = uiHeightInBaseUnits >> 1;//1/2分割
  const UInt uiQWidthInBaseUnits  = uiWidthInBaseUnits  >> 2;//1/4分割
  const UInt uiQHeightInBaseUnits = uiHeightInBaseUnits >> 2;//1/4分割

  xSetEdgefilterMultiple( pcCU, uiAbsZorderIdx, uiDepth, EDGE_VER, 0, m_stLFCUParam.bLeftEdge );//该CU块左边界设置滤波标志及边界强度
  xSetEdgefilterMultiple( pcCU, uiAbsZorderIdx, uiDepth, EDGE_HOR, 0, m_stLFCUParam.bTopEdge );//该CU块上边界设置滤波标志及边界强度

  switch ( pcCU->getPartitionSize( uiAbsZorderIdx ) )
  {//由CU块不同的PU块分割模式 对PU块边界设置滤波标志 一个CU块中的PU块边界必为中间边界
    case SIZE_2Nx2N:
    {
      break;
    }
    case SIZE_2NxN:
    {
      xSetEdgefilterMultiple( pcCU, uiAbsZorderIdx, uiDepth, EDGE_HOR, uiHHeightInBaseUnits, m_stLFCUParam.bInternalEdge );
      break;
    }
    case SIZE_Nx2N:
    {
      xSetEdgefilterMultiple( pcCU, uiAbsZorderIdx, uiDepth, EDGE_VER, uiHWidthInBaseUnits, m_stLFCUParam.bInternalEdge );
      break;
    }
    case SIZE_NxN:
    {
      xSetEdgefilterMultiple( pcCU, uiAbsZorderIdx, uiDepth, EDGE_VER, uiHWidthInBaseUnits, m_stLFCUParam.bInternalEdge );
      xSetEdgefilterMultiple( pcCU, uiAbsZorderIdx, uiDepth, EDGE_HOR, uiHHeightInBaseUnits, m_stLFCUParam.bInternalEdge );
      break;
    }
    case SIZE_2NxnU:
    {
      xSetEdgefilterMultiple( pcCU, uiAbsZorderIdx, uiDepth, EDGE_HOR, uiQHeightInBaseUnits, m_stLFCUParam.bInternalEdge );
      break;
    }
    case SIZE_2NxnD:
    {
      xSetEdgefilterMultiple( pcCU, uiAbsZorderIdx, uiDepth, EDGE_HOR, uiHeightInBaseUnits - uiQHeightInBaseUnits, m_stLFCUParam.bInternalEdge );
      break;
    }
    case SIZE_nLx2N:
    {
      xSetEdgefilterMultiple( pcCU, uiAbsZorderIdx, uiDepth, EDGE_VER, uiQWidthInBaseUnits, m_stLFCUParam.bInternalEdge );
      break;
    }
    case SIZE_nRx2N:
    {
      xSetEdgefilterMultiple( pcCU, uiAbsZorderIdx, uiDepth, EDGE_VER, uiWidthInBaseUnits - uiQWidthInBaseUnits, m_stLFCUParam.bInternalEdge );
      break;
    }
    default:
    {
      break;
    }
  }
}


Void TComLoopFilter::xSetLoopfilterParam( TComDataCU* pcCU, UInt uiAbsZorderIdx )
{
  UInt uiX           = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[ uiAbsZorderIdx ] ];//该小块在图像中的X坐标
  UInt uiY           = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[ uiAbsZorderIdx ] ];//该小块在图像中的Y坐标

  TComDataCU* pcTempCU;
  UInt        uiTempPartIdx;

  m_stLFCUParam.bInternalEdge = ! pcCU->getSlice()->getDeblockingFilterDisable();//中间边界滤波标志只由Slice中m_deblockingFilterDisable决定

  if ( (uiX == 0) || pcCU->getSlice()->getDeblockingFilterDisable() )//若CU块为图像左边界或未使能方块滤波
  {
    m_stLFCUParam.bLeftEdge = false;//则该边界不需要边界滤波
  }
  else//若不为图像左边界且开启方块滤波
  {
    m_stLFCUParam.bLeftEdge = true;
  }
  if ( m_stLFCUParam.bLeftEdge )
  {
    pcTempCU = pcCU->getPULeft( uiTempPartIdx, uiAbsZorderIdx, !pcCU->getSlice()->getLFCrossSliceBoundaryFlag(), !m_bLFCrossTileBoundary);//得到该PU块左边邻近CU块

    if ( pcTempCU != NULL )//若左侧CU块不为空（可以得到左侧CU块）
    {
      m_stLFCUParam.bLeftEdge = true;//则需要边界滤波
    }
    else//若为空（无法得到左侧CU块 为tile的边界或Slice的边界）
    {
      m_stLFCUParam.bLeftEdge = false;//则不需要边界滤波
    }
  }

  if ( (uiY == 0 ) || pcCU->getSlice()->getDeblockingFilterDisable() )
  {//同左边界 判断CU边界是否为图像边界 tile slice边界 若是则不需要边界滤波
    m_stLFCUParam.bTopEdge = false;
  }
  else
  {
    m_stLFCUParam.bTopEdge = true;
  }
  if ( m_stLFCUParam.bTopEdge )
  {
    pcTempCU = pcCU->getPUAbove( uiTempPartIdx, uiAbsZorderIdx, !pcCU->getSlice()->getLFCrossSliceBoundaryFlag(), false, !m_bLFCrossTileBoundary);

    if ( pcTempCU != NULL )
    {
      m_stLFCUParam.bTopEdge = true;
    }
    else
    {
      m_stLFCUParam.bTopEdge = false;
    }
  }
}

Void TComLoopFilter::xGetBoundaryStrengthSingle ( TComDataCU* pCtu, DeblockEdgeDir edgeDir, UInt uiAbsPartIdx4x4BlockWithinCtu )
{
  TComSlice * const pcSlice = pCtu->getSlice();

  const Bool lfCrossSliceBoundaryFlag=pCtu->getSlice()->getLFCrossSliceBoundaryFlag();

  const UInt uiPartQ = uiAbsPartIdx4x4BlockWithinCtu;
  TComDataCU* const pcCUQ = pCtu;

  UInt uiPartP;
  TComDataCU* pcCUP;
  UInt uiBs = 0;

  //-- Calculate Block Index 
  if (edgeDir == EDGE_VER)//垂直边界
  {
    pcCUP = pcCUQ->getPULeft(uiPartP, uiPartQ, !lfCrossSliceBoundaryFlag, !m_bLFCrossTileBoundary);//pcCUP为pCtu左侧PU 即uiPartP所在PU uiPartP为边界左侧4*4小块
  }
  else  // (edgeDir == EDGE_HOR)
  {
    pcCUP = pcCUQ->getPUAbove(uiPartP, uiPartQ, !pCtu->getSlice()->getLFCrossSliceBoundaryFlag(), false, !m_bLFCrossTileBoundary);//pcCUP为pCtu上方PU 即uiPartP所在PU uiPartP为边界上方 4*4小块
  }

  //-- Set BS for Intra MB : BS = 4 or 3
  if ( pcCUP->isIntra(uiPartP) || pcCUQ->isIntra(uiPartQ) )//P或Q采用帧内预测
  {
    uiBs = 2;
  }

  //-- Set BS for not Intra MB : BS = 2 or 1 or 0
  if ( !pcCUP->isIntra(uiPartP) && !pcCUQ->isIntra(uiPartQ) )//P和Q均不是帧内预测
  {
    UInt nsPartQ = uiPartQ;
    UInt nsPartP = uiPartP;

    if ( m_aapucBS[edgeDir][uiAbsPartIdx4x4BlockWithinCtu] && (pcCUQ->getCbf( nsPartQ, COMPONENT_Y, pcCUQ->getTransformIdx(nsPartQ)) != 0 || pcCUP->getCbf( nsPartP, COMPONENT_Y, pcCUP->getTransformIdx(nsPartP) ) != 0) )
    {//P或Q有非零系数
      uiBs = 1;
    }
    else
    {
      if (pcSlice->isInterB() || pcCUP->getSlice()->isInterB())//CUQ或CUP为帧间 B模式（双向预测）
      {
        Int iRefIdx;
        TComPic *piRefP0, *piRefP1, *piRefQ0, *piRefQ1;
        iRefIdx = pcCUP->getCUMvField(REF_PIC_LIST_0)->getRefIdx(uiPartP);
        piRefP0 = (iRefIdx < 0) ? NULL : pcCUP->getSlice()->getRefPic(REF_PIC_LIST_0, iRefIdx);//PartP的参考图像索引（list0中）
        iRefIdx = pcCUP->getCUMvField(REF_PIC_LIST_1)->getRefIdx(uiPartP);
        piRefP1 = (iRefIdx < 0) ? NULL : pcCUP->getSlice()->getRefPic(REF_PIC_LIST_1, iRefIdx);//PartP的参考图像索引 （list1中）
        iRefIdx = pcCUQ->getCUMvField(REF_PIC_LIST_0)->getRefIdx(uiPartQ);
        piRefQ0 = (iRefIdx < 0) ? NULL : pcSlice->getRefPic(REF_PIC_LIST_0, iRefIdx);//PartQ的参考图像索引（list0中）
        iRefIdx = pcCUQ->getCUMvField(REF_PIC_LIST_1)->getRefIdx(uiPartQ);
        piRefQ1 = (iRefIdx < 0) ? NULL : pcSlice->getRefPic(REF_PIC_LIST_1, iRefIdx);//PartQ的参考图像索引（list1中）

        TComMv pcMvP0 = pcCUP->getCUMvField(REF_PIC_LIST_0)->getMv(uiPartP);//得到运动矢量
        TComMv pcMvP1 = pcCUP->getCUMvField(REF_PIC_LIST_1)->getMv(uiPartP);
        TComMv pcMvQ0 = pcCUQ->getCUMvField(REF_PIC_LIST_0)->getMv(uiPartQ);
        TComMv pcMvQ1 = pcCUQ->getCUMvField(REF_PIC_LIST_1)->getMv(uiPartQ);

        if (piRefP0 == NULL) //若不存在则赋零
        {
          pcMvP0.setZero();
        }
        if (piRefP1 == NULL)
        {
          pcMvP1.setZero();
        }
        if (piRefQ0 == NULL)
        {
          pcMvQ0.setZero();
        }
        if (piRefQ1 == NULL)
        {
          pcMvQ1.setZero();
        }

        if ( ((piRefP0==piRefQ0)&&(piRefP1==piRefQ1)) || ((piRefP0==piRefQ1)&&(piRefP1==piRefQ0)) )//若使用相同的参考图像
        {
          if ( piRefP0 != piRefP1 )   // Different L0 & L1
          {
            if ( piRefP0 == piRefQ0 )
            {//若P和Q的运动矢量（h或v分量）的差值大于等于4 则 BS=1
              uiBs  = ((abs(pcMvQ0.getHor() - pcMvP0.getHor()) >= 4) ||
                       (abs(pcMvQ0.getVer() - pcMvP0.getVer()) >= 4) ||
                       (abs(pcMvQ1.getHor() - pcMvP1.getHor()) >= 4) ||
                       (abs(pcMvQ1.getVer() - pcMvP1.getVer()) >= 4)) ? 1 : 0;
            }
            else
            {//若P和Q的运动矢量（h或v分量）的差值大于等于4 则 BS=1
              uiBs  = ((abs(pcMvQ1.getHor() - pcMvP0.getHor()) >= 4) ||
                       (abs(pcMvQ1.getVer() - pcMvP0.getVer()) >= 4) ||
                       (abs(pcMvQ0.getHor() - pcMvP1.getHor()) >= 4) ||
                       (abs(pcMvQ0.getVer() - pcMvP1.getVer()) >= 4)) ? 1 : 0;
            }
          }
          else    // Same L0 & L1
          {
            uiBs  = ((abs(pcMvQ0.getHor() - pcMvP0.getHor()) >= 4) ||
                     (abs(pcMvQ0.getVer() - pcMvP0.getVer()) >= 4) ||
                     (abs(pcMvQ1.getHor() - pcMvP1.getHor()) >= 4) ||
                     (abs(pcMvQ1.getVer() - pcMvP1.getVer()) >= 4)) &&
                    ((abs(pcMvQ1.getHor() - pcMvP0.getHor()) >= 4) ||
                     (abs(pcMvQ1.getVer() - pcMvP0.getVer()) >= 4) ||
                     (abs(pcMvQ0.getHor() - pcMvP1.getHor()) >= 4) ||
                     (abs(pcMvQ0.getVer() - pcMvP1.getVer()) >= 4)) ? 1 : 0;
          }
        }
        else // for all different Ref_Idx
        {//若使用不同的参考图像 则BS=1
          uiBs = 1;
        }
      }
      else  // pcSlice->isInterP() slice为帧间 P预测
      {//判断同上
        Int iRefIdx;
        TComPic *piRefP0, *piRefQ0;
        iRefIdx = pcCUP->getCUMvField(REF_PIC_LIST_0)->getRefIdx(uiPartP);
        piRefP0 = (iRefIdx < 0) ? NULL : pcCUP->getSlice()->getRefPic(REF_PIC_LIST_0, iRefIdx);
        iRefIdx = pcCUQ->getCUMvField(REF_PIC_LIST_0)->getRefIdx(uiPartQ);
        piRefQ0 = (iRefIdx < 0) ? NULL : pcSlice->getRefPic(REF_PIC_LIST_0, iRefIdx);
        TComMv pcMvP0 = pcCUP->getCUMvField(REF_PIC_LIST_0)->getMv(uiPartP);
        TComMv pcMvQ0 = pcCUQ->getCUMvField(REF_PIC_LIST_0)->getMv(uiPartQ);

        if (piRefP0 == NULL)
        {
          pcMvP0.setZero();
        }
        if (piRefQ0 == NULL)
        {
          pcMvQ0.setZero();
        }

        uiBs  = ((piRefP0 != piRefQ0) ||
                 (abs(pcMvQ0.getHor() - pcMvP0.getHor()) >= 4) ||
                 (abs(pcMvQ0.getVer() - pcMvP0.getVer()) >= 4)) ? 1 : 0;
      }
    }   // enf of "if( one of BCBP == 0 )"
  }   // enf of "if( not Intra )"

  m_aapucBS[edgeDir][uiAbsPartIdx4x4BlockWithinCtu] = uiBs;
}


Void TComLoopFilter::xEdgeFilterLuma( TComDataCU* const pcCU, const UInt uiAbsZorderIdx, const UInt uiDepth, const DeblockEdgeDir edgeDir, const Int iEdge  )
{
        TComPicYuv *pcPicYuvRec                   = pcCU->getPic()->getPicYuvRec();
        Pel        *piSrc                         = pcPicYuvRec->getAddr(COMPONENT_Y, pcCU->getCtuRsAddr(), uiAbsZorderIdx );//该CU块地址（左上像素地址）
        Pel        *piTmpSrc                      = piSrc;
  const TComSPS    &sps                           = *(pcCU->getSlice()->getSPS());
  const Bool        ppsTransquantBypassEnableFlag = pcCU->getSlice()->getPPS()->getTransquantBypassEnableFlag();
  const Int         bitDepthLuma                  = sps.getBitDepth(CHANNEL_TYPE_LUMA);
  const Bool        lfCrossSliceBoundaryFlag      = pcCU->getSlice()->getLFCrossSliceBoundaryFlag();

  Int  iStride = pcPicYuvRec->getStride(COMPONENT_Y);
  Int iQP = 0;
  Int iQP_P = 0;
  Int iQP_Q = 0;
  UInt uiNumParts = pcCU->getPic()->getNumPartInCtuWidth()>>uiDepth;//当前深度CU最小块个数

  UInt  uiPelsInPart = sps.getMaxCUWidth() >> sps.getMaxTotalCUDepth();//最小块大小
  UInt  uiBsAbsIdx = 0, uiBs = 0;
  Int   iOffset, iSrcStep;

  Bool  bPCMFilter = (sps.getUsePCM() && sps.getPCMFilterDisableFlag())? true : false;//pcm条件下是否使用去方块滤波
  Bool  bPartPNoFilter = false;
  Bool  bPartQNoFilter = false;
  UInt  uiPartPIdx = 0;//p块地址索引
  UInt  uiPartQIdx = 0;//q块地址索引
  TComDataCU* pcCUP = pcCU;
  TComDataCU* pcCUQ = pcCU;
  Int  betaOffsetDiv2 = pcCUQ->getSlice()->getDeblockingFilterBetaOffsetDiv2();//计算beta时的偏移量
  Int  tcOffsetDiv2 = pcCUQ->getSlice()->getDeblockingFilterTcOffsetDiv2();//计算tc时的偏移量

  if (edgeDir == EDGE_VER)//垂直边界
  {
    iOffset = 1;//同一行一个像素偏移量 表示计算每一行像素变化率相邻一个像素的偏移量
    iSrcStep = iStride;//同一列一个像素偏移量 表示相邻一行的像素偏移量
    piTmpSrc += iEdge*uiPelsInPart;//iEdge处像素地址
  }
  else  // (edgeDir == EDGE_HOR) 水平模式则为以每一列为基础计算
  {
    iOffset = iStride;
    iSrcStep = 1;
    piTmpSrc += iEdge*uiPelsInPart*iStride;
  }

  const Int iBitdepthScale = 1 << (bitDepthLuma-8);

  for ( UInt iIdx = 0; iIdx < uiNumParts; iIdx++ )//遍历iEdge处所有边界
  {
    uiBsAbsIdx = xCalcBsIdx( pcCU, uiAbsZorderIdx, edgeDir, iEdge, iIdx);
    uiBs = m_aapucBS[edgeDir][uiBsAbsIdx];//该处边界强度
    if ( uiBs )//若BS不为0
    {
      iQP_Q = pcCU->getQP( uiBsAbsIdx );
      uiPartQIdx = uiBsAbsIdx;
      // Derive neighboring PU index
      if (edgeDir == EDGE_VER)
      {
        pcCUP = pcCUQ->getPULeft (uiPartPIdx, uiPartQIdx,!lfCrossSliceBoundaryFlag, !m_bLFCrossTileBoundary);//pcCUP为pCtu左侧PU 即uiPartP所在PU uiPartP为边界左侧4*4小块
      }
      else  // (iDir == EDGE_HOR)
      {
        pcCUP = pcCUQ->getPUAbove(uiPartPIdx, uiPartQIdx,!lfCrossSliceBoundaryFlag, false, !m_bLFCrossTileBoundary);//pcCUP为pCtu上方PU 即uiPartP所在PU uiPartP为边界上方 4*4小块
      }

      iQP_P = pcCUP->getQP(uiPartPIdx);//p块量化参数
      iQP = (iQP_P + iQP_Q + 1) >> 1;//q p块量化参数均值
	   
      Int iIndexTC = Clip3(0, MAX_QP+DEFAULT_INTRA_TC_OFFSET, Int(iQP + DEFAULT_INTRA_TC_OFFSET*(uiBs-1) + (tcOffsetDiv2 << 1)));//调整后的量化参数值 为TC索引
      Int iIndexB = Clip3(0, MAX_QP, iQP + (betaOffsetDiv2 << 1));////调整后的量化参数值 为Beta索引

      Int iTc =  sm_tcTable[iIndexTC]*iBitdepthScale;//该边界Tc值
      Int iBeta = sm_betaTable[iIndexB]*iBitdepthScale;//Beta值
      Int iSideThreshold = (iBeta+(iBeta>>1))>>3;//后续需使用到的判断阈值
      Int iThrCut = iTc*10;


      UInt  uiBlocksInPart = uiPelsInPart / 4 ? uiPelsInPart / 4 : 1;
      for (UInt iBlkIdx = 0; iBlkIdx<uiBlocksInPart; iBlkIdx ++)
      {
        Int dp0 = xCalcDP( piTmpSrc+iSrcStep*(iIdx*uiPelsInPart+iBlkIdx*4+0), iOffset);//P块首行（列）像素变化率
        Int dq0 = xCalcDQ( piTmpSrc+iSrcStep*(iIdx*uiPelsInPart+iBlkIdx*4+0), iOffset);//q块首行（列）像素变化率
        Int dp3 = xCalcDP( piTmpSrc+iSrcStep*(iIdx*uiPelsInPart+iBlkIdx*4+3), iOffset);//P块末行（列）像素变化率
        Int dq3 = xCalcDQ( piTmpSrc+iSrcStep*(iIdx*uiPelsInPart+iBlkIdx*4+3), iOffset);//q块末行（列）像素变化率
        Int d0 = dp0 + dq0;//首行（列）像素变化率
        Int d3 = dp3 + dq3;//末行（列）像素变化率

        Int dp = dp0 + dp3;//p块像素变化率
        Int dq = dq0 + dq3;//q块像素变化率
        Int d =  d0 + d3;//总像素变化率

        if (bPCMFilter || ppsTransquantBypassEnableFlag)
        {
          // Check if each of PUs is I_PCM with LF disabling
          bPartPNoFilter = (bPCMFilter && pcCUP->getIPCMFlag(uiPartPIdx));
          bPartQNoFilter = (bPCMFilter && pcCUQ->getIPCMFlag(uiPartQIdx));

          // check if each of PUs is lossless coded
          bPartPNoFilter = bPartPNoFilter || (pcCUP->isLosslessCoded(uiPartPIdx) );
          bPartQNoFilter = bPartQNoFilter || (pcCUQ->isLosslessCoded(uiPartQIdx) );
        }

        if (d < iBeta)
        {
          Bool bFilterP = (dp < iSideThreshold);//弱滤波时根据像素变化判断q(1)是否需要修正
          Bool bFilterQ = (dq < iSideThreshold);//弱滤波时根据像素变化判断p(-2)是否需要修正

          Bool sw =  xUseStrongFiltering( iOffset, 2*d0, iBeta, iTc, piTmpSrc+iSrcStep*(iIdx*uiPelsInPart+iBlkIdx*4+0))
          && xUseStrongFiltering( iOffset, 2*d3, iBeta, iTc, piTmpSrc+iSrcStep*(iIdx*uiPelsInPart+iBlkIdx*4+3));//该边界两侧像素是否使用强滤波

          for ( Int i = 0; i < DEBLOCK_SMALLEST_BLOCK/2; i++)//对每行（列）像素进行滤波
          {
            xPelFilterLuma( piTmpSrc+iSrcStep*(iIdx*uiPelsInPart+iBlkIdx*4+i), iOffset, iTc, sw, bPartPNoFilter, bPartQNoFilter, iThrCut, bFilterP, bFilterQ, bitDepthLuma);
          }
        }
      }
    }
  }
}


Void TComLoopFilter::xEdgeFilterChroma( TComDataCU* const pcCU, const UInt uiAbsZorderIdx, const UInt uiDepth, const DeblockEdgeDir edgeDir, const Int iEdge )
{//对色度分量去方块滤波 色度滤波过程相对简单且大多参数已在亮度滤波中解释过 故不再叙述
        TComPicYuv *pcPicYuvRec    = pcCU->getPic()->getPicYuvRec();
        Int         iStride        = pcPicYuvRec->getStride(COMPONENT_Cb);
        Pel        *piSrcCb        = pcPicYuvRec->getAddr( COMPONENT_Cb, pcCU->getCtuRsAddr(), uiAbsZorderIdx );
        Pel        *piSrcCr        = pcPicYuvRec->getAddr( COMPONENT_Cr, pcCU->getCtuRsAddr(), uiAbsZorderIdx );
  const TComSPS    &sps            = *(pcCU->getSlice()->getSPS());
  const Int         bitDepthChroma = sps.getBitDepth(CHANNEL_TYPE_CHROMA);

  const UInt  uiPelsInPartChromaH = sps.getMaxCUWidth() >> (sps.getMaxTotalCUDepth()+pcPicYuvRec->getComponentScaleX(COMPONENT_Cb));
  const UInt  uiPelsInPartChromaV = sps.getMaxCUHeight() >> (sps.getMaxTotalCUDepth()+pcPicYuvRec->getComponentScaleY(COMPONENT_Cb));

  Int iQP = 0;
  Int iQP_P = 0;
  Int iQP_Q = 0;

  Int   iOffset, iSrcStep;
  UInt  uiLoopLength;

  const UInt uiCtuWidthInBaseUnits = pcCU->getPic()->getNumPartInCtuWidth();

  Bool  bPCMFilter = (pcCU->getSlice()->getSPS()->getUsePCM() && pcCU->getSlice()->getSPS()->getPCMFilterDisableFlag())? true : false;
  Bool  bPartPNoFilter = false;
  Bool  bPartQNoFilter = false;
  TComDataCU* pcCUQ = pcCU;
  Int tcOffsetDiv2 = pcCU->getSlice()->getDeblockingFilterTcOffsetDiv2();

  // Vertical Position
  UInt uiEdgeNumInCtuVert = g_auiZscanToRaster[uiAbsZorderIdx]%uiCtuWidthInBaseUnits + iEdge;
  UInt uiEdgeNumInCtuHor = g_auiZscanToRaster[uiAbsZorderIdx]/uiCtuWidthInBaseUnits + iEdge;

  if ( (uiPelsInPartChromaH < DEBLOCK_SMALLEST_BLOCK) && (uiPelsInPartChromaV < DEBLOCK_SMALLEST_BLOCK) &&
       (
         ( (uiEdgeNumInCtuVert%(DEBLOCK_SMALLEST_BLOCK/uiPelsInPartChromaH)) && (edgeDir==EDGE_VER) ) ||
         ( (uiEdgeNumInCtuHor %(DEBLOCK_SMALLEST_BLOCK/uiPelsInPartChromaV)) && (edgeDir==EDGE_HOR) )
       )
     )
  {
    return;
  }


  const Bool lfCrossSliceBoundaryFlag=pcCU->getSlice()->getLFCrossSliceBoundaryFlag();

  UInt  uiNumParts = pcCU->getPic()->getNumPartInCtuWidth()>>uiDepth;

  UInt  uiBsAbsIdx;
  UChar ucBs;

  Pel* piTmpSrcCb = piSrcCb;
  Pel* piTmpSrcCr = piSrcCr;

  if (edgeDir == EDGE_VER)
  {
    iOffset   = 1;
    iSrcStep  = iStride;
    piTmpSrcCb += iEdge*uiPelsInPartChromaH;
    piTmpSrcCr += iEdge*uiPelsInPartChromaH;
    uiLoopLength=uiPelsInPartChromaV;
  }
  else  // (edgeDir == EDGE_HOR)
  {
    iOffset   = iStride;
    iSrcStep  = 1;
    piTmpSrcCb += iEdge*iStride*uiPelsInPartChromaV;
    piTmpSrcCr += iEdge*iStride*uiPelsInPartChromaV;
    uiLoopLength=uiPelsInPartChromaH;
  }

  const Int iBitdepthScale = 1 << (pcCU->getSlice()->getSPS()->getBitDepth(CHANNEL_TYPE_CHROMA)-8);

  for ( UInt iIdx = 0; iIdx < uiNumParts; iIdx++ )
  {
    uiBsAbsIdx = xCalcBsIdx( pcCU, uiAbsZorderIdx, edgeDir, iEdge, iIdx);
    ucBs = m_aapucBS[edgeDir][uiBsAbsIdx];

    if ( ucBs > 1)//只有边界强度值为2时才会对其进行滤波
    {
      iQP_Q = pcCU->getQP( uiBsAbsIdx );
      UInt  uiPartQIdx = uiBsAbsIdx;
      // Derive neighboring PU index
      TComDataCU* pcCUP;
      UInt  uiPartPIdx;

      if (edgeDir == EDGE_VER)
      {
        pcCUP = pcCUQ->getPULeft (uiPartPIdx, uiPartQIdx,!lfCrossSliceBoundaryFlag, !m_bLFCrossTileBoundary);
      }
      else  // (edgeDir == EDGE_HOR)
      {
        pcCUP = pcCUQ->getPUAbove(uiPartPIdx, uiPartQIdx,!lfCrossSliceBoundaryFlag, false, !m_bLFCrossTileBoundary);
      }

      iQP_P = pcCUP->getQP(uiPartPIdx);

      if (bPCMFilter || pcCU->getSlice()->getPPS()->getTransquantBypassEnableFlag())
      {
        // Check if each of PUs is I_PCM with LF disabling
        bPartPNoFilter = (bPCMFilter && pcCUP->getIPCMFlag(uiPartPIdx));
        bPartQNoFilter = (bPCMFilter && pcCUQ->getIPCMFlag(uiPartQIdx));

        // check if each of PUs is lossless coded
        bPartPNoFilter = bPartPNoFilter || (pcCUP->isLosslessCoded(uiPartPIdx));
        bPartQNoFilter = bPartQNoFilter || (pcCUQ->isLosslessCoded(uiPartQIdx));
      }

      for ( UInt chromaIdx = 0; chromaIdx < 2; chromaIdx++ )
      {
        Int chromaQPOffset  = pcCU->getSlice()->getPPS()->getQpOffset(ComponentID(chromaIdx + 1));
        Pel* piTmpSrcChroma = (chromaIdx == 0) ? piTmpSrcCb : piTmpSrcCr;

        iQP = ((iQP_P + iQP_Q + 1) >> 1) + chromaQPOffset;
        if (iQP >= chromaQPMappingTableSize)
        {
          if (pcPicYuvRec->getChromaFormat()==CHROMA_420)
          {
            iQP -=6;
          }
          else if (iQP>51)
          {
            iQP=51;
          }
        }
        else if (iQP >= 0 )
        {
          iQP = getScaledChromaQP(iQP, pcPicYuvRec->getChromaFormat());
        }

        Int iIndexTC = Clip3(0, MAX_QP+DEFAULT_INTRA_TC_OFFSET, iQP + DEFAULT_INTRA_TC_OFFSET*(ucBs - 1) + (tcOffsetDiv2 << 1));
        Int iTc =  sm_tcTable[iIndexTC]*iBitdepthScale;

        for ( UInt uiStep = 0; uiStep < uiLoopLength; uiStep++ )
        {
          xPelFilterChroma( piTmpSrcChroma + iSrcStep*(uiStep+iIdx*uiLoopLength), iOffset, iTc , bPartPNoFilter, bPartQNoFilter, bitDepthChroma);
        }
      }
    }
  }
}

/**
 - Deblocking for the luminance component with strong or weak filter
 .
 \param piSrc           pointer to picture data
 \param iOffset         offset value for picture data
 \param tc              tc value
 \param sw              decision strong/weak filter
 \param bPartPNoFilter  indicator to disable filtering on partP
 \param bPartQNoFilter  indicator to disable filtering on partQ
 \param iThrCut         threshold value for weak filter decision
 \param bFilterSecondP  decision weak filter/no filter for partP
 \param bFilterSecondQ  decision weak filter/no filter for partQ
 \param bitDepthLuma    luma bit depth
*/
__inline Void TComLoopFilter::xPelFilterLuma( Pel* piSrc, Int iOffset, Int tc, Bool sw, Bool bPartPNoFilter, Bool bPartQNoFilter, Int iThrCut, Bool bFilterSecondP, Bool bFilterSecondQ, const Int bitDepthLuma)
{
  Int delta;

  Pel m4  = piSrc[0];//q(0)
  Pel m3  = piSrc[-iOffset];//p(-1)
  Pel m5  = piSrc[ iOffset];//q(1)
  Pel m2  = piSrc[-iOffset*2];//p(-2)
  Pel m6  = piSrc[ iOffset*2];//q（2）
  Pel m1  = piSrc[-iOffset*3];//p(-3)
  Pel m7  = piSrc[ iOffset*3];//q(3)
  Pel m0  = piSrc[-iOffset*4];//p(-4)

  if (sw)//如果边界强滤波
  {//计算滤波后的像素值
    piSrc[-iOffset]   = Clip3(m3-2*tc, m3+2*tc, ((m1 + 2*m2 + 2*m3 + 2*m4 + m5 + 4) >> 3));//滤波系数（1，2，2，2，1）/8
    piSrc[0]          = Clip3(m4-2*tc, m4+2*tc, ((m2 + 2*m3 + 2*m4 + 2*m5 + m6 + 4) >> 3));//滤波系数（1，2，2，2，1）/8
    piSrc[-iOffset*2] = Clip3(m2-2*tc, m2+2*tc, ((m1 + m2 + m3 + m4 + 2)>>2));//滤波系数（1，1，1，1，1）/4
    piSrc[ iOffset]   = Clip3(m5-2*tc, m5+2*tc, ((m3 + m4 + m5 + m6 + 2)>>2));//滤波系数（1，1，1，1，1）/4
    piSrc[-iOffset*3] = Clip3(m1-2*tc, m1+2*tc, ((2*m0 + 3*m1 + m2 + m3 + m4 + 4 )>>3));//滤波系数（2，3，1，1，1）/8
    piSrc[ iOffset*2] = Clip3(m6-2*tc, m6+2*tc, ((m3 + m4 + m5 + 3*m6 + 2*m7 +4 )>>3));//滤波系数（2，3，1，1，1）/8
  }
  else
  {
    /* Weak filter */
    delta = (9*(m4-m3) -3*(m5-m2) + 8)>>4 ;//边界处像素变化程度

    if ( abs(delta) < iThrCut )//|Δ|<10tc
    {//计算q(-1) p(0)滤波后的像素值  滤波系数为（3,7,9,-3）/16
      delta = Clip3(-tc, tc, delta);
      piSrc[-iOffset] = ClipBD((m3+delta), bitDepthLuma);
      piSrc[0] = ClipBD((m4-delta), bitDepthLuma);

      Int tc2 = tc>>1;
      if(bFilterSecondP)//若q(-2)需要滤波
      {//计算q(-2)滤波后的像素值 滤波系数为（8，19，-1，9，3）/32
        Int delta1 = Clip3(-tc2, tc2, (( ((m1+m3+1)>>1)- m2+delta)>>1));
        piSrc[-iOffset*2] = ClipBD((m2+delta1), bitDepthLuma);
      }
      if(bFilterSecondQ)//若p（1）需要滤波
      {//计算p(-1)滤波后的像素值 滤波系数为（8，19，-1，9，3）/32
        Int delta2 = Clip3(-tc2, tc2, (( ((m6+m4+1)>>1)- m5-delta)>>1));
        piSrc[ iOffset] = ClipBD((m5+delta2), bitDepthLuma);
      }
    }
  }

  if(bPartPNoFilter)
  {//若q不需要滤波 则值不变
    piSrc[-iOffset] = m3;
    piSrc[-iOffset*2] = m2;
    piSrc[-iOffset*3] = m1;
  }
  if(bPartQNoFilter)
  {//若p不需要滤波 则值不变
    piSrc[0] = m4;
    piSrc[ iOffset] = m5;
    piSrc[ iOffset*2] = m6;
  }
}

/**
 - Deblocking of one line/column for the chrominance component
 .
 \param piSrc           pointer to picture data
 \param iOffset         offset value for picture data
 \param tc              tc value
 \param bPartPNoFilter  indicator to disable filtering on partP
 \param bPartQNoFilter  indicator to disable filtering on partQ
 \param bitDepthChroma  chroma bit depth
 */
__inline Void TComLoopFilter::xPelFilterChroma( Pel* piSrc, Int iOffset, Int tc, Bool bPartPNoFilter, Bool bPartQNoFilter, const Int bitDepthChroma)
{
  Int delta;

  Pel m4  = piSrc[0];
  Pel m3  = piSrc[-iOffset];
  Pel m5  = piSrc[ iOffset];
  Pel m2  = piSrc[-iOffset*2];

  delta = Clip3(-tc,tc, (((( m4 - m3 ) << 2 ) + m2 - m5 + 4 ) >> 3) );
  piSrc[-iOffset] = ClipBD((m3+delta), bitDepthChroma);
  piSrc[0] = ClipBD((m4-delta), bitDepthChroma);

  if(bPartPNoFilter)
  {
    piSrc[-iOffset] = m3;
  }
  if(bPartQNoFilter)
  {
    piSrc[0] = m4;
  }
}

/**
 - Decision between strong and weak filter
 .
 \param offset         offset value for picture data
 \param d               d value
 \param beta            beta value
 \param tc              tc value
 \param piSrc           pointer to picture data
 */
__inline Bool TComLoopFilter::xUseStrongFiltering( Int offset, Int d, Int beta, Int tc, Pel* piSrc)
{//判断是否该行是否满足强滤波条件
  Pel m4  = piSrc[0];//q(0)
  Pel m3  = piSrc[-offset];//p(-1)
  Pel m7  = piSrc[ offset*3];//q(3)
  Pel m0  = piSrc[-offset*4];//p(-4)

  Int d_strong = abs(m0-m3) + abs(m7-m4);//该行像素边界两侧不平坦度

  return ( (d_strong < (beta>>3)) && (d<(beta>>2)) && ( abs(m3-m4) < ((tc*5+1)>>1)) );//强滤波条件判断
}

__inline Int TComLoopFilter::xCalcDP( Pel* piSrc, Int iOffset)//计算dp值 p块像素的变化率
{//piSrc为q块首地址
  return abs( piSrc[-iOffset*3] - 2*piSrc[-iOffset*2] + piSrc[-iOffset] ) ;//|p(-3)-2p(-2)+p(-1)|
}

__inline Int TComLoopFilter::xCalcDQ( Pel* piSrc, Int iOffset)//计算dq值 q块像素的变化率
{//piSrc为q块首地
  return abs( piSrc[0] - 2*piSrc[iOffset] + piSrc[iOffset*2] );//|q(0)-2p(1)+p(2)|
}
//! \}
