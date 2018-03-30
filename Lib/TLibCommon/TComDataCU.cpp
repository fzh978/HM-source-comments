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

/** \file     TComDataCU.cpp
    \brief    CU data structure
    \todo     not all entities are documented
*/
//TComDataCU包含了CU所有的语法元素　存储了CU中所有4*4小块的信息　TComDataCU既能表示CTU　也能表示由CTU划分的CU
#include "TComDataCU.h"
#include "TComTU.h"
#include "TComPic.h"

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TComDataCU::TComDataCU()//构造函数
{
  m_pcPic              = NULL;
  m_pcSlice            = NULL;
  m_puhDepth           = NULL;

  m_skipFlag           = NULL;

  m_pePartSize         = NULL;
  m_pePredMode         = NULL;
  m_CUTransquantBypass = NULL;
  m_puhWidth           = NULL;
  m_puhHeight          = NULL;
  m_phQP               = NULL;
  m_ChromaQpAdj        = NULL;
  m_pbMergeFlag        = NULL;
  m_puhMergeIndex      = NULL;
  for(UInt i=0; i<MAX_NUM_CHANNEL_TYPE; i++)
  {
    m_puhIntraDir[i]     = NULL;
  }
  m_puhInterDir        = NULL;
  m_puhTrIdx           = NULL;

  for (UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
  {
    m_puhCbf[comp]                        = NULL;
    m_crossComponentPredictionAlpha[comp] = NULL;
    m_puhTransformSkip[comp]              = NULL;
    m_pcTrCoeff[comp]                     = NULL;
#if ADAPTIVE_QP_SELECTION
    m_pcArlCoeff[comp]                    = NULL;
#endif
    m_pcIPCMSample[comp]                  = NULL;
    m_explicitRdpcmMode[comp]             = NULL;
  }
#if ADAPTIVE_QP_SELECTION
  m_ArlCoeffIsAliasedAllocation = false;
#endif
  m_pbIPCMFlag         = NULL;

  m_pCtuAboveLeft      = NULL;
  m_pCtuAboveRight     = NULL;
  m_pCtuAbove          = NULL;
  m_pCtuLeft           = NULL;

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_apcCUColocated[i]  = NULL;
    m_apiMVPIdx[i]       = NULL;
    m_apiMVPNum[i]       = NULL;
  }

  m_bDecSubCu          = false;
}

TComDataCU::~TComDataCU()//析构函数
{
}

Void TComDataCU::create( ChromaFormat chromaFormatIDC, UInt uiNumPartition, UInt uiWidth, UInt uiHeight, Bool bDecSubCu, Int unitSize//声明需要用到的变量　分配资源
#if ADAPTIVE_QP_SELECTION
                        , TCoeff *pParentARLBuffer
#endif
                        )
{
  m_bDecSubCu = bDecSubCu;

  m_pcPic              = NULL;
  m_pcSlice            = NULL;
  m_uiNumPartition     = uiNumPartition;
  m_unitSize = unitSize;

  if ( !bDecSubCu )
  {
    m_phQP               = (Char*     )xMalloc(Char,     uiNumPartition);
    m_puhDepth           = (UChar*    )xMalloc(UChar,    uiNumPartition);
    m_puhWidth           = (UChar*    )xMalloc(UChar,    uiNumPartition);
    m_puhHeight          = (UChar*    )xMalloc(UChar,    uiNumPartition);

    m_ChromaQpAdj        = new UChar[ uiNumPartition ];
    m_skipFlag           = new Bool[ uiNumPartition ];
    m_pePartSize         = new Char[ uiNumPartition ];
    memset( m_pePartSize, NUMBER_OF_PART_SIZES,uiNumPartition * sizeof( *m_pePartSize ) );
    m_pePredMode         = new Char[ uiNumPartition ];
    m_CUTransquantBypass = new Bool[ uiNumPartition ];

    m_pbMergeFlag        = (Bool*  )xMalloc(Bool,   uiNumPartition);
    m_puhMergeIndex      = (UChar* )xMalloc(UChar,  uiNumPartition);

    for (UInt ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
    {
      m_puhIntraDir[ch] = (UChar* )xMalloc(UChar,  uiNumPartition);
    }
    m_puhInterDir        = (UChar* )xMalloc(UChar,  uiNumPartition);

    m_puhTrIdx           = (UChar* )xMalloc(UChar,  uiNumPartition);

    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      const RefPicList rpl=RefPicList(i);
      m_apiMVPIdx[rpl]       = new Char[ uiNumPartition ];
      m_apiMVPNum[rpl]       = new Char[ uiNumPartition ];
      memset( m_apiMVPIdx[rpl], -1,uiNumPartition * sizeof( Char ) );
    }

    for (UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
    {
      const ComponentID compID = ComponentID(comp);
      const UInt chromaShift = getComponentScaleX(compID, chromaFormatIDC) + getComponentScaleY(compID, chromaFormatIDC);
      const UInt totalSize   = (uiWidth * uiHeight) >> chromaShift;

      m_crossComponentPredictionAlpha[compID] = (Char*  )xMalloc(Char,   uiNumPartition);
      m_puhTransformSkip[compID]              = (UChar* )xMalloc(UChar,  uiNumPartition);
      m_explicitRdpcmMode[compID]             = (UChar* )xMalloc(UChar,  uiNumPartition);
      m_puhCbf[compID]                        = (UChar* )xMalloc(UChar,  uiNumPartition);
      m_pcTrCoeff[compID]                     = (TCoeff*)xMalloc(TCoeff, totalSize);
      memset( m_pcTrCoeff[compID], 0, (totalSize * sizeof( TCoeff )) );

#if ADAPTIVE_QP_SELECTION
      if( pParentARLBuffer != 0 )
      {
        m_pcArlCoeff[compID] = pParentARLBuffer;
        m_ArlCoeffIsAliasedAllocation = true;
        pParentARLBuffer += totalSize;
      }
      else
      {
        m_pcArlCoeff[compID] = (TCoeff*)xMalloc(TCoeff, totalSize);
        m_ArlCoeffIsAliasedAllocation = false;
      }
#endif
      m_pcIPCMSample[compID] = (Pel*   )xMalloc(Pel , totalSize);
    }

    m_pbIPCMFlag         = (Bool*  )xMalloc(Bool, uiNumPartition);

    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      m_acCUMvField[i].create( uiNumPartition );
    }

  }
  else
  {
    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      m_acCUMvField[i].setNumPartition(uiNumPartition );
    }
  }

  // create motion vector fields

  m_pCtuAboveLeft      = NULL;
  m_pCtuAboveRight     = NULL;
  m_pCtuAbove          = NULL;
  m_pCtuLeft           = NULL;

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_apcCUColocated[i]  = NULL;
  }
}

Void TComDataCU::destroy()//销毁变量　释放资源
{
  // encoder-side buffer free
  if ( !m_bDecSubCu )
  {
    if ( m_phQP )
    {
      xFree(m_phQP);
      m_phQP = NULL;
    }
    if ( m_puhDepth )
    {
      xFree(m_puhDepth);
      m_puhDepth = NULL;
    }
    if ( m_puhWidth )
    {
      xFree(m_puhWidth);
      m_puhWidth = NULL;
    }
    if ( m_puhHeight )
    {
      xFree(m_puhHeight);
      m_puhHeight = NULL;
    }

    if ( m_skipFlag )
    {
      delete[] m_skipFlag;
      m_skipFlag = NULL;
    }

    if ( m_pePartSize )
    {
      delete[] m_pePartSize;
      m_pePartSize = NULL;
    }
    if ( m_pePredMode )
    {
      delete[] m_pePredMode;
      m_pePredMode = NULL;
    }
    if ( m_ChromaQpAdj )
    {
      delete[] m_ChromaQpAdj;
      m_ChromaQpAdj = NULL;
    }
    if ( m_CUTransquantBypass )
    {
      delete[] m_CUTransquantBypass;
      m_CUTransquantBypass = NULL;
    }
    if ( m_puhInterDir )
    {
      xFree(m_puhInterDir);
      m_puhInterDir = NULL;
    }
    if ( m_pbMergeFlag )
    {
      xFree(m_pbMergeFlag);
      m_pbMergeFlag = NULL;
    }
    if ( m_puhMergeIndex )
    {
      xFree(m_puhMergeIndex);
      m_puhMergeIndex  = NULL;
    }

    for (UInt ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
    {
      xFree(m_puhIntraDir[ch]);
      m_puhIntraDir[ch] = NULL;
    }

    if ( m_puhTrIdx )
    {
      xFree(m_puhTrIdx);
      m_puhTrIdx = NULL;
    }

    for (UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
    {
      if ( m_crossComponentPredictionAlpha[comp] )
      {
        xFree(m_crossComponentPredictionAlpha[comp]);
        m_crossComponentPredictionAlpha[comp] = NULL;
      }
      if ( m_puhTransformSkip[comp] )
      {
        xFree(m_puhTransformSkip[comp]);
        m_puhTransformSkip[comp] = NULL;
      }
      if ( m_puhCbf[comp] )
      {
        xFree(m_puhCbf[comp]);
        m_puhCbf[comp] = NULL;
      }
      if ( m_pcTrCoeff[comp] )
      {
        xFree(m_pcTrCoeff[comp]);
        m_pcTrCoeff[comp] = NULL;
      }
      if ( m_explicitRdpcmMode[comp] )
      {
        xFree(m_explicitRdpcmMode[comp]);
        m_explicitRdpcmMode[comp] = NULL;
      }

#if ADAPTIVE_QP_SELECTION
      if (!m_ArlCoeffIsAliasedAllocation)
      {
        if ( m_pcArlCoeff[comp] )
        {
          xFree(m_pcArlCoeff[comp]);
          m_pcArlCoeff[comp] = NULL;
        }
      }
#endif

      if ( m_pcIPCMSample[comp] )
      {
        xFree(m_pcIPCMSample[comp]);
        m_pcIPCMSample[comp] = NULL;
      }
    }
    if ( m_pbIPCMFlag )
    {
      xFree(m_pbIPCMFlag );
      m_pbIPCMFlag = NULL;
    }

    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      const RefPicList rpl=RefPicList(i);
      if ( m_apiMVPIdx[rpl] )
      {
        delete[] m_apiMVPIdx[rpl];
        m_apiMVPIdx[rpl] = NULL;
      }
      if ( m_apiMVPNum[rpl] )
      {
        delete[] m_apiMVPNum[rpl];
        m_apiMVPNum[rpl] = NULL;
      }
    }

    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      const RefPicList rpl=RefPicList(i);
      m_acCUMvField[rpl].destroy();
    }
  }

  m_pcPic              = NULL;
  m_pcSlice            = NULL;

  m_pCtuAboveLeft      = NULL;
  m_pCtuAboveRight     = NULL;
  m_pCtuAbove          = NULL;
  m_pCtuLeft           = NULL;


  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_apcCUColocated[i]  = NULL;
  }

}

Bool TComDataCU::CUIsFromSameTile            ( const TComDataCU *pCU /* Can be NULL */) const
{//判断当前CU和参数中给定的CU是否来自同一个Tile 
  return pCU!=NULL &&
         pCU->getSlice() != NULL &&
         m_pcPic->getPicSym()->getTileIdxMap( pCU->getCtuRsAddr() ) == m_pcPic->getPicSym()->getTileIdxMap(getCtuRsAddr());
}

Bool TComDataCU::CUIsFromSameSliceAndTile    ( const TComDataCU *pCU /* Can be NULL */) const
{//判断当前CU和参数中给定的CU是否来自同一个Tile和Slice
  return pCU!=NULL &&
         pCU->getSlice() != NULL &&
         pCU->getSlice()->getSliceCurStartCtuTsAddr() == getSlice()->getSliceCurStartCtuTsAddr() &&//是否来自同一个Slice(Slice起始CTU的位置相同则说明为同一Slice)
         m_pcPic->getPicSym()->getTileIdxMap( pCU->getCtuRsAddr() ) == m_pcPic->getPicSym()->getTileIdxMap(getCtuRsAddr())//是否来自同一个Tile
         ;
}

Bool TComDataCU::CUIsFromSameSliceTileAndWavefrontRow( const TComDataCU *pCU /* Can be NULL */) const
{//判断当前CU和参数中给定的CU是否来自同一个Tile和Slice且能否波前并行处理
  return CUIsFromSameSliceAndTile(pCU)
         && (!getSlice()->getPPS()->getEntropyCodingSyncEnabledFlag() || getPic()->getCtu(getCtuRsAddr())->getCUPelY() == getPic()->getCtu(pCU->getCtuRsAddr())->getCUPelY());
}

Bool TComDataCU::isLastSubCUOfCtu(const UInt absPartIdx)//absPartIdx为4*4的小块在其CTU中z order的位置
{//判断absPartIdx位置的CU是否为所属CTU的最后一个CU
  const TComSPS &sps=*(getSlice()->getSPS());

  const UInt picWidth = sps.getPicWidthInLumaSamples();
  const UInt picHeight = sps.getPicHeightInLumaSamples();
  const UInt granularityWidth = sps.getMaxCUWidth();

  const UInt cuPosX = getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[absPartIdx] ];//该CU（左上像素）的X Y坐标
  const UInt cuPosY = getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[absPartIdx] ];

  return (((cuPosX+getWidth( absPartIdx))%granularityWidth==0||(cuPosX+getWidth( absPartIdx)==picWidth ))
       && ((cuPosY+getHeight(absPartIdx))%granularityWidth==0||(cuPosY+getHeight(absPartIdx)==picHeight)));//该CU的右、下边界是否为CTU或图像的右、下边界
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// --------------------------------------------------------------------------------------------------------------------
// Initialization
// --------------------------------------------------------------------------------------------------------------------

/**
 Initialize top-level CU: create internal buffers and set initial values before encoding the CTU.
 
 \param  pcPic       picture (TComPic) class pointer
 \param  ctuRsAddr   CTU address in raster scan order
 */
Void TComDataCU::initCtu( TComPic* pcPic, UInt ctuRsAddr )//初始化CTU信息 
{

  const UInt maxCUWidth = pcPic->getPicSym()->getSPS().getMaxCUWidth();//最大CU宽度（默认下为64）
  const UInt maxCUHeight= pcPic->getPicSym()->getSPS().getMaxCUHeight();//最大CU高度（默认下为64）
  m_pcPic              = pcPic;
  m_pcSlice            = pcPic->getSlice(pcPic->getCurrSliceIdx());//该CTU所属的slice
  m_ctuRsAddr          = ctuRsAddr;//CTU在一帧图像中的位置索引　raster　scan
  m_uiCUPelX           = ( ctuRsAddr % pcPic->getFrameWidthInCtus() ) * maxCUWidth;//该CTU（左上角像素）在图像中的坐标位置
  m_uiCUPelY           = ( ctuRsAddr / pcPic->getFrameWidthInCtus() ) * maxCUHeight;
  m_absZIdxInCtu       = 0;//该CU起始4*4小块在该CU属于的CTU中的位置(z order)
  m_dTotalCost         = MAX_DOUBLE;//总的代价
  m_uiTotalDistortion  = 0;//总的失真
  m_uiTotalBits        = 0;//编码后总的比特数
  m_uiTotalBins        = 0;//编码前二元化后的总的二进制数
  m_uiNumPartition     = pcPic->getNumPartitionsInCtu();//CTU中4*4小块的个数（默认为256）
  //用数组给CTU中每个4*4小块附上初始化信息
  memset( m_skipFlag          , false,                      m_uiNumPartition * sizeof( *m_skipFlag ) );//跳过标志

  memset( m_pePartSize        , NUMBER_OF_PART_SIZES,       m_uiNumPartition * sizeof( *m_pePartSize ) );//帧间预测PU块划分模式
  memset( m_pePredMode        , NUMBER_OF_PREDICTION_MODES, m_uiNumPartition * sizeof( *m_pePredMode ) );//预测模式（帧间/帧内）
  memset( m_CUTransquantBypass, false,                      m_uiNumPartition * sizeof( *m_CUTransquantBypass) );//是否为lossless模式(跳过变换和量化)
  memset( m_puhDepth          , 0,                          m_uiNumPartition * sizeof( *m_puhDepth ) );//当前CU块深度
  memset( m_puhTrIdx          , 0,                          m_uiNumPartition * sizeof( *m_puhTrIdx ) );//一个CU中变换块的索引
  memset( m_puhWidth          , maxCUWidth,                 m_uiNumPartition * sizeof( *m_puhWidth ) );//CTU的宽度为最大CU宽度
  memset( m_puhHeight         , maxCUHeight,                m_uiNumPartition * sizeof( *m_puhHeight ) );//CTU的高度为最大CU高度
  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)//初始化每个参考图像列表中运动矢量预测的索引（在候选列表中的位置）和可能的个数
  {
    const RefPicList rpl=RefPicList(i);
    memset( m_apiMVPIdx[rpl]  , -1,                         m_uiNumPartition * sizeof( *m_apiMVPIdx[rpl] ) );
    memset( m_apiMVPNum[rpl]  , -1,                         m_uiNumPartition * sizeof( *m_apiMVPNum[rpl] ) );
  }
  memset( m_phQP              , getSlice()->getSliceQp(),   m_uiNumPartition * sizeof( *m_phQP ) );//该CTU的QP值
  memset( m_ChromaQpAdj       , 0,                          m_uiNumPartition * sizeof( *m_ChromaQpAdj ) );//色度分量QP相对亮度分量的调整值
  for(UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)//为每种组成类型初始化
  {
    memset( m_crossComponentPredictionAlpha[comp] , 0,                     m_uiNumPartition * sizeof( *m_crossComponentPredictionAlpha[comp] ) );//CCP中的alpha值
    memset( m_puhTransformSkip[comp]              , 0,                     m_uiNumPartition * sizeof( *m_puhTransformSkip[comp]) );//TransformSkip标志
    memset( m_puhCbf[comp]                        , 0,                     m_uiNumPartition * sizeof( *m_puhCbf[comp] ) );//cbf标志（变换块是否存在非零系数）
    memset( m_explicitRdpcmMode[comp]             , NUMBER_OF_RDPCM_MODES, m_uiNumPartition * sizeof( *m_explicitRdpcmMode[comp] ) );//RDPCM模式
  }
  memset( m_pbMergeFlag       , false,                    m_uiNumPartition * sizeof( *m_pbMergeFlag ) );//merge标志
  memset( m_puhMergeIndex     , 0,                        m_uiNumPartition * sizeof( *m_puhMergeIndex ) );//merge索引（候选列表中最优运动矢量在列表中的位置）
  for (UInt ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)//为每个通道初始化
  {
    memset( m_puhIntraDir[ch] , ((ch==0) ? DC_IDX : 0),   m_uiNumPartition * sizeof( *(m_puhIntraDir[ch]) ) );//帧内预测模式
  }
  memset( m_puhInterDir       , 0,                        m_uiNumPartition * sizeof( *m_puhInterDir ) );//帧间预测方向
  memset( m_pbIPCMFlag        , false,                    m_uiNumPartition * sizeof( *m_pbIPCMFlag ) );//PCM模式标志

  const UInt numCoeffY    = maxCUWidth*maxCUHeight;//CTU中系数个数
  for (UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)//为每种组成初始化量化值
  {
    const UInt componentShift = m_pcPic->getComponentScaleX(ComponentID(comp)) + m_pcPic->getComponentScaleY(ComponentID(comp));
    memset( m_pcTrCoeff[comp], 0, sizeof(TCoeff)* numCoeffY>>componentShift );
#if ADAPTIVE_QP_SELECTION
    memset( m_pcArlCoeff[comp], 0, sizeof(TCoeff)* numCoeffY>>componentShift );
#endif
  }

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)//初始化每个帧间预测参考图像列表的运动矢量信息
  {
    m_acCUMvField[i].clearMvField();//初始化该Cu中的MV信息为0 参考图像无效
  }

  // Setting neighbor CU
  m_pCtuLeft        = NULL;//该CTU左侧CTU
  m_pCtuAbove       = NULL;//该CTU上方CTU
  m_pCtuAboveLeft   = NULL;//该CTU左上CTU
  m_pCtuAboveRight  = NULL;//该CTU右上CTU


  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)//声明该CTU在REF_PIC_LIST_0和REF_PIC_LIST_１中同位的CTU
  {
    m_apcCUColocated[i]  = NULL;
  }

  UInt frameWidthInCtus = pcPic->getFrameWidthInCtus();//一帧图像宽度(以CTU的宽度为单位)
  if ( m_ctuRsAddr % frameWidthInCtus )
  {
    m_pCtuLeft = pcPic->getCtu( m_ctuRsAddr - 1 );//该CTU左侧CTU
  }

  if ( m_ctuRsAddr / frameWidthInCtus )
  {
    m_pCtuAbove = pcPic->getCtu( m_ctuRsAddr - frameWidthInCtus );//该CTU上方CTU
  }

  if ( m_pCtuLeft && m_pCtuAbove )
  {
    m_pCtuAboveLeft = pcPic->getCtu( m_ctuRsAddr - frameWidthInCtus - 1 );//该CTU左上CTU
  }

  if ( m_pCtuAbove && ( (m_ctuRsAddr%frameWidthInCtus) < (frameWidthInCtus-1) )  )
  {
    m_pCtuAboveRight = pcPic->getCtu( m_ctuRsAddr - frameWidthInCtus + 1 );//该CTU右上CTU
  }

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)//得到该CTU在REF_PIC_LIST_0和REF_PIC_LIST_１中同位的CTU
  {
    const RefPicList rpl=RefPicList(i);
    if ( getSlice()->getNumRefIdx( rpl ) > 0 )//参考图像有效
    {
      m_apcCUColocated[rpl] = getSlice()->getRefPic( rpl, 0)->getCtu( m_ctuRsAddr );//同位CTU为 在参考图像列表中第一帧图像中和该CTU相同位置的CTU
    }
  }
}


/** Initialize prediction data with enabling sub-CTU-level delta QP.
*   - set CU width and CU height according to depth
*   - set qp value according to input qp
*   - set last-coded qp value according to input last-coded qp
*
* \param  uiDepth            depth of the current CU
* \param  qp                 qp for the current CU
* \param  bTransquantBypass  true for transquant bypass
*/
Void TComDataCU::initEstData( const UInt uiDepth, const Int qp, const Bool bTransquantBypass )
{//用给定的uiDepth　qp　bTransquantBypass初始化该CU的部分信息　(变量含义已在上叙述过)
  m_dTotalCost         = MAX_DOUBLE;
  m_uiTotalDistortion  = 0;
  m_uiTotalBits        = 0;
  m_uiTotalBins        = 0;

  const UChar uhWidth  = getSlice()->getSPS()->getMaxCUWidth()  >> uiDepth;
  const UChar uhHeight = getSlice()->getSPS()->getMaxCUHeight() >> uiDepth;

  for (UInt ui = 0; ui < m_uiNumPartition; ui++)
  {
    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      const RefPicList rpl=RefPicList(i);
      m_apiMVPIdx[rpl][ui]  = -1;
      m_apiMVPNum[rpl][ui]  = -1;
    }
    m_puhDepth  [ui]    = uiDepth;
    m_puhWidth  [ui]    = uhWidth;
    m_puhHeight [ui]    = uhHeight;
    m_puhTrIdx  [ui]    = 0;
    for(UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
    {
      m_crossComponentPredictionAlpha[comp][ui] = 0;
      m_puhTransformSkip             [comp][ui] = 0;
      m_explicitRdpcmMode            [comp][ui] = NUMBER_OF_RDPCM_MODES;
    }
    m_skipFlag[ui]      = false;
    m_pePartSize[ui]    = NUMBER_OF_PART_SIZES;
    m_pePredMode[ui]    = NUMBER_OF_PREDICTION_MODES;
    m_CUTransquantBypass[ui] = bTransquantBypass;
    m_pbIPCMFlag[ui]    = 0;
    m_phQP[ui]          = qp;
    m_ChromaQpAdj[ui]   = 0;
    m_pbMergeFlag[ui]   = 0;
    m_puhMergeIndex[ui] = 0;

    for (UInt ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
    {
      m_puhIntraDir[ch][ui] = ((ch==0) ? DC_IDX : 0);
    }

    m_puhInterDir[ui] = 0;
    for (UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
    {
      m_puhCbf[comp][ui] = 0;
    }
  }

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_acCUMvField[i].clearMvField();
  }

  const UInt numCoeffY = uhWidth*uhHeight;

  for (UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
  {
    const ComponentID component = ComponentID(comp);
    const UInt numCoeff = numCoeffY >> (getPic()->getComponentScaleX(component) + getPic()->getComponentScaleY(component));
    memset( m_pcTrCoeff[comp],    0, numCoeff * sizeof( TCoeff ) );
#if ADAPTIVE_QP_SELECTION
    memset( m_pcArlCoeff[comp],   0, numCoeff * sizeof( TCoeff ) );
#endif
    memset( m_pcIPCMSample[comp], 0, numCoeff * sizeof( Pel) );
  }
}


// initialize Sub partition
Void TComDataCU::initSubCU( TComDataCU* pcCU, UInt uiPartUnitIdx, UInt uiDepth, Int qp )//用给定的CU及Depth qp初始化其子CU
{
  assert( uiPartUnitIdx<4 );//子CU索引一定小于４(一个CU十字划分成等大的四个子CU)

  UInt uiPartOffset = ( pcCU->getTotalNumPart()>>2 )*uiPartUnitIdx;//uiPartUnitIdx位置的子CU较CU的位置的偏移量(以4*4小块为单位)

  m_pcPic              = pcCU->getPic();
  m_pcSlice            = pcCU->getSlice();
  m_ctuRsAddr          = pcCU->getCtuRsAddr();//该子CU所在CTU的位置
  m_absZIdxInCtu       = pcCU->getZorderIdxInCtu() + uiPartOffset;//该子CU在CTU中的位置（z order 4*4小块）

  const UChar uhWidth  = getSlice()->getSPS()->getMaxCUWidth()  >> uiDepth;//该子CU的宽
  const UChar uhHeight = getSlice()->getSPS()->getMaxCUHeight() >> uiDepth;//该子CU的高
　////该CTU（左上角像素）在图像中的坐标位置
  m_uiCUPelX           = pcCU->getCUPelX() + ( uhWidth )*( uiPartUnitIdx &  1 ); //uiPartUnitIdx &  1为 uiPartUnitIdx%2 偏移的行（以一个子CU为单位）
  m_uiCUPelY           = pcCU->getCUPelY() + ( uhHeight)*( uiPartUnitIdx >> 1 );//uiPartUnitIdx >> 1为 uiPartUnitIdx/2　偏移的列（以一个子CU为单位）

  m_dTotalCost         = MAX_DOUBLE;
  m_uiTotalDistortion  = 0;
  m_uiTotalBits        = 0;
  m_uiTotalBins        = 0;
  m_uiNumPartition     = pcCU->getTotalNumPart() >> 2;//该子CU 4*4小块的个数

  Int iSizeInUchar = sizeof( UChar  ) * m_uiNumPartition;
  Int iSizeInBool  = sizeof( Bool   ) * m_uiNumPartition;
  Int sizeInChar = sizeof( Char  ) * m_uiNumPartition;
  //初始化部分基本同initCtu　不在叙述
  memset( m_phQP,              qp,  sizeInChar );
  memset( m_pbMergeFlag,        0, iSizeInBool  );
  memset( m_puhMergeIndex,      0, iSizeInUchar );
  for (UInt ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
  {
    memset( m_puhIntraDir[ch],  ((ch==0) ? DC_IDX : 0), iSizeInUchar );
  }

  memset( m_puhInterDir,        0, iSizeInUchar );
  memset( m_puhTrIdx,           0, iSizeInUchar );

  for(UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
  {
    memset( m_crossComponentPredictionAlpha[comp], 0, iSizeInUchar );
    memset( m_puhTransformSkip[comp],              0, iSizeInUchar );
    memset( m_puhCbf[comp],                        0, iSizeInUchar );
    memset( m_explicitRdpcmMode[comp],             NUMBER_OF_RDPCM_MODES, iSizeInUchar );
  }

  memset( m_puhDepth,     uiDepth, iSizeInUchar );
  memset( m_puhWidth,          uhWidth,  iSizeInUchar );
  memset( m_puhHeight,         uhHeight, iSizeInUchar );
  memset( m_pbIPCMFlag,        0, iSizeInBool  );
  for (UInt ui = 0; ui < m_uiNumPartition; ui++)
  {
    m_skipFlag[ui]   = false;
    m_pePartSize[ui] = NUMBER_OF_PART_SIZES;
    m_pePredMode[ui] = NUMBER_OF_PREDICTION_MODES;
    m_CUTransquantBypass[ui] = false;
    m_ChromaQpAdj[ui] = 0;

    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      const RefPicList rpl=RefPicList(i);
      m_apiMVPIdx[rpl][ui] = -1;
      m_apiMVPNum[rpl][ui] = -1;
    }
  }

  const UInt numCoeffY    = uhWidth*uhHeight;
  for (UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    const UInt componentShift = m_pcPic->getComponentScaleX(ComponentID(ch)) + m_pcPic->getComponentScaleY(ComponentID(ch));
    memset( m_pcTrCoeff[ch],  0, sizeof(TCoeff)*(numCoeffY>>componentShift) );
#if ADAPTIVE_QP_SELECTION
    memset( m_pcArlCoeff[ch], 0, sizeof(TCoeff)*(numCoeffY>>componentShift) );
#endif
    memset( m_pcIPCMSample[ch], 0, sizeof(Pel)* (numCoeffY>>componentShift) );
  }

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_acCUMvField[i].clearMvField();
  }

  m_pCtuLeft        = pcCU->getCtuLeft();
  m_pCtuAbove       = pcCU->getCtuAbove();
  m_pCtuAboveLeft   = pcCU->getCtuAboveLeft();
  m_pCtuAboveRight  = pcCU->getCtuAboveRight();

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_apcCUColocated[i] = pcCU->getCUColocated(RefPicList(i));
  }
}

Void TComDataCU::setOutsideCUPart( UInt uiAbsPartIdx, UInt uiDepth )//设置位置为uiAbsPartIdx 深度为uiDepth的CU的Depth　Width　Height信息
{
  const UInt     uiNumPartition = m_uiNumPartition >> (uiDepth << 1);
  const UInt     uiSizeInUchar  = sizeof( UChar  ) * uiNumPartition;
  const TComSPS &sps            = *(getSlice()->getSPS());
  const UChar    uhWidth        = sps.getMaxCUWidth()  >> uiDepth;
  const UChar    uhHeight       = sps.getMaxCUHeight() >> uiDepth;
  memset( m_puhDepth    + uiAbsPartIdx,     uiDepth,  uiSizeInUchar );
  memset( m_puhWidth    + uiAbsPartIdx,     uhWidth,  uiSizeInUchar );
  memset( m_puhHeight   + uiAbsPartIdx,     uhHeight, uiSizeInUchar );
}

// --------------------------------------------------------------------------------------------------------------------
// Copy
// --------------------------------------------------------------------------------------------------------------------

Void TComDataCU::copySubCU( TComDataCU* pcCU, UInt uiAbsPartIdx )//复制pcCU中位置为uiAbsPartIdx的子CU的信息到当前CU(uiAbsPartIdx为CTU　z order　4*4小块的索引)
{
  UInt uiPart = uiAbsPartIdx;

  m_pcPic              = pcCU->getPic();
  m_pcSlice            = pcCU->getSlice();
  m_ctuRsAddr          = pcCU->getCtuRsAddr();
  m_absZIdxInCtu       = uiAbsPartIdx;

  m_uiCUPelX           = pcCU->getCUPelX() + g_auiRasterToPelX[ g_auiZscanToRaster[uiAbsPartIdx] ];//位置uiAbsPartIdx的子CU的在图像中的X Y坐标
  m_uiCUPelY           = pcCU->getCUPelY() + g_auiRasterToPelY[ g_auiZscanToRaster[uiAbsPartIdx] ];
  //当前CU的信息数组的起始位置为pcCU的信息数组中的uiAbsPartIdx　以此来完成信息的复制
  m_skipFlag=pcCU->getSkipFlag()          + uiPart;

  m_phQP=pcCU->getQP()                    + uiPart;
  m_ChromaQpAdj = pcCU->getChromaQpAdj()  + uiPart;
  m_pePartSize = pcCU->getPartitionSize() + uiPart;
  m_pePredMode=pcCU->getPredictionMode()  + uiPart;
  m_CUTransquantBypass  = pcCU->getCUTransquantBypass()+uiPart;

  m_pbMergeFlag         = pcCU->getMergeFlag()        + uiPart;
  m_puhMergeIndex       = pcCU->getMergeIndex()       + uiPart;

  for (UInt ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
  {
    m_puhIntraDir[ch]   = pcCU->getIntraDir(ChannelType(ch)) + uiPart;
  }

  m_puhInterDir         = pcCU->getInterDir()         + uiPart;
  m_puhTrIdx            = pcCU->getTransformIdx()     + uiPart;

  for(UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
  {
    m_crossComponentPredictionAlpha[comp] = pcCU->getCrossComponentPredictionAlpha(ComponentID(comp)) + uiPart;
    m_puhTransformSkip[comp]              = pcCU->getTransformSkip(ComponentID(comp))                 + uiPart;
    m_puhCbf[comp]                        = pcCU->getCbf(ComponentID(comp))                           + uiPart;
    m_explicitRdpcmMode[comp]             = pcCU->getExplicitRdpcmMode(ComponentID(comp))             + uiPart;
  }

  m_puhDepth=pcCU->getDepth()                     + uiPart;
  m_puhWidth=pcCU->getWidth()                     + uiPart;
  m_puhHeight=pcCU->getHeight()                   + uiPart;

  m_pbIPCMFlag         = pcCU->getIPCMFlag()        + uiPart;

  m_pCtuAboveLeft      = pcCU->getCtuAboveLeft();//当前CU为pcCU子CU信息的复制　所以相邻CTU任为pcCU的相邻CTU
  m_pCtuAboveRight     = pcCU->getCtuAboveRight();
  m_pCtuAbove          = pcCU->getCtuAbove();
  m_pCtuLeft           = pcCU->getCtuLeft();

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    const RefPicList rpl=RefPicList(i);
    m_apcCUColocated[rpl] = pcCU->getCUColocated(rpl);
    m_apiMVPIdx[rpl]=pcCU->getMVPIdx(rpl)  + uiPart;
    m_apiMVPNum[rpl]=pcCU->getMVPNum(rpl)  + uiPart;
  }

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    const RefPicList rpl=RefPicList(i);
    m_acCUMvField[rpl].linkToWithOffset( pcCU->getCUMvField(rpl), uiPart );
  }

  UInt uiMaxCuWidth=pcCU->getSlice()->getSPS()->getMaxCUWidth();
  UInt uiMaxCuHeight=pcCU->getSlice()->getSPS()->getMaxCUHeight();

  UInt uiCoffOffset = uiMaxCuWidth*uiMaxCuHeight*uiAbsPartIdx/pcCU->getPic()->getNumPartitionsInCtu();

  for (UInt ch=0; ch<MAX_NUM_COMPONENT; ch++)
  {
    const ComponentID component = ComponentID(ch);
    const UInt componentShift   = m_pcPic->getComponentScaleX(component) + m_pcPic->getComponentScaleY(component);
    const UInt offset           = uiCoffOffset >> componentShift;
    m_pcTrCoeff[ch] = pcCU->getCoeff(component) + offset;
#if ADAPTIVE_QP_SELECTION
    m_pcArlCoeff[ch] = pcCU->getArlCoeff(component) + offset;
#endif
    m_pcIPCMSample[ch] = pcCU->getPCMSample(component) + offset;
  }
}

// Copy inter prediction info from the biggest CU（从大到小）
Void TComDataCU::copyInterPredInfoFrom    ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefPicList )//从最大的CU中位置为uiAbsPartIdx的子CU中复制与帧间预测相关的信息到当前CU
{
  m_pcPic              = pcCU->getPic();
  m_pcSlice            = pcCU->getSlice();
  m_ctuRsAddr          = pcCU->getCtuRsAddr();
  m_absZIdxInCtu       = uiAbsPartIdx;

  Int iRastPartIdx     = g_auiZscanToRaster[uiAbsPartIdx];
  m_uiCUPelX           = pcCU->getCUPelX() + m_pcPic->getMinCUWidth ()*( iRastPartIdx % m_pcPic->getNumPartInCtuWidth() );
  m_uiCUPelY           = pcCU->getCUPelY() + m_pcPic->getMinCUHeight()*( iRastPartIdx / m_pcPic->getNumPartInCtuWidth() );

  m_pCtuAboveLeft      = pcCU->getCtuAboveLeft();//当前CU为pcCU子CU信息的复制　所以相邻CTU任为pcCU的相邻CTU
  m_pCtuAboveRight     = pcCU->getCtuAboveRight();
  m_pCtuAbove          = pcCU->getCtuAbove();
  m_pCtuLeft           = pcCU->getCtuLeft();

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_apcCUColocated[i]  = pcCU->getCUColocated(RefPicList(i));
  }

  m_skipFlag           = pcCU->getSkipFlag ()             + uiAbsPartIdx;

  m_pePartSize         = pcCU->getPartitionSize ()        + uiAbsPartIdx;
  m_pePredMode         = pcCU->getPredictionMode()        + uiAbsPartIdx;
  m_ChromaQpAdj        = pcCU->getChromaQpAdj()           + uiAbsPartIdx;
  m_CUTransquantBypass = pcCU->getCUTransquantBypass()    + uiAbsPartIdx;
  m_puhInterDir        = pcCU->getInterDir      ()        + uiAbsPartIdx;

  m_puhDepth           = pcCU->getDepth ()                + uiAbsPartIdx;
  m_puhWidth           = pcCU->getWidth ()                + uiAbsPartIdx;
  m_puhHeight          = pcCU->getHeight()                + uiAbsPartIdx;

  m_pbMergeFlag        = pcCU->getMergeFlag()             + uiAbsPartIdx;
  m_puhMergeIndex      = pcCU->getMergeIndex()            + uiAbsPartIdx;

  m_apiMVPIdx[eRefPicList] = pcCU->getMVPIdx(eRefPicList) + uiAbsPartIdx;
  m_apiMVPNum[eRefPicList] = pcCU->getMVPNum(eRefPicList) + uiAbsPartIdx;

  m_acCUMvField[ eRefPicList ].linkToWithOffset( pcCU->getCUMvField(eRefPicList), uiAbsPartIdx );
}

// Copy small CU to bigger CU.（从小到大）
// One of quarter parts overwritten by predicted sub part.
Void TComDataCU::copyPartFrom( TComDataCU* pcCU, UInt uiPartUnitIdx, UInt uiDepth )//将深度为uiDepth的cCU的信息复制到当前CU中索引为uiPartUnitIdx的子CU
{
  assert( uiPartUnitIdx<4 );

  m_dTotalCost         += pcCU->getTotalCost();
  m_uiTotalDistortion  += pcCU->getTotalDistortion();
  m_uiTotalBits        += pcCU->getTotalBits();

  UInt uiOffset         = pcCU->getTotalNumPart()*uiPartUnitIdx;
  const UInt numValidComp=pcCU->getPic()->getNumberValidComponents();
  const UInt numValidChan=pcCU->getPic()->getChromaFormat()==CHROMA_400 ? 1:2;

  UInt uiNumPartition = pcCU->getTotalNumPart();
  Int iSizeInUchar  = sizeof( UChar ) * uiNumPartition;
  Int iSizeInBool   = sizeof( Bool  ) * uiNumPartition;

  Int sizeInChar  = sizeof( Char ) * uiNumPartition;
  memcpy( m_skipFlag   + uiOffset, pcCU->getSkipFlag(),       sizeof( *m_skipFlag )   * uiNumPartition );
  memcpy( m_phQP       + uiOffset, pcCU->getQP(),             sizeInChar                        );
  memcpy( m_pePartSize + uiOffset, pcCU->getPartitionSize(),  sizeof( *m_pePartSize ) * uiNumPartition );
  memcpy( m_pePredMode + uiOffset, pcCU->getPredictionMode(), sizeof( *m_pePredMode ) * uiNumPartition );
  memcpy( m_ChromaQpAdj + uiOffset, pcCU->getChromaQpAdj(),   sizeof( *m_ChromaQpAdj ) * uiNumPartition );
  memcpy( m_CUTransquantBypass + uiOffset, pcCU->getCUTransquantBypass(), sizeof( *m_CUTransquantBypass ) * uiNumPartition );
  memcpy( m_pbMergeFlag         + uiOffset, pcCU->getMergeFlag(),         iSizeInBool  );
  memcpy( m_puhMergeIndex       + uiOffset, pcCU->getMergeIndex(),        iSizeInUchar );

  for (UInt ch=0; ch<numValidChan; ch++)
  {
    memcpy( m_puhIntraDir[ch]   + uiOffset, pcCU->getIntraDir(ChannelType(ch)), iSizeInUchar );
  }

  memcpy( m_puhInterDir         + uiOffset, pcCU->getInterDir(),          iSizeInUchar );
  memcpy( m_puhTrIdx            + uiOffset, pcCU->getTransformIdx(),      iSizeInUchar );

  for(UInt comp=0; comp<numValidComp; comp++)
  {
    memcpy( m_crossComponentPredictionAlpha[comp] + uiOffset, pcCU->getCrossComponentPredictionAlpha(ComponentID(comp)), iSizeInUchar );
    memcpy( m_puhTransformSkip[comp]              + uiOffset, pcCU->getTransformSkip(ComponentID(comp))                , iSizeInUchar );
    memcpy( m_puhCbf[comp]                        + uiOffset, pcCU->getCbf(ComponentID(comp))                          , iSizeInUchar );
    memcpy( m_explicitRdpcmMode[comp]             + uiOffset, pcCU->getExplicitRdpcmMode(ComponentID(comp))            , iSizeInUchar );
  }

  memcpy( m_puhDepth  + uiOffset, pcCU->getDepth(),  iSizeInUchar );
  memcpy( m_puhWidth  + uiOffset, pcCU->getWidth(),  iSizeInUchar );
  memcpy( m_puhHeight + uiOffset, pcCU->getHeight(), iSizeInUchar );

  memcpy( m_pbIPCMFlag + uiOffset, pcCU->getIPCMFlag(), iSizeInBool );

  m_pCtuAboveLeft      = pcCU->getCtuAboveLeft();
  m_pCtuAboveRight     = pcCU->getCtuAboveRight();
  m_pCtuAbove          = pcCU->getCtuAbove();
  m_pCtuLeft           = pcCU->getCtuLeft();

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    const RefPicList rpl=RefPicList(i);
    memcpy( m_apiMVPIdx[rpl] + uiOffset, pcCU->getMVPIdx(rpl), iSizeInUchar );
    memcpy( m_apiMVPNum[rpl] + uiOffset, pcCU->getMVPNum(rpl), iSizeInUchar );
    m_apcCUColocated[rpl] = pcCU->getCUColocated(rpl);
  }

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    const RefPicList rpl=RefPicList(i);
    m_acCUMvField[rpl].copyFrom( pcCU->getCUMvField( rpl ), pcCU->getTotalNumPart(), uiOffset );
  }

  const UInt numCoeffY = (pcCU->getSlice()->getSPS()->getMaxCUWidth()*pcCU->getSlice()->getSPS()->getMaxCUHeight()) >> (uiDepth<<1);
  const UInt offsetY   = uiPartUnitIdx*numCoeffY;
  for (UInt ch=0; ch<numValidComp; ch++)
  {
    const ComponentID component = ComponentID(ch);
    const UInt componentShift   = m_pcPic->getComponentScaleX(component) + m_pcPic->getComponentScaleY(component);
    const UInt offset           = offsetY>>componentShift;
    memcpy( m_pcTrCoeff [ch] + offset, pcCU->getCoeff(component),    sizeof(TCoeff)*(numCoeffY>>componentShift) );
#if ADAPTIVE_QP_SELECTION
    memcpy( m_pcArlCoeff[ch] + offset, pcCU->getArlCoeff(component), sizeof(TCoeff)*(numCoeffY>>componentShift) );
#endif
    memcpy( m_pcIPCMSample[ch] + offset, pcCU->getPCMSample(component), sizeof(Pel)*(numCoeffY>>componentShift) );
  }

  m_uiTotalBins += pcCU->getTotalBins();
}

// Copy current predicted part to a CU in picture.
// It is used to predict for next part
Void TComDataCU::copyToPic( UChar uhDepth )//复制当前CU的信息到到其所属的CTU所在的位置
{
  TComDataCU* pCtu = m_pcPic->getCtu( m_ctuRsAddr );
  const UInt numValidComp=pCtu->getPic()->getNumberValidComponents();
  const UInt numValidChan=pCtu->getPic()->getChromaFormat()==CHROMA_400 ? 1:2;

  pCtu->getTotalCost()       = m_dTotalCost;
  pCtu->getTotalDistortion() = m_uiTotalDistortion;
  pCtu->getTotalBits()       = m_uiTotalBits;

  Int iSizeInUchar  = sizeof( UChar ) * m_uiNumPartition;
  Int iSizeInBool   = sizeof( Bool  ) * m_uiNumPartition;
  Int sizeInChar  = sizeof( Char ) * m_uiNumPartition;

  memcpy( pCtu->getSkipFlag() + m_absZIdxInCtu, m_skipFlag, sizeof( *m_skipFlag ) * m_uiNumPartition );

  memcpy( pCtu->getQP() + m_absZIdxInCtu, m_phQP, sizeInChar  );

  memcpy( pCtu->getPartitionSize()  + m_absZIdxInCtu, m_pePartSize, sizeof( *m_pePartSize ) * m_uiNumPartition );
  memcpy( pCtu->getPredictionMode() + m_absZIdxInCtu, m_pePredMode, sizeof( *m_pePredMode ) * m_uiNumPartition );
  memcpy( pCtu->getChromaQpAdj() + m_absZIdxInCtu, m_ChromaQpAdj, sizeof( *m_ChromaQpAdj ) * m_uiNumPartition );
  memcpy( pCtu->getCUTransquantBypass()+ m_absZIdxInCtu, m_CUTransquantBypass, sizeof( *m_CUTransquantBypass ) * m_uiNumPartition );
  memcpy( pCtu->getMergeFlag()         + m_absZIdxInCtu, m_pbMergeFlag,         iSizeInBool  );
  memcpy( pCtu->getMergeIndex()        + m_absZIdxInCtu, m_puhMergeIndex,       iSizeInUchar );
  for (UInt ch=0; ch<numValidChan; ch++)
  {
    memcpy( pCtu->getIntraDir(ChannelType(ch)) + m_absZIdxInCtu, m_puhIntraDir[ch], iSizeInUchar);
  }

  memcpy( pCtu->getInterDir()          + m_absZIdxInCtu, m_puhInterDir,         iSizeInUchar );
  memcpy( pCtu->getTransformIdx()      + m_absZIdxInCtu, m_puhTrIdx,            iSizeInUchar );

  for(UInt comp=0; comp<numValidComp; comp++)
  {
    memcpy( pCtu->getCrossComponentPredictionAlpha(ComponentID(comp)) + m_absZIdxInCtu, m_crossComponentPredictionAlpha[comp], iSizeInUchar );
    memcpy( pCtu->getTransformSkip(ComponentID(comp))                 + m_absZIdxInCtu, m_puhTransformSkip[comp],              iSizeInUchar );
    memcpy( pCtu->getCbf(ComponentID(comp))                           + m_absZIdxInCtu, m_puhCbf[comp],                        iSizeInUchar );
    memcpy( pCtu->getExplicitRdpcmMode(ComponentID(comp))             + m_absZIdxInCtu, m_explicitRdpcmMode[comp],             iSizeInUchar );
  }

  memcpy( pCtu->getDepth()  + m_absZIdxInCtu, m_puhDepth,  iSizeInUchar );
  memcpy( pCtu->getWidth()  + m_absZIdxInCtu, m_puhWidth,  iSizeInUchar );
  memcpy( pCtu->getHeight() + m_absZIdxInCtu, m_puhHeight, iSizeInUchar );

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    const RefPicList rpl=RefPicList(i);
    memcpy( pCtu->getMVPIdx(rpl) + m_absZIdxInCtu, m_apiMVPIdx[rpl], iSizeInUchar );
    memcpy( pCtu->getMVPNum(rpl) + m_absZIdxInCtu, m_apiMVPNum[rpl], iSizeInUchar );
  }

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    const RefPicList rpl=RefPicList(i);
    m_acCUMvField[rpl].copyTo( pCtu->getCUMvField( rpl ), m_absZIdxInCtu );
  }

  memcpy( pCtu->getIPCMFlag() + m_absZIdxInCtu, m_pbIPCMFlag,         iSizeInBool  );

  const UInt numCoeffY    = (pCtu->getSlice()->getSPS()->getMaxCUWidth()*pCtu->getSlice()->getSPS()->getMaxCUHeight())>>(uhDepth<<1);
  const UInt offsetY      = m_absZIdxInCtu*m_pcPic->getMinCUWidth()*m_pcPic->getMinCUHeight();
  for (UInt comp=0; comp<numValidComp; comp++)
  {
    const ComponentID component = ComponentID(comp);
    const UInt componentShift   = m_pcPic->getComponentScaleX(component) + m_pcPic->getComponentScaleY(component);
    memcpy( pCtu->getCoeff(component)   + (offsetY>>componentShift), m_pcTrCoeff[component], sizeof(TCoeff)*(numCoeffY>>componentShift) );
#if ADAPTIVE_QP_SELECTION
    memcpy( pCtu->getArlCoeff(component) + (offsetY>>componentShift), m_pcArlCoeff[component], sizeof(TCoeff)*(numCoeffY>>componentShift) );
#endif
    memcpy( pCtu->getPCMSample(component) + (offsetY>>componentShift), m_pcIPCMSample[component], sizeof(Pel)*(numCoeffY>>componentShift) );
  }

  pCtu->getTotalBins() = m_uiTotalBins;
}

// --------------------------------------------------------------------------------------------------------------------
// Other public functions
// --------------------------------------------------------------------------------------------------------------------
//得到不同方位PU的方法较为巧妙　不易理解　大体思路为：不需要准确得到相邻PU的起始位置　只需要表示出当前PU相邻的小块在整帧图的位置　因为最终的目标是相邻PU中的信息（预测模式），
//而属于相邻PU的小块已经包含相邻PU全部的信息　表示的方法是用小块所在的CTU或CU和相对其CTU或CU位置的PartUnitIdx来共同表示
TComDataCU* TComDataCU::getPULeft( UInt& uiLPartUnitIdx,
                                   UInt uiCurrPartUnitIdx,
                                   Bool bEnforceSliceRestriction,
                                   Bool bEnforceTileRestriction )
{
  UInt uiAbsPartIdx       = g_auiZscanToRaster[uiCurrPartUnitIdx];//uiCurrPartUnitIdx为当前CU中PU(左上小块)在CTU中的位置
  UInt uiAbsZorderCUIdx   = g_auiZscanToRaster[m_absZIdxInCtu];//当前CU(左上小块)在CTU中的位置（当前CU表示PU所在的CU）
  const UInt numPartInCtuWidth = m_pcPic->getNumPartInCtuWidth();

  if ( !RasterAddress::isZeroCol( uiAbsPartIdx, numPartInCtuWidth ) )//如果当前PU与其所在CTU的左边界不相邻
  {
    uiLPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdx - 1 ];//则当前PU左侧PU可用左侧相邻小块表示
    if ( RasterAddress::isEqualCol( uiAbsPartIdx, uiAbsZorderCUIdx, numPartInCtuWidth ) )//若当前PU与其所在CU的的左边界相邻（说明左侧PU不在同一个CU中　而在同一个CTU中）
    {
      return m_pcPic->getCtu( getCtuRsAddr() );//返回该CTU
    }
    else//若当前PU与其所在CU的的左边界不相邻（说明当前PU和其左侧PU在同一个CU中）
    {
      uiLPartUnitIdx -= m_absZIdxInCtu;//该PU左侧相邻小块相对于当前CU!的位置　而不是相对当前CU所属的CTU的位置　因为返回的TComDataCU*不同
      return this; //返回当前CU
    }
  }
  //如果该PU相邻其所在CTU的左边界(说明左侧PU在该CTU的左侧CTU中)
  uiLPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdx + numPartInCtuWidth - 1 ];//该PU左侧相邻小块在m_pCtuLeft中的位置
  if ( (bEnforceSliceRestriction && !CUIsFromSameSlice(m_pCtuLeft)) || (bEnforceTileRestriction && !CUIsFromSameTile(m_pCtuLeft)) )//无法获得左侧CTU
  {
    return NULL;
  }
  return m_pCtuLeft;//返回左侧CTU
}


TComDataCU* TComDataCU::getPUAbove( UInt& uiAPartUnitIdx,
                                    UInt uiCurrPartUnitIdx,
                                    Bool bEnforceSliceRestriction,
                                    Bool planarAtCtuBoundary,
                                    Bool bEnforceTileRestriction )//获得当前PU上方的PU 过程同上　不在叙述
{
  UInt uiAbsPartIdx       = g_auiZscanToRaster[uiCurrPartUnitIdx];
  UInt uiAbsZorderCUIdx   = g_auiZscanToRaster[m_absZIdxInCtu];
  const UInt numPartInCtuWidth = m_pcPic->getNumPartInCtuWidth();

  if ( !RasterAddress::isZeroRow( uiAbsPartIdx, numPartInCtuWidth ) )
  {
    uiAPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdx - numPartInCtuWidth ];
    if ( RasterAddress::isEqualRow( uiAbsPartIdx, uiAbsZorderCUIdx, numPartInCtuWidth ) )
    {
      return m_pcPic->getCtu( getCtuRsAddr() );
    }
    else
    {
      uiAPartUnitIdx -= m_absZIdxInCtu;
      return this;
    }
  }

  if(planarAtCtuBoundary)
  {
    return NULL;
  }

  uiAPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdx + m_pcPic->getNumPartitionsInCtu() - numPartInCtuWidth ];

  if ( (bEnforceSliceRestriction && !CUIsFromSameSlice(m_pCtuAbove)) || (bEnforceTileRestriction && !CUIsFromSameTile(m_pCtuAbove)) )
  {
    return NULL;
  }
  return m_pCtuAbove;
}

TComDataCU* TComDataCU::getPUAboveLeft( UInt& uiALPartUnitIdx, UInt uiCurrPartUnitIdx, Bool bEnforceSliceRestriction )//获得位置为uiCurrPartUnitIdx的PU的左上方PU
{
  UInt uiAbsPartIdx       = g_auiZscanToRaster[uiCurrPartUnitIdx];
  UInt uiAbsZorderCUIdx   = g_auiZscanToRaster[m_absZIdxInCtu];
  const UInt numPartInCtuWidth = m_pcPic->getNumPartInCtuWidth();

  if ( !RasterAddress::isZeroCol( uiAbsPartIdx, numPartInCtuWidth ) )//如果当前PU与其所在CTU的左边界不相邻
  {
    if ( !RasterAddress::isZeroRow( uiAbsPartIdx, numPartInCtuWidth ) )//如果当前PU与其所在CTU的上边界不相邻
    {
      uiALPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdx - numPartInCtuWidth - 1 ];//说明左上PU在当前CU或CTU中　计算PU在其所在CTU中的位置
      if ( RasterAddress::isEqualRowOrCol( uiAbsPartIdx, uiAbsZorderCUIdx, numPartInCtuWidth ) )//若当前PU与其所在CU的的左边界或上边界相邻（说明左上方PU不在同一个CU中　而在同一个CTU中）
      {
        return m_pcPic->getCtu( getCtuRsAddr() );//放回当前CTU
      }
      else//左上方PU在同一个CU中
      {
        uiALPartUnitIdx -= m_absZIdxInCtu;//该PU左上方相邻小块相对于当前CU的位置
        return this;
      }
    }
    //当前PU与其所在CTU的上边界相邻 与其所在CTU的左边界不相邻 说明左上方PU在当前CTU的上方的CTU中
    uiALPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdx + getPic()->getNumPartitionsInCtu() - numPartInCtuWidth - 1 ];//当前PU左上方PU在当前CTU上方CTU中的位置
    if ( bEnforceSliceRestriction && !CUIsFromSameSliceAndTile(m_pCtuAbove) )
    {
      return NULL;
    }
    return m_pCtuAbove;//返回上方CTU
  }

  if ( !RasterAddress::isZeroRow( uiAbsPartIdx, numPartInCtuWidth ) )//当前PU与其所在CTU的左边界相邻 与其所在CTU的上边界不相邻 说明左上方PU在当前CTU的左侧的CTU中
  {
    uiALPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdx - 1 ];//当前PU左上方PU在当前CTU左侧CTU中的位置
    if ( bEnforceSliceRestriction && !CUIsFromSameSliceAndTile(m_pCtuLeft) )
    {
      return NULL;
    }
    return m_pCtuLeft;//返回左侧CTU
  }
  //当前PU与其所在CTU的左边界相邻 与其所在CTU的上边界相邻 说明左上方PU在当前CTU的左上方的CTU中
  uiALPartUnitIdx = g_auiRasterToZscan[ m_pcPic->getNumPartitionsInCtu() - 1 ];//当前PU左上方PU在当前CTU左上方CTU中的位置
  if ( bEnforceSliceRestriction && !CUIsFromSameSliceAndTile(m_pCtuAboveLeft) )
  {
    return NULL;
  }
  return m_pCtuAboveLeft;//返回左上方CTU
}

TComDataCU* TComDataCU::getPUBelowLeft(UInt& uiBLPartUnitIdx,  UInt uiCurrPartUnitIdx, UInt uiPartUnitOffset, Bool bEnforceSliceRestriction)//uiPartUnitOffset表示当前PU向下的偏移量
{
  UInt uiAbsPartIdxLB     = g_auiZscanToRaster[uiCurrPartUnitIdx];//uiAbsPartIdxLB表示当前PU左下4*4小块的位置！！
  const UInt numPartInCtuWidth = m_pcPic->getNumPartInCtuWidth();
  UInt uiAbsZorderCUIdxLB = g_auiZscanToRaster[ m_absZIdxInCtu ] + ((m_puhHeight[0] / m_pcPic->getMinCUHeight()) - 1)*numPartInCtuWidth;

  if( ( m_pcPic->getCtu(m_ctuRsAddr)->getCUPelY() + g_auiRasterToPelY[uiAbsPartIdxLB] + (m_pcPic->getPicSym()->getMinCUHeight() * uiPartUnitOffset)) >= m_pcSlice->getSPS()->getPicHeightInLumaSamples())
  {//左下PU超出图像范围
    uiBLPartUnitIdx = MAX_UINT;
    return NULL;
  }

  if ( RasterAddress::lessThanRow( uiAbsPartIdxLB, m_pcPic->getNumPartInCtuHeight() - uiPartUnitOffset, numPartInCtuWidth ) )//向下不要超出当前CTU
  {
    if ( !RasterAddress::isZeroCol( uiAbsPartIdxLB, numPartInCtuWidth ) )//当前PU与其所在CTU的左边界不相邻　(说明左下PU在当前CU或CTU中)
    {
      if ( uiCurrPartUnitIdx > g_auiRasterToZscan[ uiAbsPartIdxLB + uiPartUnitOffset * numPartInCtuWidth - 1 ] )//左下角的PU z order顺序在当前PU之前　这样才能确保解码过程中先得到左下角信息　否则无法得到左下角PU!!
      {　//该判断语句不好理解　需对照CTU的z order索引表仔细观察　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　
        uiBLPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdxLB + uiPartUnitOffset * numPartInCtuWidth - 1 ];//左下小块的位置代表左下PU的位置
        if ( RasterAddress::isEqualRowOrCol( uiAbsPartIdxLB, uiAbsZorderCUIdxLB, numPartInCtuWidth ) )//若当前PU与其所在CU的的左边界或下边界相邻（说明左上方PU不在同一个CU中　而在同一个CTU中）
        {
          return m_pcPic->getCtu( getCtuRsAddr() );//返回当前CTU
        }
        else
        {
          uiBLPartUnitIdx -= m_absZIdxInCtu;////该PU左下方相邻小块相对于当前CU的位置
          return this;//返回当前CU
        }
      }
      uiBLPartUnitIdx = MAX_UINT;
      return NULL;
    }
    //当前PU与其所在CTU的左边界相邻 向下不超出当前CTU　说明左下方PU在当前CTU的左侧的CTU中
    uiBLPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdxLB + (1+uiPartUnitOffset) * numPartInCtuWidth - 1 ];//左下PU在左侧CTU中的位置
    if ( bEnforceSliceRestriction && !CUIsFromSameSliceAndTile(m_pCtuLeft) )
    {
      return NULL;
    }
    return m_pCtuLeft;//
  }
  //注意！　此处没有判断左下PU是否在左下的CTU中的原因是解码的过程中左下CTU较当前CTU后解码　故无法获得左下的CTU的信息
  uiBLPartUnitIdx = MAX_UINT;
  return NULL;
}

TComDataCU* TComDataCU::getPUAboveRight(UInt&  uiARPartUnitIdx, UInt uiCurrPartUnitIdx, UInt uiPartUnitOffset, Bool bEnforceSliceRestriction)
{//获得当前PU右上PU　过程基本同上　不在在于需判断右上PU是否在右上CTU中　因为raster顺序解码可以获得右上CTU的信息　不在叙述
  UInt uiAbsPartIdxRT     = g_auiZscanToRaster[uiCurrPartUnitIdx];//uiAbsPartIdxLB表示当前PU右上4*4小块的位置
  UInt uiAbsZorderCUIdx   = g_auiZscanToRaster[ m_absZIdxInCtu ] + (m_puhWidth[0] / m_pcPic->getMinCUWidth()) - 1;
  const UInt numPartInCtuWidth = m_pcPic->getNumPartInCtuWidth();

  if( ( m_pcPic->getCtu(m_ctuRsAddr)->getCUPelX() + g_auiRasterToPelX[uiAbsPartIdxRT] + (m_pcPic->getPicSym()->getMinCUHeight() * uiPartUnitOffset)) >= m_pcSlice->getSPS()->getPicWidthInLumaSamples() )
  {
    uiARPartUnitIdx = MAX_UINT;
    return NULL;
  }

  if ( RasterAddress::lessThanCol( uiAbsPartIdxRT, numPartInCtuWidth - uiPartUnitOffset, numPartInCtuWidth ) )
  {
    if ( !RasterAddress::isZeroRow( uiAbsPartIdxRT, numPartInCtuWidth ) )
    {
      if ( uiCurrPartUnitIdx > g_auiRasterToZscan[ uiAbsPartIdxRT - numPartInCtuWidth + uiPartUnitOffset ] )//右上角的PU z order顺序在当前PU之前　这样才能确保解码过程中先得到右上角信息　否则无法得到右上角PU
      {
        uiARPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdxRT - numPartInCtuWidth + uiPartUnitOffset ];
        if ( RasterAddress::isEqualRowOrCol( uiAbsPartIdxRT, uiAbsZorderCUIdx, numPartInCtuWidth ) )
        {
          return m_pcPic->getCtu( getCtuRsAddr() );
        }
        else
        {
          uiARPartUnitIdx -= m_absZIdxInCtu;
          return this;
        }
      }
      uiARPartUnitIdx = MAX_UINT;
      return NULL;
    }

    uiARPartUnitIdx = g_auiRasterToZscan[ uiAbsPartIdxRT + m_pcPic->getNumPartitionsInCtu() - numPartInCtuWidth + uiPartUnitOffset ];
    if ( bEnforceSliceRestriction && !CUIsFromSameSliceAndTile(m_pCtuAbove) )
    {
      return NULL;
    }
    return m_pCtuAbove;
  }

  if ( !RasterAddress::isZeroRow( uiAbsPartIdxRT, numPartInCtuWidth ) )
  {
    uiARPartUnitIdx = MAX_UINT;
    return NULL;
  }

  uiARPartUnitIdx = g_auiRasterToZscan[ m_pcPic->getNumPartitionsInCtu() - numPartInCtuWidth + uiPartUnitOffset-1 ];
  if ( bEnforceSliceRestriction && !CUIsFromSameSliceAndTile(m_pCtuAboveRight) )
  {
    return NULL;
  }
  return m_pCtuAboveRight;
}

/** Get left QpMinCu
*\param   uiLPartUnitIdx
*\param   uiCurrAbsIdxInCtu
*\returns TComDataCU*   point of TComDataCU of left QpMinCu
*/
//quantization group（QG）量化组　一个CTU中可以包含一个或多个固定大小的QG,同一个QG内的所有含有非零系数的CU共享一个QP，不同的QG可以使用不同的QP
TComDataCU* TComDataCU::getQpMinCuLeft( UInt& uiLPartUnitIdx, UInt uiCurrAbsIdxInCtu )
{
  const UInt numPartInCtuWidth = m_pcPic->getNumPartInCtuWidth();
  const UInt maxCUDepth        = getSlice()->getSPS()->getMaxTotalCUDepth();
  const UInt maxCuDQPDepth     = getSlice()->getPPS()->getMaxCuDQPDepth();//QG的深度 决定了QG的大小
  const UInt doubleDepthDifference = ((maxCUDepth - maxCuDQPDepth)<<1);//QG深度与CU最大深度差的2倍
  UInt absZorderQpMinCUIdx = (uiCurrAbsIdxInCtu>>doubleDepthDifference)<<doubleDepthDifference;//得到包含uiCurrAbsIdxInCtu位置小块的QG（左上4*4小块）的位置 uiCurrAbsIdxInCtu>>doubleDepthDifference为将CTU分成QG块大小的QG块的位置索引
  UInt absRorderQpMinCUIdx = g_auiZscanToRaster[absZorderQpMinCUIdx];

  // check for left CTU boundary
  if ( RasterAddress::isZeroCol(absRorderQpMinCUIdx, numPartInCtuWidth) )//该QG是否在CTU的边界
  {
    return NULL;//只能用当前CTU内的QG做参考！！若在CTU的边界则用之前的QG做参考
  }

  // get index of left-CU relative to top-left corner of current quantization group
  uiLPartUnitIdx = g_auiRasterToZscan[absRorderQpMinCUIdx - 1];//得到左侧QG位置（因为absRorderQpMinCUIdx的小块为当前QG左上小块　所以absRorderQpMinCUIdx - 1的小块必属于左侧QG）

  // return pointer to current CTU
  return m_pcPic->getCtu( getCtuRsAddr() );//返回左侧QG所在的CTU 也是当前QG所在的CTU
}

/** Get Above QpMinCu
*\param   uiAPartUnitIdx
*\param   uiCurrAbsIdxInCtu
*\returns TComDataCU*   point of TComDataCU of above QpMinCu
*/
TComDataCU* TComDataCU::getQpMinCuAbove( UInt& uiAPartUnitIdx, UInt uiCurrAbsIdxInCtu )
{//得到当前QG上方QG 同上　不在叙述
  const UInt numPartInCtuWidth = m_pcPic->getNumPartInCtuWidth();
  const UInt maxCUDepth        = getSlice()->getSPS()->getMaxTotalCUDepth();
  const UInt maxCuDQPDepth     = getSlice()->getPPS()->getMaxCuDQPDepth();
  const UInt doubleDepthDifference = ((maxCUDepth - maxCuDQPDepth)<<1);
  UInt absZorderQpMinCUIdx = (uiCurrAbsIdxInCtu>>doubleDepthDifference)<<doubleDepthDifference;
  UInt absRorderQpMinCUIdx = g_auiZscanToRaster[absZorderQpMinCUIdx];

  // check for top CTU boundary
  if ( RasterAddress::isZeroRow( absRorderQpMinCUIdx, numPartInCtuWidth) )
  {
    return NULL;
  }

  // get index of top-CU relative to top-left corner of current quantization group
  uiAPartUnitIdx = g_auiRasterToZscan[absRorderQpMinCUIdx - numPartInCtuWidth];

  // return pointer to current CTU
  return m_pcPic->getCtu( getCtuRsAddr() );
}



/** Get reference QP from left QpMinCu or latest coded QP
*\param   uiCurrAbsIdxInCtu
*\returns Char   reference QP value
*/
Char TComDataCU::getRefQP( UInt uiCurrAbsIdxInCtu )//计算当前QG的预测QP　用当前QG的左侧QG和上方QG的QP值取平均　如果左侧QG或上方QG无法获得则用最后一个能得到的编码的QP替换
{//uiCurrAbsIdxInCtu为QG在当前Cu中的位置
  UInt lPartIdx = MAX_UINT;
  UInt aPartIdx = MAX_UINT;
  TComDataCU* cULeft  = getQpMinCuLeft ( lPartIdx, m_absZIdxInCtu + uiCurrAbsIdxInCtu );
  TComDataCU* cUAbove = getQpMinCuAbove( aPartIdx, m_absZIdxInCtu + uiCurrAbsIdxInCtu );
  return (((cULeft? cULeft->getQP( lPartIdx ): getLastCodedQP( uiCurrAbsIdxInCtu )) + (cUAbove? cUAbove->getQP( aPartIdx ): getLastCodedQP( uiCurrAbsIdxInCtu )) + 1) >> 1);
}//一个CU只对应一个Qp!!

Int TComDataCU::getLastValidPartIdx( Int iAbsPartIdx )//得到最后一个可以得到QP的有效的位置索引
{
  Int iLastValidPartIdx = iAbsPartIdx-1;//iAbsPartIdx所在QG的上一个QG
  while ( iLastValidPartIdx >= 0
       && getPredictionMode( iLastValidPartIdx ) == NUMBER_OF_PREDICTION_MODES )
  {
    UInt uiDepth = getDepth( iLastValidPartIdx );
    iLastValidPartIdx -= m_uiNumPartition>>(uiDepth<<1);// qp的解析是以CU为单位的 从后向前寻找有效QP
  }
  return iLastValidPartIdx;//根据判断条件　iLastValidPartIdx为负说明当前CU无法得到有效的QP
}

Char TComDataCU::getLastCodedQP( UInt uiAbsPartIdx )
{
  UInt uiQUPartIdxMask = ~((1<<((getSlice()->getSPS()->getMaxTotalCUDepth() - getSlice()->getPPS()->getMaxCuDQPDepth())<<1))-1);//uiAbsPartIdx&uiQUPartIdxMask为uiAbsPartIdx所在QG的起始位置
  Int iLastValidPartIdx = getLastValidPartIdx( uiAbsPartIdx&uiQUPartIdxMask ); // A idx will be invalid if it is off the right or bottom edge of the picture.
  // If this CU is in the first CTU of the slice and there is no valid part before this one, use slice QP
  if ( getPic()->getPicSym()->getCtuTsToRsAddrMap(getSlice()->getSliceCurStartCtuTsAddr()) == getCtuRsAddr() && Int(getZorderIdxInCtu())+iLastValidPartIdx<0)//当前CTU为Slice第一个CTU并且最后一个能得到有效QP的位置在当前CTU之前的CTu
  {
    return getSlice()->getSliceQp();//当前slice的QP作为可得到的QP
  }
  else if ( iLastValidPartIdx >= 0 )//当前子CU存在可得到的QP
  {
    // If there is a valid part within the current Sub-CU, use it
    return getQP( iLastValidPartIdx );//直接获取该QP
  }
  else//当前子CU不存在可得到的QP
  {
    if ( getZorderIdxInCtu() > 0 )//当前CU不是CTU中的第一个子CU 则在当前CTU寻找可获得的有效的QP(寻找有效qp的方法是从给定位置从前往后寻找　若当前CU不为CTU中第一个子CU则有效qp可能存在在当前CTU中（CTU中该Cu之前的CU）　若当前CU为CTU中第一个子CU则有效qp在一定不在该CTU内　可能在前一个CTU中)
    {
      // If this wasn't the first sub-cu within the Ctu, explore the CTU itself.
      return getPic()->getCtu( getCtuRsAddr() )->getLastCodedQP( getZorderIdxInCtu() ); // TODO - remove this recursion
    }
    else if ( getPic()->getPicSym()->getCtuRsToTsAddrMap(getCtuRsAddr()) > 0
      && CUIsFromSameSliceTileAndWavefrontRow(getPic()->getCtu(getPic()->getPicSym()->getCtuTsToRsAddrMap(getPic()->getPicSym()->getCtuRsToTsAddrMap(getCtuRsAddr())-1))) )
    {
      // If this isn't the first Ctu (how can it be due to the first 'if'?), and the previous Ctu is from the same tile, examine the previous Ctu.
      //当前CTU和前一个CTU来自同一个tile 那么在前一个CTU中寻找可获得有效的QP
      return getPic()->getCtu( getPic()->getPicSym()->getCtuTsToRsAddrMap(getPic()->getPicSym()->getCtuRsToTsAddrMap(getCtuRsAddr())-1) )->getLastCodedQP( getPic()->getNumPartitionsInCtu() );  // TODO - remove this recursion
    }
    else//没有其他寻找有效QP的方法　用slice QP作为有效Qp
    {
      // No other options available - use the slice-level QP.
      return getSlice()->getSliceQp();
    }
  }
}


/** Check whether the CU is coded in lossless coding mode.
 * \param   absPartIdx
 * \returns true if the CU is coded in lossless coding mode; false if otherwise
 */
Bool TComDataCU::isLosslessCoded(UInt absPartIdx)//该CU是否为lossless模式
{
  return (getSlice()->getPPS()->getTransquantBypassEnableFlag() && getCUTransquantBypass (absPartIdx));
}


/** Get allowed chroma intra modes
*   - fills uiModeList with chroma intra modes
*
*\param   [in]  uiAbsPartIdx
*\param   [out] uiModeList pointer to chroma intra modes array
*/
Void TComDataCU::getAllowedChromaDir( UInt uiAbsPartIdx, UInt uiModeList[NUM_CHROMA_MODE] )//得到允许的色度帧内预测模式
{
  uiModeList[0] = PLANAR_IDX;
  uiModeList[1] = VER_IDX;
  uiModeList[2] = HOR_IDX;
  uiModeList[3] = DC_IDX;
  uiModeList[4] = DM_CHROMA_IDX;
  assert(4<NUM_CHROMA_MODE);

  UInt uiLumaMode = getIntraDir( CHANNEL_TYPE_LUMA, uiAbsPartIdx );

  for( Int i = 0; i < NUM_CHROMA_MODE - 1; i++ )//若对应亮度预测模式为前4种模式中的一种　则将其替换为角度预测中的模式34
  {
    if( uiLumaMode == uiModeList[i] )
    {
      uiModeList[i] = 34; // VER+8 mode
      break;
    }
  }
}

/** Get most probable intra modes
*\param   uiAbsPartIdx    partition index
*\param   uiIntraDirPred  pointer to the array for MPM storage //帧内预测候选模式列表
*\param   compID          colour component ID
*\param   piMode          it is set with MPM mode in case both MPM are equal. It is used to restrict RD search at encode side.
*\returns Number of MPM
*/
//相邻块的帧内预测模式相同或相似的概率较大　故可以用相邻PU的模式来预测当前PU的预测模式　来降低冗余　提高编码效率
Void TComDataCU::getIntraDirPredictor( UInt uiAbsPartIdx, Int uiIntraDirPred[NUM_MOST_PROBABLE_MODES], const ComponentID compID, Int* piMode  )
{//用相邻PU预测当前PU最有可能的帧间模式
  TComDataCU* pcCULeft, *pcCUAbove;
  UInt        LeftPartIdx  = MAX_UINT;
  UInt        AbovePartIdx = MAX_UINT;
  Int         iLeftIntraDir, iAboveIntraDir;
  const TComSPS *sps=getSlice()->getSPS();
  const UInt partsPerMinCU = 1<<(2*(sps->getMaxTotalCUDepth() - sps->getLog2DiffMaxMinCodingBlockSize()));

  const ChannelType chType = toChannelType(compID);
  const ChromaFormat chForm = getPic()->getChromaFormat();
  // Get intra direction of left PU
  pcCULeft = getPULeft( LeftPartIdx, m_absZIdxInCtu + uiAbsPartIdx );//得到当前pu左侧pu

  if (isChroma(compID))//若为色度分量则需转换LeftPartIdx为色度分量对应的位置
  {
    LeftPartIdx = getChromasCorrespondingPULumaIdx(LeftPartIdx, chForm, partsPerMinCU);
  }
  iLeftIntraDir  = pcCULeft ? ( pcCULeft->isIntra( LeftPartIdx ) ? pcCULeft->getIntraDir( chType, LeftPartIdx ) : DC_IDX ) : DC_IDX;//得到左侧pu的帧间模式　若左侧pu无法获得或不为帧间预测　则设为DC模式

  // Get intra direction of above PU
  pcCUAbove = getPUAbove( AbovePartIdx, m_absZIdxInCtu + uiAbsPartIdx, true, true );

  if (isChroma(compID))//若为色度分量则需转换LeftPartIdx为色度分量对应的位置
  {
    AbovePartIdx = getChromasCorrespondingPULumaIdx(AbovePartIdx, chForm, partsPerMinCU);
  }
  iAboveIntraDir = pcCUAbove ? ( pcCUAbove->isIntra( AbovePartIdx ) ? pcCUAbove->getIntraDir( chType, AbovePartIdx ) : DC_IDX ) : DC_IDX;//得到上方pu的帧间模式　若左侧pu无法获得或不为帧间预测　则设为DC模式

  if (isChroma(chType))//若色度分量对应亮度分量模式　则色度分量模式为该位置亮度分量模式
  {
    if (iLeftIntraDir  == DM_CHROMA_IDX)
    {
      iLeftIntraDir  = pcCULeft-> getIntraDir( CHANNEL_TYPE_LUMA, LeftPartIdx  );
    }
    if (iAboveIntraDir == DM_CHROMA_IDX)
    {
      iAboveIntraDir = pcCUAbove->getIntraDir( CHANNEL_TYPE_LUMA, AbovePartIdx );
    }
  }

  assert (2<NUM_MOST_PROBABLE_MODES);
  if(iLeftIntraDir == iAboveIntraDir)//若ModeA与ModeB相同
  {
    if( piMode )
    {
      *piMode = 1;
    }

    if (iLeftIntraDir > 1) // angular modes　//都为角度模式
    {//uiIntraDirPred[0]为ModeA　uiIntraDirPred[1]和uiIntraDirPred[２]为与ModeA相邻的两个模式
      uiIntraDirPred[0] = iLeftIntraDir;
      uiIntraDirPred[1] = ((iLeftIntraDir + 29) % 32) + 2;
      uiIntraDirPred[2] = ((iLeftIntraDir - 1 ) % 32) + 2;
    }
    else //non-angular　//都为Planar或DC模式
    {
      uiIntraDirPred[0] = PLANAR_IDX;//uiIntraDirPred[0]为Planar模式
      uiIntraDirPred[1] = DC_IDX;//uiIntraDirPred[１]为DC模式
      uiIntraDirPred[2] = VER_IDX;//uiIntraDirPred[2]为垂直模式
    }
  }
  else//ModeA与ModeB不同
  {
    if( piMode )
    {
      *piMode = 2;
    }
    uiIntraDirPred[0] = iLeftIntraDir;//uiIntraDirPred[0]为ModeA
    uiIntraDirPred[1] = iAboveIntraDir;//uiIntraDirPred[1]为ModeB 
    //uiIntraDirPred[３]需另外判断
    if (iLeftIntraDir && iAboveIntraDir ) //both modes are non-planar
    {
      uiIntraDirPred[2] = PLANAR_IDX;
    }
    else//都不为Planar模式　
    {
      uiIntraDirPred[2] =  (iLeftIntraDir+iAboveIntraDir)<2? VER_IDX : DC_IDX;//若都不是垂直模式则uiIntraDirPred[2]为垂直模式　否则为DC模式
    }
  }
  for (UInt i=0; i<NUM_MOST_PROBABLE_MODES; i++)
  {
    assert(uiIntraDirPred[i] < 35);
  }
}

UInt TComDataCU::getCtxSplitFlag( UInt uiAbsPartIdx, UInt uiDepth )
{//根据左侧和上方PU块所属的CU块是否存在及分割深度情况得到splitflag上下文索引(用于熵编码)
  TComDataCU* pcTempCU;
  UInt        uiTempPartIdx;
  UInt        uiCtx;
  // Get left split flag
  pcTempCU = getPULeft( uiTempPartIdx, m_absZIdxInCtu + uiAbsPartIdx );
  uiCtx  = ( pcTempCU ) ? ( ( pcTempCU->getDepth( uiTempPartIdx ) > uiDepth ) ? 1 : 0 ) : 0;

  // Get above split flag
  pcTempCU = getPUAbove( uiTempPartIdx, m_absZIdxInCtu + uiAbsPartIdx );
  uiCtx += ( pcTempCU ) ? ( ( pcTempCU->getDepth( uiTempPartIdx ) > uiDepth ) ? 1 : 0 ) : 0;

  return uiCtx;
}

UInt TComDataCU::getCtxQtCbf( TComTU &rTu, const ChannelType chType )
{//根据Tu块相对Cu块的深度得到cbf上下文索引(用于熵编码)
  const UInt transformDepth = rTu.GetTransformDepthRel();

  if (isChroma(chType))
  {
    return transformDepth;
  }
  else
  {
    const UInt uiCtx = ( transformDepth == 0 ? 1 : 0 );
    return uiCtx;
  }
}

UInt TComDataCU::getQuadtreeTULog2MinSizeInCU( UInt absPartIdx )//以absPartIdx处CU作为根节点能得到的最小的TU块的大小
{//此处涉及到语法元素（intraSplitFlag和interSplitFlag）
  UInt log2CbSize = g_aucConvertToBit[getWidth( absPartIdx )] + 2;//当前CU的log2大小
  PartSize  partSize  = getPartitionSize( absPartIdx );
  UInt quadtreeTUMaxDepth = isIntra( absPartIdx ) ? m_pcSlice->getSPS()->getQuadtreeTUMaxDepthIntra() : m_pcSlice->getSPS()->getQuadtreeTUMaxDepthInter();//TU允许的最大深度(相对CU)
  Int intraSplitFlag = ( isIntra( absPartIdx ) && partSize == SIZE_NxN ) ? 1 : 0;//intraSplitFlag若为真　则split_transform_flag被推断为1　TU必定会往下分割一次(可节约编码split_transform_flag所需的比特位)
  Int interSplitFlag = ((quadtreeTUMaxDepth == 1) && isInter( absPartIdx ) && (partSize != SIZE_2Nx2N) );//intraSplitFlag若为真　则split_transform_flag被推断为1

  UInt log2MinTUSizeInCU = 0;
  if (log2CbSize < (m_pcSlice->getSPS()->getQuadtreeTULog2MinSize() + quadtreeTUMaxDepth - 1 + interSplitFlag + intraSplitFlag) )//当前Cu能得到最小Tu块小于规定的最小Tu块大小
  {
    // when fully making use of signaled TUMaxDepth + inter/intraSplitFlag, resulting luma TB size is < QuadtreeTULog2MinSize
    log2MinTUSizeInCU = m_pcSlice->getSPS()->getQuadtreeTULog2MinSize();//则当前Cu最小TU为规定的最小Tu块大小
  }
  else
  {
    // when fully making use of signaled TUMaxDepth + inter/intraSplitFlag, resulting luma TB size is still >= QuadtreeTULog2MinSize
    log2MinTUSizeInCU = log2CbSize - ( quadtreeTUMaxDepth - 1 + interSplitFlag + intraSplitFlag); // stop when trafoDepth == hierarchy_depth = splitFlag
    if ( log2MinTUSizeInCU > m_pcSlice->getSPS()->getQuadtreeTULog2MaxSize())//当前Cu能得到最小Tu块大于规定的最大Tu块大小
    {
      // when fully making use of signaled TUMaxDepth + inter/intraSplitFlag, resulting luma TB size is still > QuadtreeTULog2MaxSize
      log2MinTUSizeInCU = m_pcSlice->getSPS()->getQuadtreeTULog2MaxSize();//则当前Cu最小TU为规定的最大Tu块大小
    }
  }
  return log2MinTUSizeInCU;
}

UInt TComDataCU::getCtxSkipFlag( UInt uiAbsPartIdx )
{//左侧和上方PU块所属的CU是否skip的得到SkipFlag的上下文索引
  TComDataCU* pcTempCU;
  UInt        uiTempPartIdx;
  UInt        uiCtx = 0;

  // Get BCBP of left PU
  pcTempCU = getPULeft( uiTempPartIdx, m_absZIdxInCtu + uiAbsPartIdx );
  uiCtx    = ( pcTempCU ) ? pcTempCU->isSkipped( uiTempPartIdx ) : 0;

  // Get BCBP of above PU
  pcTempCU = getPUAbove( uiTempPartIdx, m_absZIdxInCtu + uiAbsPartIdx );
  uiCtx   += ( pcTempCU ) ? pcTempCU->isSkipped( uiTempPartIdx ) : 0;

  return uiCtx;
}

UInt TComDataCU::getCtxInterDir( UInt uiAbsPartIdx )
{//根据位置为uiAbsPartIdx的C的深度得到帧间预测方向的上下文索引
  return getDepth( uiAbsPartIdx );
}


UChar TComDataCU::getQtRootCbf( UInt uiIdx )//uiIdx位置的Cu中是否存在非零变换系数(包括各个有效分量)
{
  const UInt numberValidComponents = getPic()->getNumberValidComponents();
  return getCbf( uiIdx, COMPONENT_Y, 0 )
          || ((numberValidComponents > COMPONENT_Cb) && getCbf( uiIdx, COMPONENT_Cb, 0 ))
          || ((numberValidComponents > COMPONENT_Cr) && getCbf( uiIdx, COMPONENT_Cr, 0 ));
}

Void TComDataCU::setCbfSubParts( const UInt uiCbf[MAX_NUM_COMPONENT], UInt uiAbsPartIdx, UInt uiDepth )//为CTU中深度为uiDepth　位置为uiAbsPartIdx的子块设置cbf（包括各个分量）
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartitionsInCtu() >> (uiDepth << 1);
  for(UInt comp=0; comp<MAX_NUM_COMPONENT; comp++)
  {
    memset( m_puhCbf[comp] + uiAbsPartIdx, uiCbf[comp], sizeof( UChar ) * uiCurrPartNumb );
  }
}

Void TComDataCU::setCbfSubParts( UInt uiCbf, ComponentID compID, UInt uiAbsPartIdx, UInt uiDepth )//为CTU中深度为uiDepth　位置为uiAbsPartIdx　给定的分量的Cb设置cbf
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartitionsInCtu() >> (uiDepth << 1);
  memset( m_puhCbf[compID] + uiAbsPartIdx, uiCbf, sizeof( UChar ) * uiCurrPartNumb );
}

/** Sets a coded block flag for all sub-partitions of a partition
 * \param uiCbf          The value of the coded block flag to be set
 * \param compID
 * \param uiAbsPartIdx
 * \param uiPartIdx
 * \param uiDepth
 */
Void TComDataCU::setCbfSubParts ( UInt uiCbf, ComponentID compID, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth )//为CTU中深度为uiDepth　位置为uiAbsPartIdx的给定的分量的Pb块设置cbf
{
  setSubPart<UChar>( uiCbf, m_puhCbf[compID], uiAbsPartIdx, uiDepth, uiPartIdx );
}

Void TComDataCU::setCbfPartRange ( UInt uiCbf, ComponentID compID, UInt uiAbsPartIdx, UInt uiCoveredPartIdxes )//为CTU中起始位置为uiAbsPartIdx　给定范围　给定分量的块 设置cbf
{
  memset((m_puhCbf[compID] + uiAbsPartIdx), uiCbf, (sizeof(UChar) * uiCoveredPartIdxes));
}

Void TComDataCU::bitwiseOrCbfPartRange( UInt uiCbf, ComponentID compID, UInt uiAbsPartIdx, UInt uiCoveredPartIdxes )//为CTU中起始位置为uiAbsPartIdx　给定范围　给定分量的块 按位与cbf
{
  const UInt stopAbsPartIdx = uiAbsPartIdx + uiCoveredPartIdxes;

  for (UInt subPartIdx = uiAbsPartIdx; subPartIdx < stopAbsPartIdx; subPartIdx++)
  {
    m_puhCbf[compID][subPartIdx] |= uiCbf;
  }
}

Void TComDataCU::setDepthSubParts( UInt uiDepth, UInt uiAbsPartIdx )////为CTU中深度为uiDepth　位置为uiAbsPartIdx的子块设置深度
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartitionsInCtu() >> (uiDepth << 1);//深度为uiDepth的子块含有4*4小块的个数
  memset( m_puhDepth + uiAbsPartIdx, uiDepth, sizeof(UChar)*uiCurrPartNumb );
}

Bool TComDataCU::isFirstAbsZorderIdxInDepth (UInt uiAbsPartIdx, UInt uiDepth)//判断uiAbsPartIdx位置是否为深度为uiDepth的子块的起始地址
{
  UInt uiPartNumb = m_pcPic->getNumPartitionsInCtu() >> (uiDepth << 1);
  return (((m_absZIdxInCtu + uiAbsPartIdx)% uiPartNumb) == 0);
}

Void TComDataCU::setPartSizeSubParts( PartSize eMode, UInt uiAbsPartIdx, UInt uiDepth )//为深度为uiDepth 位置为uiAbsPartIdx的CU设置Pu分割模式
{
  assert( sizeof( *m_pePartSize) == 1 );
  memset( m_pePartSize + uiAbsPartIdx, eMode, m_pcPic->getNumPartitionsInCtu() >> ( 2 * uiDepth ) );
}

Void TComDataCU::setCUTransquantBypassSubParts( Bool flag, UInt uiAbsPartIdx, UInt uiDepth )//为深度为uiDepth 位置为uiAbsPartIdx的CU设置TransquantBypass标志
{
  memset( m_CUTransquantBypass + uiAbsPartIdx, flag, m_pcPic->getNumPartitionsInCtu() >> ( 2 * uiDepth ) );
}

Void TComDataCU::setSkipFlagSubParts( Bool skip, UInt absPartIdx, UInt depth )//为深度为uiDepth 位置为uiAbsPartIdx的CU设置skip标志
{
  assert( sizeof( *m_skipFlag) == 1 );
  memset( m_skipFlag + absPartIdx, skip, m_pcPic->getNumPartitionsInCtu() >> ( 2 * depth ) );
}

Void TComDataCU::setPredModeSubParts( PredMode eMode, UInt uiAbsPartIdx, UInt uiDepth )//为深度为uiDepth 位置为uiAbsPartIdx的CU设置预测模式（帧内／帧间）
{
  assert( sizeof( *m_pePredMode) == 1 );
  memset( m_pePredMode + uiAbsPartIdx, eMode, m_pcPic->getNumPartitionsInCtu() >> ( 2 * uiDepth ) );
}

Void TComDataCU::setChromaQpAdjSubParts( UChar val, Int absPartIdx, Int depth )//为深度为uiDepth 位置为uiAbsPartIdx的CU设置色度分量QP的偏移值
{
  assert( sizeof(*m_ChromaQpAdj) == 1 );
  memset( m_ChromaQpAdj + absPartIdx, val, m_pcPic->getNumPartitionsInCtu() >> ( 2 * depth ) );
}

Void TComDataCU::setQPSubCUs( Int qp, UInt absPartIdx, UInt depth, Bool &foundNonZeroCbf )
{
  UInt currPartNumb = m_pcPic->getNumPartitionsInCtu() >> (depth << 1);//当前CU4*4小块的个数
  UInt currPartNumQ = currPartNumb >> 2;//当前CU的下一深度的子CU4*4小块的个数
  const UInt numValidComp = m_pcPic->getNumberValidComponents();

  if(!foundNonZeroCbf)//如果当前CU不存在非零变换系数
  {
    if(getDepth(absPartIdx) > depth)//存在子CU
    {
      for ( UInt partUnitIdx = 0; partUnitIdx < 4; partUnitIdx++ )//深度遍历四个子CU　为所有子CU设置QP值
      {
        setQPSubCUs( qp, absPartIdx+partUnitIdx*currPartNumQ, depth+1, foundNonZeroCbf );
      }
    }
    else//如果不存在子CU（递归遍历到最深）
    {
      if(getCbf( absPartIdx, COMPONENT_Y ) || (numValidComp>COMPONENT_Cb && getCbf( absPartIdx, COMPONENT_Cb )) || (numValidComp>COMPONENT_Cr && getCbf( absPartIdx, COMPONENT_Cr) ) )//判断是否存在非零的变换系数
      {
        foundNonZeroCbf = true;//设置foundNonZeroCbf标志值
      }
      else//如果当前CU不存在非零变换系数 设置QP
      {
        setQPSubParts(qp, absPartIdx, depth);
      }
    }
  }
}

Void TComDataCU::setQPSubParts( Int qp, UInt uiAbsPartIdx, UInt uiDepth )//为深度为uiDepth 位置为uiAbsPartIdx的子块设置QP值
{
{
  const UInt numPart = m_pcPic->getNumPartitionsInCtu() >> (uiDepth << 1);
  memset(m_phQP+uiAbsPartIdx, qp, numPart);
}

Void TComDataCU::setIntraDirSubParts( const ChannelType channelType, const UInt dir, const UInt absPartIdx, const UInt depth )//为深度为uiDepth 位置为uiAbsPartIdx 通道类型为channelType的Cb设置帧内预测模式
{
  UInt numPart = m_pcPic->getNumPartitionsInCtu() >> (depth << 1);
  memset( m_puhIntraDir[channelType] + absPartIdx, dir,sizeof(UChar)*numPart );
}

template<typename T>//模板函数
Void TComDataCU::setSubPart( T uiParameter, T* puhBaseCtu, UInt uiCUAddr, UInt uiCUDepth, UInt uiPUIdx )//为给定地址　深度　PU索引的PU块设置参数
{//对照CU帧间预测的PU划分模式图很好看懂（位置描述为CTU中4*4小块的z order）
  assert( sizeof(T) == 1 ); // Using memset() works only for types of size 1

  UInt uiCurrPartNumQ = (m_pcPic->getNumPartitionsInCtu() >> (2 * uiCUDepth)) >> 2;//当前深度CU的1/4块的小块数目　
  switch ( m_pePartSize[ uiCUAddr ] )//判断Pu分割模式
  {//对称分割模式下不需要指明PU块索引
    case SIZE_2Nx2N:
      memset( puhBaseCtu + uiCUAddr, uiParameter, 4 * uiCurrPartNumQ );//整块设置参数
      break;
    case SIZE_2NxN:
      memset( puhBaseCtu + uiCUAddr, uiParameter, 2 * uiCurrPartNumQ );//上半块设置参数
      break;
    case SIZE_Nx2N://左侧半块设置参数
      memset( puhBaseCtu + uiCUAddr, uiParameter, uiCurrPartNumQ );
      memset( puhBaseCtu + uiCUAddr + 2 * uiCurrPartNumQ, uiParameter, uiCurrPartNumQ );
      break;
    case SIZE_NxN://左上1/4块设置参数
      memset( puhBaseCtu + uiCUAddr, uiParameter, uiCurrPartNumQ );
      break;
    case SIZE_2NxnU:
      if ( uiPUIdx == 0 )//上半1/4块设置参数
      {
        memset( puhBaseCtu + uiCUAddr, uiParameter, (uiCurrPartNumQ >> 1) );
        memset( puhBaseCtu + uiCUAddr + uiCurrPartNumQ, uiParameter, (uiCurrPartNumQ >> 1) );
      }
      else if ( uiPUIdx == 1 )//下半３/4块设置参数
      {
        memset( puhBaseCtu + uiCUAddr, uiParameter, (uiCurrPartNumQ >> 1) );
        memset( puhBaseCtu + uiCUAddr + uiCurrPartNumQ, uiParameter, ((uiCurrPartNumQ >> 1) + (uiCurrPartNumQ << 1)) );
      }
      else
      {
        assert(0);
      }
      break;
    case SIZE_2NxnD:
      if ( uiPUIdx == 0 )//上半3/4块设置参数
      {
        memset( puhBaseCtu + uiCUAddr, uiParameter, ((uiCurrPartNumQ << 1) + (uiCurrPartNumQ >> 1)) );
        memset( puhBaseCtu + uiCUAddr + (uiCurrPartNumQ << 1) + uiCurrPartNumQ, uiParameter, (uiCurrPartNumQ >> 1) );
      }
      else if ( uiPUIdx == 1 )//下半1/4块设置参数
      {
        memset( puhBaseCtu + uiCUAddr, uiParameter, (uiCurrPartNumQ >> 1) );
        memset( puhBaseCtu + uiCUAddr + uiCurrPartNumQ, uiParameter, (uiCurrPartNumQ >> 1) );
      }
      else
      {
        assert(0);
      }
      break;
    case SIZE_nLx2N:
      if ( uiPUIdx == 0 )//左侧1/4块设置参数
      {
        memset( puhBaseCtu + uiCUAddr, uiParameter, (uiCurrPartNumQ >> 2) );
        memset( puhBaseCtu + uiCUAddr + (uiCurrPartNumQ >> 1), uiParameter, (uiCurrPartNumQ >> 2) );
        memset( puhBaseCtu + uiCUAddr + (uiCurrPartNumQ << 1), uiParameter, (uiCurrPartNumQ >> 2) );
        memset( puhBaseCtu + uiCUAddr + (uiCurrPartNumQ << 1) + (uiCurrPartNumQ >> 1), uiParameter, (uiCurrPartNumQ >> 2) );
      }
      else if ( uiPUIdx == 1 )//右侧３/4块设置参数
      {
        memset( puhBaseCtu + uiCUAddr, uiParameter, (uiCurrPartNumQ >> 2) );
        memset( puhBaseCtu + uiCUAddr + (uiCurrPartNumQ >> 1), uiParameter, (uiCurrPartNumQ + (uiCurrPartNumQ >> 2)) );
        memset( puhBaseCtu + uiCUAddr + (uiCurrPartNumQ << 1), uiParameter, (uiCurrPartNumQ >> 2) );
        memset( puhBaseCtu + uiCUAddr + (uiCurrPartNumQ << 1) + (uiCurrPartNumQ >> 1), uiParameter, (uiCurrPartNumQ + (uiCurrPartNumQ >> 2)) );
      }
      else
      {
        assert(0);
      }
      break;
    case SIZE_nRx2N://右侧1/4块设置参数
      if ( uiPUIdx == 0 )
      {
        memset( puhBaseCtu + uiCUAddr, uiParameter, (uiCurrPartNumQ + (uiCurrPartNumQ >> 2)) );
        memset( puhBaseCtu + uiCUAddr + uiCurrPartNumQ + (uiCurrPartNumQ >> 1), uiParameter, (uiCurrPartNumQ >> 2) );
        memset( puhBaseCtu + uiCUAddr + (uiCurrPartNumQ << 1), uiParameter, (uiCurrPartNumQ + (uiCurrPartNumQ >> 2)) );
        memset( puhBaseCtu + uiCUAddr + (uiCurrPartNumQ << 1) + uiCurrPartNumQ + (uiCurrPartNumQ >> 1), uiParameter, (uiCurrPartNumQ >> 2) );
      }
      else if ( uiPUIdx == 1 )//左侧３/4块设置参数
      {
        memset( puhBaseCtu + uiCUAddr, uiParameter, (uiCurrPartNumQ >> 2) );
        memset( puhBaseCtu + uiCUAddr + (uiCurrPartNumQ >> 1), uiParameter, (uiCurrPartNumQ >> 2) );
        memset( puhBaseCtu + uiCUAddr + (uiCurrPartNumQ << 1), uiParameter, (uiCurrPartNumQ >> 2) );
        memset( puhBaseCtu + uiCUAddr + (uiCurrPartNumQ << 1) + (uiCurrPartNumQ >> 1), uiParameter, (uiCurrPartNumQ >> 2) );
      }
      else
      {
        assert(0);
      }
      break;
    default:
      assert( 0 );
      break;
  }
}
//用模板函数为不同类型的参数对子块赋值
Void TComDataCU::setMergeFlagSubParts ( Bool bMergeFlag, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth )// 为merge标志赋值
{
  setSubPart( bMergeFlag, m_pbMergeFlag, uiAbsPartIdx, uiDepth, uiPartIdx );
}

Void TComDataCU::setMergeIndexSubParts ( UInt uiMergeIndex, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth )//为merge索引赋值（候选列表中最优运动矢量在列表中的位置）
{
  setSubPart<UChar>( uiMergeIndex, m_puhMergeIndex, uiAbsPartIdx, uiDepth, uiPartIdx );
}

Void TComDataCU::setInterDirSubParts( UInt uiDir, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth )//为帧间预测方向
{
  setSubPart<UChar>( uiDir, m_puhInterDir, uiAbsPartIdx, uiDepth, uiPartIdx );
}

Void TComDataCU::setMVPIdxSubParts( Int iMVPIdx, RefPicList eRefPicList, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth )//为参考图像列表中运动矢量预测的索引（在候选列表中的位置）赋值
{
  setSubPart<Char>( iMVPIdx, m_apiMVPIdx[eRefPicList], uiAbsPartIdx, uiDepth, uiPartIdx );
}

Void TComDataCU::setMVPNumSubParts( Int iMVPNum, RefPicList eRefPicList, UInt uiAbsPartIdx, UInt uiPartIdx, UInt uiDepth )//参考图像列表中运动矢量预测可能的个数赋值
{
  setSubPart<Char>( iMVPNum, m_apiMVPNum[eRefPicList], uiAbsPartIdx, uiDepth, uiPartIdx );
}


Void TComDataCU::setTrIdxSubParts( UInt uiTrIdx, UInt uiAbsPartIdx, UInt uiDepth )//为给定位置和深度的子块的变换索引赋值
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartitionsInCtu() >> (uiDepth << 1);

  memset( m_puhTrIdx + uiAbsPartIdx, uiTrIdx, sizeof(UChar)*uiCurrPartNumb );
}

Void TComDataCU::setTransformSkipSubParts( const UInt useTransformSkip[MAX_NUM_COMPONENT], UInt uiAbsPartIdx, UInt uiDepth )//为给定位置和深度的子块的TransformSkip标志赋值(是否跳过变换这一步)
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartitionsInCtu() >> (uiDepth << 1);

  for(UInt i=0; i<MAX_NUM_COMPONENT; i++)
  {
    memset( m_puhTransformSkip[i] + uiAbsPartIdx, useTransformSkip[i], sizeof( UChar ) * uiCurrPartNumb );
  }
}

Void TComDataCU::setTransformSkipSubParts( UInt useTransformSkip, ComponentID compID, UInt uiAbsPartIdx, UInt uiDepth)//为给定位置和深度的子块的useTransformSkip标志赋值（是否使用TransformSkip）
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartitionsInCtu() >> (uiDepth << 1);

  memset( m_puhTransformSkip[compID] + uiAbsPartIdx, useTransformSkip, sizeof( UChar ) * uiCurrPartNumb );
}

Void TComDataCU::setTransformSkipPartRange ( UInt useTransformSkip, ComponentID compID, UInt uiAbsPartIdx, UInt uiCoveredPartIdxes )//为给定位置和范围的4*4小块的useTransformSkip标志赋值
{
  memset((m_puhTransformSkip[compID] + uiAbsPartIdx), useTransformSkip, (sizeof(UChar) * uiCoveredPartIdxes));
}

Void TComDataCU::setCrossComponentPredictionAlphaPartRange( Char alphaValue, ComponentID compID, UInt uiAbsPartIdx, UInt uiCoveredPartIdxes )//为给定位置和范围的4*4小块的CCP的alpha值赋值
{
  memset((m_crossComponentPredictionAlpha[compID] + uiAbsPartIdx), alphaValue, (sizeof(Char) * uiCoveredPartIdxes));
}

Void TComDataCU::setExplicitRdpcmModePartRange ( UInt rdpcmMode, ComponentID compID, UInt uiAbsPartIdx, UInt uiCoveredPartIdxes )//为给定位置和范围的4*4小块的rdpcm模式赋值
{
  memset((m_explicitRdpcmMode[compID] + uiAbsPartIdx), rdpcmMode, (sizeof(UChar) * uiCoveredPartIdxes));
}

Void TComDataCU::setSizeSubParts( UInt uiWidth, UInt uiHeight, UInt uiAbsPartIdx, UInt uiDepth )//为给定位置和深度的子块的宽高赋值
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartitionsInCtu() >> (uiDepth << 1);

  memset( m_puhWidth  + uiAbsPartIdx, uiWidth,  sizeof(UChar)*uiCurrPartNumb );
  memset( m_puhHeight + uiAbsPartIdx, uiHeight, sizeof(UChar)*uiCurrPartNumb );
}

UChar TComDataCU::getNumPartitions(const UInt uiAbsPartIdx)//得到给定位置的CU块分割的PU块的个数
{
  UChar iNumPart = 0;

  switch ( m_pePartSize[uiAbsPartIdx] )
  {
    case SIZE_2Nx2N:    iNumPart = 1; break;
    case SIZE_2NxN:     iNumPart = 2; break;
    case SIZE_Nx2N:     iNumPart = 2; break;
    case SIZE_NxN:      iNumPart = 4; break;
    case SIZE_2NxnU:    iNumPart = 2; break;
    case SIZE_2NxnD:    iNumPart = 2; break;
    case SIZE_nLx2N:    iNumPart = 2; break;
    case SIZE_nRx2N:    iNumPart = 2; break;
    default:            assert (0);   break;
  }

  return  iNumPart;
}

// This is for use by a leaf/sub CU object only, with no additional AbsPartIdx
Void TComDataCU::getPartIndexAndSize( UInt uiPartIdx, UInt& ruiPartAddr, Int& riWidth, Int& riHeight )
{//uiPartIdx为当前CU不同PU块的划分对应PU块的索引　ruiPartAddr为索引对应PU块起始４＊４小块在CU块中的偏移量（Z形递归扫描　４＊４小块为基本扫描单位）
  switch ( m_pePartSize[0] )//CU分割成PU共有８种分法
  {//m_uiNumPartition为当前CU可分为４＊４最小块的个数　//对照CU的PU分割模式图易懂　不在详述
    case SIZE_2NxN:
      riWidth = getWidth(0);      riHeight = getHeight(0) >> 1; ruiPartAddr = ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 1;
      break;
    case SIZE_Nx2N:
      riWidth = getWidth(0) >> 1; riHeight = getHeight(0);      ruiPartAddr = ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 2;
      break;
    case SIZE_NxN:
      riWidth = getWidth(0) >> 1; riHeight = getHeight(0) >> 1; ruiPartAddr = ( m_uiNumPartition >> 2 ) * uiPartIdx;
      break;
    case SIZE_2NxnU:
      riWidth     = getWidth(0);
      riHeight    = ( uiPartIdx == 0 ) ?  getHeight(0) >> 2 : ( getHeight(0) >> 2 ) + ( getHeight(0) >> 1 );
      ruiPartAddr = ( uiPartIdx == 0 ) ? 0 : m_uiNumPartition >> 3;
      break;
    case SIZE_2NxnD:
      riWidth     = getWidth(0);
      riHeight    = ( uiPartIdx == 0 ) ?  ( getHeight(0) >> 2 ) + ( getHeight(0) >> 1 ) : getHeight(0) >> 2;
      ruiPartAddr = ( uiPartIdx == 0 ) ? 0 : (m_uiNumPartition >> 1) + (m_uiNumPartition >> 3);
      break;
    case SIZE_nLx2N:
      riWidth     = ( uiPartIdx == 0 ) ? getWidth(0) >> 2 : ( getWidth(0) >> 2 ) + ( getWidth(0) >> 1 );
      riHeight    = getHeight(0);
      ruiPartAddr = ( uiPartIdx == 0 ) ? 0 : m_uiNumPartition >> 4;
      break;
    case SIZE_nRx2N:
      riWidth     = ( uiPartIdx == 0 ) ? ( getWidth(0) >> 2 ) + ( getWidth(0) >> 1 ) : getWidth(0) >> 2;
      riHeight    = getHeight(0);
      ruiPartAddr = ( uiPartIdx == 0 ) ? 0 : (m_uiNumPartition >> 2) + (m_uiNumPartition >> 4);
      break;
    default:
      assert ( m_pePartSize[0] == SIZE_2Nx2N );
      riWidth = getWidth(0);      riHeight = getHeight(0);      ruiPartAddr = 0;
      break;
  }
}


Void TComDataCU::getMvField ( TComDataCU* pcCU, UInt uiAbsPartIdx, RefPicList eRefPicList, TComMvField& rcMvField )//得到给定位置(PU)的运动矢量信息
{
  if ( pcCU == NULL )  // OUT OF BOUNDARY//为空（超出能获取的返回）　运动矢量信息置为无效
  {
    TComMv  cZeroMv;
    rcMvField.setMvField( cZeroMv, NOT_VALID );
    return;
  }

  TComCUMvField*  pcCUMvField = pcCU->getCUMvField( eRefPicList );//该CU在参考图像列表eRefPicList中的运动矢量信息
  rcMvField.setMvField( pcCUMvField->getMv( uiAbsPartIdx ), pcCUMvField->getRefIdx( uiAbsPartIdx ) );//为uiAbsPartIdx位置设置运动矢量信息
}

Void TComDataCU::deriveLeftRightTopIdxGeneral ( UInt uiAbsPartIdx, UInt uiPartIdx, UInt& ruiPartIdxLT, UInt& ruiPartIdxRT )//得到给定位置pU块内左上角和右上角的位置
{//uiAbsPartIdx为CU中Pu块的起始位置
  ruiPartIdxLT = m_absZIdxInCtu + uiAbsPartIdx;//PU块左上角位置
  UInt uiPUWidth = 0;

  switch ( m_pePartSize[uiAbsPartIdx] )//判断PU分割模式
  {//得到不同模式下对应uiPartIdx索引的PU块的宽
    case SIZE_2Nx2N: uiPUWidth = m_puhWidth[uiAbsPartIdx];  break;
    case SIZE_2NxN:  uiPUWidth = m_puhWidth[uiAbsPartIdx];   break;
    case SIZE_Nx2N:  uiPUWidth = m_puhWidth[uiAbsPartIdx]  >> 1;  break;
    case SIZE_NxN:   uiPUWidth = m_puhWidth[uiAbsPartIdx]  >> 1; break;
    case SIZE_2NxnU:   uiPUWidth = m_puhWidth[uiAbsPartIdx]; break;
    case SIZE_2NxnD:   uiPUWidth = m_puhWidth[uiAbsPartIdx]; break;
    case SIZE_nLx2N:
      if ( uiPartIdx == 0 )
      {
        uiPUWidth = m_puhWidth[uiAbsPartIdx]  >> 2;
      }
      else if ( uiPartIdx == 1 )
      {
        uiPUWidth = (m_puhWidth[uiAbsPartIdx]  >> 1) + (m_puhWidth[uiAbsPartIdx]  >> 2);
      }
      else
      {
        assert(0);
      }
      break;
    case SIZE_nRx2N:
      if ( uiPartIdx == 0 )
      {
        uiPUWidth = (m_puhWidth[uiAbsPartIdx]  >> 1) + (m_puhWidth[uiAbsPartIdx]  >> 2);
      }
      else if ( uiPartIdx == 1 )
      {
        uiPUWidth = m_puhWidth[uiAbsPartIdx]  >> 2;
      }
      else
      {
        assert(0);
      }
      break;
    default:
      assert (0);
      break;
  }

  ruiPartIdxRT = g_auiRasterToZscan [g_auiZscanToRaster[ ruiPartIdxLT ] + uiPUWidth / m_pcPic->getMinCUWidth() - 1 ];//PU块右上角位置
}

Void TComDataCU::deriveLeftBottomIdxGeneral( UInt uiAbsPartIdx, UInt uiPartIdx, UInt& ruiPartIdxLB )//得到给定位置pU块内左下角位置
{
  UInt uiPUHeight = 0;
  switch ( m_pePartSize[uiAbsPartIdx] )//判断PU分割模式
  {//得到不同模式下对应uiPartIdx索引的PU块的高
    case SIZE_2Nx2N: uiPUHeight = m_puhHeight[uiAbsPartIdx];    break;
    case SIZE_2NxN:  uiPUHeight = m_puhHeight[uiAbsPartIdx] >> 1;    break;
    case SIZE_Nx2N:  uiPUHeight = m_puhHeight[uiAbsPartIdx];  break;
    case SIZE_NxN:   uiPUHeight = m_puhHeight[uiAbsPartIdx] >> 1;    break;
    case SIZE_2NxnU:
      if ( uiPartIdx == 0 )
      {
        uiPUHeight = m_puhHeight[uiAbsPartIdx] >> 2;
      }
      else if ( uiPartIdx == 1 )
      {
        uiPUHeight = (m_puhHeight[uiAbsPartIdx] >> 1) + (m_puhHeight[uiAbsPartIdx] >> 2);
      }
      else
      {
        assert(0);
      }
      break;
    case SIZE_2NxnD:
      if ( uiPartIdx == 0 )
      {
        uiPUHeight = (m_puhHeight[uiAbsPartIdx] >> 1) + (m_puhHeight[uiAbsPartIdx] >> 2);
      }
      else if ( uiPartIdx == 1 )
      {
        uiPUHeight = m_puhHeight[uiAbsPartIdx] >> 2;
      }
      else
      {
        assert(0);
      }
      break;
    case SIZE_nLx2N: uiPUHeight = m_puhHeight[uiAbsPartIdx];  break;
    case SIZE_nRx2N: uiPUHeight = m_puhHeight[uiAbsPartIdx];  break;
    default:
      assert (0);
      break;
  }

  ruiPartIdxLB = g_auiRasterToZscan [g_auiZscanToRaster[ m_absZIdxInCtu + uiAbsPartIdx ] + ((uiPUHeight / m_pcPic->getMinCUHeight()) - 1)*m_pcPic->getNumPartInCtuWidth()];//PU块左下角位置
}

Void TComDataCU::deriveLeftRightTopIdx ( UInt uiPartIdx, UInt& ruiPartIdxLT, UInt& ruiPartIdxRT )//得到给定位置pU块内左上和右上角位置（只用指出PU块的索引　而不用指出具体起始位置）
{
  ruiPartIdxLT = m_absZIdxInCtu;//CU块内左上角位置
  ruiPartIdxRT = g_auiRasterToZscan [g_auiZscanToRaster[ ruiPartIdxLT ] + m_puhWidth[0] / m_pcPic->getMinCUWidth() - 1 ];//CU块内由上角位置

  switch ( m_pePartSize[0] )//判断PU分割模式
  {//索引为uiPartIdx的PU块左上和右上角位置相对于CU左上和右上角位置移动（对照PU分割模式图　很好理解）
    case SIZE_2Nx2N:      break;//2Nx2N下　CU块同PU块　故左上和右上角位置相同
    case SIZE_2NxN://2NxN下　如果为CU中上半块PU则左上和右上角位置不变　若为CU中上半块PU则左上和右上角位置需下移CU高的一半
      ruiPartIdxLT += ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 1; ruiPartIdxRT += ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 1;
      break;
    case SIZE_Nx2N://Nx２N下　如果为CU中左侧半块PU则左上位置不变右上角位置左移CU宽的一半　若为CU中右侧半块PU则左上右移CU宽的一半右上角位置不变
      ruiPartIdxLT += ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 2; ruiPartIdxRT -= ( uiPartIdx == 1 )? 0 : m_uiNumPartition >> 2;
      break;
    case SIZE_NxN://NxN下　索引为uiPartIdx的PU块左上角相对CU块的左上角的偏移量为( m_uiNumPartition >> 2 ) * uiPartIdx　右上角偏移量为( m_uiNumPartition >> 2 ) * ( uiPartIdx - 1 )
      ruiPartIdxLT += ( m_uiNumPartition >> 2 ) * uiPartIdx;         ruiPartIdxRT +=  ( m_uiNumPartition >> 2 ) * ( uiPartIdx - 1 );
      break;
    case SIZE_2NxnU://2NxnU下 如果为CU中上半块PU则左上和右上角位置不变　若为CU中上半块PU则左上和右上角位置需下移CU高的1/4
      ruiPartIdxLT += ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 3;
      ruiPartIdxRT += ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 3;
      break;
    case SIZE_2NxnD://2NxnD下　如果为CU中上半块PU则左上和右上角位置不变　若为CU中上半块PU则左上和右上角位置需下移CU高的3/4
      ruiPartIdxLT += ( uiPartIdx == 0 )? 0 : ( m_uiNumPartition >> 1 ) + ( m_uiNumPartition >> 3 );
      ruiPartIdxRT += ( uiPartIdx == 0 )? 0 : ( m_uiNumPartition >> 1 ) + ( m_uiNumPartition >> 3 );
      break;
    case SIZE_nLx2N://nLx2N下　如果为CU中左侧半块PU则左上位置不变右上角位置左移CU宽的3/4　若为CU中右侧半块PU则左上右移CU宽的1/4右上角位置不变
      ruiPartIdxLT += ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 4;
      ruiPartIdxRT -= ( uiPartIdx == 1 )? 0 : ( m_uiNumPartition >> 2 ) + ( m_uiNumPartition >> 4 );
      break;
    case SIZE_nRx2N://nRx2N下　如果为CU中左侧半块PU则左上位置不变右上角位置左移CU宽的1/4　若为CU中右侧半块PU则左上右移CU宽的3/4右上角位置不变
      ruiPartIdxLT += ( uiPartIdx == 0 )? 0 : ( m_uiNumPartition >> 2 ) + ( m_uiNumPartition >> 4 );
      ruiPartIdxRT -= ( uiPartIdx == 1 )? 0 : m_uiNumPartition >> 4;
      break;
    default:
      assert (0);
      break;
  }

}

Void TComDataCU::deriveLeftBottomIdx( UInt  uiPartIdx,      UInt&      ruiPartIdxLB )
{//同上分析　不再叙述　（对照PU分割模式图　很好理解）
  ruiPartIdxLB      = g_auiRasterToZscan [g_auiZscanToRaster[ m_absZIdxInCtu ] + ( ((m_puhHeight[0] / m_pcPic->getMinCUHeight())>>1) - 1)*m_pcPic->getNumPartInCtuWidth()];

  switch ( m_pePartSize[0] )
  {
    case SIZE_2Nx2N:
      ruiPartIdxLB += m_uiNumPartition >> 1;
      break;
    case SIZE_2NxN:
      ruiPartIdxLB += ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 1;
      break;
    case SIZE_Nx2N:
      ruiPartIdxLB += ( uiPartIdx == 0 )? m_uiNumPartition >> 1 : (m_uiNumPartition >> 2)*3;
      break;
    case SIZE_NxN:
      ruiPartIdxLB += ( m_uiNumPartition >> 2 ) * uiPartIdx;
      break;
    case SIZE_2NxnU:
      ruiPartIdxLB += ( uiPartIdx == 0 ) ? -((Int)m_uiNumPartition >> 3) : m_uiNumPartition >> 1;
      break;
    case SIZE_2NxnD:
      ruiPartIdxLB += ( uiPartIdx == 0 ) ? (m_uiNumPartition >> 2) + (m_uiNumPartition >> 3): m_uiNumPartition >> 1;
      break;
    case SIZE_nLx2N:
      ruiPartIdxLB += ( uiPartIdx == 0 ) ? m_uiNumPartition >> 1 : (m_uiNumPartition >> 1) + (m_uiNumPartition >> 4);
      break;
    case SIZE_nRx2N:
      ruiPartIdxLB += ( uiPartIdx == 0 ) ? m_uiNumPartition >> 1 : (m_uiNumPartition >> 1) + (m_uiNumPartition >> 2) + (m_uiNumPartition >> 4);
      break;
    default:
      assert (0);
      break;
  }
}

/** Derive the partition index of neighbouring bottom right block
 * \param [in]  uiPartIdx     current partition index
 * \param [out] ruiPartIdxRB  partition index of neighbouring bottom right block
 */
Void TComDataCU::deriveRightBottomIdx( UInt uiPartIdx, UInt &ruiPartIdxRB )
{//同上分析　不再叙述　（对照PU分割模式图　很好理解）
  ruiPartIdxRB      = g_auiRasterToZscan [g_auiZscanToRaster[ m_absZIdxInCtu ] + ( ((m_puhHeight[0] / m_pcPic->getMinCUHeight())>>1) - 1)*m_pcPic->getNumPartInCtuWidth() +  m_puhWidth[0] / m_pcPic->getMinCUWidth() - 1];

  switch ( m_pePartSize[0] )
  {
    case SIZE_2Nx2N:
      ruiPartIdxRB += m_uiNumPartition >> 1;
      break;
    case SIZE_2NxN:
      ruiPartIdxRB += ( uiPartIdx == 0 )? 0 : m_uiNumPartition >> 1;
      break;
    case SIZE_Nx2N:
      ruiPartIdxRB += ( uiPartIdx == 0 )? m_uiNumPartition >> 2 : (m_uiNumPartition >> 1);
      break;
    case SIZE_NxN:
      ruiPartIdxRB += ( m_uiNumPartition >> 2 ) * ( uiPartIdx - 1 );
      break;
    case SIZE_2NxnU:
      ruiPartIdxRB += ( uiPartIdx == 0 ) ? -((Int)m_uiNumPartition >> 3) : m_uiNumPartition >> 1;
      break;
    case SIZE_2NxnD:
      ruiPartIdxRB += ( uiPartIdx == 0 ) ? (m_uiNumPartition >> 2) + (m_uiNumPartition >> 3): m_uiNumPartition >> 1;
      break;
    case SIZE_nLx2N:
      ruiPartIdxRB += ( uiPartIdx == 0 ) ? (m_uiNumPartition >> 3) + (m_uiNumPartition >> 4): m_uiNumPartition >> 1;
      break;
    case SIZE_nRx2N:
      ruiPartIdxRB += ( uiPartIdx == 0 ) ? (m_uiNumPartition >> 2) + (m_uiNumPartition >> 3) + (m_uiNumPartition >> 4) : m_uiNumPartition >> 1;
      break;
    default:
      assert (0);
      break;
  }
}

Bool TComDataCU::hasEqualMotion( UInt uiAbsPartIdx, TComDataCU* pcCandCU, UInt uiCandAbsPartIdx )//判断两处的运动矢量是否相同
{
  if ( getInterDir( uiAbsPartIdx ) != pcCandCU->getInterDir( uiCandAbsPartIdx ) )//帧间预测方向不同则运动矢量不可能相同
  {
    return false;
  }

  for ( UInt uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )//只有不同参考图像列表中参考图像索引和运动矢量值都相同　才认为这两处运动矢量相同
  {
    if ( getInterDir( uiAbsPartIdx ) & ( 1 << uiRefListIdx ) )
    {
      if ( getCUMvField( RefPicList( uiRefListIdx ) )->getMv( uiAbsPartIdx )     != pcCandCU->getCUMvField( RefPicList( uiRefListIdx ) )->getMv( uiCandAbsPartIdx ) ||
        getCUMvField( RefPicList( uiRefListIdx ) )->getRefIdx( uiAbsPartIdx ) != pcCandCU->getCUMvField( RefPicList( uiRefListIdx ) )->getRefIdx( uiCandAbsPartIdx ) )
      {
        return false;
      }
    }
  }

  return true;
}

//! Construct a list of merging candidates
//利用空域或时域上相邻块的MV对当前块MV进行预测，仅对预测残差编码　则能大幅节省MV的编码比特数
Void TComDataCU::getInterMergeCandidates( UInt uiAbsPartIdx, UInt uiPUIdx, TComMvField* pcMvFieldNeighbours, UChar* puhInterDirNeighbours, Int& numValidMergeCand, Int mrgCandIdx )//mrgCandIdx限制候选MV能达到的索引
{//空域最多提供4个候选MV　时域最多提供１个候选MV 
  UInt uiAbsPartAddr = m_absZIdxInCtu + uiAbsPartIdx;
  Bool abCandIsInter[ MRG_MAX_NUM_CANDS ];
  for( UInt ui = 0; ui < getSlice()->getMaxNumMergeCand(); ++ui )//为需要用到的参数初始化（附上未处理的值）
  {
    abCandIsInter[ui] = false;
    pcMvFieldNeighbours[ ( ui << 1 )     ].setRefIdx(NOT_VALID);//最大候选MV个数的两倍　表示双向预测时对应参考图像列表0和参考图像列表1的运动矢量
    pcMvFieldNeighbours[ ( ui << 1 ) + 1 ].setRefIdx(NOT_VALID);
  }
  numValidMergeCand = getSlice()->getMaxNumMergeCand();//有效候选MV个数为规定的最大候选MV个数
  // compute the location of the current PU
  Int xP, yP, nPSW, nPSH;
  this->getPartPosition(uiPUIdx, xP, yP, nPSW, nPSH);//当前PU的位置（x y坐标　宽　高）

  Int iCount = 0;

  UInt uiPartIdxLT, uiPartIdxRT, uiPartIdxLB;
  PartSize cCurPS = getPartitionSize( uiAbsPartIdx );//当前CU的PU分割模式
  deriveLeftRightTopIdxGeneral( uiAbsPartIdx, uiPUIdx, uiPartIdxLT, uiPartIdxRT );//得到当前PU左上角和右上角位置
  deriveLeftBottomIdxGeneral( uiAbsPartIdx, uiPUIdx, uiPartIdxLB );//得到当前PU左下位置

  //left
  UInt uiLeftPartIdx = 0;
  TComDataCU* pcCULeft = 0;
  pcCULeft = getPULeft( uiLeftPartIdx, uiPartIdxLB );//当前左侧PU最下方的PU(记为A1　下文中用A1表示)

  Bool isAvailableA1 = pcCULeft &&//当前左侧最PU下方的PU可作为候选的条件为：当前左侧最PU下方的PU可获得即不为空
                       pcCULeft->isDiffMER(xP -1, yP+nPSH-1, xP, yP) &&//是否来自不同MER　由于并行处理　只有与当前PU来自不同的MER的PU才能得到其信息　
                       !( uiPUIdx == 1 && (cCurPS == SIZE_Nx2N || cCurPS == SIZE_nLx2N || cCurPS == SIZE_nRx2N) ) &&//当分割模式为Nx2N或nLx2N或SIZE_nRx2N且PU为右侧PU时候选列表中不能存在当前左侧PU最下方的PU的运动信息　
                       pcCULeft->isInter( uiLeftPartIdx ) ;//为帧间预测（MV就是针对帧间预测而言）                         //因为一旦使用会使左右两侧的PU运动信息一致　这与２Nx2N的分割模式无异

  if ( isAvailableA1 )//若可通过当前左侧PU最下方的PU得到候选列表中的候选MV
  {
    abCandIsInter[iCount] = true;//对应候选列表处置为真
    // get Inter Dir
    puhInterDirNeighbours[iCount] = pcCULeft->getInterDir( uiLeftPartIdx );//得到A1的帧间预测方向
    // get Mv from Left
    pcCULeft->getMvField( pcCULeft, uiLeftPartIdx, REF_PIC_LIST_0, pcMvFieldNeighbours[iCount<<1] );//将A1的在参考图像列表0中运动信息设置到pcMvFieldNeighbours数组中的iCount<<1位置
    if ( getSlice()->isInterB() )//若为双向预测 以slice为单位（需要两个参考图像列表中的运动矢量信息）
    {
      pcCULeft->getMvField( pcCULeft, uiLeftPartIdx, REF_PIC_LIST_1, pcMvFieldNeighbours[(iCount<<1)+1] );//则还需将A1的在参考图像列表１中运动信息设置到pcMvFieldNeighbours数组中的(iCount<<1＋1)位置
    }
    if ( mrgCandIdx == iCount )//候选MV的个数达到指定的候选MV的索引　则结束方法
    {
      return;
    }
    iCount ++;//下一个候选MV的索引
  }

  // early termination
  if (iCount == getSlice()->getMaxNumMergeCand())//候选列表数已达到最大　则结束方法(icount是从０开始的)
  {
    return;
  }
  // above
  UInt uiAbovePartIdx = 0;
  TComDataCU* pcCUAbove = 0;
  pcCUAbove = getPUAbove( uiAbovePartIdx, uiPartIdxRT );//当前PU上方最右侧当PU(记为B1)
  //获取候选MV的过程同上　不在叙述
  Bool isAvailableB1 = pcCUAbove &&
                       pcCUAbove->isDiffMER(xP+nPSW-1, yP-1, xP, yP) &&
                       !( uiPUIdx == 1 && (cCurPS == SIZE_2NxN || cCurPS == SIZE_2NxnU || cCurPS == SIZE_2NxnD) ) &&
                       pcCUAbove->isInter( uiAbovePartIdx );

  if ( isAvailableB1 && (!isAvailableA1 || !pcCULeft->hasEqualMotion( uiLeftPartIdx, pcCUAbove, uiAbovePartIdx ) ) )//确保候选列表中不存在相同的运动矢量
  {
    abCandIsInter[iCount] = true;
    // get Inter Dir
    puhInterDirNeighbours[iCount] = pcCUAbove->getInterDir( uiAbovePartIdx );
    // get Mv from Left
    pcCUAbove->getMvField( pcCUAbove, uiAbovePartIdx, REF_PIC_LIST_0, pcMvFieldNeighbours[iCount<<1] );
    if ( getSlice()->isInterB() )
    {
      pcCUAbove->getMvField( pcCUAbove, uiAbovePartIdx, REF_PIC_LIST_1, pcMvFieldNeighbours[(iCount<<1)+1] );
    }
    if ( mrgCandIdx == iCount )
    {
      return;
    }
    iCount ++;
  }
  // early termination
  if (iCount == getSlice()->getMaxNumMergeCand())
  {
    return;
  }

  // above right
  UInt uiAboveRightPartIdx = 0;
  TComDataCU* pcCUAboveRight = 0;
  pcCUAboveRight = getPUAboveRight( uiAboveRightPartIdx, uiPartIdxRT );//当前PU右上方距离最近的PU(记为B0)
  //获取候选MV的过程同上  不在叙述
  Bool isAvailableB0 = pcCUAboveRight &&
                       pcCUAboveRight->isDiffMER(xP+nPSW, yP-1, xP, yP) &&//是否来自不同MER　由于并行处理　只有与当前PU来自不同的MER的PU才能得到其信息　
                       pcCUAboveRight->isInter( uiAboveRightPartIdx );//不再需要对特定的分割模式做判断　因为不再会出现两PU块因运动信息相同而融合成一种PU块的情况

  if ( isAvailableB0 && ( !isAvailableB1 || !pcCUAbove->hasEqualMotion( uiAbovePartIdx, pcCUAboveRight, uiAboveRightPartIdx ) ) )//确保候选列表中不存在相同的运动矢量
  {
    abCandIsInter[iCount] = true;
    // get Inter Dir
    puhInterDirNeighbours[iCount] = pcCUAboveRight->getInterDir( uiAboveRightPartIdx );
    // get Mv from Left
    pcCUAboveRight->getMvField( pcCUAboveRight, uiAboveRightPartIdx, REF_PIC_LIST_0, pcMvFieldNeighbours[iCount<<1] );
    if ( getSlice()->isInterB() )
    {
      pcCUAboveRight->getMvField( pcCUAboveRight, uiAboveRightPartIdx, REF_PIC_LIST_1, pcMvFieldNeighbours[(iCount<<1)+1] );
    }
    if ( mrgCandIdx == iCount )
    {
      return;
    }
    iCount ++;
  }
  // early termination
  if (iCount == getSlice()->getMaxNumMergeCand())
  {
    return;
  }

  //left bottom
  UInt uiLeftBottomPartIdx = 0;
  TComDataCU* pcCULeftBottom = 0;
  pcCULeftBottom = this->getPUBelowLeft( uiLeftBottomPartIdx, uiPartIdxLB );//当前PU左下方距离最近的PU(记为A0)
  //获取候选MV的过程同上  不在叙述
  Bool isAvailableA0 = pcCULeftBottom &&
                       pcCULeftBottom->isDiffMER(xP-1, yP+nPSH, xP, yP) &&
                       pcCULeftBottom->isInter( uiLeftBottomPartIdx ) ;

  if ( isAvailableA0 && ( !isAvailableA1 || !pcCULeft->hasEqualMotion( uiLeftPartIdx, pcCULeftBottom, uiLeftBottomPartIdx ) ) )//确保候选列表中不存在相同的运动矢量
  {
    abCandIsInter[iCount] = true;
    // get Inter Dir
    puhInterDirNeighbours[iCount] = pcCULeftBottom->getInterDir( uiLeftBottomPartIdx );
    // get Mv from Left
    pcCULeftBottom->getMvField( pcCULeftBottom, uiLeftBottomPartIdx, REF_PIC_LIST_0, pcMvFieldNeighbours[iCount<<1] );
    if ( getSlice()->isInterB() )
    {
      pcCULeftBottom->getMvField( pcCULeftBottom, uiLeftBottomPartIdx, REF_PIC_LIST_1, pcMvFieldNeighbours[(iCount<<1)+1] );
    }
    if ( mrgCandIdx == iCount )
    {
      return;
    }
    iCount ++;
  }
  // early termination
  if (iCount == getSlice()->getMaxNumMergeCand())
  {
    return;
  }

  // above left
  if( iCount < 4 )//如果候选MV少于４个　则需要当前pU左上方距离最近的pU的运动信息
  {
    UInt uiAboveLeftPartIdx = 0;
    TComDataCU* pcCUAboveLeft = 0;
    pcCUAboveLeft = getPUAboveLeft( uiAboveLeftPartIdx, uiAbsPartAddr );//当前pU左上方距离最近的pU（记为B2）
    //获取候选MV的过程同上  不在叙述
    Bool isAvailableB2 = pcCUAboveLeft &&
                         pcCUAboveLeft->isDiffMER(xP-1, yP-1, xP, yP) &&
                         pcCUAboveLeft->isInter( uiAboveLeftPartIdx );

    if ( isAvailableB2 && ( !isAvailableA1 || !pcCULeft->hasEqualMotion( uiLeftPartIdx, pcCUAboveLeft, uiAboveLeftPartIdx ) )//确保候选列表中不存在相同的运动矢量
        && ( !isAvailableB1 || !pcCUAbove->hasEqualMotion( uiAbovePartIdx, pcCUAboveLeft, uiAboveLeftPartIdx ) ) )
    {
      abCandIsInter[iCount] = true;
      // get Inter Dir
      puhInterDirNeighbours[iCount] = pcCUAboveLeft->getInterDir( uiAboveLeftPartIdx );
      // get Mv from Left
      pcCUAboveLeft->getMvField( pcCUAboveLeft, uiAboveLeftPartIdx, REF_PIC_LIST_0, pcMvFieldNeighbours[iCount<<1] );
      if ( getSlice()->isInterB() )
      {
        pcCUAboveLeft->getMvField( pcCUAboveLeft, uiAboveLeftPartIdx, REF_PIC_LIST_1, pcMvFieldNeighbours[(iCount<<1)+1] );
      }
      if ( mrgCandIdx == iCount )
      {
        return;
      }
      iCount ++;
    }
  }
  // early termination
  if (iCount == getSlice()->getMaxNumMergeCand())
  {
    return;
  }

  if ( getSlice()->getEnableTMVPFlag() )//如果使用时域MV预测技术
  {
    //>> MTK colocated-RightBottom
    UInt uiPartIdxRB;//当前PU内右下角位置

    deriveRightBottomIdx( uiPUIdx, uiPartIdxRB );//得到当前PU内右下角位置

    UInt uiAbsPartIdxTmp = g_auiZscanToRaster[uiPartIdxRB];
    const UInt numPartInCtuWidth  = m_pcPic->getNumPartInCtuWidth();//图像的宽(以CTU为单位)
    const UInt numPartInCtuHeight = m_pcPic->getNumPartInCtuHeight();//图像的高(以CTU为单位)

    TComMv cColMv;
    Int iRefIdx;
    Int ctuRsAddr = -1;//初始化同位pu所在CTu的位置　-1表示同位Pu不可获得
    //时域上的首先同位PU为邻近已编码图像对应位置PU右下角PU（同位Pu的起始位置即左上角位置记为C0）
    if (   ( ( m_pcPic->getCtu(m_ctuRsAddr)->getCUPelX() + g_auiRasterToPelX[uiAbsPartIdxTmp] + m_pcPic->getMinCUWidth () ) < m_pcSlice->getSPS()->getPicWidthInLumaSamples () )  // image boundary check
        && ( ( m_pcPic->getCtu(m_ctuRsAddr)->getCUPelY() + g_auiRasterToPelY[uiAbsPartIdxTmp] + m_pcPic->getMinCUHeight() ) < m_pcSlice->getSPS()->getPicHeightInLumaSamples() ) )//在图像内且不与图像边界相邻
    {
      if ( ( uiAbsPartIdxTmp % numPartInCtuWidth < numPartInCtuWidth - 1 ) &&           // is not at the last column of CTU
        ( uiAbsPartIdxTmp / numPartInCtuWidth < numPartInCtuHeight - 1 ) ) //不是CTU的最后一列也不是最后一行  // is not at the last row    of CTU
      {
        uiAbsPartAddr = g_auiRasterToZscan[ uiAbsPartIdxTmp + numPartInCtuWidth + 1 ];//同位PU在当前CTU中的位置　时域上的同位PU为邻近已编码图像对应位置PU右下角PU
        ctuRsAddr = getCtuRsAddr();//不为CTU的最后一行和最后一列保证右下角PU还在当前CTU中
      }
      else if ( uiAbsPartIdxTmp % numPartInCtuWidth < numPartInCtuWidth - 1 ) //CTU的最后一行但不是最后一列  // is not at the last column of CTU But is last row of CTU
      {
        uiAbsPartAddr = g_auiRasterToZscan[ (uiAbsPartIdxTmp + numPartInCtuWidth + 1) % m_pcPic->getNumPartitionsInCtu() ];//同位Pu在当前CTU下方CTu中的位置
      }
      else if ( uiAbsPartIdxTmp / numPartInCtuWidth < numPartInCtuHeight - 1 )//CTU的最后一列但不是最后一行        // is not at the last row of CTU But is last column of CTU
      {
        uiAbsPartAddr = g_auiRasterToZscan[ uiAbsPartIdxTmp + 1 ];//同位Pu在当前CTU右侧CTu中的位置
        ctuRsAddr = getCtuRsAddr() + 1;//CTU的最后一列但不是最后一行 所以右下角PU当在前CTU右侧CTu中
      }
      else //is the right bottom corner of CTU//为当前CTu的右下角
      {
        uiAbsPartAddr = 0;//同位PU即当前位置右下角PU的位置为右下相邻CTU的起始位置　所以uiAbsPartAddr为０
      }
    }
    //当同位PU所属的CTU大于当前pu所属CTU的行（级在当前CTu的下方）时认为同位pU不可获得
    //这是因为保存相邻下方的CTu所需的记忆带宽远大于保存相邻右侧的CTU
    iRefIdx = 0;

    Bool bExistMV = false;
    UInt uiPartIdxCenter;
    Int dir = 0;
    UInt uiArrayAddr = iCount;
    xDeriveCenterIdx( uiPUIdx, uiPartIdxCenter );//当前pU的中间位置为　将当前PU十字4等分后　第４块的起始位置　为次选同位Pu(记为C1)
    bExistMV = ctuRsAddr >= 0 && xGetColMVP( REF_PIC_LIST_0, ctuRsAddr, uiAbsPartAddr, cColMv, iRefIdx );//尝试获取C0位置的运动矢量信息
    if( bExistMV == false )//若CO处位置无效或运动信息无法获得
    {
      bExistMV = xGetColMVP( REF_PIC_LIST_0, getCtuRsAddr(), uiPartIdxCenter,  cColMv, iRefIdx );//则从C1位置得到运动矢量信息
    }
    if( bExistMV )//若可得到同位Pu的运动矢量信息
    {
      dir |= 1;
      pcMvFieldNeighbours[ 2 * uiArrayAddr ].setMvField( cColMv, iRefIdx );//将此运动矢量设为对应位置的候选运动矢量
    }

    if ( getSlice()->isInterB() )//若为帧间双向预测　则还需获取同位块在参考图像列表1中的运动矢量信息
    {
      bExistMV = ctuRsAddr >= 0 && xGetColMVP( REF_PIC_LIST_1, ctuRsAddr, uiAbsPartAddr, cColMv, iRefIdx);
      if( bExistMV == false )
      {
        bExistMV = xGetColMVP( REF_PIC_LIST_1, getCtuRsAddr(), uiPartIdxCenter, cColMv, iRefIdx );
      }
      if( bExistMV )
      {
        dir |= 2;//当为双向预测且参考图像列表0和１中运动矢量都可获得时　dir=3 只有参考图像列表1可获得时　dir=２　只有参考图像列表０可获得时　dir=１　都不可获得时dir=０
        pcMvFieldNeighbours[ 2 * uiArrayAddr + 1 ].setMvField( cColMv, iRefIdx );
      }
    }

    if (dir != 0)//存在可获得的运动矢量作为候选运动矢量
    {
      puhInterDirNeighbours[uiArrayAddr] = dir;
      abCandIsInter[uiArrayAddr] = true;

      if ( mrgCandIdx == iCount )
      {
        return;
      }
      iCount++;
    }
  }
  // early termination
  if (iCount == getSlice()->getMaxNumMergeCand())
  {
    return;
  }

  UInt uiArrayAddr = iCount;
  UInt uiCutoff = uiArrayAddr;
  //若此时候选运动矢量还未达到规定的最后候选运动矢量数
  if ( getSlice()->isInterB() )//对于B slice　则可以通过组合list0的一个候选运动矢量和list1中另一个运动矢量（不同参考列表不同块）来补充候选运动矢量直到最大值
  {
    static const UInt NUM_PRIORITY_LIST=12;
    static const UInt uiPriorityList0[NUM_PRIORITY_LIST] = {0 , 1, 0, 2, 1, 2, 0, 3, 1, 3, 2, 3};//list0 list1运动矢量组合表
    static const UInt uiPriorityList1[NUM_PRIORITY_LIST] = {1 , 0, 2, 0, 2, 1, 3, 0, 3, 1, 3, 2};

    for (Int idx=0; idx<uiCutoff*(uiCutoff-1) && uiArrayAddr!= getSlice()->getMaxNumMergeCand(); idx++)//遍历所有组合情况　直到达到最大候选运动矢量值
    {
      assert(idx<NUM_PRIORITY_LIST);
      Int i = uiPriorityList0[idx];
      Int j = uiPriorityList1[idx];
      if (abCandIsInter[i] && abCandIsInter[j]&& (puhInterDirNeighbours[i]&0x1)&&(puhInterDirNeighbours[j]&0x2))//确保所组合的运动矢量存在并来自对应的list0 list1
      {
        abCandIsInter[uiArrayAddr] = true;
        puhInterDirNeighbours[uiArrayAddr] = 3;

        // get Mv from cand[i] and cand[j]
        pcMvFieldNeighbours[uiArrayAddr << 1].setMvField(pcMvFieldNeighbours[i<<1].getMv(), pcMvFieldNeighbours[i<<1].getRefIdx());
        pcMvFieldNeighbours[( uiArrayAddr << 1 ) + 1].setMvField(pcMvFieldNeighbours[(j<<1)+1].getMv(), pcMvFieldNeighbours[(j<<1)+1].getRefIdx());//将已存在的候选运动矢量组合成新的候选运动矢量

        Int iRefPOCL0 = m_pcSlice->getRefPOC( REF_PIC_LIST_0, pcMvFieldNeighbours[(uiArrayAddr<<1)].getRefIdx() );
        Int iRefPOCL1 = m_pcSlice->getRefPOC( REF_PIC_LIST_1, pcMvFieldNeighbours[(uiArrayAddr<<1)+1].getRefIdx() );
        if (iRefPOCL0 == iRefPOCL1 && pcMvFieldNeighbours[(uiArrayAddr<<1)].getMv() == pcMvFieldNeighbours[(uiArrayAddr<<1)+1].getMv())//判断用于组合的两运动矢量是否完全相同
        {
          abCandIsInter[uiArrayAddr] = false;//若相同　则舍弃
        }
        else//若不相同　则可以作为候选运动矢量
        {
          uiArrayAddr++;
        }
      }
    }
  }
  // early termination
  if (uiArrayAddr == getSlice()->getMaxNumMergeCand())//达到最大候选运动矢量数　则方法直接结束
  {
    return;
  }
  //若此时还未达到最大的候选运动矢量数
  Int iNumRefIdx = (getSlice()->isInterB()) ? min(m_pcSlice->getNumRefIdx(REF_PIC_LIST_0), m_pcSlice->getNumRefIdx(REF_PIC_LIST_1)) : m_pcSlice->getNumRefIdx(REF_PIC_LIST_0);//参考图像列表中较小的参考图像数

  Int r = 0;//参考图像索引
  Int refcnt = 0;
  while (uiArrayAddr < getSlice()->getMaxNumMergeCand())//对剩余位置补(0,0)　直达达到最大候选运动矢量
  {
    abCandIsInter[uiArrayAddr] = true;
    puhInterDirNeighbours[uiArrayAddr] = 1;
    pcMvFieldNeighbours[uiArrayAddr << 1].setMvField( TComMv(0, 0), r);

    if ( getSlice()->isInterB() )//B slice 则还需对list1中候选运动矢量补(0,0)
    {
      puhInterDirNeighbours[uiArrayAddr] = 3;
      pcMvFieldNeighbours[(uiArrayAddr << 1) + 1].setMvField(TComMv(0, 0), r);
    }
    uiArrayAddr++;

    if ( refcnt == iNumRefIdx - 1 )
    {
      r = 0;
    }
    else
    {
      ++r;
      ++refcnt;
    }
  }
  numValidMergeCand = uiArrayAddr;
}

/** Check whether the current PU and a spatial neighboring PU are in a same ME region.
 * \param xN, yN   location of the upper-left corner pixel of a neighboring PU
 * \param xP, yP   location of the upper-left corner pixel of the current PU
 */
Bool TComDataCU::isDiffMER(Int xN, Int yN, Int xP, Int yP)//判断给定的像素位置所在的PU是否来自不同的MER （不同的像素位置代表了不同的Pu）
{

  UInt plevel = this->getSlice()->getPPS()->getLog2ParallelMergeLevelMinus2() + 2;//可以并行处理一个块的大小　其值越大　并行程度越高
  if ((xN>>plevel)!= (xP>>plevel))//不在同一列MER中（以MER块为单位）
  {
    return true;
  }
  if ((yN>>plevel)!= (yP>>plevel))//不在同一行MER中（以MER块为单位）
  {
    return true;
  }
  return false;
}

/** Calculate the location of upper-left corner pixel and size of the current PU.
 * \param partIdx       PU index within a CU
 * \param xP, yP        location of the upper-left corner pixel of the current PU
 * \param nPSW, nPSH    size of the current PU
 */
Void TComDataCU::getPartPosition( UInt partIdx, Int& xP, Int& yP, Int& nPSW, Int& nPSH)//根据给定的pu块索引得到pu块的位置信息
{
  UInt col = m_uiCUPelX;
  UInt row = m_uiCUPelY;

  switch ( m_pePartSize[0] )//对照CU的PU分割模式　易懂　不在叙述
  {
  case SIZE_2NxN:
    nPSW = getWidth(0);
    nPSH = getHeight(0) >> 1;
    xP   = col;
    yP   = (partIdx ==0)? row: row + nPSH;
    break;
  case SIZE_Nx2N:
    nPSW = getWidth(0) >> 1;
    nPSH = getHeight(0);
    xP   = (partIdx ==0)? col: col + nPSW;
    yP   = row;
    break;
  case SIZE_NxN:
    nPSW = getWidth(0) >> 1;
    nPSH = getHeight(0) >> 1;
    xP   = col + (partIdx&0x1)*nPSW;
    yP   = row + (partIdx>>1)*nPSH;
    break;
  case SIZE_2NxnU:
    nPSW = getWidth(0);
    nPSH = ( partIdx == 0 ) ?  getHeight(0) >> 2 : ( getHeight(0) >> 2 ) + ( getHeight(0) >> 1 );
    xP   = col;
    yP   = (partIdx ==0)? row: row + getHeight(0) - nPSH;

    break;
  case SIZE_2NxnD:
    nPSW = getWidth(0);
    nPSH = ( partIdx == 0 ) ?  ( getHeight(0) >> 2 ) + ( getHeight(0) >> 1 ) : getHeight(0) >> 2;
    xP   = col;
    yP   = (partIdx ==0)? row: row + getHeight(0) - nPSH;
    break;
  case SIZE_nLx2N:
    nPSW = ( partIdx == 0 ) ? getWidth(0) >> 2 : ( getWidth(0) >> 2 ) + ( getWidth(0) >> 1 );
    nPSH = getHeight(0);
    xP   = (partIdx ==0)? col: col + getWidth(0) - nPSW;
    yP   = row;
    break;
  case SIZE_nRx2N:
    nPSW = ( partIdx == 0 ) ? ( getWidth(0) >> 2 ) + ( getWidth(0) >> 1 ) : getWidth(0) >> 2;
    nPSH = getHeight(0);
    xP   = (partIdx ==0)? col: col + getWidth(0) - nPSW;
    yP   = row;
    break;
  default:
    assert ( m_pePartSize[0] == SIZE_2Nx2N );
    nPSW = getWidth(0);
    nPSH = getHeight(0);
    xP   = col ;
    yP   = row ;

    break;
  }
}

/** Constructs a list of candidates for AMVP
 * \param uiPartIdx
 * \param uiPartAddr
 * \param eRefPicList
 * \param iRefIdx
 * \param pInfo
 */
Void TComDataCU::fillMvpCand ( UInt uiPartIdx, UInt uiPartAddr, RefPicList eRefPicList, Int iRefIdx, AMVPInfo* pInfo )//构建运动矢量候选列表　AMVP
{
  TComMv cMvPred;
  Bool bAddedSmvp = false;

  pInfo->iN = 0;//初始化候选运动矢量数为零
  if (iRefIdx < 0)//给定的参考图像无效　直接结束方法
  {
    return;
  }

  //-- Get Spatial MV
  UInt uiPartIdxLT, uiPartIdxRT, uiPartIdxLB;
  const UInt numPartInCtuWidth  = m_pcPic->getNumPartInCtuWidth();
  const UInt numPartInCtuHeight = m_pcPic->getNumPartInCtuHeight();
  Bool bAdded = false;

  deriveLeftRightTopIdx( uiPartIdx, uiPartIdxLT, uiPartIdxRT );//得到当前PU块内左上角和右上角位置　用于下步得到相邻的PU
  deriveLeftBottomIdx( uiPartIdx, uiPartIdxLB );//得到当前PU块内左下角位置　用于下步得到相邻的PU

  TComDataCU* tmpCU = NULL;
  UInt idx;
  tmpCU = getPUBelowLeft(idx, uiPartIdxLB);//左下角pU
  bAddedSmvp = (tmpCU != NULL) && (tmpCU->isInter(idx));//标志　判断A0 A1处运动矢量是否可获得（因为帧内预测不存在运动矢量信息　故无法获得）

  if (!bAddedSmvp)
  {
    tmpCU = getPULeft(idx, uiPartIdxLB);
    bAddedSmvp = (tmpCU != NULL) && (tmpCU->isInter(idx));
  }//若A0 A1都不可获得则bAddedSmvp为假　否则为真
  //左侧选择候选MV的顺序为：A0-->A1-->scaled A0-->scaled A1
  // Left predictor search
  bAdded = xAddMVPCand( pInfo, eRefPicList, iRefIdx, uiPartIdxLB, MD_BELOW_LEFT);//首选尝试获取A0处运动矢量（要求当前Pu参考图像和A0处处参考图像相同）
  if (!bAdded)
  {
    bAdded = xAddMVPCand( pInfo, eRefPicList, iRefIdx, uiPartIdxLB, MD_LEFT );//若不可获取则尝试获取A1处运动矢量（要求当前Pu参考图像和A0处处参考图像相同）
  }

  if(!bAdded)
  {
    bAdded = xAddMVPCandOrder( pInfo, eRefPicList, iRefIdx, uiPartIdxLB, MD_BELOW_LEFT);//若不可获取则尝试获取A0处比例伸缩后的运动矢量
    if (!bAdded)
    {
      xAddMVPCandOrder( pInfo, eRefPicList, iRefIdx, uiPartIdxLB, MD_LEFT );//若不可获取则尝试获取A1处比例伸缩后的运动矢量
    }
  }

  // Above predictor search
  //上方选择候选MV的顺序为：B0-->B1-->B2-->scaled B0-->scaled B1-->scaled B2
  bAdded = xAddMVPCand( pInfo, eRefPicList, iRefIdx, uiPartIdxRT, MD_ABOVE_RIGHT);//首选尝试获取B0处运动矢量（要求当前Pu参考图像和A0处处参考图像相同）

  if (!bAdded)
  {
    bAdded = xAddMVPCand( pInfo, eRefPicList, iRefIdx, uiPartIdxRT, MD_ABOVE);//若不可获取则尝试获取B1处运动矢量（要求当前Pu参考图像和A0处处参考图像相同）
  }

  if(!bAdded)
  {
    xAddMVPCand( pInfo, eRefPicList, iRefIdx, uiPartIdxLT, MD_ABOVE_LEFT);//若不可获取则尝试获取B2处运动矢量（要求当前Pu参考图像和A0处处参考图像相同）
  }

  if(!bAddedSmvp)//上方的三个PU 其比例伸缩只有在左侧PU不存在或都为帧间预测时才进行　该选择模式设计可以在基本降低编码效率的情况下　有效减少因比例伸缩减少的计算复杂　
  {
    bAdded = xAddMVPCandOrder( pInfo, eRefPicList, iRefIdx, uiPartIdxRT, MD_ABOVE_RIGHT);//若不可获取则尝试获取B0处比例伸缩后的运动矢量
    if (!bAdded)
    {
      bAdded = xAddMVPCandOrder( pInfo, eRefPicList, iRefIdx, uiPartIdxRT, MD_ABOVE);//若不可获取则尝试获取B1处比例伸缩后的运动矢量
    }

    if(!bAdded)
    {
      xAddMVPCandOrder( pInfo, eRefPicList, iRefIdx, uiPartIdxLT, MD_ABOVE_LEFT);//若不可获取则尝试获取B2处比例伸缩后的运动矢量
    }
  }

  if ( pInfo->iN == 2 )//若空域的候选运动矢量个数为２　则判断两个运动矢量是否相同
  {
    if ( pInfo->m_acMvCand[ 0 ] == pInfo->m_acMvCand[ 1 ] )//若相同则去掉后一个运动矢量
    {
      pInfo->iN = 1;
    }
  }

  if ( getSlice()->getEnableTMVPFlag() )////允许时域MV预测技术　获取时域候选运动矢量的过程同merge　不在叙述
  {
    // Get Temporal Motion Predictor
    Int iRefIdx_Col = iRefIdx;
    TComMv cColMv;
    UInt uiPartIdxRB;
    UInt uiAbsPartIdx;
    UInt uiAbsPartAddr;

    deriveRightBottomIdx( uiPartIdx, uiPartIdxRB );
    uiAbsPartAddr = m_absZIdxInCtu + uiPartAddr;

    //----  co-located RightBottom Temporal Predictor (H) ---//
    uiAbsPartIdx = g_auiZscanToRaster[uiPartIdxRB];
    Int ctuRsAddr = -1;
    if (  ( ( m_pcPic->getCtu(m_ctuRsAddr)->getCUPelX() + g_auiRasterToPelX[uiAbsPartIdx] + m_pcPic->getMinCUWidth () ) < m_pcSlice->getSPS()->getPicWidthInLumaSamples () )  // image boundary check
       && ( ( m_pcPic->getCtu(m_ctuRsAddr)->getCUPelY() + g_auiRasterToPelY[uiAbsPartIdx] + m_pcPic->getMinCUHeight() ) < m_pcSlice->getSPS()->getPicHeightInLumaSamples() ) )
    {
      if ( ( uiAbsPartIdx % numPartInCtuWidth < numPartInCtuWidth - 1 ) &&  // is not at the last column of CTU
           ( uiAbsPartIdx / numPartInCtuWidth < numPartInCtuHeight - 1 ) )  // is not at the last row    of CTU
      {
        uiAbsPartAddr = g_auiRasterToZscan[ uiAbsPartIdx + numPartInCtuWidth + 1 ];
        ctuRsAddr = getCtuRsAddr();
      }
      else if ( uiAbsPartIdx % numPartInCtuWidth < numPartInCtuWidth - 1 )  // is not at the last column of CTU But is last row of CTU
      {
        uiAbsPartAddr = g_auiRasterToZscan[ (uiAbsPartIdx + numPartInCtuWidth + 1) % m_pcPic->getNumPartitionsInCtu() ];
      }
      else if ( uiAbsPartIdx / numPartInCtuWidth < numPartInCtuHeight - 1 ) // is not at the last row of CTU But is last column of CTU
      {
        uiAbsPartAddr = g_auiRasterToZscan[ uiAbsPartIdx + 1 ];
        ctuRsAddr = getCtuRsAddr() + 1;
      }
      else //is the right bottom corner of CTU
      {
        uiAbsPartAddr = 0;
      }
    }
    if ( ctuRsAddr >= 0 && xGetColMVP( eRefPicList, ctuRsAddr, uiAbsPartAddr, cColMv, iRefIdx_Col ) )
    {
      pInfo->m_acMvCand[pInfo->iN++] = cColMv;
    }
    else
    {
      UInt uiPartIdxCenter;
      xDeriveCenterIdx( uiPartIdx, uiPartIdxCenter );
      if (xGetColMVP( eRefPicList, getCtuRsAddr(), uiPartIdxCenter,  cColMv, iRefIdx_Col ))
      {
        pInfo->m_acMvCand[pInfo->iN++] = cColMv;
      }
    }
    //----  co-located RightBottom Temporal Predictor  ---//
  }

  if (pInfo->iN > AMVP_MAX_NUM_CANDS)//若候选运动矢量数大于２（AMVP规定的运动矢量数）
  {
    pInfo->iN = AMVP_MAX_NUM_CANDS;//则保留候选列表中的前两个运动矢量
  }

  while (pInfo->iN < AMVP_MAX_NUM_CANDS)//若候选运动矢量数不足２　则补(0.0)　直到规定的最大候选运动矢量数
  {
    pInfo->m_acMvCand[pInfo->iN].set(0,0);
    pInfo->iN++;
  }
  return ;
}


Bool TComDataCU::isBipredRestriction(UInt puIdx)
{
  Int width = 0;
  Int height = 0;
  UInt partAddr;

  getPartIndexAndSize( puIdx, partAddr, width, height );
  if ( getWidth(0) == 8 && (width < 8 || height < 8) )//CU块大小为8*8并且分割成更小的PU
  {
    return true;
  }
  return false;
}


Void TComDataCU::clipMv    (TComMv&  rcMv)//根据CU的位置限制运动矢量可能达到的值
{//m_uiCUPelX m_uiCUPelY为当前CU的左上角像素位置
  const TComSPS &sps=*(m_pcSlice->getSPS());
  Int  iMvShift = 2;//1/4像素精度
  Int iOffset = 8;
  Int iHorMax = ( sps.getPicWidthInLumaSamples() + iOffset - (Int)m_uiCUPelX - 1 ) << iMvShift;//不能超出图像的右边界
  Int iHorMin = (      -(Int)sps.getMaxCUWidth() - iOffset - (Int)m_uiCUPelX + 1 ) << iMvShift;//不能超出图像的左边界

  Int iVerMax = ( sps.getPicHeightInLumaSamples() + iOffset - (Int)m_uiCUPelY - 1 ) << iMvShift;//不能超出图像的下边界
  Int iVerMin = (      -(Int)sps.getMaxCUHeight() - iOffset - (Int)m_uiCUPelY + 1 ) << iMvShift;//不能超出图像的上边界

  rcMv.setHor( min (iHorMax, max (iHorMin, rcMv.getHor())) );
  rcMv.setVer( min (iVerMax, max (iVerMin, rcMv.getVer())) );
}


UInt TComDataCU::getIntraSizeIdx(UInt uiAbsPartIdx)//计算帧间预测　PU块的大小
{
  UInt uiShift = ( m_pePartSize[uiAbsPartIdx]==SIZE_NxN ? 1 : 0 );

  UChar uiWidth = m_puhWidth[uiAbsPartIdx]>>uiShift;
  UInt  uiCnt = 0;
  while( uiWidth )
  {
    uiCnt++;
    uiWidth>>=1;
  }
  uiCnt-=2;
  return uiCnt > 6 ? 6 : uiCnt;
}

Void TComDataCU::clearCbf( UInt uiIdx, ComponentID compID, UInt uiNumParts )//清除当前CU的CBF
{
  memset( &m_puhCbf[compID][uiIdx], 0, sizeof(UChar)*uiNumParts);
}

/** Set a I_PCM flag for all sub-partitions of a partition.
 * \param bIpcmFlag I_PCM flag
 * \param uiAbsPartIdx patition index
 * \param uiDepth CU depth
 * \returns Void
 */
Void TComDataCU::setIPCMFlagSubParts  (Bool bIpcmFlag, UInt uiAbsPartIdx, UInt uiDepth)//为给定子块设置PCM标志
{
  UInt uiCurrPartNumb = m_pcPic->getNumPartitionsInCtu() >> (uiDepth << 1);

  memset(m_pbIPCMFlag + uiAbsPartIdx, bIpcmFlag, sizeof(Bool)*uiCurrPartNumb );
}

/** Test whether the block at uiPartIdx is skipped.
 * \param uiPartIdx Partition index
 * \returns true if the current the block is skipped
 */
Bool TComDataCU::isSkipped( UInt uiPartIdx )
{
  return ( getSkipFlag( uiPartIdx ) );
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

Bool TComDataCU::xAddMVPCand( AMVPInfo* pInfo, RefPicList eRefPicList, Int iRefIdx, UInt uiPartUnitIdx, MVP_DIR eDir )//得到给定位置的候选运动矢量并返回是否得到运动矢量　只能处理所参考图像是相同的块　不可进行比例缩放
{
  TComDataCU* pcTmpCU = NULL;
  UInt uiIdx;
  switch( eDir )//得到不同候选块的位置　
  {
    case MD_LEFT:
    {
      pcTmpCU = getPULeft(uiIdx, uiPartUnitIdx);
      break;
    }
    case MD_ABOVE:
    {
      pcTmpCU = getPUAbove(uiIdx, uiPartUnitIdx);
      break;
    }
    case MD_ABOVE_RIGHT:
    {
      pcTmpCU = getPUAboveRight(uiIdx, uiPartUnitIdx);
      break;
    }
    case MD_BELOW_LEFT:
    {
      pcTmpCU = getPUBelowLeft(uiIdx, uiPartUnitIdx);
      break;
    }
    case MD_ABOVE_LEFT:
    {
      pcTmpCU = getPUAboveLeft(uiIdx, uiPartUnitIdx);
      break;
    }
    default:
    {
      break;
    }
  }

  if ( pcTmpCU == NULL )//若候选块为空　则直接返回false方法结束
  {
    return false;
  }

  if ( pcTmpCU->getCUMvField(eRefPicList)->getRefIdx(uiIdx) >= 0 && m_pcSlice->getRefPic( eRefPicList, iRefIdx)->getPOC() == pcTmpCU->getSlice()->getRefPOC( eRefPicList, pcTmpCU->getCUMvField(eRefPicList)->getRefIdx(uiIdx) ))
  {//候选块（候选块包含候选运动矢量）的参考图像与当前块相同
    TComMv cMvPred = pcTmpCU->getCUMvField(eRefPicList)->getMv(uiIdx);

    pInfo->m_acMvCand[ pInfo->iN++] = cMvPred;//则　将候选块的运动矢量添加候选运动矢量列表中
    return true;
  }

  RefPicList eRefPicList2nd = REF_PIC_LIST_0;//定义另一个与当前参考图像列表不同的参考图像列表
  if( eRefPicList == REF_PIC_LIST_0 )
  {
    eRefPicList2nd = REF_PIC_LIST_1;
  }
  else if ( eRefPicList == REF_PIC_LIST_1)
  {
    eRefPicList2nd = REF_PIC_LIST_0;
  }


  Int iCurrRefPOC = m_pcSlice->getRefPic( eRefPicList, iRefIdx)->getPOC();//当前块参考图像的POC值　(picture order count　图像输出顺序　POC值可唯一指定一帧图像)
  Int iNeibRefPOC;


  if( pcTmpCU->getCUMvField(eRefPicList2nd)->getRefIdx(uiIdx) >= 0 )
  {
    iNeibRefPOC = pcTmpCU->getSlice()->getRefPOC( eRefPicList2nd, pcTmpCU->getCUMvField(eRefPicList2nd)->getRefIdx(uiIdx) );
    if( iNeibRefPOC == iCurrRefPOC ) // Same Reference Frame But Diff List//当前块和候选块参考图像列表不同　但参考图像相同
    {
      TComMv cMvPred = pcTmpCU->getCUMvField(eRefPicList2nd)->getMv(uiIdx);
      pInfo->m_acMvCand[ pInfo->iN++] = cMvPred;　//将候选块的运动矢量添加候选运动矢量列表中并返回真
      return true;
    }
  }
  return false;
}

/**
 * \param pInfo
 * \param eRefPicList
 * \param iRefIdx
 * \param uiPartUnitIdx
 * \param eDir
 * \returns Bool
 */
Bool TComDataCU::xAddMVPCandOrder( AMVPInfo* pInfo, RefPicList eRefPicList, Int iRefIdx, UInt uiPartUnitIdx, MVP_DIR eDir )//得到给定位置的候选运动矢量并返回是否得到运动矢量　能处理所参考图像不相同的块　
{
  TComDataCU* pcTmpCU = NULL;
  UInt uiIdx;
  switch( eDir )//得到不同候选块的位置
  {
  case MD_LEFT:
    {
      pcTmpCU = getPULeft(uiIdx, uiPartUnitIdx);
      break;
    }
  case MD_ABOVE:
    {
      pcTmpCU = getPUAbove(uiIdx, uiPartUnitIdx);
      break;
    }
  case MD_ABOVE_RIGHT:
    {
      pcTmpCU = getPUAboveRight(uiIdx, uiPartUnitIdx);
      break;
    }
  case MD_BELOW_LEFT:
    {
      pcTmpCU = getPUBelowLeft(uiIdx, uiPartUnitIdx);
      break;
    }
  case MD_ABOVE_LEFT:
    {
      pcTmpCU = getPUAboveLeft(uiIdx, uiPartUnitIdx);
      break;
    }
  default:
    {
      break;
    }
  }

  if ( pcTmpCU == NULL )
  {
    return false;
  }

  RefPicList eRefPicList2nd = REF_PIC_LIST_0;//定义另一个与当前参考图像列表不同的参考图像列表
  if(       eRefPicList == REF_PIC_LIST_0 )
  {
    eRefPicList2nd = REF_PIC_LIST_1;
  }
  else if ( eRefPicList == REF_PIC_LIST_1)
  {
    eRefPicList2nd = REF_PIC_LIST_0;
  }

  Int iCurrPOC = m_pcSlice->getPOC();
  Int iCurrRefPOC = m_pcSlice->getRefPic( eRefPicList, iRefIdx)->getPOC();
  Int iNeibPOC = iCurrPOC;
  Int iNeibRefPOC;
  Bool bIsCurrRefLongTerm = m_pcSlice->getRefPic( eRefPicList, iRefIdx)->getIsLongTerm();
  Bool bIsNeibRefLongTerm = false;

  //---------------  V1 (END) ------------------//
  if( pcTmpCU->getCUMvField(eRefPicList)->getRefIdx(uiIdx) >= 0)//该PU块帧间预测在给定的参考图像列表中存在有效参考图像
  {//curMV=td/tb*colMV
    iNeibRefPOC = pcTmpCU->getSlice()->getRefPOC( eRefPicList, pcTmpCU->getCUMvField(eRefPicList)->getRefIdx(uiIdx) );
    TComMv cMvPred = pcTmpCU->getCUMvField(eRefPicList)->getMv(uiIdx);
    TComMv rcMv;

    bIsNeibRefLongTerm = pcTmpCU->getSlice()->getRefPic( eRefPicList, pcTmpCU->getCUMvField(eRefPicList)->getRefIdx(uiIdx) )->getIsLongTerm();
    if ( bIsCurrRefLongTerm == bIsNeibRefLongTerm )
    {
      if ( bIsCurrRefLongTerm || bIsNeibRefLongTerm )//都为long term reference pictures时直接获取该运动矢量
      {
        rcMv = cMvPred;
      }
      else//只有当前块参考图像和候选块参考图像都为short term reference pictures时才进行运动矢量的比例伸缩
      {
        Int iScale = xGetDistScaleFactor( iCurrPOC, iCurrRefPOC, iNeibPOC, iNeibRefPOC );//根据当前块参考图像的POC值和候选块参考图像的POC值计算伸缩比例
        if ( iScale == 4096 )
        {
          rcMv = cMvPred;
        }
        else
        {
          rcMv = cMvPred.scaleMv( iScale );//计算scaled mv
        }
      }

      pInfo->m_acMvCand[ pInfo->iN++] = rcMv;
      return true;
    }
  }
  //---------------------- V2(END) --------------------//
  if( pcTmpCU->getCUMvField(eRefPicList2nd)->getRefIdx(uiIdx) >= 0)////该PU块帧间预测在给定的参考图像列表中不存在有效参考图像　而在另一个参考列表中存在有效参考图像
  {//计算scaled mv过程同上
    iNeibRefPOC = pcTmpCU->getSlice()->getRefPOC( eRefPicList2nd, pcTmpCU->getCUMvField(eRefPicList2nd)->getRefIdx(uiIdx) );
    TComMv cMvPred = pcTmpCU->getCUMvField(eRefPicList2nd)->getMv(uiIdx);
    TComMv rcMv;

    bIsNeibRefLongTerm = pcTmpCU->getSlice()->getRefPic( eRefPicList2nd, pcTmpCU->getCUMvField(eRefPicList2nd)->getRefIdx(uiIdx) )->getIsLongTerm();
    if ( bIsCurrRefLongTerm == bIsNeibRefLongTerm )
    {
      if ( bIsCurrRefLongTerm || bIsNeibRefLongTerm )
      {
        rcMv = cMvPred;
      }
      else
      {
        Int iScale = xGetDistScaleFactor( iCurrPOC, iCurrRefPOC, iNeibPOC, iNeibRefPOC );
        if ( iScale == 4096 )
        {
          rcMv = cMvPred;
        }
        else
        {
          rcMv = cMvPred.scaleMv( iScale );
        }
      }

      pInfo->m_acMvCand[ pInfo->iN++] = rcMv;
      return true;
    }
  }
  //---------------------- V3(END) --------------------//
  return false;
}

Bool TComDataCU::xGetColMVP( RefPicList eRefPicList, Int ctuRsAddr, Int uiPartUnitIdx, TComMv& rcMv, Int& riRefIdx )//得到同位块的运动矢量预测
{
  UInt uiAbsPartAddr = uiPartUnitIdx;

  RefPicList  eColRefPicList;
  Int iColPOC, iColRefPOC, iCurrPOC, iCurrRefPOC, iScale;
  TComMv cColMv;

  // use coldir.
  TComPic *pColPic = getSlice()->getRefPic( RefPicList(getSlice()->isInterB() ? 1-getSlice()->getColFromL0Flag() : 0), getSlice()->getColRefIdx());//得到同位图像（即同位块所在的图像）
  TComDataCU *pColCtu = pColPic->getCtu( ctuRsAddr );//同位块所在的CTu
  if(pColCtu->getPic()==0||pColCtu->getPartitionSize(uiPartUnitIdx)==NUMBER_OF_PART_SIZES)//同位块所在图像为第一帧图像　或同位块还未解码　则无法参考同位块的运动信息
  {
    return false;
  }
  iCurrPOC = m_pcSlice->getPOC();
  iColPOC = pColCtu->getSlice()->getPOC();

  if (!pColCtu->isInter(uiAbsPartAddr))//同位块所在CTU为帧内编码　帧内编码不存在运动矢量　　
  {
    return false;
  }

  eColRefPicList = getSlice()->getCheckLDC() ? eRefPicList : RefPicList(getSlice()->getColFromL0Flag());

  Int iColRefIdx = pColCtu->getCUMvField(RefPicList(eColRefPicList))->getRefIdx(uiAbsPartAddr);//同位块的参考图像的索引

  if (iColRefIdx < 0 )
  {
    eColRefPicList = RefPicList(1 - eColRefPicList);
    iColRefIdx = pColCtu->getCUMvField(RefPicList(eColRefPicList))->getRefIdx(uiAbsPartAddr);

    if (iColRefIdx < 0 )
    {
      return false;//参考图像列表list0 list1中均不存在有效参考图像
    }
  }

  // Scale the vector.
  iColRefPOC = pColCtu->getSlice()->getRefPOC(eColRefPicList, iColRefIdx);
  cColMv = pColCtu->getCUMvField(eColRefPicList)->getMv(uiAbsPartAddr);

  iCurrRefPOC = m_pcSlice->getRefPic(eRefPicList, riRefIdx)->getPOC();

  Bool bIsCurrRefLongTerm = m_pcSlice->getRefPic(eRefPicList, riRefIdx)->getIsLongTerm();
  Bool bIsColRefLongTerm = pColCtu->getSlice()->getIsUsedAsLongTerm(eColRefPicList, iColRefIdx);

  if ( bIsCurrRefLongTerm != bIsColRefLongTerm )//参考图像类型不同　则当前块无法参考同位块的运动矢量信息
  {
    return false;
  }

  if ( bIsCurrRefLongTerm || bIsColRefLongTerm )//都为long term reference pictures时直接获取该运动矢量
  {
    rcMv = cColMv;
  }
  else
  {
    iScale = xGetDistScaleFactor(iCurrPOC, iCurrRefPOC, iColPOC, iColRefPOC);//根据当前块参考图像的POC值和候选块参考图像的POC值计算伸缩比例
    if ( iScale == 4096 )
    {
      rcMv = cColMv;
    }
    else
    {
      rcMv = cColMv.scaleMv( iScale );//计算scaled mv
    }
  }

  return true;
}

Int TComDataCU::xGetDistScaleFactor(Int iCurrPOC, Int iCurrRefPOC, Int iColPOC, Int iColRefPOC)//根据当前块参考图像的POC值和候选块参考图像的POC值计算伸缩比例
{//iscale=tb/td  计算过程中的移位是为了：用整数运算来完成小数计算时保证小数计算的精度
  Int iDiffPocD = iColPOC - iColRefPOC;
  Int iDiffPocB = iCurrPOC - iCurrRefPOC;

  if( iDiffPocD == iDiffPocB )
  {
    return 4096;
  }
  else
  {
    Int iTDB      = Clip3( -128, 127, iDiffPocB );
    Int iTDD      = Clip3( -128, 127, iDiffPocD );
    Int iX        = (0x4000 + abs(iTDD/2)) / iTDD;//abs(iTDD/2)为0.5的舍入偏移量　确保小数取整时的四舍五入(而不是向下取整)
    Int iScale    = Clip3( -4096, 4095, (iTDB * iX + 32) >> 6 );
    return iScale;
  }
}

Void TComDataCU::xDeriveCenterIdx( UInt uiPartIdx, UInt& ruiPartIdxCenter )//得到次选同位块C1位置
{//C1位置为将当前PU十字划分后第四块的起始位置
  UInt uiPartAddr;
  Int  iPartWidth;
  Int  iPartHeight;
  getPartIndexAndSize( uiPartIdx, uiPartAddr, iPartWidth, iPartHeight);

  ruiPartIdxCenter = m_absZIdxInCtu+uiPartAddr; // partition origin.
  ruiPartIdxCenter = g_auiRasterToZscan[ g_auiZscanToRaster[ ruiPartIdxCenter ]
                                        + ( iPartHeight/m_pcPic->getMinCUHeight()  )/2*m_pcPic->getNumPartInCtuWidth()
                                        + ( iPartWidth/m_pcPic->getMinCUWidth()  )/2];//PU块中心位置
}

Void TComDataCU::compressMV()
{
  Int scaleFactor = 4 * AMVP_DECIMATION_FACTOR / m_unitSize;//下采样比
  if (scaleFactor > 0)
  {
    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      m_acCUMvField[i].compress(m_pePredMode, scaleFactor);////对存储好的运动信息下采样
    }
  }
}

UInt TComDataCU::getCoefScanIdx(const UInt uiAbsPartIdx, const UInt uiWidth, const UInt uiHeight, const ComponentID compID) const
{//根据块的信息得到块的变换系数的扫描方式
  //------------------------------------------------

  //this mechanism is available for intra only

  if (!isIntra(uiAbsPartIdx))//不为帧内预测　扫描方式为对角扫描
  {
    return SCAN_DIAG;
  }

  //------------------------------------------------

  //check that MDCS can be used for this TU

  const ChromaFormat format = getPic()->getChromaFormat();

  const UInt maximumWidth  = MDCS_MAXIMUM_WIDTH  >> getComponentScaleX(compID, format);
  const UInt maximumHeight = MDCS_MAXIMUM_HEIGHT >> getComponentScaleY(compID, format);

  if ((uiWidth > maximumWidth) || (uiHeight > maximumHeight))//大于规定块的大小(8*8)　用对角扫描
  {
    return SCAN_DIAG;
  }

  //------------------------------------------------

  //otherwise, select the appropriate mode

  UInt uiDirMode  = getIntraDir(toChannelType(compID), uiAbsPartIdx);//亮度分量的预测模式

  if (uiDirMode==DM_CHROMA_IDX)//色度分量的预测模式
  {
    const TComSPS *sps=getSlice()->getSPS();
    const UInt partsPerMinCU = 1<<(2*(sps->getMaxTotalCUDepth() - sps->getLog2DiffMaxMinCodingBlockSize()));
    uiDirMode = getIntraDir(CHANNEL_TYPE_LUMA, getChromasCorrespondingPULumaIdx(uiAbsPartIdx, getPic()->getChromaFormat(), partsPerMinCU));
  }

  if (isChroma(compID) && (format == CHROMA_422))
  {
    uiDirMode = g_chroma422IntraAngleMappingTable[uiDirMode];
  }

  //------------------

  if　(abs((Int)uiDirMode - VER_IDX) <= MDCS_ANGLE_LIMIT)//若预测模式与垂直模式相在给定的范围内　
  {
    return SCAN_HOR;//则水平扫描
  }
  else if (abs((Int)uiDirMode - HOR_IDX) <= MDCS_ANGLE_LIMIT)//若预测模式与水平模式相在给定的范围内
  {
    return SCAN_VER;//则垂直扫描
  }
  else
  {
    return SCAN_DIAG;//否则水平扫描
  }
}

//! \}
