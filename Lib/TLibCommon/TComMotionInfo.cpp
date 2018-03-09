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

/** \file     TComMotionInfo.cpp
    \brief    motion information handling classes
*/

#include <memory.h>
#include "TComMotionInfo.h"
#include "assert.h"
#include <stdlib.h>

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// --------------------------------------------------------------------------------------------------------------------
// Create / destroy
// --------------------------------------------------------------------------------------------------------------------

<<<<<<< HEAD
<<<<<<< HEAD
Void TComCUMvField::create( UInt uiNumPartition )//创建　声明需要的数组
=======
Void TComCUMvField::create( UInt uiNumPartition )
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
=======
Void TComCUMvField::create( UInt uiNumPartition )
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
{
  assert(m_pcMv     == NULL);
  assert(m_pcMvd    == NULL);
  assert(m_piRefIdx == NULL);

<<<<<<< HEAD
<<<<<<< HEAD
  m_pcMv     = new TComMv[ uiNumPartition ];//运动矢量
  m_pcMvd    = new TComMv[ uiNumPartition ];//运动矢量差
  m_piRefIdx = new Char  [ uiNumPartition ];//参考索引
=======
  m_pcMv     = new TComMv[ uiNumPartition ];
  m_pcMvd    = new TComMv[ uiNumPartition ];
  m_piRefIdx = new Char  [ uiNumPartition ];
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
=======
  m_pcMv     = new TComMv[ uiNumPartition ];
  m_pcMvd    = new TComMv[ uiNumPartition ];
  m_piRefIdx = new Char  [ uiNumPartition ];
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2

  m_uiNumPartition = uiNumPartition;
}

<<<<<<< HEAD
<<<<<<< HEAD
Void TComCUMvField::destroy()//销毁　释放资源
=======
Void TComCUMvField::destroy()
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
=======
Void TComCUMvField::destroy()
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
{
  assert(m_pcMv     != NULL);
  assert(m_pcMvd    != NULL);
  assert(m_piRefIdx != NULL);

  delete[] m_pcMv;
  delete[] m_pcMvd;
  delete[] m_piRefIdx;

  m_pcMv     = NULL;
  m_pcMvd    = NULL;
  m_piRefIdx = NULL;

  m_uiNumPartition = 0;
}

// --------------------------------------------------------------------------------------------------------------------
// Clear / copy
// --------------------------------------------------------------------------------------------------------------------

<<<<<<< HEAD
<<<<<<< HEAD
Void TComCUMvField::clearMvField()//清除运动矢量及运动矢量差
=======
Void TComCUMvField::clearMvField()
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
=======
Void TComCUMvField::clearMvField()
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
{
  for ( Int i = 0; i < m_uiNumPartition; i++ )
  {
    m_pcMv [ i ].setZero();
    m_pcMvd[ i ].setZero();
  }
  assert( sizeof( *m_piRefIdx ) == 1 );
  memset( m_piRefIdx, NOT_VALID, m_uiNumPartition * sizeof( *m_piRefIdx ) );
}

<<<<<<< HEAD
<<<<<<< HEAD
Void TComCUMvField::copyFrom( TComCUMvField const * pcCUMvFieldSrc, Int iNumPartSrc, Int iPartAddrDst )//从pcCUMvFieldSrc开始复制iNumPartSrc个TComMv到目的地址
=======
Void TComCUMvField::copyFrom( TComCUMvField const * pcCUMvFieldSrc, Int iNumPartSrc, Int iPartAddrDst )
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
=======
Void TComCUMvField::copyFrom( TComCUMvField const * pcCUMvFieldSrc, Int iNumPartSrc, Int iPartAddrDst )
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
{
  Int iSizeInTComMv = sizeof( TComMv ) * iNumPartSrc;

  memcpy( m_pcMv     + iPartAddrDst, pcCUMvFieldSrc->m_pcMv,     iSizeInTComMv );
  memcpy( m_pcMvd    + iPartAddrDst, pcCUMvFieldSrc->m_pcMvd,    iSizeInTComMv );
  memcpy( m_piRefIdx + iPartAddrDst, pcCUMvFieldSrc->m_piRefIdx, sizeof( *m_piRefIdx ) * iNumPartSrc );
}

Void TComCUMvField::copyTo( TComCUMvField* pcCUMvFieldDst, Int iPartAddrDst ) const
{
  copyTo( pcCUMvFieldDst, iPartAddrDst, 0, m_uiNumPartition );
}

<<<<<<< HEAD
<<<<<<< HEAD
Void TComCUMvField::copyTo( TComCUMvField* pcCUMvFieldDst, Int iPartAddrDst, UInt uiOffset, UInt uiNumPart ) const//将运动矢量信息复制到目的地址
=======
Void TComCUMvField::copyTo( TComCUMvField* pcCUMvFieldDst, Int iPartAddrDst, UInt uiOffset, UInt uiNumPart ) const
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
=======
Void TComCUMvField::copyTo( TComCUMvField* pcCUMvFieldDst, Int iPartAddrDst, UInt uiOffset, UInt uiNumPart ) const
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
{
  Int iSizeInTComMv = sizeof( TComMv ) * uiNumPart;
  Int iOffset = uiOffset + iPartAddrDst;

  memcpy( pcCUMvFieldDst->m_pcMv     + iOffset, m_pcMv     + uiOffset, iSizeInTComMv );
  memcpy( pcCUMvFieldDst->m_pcMvd    + iOffset, m_pcMvd    + uiOffset, iSizeInTComMv );
  memcpy( pcCUMvFieldDst->m_piRefIdx + iOffset, m_piRefIdx + uiOffset, sizeof( *m_piRefIdx ) * uiNumPart );
}

// --------------------------------------------------------------------------------------------------------------------
// Set
// --------------------------------------------------------------------------------------------------------------------

<<<<<<< HEAD
<<<<<<< HEAD
template <typename T>//模板函数
Void TComCUMvField::setAll( T *p, T const & val, PartSize eCUMode, Int iPartAddr, UInt uiDepth, Int iPartIdx  )//对Cu中不同的Pu设置运动矢量信息
{
  Int i;
  p += iPartAddr;//CU中pu的地址　决定对称分割中哪一pU被设置　在一个CU中每小块的位置为：将CU十字分成四等块　按Z形分别遍历每四分之一块
  Int numElements = m_uiNumPartition >> ( 2 * uiDepth );//深度为uiDepth的CU含有的小块个数

  switch( eCUMode )//判读该Cu的PU分割模式
  {
    case SIZE_2Nx2N:
      for ( i = 0; i < numElements; i++ )//该CU所有小块设为val
=======
=======
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
template <typename T>
Void TComCUMvField::setAll( T *p, T const & val, PartSize eCUMode, Int iPartAddr, UInt uiDepth, Int iPartIdx  )
{
  Int i;
  p += iPartAddr;
  Int numElements = m_uiNumPartition >> ( 2 * uiDepth );

  switch( eCUMode )
  {
    case SIZE_2Nx2N:
      for ( i = 0; i < numElements; i++ )
<<<<<<< HEAD
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
=======
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
      {
        p[ i ] = val;
      }
      break;

    case SIZE_2NxN:
      numElements >>= 1;
<<<<<<< HEAD
<<<<<<< HEAD
      for ( i = 0; i < numElements; i++ )//该CU上部（下部）PU所有小块设为val
=======
      for ( i = 0; i < numElements; i++ )
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
=======
      for ( i = 0; i < numElements; i++ )
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
      {
        p[ i ] = val;
      }
      break;

    case SIZE_Nx2N:
      numElements >>= 2;
<<<<<<< HEAD
<<<<<<< HEAD
      for ( i = 0; i < numElements; i++ )//该CU左侧（右侧）PU所有小块设为val
=======
      for ( i = 0; i < numElements; i++ )
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
=======
      for ( i = 0; i < numElements; i++ )
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
      {
        p[ i                   ] = val;
        p[ i + 2 * numElements ] = val;
      }
      break;

    case SIZE_NxN:
      numElements >>= 2;
<<<<<<< HEAD
<<<<<<< HEAD
      for ( i = 0; i < numElements; i++)//该CU左上（右上　左下　右下）PU所有小块设为val
=======
      for ( i = 0; i < numElements; i++)
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
=======
      for ( i = 0; i < numElements; i++)
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
      {
        p[ i ] = val;
      }
      break;
    case SIZE_2NxnU:
    {
      Int iCurrPartNumQ = numElements>>2;
<<<<<<< HEAD
<<<<<<< HEAD
      if( iPartIdx == 0 )//CU中第一块Pu 即上方1/4大小的PU
=======
      if( iPartIdx == 0 )
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
=======
      if( iPartIdx == 0 )
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
      {
        T *pT  = p;
        T *pT2 = p + iCurrPartNumQ;
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pT [i] = val;
          pT2[i] = val;
        }
      }
<<<<<<< HEAD
<<<<<<< HEAD
      else//CU中第二块Pu 即下方３/4大小的PU
      {
        T *pT  = p;//下方PU上边界最左则处
=======
      else
      {
        T *pT  = p;
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
=======
      else
      {
        T *pT  = p;
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pT[i] = val;
        }

<<<<<<< HEAD
<<<<<<< HEAD
        pT = p + iCurrPartNumQ;//下方PU上边界中点处
=======
        pT = p + iCurrPartNumQ;
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
=======
        pT = p + iCurrPartNumQ;
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
        for (i = 0; i < ( (iCurrPartNumQ>>1) + (iCurrPartNumQ<<1) ); i++)
        {
          pT[i] = val;
        }
      }
      break;
    }
<<<<<<< HEAD
<<<<<<< HEAD
  case SIZE_2NxnD://与SIZE_2NxnU相反
=======
  case SIZE_2NxnD:
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
=======
  case SIZE_2NxnD:
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
    {
      Int iCurrPartNumQ = numElements>>2;
      if( iPartIdx == 0 )
      {
        T *pT  = p;
        for (i = 0; i < ( (iCurrPartNumQ>>1) + (iCurrPartNumQ<<1) ); i++)
        {
          pT[i] = val;
        }
        pT = p + ( numElements - iCurrPartNumQ );
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pT[i] = val;
        }
      }
      else
      {
<<<<<<< HEAD
<<<<<<< HEAD
        T *pT  = p;//下方PU上边界最左侧处
        T *pT2 = p + iCurrPartNumQ;//下方PU上边界中点处
=======
        T *pT  = p;
        T *pT2 = p + iCurrPartNumQ;
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
=======
        T *pT  = p;
        T *pT2 = p + iCurrPartNumQ;
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
        for (i = 0; i < (iCurrPartNumQ>>1); i++)
        {
          pT [i] = val;
          pT2[i] = val;
        }
      }
      break;
    }
  case SIZE_nLx2N:
    {
      Int iCurrPartNumQ = numElements>>2;
<<<<<<< HEAD
<<<<<<< HEAD
      if( iPartIdx == 0 )//CU中第一块Pu 即左侧1/4大小的PU
      {
        T *pT  = p;//左侧PU左边界上方处
        T *pT2 = p + (iCurrPartNumQ<<1);//左侧PU左边界中点处
        T *pT3 = p + (iCurrPartNumQ>>1);//左侧PU左边界1/4处
        T *pT4 = p + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);//左侧PU左边界3/4处
=======
=======
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
      if( iPartIdx == 0 )
      {
        T *pT  = p;
        T *pT2 = p + (iCurrPartNumQ<<1);
        T *pT3 = p + (iCurrPartNumQ>>1);
        T *pT4 = p + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);
<<<<<<< HEAD
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
=======
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2

        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pT [i] = val;
          pT2[i] = val;
          pT3[i] = val;
          pT4[i] = val;
        }
      }
<<<<<<< HEAD
<<<<<<< HEAD
      else//CU中第二块Pu 即右侧３/4大小的PU
      {
        T *pT  = p;//右侧PU块左侧边界最上方处
        T *pT2 = p + (iCurrPartNumQ<<1);//右侧PU块左侧边界中点处
=======
=======
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
      else
      {
        T *pT  = p;
        T *pT2 = p + (iCurrPartNumQ<<1);
<<<<<<< HEAD
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
=======
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pT [i] = val;
          pT2[i] = val;
        }

<<<<<<< HEAD
<<<<<<< HEAD
        pT  = p + (iCurrPartNumQ>>1);//右侧PU块左侧边界1/4处
        pT2 = p + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);//右侧PU块左侧边界3/4处
=======
        pT  = p + (iCurrPartNumQ>>1);
        pT2 = p + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
=======
        pT  = p + (iCurrPartNumQ>>1);
        pT2 = p + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
        for (i = 0; i < ( (iCurrPartNumQ>>2) + iCurrPartNumQ ); i++)
        {
          pT [i] = val;
          pT2[i] = val;
        }
      }
      break;
    }
<<<<<<< HEAD
<<<<<<< HEAD
  case SIZE_nRx2N://与SIZE_nRx2N相反
=======
  case SIZE_nRx2N:
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
=======
  case SIZE_nRx2N:
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
    {
      Int iCurrPartNumQ = numElements>>2;
      if( iPartIdx == 0 )
      {
        T *pT  = p;
        T *pT2 = p + (iCurrPartNumQ<<1);
        for (i = 0; i < ( (iCurrPartNumQ>>2) + iCurrPartNumQ ); i++)
        {
          pT [i] = val;
          pT2[i] = val;
        }

        pT  = p + iCurrPartNumQ + (iCurrPartNumQ>>1);
        pT2 = p + numElements - iCurrPartNumQ + (iCurrPartNumQ>>1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pT [i] = val;
          pT2[i] = val;
        }
      }
      else
      {
        T *pT  = p;
        T *pT2 = p + (iCurrPartNumQ>>1);
        T *pT3 = p + (iCurrPartNumQ<<1);
        T *pT4 = p + (iCurrPartNumQ<<1) + (iCurrPartNumQ>>1);
        for (i = 0; i < (iCurrPartNumQ>>2); i++)
        {
          pT [i] = val;
          pT2[i] = val;
          pT3[i] = val;
          pT4[i] = val;
        }
      }
      break;
    }
    default:
      assert(0);
      break;
  }
}

<<<<<<< HEAD
<<<<<<< HEAD
Void TComCUMvField::setAllMv( TComMv const & mv, PartSize eCUMode, Int iPartAddr, UInt uiDepth, Int iPartIdx )//设置运动矢量
=======
Void TComCUMvField::setAllMv( TComMv const & mv, PartSize eCUMode, Int iPartAddr, UInt uiDepth, Int iPartIdx )
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
=======
Void TComCUMvField::setAllMv( TComMv const & mv, PartSize eCUMode, Int iPartAddr, UInt uiDepth, Int iPartIdx )
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
{
  setAll(m_pcMv, mv, eCUMode, iPartAddr, uiDepth, iPartIdx);
}

<<<<<<< HEAD
<<<<<<< HEAD
Void TComCUMvField::setAllMvd( TComMv const & mvd, PartSize eCUMode, Int iPartAddr, UInt uiDepth, Int iPartIdx )//设置运动矢量差
=======
Void TComCUMvField::setAllMvd( TComMv const & mvd, PartSize eCUMode, Int iPartAddr, UInt uiDepth, Int iPartIdx )
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
=======
Void TComCUMvField::setAllMvd( TComMv const & mvd, PartSize eCUMode, Int iPartAddr, UInt uiDepth, Int iPartIdx )
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
{
  setAll(m_pcMvd, mvd, eCUMode, iPartAddr, uiDepth, iPartIdx);
}

<<<<<<< HEAD
<<<<<<< HEAD
Void TComCUMvField::setAllRefIdx ( Int iRefIdx, PartSize eCUMode, Int iPartAddr, UInt uiDepth, Int iPartIdx )//设置运动矢量参考图像
=======
Void TComCUMvField::setAllRefIdx ( Int iRefIdx, PartSize eCUMode, Int iPartAddr, UInt uiDepth, Int iPartIdx )
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
=======
Void TComCUMvField::setAllRefIdx ( Int iRefIdx, PartSize eCUMode, Int iPartAddr, UInt uiDepth, Int iPartIdx )
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
{
  setAll(m_piRefIdx, static_cast<Char>(iRefIdx), eCUMode, iPartAddr, uiDepth, iPartIdx);
}

<<<<<<< HEAD
<<<<<<< HEAD
Void TComCUMvField::setAllMvField( TComMvField const & mvField, PartSize eCUMode, Int iPartAddr, UInt uiDepth, Int iPartIdx )//设置运动矢量信息
=======
Void TComCUMvField::setAllMvField( TComMvField const & mvField, PartSize eCUMode, Int iPartAddr, UInt uiDepth, Int iPartIdx )
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
=======
Void TComCUMvField::setAllMvField( TComMvField const & mvField, PartSize eCUMode, Int iPartAddr, UInt uiDepth, Int iPartIdx )
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
{
  setAllMv    ( mvField.getMv(),     eCUMode, iPartAddr, uiDepth, iPartIdx );
  setAllRefIdx( mvField.getRefIdx(), eCUMode, iPartAddr, uiDepth, iPartIdx );
}

/**Subsampling of the stored prediction mode, reference index and motion vector
 * \param pePredMode Pointer to prediction modes
 * \param scale      Factor by which to subsample motion information
 */
<<<<<<< HEAD
<<<<<<< HEAD
Void TComCUMvField::compress(Char* pePredMode, Int scale)//对存储好的运动信息子采样
{
  Int N = scale * scale;//子采样的间隔
=======
Void TComCUMvField::compress(Char* pePredMode, Int scale)
{
  Int N = scale * scale;
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
=======
Void TComCUMvField::compress(Char* pePredMode, Int scale)
{
  Int N = scale * scale;
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
  assert( N > 0 && N <= m_uiNumPartition);

  for ( Int uiPartIdx = 0; uiPartIdx < m_uiNumPartition; uiPartIdx += N )
  {
    TComMv cMv(0,0);
    Int iRefIdx = 0;
<<<<<<< HEAD
<<<<<<< HEAD
    //子采样信息
    cMv = m_pcMv[ uiPartIdx ];
    PredMode predMode = static_cast<PredMode>( pePredMode[ uiPartIdx ] );
    iRefIdx = m_piRefIdx[ uiPartIdx ];
    for ( Int i = 0; i < N; i++ )//采样间隔类复制子采样到的信息
=======
=======
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2

    cMv = m_pcMv[ uiPartIdx ];
    PredMode predMode = static_cast<PredMode>( pePredMode[ uiPartIdx ] );
    iRefIdx = m_piRefIdx[ uiPartIdx ];
    for ( Int i = 0; i < N; i++ )
<<<<<<< HEAD
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
=======
>>>>>>> 0570385d3f2e289018a9a67ece33f3b3c8ae19b2
    {
      m_pcMv[ uiPartIdx + i ] = cMv;
      pePredMode[ uiPartIdx + i ] = predMode;
      m_piRefIdx[ uiPartIdx + i ] = iRefIdx;
    }
  }
}
//! \}
