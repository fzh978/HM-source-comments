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

/** \file     TComTrQuant.cpp
    \brief    transform and quantization class
*/

#include <stdlib.h>
#include <math.h>
#include <limits>
#include <memory.h>
#include "TComTrQuant.h"
#include "TComPic.h"
#include "ContextTables.h"
#include "TComTU.h"
#include "Debug.h"

typedef struct
{
  Int    iNNZbeforePos0;
  Double d64CodedLevelandDist; // distortion and level cost only
  Double d64UncodedDist;    // all zero coded block distortion
  Double d64SigCost;
  Double d64SigCost_0;
} coeffGroupRDStats;//coeffGroupRDStats结构体　用于率失真优化量化中记录各种损耗（具体含义见xRateDistOptQuant方法注解）

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constants
// ====================================================================================================================

#define RDOQ_CHROMA                 1           ///< use of RDOQ in chroma　色度分量也使用率失真优化量化


// ====================================================================================================================
// QpParam constructor
// ====================================================================================================================

QpParam::QpParam(const Int           qpy,
                 const ChannelType   chType,
                 const Int           qpBdOffset,
                 const Int           chromaQPOffset,
                 const ChromaFormat  chFmt )//Qp构造器　初始化给定参数的QP值
{
  Int baseQp;

  if(isLuma(chType))
  {
    baseQp = qpy + qpBdOffset;
  }
  else
  {
    baseQp = Clip3( -qpBdOffset, (chromaQPMappingTableSize - 1), qpy + chromaQPOffset );

    if(baseQp < 0)
    {
      baseQp = baseQp + qpBdOffset;
    }
    else
    {
      baseQp = getScaledChromaQP(baseQp, chFmt) + qpBdOffset;
    }
  }

  Qp =baseQp;
  per=baseQp/6;
  rem=baseQp%6;
}

QpParam::QpParam(const TComDataCU &cu, const ComponentID compID)//重载　使用另一种参数初始化QP
{
  Int chromaQpOffset = 0;

  if (isChroma(compID))
  {
    chromaQpOffset += cu.getSlice()->getPPS()->getQpOffset(compID);
    chromaQpOffset += cu.getSlice()->getSliceChromaQpDelta(compID);

    chromaQpOffset += cu.getSlice()->getPPS()->getPpsRangeExtension().getChromaQpOffsetListEntry(cu.getChromaQpAdj(0)).u.offset[Int(compID)-1];
  }

  *this = QpParam(cu.getQP( 0 ),
                  toChannelType(compID),
                  cu.getSlice()->getSPS()->getQpBDOffset(toChannelType(compID)),
                  chromaQpOffset,
                  cu.getPic()->getChromaFormat());
}


// ====================================================================================================================
// TComTrQuant class member functions
// ====================================================================================================================

TComTrQuant::TComTrQuant()//构造函数
{
  // allocate temporary buffers
  m_plTempCoeff  = new TCoeff[ MAX_CU_SIZE*MAX_CU_SIZE ];

  // allocate bit estimation class  (for RDOQ)
  m_pcEstBitsSbac = new estBitsSbacStruct;
  initScalingList();
}

TComTrQuant::~TComTrQuant()//析构函数
{
  // delete temporary buffers
  if ( m_plTempCoeff )
  {
    delete [] m_plTempCoeff;
    m_plTempCoeff = NULL;
  }

  // delete bit estimation class
  if ( m_pcEstBitsSbac )
  {
    delete m_pcEstBitsSbac;
  }
  destroyScalingList();
}

#if ADAPTIVE_QP_SELECTION//如果为自适应QP选择　下一个slice的QP值可以通过当前Slice得到
Void TComTrQuant::storeSliceQpNext(TComSlice* pcSlice)
{
  // NOTE: does this work with negative QPs or when some blocks are transquant-bypass enabled?

  Int qpBase = pcSlice->getSliceQpBase();
  Int sliceQpused = pcSlice->getSliceQp();
  Int sliceQpnext;
  Double alpha = qpBase < 17 ? 0.5 : 1;

  Int cnt=0;
  for(Int u=1; u<=LEVEL_RANGE; u++)
  {
    cnt += m_sliceNsamples[u] ;
  }

  if( !m_useRDOQ )
  {
    sliceQpused = qpBase;
    alpha = 0.5;
  }

  if( cnt > 120 )
  {
    Double sum = 0;
    Int k = 0;
    for(Int u=1; u<LEVEL_RANGE; u++)
    {
      sum += u*m_sliceSumC[u];
      k += u*u*m_sliceNsamples[u];
    }

    Int v;
    Double q[MAX_QP+1] ;
    for(v=0; v<=MAX_QP; v++)
    {
      q[v] = (Double)(g_invQuantScales[v%6] * (1<<(v/6)))/64 ;//用QP值计算Qstep(量化步长)
    }

    Double qnext = sum/k * q[sliceQpused] / (1<<ARL_C_PRECISION);

    for(v=0; v<MAX_QP; v++)
    {
      if(qnext < alpha * q[v] + (1 - alpha) * q[v+1] )
      {
        break;
      }
    }
    sliceQpnext = Clip3(sliceQpused - 3, sliceQpused + 3, v);
  }
  else
  {
    sliceQpnext = sliceQpused;
  }

  m_qpDelta[qpBase] = sliceQpnext - qpBase;
}

Void TComTrQuant::initSliceQpDelta()
{
  for(Int qp=0; qp<=MAX_QP; qp++)
  {
    m_qpDelta[qp] = qp < 17 ? 0 : 1;
  }
}

Void TComTrQuant::clearSliceARLCnt()
{
  memset(m_sliceSumC, 0, sizeof(Double)*(LEVEL_RANGE+1));
  memset(m_sliceNsamples, 0, sizeof(Int)*(LEVEL_RANGE+1));
}
#endif



#if MATRIX_MULT
/** NxN forward transform (2D) using brute force matrix multiplication (3 nested loops)
 *  \param block pointer to input data (residual)
 *  \param coeff pointer to output data (transform coefficients)
 *  \param uiStride stride of input data
 *  \param uiTrSize transform size (uiTrSize x uiTrSize)
 *  \param uiMode is Intra Prediction mode used in Mode-Dependent DCT/DST only
 */
Void xTr(Int bitDepth, Pel *block, TCoeff *coeff, UInt uiStride, UInt uiTrSize, Bool useDST, const Int maxLog2TrDynamicRange)
{
  UInt i,j,k;
  TCoeff iSum;
  TCoeff tmp[MAX_TU_SIZE * MAX_TU_SIZE];
  const TMatrixCoeff *iT;
  UInt uiLog2TrSize = g_aucConvertToBit[ uiTrSize ] + 2;

  if (uiTrSize==4)
  {
    iT  = (useDST ? g_as_DST_MAT_4[TRANSFORM_FORWARD][0] : g_aiT4[TRANSFORM_FORWARD][0]);
  }
  else if (uiTrSize==8)
  {
    iT = g_aiT8[TRANSFORM_FORWARD][0];
  }
  else if (uiTrSize==16)
  {
    iT = g_aiT16[TRANSFORM_FORWARD][0];
  }
  else if (uiTrSize==32)
  {
    iT = g_aiT32[TRANSFORM_FORWARD][0];
  }
  else
  {
    assert(0);
  }

  const Int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_FORWARD];

  const Int shift_1st = (uiLog2TrSize +  bitDepth + TRANSFORM_MATRIX_SHIFT) - maxLog2TrDynamicRange;
  const Int shift_2nd = uiLog2TrSize + TRANSFORM_MATRIX_SHIFT;
  const Int add_1st = (shift_1st>0) ? (1<<(shift_1st-1)) : 0;
  const Int add_2nd = 1<<(shift_2nd-1);

  /* Horizontal transform */

  for (i=0; i<uiTrSize; i++)
  {
    for (j=0; j<uiTrSize; j++)
    {
      iSum = 0;
      for (k=0; k<uiTrSize; k++)
      {
        iSum += iT[i*uiTrSize+k]*block[j*uiStride+k];
      }
      tmp[i*uiTrSize+j] = (iSum + add_1st)>>shift_1st;
    }
  }

  /* Vertical transform */
  for (i=0; i<uiTrSize; i++)
  {
    for (j=0; j<uiTrSize; j++)
    {
      iSum = 0;
      for (k=0; k<uiTrSize; k++)
      {
        iSum += iT[i*uiTrSize+k]*tmp[j*uiTrSize+k];
      }
      coeff[i*uiTrSize+j] = (iSum + add_2nd)>>shift_2nd;
    }
  }
}

/** NxN inverse transform (2D) using brute force matrix multiplication (3 nested loops)
 *  \param coeff pointer to input data (transform coefficients)
 *  \param block pointer to output data (residual)
 *  \param uiStride stride of output data
 *  \param uiTrSize transform size (uiTrSize x uiTrSize)
 *  \param uiMode is Intra Prediction mode used in Mode-Dependent DCT/DST only
 */
Void xITr(Int bitDepth, TCoeff *coeff, Pel *block, UInt uiStride, UInt uiTrSize, Bool useDST, const Int maxLog2TrDynamicRange)
{
  UInt i,j,k;
  TCoeff iSum;
  TCoeff tmp[MAX_TU_SIZE * MAX_TU_SIZE];
  const TMatrixCoeff *iT;

  if (uiTrSize==4)
  {
    iT  = (useDST ? g_as_DST_MAT_4[TRANSFORM_INVERSE][0] : g_aiT4[TRANSFORM_INVERSE][0]);
  }
  else if (uiTrSize==8)
  {
    iT = g_aiT8[TRANSFORM_INVERSE][0];
  }
  else if (uiTrSize==16)
  {
    iT = g_aiT16[TRANSFORM_INVERSE][0];
  }
  else if (uiTrSize==32)
  {
    iT = g_aiT32[TRANSFORM_INVERSE][0];
  }
  else
  {
    assert(0);
  }

  const Int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_INVERSE];

  const Int shift_1st = TRANSFORM_MATRIX_SHIFT + 1; //1 has been added to shift_1st at the expense of shift_2nd
  const Int shift_2nd = (TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1) - bitDepth;
  const TCoeff clipMinimum = -(1 << maxLog2TrDynamicRange);
  const TCoeff clipMaximum =  (1 << maxLog2TrDynamicRange) - 1;
  assert(shift_2nd>=0);
  const Int add_1st = 1<<(shift_1st-1);
  const Int add_2nd = (shift_2nd>0) ? (1<<(shift_2nd-1)) : 0;

  /* Horizontal transform */
  for (i=0; i<uiTrSize; i++)
  {
    for (j=0; j<uiTrSize; j++)
    {
      iSum = 0;
      for (k=0; k<uiTrSize; k++)
      {
        iSum += iT[k*uiTrSize+i]*coeff[k*uiTrSize+j];
      }

      // Clipping here is not in the standard, but is used to protect the "Pel" data type into which the inverse-transformed samples will be copied
      tmp[i*uiTrSize+j] = Clip3<TCoeff>(clipMinimum, clipMaximum, (iSum + add_1st)>>shift_1st);
    }
  }

  /* Vertical transform */
  for (i=0; i<uiTrSize; i++)
  {
    for (j=0; j<uiTrSize; j++)
    {
      iSum = 0;
      for (k=0; k<uiTrSize; k++)
      {
        iSum += iT[k*uiTrSize+j]*tmp[i*uiTrSize+k];
      }

      block[i*uiStride+j] = Clip3<TCoeff>(std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max(), (iSum + add_2nd)>>shift_2nd);
    }
  }
}

#endif //MATRIX_MULT


/** 4x4 forward transform implemented using partial butterfly structure (1D)
 *  \param src   input data (residual)
 *  \param dst   output data (transform coefficients)
 *  \param shift specifies right shift after 1D transform
 *  \param line
 */
Void partialButterfly4(TCoeff *src, TCoeff *dst, Int shift, Int line)
{
  Int j;
  TCoeff E[2],O[2];
  TCoeff add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++)
  {
    /* E and O */
    E[0] = src[0] + src[3];
    O[0] = src[0] - src[3];
    E[1] = src[1] + src[2];
    O[1] = src[1] - src[2];

    dst[0]      = (g_aiT4[TRANSFORM_FORWARD][0][0]*E[0] + g_aiT4[TRANSFORM_FORWARD][0][1]*E[1] + add)>>shift;
    dst[2*line] = (g_aiT4[TRANSFORM_FORWARD][2][0]*E[0] + g_aiT4[TRANSFORM_FORWARD][2][1]*E[1] + add)>>shift;
    dst[line]   = (g_aiT4[TRANSFORM_FORWARD][1][0]*O[0] + g_aiT4[TRANSFORM_FORWARD][1][1]*O[1] + add)>>shift;
    dst[3*line] = (g_aiT4[TRANSFORM_FORWARD][3][0]*O[0] + g_aiT4[TRANSFORM_FORWARD][3][1]*O[1] + add)>>shift;

    src += 4;
    dst ++;
  }
}

// Fast DST Algorithm. Full matrix multiplication for DST and Fast DST algorithm
// give identical results
Void fastForwardDst(TCoeff *block, TCoeff *coeff, Int shift)  // input block, output coeff
{//快速DST变换
  Int i;
  TCoeff c[4];
  TCoeff rnd_factor = (shift > 0) ? (1<<(shift-1)) : 0;
  for (i=0; i<4; i++)
  {
    // Intermediate Variables
    c[0] = block[4*i+0];
    c[1] = block[4*i+1];
    c[2] = block[4*i+2];
    c[3] = block[4*i+3];

    for (Int row = 0; row < 4; row++)
    {
      TCoeff result = 0;
      for (Int column = 0; column < 4; column++)
      {
        result += c[column] * g_as_DST_MAT_4[TRANSFORM_FORWARD][row][column]; // use the defined matrix, rather than hard-wired numbers
      }

      coeff[(row * 4) + i] = rightShift((result + rnd_factor), shift);
    }
  }
}

Void fastInverseDst(TCoeff *tmp, TCoeff *block, Int shift, const TCoeff outputMinimum, const TCoeff outputMaximum)  // input tmp, output block
{//快速DST反变换
  Int i;
  TCoeff c[4];
  TCoeff rnd_factor = (shift > 0) ? (1<<(shift-1)) : 0;
  for (i=0; i<4; i++)
  {
    // Intermediate Variables
    c[0] = tmp[   i];
    c[1] = tmp[4 +i];
    c[2] = tmp[8 +i];
    c[3] = tmp[12+i];

    for (Int column = 0; column < 4; column++)
    {
      TCoeff &result = block[(i * 4) + column];

      result = 0;
      for (Int row = 0; row < 4; row++)
      {
        result += c[row] * g_as_DST_MAT_4[TRANSFORM_INVERSE][row][column]; // use the defined matrix, rather than hard-wired numbers
      }

      result = Clip3( outputMinimum, outputMaximum, rightShift((result + rnd_factor), shift));
    }
  }
}

/** 4x4 inverse transform implemented using partial butterfly structure (1D)
 *  \param src   input data (transform coefficients)
 *  \param dst   output data (residual)
 *  \param shift specifies right shift after 1D transform
 *  \param line
 *  \param outputMinimum  minimum for clipping
 *  \param outputMaximum  maximum for clipping
 */
Void partialButterflyInverse4(TCoeff *src, TCoeff *dst, Int shift, Int line, const TCoeff outputMinimum, const TCoeff outputMaximum)
{//４系数反变换　教科书上经典的蝶形快速算法实现　不在叙述
  Int j;
  TCoeff E[2],O[2];
  TCoeff add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++)
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    O[0] = g_aiT4[TRANSFORM_INVERSE][1][0]*src[line] + g_aiT4[TRANSFORM_INVERSE][3][0]*src[3*line];
    O[1] = g_aiT4[TRANSFORM_INVERSE][1][1]*src[line] + g_aiT4[TRANSFORM_INVERSE][3][1]*src[3*line];
    E[0] = g_aiT4[TRANSFORM_INVERSE][0][0]*src[0]    + g_aiT4[TRANSFORM_INVERSE][2][0]*src[2*line];
    E[1] = g_aiT4[TRANSFORM_INVERSE][0][1]*src[0]    + g_aiT4[TRANSFORM_INVERSE][2][1]*src[2*line];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    dst[0] = Clip3( outputMinimum, outputMaximum, (E[0] + O[0] + add)>>shift );
    dst[1] = Clip3( outputMinimum, outputMaximum, (E[1] + O[1] + add)>>shift );
    dst[2] = Clip3( outputMinimum, outputMaximum, (E[1] - O[1] + add)>>shift );
    dst[3] = Clip3( outputMinimum, outputMaximum, (E[0] - O[0] + add)>>shift );

    src   ++;
    dst += 4;
  }
}

/** 8x8 forward transform implemented using partial butterfly structure (1D)
 *  \param src   input data (residual)
 *  \param dst   output data (transform coefficients)
 *  \param shift specifies right shift after 1D transform
 *  \param line
 */
Void partialButterfly8(TCoeff *src, TCoeff *dst, Int shift, Int line)
{//８系数　教科书上经典的蝶形快速算法实现　不在叙述
  Int j,k;
  TCoeff E[4],O[4];
  TCoeff EE[2],EO[2];
  TCoeff add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++)
  {
    /* E and O*/
    for (k=0;k<4;k++)
    {
      E[k] = src[k] + src[7-k];
      O[k] = src[k] - src[7-k];
    }
    /* EE and EO */
    EE[0] = E[0] + E[3];
    EO[0] = E[0] - E[3];
    EE[1] = E[1] + E[2];
    EO[1] = E[1] - E[2];

    dst[0]      = (g_aiT8[TRANSFORM_FORWARD][0][0]*EE[0] + g_aiT8[TRANSFORM_FORWARD][0][1]*EE[1] + add)>>shift;
    dst[4*line] = (g_aiT8[TRANSFORM_FORWARD][4][0]*EE[0] + g_aiT8[TRANSFORM_FORWARD][4][1]*EE[1] + add)>>shift;
    dst[2*line] = (g_aiT8[TRANSFORM_FORWARD][2][0]*EO[0] + g_aiT8[TRANSFORM_FORWARD][2][1]*EO[1] + add)>>shift;
    dst[6*line] = (g_aiT8[TRANSFORM_FORWARD][6][0]*EO[0] + g_aiT8[TRANSFORM_FORWARD][6][1]*EO[1] + add)>>shift;

    dst[line]   = (g_aiT8[TRANSFORM_FORWARD][1][0]*O[0] + g_aiT8[TRANSFORM_FORWARD][1][1]*O[1] + g_aiT8[TRANSFORM_FORWARD][1][2]*O[2] + g_aiT8[TRANSFORM_FORWARD][1][3]*O[3] + add)>>shift;
    dst[3*line] = (g_aiT8[TRANSFORM_FORWARD][3][0]*O[0] + g_aiT8[TRANSFORM_FORWARD][3][1]*O[1] + g_aiT8[TRANSFORM_FORWARD][3][2]*O[2] + g_aiT8[TRANSFORM_FORWARD][3][3]*O[3] + add)>>shift;
    dst[5*line] = (g_aiT8[TRANSFORM_FORWARD][5][0]*O[0] + g_aiT8[TRANSFORM_FORWARD][5][1]*O[1] + g_aiT8[TRANSFORM_FORWARD][5][2]*O[2] + g_aiT8[TRANSFORM_FORWARD][5][3]*O[3] + add)>>shift;
    dst[7*line] = (g_aiT8[TRANSFORM_FORWARD][7][0]*O[0] + g_aiT8[TRANSFORM_FORWARD][7][1]*O[1] + g_aiT8[TRANSFORM_FORWARD][7][2]*O[2] + g_aiT8[TRANSFORM_FORWARD][7][3]*O[3] + add)>>shift;

    src += 8;
    dst ++;
  }
}

/** 8x8 inverse transform implemented using partial butterfly structure (1D)
 *  \param src   input data (transform coefficients)
 *  \param dst   output data (residual)
 *  \param shift specifies right shift after 1D transform
 *  \param line
 *  \param outputMinimum  minimum for clipping
 *  \param outputMaximum  maximum for clipping
 */
Void partialButterflyInverse8(TCoeff *src, TCoeff *dst, Int shift, Int line, const TCoeff outputMinimum, const TCoeff outputMaximum)
{//8系数反变换　教科书上经典的蝶形快速算法实现　不在叙述
  Int j,k;
  TCoeff E[4],O[4];
  TCoeff EE[2],EO[2];
  TCoeff add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++)
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    for (k=0;k<4;k++)
    {
      O[k] = g_aiT8[TRANSFORM_INVERSE][ 1][k]*src[line]   + g_aiT8[TRANSFORM_INVERSE][ 3][k]*src[3*line] +
             g_aiT8[TRANSFORM_INVERSE][ 5][k]*src[5*line] + g_aiT8[TRANSFORM_INVERSE][ 7][k]*src[7*line];
    }

    EO[0] = g_aiT8[TRANSFORM_INVERSE][2][0]*src[ 2*line ] + g_aiT8[TRANSFORM_INVERSE][6][0]*src[ 6*line ];
    EO[1] = g_aiT8[TRANSFORM_INVERSE][2][1]*src[ 2*line ] + g_aiT8[TRANSFORM_INVERSE][6][1]*src[ 6*line ];
    EE[0] = g_aiT8[TRANSFORM_INVERSE][0][0]*src[ 0      ] + g_aiT8[TRANSFORM_INVERSE][4][0]*src[ 4*line ];
    EE[1] = g_aiT8[TRANSFORM_INVERSE][0][1]*src[ 0      ] + g_aiT8[TRANSFORM_INVERSE][4][1]*src[ 4*line ];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    E[0] = EE[0] + EO[0];
    E[3] = EE[0] - EO[0];
    E[1] = EE[1] + EO[1];
    E[2] = EE[1] - EO[1];
    for (k=0;k<4;k++)
    {
      dst[ k   ] = Clip3( outputMinimum, outputMaximum, (E[k] + O[k] + add)>>shift );
      dst[ k+4 ] = Clip3( outputMinimum, outputMaximum, (E[3-k] - O[3-k] + add)>>shift );
    }
    src ++;
    dst += 8;
  }
}

/** 16x16 forward transform implemented using partial butterfly structure (1D)
 *  \param src   input data (residual)
 *  \param dst   output data (transform coefficients)
 *  \param shift specifies right shift after 1D transform
 *  \param line
 */
Void partialButterfly16(TCoeff *src, TCoeff *dst, Int shift, Int line)
{//16系数　教科书上经典的蝶形快速算法实现　不在叙述
  Int j,k;
  TCoeff E[8],O[8];
  TCoeff EE[4],EO[4];
  TCoeff EEE[2],EEO[2];
  TCoeff add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++)
  {
    /* E and O*/
    for (k=0;k<8;k++)
    {
      E[k] = src[k] + src[15-k];
      O[k] = src[k] - src[15-k];
    }
    /* EE and EO */
    for (k=0;k<4;k++)
    {
      EE[k] = E[k] + E[7-k];
      EO[k] = E[k] - E[7-k];
    }
    /* EEE and EEO */
    EEE[0] = EE[0] + EE[3];
    EEO[0] = EE[0] - EE[3];
    EEE[1] = EE[1] + EE[2];
    EEO[1] = EE[1] - EE[2];

    dst[ 0      ] = (g_aiT16[TRANSFORM_FORWARD][ 0][0]*EEE[0] + g_aiT16[TRANSFORM_FORWARD][ 0][1]*EEE[1] + add)>>shift;
    dst[ 8*line ] = (g_aiT16[TRANSFORM_FORWARD][ 8][0]*EEE[0] + g_aiT16[TRANSFORM_FORWARD][ 8][1]*EEE[1] + add)>>shift;
    dst[ 4*line ] = (g_aiT16[TRANSFORM_FORWARD][ 4][0]*EEO[0] + g_aiT16[TRANSFORM_FORWARD][ 4][1]*EEO[1] + add)>>shift;
    dst[ 12*line] = (g_aiT16[TRANSFORM_FORWARD][12][0]*EEO[0] + g_aiT16[TRANSFORM_FORWARD][12][1]*EEO[1] + add)>>shift;

    for (k=2;k<16;k+=4)
    {
      dst[ k*line ] = (g_aiT16[TRANSFORM_FORWARD][k][0]*EO[0] + g_aiT16[TRANSFORM_FORWARD][k][1]*EO[1] +
                       g_aiT16[TRANSFORM_FORWARD][k][2]*EO[2] + g_aiT16[TRANSFORM_FORWARD][k][3]*EO[3] + add)>>shift;
    }

    for (k=1;k<16;k+=2)
    {
      dst[ k*line ] = (g_aiT16[TRANSFORM_FORWARD][k][0]*O[0] + g_aiT16[TRANSFORM_FORWARD][k][1]*O[1] +
                       g_aiT16[TRANSFORM_FORWARD][k][2]*O[2] + g_aiT16[TRANSFORM_FORWARD][k][3]*O[3] +
                       g_aiT16[TRANSFORM_FORWARD][k][4]*O[4] + g_aiT16[TRANSFORM_FORWARD][k][5]*O[5] +
                       g_aiT16[TRANSFORM_FORWARD][k][6]*O[6] + g_aiT16[TRANSFORM_FORWARD][k][7]*O[7] + add)>>shift;
    }

    src += 16;
    dst ++;

  }
}

/** 16x16 inverse transform implemented using partial butterfly structure (1D)
 *  \param src            input data (transform coefficients)
 *  \param dst            output data (residual)
 *  \param shift          specifies right shift after 1D transform
 *  \param line
 *  \param outputMinimum  minimum for clipping
 *  \param outputMaximum  maximum for clipping
 */
Void partialButterflyInverse16(TCoeff *src, TCoeff *dst, Int shift, Int line, const TCoeff outputMinimum, const TCoeff outputMaximum)
{//１６系数反变换　教科书上经典的蝶形快速算法实现　不在叙述
  Int j,k;
  TCoeff E[8],O[8];
  TCoeff EE[4],EO[4];
  TCoeff EEE[2],EEO[2];
  TCoeff add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++)
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    for (k=0;k<8;k++)
    {
      O[k] = g_aiT16[TRANSFORM_INVERSE][ 1][k]*src[ line]   + g_aiT16[TRANSFORM_INVERSE][ 3][k]*src[ 3*line] +
             g_aiT16[TRANSFORM_INVERSE][ 5][k]*src[ 5*line] + g_aiT16[TRANSFORM_INVERSE][ 7][k]*src[ 7*line] +
             g_aiT16[TRANSFORM_INVERSE][ 9][k]*src[ 9*line] + g_aiT16[TRANSFORM_INVERSE][11][k]*src[11*line] +
             g_aiT16[TRANSFORM_INVERSE][13][k]*src[13*line] + g_aiT16[TRANSFORM_INVERSE][15][k]*src[15*line];
    }
    for (k=0;k<4;k++)
    {
      EO[k] = g_aiT16[TRANSFORM_INVERSE][ 2][k]*src[ 2*line] + g_aiT16[TRANSFORM_INVERSE][ 6][k]*src[ 6*line] +
              g_aiT16[TRANSFORM_INVERSE][10][k]*src[10*line] + g_aiT16[TRANSFORM_INVERSE][14][k]*src[14*line];
    }
    EEO[0] = g_aiT16[TRANSFORM_INVERSE][4][0]*src[ 4*line ] + g_aiT16[TRANSFORM_INVERSE][12][0]*src[ 12*line ];
    EEE[0] = g_aiT16[TRANSFORM_INVERSE][0][0]*src[ 0      ] + g_aiT16[TRANSFORM_INVERSE][ 8][0]*src[ 8*line  ];
    EEO[1] = g_aiT16[TRANSFORM_INVERSE][4][1]*src[ 4*line ] + g_aiT16[TRANSFORM_INVERSE][12][1]*src[ 12*line ];
    EEE[1] = g_aiT16[TRANSFORM_INVERSE][0][1]*src[ 0      ] + g_aiT16[TRANSFORM_INVERSE][ 8][1]*src[ 8*line  ];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    for (k=0;k<2;k++)
    {
      EE[k] = EEE[k] + EEO[k];
      EE[k+2] = EEE[1-k] - EEO[1-k];
    }
    for (k=0;k<4;k++)
    {
      E[k] = EE[k] + EO[k];
      E[k+4] = EE[3-k] - EO[3-k];
    }
    for (k=0;k<8;k++)
    {
      dst[k]   = Clip3( outputMinimum, outputMaximum, (E[k] + O[k] + add)>>shift );
      dst[k+8] = Clip3( outputMinimum, outputMaximum, (E[7-k] - O[7-k] + add)>>shift );
    }
    src ++;
    dst += 16;
  }
}

/** 32x32 forward transform implemented using partial butterfly structure (1D)
 *  \param src   input data (residual)
 *  \param dst   output data (transform coefficients)
 *  \param shift specifies right shift after 1D transform
 *  \param line
 */
Void partialButterfly32(TCoeff *src, TCoeff *dst, Int shift, Int line)
{//32系数　教科书上经典的蝶形快速算法实现　不在叙述
  Int j,k;
  TCoeff E[16],O[16];
  TCoeff EE[8],EO[8];
  TCoeff EEE[4],EEO[4];
  TCoeff EEEE[2],EEEO[2];
  TCoeff add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++)
  {
    /* E and O*/
    for (k=0;k<16;k++)
    {
      E[k] = src[k] + src[31-k];
      O[k] = src[k] - src[31-k];
    }
    /* EE and EO */
    for (k=0;k<8;k++)
    {
      EE[k] = E[k] + E[15-k];
      EO[k] = E[k] - E[15-k];
    }
    /* EEE and EEO */
    for (k=0;k<4;k++)
    {
      EEE[k] = EE[k] + EE[7-k];
      EEO[k] = EE[k] - EE[7-k];
    }
    /* EEEE and EEEO */
    EEEE[0] = EEE[0] + EEE[3];
    EEEO[0] = EEE[0] - EEE[3];
    EEEE[1] = EEE[1] + EEE[2];
    EEEO[1] = EEE[1] - EEE[2];

    dst[ 0       ] = (g_aiT32[TRANSFORM_FORWARD][ 0][0]*EEEE[0] + g_aiT32[TRANSFORM_FORWARD][ 0][1]*EEEE[1] + add)>>shift;
    dst[ 16*line ] = (g_aiT32[TRANSFORM_FORWARD][16][0]*EEEE[0] + g_aiT32[TRANSFORM_FORWARD][16][1]*EEEE[1] + add)>>shift;
    dst[ 8*line  ] = (g_aiT32[TRANSFORM_FORWARD][ 8][0]*EEEO[0] + g_aiT32[TRANSFORM_FORWARD][ 8][1]*EEEO[1] + add)>>shift;
    dst[ 24*line ] = (g_aiT32[TRANSFORM_FORWARD][24][0]*EEEO[0] + g_aiT32[TRANSFORM_FORWARD][24][1]*EEEO[1] + add)>>shift;
    for (k=4;k<32;k+=8)
    {
      dst[ k*line ] = (g_aiT32[TRANSFORM_FORWARD][k][0]*EEO[0] + g_aiT32[TRANSFORM_FORWARD][k][1]*EEO[1] +
                       g_aiT32[TRANSFORM_FORWARD][k][2]*EEO[2] + g_aiT32[TRANSFORM_FORWARD][k][3]*EEO[3] + add)>>shift;
    }
    for (k=2;k<32;k+=4)
    {
      dst[ k*line ] = (g_aiT32[TRANSFORM_FORWARD][k][0]*EO[0] + g_aiT32[TRANSFORM_FORWARD][k][1]*EO[1] +
                       g_aiT32[TRANSFORM_FORWARD][k][2]*EO[2] + g_aiT32[TRANSFORM_FORWARD][k][3]*EO[3] +
                       g_aiT32[TRANSFORM_FORWARD][k][4]*EO[4] + g_aiT32[TRANSFORM_FORWARD][k][5]*EO[5] +
                       g_aiT32[TRANSFORM_FORWARD][k][6]*EO[6] + g_aiT32[TRANSFORM_FORWARD][k][7]*EO[7] + add)>>shift;
    }
    for (k=1;k<32;k+=2)
    {
      dst[ k*line ] = (g_aiT32[TRANSFORM_FORWARD][k][ 0]*O[ 0] + g_aiT32[TRANSFORM_FORWARD][k][ 1]*O[ 1] +
                       g_aiT32[TRANSFORM_FORWARD][k][ 2]*O[ 2] + g_aiT32[TRANSFORM_FORWARD][k][ 3]*O[ 3] +
                       g_aiT32[TRANSFORM_FORWARD][k][ 4]*O[ 4] + g_aiT32[TRANSFORM_FORWARD][k][ 5]*O[ 5] +
                       g_aiT32[TRANSFORM_FORWARD][k][ 6]*O[ 6] + g_aiT32[TRANSFORM_FORWARD][k][ 7]*O[ 7] +
                       g_aiT32[TRANSFORM_FORWARD][k][ 8]*O[ 8] + g_aiT32[TRANSFORM_FORWARD][k][ 9]*O[ 9] +
                       g_aiT32[TRANSFORM_FORWARD][k][10]*O[10] + g_aiT32[TRANSFORM_FORWARD][k][11]*O[11] +
                       g_aiT32[TRANSFORM_FORWARD][k][12]*O[12] + g_aiT32[TRANSFORM_FORWARD][k][13]*O[13] +
                       g_aiT32[TRANSFORM_FORWARD][k][14]*O[14] + g_aiT32[TRANSFORM_FORWARD][k][15]*O[15] + add)>>shift;
    }

    src += 32;
    dst ++;
  }
}

/** 32x32 inverse transform implemented using partial butterfly structure (1D)
 *  \param src   input data (transform coefficients)
 *  \param dst   output data (residual)
 *  \param shift specifies right shift after 1D transform
 *  \param line
 *  \param outputMinimum  minimum for clipping
 *  \param outputMaximum  maximum for clipping
 */
Void partialButterflyInverse32(TCoeff *src, TCoeff *dst, Int shift, Int line, const TCoeff outputMinimum, const TCoeff outputMaximum)
{//32系数　反变换蝶形快速算法
  Int j,k;
  TCoeff E[16],O[16];
  TCoeff EE[8],EO[8];
  TCoeff EEE[4],EEO[4];
  TCoeff EEEE[2],EEEO[2];
  TCoeff add = (shift > 0) ? (1<<(shift-1)) : 0;

  for (j=0; j<line; j++)
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    for (k=0;k<16;k++)
    {
      O[k] = g_aiT32[TRANSFORM_INVERSE][ 1][k]*src[ line    ] + g_aiT32[TRANSFORM_INVERSE][ 3][k]*src[ 3*line  ] +
             g_aiT32[TRANSFORM_INVERSE][ 5][k]*src[ 5*line  ] + g_aiT32[TRANSFORM_INVERSE][ 7][k]*src[ 7*line  ] +
             g_aiT32[TRANSFORM_INVERSE][ 9][k]*src[ 9*line  ] + g_aiT32[TRANSFORM_INVERSE][11][k]*src[ 11*line ] +
             g_aiT32[TRANSFORM_INVERSE][13][k]*src[ 13*line ] + g_aiT32[TRANSFORM_INVERSE][15][k]*src[ 15*line ] +
             g_aiT32[TRANSFORM_INVERSE][17][k]*src[ 17*line ] + g_aiT32[TRANSFORM_INVERSE][19][k]*src[ 19*line ] +
             g_aiT32[TRANSFORM_INVERSE][21][k]*src[ 21*line ] + g_aiT32[TRANSFORM_INVERSE][23][k]*src[ 23*line ] +
             g_aiT32[TRANSFORM_INVERSE][25][k]*src[ 25*line ] + g_aiT32[TRANSFORM_INVERSE][27][k]*src[ 27*line ] +
             g_aiT32[TRANSFORM_INVERSE][29][k]*src[ 29*line ] + g_aiT32[TRANSFORM_INVERSE][31][k]*src[ 31*line ];
    }
    for (k=0;k<8;k++)
    {
      EO[k] = g_aiT32[TRANSFORM_INVERSE][ 2][k]*src[ 2*line  ] + g_aiT32[TRANSFORM_INVERSE][ 6][k]*src[ 6*line  ] +
              g_aiT32[TRANSFORM_INVERSE][10][k]*src[ 10*line ] + g_aiT32[TRANSFORM_INVERSE][14][k]*src[ 14*line ] +
              g_aiT32[TRANSFORM_INVERSE][18][k]*src[ 18*line ] + g_aiT32[TRANSFORM_INVERSE][22][k]*src[ 22*line ] +
              g_aiT32[TRANSFORM_INVERSE][26][k]*src[ 26*line ] + g_aiT32[TRANSFORM_INVERSE][30][k]*src[ 30*line ];
    }
    for (k=0;k<4;k++)
    {
      EEO[k] = g_aiT32[TRANSFORM_INVERSE][ 4][k]*src[  4*line ] + g_aiT32[TRANSFORM_INVERSE][12][k]*src[ 12*line ] +
               g_aiT32[TRANSFORM_INVERSE][20][k]*src[ 20*line ] + g_aiT32[TRANSFORM_INVERSE][28][k]*src[ 28*line ];
    }
    EEEO[0] = g_aiT32[TRANSFORM_INVERSE][8][0]*src[ 8*line ] + g_aiT32[TRANSFORM_INVERSE][24][0]*src[ 24*line ];
    EEEO[1] = g_aiT32[TRANSFORM_INVERSE][8][1]*src[ 8*line ] + g_aiT32[TRANSFORM_INVERSE][24][1]*src[ 24*line ];
    EEEE[0] = g_aiT32[TRANSFORM_INVERSE][0][0]*src[ 0      ] + g_aiT32[TRANSFORM_INVERSE][16][0]*src[ 16*line ];
    EEEE[1] = g_aiT32[TRANSFORM_INVERSE][0][1]*src[ 0      ] + g_aiT32[TRANSFORM_INVERSE][16][1]*src[ 16*line ];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    EEE[0] = EEEE[0] + EEEO[0];
    EEE[3] = EEEE[0] - EEEO[0];
    EEE[1] = EEEE[1] + EEEO[1];
    EEE[2] = EEEE[1] - EEEO[1];
    for (k=0;k<4;k++)
    {
      EE[k] = EEE[k] + EEO[k];
      EE[k+4] = EEE[3-k] - EEO[3-k];
    }
    for (k=0;k<8;k++)
    {
      E[k] = EE[k] + EO[k];
      E[k+8] = EE[7-k] - EO[7-k];
    }
    for (k=0;k<16;k++)
    {
      dst[k]    = Clip3( outputMinimum, outputMaximum, (E[k] + O[k] + add)>>shift );
      dst[k+16] = Clip3( outputMinimum, outputMaximum, (E[15-k] - O[15-k] + add)>>shift );
    }
    src ++;
    dst += 32;
  }
}

/** MxN forward transform (2D)
*  \param bitDepth              [in]  bit depth
*  \param block                 [in]  residual block
*  \param coeff                 [out] transform coefficients
*  \param iWidth                [in]  width of transform
*  \param iHeight               [in]  height of transform
*  \param useDST                [in]
*  \param maxLog2TrDynamicRange [in]

*/
Void xTrMxN(Int bitDepth, TCoeff *block, TCoeff *coeff, Int iWidth, Int iHeight, Bool useDST, const Int maxLog2TrDynamicRange)//系数的二维变换（可分解两次一维变换）
{
  const Int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_FORWARD];

  const Int shift_1st = ((g_aucConvertToBit[iWidth] + 2) +  bitDepth + TRANSFORM_MATRIX_SHIFT) - maxLog2TrDynamicRange;
  const Int shift_2nd = (g_aucConvertToBit[iHeight] + 2) + TRANSFORM_MATRIX_SHIFT;

  assert(shift_1st >= 0);
  assert(shift_2nd >= 0);

  TCoeff tmp[ MAX_TU_SIZE * MAX_TU_SIZE ];

  switch (iWidth)//先对像素块的行做一维DCT　
  {
    case 4:
      {
        if ((iHeight == 4) && useDST)    // Check for DCT or DST
        {
           fastForwardDst( block, tmp, shift_1st );
        }
        else
        {
          partialButterfly4 ( block, tmp, shift_1st, iHeight );
        }
      }
      break;

    case 8:     partialButterfly8 ( block, tmp, shift_1st, iHeight );  break;
    case 16:    partialButterfly16( block, tmp, shift_1st, iHeight );  break;
    case 32:    partialButterfly32( block, tmp, shift_1st, iHeight );  break;
    default:
      assert(0); exit (1); break;
  }//得到变换的中间结果tmp

  switch (iHeight)//在对像素块的列做一维DCT
  {
    case 4:
      {
        if ((iWidth == 4) && useDST)    // Check for DCT or DST
        {
          fastForwardDst( tmp, coeff, shift_2nd );
        }
        else
        {
          partialButterfly4 ( tmp, coeff, shift_2nd, iWidth );
        }
      }
      break;

    case 8:     partialButterfly8 ( tmp, coeff, shift_2nd, iWidth );    break;
    case 16:    partialButterfly16( tmp, coeff, shift_2nd, iWidth );    break;
    case 32:    partialButterfly32( tmp, coeff, shift_2nd, iWidth );    break;
    default:
      assert(0); exit (1); break;
  }
}//得到变换的最终结果coeff


/** MxN inverse transform (2D)
*  \param bitDepth              [in]  bit depth
*  \param coeff                 [in]  transform coefficients
*  \param block                 [out] residual block
*  \param iWidth                [in]  width of transform
*  \param iHeight               [in]  height of transform
*  \param useDST                [in]
*  \param maxLog2TrDynamicRange [in]
*/
Void xITrMxN(Int bitDepth, TCoeff *coeff, TCoeff *block, Int iWidth, Int iHeight, Bool useDST, const Int maxLog2TrDynamicRange)
{//反变换实现同正变换　不在叙述
  const Int TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_INVERSE];

  Int shift_1st = TRANSFORM_MATRIX_SHIFT + 1; //1 has been added to shift_1st at the expense of shift_2nd
  Int shift_2nd = (TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1) - bitDepth;
  const TCoeff clipMinimum = -(1 << maxLog2TrDynamicRange);
  const TCoeff clipMaximum =  (1 << maxLog2TrDynamicRange) - 1;

  assert(shift_1st >= 0);
  assert(shift_2nd >= 0);

  TCoeff tmp[MAX_TU_SIZE * MAX_TU_SIZE];

  switch (iHeight)
  {
    case 4:
      {
        if ((iWidth == 4) && useDST)    // Check for DCT or DST
        {
          fastInverseDst( coeff, tmp, shift_1st, clipMinimum, clipMaximum);
        }
        else
        {
          partialButterflyInverse4 ( coeff, tmp, shift_1st, iWidth, clipMinimum, clipMaximum);
        }
      }
      break;

    case  8: partialButterflyInverse8 ( coeff, tmp, shift_1st, iWidth, clipMinimum, clipMaximum); break;
    case 16: partialButterflyInverse16( coeff, tmp, shift_1st, iWidth, clipMinimum, clipMaximum); break;
    case 32: partialButterflyInverse32( coeff, tmp, shift_1st, iWidth, clipMinimum, clipMaximum); break;

    default:
      assert(0); exit (1); break;
  }

  switch (iWidth)
  {
    // Clipping here is not in the standard, but is used to protect the "Pel" data type into which the inverse-transformed samples will be copied
    case 4:
      {
        if ((iHeight == 4) && useDST)    // Check for DCT or DST
        {
          fastInverseDst( tmp, block, shift_2nd, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max() );
        }
        else
        {
          partialButterflyInverse4 ( tmp, block, shift_2nd, iHeight, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max());
        }
      }
      break;

    case  8: partialButterflyInverse8 ( tmp, block, shift_2nd, iHeight, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max()); break;
    case 16: partialButterflyInverse16( tmp, block, shift_2nd, iHeight, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max()); break;
    case 32: partialButterflyInverse32( tmp, block, shift_2nd, iHeight, std::numeric_limits<Pel>::min(), std::numeric_limits<Pel>::max()); break;

    default:
      assert(0); exit (1); break;
  }
}


// To minimize the distortion only. No rate is considered.
Void TComTrQuant::signBitHidingHDQ( TCoeff* pQCoef, TCoeff* pCoef, TCoeff* deltaU, const TUEntropyCodingParameters &codingParameters, const Int maxLog2TrDynamicRange )
{//SDH技术　首先计算CG中所有非零系数幅值绝对值之和　然后对和值奇偶判断　若为偶数　则第一个非零系数的符号倍判断为＋　否则为－　这样可以节省第一个非零系数coeff_sign_flag的熵编码
  const UInt width     = codingParameters.widthInGroups  << MLS_CG_LOG2_WIDTH;//以CG为单位的宽
  const UInt height    = codingParameters.heightInGroups << MLS_CG_LOG2_HEIGHT;//以CG为单位的高
  const UInt groupSize = 1 << MLS_CG_SIZE;//CG中系数个数

  const TCoeff entropyCodingMinimum = -(1 << maxLog2TrDynamicRange);
  const TCoeff entropyCodingMaximum =  (1 << maxLog2TrDynamicRange) - 1;

  Int lastCG = -1;
  Int absSum = 0 ;
  Int n ;

  for( Int subSet = (width*height-1) >> MLS_CG_SIZE; subSet >= 0; subSet-- )//遍历所有Tu中所有CG
  {
    Int  subPos = subSet << MLS_CG_SIZE;
    Int  firstNZPosInCG=groupSize , lastNZPosInCG=-1 ;//初始化
    absSum = 0 ;

    for(n = groupSize-1; n >= 0; --n )//从后向前遍历CG中所有系数　寻找CG中最后一个非零系数
    {
      if( pQCoef[ codingParameters.scan[ n + subPos ]] )//如果找到非零系数
      {
        lastNZPosInCG = n;
        break;
      }
    }

    for(n = 0; n <groupSize; n++ )//从前向后遍历CG中所有系数　寻找CG中第一个一个非零系数
    {
      if( pQCoef[ codingParameters.scan[ n + subPos ]] )
      {
        firstNZPosInCG = n;
        break;
      }
    }

    for(n = firstNZPosInCG; n <=lastNZPosInCG; n++ )//计算CG中系数绝对值之和
    {
      absSum += Int(pQCoef[ codingParameters.scan[ n + subPos ]]);
    }

    if(lastNZPosInCG>=0 && lastCG==-1)//存在最后一个非零系数　lastCG置１
    {
      lastCG = 1 ;
    }

    if( lastNZPosInCG-firstNZPosInCG>=SBH_THRESHOLD )//第一个非零系数和最后一个非零系数之间的间隔大于４时　该CG才能省略第一个非零系数符号的熵编码
    {
      UInt signbit = (pQCoef[codingParameters.scan[subPos+firstNZPosInCG]]>0?0:1) ;//第一个非零系数符号
      if( signbit!=(absSum&0x1) )  //compare signbit with sum_parity　SDH结果与CG中第一个非零系数结果不一致
      {
        TCoeff curCost    = std::numeric_limits<TCoeff>::max();//初始化需要用到的参数
        TCoeff minCostInc = std::numeric_limits<TCoeff>::max();
        Int minPos =-1, finalChange=0, curChange=0;
        //在一个CG中　计算原始系数值和反量化值之间的差值，对差值最大的量化值进行修正
        for( n = (lastCG==1?lastNZPosInCG:groupSize-1) ; n >= 0; --n )//遍历CG中所有系数　若存在最后一个非零系数则从最后一个非零系数开始向前遍历
        {
          UInt blkPos   = codingParameters.scan[ n+subPos ];
          if(pQCoef[ blkPos ] != 0 )//量化值不为０
          {
            if(deltaU[blkPos]>0)//原始系数值比反量化值大
            {
              curCost = - deltaU[blkPos];//修正后的误差（相对值　只用于做比较）
              curChange=1 ;//修正值为量化值加１
            }
            else//原始系数值比反量化值大
            {
              //curChange =-1;
              if(n==firstNZPosInCG && abs(pQCoef[blkPos])==1)
              {
                curCost = std::numeric_limits<TCoeff>::max();
              }
              else
              {
                curCost = deltaU[blkPos];//修正后的误差（相对值　只用于做比较）
                curChange =-1;//修正值为量化值减１
              }
            }
          }
          else//量化值为０ 修正量化值只能加１（因为原始系数值一定比反量化值大）
          {
            if(n<firstNZPosInCG)
            {
              UInt thisSignBit = (pCoef[blkPos]>=0?0:1);
              if(thisSignBit != signbit )
              {
                curCost = std::numeric_limits<TCoeff>::max();
              }
              else
              {
                curCost = - (deltaU[blkPos])  ;
                curChange = 1 ;
              }
            }
            else
            {
              curCost = - (deltaU[blkPos])  ;
              curChange = 1 ;
            }
          }

          if( curCost<minCostInc)//比较各系数修正后的误差　选择修正后误差最小的系数进行修正
          {
            minCostInc = curCost ;
            finalChange = curChange ;
            minPos = blkPos ;
          }
        } //CG loop

        if(pQCoef[minPos] == entropyCodingMaximum || pQCoef[minPos] == entropyCodingMinimum)//系数达到最值只能减１
        {
          finalChange = -1;
        }

        if(pCoef[minPos]>=0)//系数正负影响最终修正值
        {
          pQCoef[minPos] += finalChange ;
        }
        else
        {
          pQCoef[minPos] -= finalChange ;
        }
      } // Hide
    }
    if(lastCG==1)
    {
      lastCG=0 ;
    }
  } // TU loop

  return;
}


Void TComTrQuant::xQuant(       TComTU       &rTu,
                                TCoeff      * pSrc,
                                TCoeff      * pDes,
#if ADAPTIVE_QP_SELECTION
                                TCoeff      *pArlDes,
#endif
                                TCoeff       &uiAbsSum,
                          const ComponentID   compID,
                          const QpParam      &cQP )
{//对变化后的系数进行量化
  const TComRectangle &rect = rTu.getRect(compID);//Tu块位置
  const UInt uiWidth        = rect.width;
  const UInt uiHeight       = rect.height;
  TComDataCU* pcCU          = rTu.getCU();
  const UInt uiAbsPartIdx   = rTu.GetAbsPartIdxTU();
  const Int channelBitDepth = pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID));//通道位深

  TCoeff* piCoef    = pSrc;
  TCoeff* piQCoef   = pDes;
#if ADAPTIVE_QP_SELECTION
  TCoeff* piArlCCoef = pArlDes;
#endif

  const Bool useTransformSkip      = pcCU->getTransformSkip(uiAbsPartIdx, compID);
  const Int  maxLog2TrDynamicRange = pcCU->getSlice()->getSPS()->getMaxLog2TrDynamicRange(toChannelType(compID));

  Bool useRDOQ = useTransformSkip ? m_useRDOQTS : m_useRDOQ;
  if ( useRDOQ && (isLuma(compID) || RDOQ_CHROMA) )//如果使用率失真优化量化
  {
#if T0196_SELECTIVE_RDOQ
    if ( !m_useSelectiveRDOQ || xNeedRDOQ( rTu, piCoef, compID, cQP ) )
    {
#endif
#if ADAPTIVE_QP_SELECTION
      xRateDistOptQuant( rTu, piCoef, pDes, pArlDes, uiAbsSum, compID, cQP );//率失真优化量化
#else
      xRateDistOptQuant( rTu, piCoef, pDes, uiAbsSum, compID, cQP );
#endif
#if T0196_SELECTIVE_RDOQ
    }
    else
    {
      memset( pDes, 0, sizeof( TCoeff ) * uiWidth *uiHeight );
      uiAbsSum = 0;
    }
#endif
  }
  else//普通量化（不使用率失真优化量化）
  {
    TUEntropyCodingParameters codingParameters;
    getTUEntropyCodingParameters(codingParameters, rTu, compID);

    const TCoeff entropyCodingMinimum = -(1 << maxLog2TrDynamicRange);
    const TCoeff entropyCodingMaximum =  (1 << maxLog2TrDynamicRange) - 1;

    TCoeff deltaU[MAX_TU_SIZE * MAX_TU_SIZE];

    const UInt uiLog2TrSize = rTu.GetEquivalentLog2TrSize(compID);//Tu块log2大小

    Int scalingListType = getScalingListType(pcCU->getPredictionMode(uiAbsPartIdx), compID);//量化矩阵类型
    assert(scalingListType < SCALING_LIST_NUM);
    Int *piQuantCoeff = getQuantCoeff(scalingListType, cQP.rem, uiLog2TrSize-2);//QP对应的量化系数地址

    const Bool enableScalingLists             = getUseScalingList(uiWidth, uiHeight, (pcCU->getTransformSkip(uiAbsPartIdx, compID) != 0));//是否使用量化矩阵
    const Int  defaultQuantisationCoefficient = g_quantScales[cQP.rem];//不使用量化矩阵QP对应的量化系数

    /* for 422 chroma blocks, the effective scaling applied during transformation is not a power of 2, hence it cannot be
     * implemented as a bit-shift (the quantised result will be sqrt(2) * larger than required). Alternatively, adjust the
     * uiLog2TrSize applied in iTransformShift, such that the result is 1/sqrt(2) the required result (i.e. smaller)
     * Then a QP+3 (sqrt(2)) or QP-3 (1/sqrt(2)) method could be used to get the required result
     */

    // Represents scaling through forward transform
    Int iTransformShift = getTransformShift(channelBitDepth, uiLog2TrSize, maxLog2TrDynamicRange);//变化移位量
    if (useTransformSkip && pcCU->getSlice()->getSPS()->getSpsRangeExtension().getExtendedPrecisionProcessingFlag())//若使用TransformSkip和精度扩展　
    {
      iTransformShift = std::max<Int>(0, iTransformShift);//则iTransformShift一定为正
    }

    const Int iQBits = QUANT_SHIFT + cQP.per + iTransformShift;//乘以量化系数后的右移位量
    // QBits will be OK for any internal bit depth as the reduction in transform shift is balanced by an increase in Qp_per due to QpBDOffset

#if ADAPTIVE_QP_SELECTION
    Int iQBitsC = MAX_INT;
    Int iAddC   = MAX_INT;

    if (m_bUseAdaptQpSelect)
    {
      iQBitsC = iQBits - ARL_C_PRECISION;
      iAddC   = 1 << (iQBitsC-1);
    }
#endif

    const Int iAdd   = (pcCU->getSlice()->getSliceType()==I_SLICE ? 171 : 85) << (iQBits-9);//舍入偏移量 I 1/3 P/B 1/6
    const Int qBits8 = iQBits - 8;

    for( Int uiBlockPos = 0; uiBlockPos < uiWidth*uiHeight; uiBlockPos++ )//遍历所有系数
    {
      const TCoeff iLevel   = piCoef[uiBlockPos];
      const TCoeff iSign    = (iLevel < 0 ? -1: 1);

      const Int64  tmpLevel = (Int64)abs(iLevel) * (enableScalingLists ? piQuantCoeff[uiBlockPos] : defaultQuantisationCoefficient);//量化中间值(dij*MF)

#if ADAPTIVE_QP_SELECTION
      if( m_bUseAdaptQpSelect )
      {
        piArlCCoef[uiBlockPos] = (TCoeff)((tmpLevel + iAddC ) >> iQBitsC);
      }
#endif

      const TCoeff quantisedMagnitude = TCoeff((tmpLevel + iAdd ) >> iQBits);//最终量化值绝对值　(dij*MF＋f')>>(qbits+T_shift)
      deltaU[uiBlockPos] = (TCoeff)((tmpLevel - (quantisedMagnitude<<iQBits) )>> qBits8);//量化产生的误差

      uiAbsSum += quantisedMagnitude;//量化值绝对值之和
      const TCoeff quantisedCoefficient = quantisedMagnitude * iSign;//最终量化值（量化绝对值乘上量化值符号）

      piQCoef[uiBlockPos] = Clip3<TCoeff>( entropyCodingMinimum, entropyCodingMaximum, quantisedCoefficient );//限制量化值大小
    } // for n

    if( pcCU->getSlice()->getPPS()->getSignHideFlag() )
    {
      if(uiAbsSum >= 2) //this prevents TUs with only one coefficient of value 1 from being tested
      {
        signBitHidingHDQ( piQCoef, piCoef, deltaU, codingParameters, maxLog2TrDynamicRange ) ;//SDH技术　隐藏最后一个非零量化值的符号位　节约编码bit位
      }
    }
  } //if RDOQ
  //return;
}

#if T0196_SELECTIVE_RDOQ
Bool TComTrQuant::xNeedRDOQ( TComTU &rTu, TCoeff * pSrc, const ComponentID compID, const QpParam &cQP )
{//判断变化后的系数是否需要率失真优化量化
  const TComRectangle &rect = rTu.getRect(compID);//Tu块位置
  const UInt uiWidth        = rect.width;
  const UInt uiHeight       = rect.height;
  TComDataCU* pcCU          = rTu.getCU();
  const UInt uiAbsPartIdx   = rTu.GetAbsPartIdxTU();
  const Int channelBitDepth = pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID));//通道位深

  TCoeff* piCoef    = pSrc;

  const Bool useTransformSkip      = pcCU->getTransformSkip(uiAbsPartIdx, compID);
  const Int  maxLog2TrDynamicRange = pcCU->getSlice()->getSPS()->getMaxLog2TrDynamicRange(toChannelType(compID));

  const UInt uiLog2TrSize = rTu.GetEquivalentLog2TrSize(compID);

  Int scalingListType = getScalingListType(pcCU->getPredictionMode(uiAbsPartIdx), compID);//量化矩阵类型
  assert(scalingListType < SCALING_LIST_NUM);
  Int *piQuantCoeff = getQuantCoeff(scalingListType, cQP.rem, uiLog2TrSize-2);//QP对应的量化系数地址

  const Bool enableScalingLists             = getUseScalingList(uiWidth, uiHeight, (pcCU->getTransformSkip(uiAbsPartIdx, compID) != 0));//是否使用量化矩阵
  const Int  defaultQuantisationCoefficient = g_quantScales[cQP.rem];//不使用量化矩阵QP对应的量化系数

  /* for 422 chroma blocks, the effective scaling applied during transformation is not a power of 2, hence it cannot be
    * implemented as a bit-shift (the quantised result will be sqrt(2) * larger than required). Alternatively, adjust the
    * uiLog2TrSize applied in iTransformShift, such that the result is 1/sqrt(2) the required result (i.e. smaller)
    * Then a QP+3 (sqrt(2)) or QP-3 (1/sqrt(2)) method could be used to get the required result
    */

  // Represents scaling through forward transform
  Int iTransformShift = getTransformShift(channelBitDepth, uiLog2TrSize, maxLog2TrDynamicRange);
  if (useTransformSkip && pcCU->getSlice()->getSPS()->getSpsRangeExtension().getExtendedPrecisionProcessingFlag())
  {
    iTransformShift = std::max<Int>(0, iTransformShift);
  }

  const Int iQBits = QUANT_SHIFT + cQP.per + iTransformShift;
  // QBits will be OK for any internal bit depth as the reduction in transform shift is balanced by an increase in Qp_per due to QpBDOffset

  // iAdd is different from the iAdd used in normal quantization
  const Int iAdd   = (compID == COMPONENT_Y ? 171 : 256) << (iQBits-9);//舍入偏移量 I 1/3 P/B 1/6

  for( Int uiBlockPos = 0; uiBlockPos < uiWidth*uiHeight; uiBlockPos++ )//遍历所有系数
  {
    const TCoeff iLevel   = piCoef[uiBlockPos];
    const Int64  tmpLevel = (Int64)abs(iLevel) * (enableScalingLists ? piQuantCoeff[uiBlockPos] : defaultQuantisationCoefficient);
    const TCoeff quantisedMagnitude = TCoeff((tmpLevel + iAdd ) >> iQBits);

    if ( quantisedMagnitude != 0 )//如果该Tu存在非零量化值　则需要率失真优化量化　否则不需要
    {
      return true;
    }
  } // for n
  return false;
}
#endif

Void TComTrQuant::xDeQuant(       TComTU        &rTu,
                            const TCoeff       * pSrc,
                                  TCoeff       * pDes,
                            const ComponentID    compID,
                            const QpParam       &cQP )
{//反量化过程较量化过程简单　且与普通量化过程基本相同　故不再叙述
  assert(compID<MAX_NUM_COMPONENT);

        TComDataCU          *pcCU               = rTu.getCU();
  const UInt                 uiAbsPartIdx       = rTu.GetAbsPartIdxTU();
  const TComRectangle       &rect               = rTu.getRect(compID);
  const UInt                 uiWidth            = rect.width;
  const UInt                 uiHeight           = rect.height;
  const TCoeff        *const piQCoef            = pSrc;
        TCoeff        *const piCoef             = pDes;
  const UInt                 uiLog2TrSize       = rTu.GetEquivalentLog2TrSize(compID);
  const UInt                 numSamplesInBlock  = uiWidth*uiHeight;
  const Int                  maxLog2TrDynamicRange  = pcCU->getSlice()->getSPS()->getMaxLog2TrDynamicRange(toChannelType(compID));
  const TCoeff               transformMinimum   = -(1 << maxLog2TrDynamicRange);
  const TCoeff               transformMaximum   =  (1 << maxLog2TrDynamicRange) - 1;
  const Bool                 enableScalingLists = getUseScalingList(uiWidth, uiHeight, (pcCU->getTransformSkip(uiAbsPartIdx, compID) != 0));
  const Int                  scalingListType    = getScalingListType(pcCU->getPredictionMode(uiAbsPartIdx), compID);
#if O0043_BEST_EFFORT_DECODING
  const Int                  channelBitDepth    = pcCU->getSlice()->getSPS()->getStreamBitDepth(toChannelType(compID));
#else
  const Int                  channelBitDepth    = pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID));
#endif

  assert (scalingListType < SCALING_LIST_NUM);
  assert ( uiWidth <= m_uiMaxTrSize );

  // Represents scaling through forward transform
  const Bool bClipTransformShiftTo0 = (pcCU->getTransformSkip(uiAbsPartIdx, compID) != 0) && pcCU->getSlice()->getSPS()->getSpsRangeExtension().getExtendedPrecisionProcessingFlag();
  const Int  originalTransformShift = getTransformShift(channelBitDepth, uiLog2TrSize, maxLog2TrDynamicRange);
  const Int  iTransformShift        = bClipTransformShiftTo0 ? std::max<Int>(0, originalTransformShift) : originalTransformShift;

  const Int QP_per = cQP.per;
  const Int QP_rem = cQP.rem;

  const Int rightShift = (IQUANT_SHIFT - (iTransformShift + QP_per)) + (enableScalingLists ? LOG2_SCALING_LIST_NEUTRAL_VALUE : 0);

  if(enableScalingLists)
  {
    //from the dequantisation equation:
    //iCoeffQ                         = ((Intermediate_Int(clipQCoef) * piDequantCoef[deQuantIdx]) + iAdd ) >> rightShift
    //(sizeof(Intermediate_Int) * 8)  =              inputBitDepth    +    dequantCoefBits                   - rightShift
    const UInt             dequantCoefBits     = 1 + IQUANT_SHIFT + SCALING_LIST_BITS;
    const UInt             targetInputBitDepth = std::min<UInt>((maxLog2TrDynamicRange + 1), (((sizeof(Intermediate_Int) * 8) + rightShift) - dequantCoefBits));

    const Intermediate_Int inputMinimum        = -(1 << (targetInputBitDepth - 1));
    const Intermediate_Int inputMaximum        =  (1 << (targetInputBitDepth - 1)) - 1;

    Int *piDequantCoef = getDequantCoeff(scalingListType,QP_rem,uiLog2TrSize-2);

    if(rightShift > 0)
    {
      const Intermediate_Int iAdd = 1 << (rightShift - 1);

      for( Int n = 0; n < numSamplesInBlock; n++ )
      {
        const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
        const Intermediate_Int iCoeffQ   = ((Intermediate_Int(clipQCoef) * piDequantCoef[n]) + iAdd ) >> rightShift;

        piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
      }
    }
    else
    {
      const Int leftShift = -rightShift;

      for( Int n = 0; n < numSamplesInBlock; n++ )
      {
        const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
        const Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * piDequantCoef[n]) << leftShift;

        piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
      }
    }
  }
  else
  {
    const Int scale     =  g_invQuantScales[QP_rem];
    const Int scaleBits =     (IQUANT_SHIFT + 1)   ;

    //from the dequantisation equation:
    //iCoeffQ                         = Intermediate_Int((Int64(clipQCoef) * scale + iAdd) >> rightShift);
    //(sizeof(Intermediate_Int) * 8)  =                    inputBitDepth   + scaleBits      - rightShift
    const UInt             targetInputBitDepth = std::min<UInt>((maxLog2TrDynamicRange + 1), (((sizeof(Intermediate_Int) * 8) + rightShift) - scaleBits));
    const Intermediate_Int inputMinimum        = -(1 << (targetInputBitDepth - 1));
    const Intermediate_Int inputMaximum        =  (1 << (targetInputBitDepth - 1)) - 1;

    if (rightShift > 0)
    {
      const Intermediate_Int iAdd = 1 << (rightShift - 1);

      for( Int n = 0; n < numSamplesInBlock; n++ )
      {
        const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
        const Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * scale + iAdd) >> rightShift;

        piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
      }
    }
    else
    {
      const Int leftShift = -rightShift;

      for( Int n = 0; n < numSamplesInBlock; n++ )
      {
        const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
        const Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * scale) << leftShift;

        piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
      }
    }
  }
}


Void TComTrQuant::init(   UInt  uiMaxTrSize,
                          Bool  bUseRDOQ,
                          Bool  bUseRDOQTS,
#if T0196_SELECTIVE_RDOQ
                          Bool  useSelectiveRDOQ,
#endif
                          Bool  bEnc,
                          Bool  useTransformSkipFast
#if ADAPTIVE_QP_SELECTION
                        , Bool bUseAdaptQpSelect
#endif
                       )
{
  m_uiMaxTrSize  = uiMaxTrSize;
  m_bEnc         = bEnc;
  m_useRDOQ      = bUseRDOQ;
  m_useRDOQTS    = bUseRDOQTS;
#if T0196_SELECTIVE_RDOQ
  m_useSelectiveRDOQ = useSelectiveRDOQ;
#endif
#if ADAPTIVE_QP_SELECTION
  m_bUseAdaptQpSelect = bUseAdaptQpSelect;
#endif
  m_useTransformSkipFast = useTransformSkipFast;
}


Void TComTrQuant::transformNxN(       TComTU        & rTu,
                                const ComponentID     compID,
                                      Pel          *  pcResidual,
                                const UInt            uiStride,
                                      TCoeff       *  rpcCoeff,
#if ADAPTIVE_QP_SELECTION
                                      TCoeff       *  pcArlCoeff,
#endif
                                      TCoeff        & uiAbsSum,
                                const QpParam       & cQP
                              )
{//N*N Tu块对残差值进行变换量化得到最终量化值
  const TComRectangle &rect = rTu.getRect(compID);
  const UInt uiWidth        = rect.width;
  const UInt uiHeight       = rect.height;
  TComDataCU* pcCU          = rTu.getCU();
  const UInt uiAbsPartIdx   = rTu.GetAbsPartIdxTU();
  const UInt uiOrgTrDepth   = rTu.GetTransformDepthRel();

  uiAbsSum=0;//初始化量化值系数之和为０

  RDPCMMode rdpcmMode = RDPCM_OFF;//初始化PCM模式为RDPCM_OFF
  rdpcmNxN( rTu, compID, pcResidual, uiStride, cQP, rpcCoeff, uiAbsSum, rdpcmMode );//计算该TU块最优PCM并赋给rdpcmMode　若rdpcmMode ！= RDPCM_OFF　则rpcCoeff为最终量化值

  if (rdpcmMode == RDPCM_OFF)//若不使用PCM模式　则需要进行变换和量化
  {
    uiAbsSum = 0;
    //transform and quantise
    if(pcCU->getCUTransquantBypass(uiAbsPartIdx))//如果该CU为lossless模式　不需要变换及量化
    {
      const Bool rotateResidual = rTu.isNonTransformedResidualRotated(compID);
      const UInt uiSizeMinus1   = (uiWidth * uiHeight) - 1;

      for (UInt y = 0, coefficientIndex = 0; y<uiHeight; y++)//依次遍历所有残差值
      {
        for (UInt x = 0; x<uiWidth; x++, coefficientIndex++)
        {
          const Pel currentSample = pcResidual[(y * uiStride) + x];

          rpcCoeff[rotateResidual ? (uiSizeMinus1 - coefficientIndex) : coefficientIndex] = currentSample;//原始残差值即为最终值
          uiAbsSum += TCoeff(abs(currentSample));
        }
      }
    }
    else//不为lossless模式
    {
#if DEBUG_TRANSFORM_AND_QUANTISE
      std::cout << g_debugCounter << ": " << uiWidth << "x" << uiHeight << " channel " << compID << " TU at input to transform\n";
      printBlock(pcResidual, uiWidth, uiHeight, uiStride);
#endif

      assert( (pcCU->getSlice()->getSPS()->getMaxTrSize() >= uiWidth) );

      if(pcCU->getTransformSkip(uiAbsPartIdx, compID) != 0)///为TransformSkip模式　（不需要变换）　
      {
        xTransformSkip( pcResidual, uiStride, m_plTempCoeff, rTu, compID );//执行TransformSkip模式
      }
      else//否则对残差值变换
      {
        const Int channelBitDepth=pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID));
        xT( channelBitDepth, rTu.useDST(compID), pcResidual, uiStride, m_plTempCoeff, uiWidth, uiHeight, pcCU->getSlice()->getSPS()->getMaxLog2TrDynamicRange(toChannelType(compID)) );//对残差值做变换
      }

#if DEBUG_TRANSFORM_AND_QUANTISE
      std::cout << g_debugCounter << ": " << uiWidth << "x" << uiHeight << " channel " << compID << " TU between transform and quantiser\n";
      printBlock(m_plTempCoeff, uiWidth, uiHeight, uiWidth);
#endif

      xQuant( rTu, m_plTempCoeff, rpcCoeff,

#if ADAPTIVE_QP_SELECTION
              pcArlCoeff,
#endif
              uiAbsSum, compID, cQP );//对变换后的值（包括TransformSkip）做量化

#if DEBUG_TRANSFORM_AND_QUANTISE
      std::cout << g_debugCounter << ": " << uiWidth << "x" << uiHeight << " channel " << compID << " TU at output of quantiser\n";
      printBlock(rpcCoeff, uiWidth, uiHeight, uiWidth);
#endif
    }
  }

    //set the CBF
  pcCU->setCbfPartRange((((uiAbsSum > 0) ? 1 : 0) << uiOrgTrDepth), compID, uiAbsPartIdx, rTu.GetAbsPartIdxNumParts(compID));//设置cbf 表明该Tu块是否存在非零系数
}


Void TComTrQuant::invTransformNxN(      TComTU        &rTu,
                                  const ComponentID    compID,
                                        Pel          *pcResidual,
                                  const UInt           uiStride,
                                        TCoeff       * pcCoeff,
                                  const QpParam       &cQP
                                        DEBUG_STRING_FN_DECLAREP(psDebug))
{//由量化值反量化反变换的得到重建的残差值　过程基本同transformNxN　不再叙述
  TComDataCU* pcCU=rTu.getCU();
  const UInt uiAbsPartIdx = rTu.GetAbsPartIdxTU();
  const TComRectangle &rect = rTu.getRect(compID);
  const UInt uiWidth = rect.width;
  const UInt uiHeight = rect.height;

  if (uiWidth != uiHeight) //for intra, the TU will have been split above this level, so this condition won't be true, hence this only affects inter
    //------------------------------------------------

    //recurse deeper

    TComTURecurse subTURecurse(rTu, false, TComTU::VERTICAL_SPLIT, true, compID);

    do
    {
      //------------------

      const UInt lineOffset = subTURecurse.GetSectionNumber() * subTURecurse.getRect(compID).height;

      Pel    *subTUResidual     = pcResidual + (lineOffset * uiStride);
      TCoeff *subTUCoefficients = pcCoeff     + (lineOffset * subTURecurse.getRect(compID).width);

      invTransformNxN(subTURecurse, compID, subTUResidual, uiStride, subTUCoefficients, cQP DEBUG_STRING_PASS_INTO(psDebug));

      //------------------

    } while (subTURecurse.nextSection(rTu));//存在下一子N*N的TU块

    //------------------------------------------------

    return;
  }

#if DEBUG_STRING
  if (psDebug)
  {
    std::stringstream ss(stringstream::out);
    printBlockToStream(ss, (compID==0)?"###InvTran ip Ch0: " : ((compID==1)?"###InvTran ip Ch1: ":"###InvTran ip Ch2: "), pcCoeff, uiWidth, uiHeight, uiWidth);
    DEBUG_STRING_APPEND((*psDebug), ss.str())
  }
#endif

  if(pcCU->getCUTransquantBypass(uiAbsPartIdx))
  {
    const Bool rotateResidual = rTu.isNonTransformedResidualRotated(compID);
    const UInt uiSizeMinus1   = (uiWidth * uiHeight) - 1;

    for (UInt y = 0, coefficientIndex = 0; y<uiHeight; y++)
    {
      for (UInt x = 0; x<uiWidth; x++, coefficientIndex++)
      {
        pcResidual[(y * uiStride) + x] = Pel(pcCoeff[rotateResidual ? (uiSizeMinus1 - coefficientIndex) : coefficientIndex]);
      }
    }
  }
  else
  {
#if DEBUG_TRANSFORM_AND_QUANTISE
    std::cout << g_debugCounter << ": " << uiWidth << "x" << uiHeight << " channel " << compID << " TU at input to dequantiser\n";
    printBlock(pcCoeff, uiWidth, uiHeight, uiWidth);
#endif

    xDeQuant(rTu, pcCoeff, m_plTempCoeff, compID, cQP);

#if DEBUG_TRANSFORM_AND_QUANTISE
    std::cout << g_debugCounter << ": " << uiWidth << "x" << uiHeight << " channel " << compID << " TU between dequantiser and inverse-transform\n";
    printBlock(m_plTempCoeff, uiWidth, uiHeight, uiWidth);
#endif

#if DEBUG_STRING
    if (psDebug)
    {
      std::stringstream ss(stringstream::out);
      printBlockToStream(ss, "###InvTran deq: ", m_plTempCoeff, uiWidth, uiHeight, uiWidth);
      (*psDebug)+=ss.str();
    }
#endif

    if(pcCU->getTransformSkip(uiAbsPartIdx, compID))
    {
      xITransformSkip( m_plTempCoeff, pcResidual, uiStride, rTu, compID );

#if DEBUG_STRING
      if (psDebug)
      {
        std::stringstream ss(stringstream::out);
        printBlockToStream(ss, "###InvTran resi: ", pcResidual, uiWidth, uiHeight, uiStride);
        (*psDebug)+=ss.str();
        (*psDebug)+="(<- was a Transform-skipped block)\n";
      }
#endif
    }
    else
    {
#if O0043_BEST_EFFORT_DECODING
      const Int channelBitDepth = pcCU->getSlice()->getSPS()->getStreamBitDepth(toChannelType(compID));
#else
      const Int channelBitDepth = pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID));
#endif
      xIT( channelBitDepth, rTu.useDST(compID), m_plTempCoeff, pcResidual, uiStride, uiWidth, uiHeight, pcCU->getSlice()->getSPS()->getMaxLog2TrDynamicRange(toChannelType(compID)) );

#if DEBUG_STRING
      if (psDebug)
      {
        std::stringstream ss(stringstream::out);
        printBlockToStream(ss, "###InvTran resi: ", pcResidual, uiWidth, uiHeight, uiStride);
        (*psDebug)+=ss.str();
        (*psDebug)+="(<- was a Transformed block)\n";
      }
#endif
    }

#if DEBUG_TRANSFORM_AND_QUANTISE
    std::cout << g_debugCounter << ": " << uiWidth << "x" << uiHeight << " channel " << compID << " TU at output of inverse-transform\n";
    printBlock(pcResidual, uiWidth, uiHeight, uiStride);
    g_debugCounter++;
#endif
  }

  invRdpcmNxN( rTu, compID, pcResidual, uiStride );
}

Void TComTrQuant::invRecurTransformNxN( const ComponentID compID,//通道类型
                                        TComYuv *pResidual,//整帧图的残差值
                                        TComTU &rTu)//Tu块信息
{
  if (!rTu.ProcessComponentSection(compID))//该Tu不需要处理
  {
    return;//方法结束
  }

  TComDataCU* pcCU = rTu.getCU();
  UInt absPartIdxTU = rTu.GetAbsPartIdxTU();
  UInt uiTrMode=rTu.GetTransformDepthRel();//Tu相对Cu的深度
  if( (pcCU->getCbf(absPartIdxTU, compID, uiTrMode) == 0) && (isLuma(compID) || !pcCU->getSlice()->getPPS()->getPpsRangeExtension().getCrossComponentPredictionEnabledFlag()) )//如果该Tu不存在非零系数并且该Tb为亮度组成或为使能CCp 则方法结束
  {
    return;
  }

  if( uiTrMode == pcCU->getTransformIdx( absPartIdxTU ) )//该Tu不存在子Tu(与cu同一深度)
  {
    const TComRectangle &tuRect      = rTu.getRect(compID);
    const Int            uiStride    = pResidual->getStride( compID );
          Pel           *rpcResidual = pResidual->getAddr( compID );
          UInt           uiAddr      = (tuRect.x0 + uiStride*tuRect.y0);
          Pel           *pResi       = rpcResidual + uiAddr;
          TCoeff        *pcCoeff     = pcCU->getCoeff(compID) + rTu.getCoefficientOffset(compID);

    const QpParam cQP(*pcCU, compID);

    if(pcCU->getCbf(absPartIdxTU, compID, uiTrMode) != 0)//该Tu存在非零系数
    {
      DEBUG_STRING_NEW(sTemp)
#if DEBUG_STRING
      std::string *psDebug=((DebugOptionList::DebugString_InvTran.getInt()&(pcCU->isIntra(absPartIdxTU)?1:(pcCU->isInter(absPartIdxTU)?2:4)))!=0) ? &sTemp : 0;
#endif

      invTransformNxN( rTu, compID, pResi, uiStride, pcCoeff, cQP DEBUG_STRING_PASS_INTO(psDebug) );//做反变换(包括反量化和反变化)由量化值得到亮度残差值

#if DEBUG_STRING
      if (psDebug != 0)
      {
        std::cout << (*psDebug);
      }
#endif
    }

    if (isChroma(compID) && (pcCU->getCrossComponentPredictionAlpha(absPartIdxTU, compID) != 0))//为色度分量且CCP中alpha不为０　
    {
      const Pel *piResiLuma = pResidual->getAddr( COMPONENT_Y );
      const Int  strideLuma = pResidual->getStride( COMPONENT_Y );
      const Int  tuWidth    = rTu.getRect( compID ).width;
      const Int  tuHeight   = rTu.getRect( compID ).height;

      if(pcCU->getCbf(absPartIdxTU, COMPONENT_Y, uiTrMode) != 0)
      {
        pResi = rpcResidual + uiAddr;
        const Pel *pResiLuma = piResiLuma + uiAddr;

        crossComponentPrediction( rTu, compID, pResiLuma, pResi, pResi, tuWidth, tuHeight, strideLuma, uiStride, uiStride, true );//由CCP　用亮度残差预测得到色度残差
      }
    }
  }
  else
  {
    TComTURecurse tuRecurseChild(rTu, false);
    do
    {
      invRecurTransformNxN( compID, pResidual, tuRecurseChild );
    } while (tuRecurseChild.nextSection(rTu));//该Tu存在子TU 则深度遍历所有子Tu
  }
}

Void TComTrQuant::applyForwardRDPCM( TComTU& rTu, const ComponentID compID, Pel* pcResidual, const UInt uiStride, const QpParam& cQP, TCoeff* pcCoeff, TCoeff &uiAbsSum, const RDPCMMode mode )
{
  TComDataCU *pcCU=rTu.getCU();//TU������CU�����Ϣ
  const UInt uiAbsPartIdx=rTu.GetAbsPartIdxTU();//Tu��λ��

  const Bool bLossless      = pcCU->getCUTransquantBypass( uiAbsPartIdx );//�Ƿ�losslessģʽ ��ģʽ�²���Ҫ�任������
  const UInt uiWidth        = rTu.getRect(compID).width;//Tu��Ŀ�͸�
  const UInt uiHeight       = rTu.getRect(compID).height;
  const Bool rotateResidual = rTu.isNonTransformedResidualRotated(compID);//�Ƿ���ת����ǰ�����ǴӺ���ǰ��
  const UInt uiSizeMinus1   = (uiWidth * uiHeight) - 1;//ϵ��������1

  UInt uiX = 0;
  UInt uiY = 0;
  //����PCMģʽ ȷ�����������ķ���
        UInt &majorAxis             = (mode == RDPCM_VER) ? uiX      : uiY;//���ñ��� C++���� ��ͬ��ֱ�Ӹ�ֵ majorAxis�ĸı�Ҳ��ı�uiX ��uiY
        UInt &minorAxis             = (mode == RDPCM_VER) ? uiY      : uiX;
  const UInt  majorAxisLimit        = (mode == RDPCM_VER) ? uiWidth  : uiHeight;
  const UInt  minorAxisLimit        = (mode == RDPCM_VER) ? uiHeight : uiWidth;

  const Bool bUseHalfRoundingPoint  = (mode != RDPCM_OFF);//使用PCM模式时使用半点精度（小数点四舍五入取整）

  uiAbsSum = 0;

  for ( majorAxis = 0; majorAxis < majorAxisLimit; majorAxis++ )//�������������ÿ��λ���ϵ�ϵ��
  {
    TCoeff accumulatorValue = 0; // 32-bit accumulator
    for ( minorAxis = 0; minorAxis < minorAxisLimit; minorAxis++ )
    {
      const UInt sampleIndex      = (uiY * uiWidth) + uiX;
      const UInt coefficientIndex = (rotateResidual ? (uiSizeMinus1-sampleIndex) : sampleIndex);//ȷ��ϵ����λ��
      const Pel  currentSample    = pcResidual[(uiY * uiStride) + uiX];//��λ���ϵ�ϵ��ֵ ʵ����Ϊÿ�У��У�����λ���ϵ��ۼ�ֵ
      const TCoeff encoderSideDelta = TCoeff(currentSample) - accumulatorValue;//�ۼ�ǰ��ʵ�ʲв�ֵ

      Pel reconstructedDelta;
      if ( bLossless )//��Ϊlosslessģʽ
      {
        pcCoeff[coefficientIndex] = encoderSideDelta;//�ô�ֱֵ��Ϊ����ϵ��(����Ҫ�����任������)
        reconstructedDelta        = (Pel) encoderSideDelta;//没有变换和量化过程　所以重建值和原始值相同
      }
      else
      {
        transformSkipQuantOneSample(rTu, compID, encoderSideDelta, pcCoeff, coefficientIndex, cQP, bUseHalfRoundingPoint);//��ϵ�������б任
        invTrSkipDeQuantOneSample  (rTu, compID, pcCoeff[coefficientIndex], reconstructedDelta, cQP, coefficientIndex);//由pcCoeff[coefficientIndex]进行反量化和TrSkip得到重建值
      }

      uiAbsSum += abs(pcCoeff[coefficientIndex]);//����ϵ������ֵ֮��

      if (mode != RDPCM_OFF)//��ΪRDPCM_OFF ������ۼ�ֵ �������PCMģʽ��ʵ�ʲв�ֵ
      {
        accumulatorValue += reconstructedDelta;
      }
    }
  }
}

Void TComTrQuant::rdpcmNxN   ( TComTU& rTu, const ComponentID compID, Pel* pcResidual, const UInt uiStride, const QpParam& cQP, TCoeff* pcCoeff, TCoeff &uiAbsSum, RDPCMMode& rdpcmMode )
{//��Tu�����pcm����
  TComDataCU *pcCU=rTu.getCU();//��Tu���CU��Ϣ
  const UInt uiAbsPartIdx=rTu.GetAbsPartIdxTU();//TU���λ��

  if (!pcCU->isRDPCMEnabled(uiAbsPartIdx) || ((pcCU->getTransformSkip(uiAbsPartIdx, compID) == 0) && !pcCU->getCUTransquantBypass(uiAbsPartIdx)))//���PCMģʽδʹ�ܻ��߸�Tu�鲻ʹ��TransformSkip��TransquantBypass ��pcmģʽ�ر�
  {
    rdpcmMode = RDPCM_OFF;
  }
  else if ( pcCU->isIntra( uiAbsPartIdx ) )//Ϊ֡��Ԥ��
  {
    const ChromaFormat chFmt = pcCU->getPic()->getPicYuvOrg()->getChromaFormat();//ɫ�ȸ�ʽ
    const ChannelType chType = toChannelType(compID);//ͨ������
    const UInt uiChPredMode  = pcCU->getIntraDir( chType, uiAbsPartIdx );//�õ���ͨ�����͵�֡��Ԥ��ģʽ
    const TComSPS *sps=pcCU->getSlice()->getSPS();//SPS��������Ϣ
    const UInt partsPerMinCU = 1<<(2*(sps->getMaxTotalCUDepth() - sps->getLog2DiffMaxMinCodingBlockSize()));
    const UInt uiChCodedMode = (uiChPredMode==DM_CHROMA_IDX && isChroma(compID)) ? pcCU->getIntraDir(CHANNEL_TYPE_LUMA, getChromasCorrespondingPULumaIdx(uiAbsPartIdx, chFmt, partsPerMinCU)) : uiChPredMode;
    const UInt uiChFinalMode = ((chFmt == CHROMA_422)       && isChroma(compID)) ? g_chroma422IntraAngleMappingTable[uiChCodedMode] : uiChCodedMode;//����֡��Ԥ��ģʽ���������ɫ��ģʽ�õ�

    if (uiChFinalMode == VER_IDX || uiChFinalMode == HOR_IDX)//���֡��Ԥ��ģʽΪ��׼��ֱģʽ��ˮƽģʽ  
    {
      rdpcmMode = (uiChFinalMode == VER_IDX) ? RDPCM_VER : RDPCM_HOR;
      applyForwardRDPCM( rTu, compID, pcResidual, uiStride, cQP, pcCoeff, uiAbsSum, rdpcmMode );//Ӧ�ö�Ӧ��PCMģʽ
    }
    else//���� PCM�ر�
    {
      rdpcmMode = RDPCM_OFF;
    }
  }
  else // not intra, need to select the best mode //����֡��Ԥ��ģʽ ����Ҫ�ж���õ�PCMģʽ
  {
    const UInt uiWidth  = rTu.getRect(compID).width;//Tu��Ŀ�͸�
    const UInt uiHeight = rTu.getRect(compID).height;

    RDPCMMode bestMode   = NUMBER_OF_RDPCM_MODES;//��ʼ����Ҫ�õ��Ĳ���
    TCoeff    bestAbsSum = std::numeric_limits<TCoeff>::max();
    TCoeff    bestCoefficients[MAX_TU_SIZE * MAX_TU_SIZE];

    for (UInt modeIndex = 0; modeIndex < NUMBER_OF_RDPCM_MODES; modeIndex++)//��������PCMģʽ
    {
      const RDPCMMode mode = RDPCMMode(modeIndex);

      TCoeff currAbsSum = 0;

      applyForwardRDPCM( rTu, compID, pcResidual, uiStride, cQP, pcCoeff, currAbsSum, mode );//��PCM����

      if (currAbsSum < bestAbsSum)//�任��ϵ������ֵ֮��һ���̶��Ϸ�ӳ�����Ĵ�С ������Ϊģʽѡ�������
      {
        bestMode   = mode;
        bestAbsSum = currAbsSum;
        if (mode != RDPCM_OFF)//��ʹ��PCM
        {
          memcpy(bestCoefficients, pcCoeff, (uiWidth * uiHeight * sizeof(TCoeff)));//�򱣴��PCMģʽϵ����Ϊ���ϵ��
        }
      }
    }

    rdpcmMode = bestMode;
    uiAbsSum  = bestAbsSum;

    if (rdpcmMode != RDPCM_OFF) //the TU is re-transformed and quantised if DPCM_OFF is returned, so there is no need to preserve it here
    {
      memcpy(pcCoeff, bestCoefficients, (uiWidth * uiHeight * sizeof(TCoeff)));//��ʹ��PCMģʽ ����Ҫ����ϵ�� ������PCMģʽ �����������transform ����Ҫ����ϵ��
    }
  }

  pcCU->setExplicitRdpcmModePartRange(rdpcmMode, compID, uiAbsPartIdx, rTu.GetAbsPartIdxNumParts(compID));
}

Void TComTrQuant::invRdpcmNxN( TComTU& rTu, const ComponentID compID, Pel* pcResidual, const UInt uiStride )
{
  TComDataCU *pcCU=rTu.getCU();
  const UInt uiAbsPartIdx=rTu.GetAbsPartIdxTU();
  //ȷ��PCMģʽ
  if (pcCU->isRDPCMEnabled( uiAbsPartIdx ) && ((pcCU->getTransformSkip(uiAbsPartIdx, compID ) != 0) || pcCU->getCUTransquantBypass(uiAbsPartIdx)))
  {
    const UInt uiWidth  = rTu.getRect(compID).width;
    const UInt uiHeight = rTu.getRect(compID).height;

    RDPCMMode rdpcmMode = RDPCM_OFF;

    if ( pcCU->isIntra( uiAbsPartIdx ) )
    {
      const ChromaFormat chFmt = pcCU->getPic()->getPicYuvRec()->getChromaFormat();
      const ChannelType chType = toChannelType(compID);
      const UInt uiChPredMode  = pcCU->getIntraDir( chType, uiAbsPartIdx );
      const TComSPS *sps=pcCU->getSlice()->getSPS();
      const UInt partsPerMinCU = 1<<(2*(sps->getMaxTotalCUDepth() - sps->getLog2DiffMaxMinCodingBlockSize()));
      const UInt uiChCodedMode = (uiChPredMode==DM_CHROMA_IDX && isChroma(compID)) ? pcCU->getIntraDir(CHANNEL_TYPE_LUMA, getChromasCorrespondingPULumaIdx(uiAbsPartIdx, chFmt, partsPerMinCU)) : uiChPredMode;
      const UInt uiChFinalMode = ((chFmt == CHROMA_422)       && isChroma(compID)) ? g_chroma422IntraAngleMappingTable[uiChCodedMode] : uiChCodedMode;

      if (uiChFinalMode == VER_IDX || uiChFinalMode == HOR_IDX)
      {
        rdpcmMode = (uiChFinalMode == VER_IDX) ? RDPCM_VER : RDPCM_HOR;
      }
    }
    else  // not intra case
    {
      rdpcmMode = RDPCMMode(pcCU->getExplicitRdpcmMode( compID, uiAbsPartIdx ));
    }
	
    const TCoeff pelMin=(TCoeff) std::numeric_limits<Pel>::min();
    const TCoeff pelMax=(TCoeff) std::numeric_limits<Pel>::max();
    if (rdpcmMode == RDPCM_VER)//��ֱPCMģʽ
    {
      for( UInt uiX = 0; uiX < uiWidth; uiX++ )//ȷ��������
      {
        Pel *pcCurResidual = pcResidual+uiX;//���е���ʼλ��
        TCoeff accumulator = *pcCurResidual; // 32-bit accumulator
        pcCurResidual+=uiStride;//�����ϵĵڶ���λ��
        for( UInt uiY = 1; uiY < uiHeight; uiY++, pcCurResidual+=uiStride )//���������ϵ�����ֵ
        {
          accumulator += *(pcCurResidual);//����ÿ���ϵ��ۼ�ֵ
          *pcCurResidual = (Pel)Clip3<TCoeff>(pelMin, pelMax, accumulator);//���ս��ֵ������������Сֵ֮��
        }
      }
    }
    else if (rdpcmMode == RDPCM_HOR)//ˮƽPCMģʽ
    {
      for( UInt uiY = 0; uiY < uiHeight; uiY++ )//ȷ��������
      {
        Pel *pcCurResidual = pcResidual+uiY*uiStride;//���е���ʼλ��
        TCoeff accumulator = *pcCurResidual;
        pcCurResidual++;//���еĵڶ���λ��
        for( UInt uiX = 1; uiX < uiWidth; uiX++, pcCurResidual++ )//���������ϵ�����ֵ
        {
          accumulator += *(pcCurResidual);//����ÿ���ϵ��ۼ�ֵ
          *pcCurResidual = (Pel)Clip3<TCoeff>(pelMin, pelMax, accumulator);//���ս��ֵ������������Сֵ֮��
        }
        }
      }
    }
  }
}

// ------------------------------------------------------------------------------------------------
// Logical transform
// ------------------------------------------------------------------------------------------------

/** Wrapper function between HM interface and core NxN forward transform (2D)
 *  \param channelBitDepth bit depth of channel
 *  \param useDST
 *  \param piBlkResi input data (residual)
 *  \param uiStride stride of input residual data
 *  \param psCoeff output data (transform coefficients)
 *  \param iWidth transform width
 *  \param iHeight transform height
 *  \param maxLog2TrDynamicRange
 */
Void TComTrQuant::xT( const Int channelBitDepth, Bool useDST, Pel* piBlkResi, UInt uiStride, TCoeff* psCoeff, Int iWidth, Int iHeight, const Int maxLog2TrDynamicRange )
{//HM�ӿ���xTrMxN֮��İ�װ���� ʹ��tranform����̵ĵ�����HM�и��ӷ��� ĳ�������϶�xTrMxN������
#if MATRIX_MULT
  if( iWidth == iHeight)
  {
    xTr(channelBitDepth, piBlkResi, psCoeff, uiStride, (UInt)iWidth, useDST, maxLog2TrDynamicRange);
    return;
  }
#endif

  TCoeff block[ MAX_TU_SIZE * MAX_TU_SIZE ];
  TCoeff coeff[ MAX_TU_SIZE * MAX_TU_SIZE ];

  for (Int y = 0; y < iHeight; y++)
  {
    for (Int x = 0; x < iWidth; x++)
    {
      block[(y * iWidth) + x] = piBlkResi[(y * uiStride) + x];
    }
  }

  xTrMxN( channelBitDepth, block, coeff, iWidth, iHeight, useDST, maxLog2TrDynamicRange );

  memcpy(psCoeff, coeff, (iWidth * iHeight * sizeof(TCoeff)));
}

/** Wrapper function between HM interface and core NxN inverse transform (2D)
 *  \param channelBitDepth bit depth of channel
 *  \param useDST
 *  \param plCoef input data (transform coefficients)
 *  \param pResidual output data (residual)
 *  \param uiStride stride of input residual data
 *  \param iWidth transform width
 *  \param iHeight transform height
 *  \param maxLog2TrDynamicRange
 */
Void TComTrQuant::xIT( const Int channelBitDepth, Bool useDST, TCoeff* plCoef, Pel* pResidual, UInt uiStride, Int iWidth, Int iHeight, const Int maxLog2TrDynamicRange )
{//HM�ӿ���xITrMxN֮��İ�װ���� ʹ��inverser tranform����̵ĵ�����HM�и��ӷ��� 
#if MATRIX_MULT
  if( iWidth == iHeight )
  {
    xITr(channelBitDepth, plCoef, pResidual, uiStride, (UInt)iWidth, useDST, maxLog2TrDynamicRange);
    return;
  }
#endif

  TCoeff block[ MAX_TU_SIZE * MAX_TU_SIZE ];
  TCoeff coeff[ MAX_TU_SIZE * MAX_TU_SIZE ];

  memcpy(coeff, plCoef, (iWidth * iHeight * sizeof(TCoeff)));

  xITrMxN( channelBitDepth, coeff, block, iWidth, iHeight, useDST, maxLog2TrDynamicRange );

  for (Int y = 0; y < iHeight; y++)
  {
    for (Int x = 0; x < iWidth; x++)
    {
      pResidual[(y * uiStride) + x] = Pel(block[(y * iWidth) + x]);
    }
  }
}

/** Wrapper function between HM interface and core 4x4 transform skipping
 *  \param piBlkResi input data (residual)
 *  \param uiStride stride of input residual data
 *  \param psCoeff output data (transform coefficients)
 *  \param rTu reference to transform data
 *  \param component colour component
 */
Void TComTrQuant::xTransformSkip( Pel* piBlkResi, UInt uiStride, TCoeff* psCoeff, TComTU &rTu, const ComponentID component )
{//TransformSkip �����任��һ�� ����任��
  const TComRectangle &rect = rTu.getRect(component);//Tu��λ����Ϣ
  const Int width           = rect.width;
  const Int height          = rect.height;
  const Int maxLog2TrDynamicRange = rTu.getCU()->getSlice()->getSPS()->getMaxLog2TrDynamicRange(toChannelType(component));
  const Int channelBitDepth = rTu.getCU()->getSlice()->getSPS()->getBitDepth(toChannelType(component));//ͨ��λ��

  Int iTransformShift = getTransformShift(channelBitDepth, rTu.GetEquivalentLog2TrSize(component), maxLog2TrDynamicRange);//�任��λ��
  if (rTu.getCU()->getSlice()->getSPS()->getSpsRangeExtension().getExtendedPrecisionProcessingFlag())//����չ���� ��֤��λ��Ϊ��
  {
    iTransformShift = std::max<Int>(0, iTransformShift);
  }

  const Bool rotateResidual = rTu.isNonTransformedResidualRotated(component);//�Ƿ���ת�в�
  const UInt uiSizeMinus1   = (width * height) - 1;//Tu��ϵ��������1

  if (iTransformShift >= 0)
  {
    for (UInt y = 0, coefficientIndex = 0; y < height; y++)//������������ϵ��
    {
      for (UInt x = 0; x < width; x++, coefficientIndex++)
      {
        psCoeff[rotateResidual ? (uiSizeMinus1 - coefficientIndex) : coefficientIndex] = TCoeff(piBlkResi[(y * uiStride) + x]) << iTransformShift;
      }
    }
  }
  else //for very high bit depths
  {
    iTransformShift = -iTransformShift;
    const TCoeff offset = 1 << (iTransformShift - 1);//��֤С�����������ƫ��

    for (UInt y = 0, coefficientIndex = 0; y < height; y++)
    {
      for (UInt x = 0; x < width; x++, coefficientIndex++)
      {
        psCoeff[rotateResidual ? (uiSizeMinus1 - coefficientIndex) : coefficientIndex] = (TCoeff(piBlkResi[(y * uiStride) + x]) + offset) >> iTransformShift;
      }
    }
  }
}

/** Wrapper function between HM interface and core NxN transform skipping
 *  \param plCoef input data (coefficients)
 *  \param pResidual output data (residual)
 *  \param uiStride stride of input residual data
 *  \param rTu reference to transform data
 *  \param component colour component ID
 */
Void TComTrQuant::xITransformSkip( TCoeff* plCoef, Pel* pResidual, UInt uiStride, TComTU &rTu, const ComponentID component )
{//TransformSkip �����任��һ�� ����任�����̻���ͬ�� ֻ�������任����λ�����෴
  const TComRectangle &rect = rTu.getRect(component);//Tu��λ����Ϣ
  const Int width           = rect.width;
  const Int height          = rect.height;
  const Int maxLog2TrDynamicRange = rTu.getCU()->getSlice()->getSPS()->getMaxLog2TrDynamicRange(toChannelType(component));
#if O0043_BEST_EFFORT_DECODING
  const Int channelBitDepth = rTu.getCU()->getSlice()->getSPS()->getStreamBitDepth(toChannelType(component));
#else
  const Int channelBitDepth = rTu.getCU()->getSlice()->getSPS()->getBitDepth(toChannelType(component));//ͨ��λ��
#endif

  Int iTransformShift = getTransformShift(channelBitDepth, rTu.GetEquivalentLog2TrSize(component), maxLog2TrDynamicRange);//�任��λ��
  if (rTu.getCU()->getSlice()->getSPS()->getSpsRangeExtension().getExtendedPrecisionProcessingFlag())//����չ���� ��֤��λ��Ϊ��
  {
    iTransformShift = std::max<Int>(0, iTransformShift);
  }

  const Bool rotateResidual = rTu.isNonTransformedResidualRotated(component);//�Ƿ���ת�в�
  const UInt uiSizeMinus1   = (width * height) - 1;//Tu��ϵ��������1

  if (iTransformShift >= 0)
  {
    const TCoeff offset = iTransformShift==0 ? 0 : (1 << (iTransformShift - 1));//��֤С�����������ƫ��

    for (UInt y = 0, coefficientIndex = 0; y < height; y++)//������������ϵ��
    {
      for (UInt x = 0; x < width; x++, coefficientIndex++)
      {
        pResidual[(y * uiStride) + x] =  Pel((plCoef[rotateResidual ? (uiSizeMinus1 - coefficientIndex) : coefficientIndex] + offset) >> iTransformShift);//�����б任 ֻ��λ��ԭ
      }
    }
  }
  else //for very high bit depths
  {//����ͬ��
    iTransformShift = -iTransformShift;

    for (UInt y = 0, coefficientIndex = 0; y < height; y++)
    {
      for (UInt x = 0; x < width; x++, coefficientIndex++)
      {
        pResidual[(y * uiStride) + x] = Pel(plCoef[rotateResidual ? (uiSizeMinus1 - coefficientIndex) : coefficientIndex] << iTransformShift);
      }
    }
  }
}

/** RDOQ with CABAC
 * \param rTu reference to transform data
 * \param plSrcCoeff pointer to input buffer
 * \param piDstCoeff reference to pointer to output buffer
 * \param piArlDstCoeff
 * \param uiAbsSum reference to absolute sum of quantized transform coefficient
 * \param compID colour component ID
 * \param cQP reference to quantization parameters

 * Rate distortion optimized quantization for entropy
 * coding engines using probability models like CABAC
 */
Void TComTrQuant::xRateDistOptQuant                 (       TComTU       &rTu,
                                                            TCoeff      * plSrcCoeff,
                                                            TCoeff      * piDstCoeff,//����Ľ��ֵ
#if ADAPTIVE_QP_SELECTION
                                                            TCoeff      * piArlDstCoeff,
#endif
                                                            TCoeff       &uiAbsSum,
                                                      const ComponentID   compID,
                                                      const QpParam      &cQP  )
{//��ʧ���Ż�����
  const TComRectangle  & rect             = rTu.getRect(compID);//TU��λ����Ϣ
  const UInt             uiWidth          = rect.width;
  const UInt             uiHeight         = rect.height;
        TComDataCU    *  pcCU             = rTu.getCU();
  const UInt             uiAbsPartIdx     = rTu.GetAbsPartIdxTU();
  const ChannelType      channelType      = toChannelType(compID);//ͨ������
  const UInt             uiLog2TrSize     = rTu.GetEquivalentLog2TrSize(compID);//TU�� log2��С

  const Bool             extendedPrecision = pcCU->getSlice()->getSPS()->getSpsRangeExtension().getExtendedPrecisionProcessingFlag();//�Ƿ���չ����
  const Int              maxLog2TrDynamicRange = pcCU->getSlice()->getSPS()->getMaxLog2TrDynamicRange(toChannelType(compID));
  const Int              channelBitDepth = rTu.getCU()->getSlice()->getSPS()->getBitDepth(channelType);//ͨ��λ��

  /* for 422 chroma blocks, the effective scaling applied during transformation is not a power of 2, hence it cannot be
   * implemented as a bit-shift (the quantised result will be sqrt(2) * larger than required). Alternatively, adjust the
   * uiLog2TrSize applied in iTransformShift, such that the result is 1/sqrt(2) the required result (i.e. smaller)
   * Then a QP+3 (sqrt(2)) or QP-3 (1/sqrt(2)) method could be used to get the required result
   */

  // Represents scaling through forward transform
  Int iTransformShift = getTransformShift(channelBitDepth, uiLog2TrSize, maxLog2TrDynamicRange);//�任��λ���������任���������ӣ�
  if ((pcCU->getTransformSkip(uiAbsPartIdx, compID) != 0) && extendedPrecision)//��Ҫ��չ�仯���Ⱥͷ�TransformSkipģʽʱ ȷ��iTransformShiftΪ��
  {
    iTransformShift = std::max<Int>(0, iTransformShift);
  }

  const Bool bUseGolombRiceParameterAdaptation = pcCU->getSlice()->getSPS()->getSpsRangeExtension().getPersistentRiceAdaptationEnabledFlag();
  const UInt initialGolombRiceParameter        = m_pcEstBitsSbac->golombRiceAdaptationStatistics[rTu.getGolombRiceStatisticsIndex(compID)] / RExt__GOLOMB_RICE_INCREMENT_DIVISOR;
        UInt uiGoRiceParam                     = initialGolombRiceParameter;//���ײ���Ԫ������ Ӱ���Ԫ����׺����
  Double     d64BlockUncodedCost               = 0;
  const UInt uiLog2BlockWidth                  = g_aucConvertToBit[ uiWidth  ] + 2;//Tu���log2���
  const UInt uiLog2BlockHeight                 = g_aucConvertToBit[ uiHeight ] + 2;//Tu���log2�߶�
  const UInt uiMaxNumCoeff                     = uiWidth * uiHeight;//ϵ������Ŀ
  assert(compID<MAX_NUM_COMPONENT);

  Int scalingListType = getScalingListType(pcCU->getPredictionMode(uiAbsPartIdx), compID);//��Ԥ��ģʽ��ͨ�����͵õ������������
  assert(scalingListType < SCALING_LIST_NUM);

#if ADAPTIVE_QP_SELECTION
  memset(piArlDstCoeff, 0, sizeof(TCoeff) *  uiMaxNumCoeff);
#endif
  //��������ʼ���������
  Double pdCostCoeff [ MAX_TU_SIZE * MAX_TU_SIZE ];
  Double pdCostSig   [ MAX_TU_SIZE * MAX_TU_SIZE ];
  Double pdCostCoeff0[ MAX_TU_SIZE * MAX_TU_SIZE ];
  memset( pdCostCoeff, 0, sizeof(Double) *  uiMaxNumCoeff );
  memset( pdCostSig,   0, sizeof(Double) *  uiMaxNumCoeff );
  Int rateIncUp   [ MAX_TU_SIZE * MAX_TU_SIZE ];
  Int rateIncDown [ MAX_TU_SIZE * MAX_TU_SIZE ];
  Int sigRateDelta[ MAX_TU_SIZE * MAX_TU_SIZE ];
  TCoeff deltaU   [ MAX_TU_SIZE * MAX_TU_SIZE ];
  memset( rateIncUp,    0, sizeof(Int   ) *  uiMaxNumCoeff );
  memset( rateIncDown,  0, sizeof(Int   ) *  uiMaxNumCoeff );
  memset( sigRateDelta, 0, sizeof(Int   ) *  uiMaxNumCoeff );
  memset( deltaU,       0, sizeof(TCoeff) *  uiMaxNumCoeff );

  const Int iQBits = QUANT_SHIFT + cQP.per + iTransformShift;                   // Right shift of non-RDOQ quantizer;  level = (coeff*uiQ + offset)>>q_bits
  const Double *const pdErrScale = getErrScaleCoeff(scalingListType, (uiLog2TrSize-2), cQP.rem);
  const Int    *const piQCoef    = getQuantCoeff(scalingListType, cQP.rem, (uiLog2TrSize-2));//����ϵ����ַ

  const Bool   enableScalingLists             = getUseScalingList(uiWidth, uiHeight, (pcCU->getTransformSkip(uiAbsPartIdx, compID) != 0));//�Ƿ�ʹ����������
  const Int    defaultQuantisationCoefficient = g_quantScales[cQP.rem];//��ʹ����������ʱ������ϵ��
  const Double defaultErrorScale              = getErrScaleCoeffNoScalingList(scalingListType, (uiLog2TrSize-2), cQP.rem);

  const TCoeff entropyCodingMinimum = -(1 << maxLog2TrDynamicRange);
  const TCoeff entropyCodingMaximum =  (1 << maxLog2TrDynamicRange) - 1;

#if ADAPTIVE_QP_SELECTION
  Int iQBitsC = iQBits - ARL_C_PRECISION;
  Int iAddC =  1 << (iQBitsC-1);
#endif

  TUEntropyCodingParameters codingParameters;//�ر����йز���
  getTUEntropyCodingParameters(codingParameters, rTu, compID);
  const UInt uiCGSize = (1 << MLS_CG_SIZE);//һ��CG��ϵ������

  Double pdCostCoeffGroupSig[ MLS_GRP_NUM ];// һ��CG���з���ϵ���ı������ MLS_GRP_NUMΪһ��Tu�����CG��
  UInt uiSigCoeffGroupFlag[ MLS_GRP_NUM ];//CG�����Ƿ���ڷ���ϵ��
  Int iCGLastScanPos = -1;

  UInt    uiCtxSet            = 0;
  Int     c1                  = 1;
  Int     c2                  = 0;
  Double  d64BaseCost         = 0;
  Int     iLastScanPos        = -1;

  UInt    c1Idx     = 0;
  UInt    c2Idx     = 0;
  Int     baseLevel;

  memset( pdCostCoeffGroupSig,   0, sizeof(Double) * MLS_GRP_NUM );//��ʼ��Ϊ0
  memset( uiSigCoeffGroupFlag,   0, sizeof(UInt) * MLS_GRP_NUM );

  UInt uiCGNum = uiWidth * uiHeight >> MLS_CG_SIZE;//��Tu��CG��ĸ���
  Int iScanPos;
  coeffGroupRDStats rdStats;

  const UInt significanceMapContextOffset = getSignificanceMapContextOffset(compID);//SignificanceMap���ô�ϵ���Ƿ�δ�㣩������ƫ�� ����ȷ�����յ���������

  for (Int iCGScanPos = uiCGNum-1; iCGScanPos >= 0; iCGScanPos--)//��������CG
  {
    UInt uiCGBlkPos = codingParameters.scanCG[ iCGScanPos ];
    UInt uiCGPosY   = uiCGBlkPos / codingParameters.widthInGroups;//��CG���е�Y����
    UInt uiCGPosX   = uiCGBlkPos - (uiCGPosY * codingParameters.widthInGroups);//��CG���е�X����

    memset( &rdStats, 0, sizeof (coeffGroupRDStats));

    const Int patternSigCtx = TComTrQuant::calcPatternSigCtx(uiSigCoeffGroupFlag, uiCGPosX, uiCGPosY, codingParameters.widthInGroups, codingParameters.heightInGroups);//��CG���ģʽ ����ȷ��CG����ϵ�����յ�����������

    for (Int iScanPosinCG = uiCGSize-1; iScanPosinCG >= 0; iScanPosinCG--)//�Ӻ���ǰ����CG���е�ÿ��ϵ��
    {
      iScanPos = iCGScanPos*uiCGSize + iScanPosinCG;//Tb���е�λ��
      //===== quantization =====
      UInt    uiBlkPos          = codingParameters.scan[iScanPos];
      // set coeff

      const Int    quantisationCoefficient = (enableScalingLists) ? piQCoef   [uiBlkPos] : defaultQuantisationCoefficient;//�����Ƿ�ʹ����������ѡ���Ӧ������ϵ��
      const Double errorScale              = (enableScalingLists) ? pdErrScale[uiBlkPos] : defaultErrorScale;

      const Int64  tmpLevel                = Int64(abs(plSrcCoeff[ uiBlkPos ])) * quantisationCoefficient;//��������м�ֵ

      const Intermediate_Int lLevelDouble  = (Intermediate_Int)min<Int64>(tmpLevel, std::numeric_limits<Intermediate_Int>::max() - (Intermediate_Int(1) << (iQBits - 1)));//��tmpLevel�޷�

#if ADAPTIVE_QP_SELECTION
      if( m_bUseAdaptQpSelect )
      {
        piArlDstCoeff[uiBlkPos]   = (TCoeff)(( lLevelDouble + iAddC) >> iQBitsC );
      }
#endif
      const UInt uiMaxAbsLevel  = std::min<UInt>(UInt(entropyCodingMaximum), UInt((lLevelDouble + (Intermediate_Int(1) << (iQBits - 1))) >> iQBits));//�������ֵ

      const Double dErr         = Double( lLevelDouble );
      pdCostCoeff0[ iScanPos ]  = dErr * dErr * errorScale;//ϵ������Ϊ0�����
      d64BlockUncodedCost      += pdCostCoeff0[ iScanPos ];//TbΪ����Ϊ0ʱ����ģ���ʱû�б������ֻ��ʧ����ģ�
      piDstCoeff[ uiBlkPos ]    = uiMaxAbsLevel;//������ֵ����piDstCoeff

      if ( uiMaxAbsLevel > 0 && iLastScanPos < 0 )//����ֵ����0����δ������һ������ϵ����λ��
      {
        iLastScanPos            = iScanPos;//���һ������ϵ����Tb���е�λ��
        uiCtxSet                = getContextSetIndex(compID, (iScanPos >> MLS_CG_SIZE), 0);//��������������set
        iCGLastScanPos          = iCGScanPos;//���һ������ϵ������CG��
      }

      if ( iLastScanPos >= 0 )//CG�д��ڷ�������ֵ �����һ������ϵ����ʼ������������ֵ
      {
        //===== coefficient level estimation =====
        UInt  uiLevel;
        UInt  uiOneCtx         = (NUM_ONE_FLAG_CTX_PER_SET * uiCtxSet) + c1;//coeff_abs_level_greater1������������
        UInt  uiAbsCtx         = (NUM_ABS_FLAG_CTX_PER_SET * uiCtxSet) + c2;//coeff_abs_level_greater2������������

        if( iScanPos == iLastScanPos )//���һ������ϵ��
        {
          uiLevel              = xGetCodedLevel( pdCostCoeff[ iScanPos ], pdCostCoeff0[ iScanPos ], pdCostSig[ iScanPos ],
                                                  lLevelDouble, uiMaxAbsLevel, significanceMapContextOffset, uiOneCtx, uiAbsCtx, uiGoRiceParam,
                                                  c1Idx, c2Idx, iQBits, errorScale, 1, extendedPrecision, maxLog2TrDynamicRange
                                                  );//RDO������������ֵ����Ӧ����ʧ�����
        }
        else//��Ϊ���һ������ϵ�� �����SigCtxInc�����sig������������
        {
          UShort uiCtxSig      = significanceMapContextOffset + getSigCtxInc( patternSigCtx, codingParameters, iScanPos, uiLog2BlockWidth, uiLog2BlockHeight, channelType );//sig������������

          uiLevel              = xGetCodedLevel( pdCostCoeff[ iScanPos ], pdCostCoeff0[ iScanPos ], pdCostSig[ iScanPos ],
                                                  lLevelDouble, uiMaxAbsLevel, uiCtxSig, uiOneCtx, uiAbsCtx, uiGoRiceParam,
                                                  c1Idx, c2Idx, iQBits, errorScale, 0, extendedPrecision, maxLog2TrDynamicRange
                                                  );//RDO������������ֵ����Ӧ����ʧ�����

          sigRateDelta[ uiBlkPos ] = m_pcEstBitsSbac->significantBits[ uiCtxSig ][ 1 ] - m_pcEstBitsSbac->significantBits[ uiCtxSig ][ 0 ];
        }

        deltaU[ uiBlkPos ]        = TCoeff((lLevelDouble - (Intermediate_Int(uiLevel) << iQBits)) >> (iQBits-8));//ȷ����������ֵ����������

        if( uiLevel > 0 )
        {
          Int rateNow = xGetICRate( uiLevel, uiOneCtx, uiAbsCtx, uiGoRiceParam, c1Idx, c2Idx, extendedPrecision, maxLog2TrDynamicRange );//��ǰrate
          rateIncUp   [ uiBlkPos ] = xGetICRate( uiLevel+1, uiOneCtx, uiAbsCtx, uiGoRiceParam, c1Idx, c2Idx, extendedPrecision, maxLog2TrDynamicRange ) - rateNow;//����ֵ��1����֮ǰrate�Ĳ�ֵ
          rateIncDown [ uiBlkPos ] = xGetICRate( uiLevel-1, uiOneCtx, uiAbsCtx, uiGoRiceParam, c1Idx, c2Idx, extendedPrecision, maxLog2TrDynamicRange ) - rateNow;//����ֵ��1����֮ǰrate�Ĳ�ֵ
        }
        else // uiLevel == 0 �޷���1
        {
          rateIncUp   [ uiBlkPos ] = m_pcEstBitsSbac->m_greaterOneBits[ uiOneCtx ][ 0 ];//����ֵ��1����֮ǰ��rate��(����coeff_abs_level_greater1_flag=0�ı������)
        }
        piDstCoeff[ uiBlkPos ] = uiLevel;
        d64BaseCost           += pdCostCoeff [ iScanPos ];//TB�������

        baseLevel = (c1Idx < C1FLAG_NUMBER) ? (2 + (c2Idx < C2FLAG_NUMBER)) : 1;//����ֵ��baselevel
        if( uiLevel >= baseLevel )//����coeff_abs_level_remaining
        {
          if (uiLevel > 3*(1<<uiGoRiceParam))//����uiLevel�Ĵ�Сȷ��uiGoRiceParam������R��
          {//R=Min(R+(cAbsLevel>(3*(1<<R)?1:0),4)
            uiGoRiceParam = bUseGolombRiceParameterAdaptation ? (uiGoRiceParam + 1) : (std::min<UInt>((uiGoRiceParam + 1), 4));//TR��EGK��Ԫ�� ��ò���R K=R-1
          }
        }
        if ( uiLevel >= 1)//����ϵ��
        {
          c1Idx ++;//coeff_abs_level_greater1_flag���� ֻ����ǰ8������ϵ����coeff_abs_level_greater1_flag
        }

        //===== update bin model =====
        if( uiLevel > 1 )
        {
          c1 = 0;
          c2 += (c2 < 2);
          c2Idx ++;//coeff_abs_level_greater2_flag����2
        }
        else if( (c1 < 3) && (c1 > 0) && uiLevel)
        {
          c1++;
        }

        //===== context set update =====
        if( ( iScanPos % uiCGSize == 0 ) && ( iScanPos > 0 ) )//CG�����һ��ϵ�������� ��������������
        {
          uiCtxSet          = getContextSetIndex(compID, ((iScanPos - 1) >> MLS_CG_SIZE), (c1 == 0)); //(iScanPos - 1) because we do this **before** entering the final group
          c1                = 1;
          c2                = 0;
          c1Idx             = 0;
          c2Idx             = 0;
          uiGoRiceParam     = initialGolombRiceParameter;
        }
      }//end  if ( iLastScanPos >= 0 )
      else//�ô�ϵ��һ��Ϊ��
      {
        d64BaseCost    += pdCostCoeff0[ iScanPos ];//����ļ���ϵ��Ϊ������
      }
      rdStats.d64SigCost += pdCostSig[ iScanPos ];//sig�����
      if (iScanPosinCG == 0 )
      {
        rdStats.d64SigCost_0 = pdCostSig[ iScanPos ];//tb��DC��sig���
      }
      if (piDstCoeff[ uiBlkPos ] )//�������ֵ��Ϊ0
      {
        uiSigCoeffGroupFlag[ uiCGBlkPos ] = 1;//��CG���д��ڷ���ϵ�� uiSigCoeffGroupFlag��1
        rdStats.d64CodedLevelandDist += pdCostCoeff[ iScanPos ] - pdCostSig[ iScanPos ];//��CG��ȥsig����ʧ����ۣ�����ֻ�������+ʧ����ģ�
        rdStats.d64UncodedDist += pdCostCoeff0[ iScanPos ];//CG�����Ϊ��ʱ��ʧ�����
        if ( iScanPosinCG != 0 )
        {
          rdStats.iNNZbeforePos0++;//��iScanPosinCG 0λ��֮ǰ����ֵ��Ϊ��ĸ���
        }
      }
    } //end for (iScanPosinCG)

    if (iCGLastScanPos >= 0)//Tb���д����з���ϵ����CG
    {
      if( iCGScanPos )//��ΪTb�е�һ��CG
      {
        if (uiSigCoeffGroupFlag[ uiCGBlkPos ] == 0)//��CG�в����ڷ���ϵ��
        {
          UInt  uiCtxSig = getSigCoeffGroupCtxInc( uiSigCoeffGroupFlag, uiCGPosX, uiCGPosY, codingParameters.widthInGroups, codingParameters.heightInGroups );//��ǰCG��coeff_abs_significant_flag��CtxInc
          d64BaseCost += xGetRateSigCoeffGroup(0, uiCtxSig) - rdStats.d64SigCost;//����ʧ����ϸ�CG��uiSigCoeffGroupFlag=0����ģ���ΪCGϵ��ȫΪ�����Բ�����sig��
          pdCostCoeffGroupSig[ iCGScanPos ] = xGetRateSigCoeffGroup(0, uiCtxSig);//��CG��CoeffGroupSig�ı������
        }
        else//��CG�д��ڷ���ϵ�� �жϵ�ǰCG�Ƿ�����Ϊȫ����
        {
          if (iCGScanPos < iCGLastScanPos) //skip the last coefficient group, which will be handled together with last position below.��������һ��CG��������
          {
            if ( rdStats.iNNZbeforePos0 == 0 )//ֻ��CG���ϵ�һ��λ��ϵ����Ϊ��
            {
              d64BaseCost -= rdStats.d64SigCost_0;//��ȥ��Ӧ���
              rdStats.d64SigCost -= rdStats.d64SigCost_0;
            }
            // rd-cost if SigCoeffGroupFlag = 0, initialization
            Double d64CostZeroCG = d64BaseCost;//��ʼ��ȫ����CG����ʧ��Ϊd64BaseCost

            // add SigCoeffGroupFlag cost to total cost
            UInt  uiCtxSig = getSigCoeffGroupCtxInc( uiSigCoeffGroupFlag, uiCGPosX, uiCGPosY, codingParameters.widthInGroups, codingParameters.heightInGroups );//uiSigCoeffGroupFlag��ctxInc

            if (iCGScanPos < iCGLastScanPos)
            {
              d64BaseCost  += xGetRateSigCoeffGroup(1, uiCtxSig);//d64BaseCost��CG�з�������ֵ uiSignificanceCoeffGroupΪ1
              d64CostZeroCG += xGetRateSigCoeffGroup(0, uiCtxSig);//d64CostZeroCG��CG������ֵΪȫ��  uiSignificanceCoeffGroupΪ0
              pdCostCoeffGroupSig[ iCGScanPos ] = xGetRateSigCoeffGroup(1, uiCtxSig);
            }

            // try to convert the current coeff group from non-zero to all-zero ����ȫ�������ʧ�棨��ԭʼ����CG��ʧ�����Ӽ�����
            d64CostZeroCG += rdStats.d64UncodedDist;  // distortion for resetting non-zero levels to zero levels
            d64CostZeroCG -= rdStats.d64CodedLevelandDist;   // distortion and level cost for keeping all non-zero levels
            d64CostZeroCG -= rdStats.d64SigCost;     // sig cost for all coeffs, including zero levels and non-zerl levels

            // if we can save cost, change this block to all-zero block
            if ( d64CostZeroCG < d64BaseCost )//����ʧ���С �򽫸�CG��Ϊȫ����
            {
              uiSigCoeffGroupFlag[ uiCGBlkPos ] = 0;
              d64BaseCost = d64CostZeroCG;//������ʧ��
              if (iCGScanPos < iCGLastScanPos)
              {
                pdCostCoeffGroupSig[ iCGScanPos ] = xGetRateSigCoeffGroup(0, uiCtxSig);
              }
              // reset coeffs to 0 in this block
              for (Int iScanPosinCG = uiCGSize-1; iScanPosinCG >= 0; iScanPosinCG--)//������������ֵ
              {
                iScanPos      = iCGScanPos*uiCGSize + iScanPosinCG;
                UInt uiBlkPos = codingParameters.scan[ iScanPos ];

                if (piDstCoeff[ uiBlkPos ])//��ԭ������ֵ����
                {
                  piDstCoeff [ uiBlkPos ] = 0;//��������ֵΪ0
                  pdCostCoeff[ iScanPos ] = pdCostCoeff0[ iScanPos ];
                  pdCostSig  [ iScanPos ] = 0;
                }
              }
            } // end if ( d64CostAllZeros < d64BaseCost )
          }
        } // end if if (uiSigCoeffGroupFlag[ uiCGBlkPos ] == 0)
      }
      else
      {
        uiSigCoeffGroupFlag[ uiCGBlkPos ] = 1;
      }
    }
  } //end for (iCGScanPos)

  //===== estimate last position ===== ȷ�����һ������ϵ��λ��
  if ( iLastScanPos < 0 )//�����ڷ���ϵ��
  {
    return;//��������
  }

  Double  d64BestCost         = 0;
  Int     ui16CtxCbf          = 0;
  Int     iBestLastIdxP1      = 0;
  if( !pcCU->isIntra( uiAbsPartIdx ) && isLuma(compID) && pcCU->getTransformIdx( uiAbsPartIdx ) == 0 )
  {
    ui16CtxCbf   = 0;
    d64BestCost  = d64BlockUncodedCost + xGetICost( m_pcEstBitsSbac->blockRootCbpBits[ ui16CtxCbf ][ 0 ] );
    d64BaseCost += xGetICost( m_pcEstBitsSbac->blockRootCbpBits[ ui16CtxCbf ][ 1 ] );
  }
  else
  {
    ui16CtxCbf   = pcCU->getCtxQtCbf( rTu, channelType );
    ui16CtxCbf  += getCBFContextOffset(compID);
    d64BestCost  = d64BlockUncodedCost + xGetICost( m_pcEstBitsSbac->blockCbpBits[ ui16CtxCbf ][ 0 ] );
    d64BaseCost += xGetICost( m_pcEstBitsSbac->blockCbpBits[ ui16CtxCbf ][ 1 ] );
  }


  Bool bFoundLast = false;
  for (Int iCGScanPos = iCGLastScanPos; iCGScanPos >= 0; iCGScanPos--)//��������CG
  {
    UInt uiCGBlkPos = codingParameters.scanCG[ iCGScanPos ];

    d64BaseCost -= pdCostCoeffGroupSig [ iCGScanPos ];
    if (uiSigCoeffGroupFlag[ uiCGBlkPos ])
    {
      for (Int iScanPosinCG = uiCGSize-1; iScanPosinCG >= 0; iScanPosinCG--)//����CG����������ֵ
      {
        iScanPos = iCGScanPos*uiCGSize + iScanPosinCG;

        if (iScanPos > iLastScanPos)//�����һ������ϵ������ǰȷ�����ŵķ���ϵ��λ��
        {
          continue;
        }
        UInt   uiBlkPos     = codingParameters.scan[iScanPos];

        if( piDstCoeff[ uiBlkPos ] )//����ֵ����0
        {
          UInt   uiPosY       = uiBlkPos >> uiLog2BlockWidth;
          UInt   uiPosX       = uiBlkPos - ( uiPosY << uiLog2BlockWidth );

          Double d64CostLast= codingParameters.scanType == SCAN_VER ? xGetRateLast( uiPosY, uiPosX, compID ) : xGetRateLast( uiPosX, uiPosY, compID );
          Double totalCost = d64BaseCost + d64CostLast - pdCostSig[ iScanPos ];

          if( totalCost < d64BestCost )//�µ�������ϵ��λ�õ�cost��֮ǰС 
          {
            iBestLastIdxP1  = iScanPos + 1;//��������������ϵ��λ��
            d64BestCost     = totalCost;//����bestcost
          }
          if( piDstCoeff[ uiBlkPos ] > 1 )//���һ������ϵ��λ���� ���һ��Ԥ����ֵ����0 �� ���һ��Ԥ����ֵ����1 ֮�� 
          {//����������ֵ����1 ���ټ���Ѱ�����ŷ���ϵ����λ��
            bFoundLast = true;
            break;
          }
          d64BaseCost      -= pdCostCoeff[ iScanPos ];//����ô�����ֵΪ����Tb������ʧ��
          d64BaseCost      += pdCostCoeff0[ iScanPos ];//�ô�����ֵ��Ϊ����Tb������ʧ��Ϊ��ԭ��ʧ���ȥ�ô�ԭ����ֵcost��������ֵΪ0��cost
        }
        else//����ֵ����0
        {
          d64BaseCost      -= pdCostSig[ iScanPos ];//�ô�������Ҫ����sig
        }
      } //end for
      if (bFoundLast)//�ҵ����ŷ���ϵ��λ�� ����ѭ��
      {
        break;
      }
    } // end if (uiSigCoeffGroupFlag[ uiCGBlkPos ])
  } // end for


  for ( Int scanPos = 0; scanPos < iBestLastIdxP1; scanPos++ )//������������ϵ��λ����ǰ������ֵ�ľ���ֵ���϶�Ӧ����
  {
    Int blkPos = codingParameters.scan[ scanPos ];
    TCoeff level = piDstCoeff[ blkPos ];
    uiAbsSum += level;//����ֵ����ֵ֮��
    piDstCoeff[ blkPos ] = ( plSrcCoeff[ blkPos ] < 0 ) ? -level : level;
  }

  //===== clean uncoded coefficients =====
  for ( Int scanPos = iBestLastIdxP1; scanPos <= iLastScanPos; scanPos++ )//CG�н�����������ϵ��֮���λ�õ�����ֵ��0
  {
    piDstCoeff[ codingParameters.scan[ scanPos ] ] = 0;
  }


  if( pcCU->getSlice()->getPPS()->getSignHideFlag() && uiAbsSum>=2)//SDH���� ���ٱ���������ݵı����� ֮ǰ�������� ����׸��
  {
    const Double inverseQuantScale = Double(g_invQuantScales[cQP.rem]);
    Int64 rdFactor = (Int64)(inverseQuantScale * inverseQuantScale * (1 << (2 * cQP.per))
                             / m_dLambda / 16 / (1 << (2 * DISTORTION_PRECISION_ADJUSTMENT(channelBitDepth - 8)))
                             + 0.5);

    Int lastCG = -1;
    Int absSum = 0 ;
    Int n ;

    for( Int subSet = (uiWidth*uiHeight-1) >> MLS_CG_SIZE; subSet >= 0; subSet-- )
    {
      Int  subPos     = subSet << MLS_CG_SIZE;
      Int  firstNZPosInCG=uiCGSize , lastNZPosInCG=-1 ;
      absSum = 0 ;

      for(n = uiCGSize-1; n >= 0; --n )
      {
        if( piDstCoeff[ codingParameters.scan[ n + subPos ]] )
        {
          lastNZPosInCG = n;
          break;
        }
      }

      for(n = 0; n <uiCGSize; n++ )
      {
        if( piDstCoeff[ codingParameters.scan[ n + subPos ]] )
        {
          firstNZPosInCG = n;
          break;
        }
      }

      for(n = firstNZPosInCG; n <=lastNZPosInCG; n++ )
      {
        absSum += Int(piDstCoeff[ codingParameters.scan[ n + subPos ]]);
      }

      if(lastNZPosInCG>=0 && lastCG==-1)
      {
        lastCG = 1;
      }

      if( lastNZPosInCG-firstNZPosInCG>=SBH_THRESHOLD )
      {
        UInt signbit = (piDstCoeff[codingParameters.scan[subPos+firstNZPosInCG]]>0?0:1);
        if( signbit!=(absSum&0x1) )  // hide but need tune
        {
          // calculate the cost
          Int64 minCostInc = std::numeric_limits<Int64>::max(), curCost = std::numeric_limits<Int64>::max();
          Int minPos = -1, finalChange = 0, curChange = 0;

          for( n = (lastCG==1?lastNZPosInCG:uiCGSize-1) ; n >= 0; --n )
          {
            UInt uiBlkPos   = codingParameters.scan[ n + subPos ];
            if(piDstCoeff[ uiBlkPos ] != 0 )
            {
              Int64 costUp   = rdFactor * ( - deltaU[uiBlkPos] ) + rateIncUp[uiBlkPos];
              Int64 costDown = rdFactor * (   deltaU[uiBlkPos] ) + rateIncDown[uiBlkPos]
                               -   ((abs(piDstCoeff[uiBlkPos]) == 1) ? sigRateDelta[uiBlkPos] : 0);

              if(lastCG==1 && lastNZPosInCG==n && abs(piDstCoeff[uiBlkPos])==1)
              {
                costDown -= (4<<15);
              }

              if(costUp<costDown)
              {
                curCost = costUp;
                curChange =  1;
              }
              else
              {
                curChange = -1;
                if(n==firstNZPosInCG && abs(piDstCoeff[uiBlkPos])==1)
                {
                  curCost = std::numeric_limits<Int64>::max();
                }
                else
                {
                  curCost = costDown;
                }
              }
            }
            else
            {
              curCost = rdFactor * ( - (abs(deltaU[uiBlkPos])) ) + (1<<15) + rateIncUp[uiBlkPos] + sigRateDelta[uiBlkPos] ;
              curChange = 1 ;

              if(n<firstNZPosInCG)
              {
                UInt thissignbit = (plSrcCoeff[uiBlkPos]>=0?0:1);
                if(thissignbit != signbit )
                {
                  curCost = std::numeric_limits<Int64>::max();
                }
              }
            }

            if( curCost<minCostInc)
            {
              minCostInc = curCost;
              finalChange = curChange;
              minPos = uiBlkPos;
            }
          }

          if(piDstCoeff[minPos] == entropyCodingMaximum || piDstCoeff[minPos] == entropyCodingMinimum)
          {
            finalChange = -1;
          }

          if(plSrcCoeff[minPos]>=0)
          {
            piDstCoeff[minPos] += finalChange ;
          }
          else
          {
            piDstCoeff[minPos] -= finalChange ;
          }
        }
      }

      if(lastCG==1)
      {
        lastCG=0 ;
      }
    }
  }
}


/** Pattern decision for context derivation process of significant_coeff_flag
 * \param sigCoeffGroupFlag pointer to prior coded significant coeff group
 * \param uiCGPosX column of current coefficient group
 * \param uiCGPosY row of current coefficient group
 * \param widthInGroups width of the block
 * \param heightInGroups height of the block
 * \returns pattern for current coefficient group
 */
Int  TComTrQuant::calcPatternSigCtx( const UInt* sigCoeffGroupFlag, UInt uiCGPosX, UInt uiCGPosY, UInt widthInGroups, UInt heightInGroups )
{//�����ڽ���CG��CSBF�õ���ǰCG��ģʽ ���ڵõ�CG��ÿ��ϵ��������������
  if ((widthInGroups <= 1) && (heightInGroups <= 1))//CG��͸߲�����1 ���������²���Ҳ�CG ֱ�ӷ���0
  {
    return 0;
  }

  const Bool rightAvailable = uiCGPosX < (widthInGroups  - 1);//�����Ҳ�CG
  const Bool belowAvailable = uiCGPosY < (heightInGroups - 1);//�����²�CG

  UInt sigRight = 0;
  UInt sigLower = 0;

  if (rightAvailable)
  {
    sigRight = ((sigCoeffGroupFlag[ (uiCGPosY * widthInGroups) + uiCGPosX + 1 ] != 0) ? 1 : 0);//�Ҳ�CG��CSBF
  }
  if (belowAvailable)
  {
    sigLower = ((sigCoeffGroupFlag[ (uiCGPosY + 1) * widthInGroups + uiCGPosX ] != 0) ? 1 : 0);//�²�CG��CSBF
  }

  return sigRight + (sigLower << 1);//����ģʽ
}


/** Context derivation process of coeff_abs_significant_flag
 * \param patternSigCtx pattern for current coefficient group
 * \param codingParameters coding parameters for the TU (includes the scan)
 * \param scanPosition current position in scan order
 * \param log2BlockWidth log2 width of the block
 * \param log2BlockHeight log2 height of the block
 * \param chanType channel type (CHANNEL_TYPE_LUMA/CHROMA)
 * \returns ctxInc for current scan position
 */
Int TComTrQuant::getSigCtxInc    (       Int                        patternSigCtx,
                                   const TUEntropyCodingParameters &codingParameters,
                                   const Int                        scanPosition,
                                   const Int                        log2BlockWidth,
                                   const Int                        log2BlockHeight,
                                   const ChannelType                chanType)
{//��CG��ģʽ�õ���CG�ж�Ӧλ��ϵ����CtxInc(���������������� ����ȷ��ÿ��ϵ��������������)
  if (codingParameters.firstSignificanceMapContext == significanceMapContextSetStart[chanType][CONTEXT_TYPE_SINGLE])
  {//��Ϊ��������ģʽ ��ֱ�ӷ���significanceMapContextSetStart��ʼ������Ϊ��������
    //single context mode
    return significanceMapContextSetStart[chanType][CONTEXT_TYPE_SINGLE];
  }

  const UInt rasterPosition = codingParameters.scan[scanPosition];
  const UInt posY           = rasterPosition >> log2BlockWidth;//��ϵ����TB�е�Y����
  const UInt posX           = rasterPosition - (posY << log2BlockWidth);//��ϵ����TB�е�X����

  if ((posX + posY) == 0)//ֱ��������TB�����Ͻ�ϵ����ctxIdxֱ��Ϊ0
  {
    return 0; //special case for the DC context variable
  }

  Int offset = MAX_INT;

  if ((log2BlockWidth == 2) && (log2BlockHeight == 2)) //4x4 TB��Ϊ4*4
  {
    offset = ctxIndMap4x4[ (4 * posY) + posX ];//ֱ���ɱ�õ�offset
  }
  else//��TB��Ϊ4*4
  {
    Int cnt = 0;

    switch (patternSigCtx)//�ж�CGģʽ
    {
      //------------------

      case 0: //neither neighbouring group is significant
        {
          const Int posXinSubset     = posX & ((1 << MLS_CG_LOG2_WIDTH)  - 1);//�ڵ�ǰCG�е�X����
          const Int posYinSubset     = posY & ((1 << MLS_CG_LOG2_HEIGHT) - 1);//�ڵ�ǰCG�е�Y����
          const Int posTotalInSubset = posXinSubset + posYinSubset;

          //first N coefficients in scan order use 2; the next few use 1; the rest use 0.
          const UInt context1Threshold = NEIGHBOURHOOD_00_CONTEXT_1_THRESHOLD_4x4;
          const UInt context2Threshold = NEIGHBOURHOOD_00_CONTEXT_2_THRESHOLD_4x4;

          cnt = (posTotalInSubset >= context1Threshold) ? 0 : ((posTotalInSubset >= context2Threshold) ? 1 : 2);//��CG���϶Խ�һ��Ϊ2 ������Ϊ1 ����Ϊ0
        }
        break;

      //------------------

      case 1: //right group is significant, below is not
        {
          const Int posYinSubset = posY & ((1 << MLS_CG_LOG2_HEIGHT) - 1);
          const Int groupHeight  = 1 << MLS_CG_LOG2_HEIGHT;

          cnt = (posYinSubset >= (groupHeight >> 1)) ? 0 : ((posYinSubset >= (groupHeight >> 2)) ? 1 : 2); //top quarter uses 2; second-from-top quarter uses 1; bottom half uses 0
        }//ˮƽ��һ��Ϊ2 �ڶ���Ϊ1 ����Ϊ0
        break;

      //------------------

      case 2: //below group is significant, right is not
        {
          const Int posXinSubset = posX & ((1 << MLS_CG_LOG2_WIDTH)  - 1);
          const Int groupWidth   = 1 << MLS_CG_LOG2_WIDTH;

          cnt = (posXinSubset >= (groupWidth >> 1)) ? 0 : ((posXinSubset >= (groupWidth >> 2)) ? 1 : 2); //left quarter uses 2; second-from-left quarter uses 1; right half uses 0
        }//��ֱ��һ��Ϊ2 �ڶ���Ϊ1 ������Ϊ0
        break;

      //------------------

      case 3: //both neighbouring groups are significant
        {
          cnt = 2;//ȫ��Ϊ2
        }
        break;

      //------------------

      default:
        std::cerr << "ERROR: Invalid patternSigCtx \"" << Int(patternSigCtx) << "\" in getSigCtxInc" << std::endl;
        exit(1);
        break;
    }

    //------------------------------------------------

    const Bool notFirstGroup = ((posX >> MLS_CG_LOG2_WIDTH) + (posY >> MLS_CG_LOG2_HEIGHT)) > 0;//��ΪTB�е�һ��CG

    offset = (notFirstGroup ? notFirstGroupNeighbourhoodContextOffset[chanType] : 0) + cnt;//��Ϊ��һ��CG����϶�Ӧƫ��
  }

  return codingParameters.firstSignificanceMapContext + offset;//����CtxInc  ��ȥluma��chroma offest���CtxIdx
}


/** Get the best level in RD sense
 *
 * \returns best quantized transform level for given scan position
 *
 * This method calculates the best quantized transform level for a given scan position.
 */
__inline UInt TComTrQuant::xGetCodedLevel ( Double&          rd64CodedCost,          //< reference to coded cost
                                            Double&          rd64CodedCost0,         //< reference to cost when coefficient is 0
                                            Double&          rd64CodedCostSig,       //< rd64CodedCostSig reference to cost of significant coefficient
                                            Intermediate_Int lLevelDouble,           //< reference to unscaled quantized level
                                            UInt             uiMaxAbsLevel,          //< scaled quantized level
                                            UShort           ui16CtxNumSig,          //< current ctxInc for coeff_abs_significant_flag
                                            UShort           ui16CtxNumOne,          //< current ctxInc for coeff_abs_level_greater1 (1st bin of coeff_abs_level_minus1 in AVC)
                                            UShort           ui16CtxNumAbs,          //< current ctxInc for coeff_abs_level_greater2 (remaining bins of coeff_abs_level_minus1 in AVC)
                                            UShort           ui16AbsGoRice,          //< current Rice parameter for coeff_abs_level_minus3
                                            UInt             c1Idx,                  //< 
                                            UInt             c2Idx,                  //< 
                                            Int              iQBits,                 //< quantization step size
                                            Double           errorScale,             //< 
                                            Bool             bLast,                  //< indicates if the coefficient is the last significant
                                            Bool             useLimitedPrefixLength, //< 
                                            const Int        maxLog2TrDynamicRange   //< 
                                            ) const
{//����RDO׼��ȷ����ǰϵ������������ֵ
  Double dCurrCostSig   = 0;
  UInt   uiBestAbsLevel = 0;

  if( !bLast && uiMaxAbsLevel < 3 )//������ֵΪ0��1��2ʱ��������ֵ����Ϊ0 ��ʱ��0��ΪĬ������ֵ
  {
    rd64CodedCostSig    = xGetRateSigCoef( 0, ui16CtxNumSig );//��ǰ����ֵΪ0
    rd64CodedCost       = rd64CodedCost0 + rd64CodedCostSig;//����ֵΪ0ʱ����ʧ�����
    if( uiMaxAbsLevel == 0 )//������ֵΪ0ʱ ��������ֵ��Ϊ0
    {
      return uiBestAbsLevel;
    }
  }
  else
  {
    rd64CodedCost       = MAX_DOUBLE;
  }

  if( !bLast )
  {
    dCurrCostSig        = xGetRateSigCoef( 1, ui16CtxNumSig );//��ǰ����ֵ��0
  }

  UInt uiMinAbsLevel    = ( uiMaxAbsLevel > 1 ? uiMaxAbsLevel - 1 : 1 );//������ֵ���ڵ���3ʱ ���ܵ���������ֵΪN��N-1
  for( Int uiAbsLevel  = uiMaxAbsLevel; uiAbsLevel >= uiMinAbsLevel ; uiAbsLevel-- )//�������ܵ���������ֵ
  {
    Double dErr         = Double( lLevelDouble  - ( Intermediate_Int(uiAbsLevel) << iQBits ) );//�������
    Double dCurrCost    = dErr * dErr * errorScale + xGetICost( xGetICRate( uiAbsLevel, ui16CtxNumOne, ui16CtxNumAbs, ui16AbsGoRice, c1Idx, c2Idx, useLimitedPrefixLength, maxLog2TrDynamicRange ) );
    dCurrCost          += dCurrCostSig;//�ܵ���ʧ�����

    if( dCurrCost < rd64CodedCost )//ѡ����ʧ����ʧ��С������ֵΪ��������ֵ
    {
      uiBestAbsLevel    = uiAbsLevel;
      rd64CodedCost     = dCurrCost;
      rd64CodedCostSig  = dCurrCostSig;
    }
  }

  return uiBestAbsLevel;
}

/** Calculates the cost for specific absolute transform level
 * \param uiAbsLevel scaled quantized level
 * \param ui16CtxNumOne current ctxInc for coeff_abs_level_greater1 (1st bin of coeff_abs_level_minus1 in AVC)
 * \param ui16CtxNumAbs current ctxInc for coeff_abs_level_greater2 (remaining bins of coeff_abs_level_minus1 in AVC)
 * \param ui16AbsGoRice Rice parameter for coeff_abs_level_minus3
 * \param c1Idx
 * \param c2Idx
 * \param useLimitedPrefixLength
 * \param maxLog2TrDynamicRange
 * \returns cost of given absolute transform level
 */
__inline Int TComTrQuant::xGetICRate         ( const UInt    uiAbsLevel,
                                               const UShort  ui16CtxNumOne,
                                               const UShort  ui16CtxNumAbs,
                                               const UShort  ui16AbsGoRice,
                                               const UInt    c1Idx,
                                               const UInt    c2Idx,
                                               const Bool    useLimitedPrefixLength,
                                               const Int     maxLog2TrDynamicRange
                                               ) const
{//�����������ֵ��Rֵ���������ֵ��
  Int  iRate      = Int(xGetIEPRate()); // cost of sign bit
  UInt baseLevel  = (c1Idx < C1FLAG_NUMBER) ? (2 + (c2Idx < C2FLAG_NUMBER)) : 1;//largerThan1 flag�����ĿΪ8  largerThan2 flag�����ĿΪ1

  if ( uiAbsLevel >= baseLevel )//����ֵ���ڵ���base Level ������coeff_abs_level_remaining
  {
    UInt symbol     = uiAbsLevel - baseLevel;
    UInt length;
    if (symbol < (COEF_REMAIN_BIN_REDUCTION << ui16AbsGoRice))//ALRem С�ڣ�3<<k����׺���ȹ̶�Ϊk
    {
      length = symbol>>ui16AbsGoRice;//ǰ׺����
      iRate += (length+1+ui16AbsGoRice)<< 15;//������ʧ
    }
    else if (useLimitedPrefixLength)//����ǰ׺����
    {
      const UInt maximumPrefixLength = (32 - (COEF_REMAIN_BIN_REDUCTION + maxLog2TrDynamicRange));//ALRem���bin����Ϊ32

      UInt prefixLength = 0;
      UInt suffix       = (symbol >> ui16AbsGoRice) - COEF_REMAIN_BIN_REDUCTION;

      while ((prefixLength < maximumPrefixLength) && (suffix > ((2 << prefixLength) - 2)))
      {
        prefixLength++;
      }

      const UInt suffixLength = (prefixLength == maximumPrefixLength) ? (maxLog2TrDynamicRange - ui16AbsGoRice) : (prefixLength + 1/*separator*/);

      iRate += (COEF_REMAIN_BIN_REDUCTION + prefixLength + suffixLength + ui16AbsGoRice) << 15;
    }
    else//������ǰ׺���� ALRem ��С�ڣ�3<<k��
    {//����ALRem��Ԫ������ ��ֵΪ log2((N-(3<<k)>>k)+1)����ȡ��+k 
      length = ui16AbsGoRice;
      symbol  = symbol - ( COEF_REMAIN_BIN_REDUCTION << ui16AbsGoRice);
      while (symbol >= (1<<length))
      {
        symbol -=  (1<<(length++));
      }
      iRate += (COEF_REMAIN_BIN_REDUCTION+length+1-ui16AbsGoRice+length)<< 15;
    }//���㲿��ʹ����λ��ѭ��������� ����Ϊ����ֵ

    if (c1Idx < C1FLAG_NUMBER)//��c1��Ч
    {
      iRate += m_pcEstBitsSbac->m_greaterOneBits[ ui16CtxNumOne ][ 1 ];//����Rate�����largerThan1 flag�ı���Rate

      if (c2Idx < C2FLAG_NUMBER)//��c2��Ч 
      {
        iRate += m_pcEstBitsSbac->m_levelAbsBits[ ui16CtxNumAbs ][ 1 ];//����Rate�����largerThan2 flag�ı���Rate
      }
    }
  }
  else if( uiAbsLevel == 1 )//����ֵΪ1 ��������largerThan1 flag2��ALRem
  {
    iRate += m_pcEstBitsSbac->m_greaterOneBits[ ui16CtxNumOne ][ 0 ];//�����ձ���RateΪlargerThan1 flag=0�ı���Rate
  }
  else if( uiAbsLevel == 2 )//����ֵΪ2 ��������ALRem
  {
    iRate += m_pcEstBitsSbac->m_greaterOneBits[ ui16CtxNumOne ][ 1 ];//�����ձ���RateΪlargerThan1 flag=1��largerThan1 flag=0�ı���Rate
    iRate += m_pcEstBitsSbac->m_levelAbsBits[ ui16CtxNumAbs ][ 0 ];
  }
  else
  {
    iRate = 0;
  }

  return  iRate;
}

__inline Double TComTrQuant::xGetRateSigCoeffGroup  ( UShort                    uiSignificanceCoeffGroup,
                                                UShort                          ui16CtxNumSig ) const
{
  return xGetICost( m_pcEstBitsSbac->significantCoeffGroupBits[ ui16CtxNumSig ][ uiSignificanceCoeffGroup ] );//significantCoeffGroupBits������� ����CG�Ƿ���ڷ���ϵ��2
}

/** Calculates the cost of signaling the last significant coefficient in the block
 * \param uiPosX X coordinate of the last significant coefficient
 * \param uiPosY Y coordinate of the last significant coefficient
 * \param component colour component ID
 * \returns cost of last significant coefficient
 */
/*
 * \param uiWidth width of the transform unit (TU)
*/
__inline Double TComTrQuant::xGetRateLast   ( const UInt                      uiPosX,
                                              const UInt                      uiPosY,
                                              const ComponentID               component  ) const
{//����������ϵ��λ�õı������
  UInt uiCtxX   = g_uiGroupIdx[uiPosX];//X���ڵ���������
  UInt uiCtxY   = g_uiGroupIdx[uiPosY];//Y���ڵ���������

  Double uiCost = m_pcEstBitsSbac->lastXBits[toChannelType(component)][ uiCtxX ] + m_pcEstBitsSbac->lastYBits[toChannelType(component)][ uiCtxY ];//ǰ׺�ı������

  if( uiCtxX > 3 )//��������������3ʱ ���ں�׺ ��׺������·����
  {
    uiCost += xGetIEPRate() * ((uiCtxX-2)>>1);//��׺�ı������ (uiCtxX-2)>>1��Ϊ��׺bin��λ��
  }
  if( uiCtxY > 3 )//Y���� ͬ X
  {
    uiCost += xGetIEPRate() * ((uiCtxY-2)>>1);
  }
  return xGetICost( uiCost );
}

__inline Double TComTrQuant::xGetRateSigCoef  ( UShort                          uiSignificance,
                                                UShort                          ui16CtxNumSig ) const
{
  return xGetICost( m_pcEstBitsSbac->significantBits[ ui16CtxNumSig ][ uiSignificance ] );//significantBits������� ��������ϵ���Ƿ�Ϊ��
}

/** Get the cost for a specific rate
 * \param dRate rate of a bit
 * \returns cost at the specific rate
 */
__inline Double TComTrQuant::xGetICost        ( Double                          dRate         ) const
{
  return m_dLambda * dRate;//����������
}

/** Get the cost of an equal probable bit
 * \returns cost of equal probable bit
 */
__inline Double TComTrQuant::xGetIEPRate      (                                               ) const
{//һ�ȸ���λ������� ��·����
  return 32768;//1<<15
}

/** Context derivation process of coeff_abs_significant_flag
 * \param uiSigCoeffGroupFlag significance map of L1
 * \param uiCGPosX column of current scan position
 * \param uiCGPosY row of current scan position
 * \param widthInGroups width of the block
 * \param heightInGroups height of the block
 * \returns ctxInc for current scan position
 */
UInt TComTrQuant::getSigCoeffGroupCtxInc  (const UInt*  uiSigCoeffGroupFlag,
                                           const UInt   uiCGPosX,
                                           const UInt   uiCGPosY,
                                           const UInt   widthInGroups,
                                           const UInt   heightInGroups)
{//��ǰCG��coeff_abs_significant_flag��CtxInc�����·����Ҳ�CG��coeff_abs_significant_flag�й�
  UInt sigRight = 0;
  UInt sigLower = 0;

  if (uiCGPosX < (widthInGroups  - 1))//�����Ҳ�CG
  {
    sigRight = ((uiSigCoeffGroupFlag[ (uiCGPosY * widthInGroups) + uiCGPosX + 1 ] != 0) ? 1 : 0);//�Ҳ�CG��coeff_abs_significant_flag
  }
  if (uiCGPosY < (heightInGroups - 1))//�����·�CG
  {
    sigLower = ((uiSigCoeffGroupFlag[ (uiCGPosY + 1) * widthInGroups + uiCGPosX ] != 0) ? 1 : 0);//�·�CG��coeff_abs_significant_flag
  }

  return ((sigRight + sigLower) != 0) ? 1 : 0;
}


/** set quantized matrix coefficient for encode
 * \param scalingList            quantized matrix address
 * \param format                 chroma format
 * \param maxLog2TrDynamicRange
 * \param bitDepths              reference to bit depth array for all channels
 */
Void TComTrQuant::setScalingList(TComScalingList *scalingList, const Int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE], const BitDepths &bitDepths)
{//����ʱΪ����ϵ����ֵ
  const Int minimumQp = 0;
  const Int maximumQp = SCALING_LIST_REM_NUM;

  for(UInt size = 0; size < SCALING_LIST_SIZE_NUM; size++)//4*4 8*8 16*16 32*32
  {
    for(UInt list = 0; list < SCALING_LIST_NUM; list++)//MAX_NUM_COMPONENT * NUMBER_OF_PREDICTION_MODES
    {
      for(Int qp = minimumQp; qp < maximumQp; qp++)//qp����6 Qstep����һ��
      {
        xSetScalingListEnc(scalingList,list,size,qp);
        xSetScalingListDec(*scalingList,list,size,qp);
        setErrScaleCoeff(list,size,qp,maxLog2TrDynamicRange, bitDepths);
      }
    }
  }
}
/** set quantized matrix coefficient for decode
 * \param scalingList quantized matrix address
 * \param format      chroma format
 */
Void TComTrQuant::setScalingListDec(const TComScalingList &scalingList)
{//����ͬ�� ��ÿ�������ֵ
  const Int minimumQp = 0;
  const Int maximumQp = SCALING_LIST_REM_NUM;

  for(UInt size = 0; size < SCALING_LIST_SIZE_NUM; size++)
  {
    for(UInt list = 0; list < SCALING_LIST_NUM; list++)
    {
      for(Int qp = minimumQp; qp < maximumQp; qp++)
      {
        xSetScalingListDec(scalingList,list,size,qp);
      }
    }
  }
}
/** set error scale coefficients
 * \param list                   list ID
 * \param size                   
 * \param qp                     quantization parameter
 * \param maxLog2TrDynamicRange
 * \param bitDepths              reference to bit depth array for all channels
 */
Void TComTrQuant::setErrScaleCoeff(UInt list, UInt size, Int qp, const Int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE], const BitDepths &bitDepths)
{
  const UInt uiLog2TrSize = g_aucConvertToBit[ g_scalingListSizeX[size] ] + 2;//TB���Сȡ����
  const ChannelType channelType = ((list == 0) || (list == MAX_NUM_COMPONENT)) ? CHANNEL_TYPE_LUMA : CHANNEL_TYPE_CHROMA;//ͨ������

  const Int channelBitDepth    = bitDepths.recon[channelType];//ͨ��λ��
  const Int iTransformShift = getTransformShift(channelBitDepth, uiLog2TrSize, maxLog2TrDynamicRange[channelType]);  // Represents scaling through forward transform

  UInt i,uiMaxNumCoeff = g_scalingListSize[size];//��������ϵ������
  Int *piQuantcoeff;
  Double *pdErrScale;
  piQuantcoeff   = getQuantCoeff(list, qp,size);//����ϵ��
  pdErrScale     = getErrScaleCoeff(list, size, qp);

  Double dErrScale = (Double)(1<<SCALE_BITS);                                // Compensate for scaling of bitcount in Lagrange cost function
  dErrScale = dErrScale*pow(2.0,(-2.0*iTransformShift));                     // Compensate for scaling through forward transform

  for(i=0;i<uiMaxNumCoeff;i++)//������������ϵ��
  {
    pdErrScale[i] =  dErrScale / piQuantcoeff[i] / piQuantcoeff[i] / (1 << DISTORTION_PRECISION_ADJUSTMENT(2 * (bitDepths.recon[channelType] - 8)));//����ÿ��ϵ����ErrScale
  }

  getErrScaleCoeffNoScalingList(list, size, qp) = dErrScale / g_quantScales[qp] / g_quantScales[qp] / (1 << DISTORTION_PRECISION_ADJUSTMENT(2 * (bitDepths.recon[channelType] - 8)));//δʹ�����������ErrScale
}

/** set quantized matrix coefficient for encode
 * \param scalingList quantized matrix address
 * \param listId List index
 * \param sizeId size index
 * \param qp Quantization parameter
 * \param format chroma format
 */
Void TComTrQuant::xSetScalingListEnc(TComScalingList *scalingList, UInt listId, UInt sizeId, Int qp)
{//����ʱ ��������ϵ��
  UInt width  = g_scalingListSizeX[sizeId];//TB��Ŀ�
  UInt height = g_scalingListSizeX[sizeId];//TB��ĸ�
  UInt ratio  = g_scalingListSizeX[sizeId]/min(MAX_MATRIX_SIZE_NUM,(Int)g_scalingListSizeX[sizeId]);//�ϲ����� 16*16/32*32 ��ͨ��8*8�ϲ����õ�
  Int *quantcoeff;
  Int *coeff  = scalingList->getScalingListAddress(sizeId,listId);//��������ĵ�ַ
  quantcoeff  = getQuantCoeff(listId, qp, sizeId);//��������ϵ��

  Int quantScales = g_quantScales[qp];

  processScalingListEnc(coeff,
                        quantcoeff,
                        (quantScales << LOG2_SCALING_LIST_NEUTRAL_VALUE),
                        height, width, ratio,
                        min(MAX_MATRIX_SIZE_NUM, (Int)g_scalingListSizeX[sizeId]),
                        scalingList->getScalingListDC(sizeId,listId));//��������ϵ��
}

/** set quantized matrix coefficient for decode
 * \param scalingList quantaized matrix address
 * \param listId List index
 * \param sizeId size index
 * \param qp Quantization parameter
 * \param format chroma format
 */
Void TComTrQuant::xSetScalingListDec(const TComScalingList &scalingList, UInt listId, UInt sizeId, Int qp)
{//�������ʱ������ϵ��  ����ͬ���� 
  UInt width  = g_scalingListSizeX[sizeId];
  UInt height = g_scalingListSizeX[sizeId];
  UInt ratio  = g_scalingListSizeX[sizeId]/min(MAX_MATRIX_SIZE_NUM,(Int)g_scalingListSizeX[sizeId]);
  Int *dequantcoeff;
  const Int *coeff  = scalingList.getScalingListAddress(sizeId,listId);

  dequantcoeff = getDequantCoeff(listId, qp, sizeId);

  Int invQuantScale = g_invQuantScales[qp];

  processScalingListDec(coeff,
                        dequantcoeff,
                        invQuantScale,
                        height, width, ratio,
                        min(MAX_MATRIX_SIZE_NUM, (Int)g_scalingListSizeX[sizeId]),
                        scalingList.getScalingListDC(sizeId,listId));
}

/** set flat matrix value to quantized coefficient
 */
Void TComTrQuant::setFlatScalingList(const Int maxLog2TrDynamicRange[MAX_NUM_CHANNEL_TYPE], const BitDepths &bitDepths)
{//��ʹ�����������ʹ��ͬһqpֵʱTB�������ϵ��
  const Int minimumQp = 0;
  const Int maximumQp = SCALING_LIST_REM_NUM;

  for(UInt size = 0; size < SCALING_LIST_SIZE_NUM; size++)
  {
    for(UInt list = 0; list < SCALING_LIST_NUM; list++)
    {
      for(Int qp = minimumQp; qp < maximumQp; qp++)
      {
        xsetFlatScalingList(list,size,qp);
        setErrScaleCoeff(list,size,qp,maxLog2TrDynamicRange, bitDepths);
      }
    }
  }
}

/** set flat matrix value to quantized coefficient
 * \param list List ID
 * \param size size index
 * \param qp Quantization parameter
 * \param format chroma format
 */
Void TComTrQuant::xsetFlatScalingList(UInt list, UInt size, Int qp)
{//��ʹ�����������ʹ��ͬһqpֵʱTB�������ϵ��
  UInt i,num = g_scalingListSize[size];
  Int *quantcoeff;
  Int *dequantcoeff;

  Int quantScales    = g_quantScales   [qp];
  Int invQuantScales = g_invQuantScales[qp] << 4;

  quantcoeff   = getQuantCoeff(list, qp, size);
  dequantcoeff = getDequantCoeff(list, qp, size);

  for(i=0;i<num;i++)
  {
    *quantcoeff++ = quantScales;
    *dequantcoeff++ = invQuantScales;
  }
}

/** set quantized matrix coefficient for encode
 * \param coeff quantaized matrix address
 * \param quantcoeff quantaized matrix address
 * \param quantScales Q(QP%6)
 * \param height height
 * \param width width
 * \param ratio ratio for upscale
 * \param sizuNum matrix size
 * \param dc dc parameter
 */
Void TComTrQuant::processScalingListEnc( Int *coeff, Int *quantcoeff, Int quantScales, UInt height, UInt width, UInt ratio, Int sizuNum, UInt dc)
{
  for(UInt j=0;j<height;j++)//������
  {
    for(UInt i=0;i<width;i++)//������
    {
      quantcoeff[j*width + i] = quantScales / coeff[sizuNum * (j / ratio) + i / ratio];//�������������TB����ÿ������ϵ����������������+qp������
    }
  }

  if(ratio > 1)//�����ϲ������DC����ϵ�������¼���
  {
    quantcoeff[0] = quantScales / dc;
  }
}

/** set quantized matrix coefficient for decode
 * \param coeff quantaized matrix address
 * \param dequantcoeff quantaized matrix address
 * \param invQuantScales IQ(QP%6))
 * \param height height
 * \param width width
 * \param ratio ratio for upscale
 * \param sizuNum matrix size
 * \param dc dc parameter
 */
Void TComTrQuant::processScalingListDec( const Int *coeff, Int *dequantcoeff, Int invQuantScales, UInt height, UInt width, UInt ratio, Int sizuNum, UInt dc)
{//������������������������ϵ�� ����ͬ����
  for(UInt j=0;j<height;j++)
  {
    for(UInt i=0;i<width;i++)
    {
      dequantcoeff[j*width + i] = invQuantScales * coeff[sizuNum * (j / ratio) + i / ratio];
    }
  }

  if(ratio > 1)
  {
    dequantcoeff[0] = invQuantScales * dc;
  }
}

/** initialization process of scaling list array
 */
//��ʼ�������������
Void TComTrQuant::initScalingList()
{
  for(UInt sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(UInt qp = 0; qp < SCALING_LIST_REM_NUM; qp++)
    {
      for(UInt listId = 0; listId < SCALING_LIST_NUM; listId++)
      {
        m_quantCoef   [sizeId][listId][qp] = new Int    [g_scalingListSize[sizeId]];
        m_dequantCoef [sizeId][listId][qp] = new Int    [g_scalingListSize[sizeId]];
        m_errScale    [sizeId][listId][qp] = new Double [g_scalingListSize[sizeId]];
      } // listID loop
    }
  }
}

/** destroy quantization matrix array
 */
//ɾ������ �ͷ���Դ
Void TComTrQuant::destroyScalingList()   
{
  for(UInt sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(UInt listId = 0; listId < SCALING_LIST_NUM; listId++)
    {
      for(UInt qp = 0; qp < SCALING_LIST_REM_NUM; qp++)
      {
        if(m_quantCoef[sizeId][listId][qp])
        {
          delete [] m_quantCoef[sizeId][listId][qp];
        }
        if(m_dequantCoef[sizeId][listId][qp])
        {
          delete [] m_dequantCoef[sizeId][listId][qp];
        }
        if(m_errScale[sizeId][listId][qp])
        {
          delete [] m_errScale[sizeId][listId][qp];
        }
      }
    }
  }
}

Void TComTrQuant::transformSkipQuantOneSample(TComTU &rTu, const ComponentID compID, const TCoeff resiDiff, TCoeff* pcCoeff, const UInt uiPos, const QpParam &cQP, const Bool bUseHalfRoundingPoint)
{//对单个点进行transformSkip和Quant
        TComDataCU    *pcCU                           = rTu.getCU();//Tu所在CU信息
  const UInt           uiAbsPartIdx                   = rTu.GetAbsPartIdxTU();//TU块的位置
  const TComRectangle &rect                           = rTu.getRect(compID);
  const UInt           uiWidth                        = rect.width;//Tu块的宽和高
  const UInt           uiHeight                       = rect.height;
  const Int            maxLog2TrDynamicRange          = pcCU->getSlice()->getSPS()->getMaxLog2TrDynamicRange(toChannelType(compID));
  const Int            channelBitDepth                = pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID));//通道位深
  const Int            iTransformShift                = getTransformShift(channelBitDepth, rTu.GetEquivalentLog2TrSize(compID), maxLog2TrDynamicRange);
  const Int            scalingListType                = getScalingListType(pcCU->getPredictionMode(uiAbsPartIdx), compID);//量化矩阵类型
  const Bool           enableScalingLists             = getUseScalingList(uiWidth, uiHeight, true);//是否使用量化矩阵
  const Int            defaultQuantisationCoefficient = g_quantScales[cQP.rem];//默认量化系数（不使用量化矩阵时的量化系数）

  assert( scalingListType < SCALING_LIST_NUM );
  const Int *const piQuantCoeff = getQuantCoeff( scalingListType, cQP.rem, (rTu.GetEquivalentLog2TrSize(compID)-2) );//量化系数的起始位置


  /* for 422 chroma blocks, the effective scaling applied during transformation is not a power of 2, hence it cannot be
  * implemented as a bit-shift (the quantised result will be sqrt(2) * larger than required). Alternatively, adjust the
  * uiLog2TrSize applied in iTransformShift, such that the result is 1/sqrt(2) the required result (i.e. smaller)
  * Then a QP+3 (sqrt(2)) or QP-3 (1/sqrt(2)) method could be used to get the required result
  */

  const Int iQBits = QUANT_SHIFT + cQP.per + iTransformShift;//乘以量化系数后的缩小量　14+floor(QP/6)+i_shift
  // QBits will be OK for any internal bit depth as the reduction in transform shift is balanced by an increase in Qp_per due to QpBDOffset

  const Int iAdd = ( bUseHalfRoundingPoint ? 256 : (pcCU->getSlice()->getSliceType() == I_SLICE ? 171 : 85) ) << (iQBits - 9);//舍入偏移量　使用HalfRoundingPoint时为0.5 否则I帧为1/3 P或B图像为1/6

  TCoeff transformedCoefficient;

  // transform-skip//不进行变换　只进行相应变换位移
  if (iTransformShift >= 0)
  {
    transformedCoefficient = resiDiff << iTransformShift;
  }
  else // for very high bit depths
  {
    const Int iTrShiftNeg  = -iTransformShift;
    const Int offset       = 1 << (iTrShiftNeg - 1);
    transformedCoefficient = ( resiDiff + offset ) >> iTrShiftNeg;
  }

  // quantization量化部分
  const TCoeff iSign = (transformedCoefficient < 0 ? -1: 1);//变换后系数正负　

  const Int quantisationCoefficient = enableScalingLists ? piQuantCoeff[uiPos] : defaultQuantisationCoefficient;//根据是否使用量化矩阵选择对应量化系数

  const Int64 tmpLevel = (Int64)abs(transformedCoefficient) * quantisationCoefficient;//量化中间值（dij*MF）

  const TCoeff quantisedCoefficient = (TCoeff((tmpLevel + iAdd ) >> iQBits)) * iSign;//最终量化值

  const TCoeff entropyCodingMinimum = -(1 << maxLog2TrDynamicRange);
  const TCoeff entropyCodingMaximum =  (1 << maxLog2TrDynamicRange) - 1;
  pcCoeff[ uiPos ] = Clip3<TCoeff>( entropyCodingMinimum, entropyCodingMaximum, quantisedCoefficient );//将限定大小后的量化结果赋值给pcCoeff
}


Void TComTrQuant::invTrSkipDeQuantOneSample( TComTU &rTu, ComponentID compID, TCoeff inSample, Pel &reconSample, const QpParam &cQP, UInt uiPos )
{//对单个点进行invTrSkip和DeQuant
        TComDataCU    *pcCU               = rTu.getCU();//Tu所在CU信息
  const UInt           uiAbsPartIdx       = rTu.GetAbsPartIdxTU();//TU块的位置
  const TComRectangle &rect               = rTu.getRect(compID);
  const UInt           uiWidth            = rect.width;//Tu块的宽和高
  const UInt           uiHeight           = rect.height;
  const Int            QP_per             = cQP.per;//Qp/6
  const Int            QP_rem             = cQP.rem;//Qp%6
  const Int            maxLog2TrDynamicRange = pcCU->getSlice()->getSPS()->getMaxLog2TrDynamicRange(toChannelType(compID));
#if O0043_BEST_EFFORT_DECODING
  const Int            channelBitDepth    = pcCU->getSlice()->getSPS()->getStreamBitDepth(toChannelType(compID));
#else
  const Int            channelBitDepth    = pcCU->getSlice()->getSPS()->getBitDepth(toChannelType(compID));
#endif
  const Int            iTransformShift    = getTransformShift(channelBitDepth, rTu.GetEquivalentLog2TrSize(compID), maxLog2TrDynamicRange);
  const Int            scalingListType    = getScalingListType(pcCU->getPredictionMode(uiAbsPartIdx), compID);//量化矩阵类型
  const Bool           enableScalingLists = getUseScalingList(uiWidth, uiHeight, true);//是否使用量化矩阵
  const UInt           uiLog2TrSize       = rTu.GetEquivalentLog2TrSize(compID);//Tu log2大小

  assert( scalingListType < SCALING_LIST_NUM );

  const Int rightShift = (IQUANT_SHIFT - (iTransformShift + QP_per)) + (enableScalingLists ? LOG2_SCALING_LIST_NEUTRAL_VALUE : 0);

  const TCoeff transformMinimum = -(1 << maxLog2TrDynamicRange);
  const TCoeff transformMaximum =  (1 << maxLog2TrDynamicRange) - 1;

  // Dequantisation//反量化部分

  TCoeff dequantisedSample;

  if(enableScalingLists)//如果使用量化矩阵
  {
    const UInt             dequantCoefBits     = 1 + IQUANT_SHIFT + SCALING_LIST_BITS;//反量化系数移位量2
    const UInt             targetInputBitDepth = std::min<UInt>((maxLog2TrDynamicRange + 1), (((sizeof(Intermediate_Int) * 8) + rightShift) - dequantCoefBits));

    const Intermediate_Int inputMinimum        = -(1 << (targetInputBitDepth - 1));
    const Intermediate_Int inputMaximum        =  (1 << (targetInputBitDepth - 1)) - 1;

    Int *piDequantCoef = getDequantCoeff(scalingListType,QP_rem,uiLog2TrSize-2);//得到反量化系数

    if(rightShift > 0)//中间反量化系数向右移位
    {
      const Intermediate_Int iAdd      = 1 << (rightShift - 1);//小数四舍五入取整时的偏执
      const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, inSample));//限制需要反量化值的大小
      const Intermediate_Int iCoeffQ   = ((Intermediate_Int(clipQCoef) * piDequantCoef[uiPos]) + iAdd ) >> rightShift;//得到反量化结果

      dequantisedSample = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));//限制反量化结果大小
    }
    else//中间反量化系数向左移位
    {
      const Int              leftShift = -rightShift;
      const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, inSample));
      const Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * piDequantCoef[uiPos]) << leftShift;

      dequantisedSample = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
    }
  }
  else//不使用量化矩阵 (除反量化系数改变外　其他同上)
  {
    const Int scale     =  g_invQuantScales[QP_rem];
    const Int scaleBits =     (IQUANT_SHIFT + 1)   ;

    const UInt             targetInputBitDepth = std::min<UInt>((maxLog2TrDynamicRange + 1), (((sizeof(Intermediate_Int) * 8) + rightShift) - scaleBits));
    const Intermediate_Int inputMinimum        = -(1 << (targetInputBitDepth - 1));
    const Intermediate_Int inputMaximum        =  (1 << (targetInputBitDepth - 1)) - 1;

    if (rightShift > 0)
    {
      const Intermediate_Int iAdd      = 1 << (rightShift - 1);
      const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, inSample));
      const Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * scale + iAdd) >> rightShift;

      dequantisedSample = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
    }
    else
    {
      const Int              leftShift = -rightShift;
      const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, inSample));
      const Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * scale) << leftShift;

      dequantisedSample = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
    }
  }

  // Inverse transform-skip//不进行反变化　只进行位移　缩放还原成变换前数量级

  if (iTransformShift >= 0)
  {
    const TCoeff offset = iTransformShift==0 ? 0 : (1 << (iTransformShift - 1));
    reconSample =  Pel(( dequantisedSample + offset ) >> iTransformShift);
  }
  else //for very high bit depths
  {
    const Int iTrShiftNeg = -iTransformShift;
    reconSample = Pel(dequantisedSample << iTrShiftNeg);
  }
}

//ccp技术　消除色度与亮度间的相关性　提高编码效率
Void TComTrQuant::crossComponentPrediction(       TComTU      & rTu,
                                            const ComponentID   compID,
                                            const Pel         * piResiL,
                                            const Pel         * piResiC,
                                                  Pel         * piResiT,
                                            const Int           width,
                                            const Int           height,
                                            const Int           strideL,
                                            const Int           strideC,
                                            const Int           strideT,
                                            const Bool          reverse )
{//色度残差的预测　包括正向的预测和反向的重构：预测计算为y'=y-(alpha*x) 重构计算为y'=y+(alpha*x) 其中alpha＝sum(x*y)/sum(x*x)
  const Pel *pResiL = piResiL;//亮度残差
  const Pel *pResiC = piResiC;//原始色度残差
        Pel *pResiT = piResiT;//预测(重构)后的色度残差值

  TComDataCU *pCU = rTu.getCU();
  const Int alpha = pCU->getCrossComponentPredictionAlpha( rTu.GetAbsPartIdxTU( compID ), compID );
  const Int diffBitDepth = pCU->getSlice()->getSPS()->getDifferentialLumaChromaBitDepth();//为色度和亮度的位深差异值

  for( Int y = 0; y < height; y++ )//遍历所有残差值
  {
    if (reverse)//反向重构
    {
      // A constraint is to be added to the HEVC Standard to limit the size of pResiL and pResiC at this point.
      // The likely form of the constraint is to either restrict the values to CoeffMin to CoeffMax,
      // or to be representable in a bitDepthY+4 or bitDepthC+4 signed integer.
      //  The result of the constraint is that for 8/10/12bit profiles, the input values
      //  can be represented within a 16-bit Pel-type.
#if RExt__HIGH_BIT_DEPTH_SUPPORT
      for( Int x = 0; x < width; x++ )
      {
        pResiT[x] = pResiC[x] + (( alpha * rightShift( pResiL[x], diffBitDepth) ) >> 3);
      }
#else
      const Int minPel=std::numeric_limits<Pel>::min();//允许残差的最大最下值
      const Int maxPel=std::numeric_limits<Pel>::max();
      for( Int x = 0; x < width; x++ )
      {//rightShift<Int>(Int(pResiL[x]), diffBitDepth)处理亮度和色度组成间的bit深度差异使色度亮度位深相同　>>3位是因为乘法计算中原本alpha属于［-1.0,1.0］被映射到［-8,8］所以需右移三位
        pResiT[x] = Clip3<Int>(minPel, maxPel, pResiC[x] + (( alpha * rightShift<Int>(Int(pResiL[x]), diffBitDepth) ) >> 3));//y'=y+(alpha*x)
      }
#endif
    }
    else//正向预测
    {
      // Forward does not need clipping. Pel type should always be big enough.
      for( Int x = 0; x < width; x++ )
      {
        pResiT[x] = pResiC[x] - (( alpha * rightShift<Int>(Int(pResiL[x]), diffBitDepth) ) >> 3);//y'=y-(alpha*x)
      }
    }

    pResiL += strideL;
    pResiC += strideC;
    pResiT += strideT;
  }
}

//! \}
