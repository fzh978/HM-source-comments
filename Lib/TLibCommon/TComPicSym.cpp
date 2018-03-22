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

/** \file     TComPicSym.cpp
    \brief    picture symbol class
*/

#include "TComPicSym.h"
#include "TComSampleAdaptiveOffset.h"
#include "TComSlice.h"

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TComPicSym::TComPicSym()
:m_frameWidthInCtus(0)//该帧图像的宽（以CTU为单位）
,m_frameHeightInCtus(0)//该帧图像的宽（以CTU为单位）
,m_uiMinCUWidth(0)//最下Cu的宽（默认设置下为4）
,m_uiMinCUHeight(0)//最下Cu的高（默认设置下为4）
,m_uhTotalDepth(0)//CTu划分的总深度（默认设置下为4）
,m_numPartitionsInCtu(0)//一个CTU中4*4小块的个数
,m_numPartInCtuWidth(0)//CTU的宽（以4*4小块为单位）
,m_numPartInCtuHeight(0)//CTU的高（以4*4小块为单位）
,m_numCtusInFrame(0)//一帧图像中CTU的个数
,m_apSlices()//保存图像中slices的队列
,m_pictureCtuArray(NULL)//保存图像中每个CTU数据的数组
,m_numTileColumnsMinus1(0)//一帧图像中Tile的列数减1
,m_numTileRowsMinus1(0)//一帧图像中Tile的行数减1
,m_ctuTsToRsAddrMap(NULL)//Tile-Scan地址到Raster-Scan地址的映射
,m_puiTileIdxMap(NULL)//Tile索引到Raster-Scan 的CTu地址
,m_ctuRsToTsAddrMap(NULL)//Raster-Scan地址到Tile-Scan地址的映射
,m_saoBlkParams(NULL)//图像sao相关参数 用于SAO
#if ADAPTIVE_QP_SELECTION
,m_pParentARLBuffer(NULL)
#endif
{}


Void TComPicSym::create  ( const TComSPS &sps, const TComPPS &pps, UInt uiMaxDepth )//用sps和pps参数信息初始化当前图像信息
{
  UInt i;
  m_sps = sps;
  m_pps = pps;

  const ChromaFormat chromaFormatIDC = sps.getChromaFormatIdc();//色度格式
  const Int iPicWidth      = sps.getPicWidthInLumaSamples();//图像的宽
  const Int iPicHeight     = sps.getPicHeightInLumaSamples();//图像的高
  const UInt uiMaxCuWidth  = sps.getMaxCUWidth();//CU的最大宽度
  const UInt uiMaxCuHeight = sps.getMaxCUHeight();//CU的最大高度

  m_uhTotalDepth       = uiMaxDepth;//4
  m_numPartitionsInCtu = 1<<(m_uhTotalDepth<<1);//256

  m_uiMinCUWidth       = uiMaxCuWidth  >> m_uhTotalDepth;//4
  m_uiMinCUHeight      = uiMaxCuHeight >> m_uhTotalDepth;//4

  m_numPartInCtuWidth  = uiMaxCuWidth  / m_uiMinCUWidth;  // equivalent to 1<<m_uhTotalDepth   16
  m_numPartInCtuHeight = uiMaxCuHeight / m_uiMinCUHeight; // equivalent to 1<<m_uhTotalDepth　　16

  m_frameWidthInCtus   = ( iPicWidth %uiMaxCuWidth  ) ? iPicWidth /uiMaxCuWidth  + 1 : iPicWidth /uiMaxCuWidth;
  m_frameHeightInCtus  = ( iPicHeight%uiMaxCuHeight ) ? iPicHeight/uiMaxCuHeight + 1 : iPicHeight/uiMaxCuHeight;

  m_numCtusInFrame     = m_frameWidthInCtus * m_frameHeightInCtus;
  m_pictureCtuArray    = new TComDataCU*[m_numCtusInFrame];

  clearSliceBuffer();
  allocateNewSlice();

#if ADAPTIVE_QP_SELECTION
  if (m_pParentARLBuffer == NULL)
  {
     m_pParentARLBuffer = new TCoeff[uiMaxCuWidth*uiMaxCuHeight*MAX_NUM_COMPONENT];
  }
#endif

  for ( i=0; i<m_numCtusInFrame ; i++ )
  {
    m_pictureCtuArray[i] = new TComDataCU;
    m_pictureCtuArray[i]->create( chromaFormatIDC, m_numPartitionsInCtu, uiMaxCuWidth, uiMaxCuHeight, false, uiMaxCuWidth >> m_uhTotalDepth
#if ADAPTIVE_QP_SELECTION
      , m_pParentARLBuffer
#endif
      );
  }

  m_ctuTsToRsAddrMap = new UInt[m_numCtusInFrame+1];
  m_puiTileIdxMap    = new UInt[m_numCtusInFrame];
  m_ctuRsToTsAddrMap = new UInt[m_numCtusInFrame+1];

  for( i=0; i<m_numCtusInFrame; i++ )
  {
    m_ctuTsToRsAddrMap[i] = i;
    m_ctuRsToTsAddrMap[i] = i;
  }

  m_saoBlkParams = new SAOBlkParam[m_numCtusInFrame];


  xInitTiles();
  xInitCtuTsRsAddrMaps();

}

Void TComPicSym::destroy()//释放资源
{
  clearSliceBuffer();

  for (Int i = 0; i < m_numCtusInFrame; i++)
  {
    m_pictureCtuArray[i]->destroy();
    delete m_pictureCtuArray[i];
    m_pictureCtuArray[i] = NULL;
  }
  delete [] m_pictureCtuArray;
  m_pictureCtuArray = NULL;

  delete [] m_ctuTsToRsAddrMap;
  m_ctuTsToRsAddrMap = NULL;

  delete [] m_puiTileIdxMap;
  m_puiTileIdxMap = NULL;

  delete [] m_ctuRsToTsAddrMap;
  m_ctuRsToTsAddrMap = NULL;

  if(m_saoBlkParams)
  {
    delete[] m_saoBlkParams; m_saoBlkParams = NULL;
  }

#if ADAPTIVE_QP_SELECTION
  delete [] m_pParentARLBuffer;
  m_pParentARLBuffer = NULL;
#endif
}

Void TComPicSym::allocateNewSlice()//向m_apSlices尾部添加新的slice
{
  m_apSlices.push_back(new TComSlice);//向m_apSlices尾部添加新的slice
  m_apSlices.back()->setPPS(&m_pps);//设置新slice的pps参数为当前图像的pps参数
  m_apSlices.back()->setSPS(&m_sps);//设置新slice的sps参数为当前图像的sps参数
  if (m_apSlices.size()>=2)//如果m_apSlices中slice的个数大于或等于2
  {
    m_apSlices.back()->copySliceInfo( m_apSlices[m_apSlices.size()-2] );//将倒数第二个slice的信息复制给新slice
    m_apSlices.back()->initSlice();//将新slice复制来的部分参数重新初始化
  }
}

Void TComPicSym::clearSliceBuffer()//清空m_apSlices
{
  for (UInt i = 0; i < UInt(m_apSlices.size()); i++)
  {
    delete m_apSlices[i];
  }
  m_apSlices.clear();
}

Void TComPicSym::xInitCtuTsRsAddrMaps()//初始化Tile-Scan地址 Raster-Scan地址的映射表 m_ctuTsToRsAddrMap和m_ctuRsToTsAddrMap
{
  //generate the Coding Order Map and Inverse Coding Order Map
  for(Int ctuTsAddr=0, ctuRsAddr=0; ctuTsAddr<getNumberOfCtusInFrame(); ctuTsAddr++, ctuRsAddr = xCalculateNextCtuRSAddr(ctuRsAddr))
  {//xCalculateNextCtuRSAddr(ctuRsAddr)计算按TsAddr下个CTU的RsAddr
    setCtuTsToRsAddrMap(ctuTsAddr, ctuRsAddr);//将m_ctuTsToRsAddrMap ctuTsAddr位置的值设置成ctuRsAddr
    setCtuRsToTsAddrMap(ctuRsAddr, ctuTsAddr);//将m_ctuRsToTsAddrMap ctuRsAddr位置的值设置成ctuTsAddr
  }
  setCtuTsToRsAddrMap(getNumberOfCtusInFrame(), getNumberOfCtusInFrame());//最后一个CTU的RsAddr和TsAddr相同
  setCtuRsToTsAddrMap(getNumberOfCtusInFrame(), getNumberOfCtusInFrame());
}

Void TComPicSym::xInitTiles()//初始化化tiles 主要时确定图像中tiles的大小
{
  //set NumColumnsMinus1 and NumRowsMinus1
  setNumTileColumnsMinus1( m_pps.getNumTileColumnsMinus1() );//从m_pps参数信息中得到Tile的列数减1
  setNumTileRowsMinus1(    m_pps.getNumTileRowsMinus1()    );//从m_pps参数信息中得到Tile的行数减1

  const Int numCols = m_pps.getNumTileColumnsMinus1() + 1;//图像中Tile的列数
  const Int numRows = m_pps.getNumTileRowsMinus1() + 1;//图像中Tile的行数
  const Int numTiles = numRows * numCols;//总Tile数

  // allocate memory for tile parameters
  m_tileParameters.resize(numTiles);

  if( m_pps.getTileUniformSpacingFlag() )//tile在图像边界处分布和图像内分布需要保持一致
  {
    //set width and height for each (uniform) tile
    for(Int row=0; row < numRows; row++)//为每个tile指定宽和高(以CTU为单位)
    {
      for(Int col=0; col < numCols; col++)
      {
        const Int tileIdx = row * numCols + col;//tile索引 指明图像中tile的位置
        m_tileParameters[tileIdx].setTileWidthInCtus(  (col+1)*getFrameWidthInCtus( )/numCols - (col*getFrameWidthInCtus( ))/numCols );//相邻tile的大小可能相差正负1
        m_tileParameters[tileIdx].setTileHeightInCtus( (row+1)*getFrameHeightInCtus()/numRows - (row*getFrameHeightInCtus())/numRows );
      }//该处不直接使用getFrameWidthInCtus( )/numCols计算是因为每个tile所含有的CTu必须为整数 且保证tile在图像边界处分布和图像内分布一致
    }//该处不好直接理解的话 可以举个具体的例子看看 如该图像一行有24个CTu 要分成10个tile 则对应每个tile的大小为 2 2 3 2 3 2 2 3 2 3 且每行tile CTU的总数必为FrameWidthInCtus
  }//高也同理
  else//tile在图像边界处分布和图像内分布不需要保持一致
  {
    //set the width for each tile
    for(Int row=0; row < numRows; row++)//为每行的tiles设置宽
    {
      Int cumulativeTileWidth = 0;
      for(Int col=0; col < getNumTileColumnsMinus1(); col++)//除每行的最后一个tile外的每个tile的宽为m_pps中给定的宽
      {
        m_tileParameters[row * numCols + col].setTileWidthInCtus( m_pps.getTileColumnWidth(col) );
        cumulativeTileWidth += m_pps.getTileColumnWidth(col);
      }
      m_tileParameters[row * numCols + getNumTileColumnsMinus1()].setTileWidthInCtus( getFrameWidthInCtus()-cumulativeTileWidth );//最后一个tile的宽为一帧图像中该行未被之前tile占用的Ctu的个数
    }                                                                                                                             //即最后一个tile需保证将图像一行（列）覆盖完整 所以会出现图像边界处tile与图像内tile大小不一致

    //set the height for each tile
    for(Int col=0; col < numCols; col++)//为tile设置高 同上
    {
      Int cumulativeTileHeight = 0;
      for(Int row=0; row < getNumTileRowsMinus1(); row++)
      {
        m_tileParameters[row * numCols + col].setTileHeightInCtus( m_pps.getTileRowHeight(row) );
        cumulativeTileHeight += m_pps.getTileRowHeight(row);
      }
      m_tileParameters[getNumTileRowsMinus1() * numCols + col].setTileHeightInCtus( getFrameHeightInCtus()-cumulativeTileHeight );
    }
  }

  // Tile size check
  Int minWidth  = 1;
  Int minHeight = 1;
  const Int profileIdc = m_sps.getPTL()->getGeneralPTL()->getProfileIdc();
  if (  profileIdc == Profile::MAIN || profileIdc == Profile::MAIN10) //TODO: add more profiles to the tile-size check...
  {
    if (m_pps.getTilesEnabledFlag())//如果使用tile模式
    {
      minHeight = 64  / m_sps.getMaxCUHeight();//tile允许的高最小为64像素
      minWidth  = 256 / m_sps.getMaxCUWidth();//tile允许的高最小为256像素
    }
  }
  for(Int row=0; row < numRows; row++)//检查每个tile是否满足大小要求
  {
    for(Int col=0; col < numCols; col++)
    {
      const Int tileIdx = row * numCols + col;
      assert (m_tileParameters[tileIdx].getTileWidthInCtus() >= minWidth);
      assert (m_tileParameters[tileIdx].getTileHeightInCtus() >= minHeight);
    }
  }

  //initialize each tile of the current picture
  for( Int row=0; row < numRows; row++ )//遍历图像中的每行tile
  {
    for( Int col=0; col < numCols; col++ )//遍历图像中每行的各个tile
    {
      const Int tileIdx = row * numCols + col;//tile的位置索引

      //initialize the RightEdgePosInCU for each tile
      Int rightEdgePosInCTU = 0;
      for( Int i=0; i <= col; i++ )
      {
        rightEdgePosInCTU += m_tileParameters[row * numCols + i].getTileWidthInCtus();//该tile的右边界（距离图像左边界）的位置（以CTU为单位）
      }
      m_tileParameters[tileIdx].setRightEdgePosInCtus(rightEdgePosInCTU-1);//减1是因为起始CTU的位置是从0开始

      //initialize the BottomEdgePosInCU for each tile
      Int bottomEdgePosInCTU = 0;//同上 计算该tile的下边界（距离图像上边界）的位置（以CTU为单位）
      for( Int i=0; i <= row; i++ )
      {
        bottomEdgePosInCTU += m_tileParameters[i * numCols + col].getTileHeightInCtus();
      }
      m_tileParameters[tileIdx].setBottomEdgePosInCtus(bottomEdgePosInCTU-1);

      //initialize the FirstCUAddr for each tile//设置该tile中起始CTu在图像中的CtuRs地址
      m_tileParameters[tileIdx].setFirstCtuRsAddr( (m_tileParameters[tileIdx].getBottomEdgePosInCtus() - m_tileParameters[tileIdx].getTileHeightInCtus() + 1) * getFrameWidthInCtus() +
                                                    m_tileParameters[tileIdx].getRightEdgePosInCtus()  - m_tileParameters[tileIdx].getTileWidthInCtus()  + 1);
    }
  }

  Int  columnIdx = 0;
  Int  rowIdx = 0;

  //initialize the TileIdxMap//初始化m_puiTileIdxMap
  for( Int i=0; i<m_numCtusInFrame; i++)//为图像中的所有CTU指明其所在的tile
  {
    for( Int col=0; col < numCols; col++)
    {
      if(i % getFrameWidthInCtus() <= m_tileParameters[col].getRightEdgePosInCtus())
      {
        columnIdx = col;//该CTu所在tile在图像中的列（以tile为单位）
        break;
      }
    }
    for(Int row=0; row < numRows; row++)
    {
      if(i / getFrameWidthInCtus() <= m_tileParameters[row*numCols].getBottomEdgePosInCtus())
      {
        rowIdx = row;//该CTu所在tile在图像中的行（以tile为单位）
        break;
      }
    }
    m_puiTileIdxMap[i] = rowIdx * numCols + columnIdx;//指明该CTu所在的tile
  }
}

UInt TComPicSym::xCalculateNextCtuRSAddr( UInt currCtuRsAddr )//以Tile-Scan扫描图像 确定下一个CTu的在图像中的RS位置
{
  UInt  nextCtuRsAddr;

  //get the tile index for the current CTU
  const UInt uiTileIdx = getTileIdxMap(currCtuRsAddr);//得到当前CTu的tile索引

  //get the raster scan address for the next CTU
  if( currCtuRsAddr % m_frameWidthInCtus == getTComTile(uiTileIdx)->getRightEdgePosInCtus() && currCtuRsAddr / m_frameWidthInCtus == getTComTile(uiTileIdx)->getBottomEdgePosInCtus() )
  //the current CTU is the last CTU of the tile//如果当前CTu是tile中的最后一个CTu (与tile的右边界和下边界相邻）
  {
    if(uiTileIdx+1 == getNumTiles())//当前Ctu所在tile为图像的最后一个tile
    {
      nextCtuRsAddr = m_numCtusInFrame;//则下一个CTu超出图像范围（在处理中将超出图像范围的CTU当成图像的最后一个CTu）
    }
    else//若下一个CTu为超出图像范围
    {
      nextCtuRsAddr = getTComTile(uiTileIdx+1)->getFirstCtuRsAddr();//则下一个CTu的地址为下个tile的起始ctu
    }
  }
  else //the current CTU is not the last CTU of the tile//不为tile中的最后一个CTu
  {
    if( currCtuRsAddr % m_frameWidthInCtus == getTComTile(uiTileIdx)->getRightEdgePosInCtus() )  //the current CTU is on the rightmost edge of the tile
    {//当前Ctu与tile右边界相邻
      nextCtuRsAddr = currCtuRsAddr + m_frameWidthInCtus - getTComTile(uiTileIdx)->getTileWidthInCtus() + 1;//下一个Ctu为当前ctu所在tile下一行（以Ctu为单位）的左边界
    }
    else//若在tile内
    {
      nextCtuRsAddr = currCtuRsAddr + 1;// 则 TS顺序与Rs顺序一致
    }
  }

  return nextCtuRsAddr;
}

Void TComPicSym::deriveLoopFilterBoundaryAvailibility(Int ctuRsAddr,
                                                      Bool& isLeftAvail,
                                                      Bool& isRightAvail,
                                                      Bool& isAboveAvail,
                                                      Bool& isBelowAvail,
                                                      Bool& isAboveLeftAvail,
                                                      Bool& isAboveRightAvail,
                                                      Bool& isBelowLeftAvail,
                                                      Bool& isBelowRightAvail
                                                      )
{//ctuRsAddr为CTu在图像中RS的地址

  isLeftAvail      = (ctuRsAddr % m_frameWidthInCtus != 0);//该CTu左侧是否存在Ctu（是否在图像左边界）
  isRightAvail     = (ctuRsAddr % m_frameWidthInCtus != m_frameWidthInCtus-1);//该CTu右侧是否存在Ctu
  isAboveAvail     = (ctuRsAddr >= m_frameWidthInCtus );//该CTu上方是否存在Ctu
  isBelowAvail     = (ctuRsAddr <  m_numCtusInFrame - m_frameWidthInCtus);//该CTu下方是否存在Ctu
  isAboveLeftAvail = (isAboveAvail && isLeftAvail);//该CTu左上方是否存在Ctu
  isAboveRightAvail= (isAboveAvail && isRightAvail);//该CTu右上方是否存在Ctu
  isBelowLeftAvail = (isBelowAvail && isLeftAvail);//该CTu左下方是否存在Ctu
  isBelowRightAvail= (isBelowAvail && isRightAvail);//该CTu右下方是否存在Ctu

  Bool isLoopFiltAcrossTilePPS = getCtu(ctuRsAddr)->getSlice()->getPPS()->getLoopFilterAcrossTilesEnabledFlag();//环路滤波时是否允许跨越tile边界

  {
    TComDataCU* ctuCurr  = getCtu(ctuRsAddr);//得到该CTU对应方位相邻的Ctu
    TComDataCU* ctuLeft  = isLeftAvail ?getCtu(ctuRsAddr-1):NULL;
    TComDataCU* ctuRight = isRightAvail?getCtu(ctuRsAddr+1):NULL;
    TComDataCU* ctuAbove = isAboveAvail?getCtu(ctuRsAddr-m_frameWidthInCtus):NULL;
    TComDataCU* ctuBelow = isBelowAvail?getCtu(ctuRsAddr+m_frameWidthInCtus):NULL;
    TComDataCU* ctuAboveLeft  = isAboveLeftAvail ? getCtu(ctuRsAddr-m_frameWidthInCtus-1):NULL;
    TComDataCU* ctuAboveRight = isAboveRightAvail? getCtu(ctuRsAddr-m_frameWidthInCtus+1):NULL;
    TComDataCU* ctuBelowLeft  = isBelowLeftAvail ? getCtu(ctuRsAddr+m_frameWidthInCtus-1):NULL;
    TComDataCU* ctuBelowRight = isBelowRightAvail? getCtu(ctuRsAddr+m_frameWidthInCtus+1):NULL;

    {//如果该Ctu和不同方位相邻的Ctu在同一个slice中 则相邻Ctu可以获得 否则需要判断两个相邻的Ctu中靠右下（后解码）的Ctu是否允许跨Slice边界滤波 （因为不允许跨slice边界滤波时 无法获得其他slice的数据信息）
      //left
      if(ctuLeft != NULL)
      {
        isLeftAvail = (ctuCurr->getSlice()->getSliceCurStartCtuTsAddr() != ctuLeft->getSlice()->getSliceCurStartCtuTsAddr())?ctuCurr->getSlice()->getLFCrossSliceBoundaryFlag():true;
      }
      //above
      if(ctuAbove != NULL)
      {
        isAboveAvail = (ctuCurr->getSlice()->getSliceCurStartCtuTsAddr() != ctuAbove->getSlice()->getSliceCurStartCtuTsAddr())?ctuCurr->getSlice()->getLFCrossSliceBoundaryFlag():true;
      }
      //right
      if(ctuRight != NULL)
      {
        isRightAvail = (ctuCurr->getSlice()->getSliceCurStartCtuTsAddr() != ctuRight->getSlice()->getSliceCurStartCtuTsAddr())?ctuRight->getSlice()->getLFCrossSliceBoundaryFlag():true;
      }
      //below
      if(ctuBelow != NULL)
      {
        isBelowAvail = (ctuCurr->getSlice()->getSliceCurStartCtuTsAddr() != ctuBelow->getSlice()->getSliceCurStartCtuTsAddr())?ctuBelow->getSlice()->getLFCrossSliceBoundaryFlag():true;
      }
      //above-left
      if(ctuAboveLeft != NULL)
      {
        isAboveLeftAvail = (ctuCurr->getSlice()->getSliceCurStartCtuTsAddr() != ctuAboveLeft->getSlice()->getSliceCurStartCtuTsAddr())?ctuCurr->getSlice()->getLFCrossSliceBoundaryFlag():true;
      }
      //below-right
      if(ctuBelowRight != NULL)
      {
        isBelowRightAvail = (ctuCurr->getSlice()->getSliceCurStartCtuTsAddr() != ctuBelowRight->getSlice()->getSliceCurStartCtuTsAddr())?ctuBelowRight->getSlice()->getLFCrossSliceBoundaryFlag():true;
      }

      //above-right
      if(ctuAboveRight != NULL)//右上方位时 两Ctu所在slice不同 则无法直接判断两slice的解码先后顺序 需根据slice首CTu地址判断解码顺序
      {
        Int curSliceStartTsAddr  = ctuCurr->getSlice()->getSliceCurStartCtuTsAddr();
        Int aboveRightSliceStartTsAddr = ctuAboveRight->getSlice()->getSliceCurStartCtuTsAddr();

        isAboveRightAvail = (curSliceStartTsAddr == aboveRightSliceStartTsAddr)?(true):
          (
          (curSliceStartTsAddr > aboveRightSliceStartTsAddr)?(ctuCurr->getSlice()->getLFCrossSliceBoundaryFlag())
          :(ctuAboveRight->getSlice()->getLFCrossSliceBoundaryFlag())
          );
      }
      //below-left
      if(ctuBelowLeft != NULL)//左下方位同上
      {
        Int curSliceStartTsAddr       = ctuCurr->getSlice()->getSliceCurStartCtuTsAddr();
        Int belowLeftSliceStartTsAddr = ctuBelowLeft->getSlice()->getSliceCurStartCtuTsAddr();

        isBelowLeftAvail = (curSliceStartTsAddr == belowLeftSliceStartTsAddr)?(true):
          (
          (curSliceStartTsAddr > belowLeftSliceStartTsAddr)?(ctuCurr->getSlice()->getLFCrossSliceBoundaryFlag())
          :(ctuBelowLeft->getSlice()->getLFCrossSliceBoundaryFlag())
          );
      }
    }

    if(!isLoopFiltAcrossTilePPS)//若不能跨tile边界滤波 则需要保证当前Ctu不同方位相邻的Ctu在同一个tile中（因为不允许跨tile边界滤波时 无法获得其他tile的数据信息）
    {
      isLeftAvail      = (!isLeftAvail      ) ?false:(getTileIdxMap( ctuLeft->getCtuRsAddr()         ) == getTileIdxMap( ctuRsAddr ));
      isAboveAvail     = (!isAboveAvail     ) ?false:(getTileIdxMap( ctuAbove->getCtuRsAddr()        ) == getTileIdxMap( ctuRsAddr ));
      isRightAvail     = (!isRightAvail     ) ?false:(getTileIdxMap( ctuRight->getCtuRsAddr()        ) == getTileIdxMap( ctuRsAddr ));
      isBelowAvail     = (!isBelowAvail     ) ?false:(getTileIdxMap( ctuBelow->getCtuRsAddr()        ) == getTileIdxMap( ctuRsAddr ));
      isAboveLeftAvail = (!isAboveLeftAvail ) ?false:(getTileIdxMap( ctuAboveLeft->getCtuRsAddr()    ) == getTileIdxMap( ctuRsAddr ));
      isAboveRightAvail= (!isAboveRightAvail) ?false:(getTileIdxMap( ctuAboveRight->getCtuRsAddr()   ) == getTileIdxMap( ctuRsAddr ));
      isBelowLeftAvail = (!isBelowLeftAvail ) ?false:(getTileIdxMap( ctuBelowLeft->getCtuRsAddr()    ) == getTileIdxMap( ctuRsAddr ));
      isBelowRightAvail= (!isBelowRightAvail) ?false:(getTileIdxMap( ctuBelowRight->getCtuRsAddr()   ) == getTileIdxMap( ctuRsAddr ));
    }
  }

}


TComTile::TComTile()
: m_tileWidthInCtus     (0)
, m_tileHeightInCtus    (0)
, m_rightEdgePosInCtus  (0)
, m_bottomEdgePosInCtus (0)
, m_firstCtuRsAddr      (0)
{
}

TComTile::~TComTile()
{
}
//! \}
