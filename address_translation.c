//////////////////////////////////////////////////////////////////////////////////
// address_translation.c for Cosmos+ OpenSSD
// Copyright (c) 2017 Hanyang University ENC Lab.
// Contributed by Yong Ho Song <yhsong@enc.hanyang.ac.kr>
//				  Jaewook Kwak <jwkwak@enc.hanyang.ac.kr>
//				  Sangjin Lee <sjlee@enc.hanyang.ac.kr>
//
// This file is part of Cosmos+ OpenSSD.
//
// Cosmos+ OpenSSD is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3, or (at your option)
// any later version.
//
// Cosmos+ OpenSSD is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Cosmos+ OpenSSD; see the file COPYING.
// If not, see <http://www.gnu.org/licenses/>.
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: ENC Lab. <http://enc.hanyang.ac.kr>
// Engineer: Jaewook Kwak <jwkwak@enc.hanyang.ac.kr>
//
// Project Name: Cosmos+ OpenSSD
// Design Name: Cosmos+ Firmware
// Module Name: Address Translator
// File Name: address translation.c
//
// Version: v1.0.0
//
// Description:
//   - translate address between address space of host system and address space of NAND device
//   - manage bad blocks in NAND device
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Revision History:
//
// * v1.0.0
//   - First draft
//////////////////////////////////////////////////////////////////////////////////

#include <assert.h>
#include "memory_map.h"
#include "xil_printf.h"

P_LOGICAL_SLICE_MAP logicalSliceMapPtr;
P_VIRTUAL_SLICE_MAP virtualSliceMapPtr;
P_VIRTUAL_BLOCK_MAP virtualBlockMapPtr;
P_VIRTUAL_DIE_MAP virtualDieMapPtr;
P_PHY_BLOCK_MAP phyBlockMapPtr;
P_BAD_BLOCK_TABLE_INFO_MAP bbtInfoMapPtr;

unsigned char sliceAllocationTargetDie;
unsigned int mbPerbadBlockSpace;
unsigned int vBlock_i;

static unsigned int logicalBlockBaseVsa[LOGICAL_BLOCKS_PER_SSD];
static unsigned short logicalBlockNextOffset[LOGICAL_BLOCKS_PER_SSD];

#define BLOCK_CUR_PAGE_LOCK_MASK 0x8000// 1000 0000 0000 0000
#define BLOCK_CUR_PAGE_VALUE_MASK 0x7FFF// 0111 1111 1111 1111 
//currentPage에서 상위 1bit = block-level 잠금 flag
//currentPage에서 하위 15bit = 블록 별 실제 프로그램된 페이지 수
// user_pages_per_block = 16384 (0x4000) 이하이므로 15비트로 충분

/*
--------------------------------------------------------------------------
| 1 bit (lock)  |         15 bits (current programmed page count)        |
--------------------------------------------------------------------------
*/

// 블록 별 실제 프로그램된 페이지 수(하위 15비트)를 조회
static inline unsigned int GetBlockCurrentPage(unsigned int dieNo, unsigned int blockNo)
{
	return virtualBlockMapPtr->block[dieNo][blockNo].currentPage & BLOCK_CUR_PAGE_VALUE_MASK;
}

// 상위 1비트 잠금 플래그를 이용해 블록이 block-level 매핑 전용인지 체크
static inline unsigned int IsBlockReservedForBlkMapping(unsigned int dieNo, unsigned int blockNo)
{
	return virtualBlockMapPtr->block[dieNo][blockNo].currentPage & BLOCK_CUR_PAGE_LOCK_MASK;
}

// 실제 페이지 수만 갱신하고 잠금 비트는 유지
static inline void SetBlockCurrentPageCount(unsigned int dieNo, unsigned int blockNo, unsigned int pageCnt)
{
	unsigned int lock = virtualBlockMapPtr->block[dieNo][blockNo].currentPage & BLOCK_CUR_PAGE_LOCK_MASK;
	virtualBlockMapPtr->block[dieNo][blockNo].currentPage = lock | (pageCnt & BLOCK_CUR_PAGE_VALUE_MASK);
}

// 페이지 수와 잠금 플래그를 모두 초기화
static inline void ResetBlockCurrentPage(unsigned int dieNo, unsigned int blockNo)
{
	virtualBlockMapPtr->block[dieNo][blockNo].currentPage = 0;
}

// block-level 전용으로 사용할 블록 잠금
static inline void LockBlockForBlkMapping(unsigned int dieNo, unsigned int blockNo)
{
	virtualBlockMapPtr->block[dieNo][blockNo].currentPage |= BLOCK_CUR_PAGE_LOCK_MASK;
}

// block-level 쓰기 완료 후 잠금 해제
static inline void UnlockBlockFromBlkMapping(unsigned int dieNo, unsigned int blockNo)
{
	virtualBlockMapPtr->block[dieNo][blockNo].currentPage &= BLOCK_CUR_PAGE_VALUE_MASK;
}

void InitAddressMap()
{
	unsigned int blockNo, dieNo;

	logicalSliceMapPtr = (P_LOGICAL_SLICE_MAP ) LOGICAL_SLICE_MAP_ADDR;
	virtualSliceMapPtr = (P_VIRTUAL_SLICE_MAP) VIRTUAL_SLICE_MAP_ADDR;
	virtualBlockMapPtr = (P_VIRTUAL_BLOCK_MAP) VIRTUAL_BLOCK_MAP_ADDR;
	virtualDieMapPtr = (P_VIRTUAL_DIE_MAP) VIRTUAL_DIE_MAP_ADDR;
	phyBlockMapPtr = (P_PHY_BLOCK_MAP) PHY_BLOCK_MAP_ADDR;
	bbtInfoMapPtr = (P_BAD_BLOCK_TABLE_INFO_MAP) BAD_BLOCK_TABLE_INFO_MAP_ADDR;

	//init phyblockMap
	for(dieNo=0 ; dieNo<USER_DIES ; dieNo++)
	{
		for(blockNo=0 ; blockNo<TOTAL_BLOCKS_PER_DIE ; blockNo++)
			phyBlockMapPtr->phyBlock[dieNo][blockNo].remappedPhyBlock = blockNo;

		bbtInfoMapPtr->bbtInfo[dieNo].phyBlock = 0;
		bbtInfoMapPtr->bbtInfo[dieNo].grownBadUpdate = BBT_INFO_GROWN_BAD_UPDATE_NONE;
	}

	sliceAllocationTargetDie = FindDieForFreeSliceAllocation();

	InitSliceMap();
	InitBlockDieMap();
}

void InitSliceMap()
{
	int sliceAddr;
	for(sliceAddr=0; sliceAddr<SLICES_PER_SSD ; sliceAddr++)
	{
		logicalSliceMapPtr->logicalSlice[sliceAddr].virtualSliceAddr = VSA_NONE;
		virtualSliceMapPtr->virtualSlice[sliceAddr].logicalSliceAddr = LSA_NONE;
	}

	for(sliceAddr=0; sliceAddr<LOGICAL_BLOCKS_PER_SSD; sliceAddr++)
	{
		logicalBlockBaseVsa[sliceAddr] = VSA_NONE;
		logicalBlockNextOffset[sliceAddr] = 0;
	}
}

void RemapBadBlock()
{
	unsigned int blockNo, dieNo, remapFlag, maxBadBlockCount;
	unsigned int reservedBlockOfLun0[USER_DIES];
	unsigned int reservedBlockOfLun1[USER_DIES];
	unsigned int badBlockCount[USER_DIES];

	xil_printf("Bad block remapping start...\r\n");

	for(dieNo=0 ; dieNo<USER_DIES ; dieNo++)
	{
		reservedBlockOfLun0[dieNo] = USER_BLOCKS_PER_LUN;
		reservedBlockOfLun1[dieNo] = TOTAL_BLOCKS_PER_LUN + USER_BLOCKS_PER_LUN;
		badBlockCount[dieNo] = 0;
	}


	for(blockNo=0 ; blockNo<USER_BLOCKS_PER_LUN ; blockNo++)
	{
		for(dieNo=0 ; dieNo<USER_DIES ; dieNo++)
		{
			//lun0
			if(phyBlockMapPtr->phyBlock[dieNo][blockNo].bad)
			{
				if(reservedBlockOfLun0[dieNo] < TOTAL_BLOCKS_PER_LUN)
				{
					remapFlag = 1;
					while(phyBlockMapPtr->phyBlock[dieNo][reservedBlockOfLun0[dieNo]].bad)
					{
						reservedBlockOfLun0[dieNo]++;
						if(reservedBlockOfLun0[dieNo] >= TOTAL_BLOCKS_PER_LUN)
						{
							remapFlag = 0;
							break;
						}
					}

					if(remapFlag)
					{
						phyBlockMapPtr->phyBlock[dieNo][blockNo].remappedPhyBlock  = reservedBlockOfLun0[dieNo];
						reservedBlockOfLun0[dieNo]++;
					}
					else
					{
						xil_printf("No reserved block - Ch %d Way %d virtualBlock %d is bad block \r\n", Vdie2PchTranslation(dieNo), Vdie2PwayTranslation(dieNo), blockNo);
						badBlockCount[dieNo]++;
					}
				}
				else
				{
					xil_printf("No reserved block - Ch %d Way %d virtualBlock %d is bad block \r\n", Vdie2PchTranslation(dieNo), Vdie2PwayTranslation(dieNo), blockNo);
					badBlockCount[dieNo]++;
				}
			}

			if (LUNS_PER_DIE > 1)
			{
				//lun1
				if(phyBlockMapPtr->phyBlock[dieNo][blockNo+TOTAL_BLOCKS_PER_LUN].bad)
				{
					if(reservedBlockOfLun1[dieNo] < TOTAL_BLOCKS_PER_DIE)
					{
						remapFlag = 1;
						while(phyBlockMapPtr->phyBlock[dieNo][reservedBlockOfLun1[dieNo]].bad)
						{
							reservedBlockOfLun1[dieNo]++;
							if(reservedBlockOfLun1[dieNo] >= TOTAL_BLOCKS_PER_DIE)
							{
								remapFlag = 0;
								break;
							}
						}

						if(remapFlag)
						{
							phyBlockMapPtr->phyBlock[dieNo][blockNo+TOTAL_BLOCKS_PER_LUN].remappedPhyBlock  = reservedBlockOfLun1[dieNo];
							reservedBlockOfLun1[dieNo]++;
						}
						else
						{
							xil_printf("No reserved block - Ch %x Way %x virtualBlock %d is bad block \r\n",  Vdie2PchTranslation(dieNo), Vdie2PwayTranslation(dieNo), blockNo+USER_BLOCKS_PER_LUN);
							badBlockCount[dieNo]++;
						}
					}
					else
					{
						xil_printf("No reserved block - Ch %x Way %x virtualBlock %d is bad block \r\n",  Vdie2PchTranslation(dieNo), Vdie2PwayTranslation(dieNo), blockNo+USER_BLOCKS_PER_LUN);
						badBlockCount[dieNo]++;
					}
				}
			}
		}
	}

	xil_printf("Bad block remapping end\r\n");


	maxBadBlockCount = 0;
	for(dieNo=0; dieNo < USER_DIES; dieNo++)
	{
		if(maxBadBlockCount < badBlockCount[dieNo])
			maxBadBlockCount = badBlockCount[dieNo];
	}

	mbPerbadBlockSpace = maxBadBlockCount * USER_DIES * MB_PER_BLOCK;
}

void InitDieMap()
{
	unsigned int dieNo;

	for(dieNo=0 ; dieNo<USER_DIES ; dieNo++)
	{
		virtualDieMapPtr->die[dieNo].headFreeBlock = BLOCK_NONE;
		virtualDieMapPtr->die[dieNo].tailFreeBlock = BLOCK_NONE;
		virtualDieMapPtr->die[dieNo].freeBlockCnt = 0;
	}
}

void InitBlockMap()
{
	unsigned int dieNo, phyBlockNo, virtualBlockNo, remappedPhyBlock;

	for(dieNo=0 ; dieNo<USER_DIES ; dieNo++)
	{
		for(virtualBlockNo=0; virtualBlockNo<USER_BLOCKS_PER_DIE ; virtualBlockNo++)
		{
			phyBlockNo = Vblock2PblockOfTbsTranslation(virtualBlockNo);
			remappedPhyBlock = phyBlockMapPtr->phyBlock[dieNo][phyBlockNo].remappedPhyBlock;
			virtualBlockMapPtr->block[dieNo][virtualBlockNo].bad = phyBlockMapPtr->phyBlock[dieNo][remappedPhyBlock].bad;

			virtualBlockMapPtr->block[dieNo][virtualBlockNo].free = 1;
			virtualBlockMapPtr->block[dieNo][virtualBlockNo].invalidSliceCnt = 0;
			ResetBlockCurrentPage(dieNo, virtualBlockNo);
			virtualBlockMapPtr->block[dieNo][virtualBlockNo].eraseCnt = 0;

			if(virtualBlockMapPtr->block[dieNo][virtualBlockNo].bad)
			{
				virtualBlockMapPtr->block[dieNo][virtualBlockNo].prevBlock = BLOCK_NONE;
				virtualBlockMapPtr->block[dieNo][virtualBlockNo].nextBlock = BLOCK_NONE;
			}
			else
				PutToFbList(dieNo, virtualBlockNo);
		}
	}
}

void InitCurrentBlockOfDieMap()
{
	unsigned int dieNo;

	for(dieNo=0 ; dieNo<USER_DIES ; dieNo++)
	{
		virtualDieMapPtr->die[dieNo].currentBlock = GetFromFbList(dieNo, GET_FREE_BLOCK_NORMAL);
		if(virtualDieMapPtr->die[dieNo].currentBlock == BLOCK_FAIL)
			assert(!"[WARNING] There is no free block [WARNING]");
	}
}

void ReadBadBlockTable(unsigned int tempBbtBufAddr[], unsigned int tempBbtBufEntrySize)
{
	unsigned int tempPage, reqSlotTag, dieNo;
	int loop, dataSize;

	loop = 0;
	dataSize = DATA_SIZE_OF_BAD_BLOCK_TABLE_PER_DIE;
	tempPage = PlsbPage2VpageTranslation(START_PAGE_NO_OF_BAD_BLOCK_TABLE_BLOCK); 	//bad block table is saved at lsb pages

	while(dataSize>0)
	{
		for(dieNo = 0; dieNo < USER_DIES; dieNo++)
		{
			reqSlotTag = GetFromFreeReqQ();

			reqPoolPtr->reqPool[reqSlotTag].reqType = REQ_TYPE_NAND;
			reqPoolPtr->reqPool[reqSlotTag].reqCode = REQ_CODE_READ;
			reqPoolPtr->reqPool[reqSlotTag].reqOpt.dataBufFormat = REQ_OPT_DATA_BUF_ADDR;
			reqPoolPtr->reqPool[reqSlotTag].reqOpt.nandAddr = REQ_OPT_NAND_ADDR_PHY_ORG;
			reqPoolPtr->reqPool[reqSlotTag].reqOpt.nandEcc = REQ_OPT_NAND_ECC_ON;
			reqPoolPtr->reqPool[reqSlotTag].reqOpt.nandEccWarning = REQ_OPT_NAND_ECC_WARNING_OFF;
			reqPoolPtr->reqPool[reqSlotTag].reqOpt.rowAddrDependencyCheck = REQ_OPT_ROW_ADDR_DEPENDENCY_NONE;
			reqPoolPtr->reqPool[reqSlotTag].reqOpt.blockSpace = REQ_OPT_BLOCK_SPACE_TOTAL;

			reqPoolPtr->reqPool[reqSlotTag].dataBufInfo.addr = tempBbtBufAddr[dieNo] + loop * tempBbtBufEntrySize;

			reqPoolPtr->reqPool[reqSlotTag].nandInfo.physicalCh = Vdie2PchTranslation(dieNo);
			reqPoolPtr->reqPool[reqSlotTag].nandInfo.physicalWay = Vdie2PwayTranslation(dieNo);
			reqPoolPtr->reqPool[reqSlotTag].nandInfo.physicalBlock = bbtInfoMapPtr->bbtInfo[dieNo].phyBlock;
			reqPoolPtr->reqPool[reqSlotTag].nandInfo.physicalPage = Vpage2PlsbPageTranslation(tempPage);

			SelectLowLevelReqQ(reqSlotTag);
		}

		tempPage++;
		loop++;
		dataSize -= BYTES_PER_DATA_REGION_OF_PAGE;
	}

	SyncAllLowLevelReqDone();
}

void FindBadBlock(unsigned char dieState[], unsigned int tempBbtBufAddr[], unsigned int tempBbtBufEntrySize, unsigned int tempReadBufAddr[], unsigned int tempReadBufEntrySize)
{
	unsigned int phyBlockNo, dieNo, reqSlotTag;
	unsigned char blockChecker[USER_DIES];
	unsigned char* markPointer0;
	unsigned char* markPointer1;
	unsigned char* bbtUpdater;

	//check bad block mark of each block
	for(phyBlockNo = 0; phyBlockNo < TOTAL_BLOCKS_PER_DIE; phyBlockNo++)
	{
		for(dieNo=0; dieNo < USER_DIES; dieNo++)
			if(!dieState[dieNo])
			{
				blockChecker[dieNo] = BLOCK_STATE_NORMAL;

				reqSlotTag = GetFromFreeReqQ();

				reqPoolPtr->reqPool[reqSlotTag].reqType = REQ_TYPE_NAND;
				reqPoolPtr->reqPool[reqSlotTag].reqCode = REQ_CODE_READ;
				reqPoolPtr->reqPool[reqSlotTag].reqOpt.dataBufFormat = REQ_OPT_DATA_BUF_ADDR;
				reqPoolPtr->reqPool[reqSlotTag].reqOpt.nandAddr = REQ_OPT_NAND_ADDR_PHY_ORG;
				reqPoolPtr->reqPool[reqSlotTag].reqOpt.nandEcc = REQ_OPT_NAND_ECC_OFF;
				reqPoolPtr->reqPool[reqSlotTag].reqOpt.nandEccWarning = REQ_OPT_NAND_ECC_WARNING_OFF;
				reqPoolPtr->reqPool[reqSlotTag].reqOpt.rowAddrDependencyCheck = REQ_OPT_ROW_ADDR_DEPENDENCY_NONE;
				reqPoolPtr->reqPool[reqSlotTag].reqOpt.blockSpace = REQ_OPT_BLOCK_SPACE_TOTAL;

				reqPoolPtr->reqPool[reqSlotTag].dataBufInfo.addr = tempReadBufAddr[dieNo];

				reqPoolPtr->reqPool[reqSlotTag].nandInfo.physicalCh = Vdie2PchTranslation(dieNo);
				reqPoolPtr->reqPool[reqSlotTag].nandInfo.physicalWay = Vdie2PwayTranslation(dieNo);
				reqPoolPtr->reqPool[reqSlotTag].nandInfo.physicalBlock = phyBlockNo;
				reqPoolPtr->reqPool[reqSlotTag].nandInfo.physicalPage = BAD_BLOCK_MARK_PAGE0;

				SelectLowLevelReqQ(reqSlotTag);
			}

		SyncAllLowLevelReqDone();

		for(dieNo=0; dieNo < USER_DIES; dieNo++)
			if(!dieState[dieNo])
			{
				markPointer0 = (unsigned char*)(tempReadBufAddr[dieNo] + BAD_BLOCK_MARK_BYTE0);
				markPointer1 = (unsigned char*)(tempReadBufAddr[dieNo] + BAD_BLOCK_MARK_BYTE1);

				if((*markPointer0 == CLEAN_DATA_IN_BYTE) && (*markPointer1 == CLEAN_DATA_IN_BYTE))
				{
					reqSlotTag = GetFromFreeReqQ();

					reqPoolPtr->reqPool[reqSlotTag].reqType = REQ_TYPE_NAND;
					reqPoolPtr->reqPool[reqSlotTag].reqCode = REQ_CODE_READ;
					reqPoolPtr->reqPool[reqSlotTag].reqOpt.dataBufFormat = REQ_OPT_DATA_BUF_ADDR;
					reqPoolPtr->reqPool[reqSlotTag].reqOpt.nandAddr = REQ_OPT_NAND_ADDR_PHY_ORG;
					reqPoolPtr->reqPool[reqSlotTag].reqOpt.nandEcc = REQ_OPT_NAND_ECC_OFF;
					reqPoolPtr->reqPool[reqSlotTag].reqOpt.nandEccWarning = REQ_OPT_NAND_ECC_WARNING_OFF;
					reqPoolPtr->reqPool[reqSlotTag].reqOpt.rowAddrDependencyCheck = REQ_OPT_ROW_ADDR_DEPENDENCY_NONE;
					reqPoolPtr->reqPool[reqSlotTag].reqOpt.blockSpace = REQ_OPT_BLOCK_SPACE_TOTAL;

					reqPoolPtr->reqPool[reqSlotTag].dataBufInfo.addr = tempReadBufAddr[dieNo];

					reqPoolPtr->reqPool[reqSlotTag].nandInfo.physicalCh = Vdie2PchTranslation(dieNo);
					reqPoolPtr->reqPool[reqSlotTag].nandInfo.physicalWay = Vdie2PwayTranslation(dieNo);
					reqPoolPtr->reqPool[reqSlotTag].nandInfo.physicalBlock = phyBlockNo;
					reqPoolPtr->reqPool[reqSlotTag].nandInfo.physicalPage = BAD_BLOCK_MARK_PAGE1;

					SelectLowLevelReqQ(reqSlotTag);
				}
				else
				{
					xil_printf("	bad block is detected: Ch %d Way %d phyBlock %d \r\n",Vdie2PchTranslation(dieNo), Vdie2PwayTranslation(dieNo), phyBlockNo);

					blockChecker[dieNo] = BLOCK_STATE_BAD;
				}
			}

		SyncAllLowLevelReqDone();

		for(dieNo=0; dieNo < USER_DIES; dieNo++)
			if(!dieState[dieNo])
			{
				markPointer0 = (unsigned char*)(tempReadBufAddr[dieNo] + BAD_BLOCK_MARK_BYTE0);
				markPointer1 = (unsigned char*)(tempReadBufAddr[dieNo] + BAD_BLOCK_MARK_BYTE1);

				if(!((*markPointer0 == CLEAN_DATA_IN_BYTE) && (*markPointer1 == CLEAN_DATA_IN_BYTE)))
					if(blockChecker[dieNo] == BLOCK_STATE_NORMAL)
					{
						xil_printf("	bad block is detected: Ch %d Way %d phyBlock %d \r\n",Vdie2PchTranslation(dieNo), Vdie2PwayTranslation(dieNo), phyBlockNo);

						blockChecker[dieNo] = BLOCK_STATE_BAD;

					}

				bbtUpdater= (unsigned char*)(tempBbtBufAddr[dieNo] + phyBlockNo);
				*bbtUpdater = blockChecker[dieNo];
				phyBlockMapPtr->phyBlock[dieNo][phyBlockNo].bad = blockChecker[dieNo];
			}
	}
}


void SaveBadBlockTable(unsigned char dieState[], unsigned int tempBbtBufAddr[], unsigned int tempBbtBufEntrySize)
{
	unsigned int dieNo, reqSlotTag;
	int loop, dataSize, tempPage;

	loop = 0;
	dataSize = DATA_SIZE_OF_BAD_BLOCK_TABLE_PER_DIE;
	tempPage = PlsbPage2VpageTranslation(START_PAGE_NO_OF_BAD_BLOCK_TABLE_BLOCK);	//bad block table is saved at lsb pages

	while(dataSize>0)
	{
		for(dieNo = 0; dieNo < USER_DIES; dieNo++)
			if((dieState[dieNo] == DIE_STATE_BAD_BLOCK_TABLE_NOT_EXIST) || (dieState[dieNo] == DIE_STATE_BAD_BLOCK_TABLE_UPDATE))
			{
				if(loop == 0)
				{
					reqSlotTag = GetFromFreeReqQ();

					reqPoolPtr->reqPool[reqSlotTag].reqType = REQ_TYPE_NAND;
					reqPoolPtr->reqPool[reqSlotTag].reqCode = REQ_CODE_ERASE;
					reqPoolPtr->reqPool[reqSlotTag].reqOpt.nandAddr = REQ_OPT_NAND_ADDR_PHY_ORG;
					reqPoolPtr->reqPool[reqSlotTag].reqOpt.dataBufFormat = REQ_OPT_DATA_BUF_NONE;
					reqPoolPtr->reqPool[reqSlotTag].reqOpt.rowAddrDependencyCheck = REQ_OPT_ROW_ADDR_DEPENDENCY_NONE;
					reqPoolPtr->reqPool[reqSlotTag].reqOpt.blockSpace = REQ_OPT_BLOCK_SPACE_TOTAL;

					reqPoolPtr->reqPool[reqSlotTag].nandInfo.physicalCh = Vdie2PchTranslation(dieNo);
					reqPoolPtr->reqPool[reqSlotTag].nandInfo.physicalWay = Vdie2PwayTranslation(dieNo);
					reqPoolPtr->reqPool[reqSlotTag].nandInfo.physicalBlock = bbtInfoMapPtr->bbtInfo[dieNo].phyBlock;
					reqPoolPtr->reqPool[reqSlotTag].nandInfo.physicalPage = 0;	//dummy

					SelectLowLevelReqQ(reqSlotTag);
				}

				reqSlotTag = GetFromFreeReqQ();

				reqPoolPtr->reqPool[reqSlotTag].reqType = REQ_TYPE_NAND;
				reqPoolPtr->reqPool[reqSlotTag].reqCode = REQ_CODE_WRITE;
				reqPoolPtr->reqPool[reqSlotTag].reqOpt.dataBufFormat = REQ_OPT_DATA_BUF_ADDR;
				reqPoolPtr->reqPool[reqSlotTag].reqOpt.nandAddr = REQ_OPT_NAND_ADDR_PHY_ORG;
				reqPoolPtr->reqPool[reqSlotTag].reqOpt.nandEcc = REQ_OPT_NAND_ECC_ON;
				reqPoolPtr->reqPool[reqSlotTag].reqOpt.nandEccWarning = REQ_OPT_NAND_ECC_WARNING_OFF;
				reqPoolPtr->reqPool[reqSlotTag].reqOpt.rowAddrDependencyCheck = REQ_OPT_ROW_ADDR_DEPENDENCY_NONE;
				reqPoolPtr->reqPool[reqSlotTag].reqOpt.blockSpace = REQ_OPT_BLOCK_SPACE_TOTAL;

				reqPoolPtr->reqPool[reqSlotTag].dataBufInfo.addr = tempBbtBufAddr[dieNo] + loop * tempBbtBufEntrySize;

				reqPoolPtr->reqPool[reqSlotTag].nandInfo.physicalCh = Vdie2PchTranslation(dieNo);
				reqPoolPtr->reqPool[reqSlotTag].nandInfo.physicalWay = Vdie2PwayTranslation(dieNo);
				reqPoolPtr->reqPool[reqSlotTag].nandInfo.physicalBlock = bbtInfoMapPtr->bbtInfo[dieNo].phyBlock;
				reqPoolPtr->reqPool[reqSlotTag].nandInfo.physicalPage =  Vpage2PlsbPageTranslation(tempPage);

				SelectLowLevelReqQ(reqSlotTag);
			}

		loop++;
		dataSize++;
		dataSize -= BYTES_PER_DATA_REGION_OF_PAGE;
	}

	SyncAllLowLevelReqDone();


	for(dieNo=0; dieNo < USER_DIES; dieNo++)
		if(dieState[dieNo] == DIE_STATE_BAD_BLOCK_TABLE_NOT_EXIST)
			xil_printf("[ bad block table of Ch %d Way %d is saved. ]\r\n", dieNo%USER_CHANNELS, dieNo/USER_CHANNELS);
}


void RecoverBadBlockTable(unsigned int tempBufAddr)
{
	unsigned int dieNo, phyBlockNo, bbtMaker, tempBbtBufBaseAddr, tempBbtBufEntrySize, tempReadBufBaseAddr, tempReadBufEntrySize;
	unsigned int tempBbtBufAddr[USER_DIES];
	unsigned int tempReadBufAddr[USER_DIES];
	unsigned char dieState[USER_DIES];
	unsigned char* bbtTableChecker;

	//data buffer allocation
	tempBbtBufBaseAddr = tempBufAddr;
	tempBbtBufEntrySize = BYTES_PER_DATA_REGION_OF_PAGE + BYTES_PER_SPARE_REGION_OF_PAGE;
	tempReadBufBaseAddr = tempBbtBufBaseAddr + USER_DIES * USED_PAGES_FOR_BAD_BLOCK_TABLE_PER_DIE * tempBbtBufEntrySize;
	tempReadBufEntrySize = BYTES_PER_NAND_ROW;
	for(dieNo = 0; dieNo < USER_DIES; dieNo++)
	{
		tempBbtBufAddr[dieNo] = tempBbtBufBaseAddr + dieNo * USED_PAGES_FOR_BAD_BLOCK_TABLE_PER_DIE * tempBbtBufEntrySize;
		tempReadBufAddr[dieNo] = tempReadBufBaseAddr + dieNo * tempReadBufEntrySize;
	}

	//read bad block tables
	ReadBadBlockTable(tempBbtBufAddr, tempBbtBufEntrySize);

	//check bad block tables
	bbtMaker = BAD_BLOCK_TABLE_MAKER_IDLE;
	for(dieNo=0; dieNo<USER_DIES; dieNo++)
	{
		bbtTableChecker = (unsigned char*)(tempBbtBufAddr[dieNo]);

		if((*bbtTableChecker == BLOCK_STATE_NORMAL)||(*bbtTableChecker == BLOCK_STATE_BAD))
		{
			xil_printf("[ bad block table of ch %d way %d exists.]\r\n",Vdie2PchTranslation(dieNo), Vdie2PwayTranslation(dieNo));

			dieState[dieNo] = DIE_STATE_BAD_BLOCK_TABLE_EXIST;
			for(phyBlockNo=0; phyBlockNo<TOTAL_BLOCKS_PER_DIE; phyBlockNo++)
			{
				bbtTableChecker = (unsigned char*)(tempBbtBufAddr[dieNo] + phyBlockNo);

				phyBlockMapPtr->phyBlock[dieNo][phyBlockNo].bad = *bbtTableChecker;
				if(phyBlockMapPtr->phyBlock[dieNo][phyBlockNo].bad == BLOCK_STATE_BAD)
					xil_printf("	bad block: ch %d way %d phyBlock %d  \r\n", Vdie2PchTranslation(dieNo), Vdie2PwayTranslation(dieNo), phyBlockNo);
			}

			xil_printf("[ bad blocks of ch %d way %d are checked. ]\r\n",Vdie2PchTranslation(dieNo), Vdie2PwayTranslation(dieNo));
		}
		else
		{
			xil_printf("[ bad block table of ch %d way %d does not exist.]\r\n",Vdie2PchTranslation(dieNo), Vdie2PwayTranslation(dieNo));
			dieState[dieNo] = DIE_STATE_BAD_BLOCK_TABLE_NOT_EXIST;
			bbtMaker = BAD_BLOCK_TABLE_MAKER_TRIGGER;
		}
	}

	//if bad block table does not exist in some dies, make new bad block table for each die having no bad block table
	if(bbtMaker == BAD_BLOCK_TABLE_MAKER_TRIGGER)
	{
		FindBadBlock(dieState, tempBbtBufAddr, tempBbtBufEntrySize, tempReadBufAddr, tempReadBufEntrySize);
		SaveBadBlockTable(dieState, tempBbtBufAddr, tempBbtBufEntrySize);
	}

	//grown bad update flag initialization
	for(dieNo=0; dieNo<USER_DIES; dieNo++)
		bbtInfoMapPtr->bbtInfo[dieNo].grownBadUpdate = BBT_INFO_GROWN_BAD_UPDATE_NONE;
}



void EraseTotalBlockSpace()
{
	unsigned int blockNo, dieNo, reqSlotTag;

	xil_printf("Erase total block space...wait for a minute...\r\n");

	for(blockNo=0 ; blockNo<TOTAL_BLOCKS_PER_DIE ; blockNo++)
		for(dieNo=0 ; dieNo<USER_DIES ; dieNo++)
		{
			reqSlotTag = GetFromFreeReqQ();

			reqPoolPtr->reqPool[reqSlotTag].reqType = REQ_TYPE_NAND;
			reqPoolPtr->reqPool[reqSlotTag].reqCode = REQ_CODE_ERASE;
			reqPoolPtr->reqPool[reqSlotTag].reqOpt.nandAddr = REQ_OPT_NAND_ADDR_PHY_ORG;
			reqPoolPtr->reqPool[reqSlotTag].reqOpt.dataBufFormat = REQ_OPT_DATA_BUF_NONE;
			reqPoolPtr->reqPool[reqSlotTag].reqOpt.rowAddrDependencyCheck = REQ_OPT_ROW_ADDR_DEPENDENCY_NONE;
			reqPoolPtr->reqPool[reqSlotTag].reqOpt.blockSpace = REQ_OPT_BLOCK_SPACE_TOTAL;

			reqPoolPtr->reqPool[reqSlotTag].nandInfo.physicalCh = Vdie2PchTranslation(dieNo);
			reqPoolPtr->reqPool[reqSlotTag].nandInfo.physicalWay = Vdie2PwayTranslation(dieNo);
			reqPoolPtr->reqPool[reqSlotTag].nandInfo.physicalBlock = blockNo;
			reqPoolPtr->reqPool[reqSlotTag].nandInfo.physicalPage = 0;

			SelectLowLevelReqQ(reqSlotTag);
		}


	SyncAllLowLevelReqDone();
	xil_printf("Done.\r\n");
}


void EraseUserBlockSpace()
{
	unsigned int blockNo, dieNo, reqSlotTag;

	xil_printf("Erase User block space...wait for a minute...\r\n");

	for(blockNo=0 ; blockNo<USER_BLOCKS_PER_DIE ; blockNo++)
		for(dieNo=0 ; dieNo<USER_DIES ; dieNo++)
			if(!virtualBlockMapPtr->block[dieNo][blockNo].bad)
			{
				reqSlotTag = GetFromFreeReqQ();

				reqPoolPtr->reqPool[reqSlotTag].reqType = REQ_TYPE_NAND;
				reqPoolPtr->reqPool[reqSlotTag].reqCode = REQ_CODE_ERASE;
				reqPoolPtr->reqPool[reqSlotTag].reqOpt.nandAddr = REQ_OPT_NAND_ADDR_VSA;
				reqPoolPtr->reqPool[reqSlotTag].reqOpt.dataBufFormat = REQ_OPT_DATA_BUF_NONE;
				reqPoolPtr->reqPool[reqSlotTag].reqOpt.rowAddrDependencyCheck = REQ_OPT_ROW_ADDR_DEPENDENCY_NONE;
				reqPoolPtr->reqPool[reqSlotTag].reqOpt.blockSpace = REQ_OPT_BLOCK_SPACE_MAIN;

				reqPoolPtr->reqPool[reqSlotTag].nandInfo.virtualSliceAddr = Vorg2VsaTranslation(dieNo, blockNo, 0);

				SelectLowLevelReqQ(reqSlotTag);
			}

	SyncAllLowLevelReqDone();
	xil_printf("Done.\r\n");
}


void InitBlockDieMap()
{
	unsigned int dieNo;
	unsigned char eraseFlag = 1;

	xil_printf("Press 'X' to re-make the bad block table.\r\n");
	if (inbyte() == 'X')
	{
		EraseTotalBlockSpace();
		eraseFlag = 0;
	}

	InitDieMap();

	//make bad block table
	RecoverBadBlockTable(RESERVED_DATA_BUFFER_BASE_ADDR);

	//to prevent accessing bbtBlock by host
	for(dieNo=0 ; dieNo<USER_DIES ; dieNo++)
		phyBlockMapPtr->phyBlock[dieNo][bbtInfoMapPtr->bbtInfo[dieNo].phyBlock].bad = 1;

	RemapBadBlock();

	InitBlockMap();

	if(eraseFlag)
		EraseUserBlockSpace();

	InitCurrentBlockOfDieMap();
}

unsigned int AddrTransRead(unsigned int logicalSliceAddr)
{
	unsigned int vSlice;

	if(logicalSliceAddr < SLICES_PER_SSD)
	{
		vSlice = logicalSliceMapPtr->logicalSlice[logicalSliceAddr].virtualSliceAddr;
		if(vSlice != VSA_NONE){
			xil_printf("VSA read : LSA %d -> VSA %d \r\n", logicalSliceAddr, vSlice);
			return vSlice;
		}else{
			xil_printf("VSA read fail : LSA %d has no mapped VSA\r\n", logicalSliceAddr);
			return VSA_FAIL;
		}
	}
	else
		assert(!"[WARNING] Logical address is larger than maximum logical address served by SSD [WARNING]");
}

// 논리 슬라이스 주소를 블록 단위로 새 물리 슬라이스에 매핑
unsigned int AddrTransWrite(unsigned int logicalSliceAddr)
{
	unsigned int block, vSlice, vsaDie, vsaBlock, programmedPages;

	// 1) 호스트가 SSD 최대 LSA를 벗어난 경우 - 치명적 오류
	if(logicalSliceAddr >= SLICES_PER_SSD)
		assert(!"[WARNING] Logical address is larger than maximum logical address served by SSD [WARNING]");

	block = AddrToBlock(logicalSliceAddr);

	vSlice = logicalSliceMapPtr->logicalSlice[logicalSliceAddr].virtualSliceAddr;
	if(vSlice != VSA_NONE)
	{
		// 2) 이미 매핑된 LSA에 다시 쓰기가 들어온 경우 - 기존 VSA를 GC 대상으로만 표시
		assert(virtualSliceMapPtr->virtualSlice[vSlice].logicalSliceAddr == logicalSliceAddr);
		xil_printf("VSA rewrite : LSA %d was mapped to VSA %d, remapping\r\n", logicalSliceAddr, vSlice);
		InvalidateOldVsa(logicalSliceAddr);
	}

	if(logicalBlockBaseVsa[block] == VSA_NONE)
	{
		// 3) 해당 논리 블록에 아직 배정된 물리 블록이 없는 경우 - block-level 전용 블록 예약
		logicalBlockBaseVsa[block] = FindFreeVirtualBlock();
		logicalBlockNextOffset[block] = 0;
		xil_printf("New block allocated for logical block %d : base VSA %d\r\n", block, logicalBlockBaseVsa[block]);
	}

	// 4) 논리 블록 안에서 사용 가능한 슬롯(페이지)보다 많이 쓰려고 할 때 - 논리 오류
	if(logicalBlockNextOffset[block] >= SLICES_PER_BLOCK)
		assert(!"[WARNING] Logical block already fully populated [WARNING]");

	// base VSA에서 die/block을 추출해 동일 블록 내 page offset에 해당하는 VSA 계산
	unsigned int baseVsa = logicalBlockBaseVsa[block];
	unsigned int baseDie = Vsa2VdieTranslation(baseVsa);
	unsigned int baseBlock = Vsa2VblockTranslation(baseVsa);
	unsigned int pageOffset = logicalBlockNextOffset[block];
	vSlice = Vorg2VsaTranslation(baseDie, baseBlock, pageOffset);
	logicalBlockNextOffset[block]++;

	logicalSliceMapPtr->logicalSlice[logicalSliceAddr].virtualSliceAddr = vSlice;
	virtualSliceMapPtr->virtualSlice[vSlice].logicalSliceAddr = logicalSliceAddr;

	vsaDie = Vsa2VdieTranslation(vSlice);
	vsaBlock = Vsa2VblockTranslation(vSlice);
	// 슬라이스 단위 오프셋을 페이지 단위로 반올림해 현재까지 프로그램된 페이지 수 추적
	programmedPages = (logicalBlockNextOffset[block] + (SLICES_PER_PAGE - 1)) / SLICES_PER_PAGE;
	if(GetBlockCurrentPage(vsaDie, vsaBlock) < programmedPages)
		SetBlockCurrentPageCount(vsaDie, vsaBlock, programmedPages);

	xil_printf("VSA write new : LSA %d -> VSA %d (logical block %d, slot %d)\r\n",
			logicalSliceAddr, vSlice, block, logicalBlockNextOffset[block] - 1);

	// 5) 단위 블록의 모든 슬라이스를 채웠을 때 - lock 해제 후 다음 write에 새 블록을 할당하도록 초기화
	if(logicalBlockNextOffset[block] == SLICES_PER_BLOCK)
	{
		xil_printf("[BlkAlloc] logical block %d fully populated (base VSA %d)\r\n",
				block, logicalBlockBaseVsa[block]);
		// 전체 슬라이스를 모두 채우면 다른 경로가 재사용할 수 있도록 잠금 해제
		UnlockBlockFromBlkMapping(vsaDie, vsaBlock);
		logicalBlockBaseVsa[block] = VSA_NONE;
		logicalBlockNextOffset[block] = 0;
	}

	return vSlice;
}


unsigned int FindFreeVirtualBlock(){
	unsigned int dieNo, currentBlock, baseVsa;
	dieNo = sliceAllocationTargetDie;
	currentBlock = virtualDieMapPtr->die[dieNo].currentBlock;
	assert(currentBlock != BLOCK_FAIL);
	assert(GetBlockCurrentPage(dieNo, currentBlock) <= USER_PAGES_PER_BLOCK);
	// 이미 일부 페이지를 사용했거나 block-level 전용으로 잠긴 블록은 건너뜀
	while((GetBlockCurrentPage(dieNo, currentBlock) != 0) || IsBlockReservedForBlkMapping(dieNo, currentBlock))
	{
		currentBlock = GetFromFbList(dieNo, GET_FREE_BLOCK_NORMAL);
		if(currentBlock != BLOCK_FAIL)
		{
			virtualDieMapPtr->die[dieNo].currentBlock = currentBlock;
			ResetBlockCurrentPage(dieNo, currentBlock);
		}
		else
		{
			xil_printf("[BlkAlloc] free block short on die %d, triggering GC\r\n", dieNo);
			GarbageCollection(dieNo);
			currentBlock = virtualDieMapPtr->die[dieNo].currentBlock;
			assert(currentBlock != BLOCK_FAIL);
		}
		assert(GetBlockCurrentPage(dieNo, currentBlock) <= USER_PAGES_PER_BLOCK);
	}
	baseVsa = Vorg2VsaTranslation(dieNo, currentBlock, 0);
	ResetBlockCurrentPage(dieNo, currentBlock);
	// 잠금 플래그를 켜 block-level 쓰기 구간임을 표시
	LockBlockForBlkMapping(dieNo, currentBlock);
	sliceAllocationTargetDie = FindDieForFreeSliceAllocation();
	xil_printf("[BlkAlloc] die %d block %d reserved for block-level write (VSA %d)\r\n",
			dieNo, currentBlock, baseVsa);
	return baseVsa;
}

unsigned int FindFreeVirtualSlice()
{
	unsigned int currentBlock, virtualSliceAddr, dieNo;

	dieNo = sliceAllocationTargetDie;
	currentBlock = virtualDieMapPtr->die[dieNo].currentBlock;

	if((GetBlockCurrentPage(dieNo, currentBlock) == USER_PAGES_PER_BLOCK) || IsBlockReservedForBlkMapping(dieNo, currentBlock))
	{
		currentBlock = GetFromFbList(dieNo, GET_FREE_BLOCK_NORMAL);

		if(currentBlock != BLOCK_FAIL)
			virtualDieMapPtr->die[dieNo].currentBlock = currentBlock;
		else
		{
			GarbageCollection(dieNo);
			currentBlock = virtualDieMapPtr->die[dieNo].currentBlock;

			if((GetBlockCurrentPage(dieNo, currentBlock) == USER_PAGES_PER_BLOCK) || IsBlockReservedForBlkMapping(dieNo, currentBlock))
			{
				currentBlock = GetFromFbList(dieNo, GET_FREE_BLOCK_NORMAL);
				if(currentBlock != BLOCK_FAIL)
					virtualDieMapPtr->die[dieNo].currentBlock = currentBlock;
				else
					assert(!"[WARNING] There is no available block [WARNING]");
			}
			else if(GetBlockCurrentPage(dieNo, currentBlock) > USER_PAGES_PER_BLOCK)
				assert(!"[WARNING] Current page management fail [WARNING]");
		}
	}
	else if(GetBlockCurrentPage(dieNo, currentBlock) > USER_PAGES_PER_BLOCK)
		assert(!"[WARNING] Current page management fail [WARNING]");


	virtualSliceAddr = Vorg2VsaTranslation(dieNo, currentBlock, GetBlockCurrentPage(dieNo, currentBlock));
	SetBlockCurrentPageCount(dieNo, currentBlock, GetBlockCurrentPage(dieNo, currentBlock) + 1);
	sliceAllocationTargetDie = FindDieForFreeSliceAllocation();
	dieNo = sliceAllocationTargetDie;
	return virtualSliceAddr;
}


unsigned int FindFreeVirtualSliceForGc(unsigned int copyTargetDieNo, unsigned int victimBlockNo)
{
	unsigned int currentBlock, virtualSliceAddr, dieNo;

	dieNo = copyTargetDieNo;
	if(victimBlockNo == virtualDieMapPtr->die[dieNo].currentBlock)
	{
		virtualDieMapPtr->die[dieNo].currentBlock = GetFromFbList(dieNo, GET_FREE_BLOCK_GC);
		if(virtualDieMapPtr->die[dieNo].currentBlock == BLOCK_FAIL)
			assert(!"[WARNING] There is no available block [WARNING]");
	}
	currentBlock = virtualDieMapPtr->die[dieNo].currentBlock;

	if((GetBlockCurrentPage(dieNo, currentBlock) == USER_PAGES_PER_BLOCK) || IsBlockReservedForBlkMapping(dieNo, currentBlock))
	{

		currentBlock = GetFromFbList(dieNo, GET_FREE_BLOCK_GC);

		if(currentBlock != BLOCK_FAIL)
			virtualDieMapPtr->die[dieNo].currentBlock = currentBlock;
		else
			assert(!"[WARNING] There is no available block [WARNING]");
	}
	else if(GetBlockCurrentPage(dieNo, currentBlock) > USER_PAGES_PER_BLOCK)
		assert(!"[WARNING] Current page management fail [WARNING]");


	virtualSliceAddr = Vorg2VsaTranslation(dieNo, currentBlock, GetBlockCurrentPage(dieNo, currentBlock));
	SetBlockCurrentPageCount(dieNo, currentBlock, GetBlockCurrentPage(dieNo, currentBlock) + 1);
	return virtualSliceAddr;
}


unsigned int FindDieForFreeSliceAllocation()
{
	static unsigned char targetCh = 0;
	static unsigned char targetWay = 0;
	unsigned int targetDie;

	targetDie = Pcw2VdieTranslation(targetCh, targetWay);

	if(targetCh != (USER_CHANNELS - 1))
		targetCh = targetCh + 1;
	else
	{
		targetCh = 0;
		targetWay = (targetWay + 1) % USER_WAYS;
	}

	return targetDie;
}

void InvalidateOldVsaBlock(unsigned int logicalBlockAddr)
{
	unsigned int offset, logicalSliceAddr;

	for(offset=0; offset<SLICES_PER_BLOCK; offset++)
	{
		logicalSliceAddr = logicalBlockAddr * SLICES_PER_BLOCK + offset;
		InvalidateOldVsa(logicalSliceAddr);
	}

	if(logicalBlockAddr < LOGICAL_BLOCKS_PER_SSD)
	{
		logicalBlockBaseVsa[logicalBlockAddr] = VSA_NONE;
		logicalBlockNextOffset[logicalBlockAddr] = 0;
	}
}

void InvalidateOldVsaForBlockLevel(unsigned int logicalSliceAddr)
{
	unsigned int virtualSliceAddr, dieNo, blockNo, offset, logicalBlockAddr;
	logicalBlockAddr = AddrToBlock(logicalSliceAddr);
	offset = AddrToOffset(logicalSliceAddr);
	
	virtualSliceAddr = logicalSliceMapPtr->logicalSlice[logicalSliceAddr].virtualSliceAddr;

	if(virtualSliceAddr != VSA_NONE)
	{
		if(virtualSliceMapPtr->virtualSlice[virtualSliceAddr].logicalSliceAddr != logicalSliceAddr)
			return;

		dieNo = Vsa2VdieTranslation(virtualSliceAddr);
		blockNo = Vsa2VblockTranslation(virtualSliceAddr);

		// unlink
		SelectiveGetFromGcVictimList(dieNo, blockNo);
		virtualBlockMapPtr->block[dieNo][blockNo].invalidSliceCnt++;
		logicalSliceMapPtr->logicalSlice[logicalSliceAddr].virtualSliceAddr = VSA_NONE;

		PutToGcVictimList(dieNo, blockNo, virtualBlockMapPtr->block[dieNo][blockNo].invalidSliceCnt);
	}

}

void InvalidateOldVsa(unsigned int logicalSliceAddr)
{
	unsigned int virtualSliceAddr, dieNo, blockNo;

	virtualSliceAddr = logicalSliceMapPtr->logicalSlice[logicalSliceAddr].virtualSliceAddr;

	if(virtualSliceAddr != VSA_NONE)
	{
		if(virtualSliceMapPtr->virtualSlice[virtualSliceAddr].logicalSliceAddr != logicalSliceAddr)
			return;

		dieNo = Vsa2VdieTranslation(virtualSliceAddr);
		blockNo = Vsa2VblockTranslation(virtualSliceAddr);

		// unlink
		SelectiveGetFromGcVictimList(dieNo, blockNo);
		virtualBlockMapPtr->block[dieNo][blockNo].invalidSliceCnt++;
		logicalSliceMapPtr->logicalSlice[logicalSliceAddr].virtualSliceAddr = VSA_NONE;

		PutToGcVictimList(dieNo, blockNo, virtualBlockMapPtr->block[dieNo][blockNo].invalidSliceCnt);
	}

}


void EraseBlock(unsigned int dieNo, unsigned int blockNo)
{
	unsigned int pageNo, virtualSliceAddr, reqSlotTag;

	reqSlotTag = GetFromFreeReqQ();

	reqPoolPtr->reqPool[reqSlotTag].reqType = REQ_TYPE_NAND;
	reqPoolPtr->reqPool[reqSlotTag].reqCode = REQ_CODE_ERASE;
	reqPoolPtr->reqPool[reqSlotTag].reqOpt.nandAddr = REQ_OPT_NAND_ADDR_VSA;
	reqPoolPtr->reqPool[reqSlotTag].reqOpt.dataBufFormat = REQ_OPT_DATA_BUF_NONE;
	reqPoolPtr->reqPool[reqSlotTag].reqOpt.rowAddrDependencyCheck = REQ_OPT_ROW_ADDR_DEPENDENCY_CHECK;
	reqPoolPtr->reqPool[reqSlotTag].reqOpt.blockSpace = REQ_OPT_BLOCK_SPACE_MAIN;
	reqPoolPtr->reqPool[reqSlotTag].nandInfo.virtualSliceAddr = Vorg2VsaTranslation(dieNo, blockNo, 0);
	reqPoolPtr->reqPool[reqSlotTag].nandInfo.programmedPageCnt = GetBlockCurrentPage(dieNo, blockNo);

	SelectLowLevelReqQ(reqSlotTag);

	// block map indicated blockNo initialization
	virtualBlockMapPtr->block[dieNo][blockNo].free = 1;
	virtualBlockMapPtr->block[dieNo][blockNo].eraseCnt++;
	virtualBlockMapPtr->block[dieNo][blockNo].invalidSliceCnt = 0;
	ResetBlockCurrentPage(dieNo, blockNo);

	PutToFbList(dieNo, blockNo);

	for(pageNo=0; pageNo<USER_PAGES_PER_BLOCK; pageNo++)
	{
		virtualSliceAddr = Vorg2VsaTranslation(dieNo, blockNo, pageNo);
		virtualSliceMapPtr->virtualSlice[virtualSliceAddr].logicalSliceAddr = LSA_NONE;
	}
}

void PutToFbList(unsigned int dieNo, unsigned int blockNo) //fb means free block
{
	if(virtualDieMapPtr->die[dieNo].tailFreeBlock != BLOCK_NONE)
	{
		virtualBlockMapPtr->block[dieNo][blockNo].prevBlock = virtualDieMapPtr->die[dieNo].tailFreeBlock;
		virtualBlockMapPtr->block[dieNo][blockNo].nextBlock = BLOCK_NONE;
		virtualBlockMapPtr->block[dieNo][virtualDieMapPtr->die[dieNo].tailFreeBlock].nextBlock = blockNo;
		virtualDieMapPtr->die[dieNo].tailFreeBlock = blockNo;
	}
	else
	{
		virtualBlockMapPtr->block[dieNo][blockNo].prevBlock = BLOCK_NONE;
		virtualBlockMapPtr->block[dieNo][blockNo].nextBlock = BLOCK_NONE;
		virtualDieMapPtr->die[dieNo].headFreeBlock = blockNo;
		virtualDieMapPtr->die[dieNo].tailFreeBlock = blockNo;
	}

	virtualDieMapPtr->die[dieNo].freeBlockCnt++;
}

unsigned int GetFromFbList(unsigned int dieNo, unsigned int getFreeBlockOption) //fb means free block
{
	unsigned int evictedBlockNo;

	evictedBlockNo = virtualDieMapPtr->die[dieNo].headFreeBlock;

	if(getFreeBlockOption == GET_FREE_BLOCK_NORMAL)
	{
		if(virtualDieMapPtr->die[dieNo].freeBlockCnt <= RESERVED_FREE_BLOCK_COUNT)
			return BLOCK_FAIL;
	}
	else if(getFreeBlockOption == GET_FREE_BLOCK_GC)
	{
		if(evictedBlockNo == BLOCK_NONE)
			return BLOCK_FAIL;
	}
	else
		assert(!"[WARNING] Wrong getFreeBlockOption [WARNING]");

	if(virtualBlockMapPtr->block[dieNo][evictedBlockNo].nextBlock != BLOCK_NONE)
	{
		virtualDieMapPtr->die[dieNo].headFreeBlock = virtualBlockMapPtr->block[dieNo][evictedBlockNo].nextBlock;
		virtualBlockMapPtr->block[dieNo][virtualBlockMapPtr->block[dieNo][evictedBlockNo].nextBlock].prevBlock = BLOCK_NONE;
	}
	else
	{
		virtualDieMapPtr->die[dieNo].headFreeBlock = BLOCK_NONE;
		virtualDieMapPtr->die[dieNo].tailFreeBlock = BLOCK_NONE;
	}

	virtualBlockMapPtr->block[dieNo][evictedBlockNo].free = 0;
	virtualDieMapPtr->die[dieNo].freeBlockCnt--;

	virtualBlockMapPtr->block[dieNo][evictedBlockNo].nextBlock = BLOCK_NONE;
	virtualBlockMapPtr->block[dieNo][evictedBlockNo].prevBlock = BLOCK_NONE;

	return evictedBlockNo;
}


void UpdatePhyBlockMapForGrownBadBlock(unsigned int dieNo, unsigned int phyBlockNo)
{
	phyBlockMapPtr->phyBlock[dieNo][phyBlockNo].bad = BLOCK_STATE_BAD;

	bbtInfoMapPtr->bbtInfo[dieNo].grownBadUpdate = BBT_INFO_GROWN_BAD_UPDATE_BOOKED;
}


void UpdateBadBlockTableForGrownBadBlock(unsigned int tempBufAddr)
{
	unsigned int dieNo, phyBlockNo, tempBbtBufBaseAddr, tempBbtBufEntrySize;
	unsigned int tempBbtBufAddr[USER_DIES];
	unsigned char dieState[USER_DIES];
	unsigned char* bbtUpdater;

	//data buffer allocation
	tempBbtBufBaseAddr = tempBufAddr;
	tempBbtBufEntrySize = BYTES_PER_DATA_REGION_OF_PAGE + BYTES_PER_SPARE_REGION_OF_PAGE;
	for(dieNo = 0; dieNo < USER_DIES; dieNo++)
		tempBbtBufAddr[dieNo] = tempBbtBufBaseAddr + dieNo * USED_PAGES_FOR_BAD_BLOCK_TABLE_PER_DIE * tempBbtBufEntrySize;

	//create new bad block table
	for(dieNo = 0; dieNo < USER_DIES; dieNo++)
	{
		if(bbtInfoMapPtr->bbtInfo[dieNo].grownBadUpdate == BBT_INFO_GROWN_BAD_UPDATE_BOOKED)
		{
			for(phyBlockNo = 0; phyBlockNo < TOTAL_BLOCKS_PER_DIE; phyBlockNo++)
			{
				bbtUpdater = (unsigned char*)(tempBbtBufAddr[dieNo] + phyBlockNo);

				if(phyBlockNo != bbtInfoMapPtr->bbtInfo[dieNo].phyBlock)
					*bbtUpdater = phyBlockMapPtr->phyBlock[dieNo][phyBlockNo].bad;
				else
					*bbtUpdater = BLOCK_STATE_NORMAL;
			}

			dieState[dieNo] = DIE_STATE_BAD_BLOCK_TABLE_UPDATE;
		}
		else
			dieState[dieNo] = DIE_STATE_BAD_BLOCK_TABLE_HOLD;
	}

	//update bad block tables in flash
	SaveBadBlockTable(dieState, tempBbtBufAddr, tempBbtBufEntrySize);
}
